#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "app_settings.h"
#include "console_commands.h"
#include "esp_log.h"
#include "esp_mac.h"   // MACSTR, MAC2STR
#include "esp_netif.h" // esp_netif_init, esp_netif_t
#include "esp_system.h"
#include "fram_log.h"
#include "fram_spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "log_record.h"
#include "max31865_reader.h"
#include "mesh_transport.h"
#include "nvs_flash.h"
#include "sd_logger.h"
#include "time_sync.h"
#include <stdio.h> // sscanf

static const char* kTag = "app";

typedef struct
{
  app_settings_t settings;
  fram_spi_t fram_spi;
  fram_log_t fram_log;
  sd_logger_t sd_logger;
  sd_logger_config_t sd_config;
  max31865_reader_t sensor;
  mesh_transport_t mesh;
  time_sync_t time_sync;

  QueueHandle_t log_queue;
  uint8_t* batch_buffer;
  size_t batch_buffer_size;

  TickType_t last_flush_ticks;
  bool sd_error;
  bool fram_full;
  bool fram_full_logged;

  char node_id_string[32];
} app_state_t;

static void
FormatMacString(const uint8_t mac[6], char* out, size_t out_size)
{
  snprintf(out,
           out_size,
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],
           mac[1],
           mac[2],
           mac[3],
           mac[4],
           mac[5]);
}

static void
BuildIso8601Utc(int64_t epoch_seconds,
                int32_t millis,
                char* out,
                size_t out_size)
{
  if (epoch_seconds <= 0) {
    if (out_size > 0) {
      out[0] = '\0';
    }
    return;
  }

  time_t time_seconds = (time_t)epoch_seconds;
  struct tm time_info;
  gmtime_r(&time_seconds, &time_info);

  if (millis < 0) {
    millis = 0;
  }
  if (millis > 999) {
    millis = 999;
  }

  snprintf(out,
           out_size,
           "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
           time_info.tm_year + 1900,
           time_info.tm_mon + 1,
           time_info.tm_mday,
           time_info.tm_hour,
           time_info.tm_min,
           time_info.tm_sec,
           (int)millis);
}

static void
BuildDateStringFromRecord(const log_record_t* record,
                          char* out,
                          size_t out_size)
{
  int64_t epoch = record->timestamp_epoch_sec;
  if (epoch <= 0) {
    epoch = (int64_t)time(NULL);
  }
  time_t time_seconds = (time_t)epoch;
  struct tm time_info;
  gmtime_r(&time_seconds, &time_info);
  strftime(out, out_size, "%Y-%m-%d", &time_info);
}

static bool
FormatCsvLine(const log_record_t* record,
              const char* node_id,
              char* out,
              size_t out_size,
              size_t* written_out)
{
  char iso8601[40];
  BuildIso8601Utc(record->timestamp_epoch_sec,
                  record->timestamp_millis,
                  iso8601,
                  sizeof(iso8601));

  const double raw_c = record->raw_temp_milli_c / 1000.0;
  const double temp_c = record->temp_milli_c / 1000.0;
  const double resistance_ohm = record->resistance_milli_ohm / 1000.0;

  const int length = snprintf(out,
                              out_size,
                              "%u,%" PRId64 ",%s,%.3f,%.3f,%.3f,0x%04x,%s\n",
                              (unsigned)record->sequence,
                              record->timestamp_epoch_sec,
                              iso8601,
                              resistance_ohm,
                              raw_c,
                              temp_c,
                              (unsigned)record->flags,
                              node_id);
  if (length < 0 || (size_t)length >= out_size) {
    return false;
  }
  if (written_out != NULL) {
    *written_out = (size_t)length;
  }
  return true;
}

static void
PrintJsonRecord(const char* node_id, const log_record_t* record)
{
  const double raw_c = record->raw_temp_milli_c / 1000.0;
  const double temp_c = record->temp_milli_c / 1000.0;
  const double resistance_ohm = record->resistance_milli_ohm / 1000.0;
  printf("{\"type\":\"temp\",\"node\":\"%s\",\"ts\":%" PRId64
         ",\"temp_c\":%.3f,\"raw_c\":%.3f,\"r_ohm\":%.3f,\"seq\":%u,"
         "\"flags\":%u}\n",
         node_id,
         record->timestamp_epoch_sec,
         temp_c,
         raw_c,
         resistance_ohm,
         (unsigned)record->sequence,
         (unsigned)record->flags);
}

static void
RootRecordRxCallback(const mesh_addr_t* from,
                     const log_record_t* record,
                     void* context)
{
  (void)context;
  char node_id[32];
  FormatMacString(from->addr, node_id, sizeof(node_id));
  PrintJsonRecord(node_id, record);
}

static esp_err_t
EnsureSdSyncedForEpoch(app_state_t* state, int64_t epoch_for_file)
{
  esp_err_t sd_result =
    SdLoggerEnsureDailyFile(&state->sd_logger, epoch_for_file);
  if (sd_result != ESP_OK) {
    return sd_result;
  }

  uint32_t consumed = 0;
  esp_err_t consume_result = FramLogConsumeUpToSequence(
    &state->fram_log, SdLoggerLastSequenceOnSd(&state->sd_logger), &consumed);
  if (consume_result == ESP_ERR_INVALID_RESPONSE) {
    ESP_LOGE(kTag, "FRAM corruption while aligning with SD contents");
    return consume_result;
  }
  if (consume_result != ESP_OK) {
    return consume_result;
  }
  if (consumed > 0) {
    ESP_LOGW(kTag, "Dropped %u FRAM records already present on SD", consumed);
  }
  return ESP_OK;
}

static esp_err_t
BuildBatchForDay(app_state_t* state,
                 const char* target_date,
                 uint8_t* buffer,
                 size_t buffer_size,
                 uint32_t* records_used_out,
                 uint32_t* last_sequence_out,
                 size_t* bytes_used_out)
{
  size_t used = 0;
  uint32_t records_used = 0;
  uint32_t last_seq = 0;

  const uint32_t buffered = FramLogGetBufferedRecords(&state->fram_log);
  for (uint32_t offset = 0; offset < buffered; ++offset) {
    log_record_t record;
    esp_err_t peek_result =
      FramLogPeekOffset(&state->fram_log, offset, &record);
    if (peek_result == ESP_ERR_NOT_FOUND) {
      break;
    }
    if (peek_result == ESP_ERR_INVALID_RESPONSE) {
      ESP_LOGE(kTag, "Corrupted FRAM record detected during batch build");
      (void)FramLogSkipCorruptedRecord(&state->fram_log);
      break;
    }
    if (peek_result != ESP_OK) {
      return peek_result;
    }

    char record_date[16];
    BuildDateStringFromRecord(&record, record_date, sizeof(record_date));
    if (strcmp(record_date, target_date) != 0) {
      break; // stop at day rollover; flush current batch first
    }

    char line[196];
    size_t line_len = 0;
    if (!FormatCsvLine(
          &record, state->node_id_string, line, sizeof(line), &line_len)) {
      return ESP_ERR_NO_MEM;
    }
    if (used + line_len > buffer_size) {
      // Buffer full; flush what we have so far.
      break;
    }

    memcpy(buffer + used, line, line_len);
    used += line_len;
    records_used++;
    last_seq = record.sequence;

    if (used >= buffer_size - sizeof(line)) {
      break;
    }
  }

  *records_used_out = records_used;
  *last_sequence_out = last_seq;
  *bytes_used_out = used;
  return ESP_OK;
}

static esp_err_t
FlushFramToSd(app_state_t* state, bool flush_all)
{
  if (!state->sd_logger.is_mounted) {
    return ESP_ERR_INVALID_STATE;
  }
  if (state->batch_buffer == NULL || state->batch_buffer_size == 0) {
    return ESP_ERR_NO_MEM;
  }

  uint32_t total_flushed = 0;
  while (FramLogGetBufferedRecords(&state->fram_log) > 0) {
    log_record_t first_record;
    esp_err_t peek_result = FramLogPeekOldest(&state->fram_log, &first_record);
    if (peek_result == ESP_ERR_INVALID_RESPONSE) {
      ESP_LOGE(kTag, "Cannot flush: corrupted FRAM record at head");
      (void)FramLogSkipCorruptedRecord(&state->fram_log);
      return ESP_ERR_INVALID_RESPONSE;
    }
    if (peek_result != ESP_OK) {
      return peek_result;
    }

    char day_string[16];
    BuildDateStringFromRecord(&first_record, day_string, sizeof(day_string));

    const int64_t epoch_for_file = (first_record.timestamp_epoch_sec > 0)
                                     ? first_record.timestamp_epoch_sec
                                     : (int64_t)time(NULL);

    esp_err_t sync_result = EnsureSdSyncedForEpoch(state, epoch_for_file);
    if (sync_result != ESP_OK) {
      return sync_result;
    }

    uint32_t records_used = 0;
    uint32_t last_seq = 0;
    size_t bytes_used = 0;
    esp_err_t batch_result = BuildBatchForDay(state,
                                              day_string,
                                              state->batch_buffer,
                                              state->batch_buffer_size,
                                              &records_used,
                                              &last_seq,
                                              &bytes_used);
    if (batch_result != ESP_OK) {
      return batch_result;
    }
    if (records_used == 0 || bytes_used == 0) {
      return ESP_ERR_INVALID_STATE;
    }

    esp_err_t write_result = SdLoggerAppendVerifiedBatch(
      &state->sd_logger, state->batch_buffer, bytes_used, last_seq);
    if (write_result != ESP_OK) {
      ESP_LOGE(kTag,
               "SD append failed after %u records: %s",
               total_flushed + records_used,
               esp_err_to_name(write_result));
      return write_result;
    }

    for (uint32_t index = 0; index < records_used; ++index) {
      esp_err_t discard_result = FramLogDiscardOldest(&state->fram_log);
      if (discard_result != ESP_OK) {
        return discard_result;
      }
    }

    total_flushed += records_used;
    ESP_LOGI(kTag,
             "Flushed %u records (%zu bytes) for %s (total=%u)",
             records_used,
             bytes_used,
             day_string,
             total_flushed);

    if (!flush_all) {
      break;
    }
  }

  return (total_flushed > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static void
SensorTask(void* context)
{
  app_state_t* state = (app_state_t*)context;

  while (true) {
    const uint32_t period_ms = state->settings.log_period_ms;

    if (state->fram_full) {
      if (!state->fram_full_logged) {
        ESP_LOGW(kTag,
                 "FRAM full; pausing sensor logging until flush succeeds");
        state->fram_full_logged = true;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    float raw_c = 0;
    float resistance_ohm = 0;

    esp_err_t result =
      Max31865ReaderRead(&state->sensor, &raw_c, &resistance_ohm);

    log_record_t record;
    memset(&record, 0, sizeof(record));
    record.sequence = FramLogNextSequence(&state->fram_log);

    int64_t epoch_sec = 0;
    int32_t millis = 0;
    TimeSyncGetNow(&epoch_sec, &millis);
    record.timestamp_epoch_sec =
      TimeSyncIsSystemTimeValid() ? epoch_sec : (int64_t)0;
    record.timestamp_millis = millis;

    if (result == ESP_OK) {
      const double cal_c =
        CalibrationModelEvaluate(&state->settings.calibration, raw_c);
      record.raw_temp_milli_c = (int32_t)llround(raw_c * 1000.0);
      record.temp_milli_c = (int32_t)llround(cal_c * 1000.0);
      record.resistance_milli_ohm = (int32_t)llround(resistance_ohm * 1000.0);
    } else {
      ESP_LOGW(kTag, "sensor read failed: %s", esp_err_to_name(result));
      record.flags |= LOG_RECORD_FLAG_SENSOR_FAULT;
    }

    if (TimeSyncIsSystemTimeValid()) {
      record.flags |= LOG_RECORD_FLAG_TIME_VALID;
    }
    if (state->settings.calibration.is_valid) {
      record.flags |= LOG_RECORD_FLAG_CAL_VALID;
    }
    if (state->sd_error) {
      record.flags |= LOG_RECORD_FLAG_SD_ERROR;
    }
    if (MeshTransportIsConnected(&state->mesh)) {
      record.flags |= LOG_RECORD_FLAG_MESH_CONNECTED;
    }

    (void)xQueueSend(state->log_queue, &record, 0);
    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }
}

static void
StorageTask(void* context)
{
  app_state_t* state = (app_state_t*)context;
  state->last_flush_ticks = xTaskGetTickCount();

  while (true) {
    log_record_t record;
    if (xQueueReceive(state->log_queue, &record, pdMS_TO_TICKS(500)) ==
        pdTRUE) {
      esp_err_t append_result = FramLogAppend(&state->fram_log, &record);
      if (append_result == ESP_ERR_NO_MEM) {
        state->fram_full = true;
        state->fram_full_logged = false;
        ESP_LOGW(kTag, "FRAM is full; new samples will be dropped until flush");
      } else if (append_result != ESP_OK) {
        ESP_LOGE(
          kTag, "FRAM append failed: %s", esp_err_to_name(append_result));
      } else {
        if (state->fram_full && FramLogGetBufferedRecords(&state->fram_log) <
                                  FramLogGetCapacityRecords(&state->fram_log)) {
          state->fram_full = false;
          state->fram_full_logged = false;
          ESP_LOGI(kTag, "FRAM space available; resuming logging");
        }

        if (!state->mesh.is_root && MeshTransportIsConnected(&state->mesh)) {
          (void)MeshTransportSendRecord(&state->mesh, &record);
        }

        if (state->mesh.is_root) {
          PrintJsonRecord(state->node_id_string, &record);
        }
      }
    }

    const TickType_t now_ticks = xTaskGetTickCount();
    const bool periodic_due =
      (pdTICKS_TO_MS(now_ticks - state->last_flush_ticks) >=
       state->settings.sd_flush_period_ms);
    const uint32_t buffered = FramLogGetBufferedRecords(&state->fram_log);
    const bool watermark_hit =
      buffered >= state->settings.fram_flush_watermark_records;

    if (state->sd_logger.is_mounted && buffered > 0 &&
        (watermark_hit || periodic_due)) {
      esp_err_t flush_result = FlushFramToSd(state, false);
      if (flush_result == ESP_OK || flush_result == ESP_ERR_NOT_FOUND) {
        state->sd_error = false;
        state->last_flush_ticks = now_ticks;
        if (FramLogGetBufferedRecords(&state->fram_log) <
            FramLogGetCapacityRecords(&state->fram_log)) {
          state->fram_full = false;
          state->fram_full_logged = false;
        }
      } else {
        state->sd_error = true;
        ESP_LOGW(kTag, "SD flush failed: %s", esp_err_to_name(flush_result));
      }
    }
  }
}

static void
TimeSyncTask(void* context)
{
  app_state_t* state = (app_state_t*)context;

  if (!state->mesh.is_root) {
    while (!TimeSyncIsSystemTimeValid()) {
      if (MeshTransportIsConnected(&state->mesh)) {
        (void)MeshTransportRequestTime(&state->mesh);
      }
      vTaskDelay(pdMS_TO_TICKS(10 * 1000));
    }
  }

  if (state->mesh.is_root) {
    while (true) {
      if (TimeSyncIsSystemTimeValid() &&
          MeshTransportIsConnected(&state->mesh)) {
        const int64_t now_seconds = (int64_t)time(NULL);
        (void)MeshTransportBroadcastTime(&state->mesh, now_seconds);
      }
      vTaskDelay(pdMS_TO_TICKS(CONFIG_APP_TIME_SYNC_PERIOD_S * 1000));
    }
  }

  vTaskDelete(NULL);
}

static esp_err_t
ConsoleFlush(void* context)
{
  app_state_t* state = (app_state_t*)context;
  esp_err_t result = FlushFramToSd(state, true);
  if (result == ESP_OK || result == ESP_ERR_NOT_FOUND) {
    printf("flush complete; remaining=%u\n",
           (unsigned)FramLogGetBufferedRecords(&state->fram_log));
    return ESP_OK;
  }
  printf("flush failed: %s\n", esp_err_to_name(result));
  return result;
}

static esp_err_t
InitSpiBus(spi_host_device_t host)
{
  spi_bus_config_t bus_config = {
    .mosi_io_num = CONFIG_APP_SPI_MOSI_GPIO,
    .miso_io_num = CONFIG_APP_SPI_MISO_GPIO,
    .sclk_io_num = CONFIG_APP_SPI_SCLK_GPIO,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,
  };
  return spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO);
}

static spi_host_device_t
GetSpiHost(void)
{
  return (CONFIG_APP_SPI_HOST == 3) ? SPI3_HOST : SPI2_HOST;
}

void
app_main(void)
{
  esp_err_t result = nvs_flash_init();
  if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
      result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  app_state_t state;
  memset(&state, 0, sizeof(state));

  uint8_t mac[6] = { 0 };
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  FormatMacString(mac, state.node_id_string, sizeof(state.node_id_string));

  ESP_ERROR_CHECK(AppSettingsLoad(&state.settings));

  state.sd_config.batch_target_bytes = state.settings.sd_batch_bytes_target;
  state.sd_config.tail_scan_bytes = CONFIG_APP_SD_TAIL_SCAN_BYTES;
  state.sd_config.file_buffer_bytes = CONFIG_APP_SD_FILE_BUFFER_BYTES;

  state.batch_buffer_size = state.sd_config.batch_target_bytes;
  state.batch_buffer = (uint8_t*)malloc(state.batch_buffer_size);
  if (state.batch_buffer == NULL) {
    state.batch_buffer_size = 64 * 1024;
    state.batch_buffer = (uint8_t*)malloc(state.batch_buffer_size);
  }

  ESP_ERROR_CHECK(TimeSyncInit(&state.time_sync,
                               I2C_NUM_0,
                               CONFIG_APP_I2C_SDA_GPIO,
                               CONFIG_APP_I2C_SCL_GPIO,
                               (uint8_t)CONFIG_APP_DS3231_I2C_ADDR));
  (void)TimeSyncSetSystemFromRtc(&state.time_sync);

  const spi_host_device_t spi_host = GetSpiHost();
  ESP_ERROR_CHECK(InitSpiBus(spi_host));

  ESP_ERROR_CHECK(FramSpiInit(&state.fram_spi,
                              spi_host,
                              CONFIG_APP_FRAM_CS_GPIO,
                              CONFIG_APP_FRAM_ADDR_BYTES));

  ESP_ERROR_CHECK(
    FramLogInit(&state.fram_log, &state.fram_spi, CONFIG_APP_FRAM_SIZE_BYTES));

  SdLoggerInit(&state.sd_logger, &state.sd_config);
  (void)SdLoggerMount(&state.sd_logger, spi_host, CONFIG_APP_SD_CS_GPIO);

  ESP_ERROR_CHECK(
    Max31865ReaderInit(&state.sensor, spi_host, CONFIG_APP_MAX31865_CS_GPIO));

#if CONFIG_APP_NODE_IS_ROOT
  const bool is_root = true;
#else
  const bool is_root = false;
#endif

  const char* router_ssid = CONFIG_APP_WIFI_ROUTER_SSID;
  const char* router_password = CONFIG_APP_WIFI_ROUTER_PASSWORD;

  ESP_ERROR_CHECK(MeshTransportStart(&state.mesh,
                                     is_root,
                                     router_ssid,
                                     router_password,
                                     is_root ? &RootRecordRxCallback : NULL,
                                     NULL,
                                     &state.time_sync));

  if (is_root) {
    esp_err_t sntp_result =
      TimeSyncStartSntpAndWait(CONFIG_APP_SNTP_SERVER, 30 * 1000);
    if (sntp_result == ESP_OK) {
      (void)TimeSyncSetRtcFromSystem(&state.time_sync);
    }
  }

  if (state.sd_logger.is_mounted) {
    const int64_t epoch_for_file =
      TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : 0;
    if (EnsureSdSyncedForEpoch(&state, epoch_for_file) != ESP_OK) {
      ESP_LOGW(kTag, "Initial SD sync failed; will retry during flush");
    }
  }

  app_runtime_t runtime = {
    .settings = &state.settings,
    .fram_log = &state.fram_log,
    .sd_logger = &state.sd_logger,
    .sensor = &state.sensor,
    .mesh = &state.mesh,
    .time_sync = &state.time_sync,
    .node_id_string = state.node_id_string,
    .flush_callback = &ConsoleFlush,
    .flush_context = &state,
    .fram_full = &state.fram_full,
  };
  ESP_ERROR_CHECK(ConsoleCommandsStart(&runtime));

  state.log_queue = xQueueCreate(64, sizeof(log_record_t));
  xTaskCreate(SensorTask, "sensor", 4096, &state, 5, NULL);
  xTaskCreate(StorageTask, "storage", 6144, &state, 6, NULL);
  xTaskCreate(TimeSyncTask, "time_sync", 4096, &state, 4, NULL);

  ESP_LOGI(
    kTag, "Started (node=%s root=%d)", state.node_id_string, (int)is_root);
}
