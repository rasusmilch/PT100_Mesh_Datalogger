#include "runtime_manager.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "fram_i2c.h"
#include "fram_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "max31865_reader.h"
#include "mesh_transport.h"
#include "data_csv.h"
#include "data_port.h"
#include "sd_logger.h"
#include "time_sync.h"
#include "wifi_service.h"

static const char* kTag = "runtime";
static const uint32_t kSdFlushMaxRecordsPerPass = 100;
static const uint32_t kSdFlushMaxMsPerPass = 50;
static const uint32_t kSdFlushFailureBackoffMs = 5000;

typedef struct
{
  app_settings_t settings;
  fram_i2c_t fram_i2c;
  fram_io_t fram_io;
  fram_log_t fram_log;
  sd_logger_t sd_logger;
  max31865_reader_t sensor;
  mesh_transport_t mesh;
  time_sync_t time_sync;
  i2c_bus_t i2c_bus;

  QueueHandle_t log_queue;
  uint8_t* batch_buffer;
  size_t batch_buffer_size;

  TickType_t last_flush_ticks;
  TickType_t sd_backoff_until_ticks;
  uint32_t sd_fail_count;
  uint32_t sd_flush_records_since;
  bool sd_flush_pending;
  bool sd_degraded;
  bool fram_full;
  bool fram_full_logged;

  char node_id_string[32];

  TaskHandle_t sensor_task;
  TaskHandle_t storage_task;
  TaskHandle_t time_sync_task;

  bool initialized;
  bool is_running;
  bool stop_requested;
  bool mesh_started;
} runtime_state_t;

static runtime_state_t g_state;
static app_runtime_t g_runtime;
static esp_err_t
RuntimeFlushToSd(void* context);

static void
SetRunLogPolicy(void)
{
  esp_log_level_set("*", ESP_LOG_ERROR);
}

static void
SetDiagLogPolicy(void)
{
  esp_log_level_set("*", ESP_LOG_INFO);
}

static esp_err_t
FramI2cReadAdapter(void* context, uint32_t addr, void* out, size_t len)
{
  if (context == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (addr > 0xFFFFu) {
    return ESP_ERR_INVALID_ARG;
  }
  return FramI2cRead((const fram_i2c_t*)context, (uint16_t)addr, out, len);
}

static esp_err_t
FramI2cWriteAdapter(void* context, uint32_t addr, const void* data, size_t len)
{
  if (context == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (addr > 0xFFFFu) {
    return ESP_ERR_INVALID_ARG;
  }
  return FramI2cWrite((const fram_i2c_t*)context, (uint16_t)addr, data, len);
}

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
  localtime_r(&time_seconds, &time_info);
  strftime(out, out_size, "%Y-%m-%d", &time_info);
}

static bool
CsvDataPortWriter(const char* bytes, size_t len, void* context)
{
  (void)context;
  DataPortWrite(bytes, len);
  return true;
}

static void
PrintCsvRecord(const char* node_id, const log_record_t* record)
{
  if (!CsvWriteRow(CsvDataPortWriter, NULL, record, node_id)) {
    ESP_LOGW(kTag, "Failed to format CSV line for node %s", node_id);
  }
}

static void
RootRecordRxCallback(const pt100_mesh_addr_t* from,
                     const log_record_t* record,
                     void* context)
{
  (void)context;
  char node_id[32];
  FormatMacString(from->addr, node_id, sizeof(node_id));
  PrintCsvRecord(node_id, record);
}

static esp_err_t
EnsureSdSyncedForEpoch(runtime_state_t* state, int64_t epoch_for_file)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
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
BuildBatchForDay(runtime_state_t* state,
                 const char* target_date,
                 uint8_t* buffer,
                 size_t buffer_size,
                 uint32_t max_records,
                 TickType_t start_ticks,
                 uint32_t max_ms,
                 uint32_t* records_used_out,
                 uint32_t* last_sequence_out,
                 size_t* bytes_used_out)
{
  size_t used = 0;
  uint32_t records_used = 0;
  uint32_t last_seq = 0;

  const uint32_t buffered = FramLogGetBufferedRecords(&state->fram_log);
  for (uint32_t offset = 0; offset < buffered; ++offset) {
    if (max_records > 0 && records_used >= max_records) {
      break;
    }
    if (max_ms > 0 &&
        pdTICKS_TO_MS(xTaskGetTickCount() - start_ticks) >= max_ms) {
      break;
    }
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

    char line[208];
    size_t line_len = 0;
    if (!CsvFormatRow(
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
FlushFramToSd(runtime_state_t* state, bool flush_all)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
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
    esp_err_t batch_result =
      BuildBatchForDay(state,
                       day_string,
                       state->batch_buffer,
                       state->batch_buffer_size,
                       0,
                       xTaskGetTickCount(),
                       0,
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
MarkSdFailure(runtime_state_t* state, const char* context, esp_err_t error)
{
  if (state == NULL) {
    return;
  }
  state->sd_degraded = true;
  state->sd_fail_count++;
  state->sd_backoff_until_ticks =
    xTaskGetTickCount() + pdMS_TO_TICKS(kSdFlushFailureBackoffMs);
  ESP_LOGW(kTag,
           "%s: %s (failures=%u, backoff=%ums)",
           context,
           esp_err_to_name(error),
           (unsigned)state->sd_fail_count,
           (unsigned)kSdFlushFailureBackoffMs);
}

static esp_err_t
SdFlushWorkerTick(runtime_state_t* state,
                  uint32_t max_records,
                  uint32_t max_ms,
                  uint32_t* records_flushed_out,
                  bool* more_pending_out)
{
  if (records_flushed_out != NULL) {
    *records_flushed_out = 0;
  }
  if (more_pending_out != NULL) {
    *more_pending_out = false;
  }
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!state->sd_logger.is_mounted) {
    return ESP_OK;
  }
  if (state->batch_buffer == NULL || state->batch_buffer_size == 0) {
    return ESP_ERR_NO_MEM;
  }
  const TickType_t now_ticks = xTaskGetTickCount();
  if (state->sd_backoff_until_ticks != 0 &&
      now_ticks < state->sd_backoff_until_ticks) {
    return ESP_OK;
  }

  if (FramLogGetBufferedRecords(&state->fram_log) == 0) {
    return ESP_OK;
  }

  log_record_t first_record;
  esp_err_t peek_result = FramLogPeekOldest(&state->fram_log, &first_record);
  if (peek_result == ESP_ERR_INVALID_RESPONSE) {
    ESP_LOGE(kTag, "Corrupted FRAM record detected during SD flush");
    (void)FramLogSkipCorruptedRecord(&state->fram_log);
    return ESP_OK;
  }
  if (peek_result != ESP_OK) {
    return peek_result;
  }

  const int64_t epoch_for_file = (first_record.timestamp_epoch_sec > 0)
                                   ? first_record.timestamp_epoch_sec
                                   : (int64_t)time(NULL);
  esp_err_t sync_result = EnsureSdSyncedForEpoch(state, epoch_for_file);
  if (sync_result != ESP_OK) {
    MarkSdFailure(state, "SD sync failed", sync_result);
    return sync_result;
  }

  char day_string[16];
  BuildDateStringFromRecord(&first_record, day_string, sizeof(day_string));

  uint32_t records_used = 0;
  uint32_t last_seq = 0;
  size_t bytes_used = 0;
  esp_err_t batch_result =
    BuildBatchForDay(state,
                     day_string,
                     state->batch_buffer,
                     state->batch_buffer_size,
                     max_records,
                     now_ticks,
                     max_ms,
                     &records_used,
                     &last_seq,
                     &bytes_used);
  if (batch_result != ESP_OK) {
    return batch_result;
  }
  if (records_used == 0 || bytes_used == 0) {
    return ESP_OK;
  }

  esp_err_t write_result = SdLoggerAppendVerifiedBatch(
    &state->sd_logger, state->batch_buffer, bytes_used, last_seq);
  if (write_result != ESP_OK) {
    SdLoggerClose(&state->sd_logger);
    MarkSdFailure(state, "SD append failed", write_result);
    return write_result;
  }

  for (uint32_t index = 0; index < records_used; ++index) {
    esp_err_t discard_result = FramLogDiscardOldest(&state->fram_log);
    if (discard_result != ESP_OK) {
      return discard_result;
    }
  }

  if (records_flushed_out != NULL) {
    *records_flushed_out = records_used;
  }
  if (more_pending_out != NULL) {
    *more_pending_out = (FramLogGetBufferedRecords(&state->fram_log) > 0);
  }
  return ESP_OK;
}

static void
SensorTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;

  while (!state->stop_requested) {
    const uint32_t period_ms = state->settings.log_period_ms;

    max31865_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    esp_err_t result = Max31865ReadOnce(&state->sensor, &sample);

    log_record_t record;
    memset(&record, 0, sizeof(record));
    record.sequence = FramLogNextSequence(&state->fram_log);

    int64_t epoch_sec = 0;
    int32_t millis = 0;
    TimeSyncGetNow(&epoch_sec, &millis);
    const bool time_valid = TimeSyncIsSystemTimeValid();
    record.timestamp_epoch_sec = time_valid ? epoch_sec : (int64_t)0;
    record.timestamp_millis = time_valid ? millis : 0;

    if (result == ESP_OK) {
      const double cal_c = CalibrationModelEvaluate(
        &state->settings.calibration, sample.temperature_c);
      record.raw_temp_milli_c = (int32_t)llround(sample.temperature_c * 1000.0);
      record.temp_milli_c = (int32_t)llround(cal_c * 1000.0);
      record.resistance_milli_ohm =
        (int32_t)llround(sample.resistance_ohm * 1000.0);
      if (sample.fault_present) {
        record.flags |= LOG_RECORD_FLAG_SENSOR_FAULT;
      }
    } else {
      ESP_LOGW(kTag, "sensor read failed: %s", esp_err_to_name(result));
      record.flags |= LOG_RECORD_FLAG_SENSOR_FAULT;
    }

    if (time_valid) {
      record.flags |= LOG_RECORD_FLAG_TIME_VALID;
    }
    if (state->settings.calibration.is_valid) {
      record.flags |= LOG_RECORD_FLAG_CAL_VALID;
    }
    if (state->sd_degraded) {
      record.flags |= LOG_RECORD_FLAG_SD_ERROR;
    }
    if (state->fram_full) {
      record.flags |= LOG_RECORD_FLAG_FRAM_FULL;
    }
    if (MeshTransportIsConnected(&state->mesh)) {
      record.flags |= LOG_RECORD_FLAG_MESH_CONNECTED;
    }

    (void)xQueueSend(state->log_queue, &record, 0);
    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }

  state->sensor_task = NULL;
  vTaskDelete(NULL);
}

static void
StorageTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  state->last_flush_ticks = xTaskGetTickCount();

  while (!state->stop_requested ||
         uxQueueMessagesWaiting(state->log_queue) > 0) {
    log_record_t record;
    if (xQueueReceive(state->log_queue, &record, pdMS_TO_TICKS(500)) ==
        pdTRUE) {
      if (state->fram_full) {
        record.flags |= LOG_RECORD_FLAG_FRAM_FULL;
      }
      PrintCsvRecord(state->node_id_string, &record);

      if (!state->mesh.is_root && MeshTransportIsConnected(&state->mesh)) {
        (void)MeshTransportSendRecord(&state->mesh, &record);
      }

      if (state->fram_i2c.initialized) {
        if (state->fram_full) {
          if (!state->fram_full_logged) {
            ESP_LOGW(
              kTag,
              "FRAM is full; skipping new appends until flush succeeds");
            state->fram_full_logged = true;
          }
        } else {
          esp_err_t append_result = FramLogAppend(&state->fram_log, &record);
          if (append_result == ESP_ERR_NO_MEM) {
            state->fram_full = true;
            state->fram_full_logged = false;
            record.flags |= LOG_RECORD_FLAG_FRAM_FULL;
            ESP_LOGW(
              kTag, "FRAM is full; new samples will be dropped until flush");
            PrintCsvRecord(state->node_id_string, &record);
          } else if (append_result != ESP_OK) {
            ESP_LOGE(
              kTag, "FRAM append failed: %s", esp_err_to_name(append_result));
          } else {
            state->sd_flush_records_since++;
          }
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

    if (periodic_due) {
      state->sd_flush_pending = true;
      state->last_flush_ticks = now_ticks;
    }
    if (watermark_hit) {
      state->sd_flush_pending = true;
    }

    if (state->sd_flush_pending) {
      uint32_t flushed = 0;
      bool more_pending = false;
      esp_err_t flush_result = SdFlushWorkerTick(state,
                                                 kSdFlushMaxRecordsPerPass,
                                                 kSdFlushMaxMsPerPass,
                                                 &flushed,
                                                 &more_pending);
      if (flush_result == ESP_OK) {
        if (flushed > 0) {
          state->sd_flush_records_since = 0;
          if (FramLogGetBufferedRecords(&state->fram_log) <
              FramLogGetCapacityRecords(&state->fram_log)) {
            state->fram_full = false;
            state->fram_full_logged = false;
          }
        }
        state->sd_flush_pending = more_pending;
      }
    }
  }

  if (state->sd_logger.is_mounted) {
    (void)SdFlushWorkerTick(state,
                            kSdFlushMaxRecordsPerPass,
                            kSdFlushMaxMsPerPass,
                            NULL,
                            NULL);
  }

  state->storage_task = NULL;
  vTaskDelete(NULL);
}

static void
TimeSyncTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;

  if (!state->mesh.is_root) {
    while (!TimeSyncIsSystemTimeValid() && !state->stop_requested) {
      if (MeshTransportIsConnected(&state->mesh)) {
        (void)MeshTransportRequestTime(&state->mesh);
      }
      vTaskDelay(pdMS_TO_TICKS(10 * 1000));
    }
  }

  if (state->mesh.is_root) {
    while (!state->stop_requested) {
      if (TimeSyncIsSystemTimeValid() &&
          MeshTransportIsConnected(&state->mesh)) {
        const int64_t now_seconds = (int64_t)time(NULL);
        (void)MeshTransportBroadcastTime(&state->mesh, now_seconds);
      }
      vTaskDelay(pdMS_TO_TICKS(CONFIG_APP_TIME_SYNC_PERIOD_S * 1000));
    }
  }

  state->time_sync_task = NULL;
  vTaskDelete(NULL);
}

static esp_err_t
RuntimeFlushToSd(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t result = FlushFramToSd(state, true);
  if (result == ESP_OK || result == ESP_ERR_NOT_FOUND) {
    ESP_LOGI(kTag,
             "flush complete; remaining=%u",
             (unsigned)FramLogGetBufferedRecords(&state->fram_log));
    return ESP_OK;
  }
  ESP_LOGE(kTag, "flush failed: %s", esp_err_to_name(result));
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

static void
InitializeRuntimeStruct(void)
{
  memset(&g_state, 0, sizeof(g_state));
  memset(&g_runtime, 0, sizeof(g_runtime));

  g_runtime.settings = &g_state.settings;
  g_runtime.fram_i2c = &g_state.fram_i2c;
  g_runtime.fram_io = &g_state.fram_io;
  g_runtime.fram_log = &g_state.fram_log;
  g_runtime.sd_logger = &g_state.sd_logger;
  g_runtime.sensor = &g_state.sensor;
  g_runtime.mesh = &g_state.mesh;
  g_runtime.time_sync = &g_state.time_sync;
  g_runtime.i2c_bus = &g_state.i2c_bus;
  g_runtime.node_id_string = g_state.node_id_string;
  g_runtime.flush_callback = &RuntimeFlushToSd;
  g_runtime.flush_context = &g_state;
  g_runtime.fram_full = &g_state.fram_full;
}

const app_runtime_t*
RuntimeGetRuntime(void)
{
  return g_state.initialized ? &g_runtime : NULL;
}

esp_err_t
RuntimeManagerInit(void)
{
  InitializeRuntimeStruct();
  esp_err_t first_error = ESP_OK;

  esp_err_t data_port_result = DataPortInit();
  if (data_port_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = data_port_result;
    }
    ESP_LOGE(
      kTag, "Data port init failed: %s", esp_err_to_name(data_port_result));
  }

  uint8_t mac[6] = { 0 };
  esp_err_t mac_result = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if (mac_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = mac_result;
    }
    ESP_LOGE(kTag, "esp_read_mac failed: %s", esp_err_to_name(mac_result));
  }
  FormatMacString(mac, g_state.node_id_string, sizeof(g_state.node_id_string));

  esp_err_t settings_result = AppSettingsLoad(&g_state.settings);
  if (settings_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = settings_result;
    }
    ESP_LOGE(
      kTag, "AppSettingsLoad failed: %s", esp_err_to_name(settings_result));
  }
  AppSettingsApplyTimeZone(&g_state.settings);

  const uint32_t i2c_frequency_hz = 400000;
  esp_err_t i2c_result = I2cBusInit(&g_state.i2c_bus,
                                    I2C_NUM_0,
                                    CONFIG_APP_I2C_SDA_GPIO,
                                    CONFIG_APP_I2C_SCL_GPIO,
                                    i2c_frequency_hz);
  if (i2c_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = i2c_result;
    }
    ESP_LOGE(kTag, "I2cBusInit failed: %s", esp_err_to_name(i2c_result));
  }
  g_state.fram_io.context = &g_state.fram_i2c;
  g_state.fram_io.read = &FramI2cReadAdapter;
  g_state.fram_io.write = &FramI2cWriteAdapter;

  sd_logger_config_t sd_config = {
    .batch_target_bytes = g_state.settings.sd_batch_bytes_target,
    .tail_scan_bytes = CONFIG_APP_SD_TAIL_SCAN_BYTES,
    .file_buffer_bytes = CONFIG_APP_SD_FILE_BUFFER_BYTES,
  };
  SdLoggerInit(&g_state.sd_logger, &sd_config);

  g_state.batch_buffer_size = g_state.sd_logger.config.batch_target_bytes;
  g_state.batch_buffer = (uint8_t*)malloc(g_state.batch_buffer_size);
  if (g_state.batch_buffer == NULL) {
    g_state.batch_buffer_size = 64 * 1024;
    g_state.batch_buffer = (uint8_t*)malloc(g_state.batch_buffer_size);
  }

  esp_err_t time_result = TimeSyncInit(
    &g_state.time_sync, &g_state.i2c_bus, (uint8_t)CONFIG_APP_DS3231_I2C_ADDR);
  if (time_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = time_result;
    }
    ESP_LOGE(kTag, "TimeSyncInit failed: %s", esp_err_to_name(time_result));
  }
  if (time_result == ESP_OK) {
    (void)TimeSyncSetSystemFromRtc(&g_state.time_sync);
  }

  const spi_host_device_t spi_host = GetSpiHost();
  esp_err_t bus_result = InitSpiBus(spi_host);
  if (bus_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = bus_result;
    }
    ESP_LOGE(
      kTag, "spi_bus_initialize failed: %s", esp_err_to_name(bus_result));
  }

  esp_err_t fram_i2c_result = ESP_ERR_INVALID_STATE;
  if (g_state.i2c_bus.initialized) {
    fram_i2c_result = FramI2cInit(&g_state.fram_i2c,
                                  g_state.i2c_bus.handle,
                                  (uint8_t)CONFIG_APP_FRAM_I2C_ADDR,
                                  CONFIG_APP_FRAM_SIZE_BYTES,
                                  g_state.i2c_bus.frequency_hz);
  }
  if (fram_i2c_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = fram_i2c_result;
    }
    ESP_LOGE(kTag, "FramI2cInit failed: %s", esp_err_to_name(fram_i2c_result));
  }

  esp_err_t fram_log_result =
    FramLogInit(&g_state.fram_log, g_state.fram_io, CONFIG_APP_FRAM_SIZE_BYTES);
  if (fram_log_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = fram_log_result;
    }
    ESP_LOGE(kTag, "FramLogInit failed: %s", esp_err_to_name(fram_log_result));
  }

  (void)SdLoggerMount(&g_state.sd_logger, spi_host, CONFIG_APP_SD_CS_GPIO);

  esp_err_t sensor_result =
    Max31865ReaderInit(&g_state.sensor, spi_host, CONFIG_APP_MAX31865_CS_GPIO);
  if (sensor_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = sensor_result;
    }
    ESP_LOGE(
      kTag, "Max31865ReaderInit failed: %s", esp_err_to_name(sensor_result));
  }

  g_state.log_queue = xQueueCreate(64, sizeof(log_record_t));
  if (g_state.log_queue == NULL) {
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to create log queue");
  }

  g_state.initialized = true;
  return first_error;
}

static void
EnsureSdMounted(void)
{
  if (!g_state.sd_logger.is_mounted) {
    esp_err_t mount_result =
      SdLoggerMount(&g_state.sd_logger, GetSpiHost(), CONFIG_APP_SD_CS_GPIO);
    if (mount_result != ESP_OK) {
      MarkSdFailure(&g_state, "SD mount failed", mount_result);
    }
  }
}

esp_err_t
RuntimeStart(void)
{
  if (!g_state.initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (g_state.is_running) {
    return ESP_OK;
  }
  if (g_state.sensor_task != NULL || g_state.storage_task != NULL ||
      g_state.time_sync_task != NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  if (g_state.log_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }
  if (g_state.batch_buffer == NULL || g_state.batch_buffer_size == 0) {
    return ESP_ERR_NO_MEM;
  }

  g_state.stop_requested = false;
  g_state.fram_full = false;
  g_state.fram_full_logged = false;
  g_state.sd_degraded = false;
  g_state.sd_fail_count = 0;
  g_state.sd_backoff_until_ticks = 0;
  g_state.sd_flush_records_since = 0;
  g_state.sd_flush_pending = false;

  EnsureSdMounted();

#if CONFIG_APP_NODE_IS_ROOT
  const bool is_root = true;
#else
  const bool is_root = false;
#endif

  // Only the root should ever be configured with upstream router credentials.
  // Non-root nodes should focus on joining the Mesh-Lite network.
  const char* router_ssid = "";
  const char* router_password = "";

  bool router_disabled = false;
#if defined(CONFIG_APP_MESH_DISABLE_ROUTER)
  // CONFIG_APP_MESH_DISABLE_ROUTER
  // is a Kconfig bool and
  // expands to 0 or 1.
  router_disabled = (CONFIG_APP_MESH_DISABLE_ROUTER != 0);
#endif
  if (is_root && !router_disabled) {
    router_ssid = CONFIG_APP_WIFI_ROUTER_SSID;
    router_password = CONFIG_APP_WIFI_ROUTER_PASSWORD;
  }

  if (!g_state.mesh_started) {
    esp_err_t wifi_result = WifiServiceAcquire(WIFI_SERVICE_MODE_MESH);
    if (wifi_result != ESP_OK) {
      ESP_LOGE(
        kTag, "Wi-Fi service start failed: %s", esp_err_to_name(wifi_result));
      return wifi_result;
    }

    esp_err_t mesh_result =
      MeshTransportStart(&g_state.mesh,
                         is_root,
                         router_ssid,
                         router_password,
                         is_root ? &RootRecordRxCallback : NULL,
                         NULL,
                         &g_state.time_sync);
    if (mesh_result == ESP_OK) {
      g_state.mesh_started = true;
    } else {
      ESP_LOGE(kTag, "Mesh start failed: %s", esp_err_to_name(mesh_result));
      (void)WifiServiceRelease();
      return mesh_result;
    }
  }

  if (g_state.mesh.is_root) {
    esp_err_t sntp_result =
      TimeSyncStartSntpAndWait(CONFIG_APP_SNTP_SERVER, 30 * 1000);
    if (sntp_result == ESP_OK) {
      (void)TimeSyncSetRtcFromSystem(&g_state.time_sync);
    } else {
      ESP_LOGW(kTag, "SNTP sync failed: %s", esp_err_to_name(sntp_result));
    }
  }

  if (g_state.sd_logger.is_mounted) {
    const int64_t epoch_for_file =
      TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : 0;
    esp_err_t sync_result = EnsureSdSyncedForEpoch(&g_state, epoch_for_file);
    if (sync_result != ESP_OK) {
      MarkSdFailure(&g_state, "Initial SD sync failed", sync_result);
    }
  }

  if (!CsvWriteHeader(CsvDataPortWriter, NULL)) {
    ESP_LOGW(kTag, "Failed to write CSV header to data port");
  }

  g_state.is_running = true;

  BaseType_t sensor_created =
    xTaskCreate(&SensorTask, "sensor", 4096, &g_state, 5, &g_state.sensor_task);
  BaseType_t storage_created = xTaskCreate(
    &StorageTask, "storage", 6144, &g_state, 6, &g_state.storage_task);
  BaseType_t time_created = xTaskCreate(
    &TimeSyncTask, "time_sync", 4096, &g_state, 4, &g_state.time_sync_task);

  if (sensor_created != pdPASS || storage_created != pdPASS ||
      time_created != pdPASS) {
    g_state.stop_requested = true;
    g_state.is_running = false;
    const TickType_t wait_start = xTaskGetTickCount();
    while ((g_state.sensor_task != NULL || g_state.storage_task != NULL ||
            g_state.time_sync_task != NULL) &&
           (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < 1000)) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(
    kTag, "Runtime started (node=%s root=%d)", g_state.node_id_string, is_root);
  return ESP_OK;
}

esp_err_t
RuntimeStop(void)
{
  if (!g_state.is_running) {
    return ESP_OK;
  }

  g_state.stop_requested = true;
  g_state.is_running = false;

  const TickType_t wait_start = xTaskGetTickCount();
  while ((g_state.sensor_task != NULL || g_state.storage_task != NULL ||
          g_state.time_sync_task != NULL) &&
         (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < 5000)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (g_state.mesh_started) {
    (void)MeshTransportStop(&g_state.mesh);
    g_state.mesh_started = false;
    (void)WifiServiceRelease();
  }

  SdLoggerClose(&g_state.sd_logger);
  (void)xQueueReset(g_state.log_queue);
  return ESP_OK;
}

bool
RuntimeIsRunning(void)
{
  return g_state.is_running;
}

esp_err_t
EnterRunMode(void)
{
  SetRunLogPolicy();
  esp_err_t result = RuntimeStart();
  if (result != ESP_OK) {
    SetDiagLogPolicy();
  }
  return result;
}

esp_err_t
EnterDiagMode(void)
{
  esp_err_t result = RuntimeStop();
  SetDiagLogPolicy();
  return result;
}
