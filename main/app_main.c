#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "app_settings.h"
#include "console_commands.h"
#include "esp_log.h"
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

static const char* kTag = "app";

static void
PrintJsonRecord(const char* node_id, const log_record_t* record)
{
  const double raw_c = record->raw_temp_milli_c / 1000.0;
  const double temp_c = record->temp_milli_c / 1000.0;
  const double resistance_ohm = record->resistance_milli_ohm / 1000.0;
  printf("{\"type\":\"temp\",\"node\":\"%s\",\"ts\":%" PRId64
         ",\"temp_c\":%.3f,"
         "\"raw_c\":%.3f,\"r_ohm\":%.3f,\"seq\":%u}\n",
         node_id,
         record->timestamp_epoch_sec,
         temp_c,
         raw_c,
         resistance_ohm,
         (unsigned)record->sequence);
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
RootRecordRxCallback(const mesh_addr_t* from,
                     const log_record_t* record,
                     void* context)
{
  (void)context;

  char node_id[32];
  FormatMacString(from->addr, node_id, sizeof(node_id));
  PrintJsonRecord(node_id, record);
}

typedef struct
{
  app_settings_t settings;
  fram_spi_t fram_spi;
  fram_log_t fram_log;
  sd_logger_t sd_logger;
  max31865_reader_t sensor;
  mesh_transport_t mesh;
  time_sync_t time_sync;

  QueueHandle_t log_queue;

  uint32_t sequence;
  char node_id_string[32];
} app_state_t;

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
SensorTask(void* context)
{
  app_state_t* state = (app_state_t*)context;

  while (true) {
    const uint32_t period_ms = state->settings.log_period_ms;
    float raw_c = 0;
    float resistance_ohm = 0;

    esp_err_t result =
      Max31865ReaderRead(&state->sensor, &raw_c, &resistance_ohm);
    if (result == ESP_OK) {
      const double cal_c =
        CalibrationModelEvaluate(&state->settings.calibration, raw_c);

      int64_t epoch_sec = 0;
      int32_t millis = 0;
      TimeSyncGetNow(&epoch_sec, &millis);

      log_record_t record;
      memset(&record, 0, sizeof(record));
      record.sequence = state->sequence++;
      record.timestamp_epoch_sec = TimeSyncIsSystemTimeValid() ? epoch_sec : 0;
      record.timestamp_millis = millis;
      record.raw_temp_milli_c = (int32_t)llround(raw_c * 1000.0);
      record.temp_milli_c = (int32_t)llround(cal_c * 1000.0);
      record.resistance_milli_ohm = (int32_t)llround(resistance_ohm * 1000.0);
      record.flags = 0;
      if (TimeSyncIsSystemTimeValid()) {
        record.flags |= LOG_RECORD_FLAG_TIME_VALID;
      }
      if (state->settings.calibration.is_valid) {
        record.flags |= LOG_RECORD_FLAG_CAL_VALID;
      }

      (void)xQueueSend(state->log_queue, &record, 0);
    } else {
      ESP_LOGW(kTag, "sensor read failed: %s", esp_err_to_name(result));
    }

    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }
}

static esp_err_t
FlushSomeToSd(app_state_t* state, uint32_t max_records)
{
  if (!state->sd_logger.is_mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  log_record_t record;
  uint32_t flushed = 0;
  while (flushed < max_records &&
         FramLogGetBufferedRecords(&state->fram_log) > 0) {
    // Do not remove from FRAM until we have durably appended to SD.
    esp_err_t peek_result = FramLogPeekOldest(&state->fram_log, &record);
    if (peek_result == ESP_ERR_NOT_FOUND) {
      break;
    }
    // Even if the record CRC is invalid, still attempt to persist the raw
    // fields for forensics. SdLoggerAppendCsv() will write whatever bytes are
    // in `record`.
    esp_err_t write_result =
      SdLoggerAppendCsv(&state->sd_logger, state->node_id_string, &record);
    if (write_result != ESP_OK) {
      // Leave record in FRAM for retry later.
      return write_result;
    }

    // Now it is safe to advance the FRAM read pointer.
    esp_err_t discard_result = FramLogDiscardOldest(&state->fram_log);
    if (discard_result != ESP_OK) {
      return discard_result;
    }
    flushed++;
  }
  return ESP_OK;
}

static void
StorageTask(void* context)
{
  app_state_t* state = (app_state_t*)context;

  while (true) {
    log_record_t record;
    if (xQueueReceive(state->log_queue, &record, pdMS_TO_TICKS(500)) ==
        pdTRUE) {
      (void)FramLogAppend(&state->fram_log, &record);

      // Stream to root if we are a leaf node.
      if (!state->mesh.is_root && MeshTransportIsConnected(&state->mesh)) {
        (void)MeshTransportSendRecord(&state->mesh, &record);
      }

      // Root can optionally emit its own sensor readings to the host stream as
      // well.
      if (state->mesh.is_root) {
        PrintJsonRecord(state->node_id_string, &record);
      }
    }

    // Auto-flush to SD if buffered >= watermark.
    const uint32_t buffered = FramLogGetBufferedRecords(&state->fram_log);
    if (state->sd_logger.is_mounted &&
        buffered >= state->settings.fram_flush_watermark_records) {
      (void)FlushSomeToSd(state, 512);
    }
  }
}

static void
TimeSyncTask(void* context)
{
  app_state_t* state = (app_state_t*)context;

  // Leaf nodes: if time is still invalid after RTC attempt, request from root.
  if (!state->mesh.is_root) {
    while (!TimeSyncIsSystemTimeValid()) {
      if (MeshTransportIsConnected(&state->mesh)) {
        (void)MeshTransportRequestTime(&state->mesh);
      }
      vTaskDelay(pdMS_TO_TICKS(10 * 1000));
    }
  }

  // Root nodes: periodically broadcast time to all nodes.
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

  // Node ID (MAC).
  uint8_t mac[6] = { 0 };
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  FormatMacString(mac, state.node_id_string, sizeof(state.node_id_string));

  ESP_ERROR_CHECK(AppSettingsLoad(&state.settings));

  // I2C + RTC.
  ESP_ERROR_CHECK(TimeSyncInit(&state.time_sync,
                               I2C_NUM_0,
                               CONFIG_APP_I2C_SDA_GPIO,
                               CONFIG_APP_I2C_SCL_GPIO,
                               (uint8_t)CONFIG_APP_DS3231_I2C_ADDR));

  // Prefer RTC at boot to get plausible timestamps quickly.
  (void)TimeSyncSetSystemFromRtc(&state.time_sync);

  // SPI bus (shared by MAX31865, FRAM, SD).
  const spi_host_device_t spi_host = GetSpiHost();
  ESP_ERROR_CHECK(InitSpiBus(spi_host));

  // FRAM.
  ESP_ERROR_CHECK(FramSpiInit(&state.fram_spi,
                              spi_host,
                              CONFIG_APP_FRAM_CS_GPIO,
                              CONFIG_APP_FRAM_ADDR_BYTES));

  ESP_ERROR_CHECK(
    FramLogInit(&state.fram_log, &state.fram_spi, CONFIG_APP_FRAM_SIZE_BYTES));

  // SD card (optional).
  (void)SdLoggerMount(&state.sd_logger, spi_host, CONFIG_APP_SD_CS_GPIO);

  // MAX31865.
  ESP_ERROR_CHECK(
    Max31865ReaderInit(&state.sensor, spi_host, CONFIG_APP_MAX31865_CS_GPIO));

  // Mesh.
  const bool is_root = CONFIG_APP_NODE_IS_ROOT;

  // Provide router credentials on all nodes so any node can become root if
  // desired.
  const char* router_ssid = CONFIG_APP_WIFI_ROUTER_SSID;
  const char* router_password = CONFIG_APP_WIFI_ROUTER_PASSWORD;

  ESP_ERROR_CHECK(MeshTransportStart(&state.mesh,
                                     is_root,
                                     router_ssid,
                                     router_password,
                                     is_root ? &RootRecordRxCallback : NULL,
                                     NULL,
                                     &state.time_sync));

  // Root: after starting mesh and obtaining IP, run SNTP and update RTC.
  if (is_root) {
    esp_err_t sntp_result =
      TimeSyncStartSntpAndWait(CONFIG_APP_SNTP_SERVER, 30 * 1000);
    if (sntp_result == ESP_OK) {
      (void)TimeSyncSetRtcFromSystem(&state.time_sync);
    }
  }

  // Console.
  app_runtime_t runtime = {
    .settings = &state.settings,
    .fram_log = &state.fram_log,
    .sd_logger = &state.sd_logger,
    .sensor = &state.sensor,
    .mesh = &state.mesh,
    .time_sync = &state.time_sync,
    .node_id_string = state.node_id_string,
  };
  ESP_ERROR_CHECK(ConsoleCommandsStart(&runtime));

  // Logging pipeline.
  state.log_queue = xQueueCreate(32, sizeof(log_record_t));
  xTaskCreate(SensorTask, "sensor", 4096, &state, 5, NULL);
  xTaskCreate(StorageTask, "storage", 6144, &state, 6, NULL);
  xTaskCreate(TimeSyncTask, "time_sync", 4096, &state, 4, NULL);

  ESP_LOGI(
    kTag, "Started (node=%s root=%d)", state.node_id_string, (int)is_root);
}
