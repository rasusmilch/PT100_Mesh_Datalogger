#include "runtime_manager.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "alerts/alert_manager.h"
#include "app_net_config.h"
#include "calibration.h"
#include "data_csv.h"
#include "data_port.h"
#include "display_attention.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_port.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "fram_i2c.h"
#include "fram_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "log_rate_limit.h"
#include "max31865_reader.h"
#include "max7219_display.h"
#include "mesh_transport.h"
#include "mqtt_client_wrap.h"
#include "runtime_health.h"
#include "runtime_health_publisher.h"
#include "runtime_state.h"
#include "sd_logger.h"
#include "time_sync.h"
#include "wifi_credentials.h"
#include "wifi_manager.h"
#include "wifi_service.h"

static const char* kTag = "runtime";
static const uint32_t kSdFlushMaxRecordsPerPass = 100;
static const uint32_t kSdFlushMaxMsPerPass = 50;
static const uint32_t kSdFlushTimeSliceMs = 50;
static const uint32_t kSdFlushWarnIntervalMs = 10000;
static const uint32_t kSdFlushFailureBackoffMs = 5000;
static const uint32_t kSdFlushMinIntervalMs = 200;
static const uint32_t kSdDetectPollIntervalMs = 250;
static const uint32_t kExportQueueDepth = 64;
static const uint32_t kExportOutboxDepth = CONFIG_APP_EXPORT_OUTBOX_DEPTH;
static const uint32_t kBrokerOutboxDepth = CONFIG_APP_BROKER_OUTBOX_DEPTH;
static const uint32_t kExportLogRateLimitMs = CONFIG_APP_EXPORT_RATE_LIMIT_MS;
static const TickType_t kSdIoLockTimeoutTicks = pdMS_TO_TICKS(2000);
static const int32_t kStopDrainHardMaxDefaultMs = 15000;

static void
RuntimeNotifyTask(TaskHandle_t handle)
{
  if (handle == NULL) {
    return;
  }
  xTaskNotifyGive(handle);
}

static void
RuntimeNotifyAllRunTasks(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  RuntimeNotifyTask(state->sensor_task);
  RuntimeNotifyTask(state->storage_task);
  RuntimeNotifyTask(state->export_task);
  RuntimeNotifyTask(state->export_network_task);
  RuntimeNotifyTask(state->broker_task);
  RuntimeNotifyTask(state->bridge_task);
  RuntimeNotifyTask(state->health_publisher_task);
  RuntimeNotifyTask(state->alert_monitor_task);
  RuntimeNotifyTask(state->alert_sender_task);
  RuntimeNotifyTask(state->wifi_direct_task);
  RuntimeNotifyTask(state->topology_task);
  RuntimeNotifyTask(state->time_sync_task);
}

static void
RuntimePauseSpiUsers(runtime_state_t* state, uint32_t timeout_ms)
{
  if (state == NULL) {
    return;
  }

  state->spi_pause_requested = true;
  RuntimeNotifyTask(state->display_task);

  const TickType_t wait_start = xTaskGetTickCount();
  while (state->display_task != NULL && !state->spi_paused &&
         (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < timeout_ms)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (state->display_task != NULL && !state->spi_paused) {
    ESP_LOGW(kTag,
             "Stop: SPI pause timed out; display task still active (%p)",
             state->display_task);
  }
}

static void
RuntimeInterruptibleDelayTicks(TickType_t delay_ticks)
{
  (void)ulTaskNotifyTake(pdTRUE, delay_ticks);
}

static int32_t
ResolveStopDrainMaxMs(void)
{
  if (CONFIG_APP_STOP_DRAIN_MAX_MS < 0) {
    ESP_LOGW(kTag,
             "Stop drain max ms is unlimited; enforcing hard cap of %d ms",
             kStopDrainHardMaxDefaultMs);
    return kStopDrainHardMaxDefaultMs;
  }
  return CONFIG_APP_STOP_DRAIN_MAX_MS;
}

/**
 * @brief Execute UpdateCachedBool.
 * @param state Parameter state.
 * @param field Parameter field.
 * @param value Parameter value.
 */
static void
UpdateCachedBool(runtime_state_t* state, bool* field, bool value)
{
  if (state == NULL || field == NULL) {
    return;
  }
  if (*field == value) {
    return;
  }
  *field = value;
  RuntimeHealthMarkDirty(state);
}

/**
 * @brief Execute UpdateCachedUint32.
 * @param state Parameter state.
 * @param field Parameter field.
 * @param value Parameter value.
 */
static void
UpdateCachedUint32(runtime_state_t* state, uint32_t* field, uint32_t value)
{
  if (state == NULL || field == NULL) {
    return;
  }
  if (*field == value) {
    return;
  }
  *field = value;
  RuntimeHealthMarkDirty(state);
}

static bool
SdCardPresent(const runtime_state_t* state)
{
  if (state == NULL) {
    return true;
  }
  return SdCardDetectIsPresent(&state->sd_card_detect);
}

/**
 * @brief Execute UpdateCachedInt32.
 * @param state Parameter state.
 * @param field Parameter field.
 * @param value Parameter value.
 */
static void
UpdateCachedInt32(runtime_state_t* state, int32_t* field, int32_t value)
{
  if (state == NULL || field == NULL) {
    return;
  }
  if (*field == value) {
    return;
  }
  *field = value;
  RuntimeHealthMarkDirty(state);
}

/**
 * @brief Execute RuntimeDiagHeapCheck.
 * @param state Parameter state.
 * @param context Parameter context.
 * @param force Parameter force.
 * @return Return the function result.
 */
bool
RuntimeDiagHeapCheck(runtime_state_t* state, const char* context, bool force)
{
  if (state == NULL) {
    return true;
  }
  if (!force && !state->diag_heap_check_enabled) {
    return true;
  }
  const bool ok = heap_caps_check_integrity_all(true);
  if (!ok) {
    ESP_LOGE(kTag,
             "HEAP integrity check failed: %s",
             (context != NULL) ? context : "unknown");
    if (!state->is_running) {
      ESP_LOGE(kTag, "Diagnostics mode heap check failure; aborting");
      abort();
    }
  }
  return ok;
}

static void
MarkSdIoLockFailure(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  state->sd_degraded = true;
  state->sd_fail_count++;
  UpdateCachedBool(state, &state->cached_status.sd_degraded, true);
  UpdateCachedUint32(
    state, &state->cached_status.sd_fail_count, state->sd_fail_count);
  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
}

/**
 * @brief Execute RuntimeSdIoLock.
 * @param state Parameter state.
 * @param timeout_ticks Parameter timeout_ticks.
 * @return Return the function result.
 */
bool
RuntimeSdIoLock(runtime_state_t* state, TickType_t timeout_ticks)
{
  if (state == NULL) {
    return false;
  }
  if (state->sd_io_mutex == NULL) {
    ESP_LOGE(kTag, "SD I/O mutex unavailable; marking SD degraded");
    MarkSdIoLockFailure(state);
    return false;
  }
  if (xSemaphoreTake(state->sd_io_mutex, timeout_ticks) != pdTRUE) {
    const char* phase =
      (state->stop_requested || !state->is_running) ? "stop/diag" : "runtime";
    ESP_LOGE(kTag,
             "SD I/O mutex timeout during %s; marking SD degraded",
             phase);
    MarkSdIoLockFailure(state);
    return false;
  }
  return true;
}

/**
 * @brief Execute RuntimeSdIoUnlock.
 * @param state Parameter state.
 */
void
RuntimeSdIoUnlock(runtime_state_t* state)
{
  if (state == NULL || state->sd_io_mutex == NULL) {
    return;
  }
  (void)xSemaphoreGive(state->sd_io_mutex);
}

typedef struct
{
  log_record_t record;
  char node_id[32];
} export_item_t;

typedef struct
{
  char topic[128];
  char payload[256];
  uint16_t payload_len;
} broker_publish_item_t;

static runtime_state_t g_state;
static app_runtime_t g_runtime;
static StaticQueue_t g_export_outbox_queue_struct;
static StaticQueue_t g_broker_outbox_queue_struct;
static uint8_t* g_export_outbox_storage = NULL;
static uint8_t* g_broker_outbox_storage = NULL;

static StaticQueue_t g_log_queue_struct;
static uint8_t* g_log_queue_storage = NULL;

static StaticQueue_t g_export_queue_struct;
static uint8_t* g_export_queue_storage = NULL;

static void*
AllocatePreferPsram(size_t bytes)
{
  // Prefer PSRAM to preserve internal heap for Wi-Fi/COEX and internal-only
  // DMA.
  void* buffer = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (buffer != NULL) {
    return buffer;
  }
  return heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

static const sd_csv_append_scratch_t*
BuildSdAppendScratch(const runtime_state_t* state,
                     sd_csv_append_scratch_t* scratch_out)
{
  if (state == NULL || scratch_out == NULL) {
    return NULL;
  }
  scratch_out->io_bounce_bytes = state->sd_logger.io_bounce_bytes;
  scratch_out->io_bounce_capacity = state->sd_logger.io_bounce_capacity;
  scratch_out->verify_readback_bytes = state->sd_logger.verify_readback_bytes;
  scratch_out->verify_readback_capacity =
    state->sd_logger.verify_readback_capacity;
  if (scratch_out->io_bounce_bytes == NULL &&
      scratch_out->verify_readback_bytes == NULL) {
    return NULL;
  }
  return scratch_out;
}
static esp_err_t
RuntimeFlushToSd(void* context);
static void
ClearSdIoError(runtime_state_t* state);
static esp_err_t
DrainFramToSd(runtime_state_t* state,
              bool unmount_on_exit,
              int32_t max_duration_ms,
              int32_t max_records_per_pass,
              int32_t yield_every_records,
              sd_drain_stats_t* out_stats);
static esp_err_t
DrainFramToSdOnStartBestEffort(runtime_state_t* state,
                               sd_drain_stats_t* out_stats);
static void
ControlTask(void* context);

static void
EnsureSdMounted(void);

static void
MarkSdFailure(runtime_state_t* state,
              const char* context,
              const char* operation,
              esp_err_t error,
              int errno_value,
              bool did_unmount);

/**
 * @brief Execute SdMaintenanceTick.
 * @param state Parameter state.
 */
static void
SdMaintenanceTick(runtime_state_t* state)
{
  if (state == NULL || state->sd_logger.is_mounted) {
    return;
  }
  if (!SdCardPresent(state)) {
    return;
  }

  const TickType_t now_ticks = xTaskGetTickCount();
  if (state->sd_backoff_until_ticks != 0 &&
      now_ticks < state->sd_backoff_until_ticks) {
    return;
  }

  const bool was_degraded = state->sd_degraded;
  const uint32_t prev_fail_count = state->sd_fail_count;
  if (!RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
    return;
  }
  esp_err_t mount_result = SdLoggerTryRemount(&state->sd_logger, false);
  RuntimeSdIoUnlock(state);
  if (mount_result != ESP_OK) {
    MarkSdFailure(state, "SD mount failed", "mount", mount_result, 0, false);
    return;
  }

  state->sd_backoff_until_ticks = 0;
  state->sd_degraded = false;
  ClearSdIoError(state);
  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  UpdateCachedBool(
    state, &state->cached_status.sd_degraded, state->sd_degraded);
  UpdateCachedUint32(
    state, &state->cached_status.sd_fail_count, state->sd_fail_count);
  UpdateCachedUint32(state, &state->cached_status.sd_backoff_remaining_ms, 0u);

  if (was_degraded || prev_fail_count != 0u) {
    ESP_LOGW(kTag,
             "SD recovered (mounted). fail_count=%u backoff cleared",
             (unsigned)state->sd_fail_count);
  }

  if (FramLogGetBufferedRecords(&state->fram_log) > 0) {
    // We have backlog and SD is now available. Start draining immediately.
    state->sd_flush_pending = true;
    state->sd_start_drain_pending = true;
    state->sd_next_flush_allowed_ticks = 0;
  }
}

/**
 * @brief Execute ConversionModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
ConversionModeToString(uint8_t mode)
{
  switch ((max31865_conversion_t)mode) {
    case kMax31865ConversionTablePt100:
      return "TABLE";
    case kMax31865ConversionCvdIterative:
      return "CVD";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Execute BridgeModeUsesSerial.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static bool
BridgeModeUsesSerial(mqtt_bridge_mode_t mode)
{
  return mode == MQTT_BRIDGE_SERIAL || mode == MQTT_BRIDGE_BOTH;
}

/**
 * @brief Execute BridgeModeUsesBroker.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static bool
BridgeModeUsesBroker(mqtt_bridge_mode_t mode)
{
  return mode == MQTT_BRIDGE_BROKER || mode == MQTT_BRIDGE_BOTH;
}

/**
 * @brief Execute DoubleNear.
 * @param a Parameter a.
 * @param b Parameter b.
 * @return Return the function result.
 */
static bool
DoubleNear(double a, double b)
{
  return fabs(a - b) <= 1e-6;
}

/**
 * @brief Execute IsSdIoOperation.
 * @param operation Parameter operation.
 * @return Return the function result.
 */
static bool
IsSdIoOperation(const char* operation)
{
  if (operation == NULL) {
    return false;
  }
  return strcmp(operation, "append") == 0 || strcmp(operation, "fflush") == 0 ||
         strcmp(operation, "fsync") == 0 || strcmp(operation, "verify") == 0;
}

/**
 * @brief Execute ClearSdIoError.
 * @param state Parameter state.
 */
static void
ClearSdIoError(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  state->sd_last_io_error_active = false;
  state->sd_last_io_err = ESP_OK;
  state->sd_last_errno = 0;
  UpdateCachedBool(state, &state->cached_status.sd_io_error_active, false);
}

/**
 * @brief Execute UpdateFramFillState.
 * @param state Parameter state.
 */
static void
UpdateFramFillState(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  if (!state->fram_i2c.initialized) {
    state->fram_full = false;
    UpdateCachedUint32(state, &state->cached_status.fram_count, 0);
    UpdateCachedUint32(state, &state->cached_status.fram_capacity, 0);
    UpdateCachedBool(state, &state->cached_status.fram_full, false);
    return;
  }
  const size_t count = FramLogGetCountRecords(&state->fram_log);
  const size_t capacity = FramLogGetCapacityRecords(&state->fram_log);
  UpdateCachedUint32(state, &state->cached_status.fram_count, (uint32_t)count);
  UpdateCachedUint32(
    state, &state->cached_status.fram_capacity, (uint32_t)capacity);
  const bool fram_full = (capacity > 0 && count >= capacity);
  state->fram_full = fram_full;
  UpdateCachedBool(state, &state->cached_status.fram_full, fram_full);
}

/**
 * @brief Execute LogFramOverrunWarning.
 * @param state Parameter state.
 * @param overrun_total Parameter overrun_total.
 * @param fram_count Parameter fram_count.
 * @param fram_capacity Parameter fram_capacity.
 * @param now_ticks Parameter now_ticks.
 */
static void
LogFramOverrunWarning(runtime_state_t* state,
                      uint64_t overrun_total,
                      size_t fram_count,
                      size_t fram_capacity,
                      TickType_t now_ticks)
{
  if (state == NULL || overrun_total == 0) {
    return;
  }
  if (state->last_overrun_records_total == 0 && overrun_total > 0) {
    ESP_LOGW(
      kTag,
      "FRAM buffer full: overwriting oldest records (data loss possible until "
      "SD flush recovers).");
    state->last_overrun_log_ticks = now_ticks;
    state->last_overrun_logged_total = overrun_total;
    return;
  }

  const bool sd_down = (!state->sd_logger.is_mounted || state->sd_degraded);
  const bool overrun_advanced =
    (overrun_total > state->last_overrun_logged_total);
  if (!sd_down || !overrun_advanced) {
    return;
  }

  const uint32_t elapsed_ms =
    (uint32_t)pdTICKS_TO_MS(now_ticks - state->last_overrun_log_ticks);
  if (elapsed_ms < 30000u) {
    return;
  }

  ESP_LOGW(kTag,
           "FRAM overruns ongoing: total=%" PRIu64
           " buffered=%zu/%zu sd_mounted=%s sd_degraded=%s",
           overrun_total,
           fram_count,
           fram_capacity,
           state->sd_logger.is_mounted ? "yes" : "no",
           state->sd_degraded ? "yes" : "no");
  state->last_overrun_log_ticks = now_ticks;
  state->last_overrun_logged_total = overrun_total;
}

/**
 * @brief Execute CalibrationContextMatches.
 * @param settings Parameter settings.
 * @param current Parameter current.
 * @param reason_out Parameter reason_out.
 * @param reason_out_len Parameter reason_out_len.
 * @return Return the function result.
 */
static bool
CalibrationContextMatches(const app_settings_t* settings,
                          const calibration_context_t* current,
                          char* reason_out,
                          size_t reason_out_len)
{
  if (settings == NULL || current == NULL || reason_out == NULL) {
    return false;
  }
  if (!settings->calibration_context_valid) {
    snprintf(reason_out, reason_out_len, "calibration context missing");
    return false;
  }

  const calibration_context_t* stored = &settings->calibration_context;
  if (stored->conversion_mode != current->conversion_mode) {
    snprintf(reason_out,
             reason_out_len,
             "conversion mode changed (stored=%s current=%s)",
             ConversionModeToString(stored->conversion_mode),
             ConversionModeToString(current->conversion_mode));
    return false;
  }
  if (stored->wires != current->wires) {
    snprintf(reason_out,
             reason_out_len,
             "wire count changed (stored=%u current=%u)",
             (unsigned)stored->wires,
             (unsigned)current->wires);
    return false;
  }
  if (stored->filter_hz != current->filter_hz) {
    snprintf(reason_out,
             reason_out_len,
             "filter setting changed (stored=%uHz current=%uHz)",
             (unsigned)stored->filter_hz,
             (unsigned)current->filter_hz);
    return false;
  }
  if (!DoubleNear(stored->rref_ohm, current->rref_ohm)) {
    snprintf(reason_out,
             reason_out_len,
             "Rref changed (stored=%.6f current=%.6f)",
             stored->rref_ohm,
             current->rref_ohm);
    return false;
  }
  if (!DoubleNear(stored->r0_ohm, current->r0_ohm)) {
    snprintf(reason_out,
             reason_out_len,
             "R0 changed (stored=%.6f current=%.6f)",
             stored->r0_ohm,
             current->r0_ohm);
    return false;
  }
  if (stored->table_version != 0 && current->table_version != 0 &&
      stored->table_version != current->table_version) {
    snprintf(reason_out,
             reason_out_len,
             "PT100 table version changed (stored=%u current=%u)",
             (unsigned)stored->table_version,
             (unsigned)current->table_version);
    return false;
  }
  return true;
}

static void
CopyLastSample(runtime_state_t* state,
               int32_t* temp_milli_c,
               bool* valid,
               uint32_t* flags,
               TickType_t* update_ticks);

/**
 * @brief Execute ComputeActiveAttentionMaskFromHealth.
 * @param health Parameter health.
 * @return Return the function result.
 */
static display_attention_mask_t
ComputeActiveAttentionMaskFromHealth(const runtime_health_snapshot_t* health)
{
  if (health == NULL) {
    return 0;
  }

  display_attention_mask_t active = 0;
  const display_attention_mask_t mask = health->disp_attn_mask;

  if ((mask & kDispAttnSdOut) != 0u &&
      (!health->sd_card_present || !health->sd_mounted)) {
    active |= kDispAttnSdOut;
  }
  if ((mask & kDispAttnSdIo) != 0u && health->sd_io_error_active) {
    active |= kDispAttnSdIo;
  }
  if ((mask & kDispAttnFramOvr) != 0u && health->fram_overrun_active) {
    active |= kDispAttnFramOvr;
  }
  if ((mask & kDispAttnRtdFault) != 0u && health->sensor_fault_present) {
    active |= kDispAttnRtdFault;
  }
  if ((mask & kDispAttnTimeBad) != 0u && !health->time_valid) {
    active |= kDispAttnTimeBad;
  }
  if ((mask & kDispAttnMeshDown) != 0u && !health->mesh_connected) {
    active |= kDispAttnMeshDown;
  }

  return active;
}

/**
 * @brief Execute AttentionBitToCode.
 * @param bit Parameter bit.
 * @return Return the function result.
 */
static const char*
AttentionBitToCode(display_attention_bit_t bit)
{
  switch (bit) {
    case kDispAttnSdOut:
      return "SDOUT";
    case kDispAttnSdIo:
      return "SDIO ";
    case kDispAttnFramOvr:
      return "FRAM ";
    case kDispAttnRtdFault:
      return "PROBE";
    case kDispAttnTimeBad:
      return "TIME ";
    case kDispAttnMeshDown:
      return "MESH ";
    default:
      return "ERR  ";
  }
}

/**
 * @brief Execute CopyLastSample.
 * @param state Parameter state.
 * @param temp_milli_c Parameter temp_milli_c.
 * @param valid Parameter valid.
 * @param flags Parameter flags.
 * @param update_ticks Parameter update_ticks.
 */
static void
CopyLastSample(runtime_state_t* state,
               int32_t* temp_milli_c,
               bool* valid,
               uint32_t* flags,
               TickType_t* update_ticks)
{
  if (state == NULL) {
    return;
  }
  taskENTER_CRITICAL(&state->last_temp_lock);
  if (temp_milli_c != NULL) {
    *temp_milli_c = state->last_temp_milli_c;
  }
  if (valid != NULL) {
    *valid = state->last_temp_valid;
  }
  if (flags != NULL) {
    *flags = state->last_flags;
  }
  if (update_ticks != NULL) {
    *update_ticks = state->last_update_ticks;
  }
  taskEXIT_CRITICAL(&state->last_temp_lock);
}

/**
 * @brief Execute FormatTemperatureText.
 * @param out Parameter out.
 * @param out_len Parameter out_len.
 * @param temp_milli_c Parameter temp_milli_c.
 * @param units Parameter units.
 * @param valid Parameter valid.
 */
static void
FormatTemperatureText(char* out,
                      size_t out_len,
                      int32_t temp_milli_c,
                      app_display_units_t units,
                      bool valid)
{
  if (out == NULL || out_len == 0) {
    return;
  }
  out[0] = '\0';
  if (!valid) {
    snprintf(out, out_len, "----");
    return;
  }

  char unit_char = 'C';
  int64_t temp_milli = temp_milli_c;
  // Avoid float formatting in DisplayTask to prevent newlib heap use.
  if (units == APP_DISPLAY_UNITS_F) {
    const int64_t scaled = (int64_t)temp_milli_c * 9;
    const int64_t rounded = (scaled >= 0) ? (scaled + 2) / 5 : (scaled - 2) / 5;
    temp_milli = rounded + 32000;
    unit_char = 'F';
  }

  const int32_t tenths = (int32_t)((temp_milli >= 0) ? (temp_milli + 50) / 100
                                                     : (temp_milli - 50) / 100);
  const int32_t abs_tenths = (tenths < 0) ? -tenths : tenths;
  if (abs_tenths >= 10000) {
    snprintf(out, out_len, (tenths >= 0) ? "HI" : "LO");
    return;
  }

  if (abs_tenths >= 1000) {
    const int32_t whole = tenths / 10;
    snprintf(out, out_len, "%ld%c", (long)whole, unit_char);
    return;
  }

  const int32_t whole = abs_tenths / 10;
  const int32_t frac = abs_tenths % 10;
  if (tenths < 0) {
    snprintf(out, out_len, "-%ld.%ld%c", (long)whole, (long)frac, unit_char);
  } else {
    snprintf(out, out_len, "%ld.%ld%c", (long)whole, (long)frac, unit_char);
  }
  if (strlen(out) > 5) {
    const int32_t whole_no_decimal = tenths / 10;
    snprintf(out, out_len, "%ld%c", (long)whole_no_decimal, unit_char);
  }
}

/**
 * @brief Execute UpdateTimeHealthState.
 * @param state Parameter state.
 * @param time_valid Parameter time_valid.
 */
static void
UpdateTimeHealthState(runtime_state_t* state, bool time_valid)
{
  if (state == NULL) {
    return;
  }
  UpdateCachedBool(state, &state->cached_status.time_valid, time_valid);
  if (!time_valid) {
    UpdateCachedInt32(state, &state->cached_status.utc_offset_sec, 0);
    UpdateCachedBool(state, &state->cached_status.dst_in_effect, false);
    return;
  }

  const time_t now = time(NULL);
  struct tm utc_as_local;
  struct tm local_time;
  gmtime_r(&now, &utc_as_local);
  localtime_r(&now, &local_time);
  const time_t utc_epoch_as_local = mktime(&utc_as_local);
  const long utc_offset_sec = (long)difftime(now, utc_epoch_as_local);
  UpdateCachedInt32(
    state, &state->cached_status.utc_offset_sec, (int32_t)utc_offset_sec);
  UpdateCachedBool(
    state, &state->cached_status.dst_in_effect, (local_time.tm_isdst > 0));
}

/**
 * @brief Execute ComputeSdBackoffRemainingMs.
 * @param state Parameter state.
 * @param now_ticks Parameter now_ticks.
 * @return Return the function result.
 */
static uint32_t
ComputeSdBackoffRemainingMs(const runtime_state_t* state, TickType_t now_ticks)
{
  if (state == NULL || state->sd_backoff_until_ticks == 0 ||
      now_ticks >= state->sd_backoff_until_ticks) {
    return 0;
  }
  return (uint32_t)pdTICKS_TO_MS(state->sd_backoff_until_ticks - now_ticks);
}

/**
 * @brief Execute BuildDisplayTestText.
 * @param text Parameter text.
 * @param text_size Parameter text_size.
 * @param elapsed_ms Parameter elapsed_ms.
 */
static void
BuildDisplayTestText(char* text, size_t text_size, uint32_t elapsed_ms)
{
  if (text == NULL || text_size < 6u) {
    return;
  }

  const uint32_t step_ms = 250u;
  const char* banners[] = { "IDLE ", "STOP " };
  const size_t banner_steps = 4u;
  const char glyphs[] = "IDLESTOP ";
  const size_t glyph_count = sizeof(glyphs) - 1u;

  const size_t banner_total_steps =
    (sizeof(banners) / sizeof(banners[0])) * banner_steps;
  const size_t glyph_total_steps = glyph_count * 5u;
  const size_t total_steps = banner_total_steps + glyph_total_steps;

  size_t step = 0u;
  if (total_steps > 0u) {
    step = (elapsed_ms / step_ms) % total_steps;
  }

  if (step < banner_total_steps) {
    const size_t banner_index = step / banner_steps;
    snprintf(text, text_size, "%s", banners[banner_index]);
    return;
  }

  step -= banner_total_steps;
  const size_t glyph_index = step / 5u;
  const size_t position = step % 5u;
  memset(text, ' ', text_size);
  text[5] = '\0';
  if (glyph_index < glyph_count && position < 5u) {
    text[position] = glyphs[glyph_index];
  }
}

/**
 * @brief Execute DisplayTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the DisplayTask task.
 */
static void
DisplayTask(void* context)
{
  // DisplayTask must only read RuntimeHealth snapshot; do not call subsystem
  // APIs here. Guardrail: grep -R
  // "MeshTransportIsConnected|esp_mesh_lite_get_level|TimeSyncIsSystemTimeValid|SdLogger|FramLog"
  // main/*display*
  runtime_state_t* state = (runtime_state_t*)context;
  char last_text[12] = { 0 };
  display_attention_mask_t last_active_mask = 0;
  display_attention_mask_t last_warn_mask = 0;
  size_t code_index = 0;
  size_t warn_code_index = 0;
  uint32_t last_code_tick = 0;
  uint32_t last_warn_tick = 0;
  const uint32_t warn_overlay_period_ms = 10000u;
  const uint32_t warn_overlay_duration_ms = 2000u;

  while (state != NULL) {
    if (!state->display_initialized) {
      if (state->spi_pause_requested) {
        state->spi_paused = true;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (state->spi_pause_requested) {
      state->spi_paused = true;
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(50));
      continue;
    }
    state->spi_paused = false;

    if (state->display_test_active) {
      const TickType_t now_ticks = xTaskGetTickCount();
      if (now_ticks < state->display_test_until_ticks) {
        char text[12];
        const uint32_t elapsed_ms =
          (uint32_t)pdTICKS_TO_MS(now_ticks - state->display_test_start_ticks);
        BuildDisplayTestText(text, sizeof(text), elapsed_ms);
        if (strncmp(last_text, text, sizeof(last_text)) != 0) {
          Max7219DisplaySetText(&state->display, text);
          snprintf(last_text, sizeof(last_text), "%s", text);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
        continue;
      }
      state->display_test_active = false;
      Max7219DisplayClear(&state->display);
      last_text[0] = '\0';
      last_active_mask = 0;
      last_warn_mask = 0;
    }

    const TickType_t now_ticks = xTaskGetTickCount();
    const uint32_t now_ms = (uint32_t)pdTICKS_TO_MS(now_ticks);
    const bool flash_on = ((now_ms / 500u) % 2u) == 0u;

    runtime_health_snapshot_t health;
    RuntimeHealthRead(&state->health_cache, &health);
    const uint32_t policy = health.disp_attn_pol;
    const display_attention_mask_t active_all =
      ComputeActiveAttentionMaskFromHealth(&health);
    display_attention_bit_t error_bits[8];
    display_attention_bit_t warn_bits[8];
    size_t error_count = 0;
    size_t warn_count = 0;
    display_attention_mask_t error_mask = 0;
    display_attention_mask_t warn_mask = 0;
    const display_attention_item_t items[] = {
      kDispAttnItemSdOut,    kDispAttnItemSdIo,    kDispAttnItemFramOvr,
      kDispAttnItemRtdFault, kDispAttnItemTimeBad, kDispAttnItemMeshDown,
    };
    for (size_t idx = 0; idx < sizeof(items) / sizeof(items[0]); ++idx) {
      const display_attention_item_t item = items[idx];
      const display_attention_bit_t bit =
        (display_attention_bit_t)(1u << (uint32_t)item);
      if ((active_all & bit) == 0u) {
        continue;
      }
      const display_attention_severity_t severity =
        DisplayAttentionPolicyGet(policy, item);
      if (severity == DISP_SEV_ERROR) {
        error_bits[error_count++] = bit;
        error_mask |= bit;
      } else if (severity == DISP_SEV_WARN) {
        warn_bits[warn_count++] = bit;
        warn_mask |= bit;
      }
    }

    if (error_count > 0) {
      if (error_mask != last_active_mask) {
        code_index = 0;
        last_code_tick = now_ms / 1000u;
        last_active_mask = error_mask;
      }

      const uint32_t now_code_tick = now_ms / 1000u;
      if (now_code_tick != last_code_tick) {
        code_index = (code_index + 1u) % error_count;
        last_code_tick = now_code_tick;
      }

      if (flash_on) {
        const char* text = AttentionBitToCode(error_bits[code_index]);
        if (strncmp(last_text, text, sizeof(last_text)) != 0) {
          Max7219DisplaySetText(&state->display, text);
          snprintf(last_text, sizeof(last_text), "%s", text);
        }
      } else if (last_text[0] != '\0') {
        Max7219DisplayClear(&state->display);
        last_text[0] = '\0';
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    last_active_mask = 0;
    if (warn_count == 0) {
      last_warn_mask = 0;
    }

    const bool warn_overlay_active =
      warn_count > 0 &&
      ((now_ms % warn_overlay_period_ms) < warn_overlay_duration_ms);
    if (warn_overlay_active) {
      if (warn_mask != last_warn_mask) {
        warn_code_index = 0;
        last_warn_tick = now_ms / 1000u;
        last_warn_mask = warn_mask;
      }

      const uint32_t now_warn_tick = now_ms / 1000u;
      if (now_warn_tick != last_warn_tick) {
        warn_code_index = (warn_code_index + 1u) % warn_count;
        last_warn_tick = now_warn_tick;
      }

      const char* text = AttentionBitToCode(warn_bits[warn_code_index]);
      if (strncmp(last_text, text, sizeof(last_text)) != 0) {
        Max7219DisplaySetText(&state->display, text);
        snprintf(last_text, sizeof(last_text), "%s", text);
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    const bool runtime_running = state->cached_status.runtime_running;
    const bool stop_requested = state->cached_status.stop_requested;
    if (!runtime_running) {
      const bool stop_save_active =
        stop_requested && (state->cached_status.fram_count > 0u ||
                           state->cached_status.sd_mounted);

      // Operator feedback:
      // - "SAVE" while draining/unmounting after stop
      // - "IDLE" once fully stopped and SD is unmounted
      const char* text = stop_save_active ? "SAVE " : "IDLE ";
      if (strncmp(last_text, text, sizeof(last_text)) != 0) {
        Max7219DisplaySetText(&state->display, text);
        snprintf(last_text, sizeof(last_text), "%s", text);
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    int32_t temp_milli_c = 0;
    bool temp_valid = false;
    uint32_t flags = 0;
    CopyLastSample(state, &temp_milli_c, &temp_valid, &flags, NULL);
    (void)flags;

    char text[12];
    FormatTemperatureText(text,
                          sizeof(text),
                          temp_milli_c,
                          state->settings.display_units,
                          temp_valid);
    if (strncmp(last_text, text, sizeof(last_text)) != 0) {
      Max7219DisplaySetText(&state->display, text);
      snprintf(last_text, sizeof(last_text), "%s", text);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }

  if (state != NULL) {
    state->display_task = NULL;
  }
  vTaskDelete(NULL);
}

/**
 * @brief Execute HealthPublisherTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the HealthPublisherTask task.
 */
static void
HealthPublisherTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  const TickType_t tick_delay = pdMS_TO_TICKS(50);

  // uint32_t last_watermark_log_ms = 0;

  while (!state->stop_requested) {
    RuntimeHealthPublisherTick(state);

    // Debug: periodically log stack watermark to catch regressions.
    // const uint32_t now_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    // if ((now_ms - last_watermark_log_ms) >= 30000u) {
    //   const UBaseType_t watermark_words = uxTaskGetStackHighWaterMark(NULL);
    //   const uint32_t watermark_bytes =
    //     (uint32_t)watermark_words * (uint32_t)sizeof(StackType_t);
    //   ESP_LOGI("runtime",
    //            "health_pub stack watermark: %u words (%u bytes) free",
    //            (unsigned)watermark_words,
    //            (unsigned)watermark_bytes);
    //   last_watermark_log_ms = now_ms;
    // }
    vTaskDelay(tick_delay);
  }

  state->health_publisher_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute SetRunLogPolicy.
 */
static void
SetRunLogPolicy(void)
{
  // Operator console should not be drowned by Mesh-Lite scan chatter.
  // Do NOT globally mute logs; only reduce known-noisy tags.
  //
  // Keep your own subsystem logs (runtime/sd_logger/etc.) unchanged so
  // mount/flush INFO remains visible.
  esp_log_level_set("wifi", ESP_LOG_ERROR);     // hides repetitive WARN spam
  esp_log_level_set("vendor_ie", ESP_LOG_WARN); // hides scan start/stop INFO
  esp_log_level_set("Mesh-Lite", ESP_LOG_WARN); // hides "connecting" INFO

  // Suppress periodic printf()-style noise (e.g. topology line). Your operator
  // console still gets command responses and ESP_LOG* output.
  g_state.log_quiet = true;
}

/**
 * @brief Execute SetDiagLogPolicy.
 */
static void
SetDiagLogPolicy(void)
{
  esp_log_level_set("*", ESP_LOG_INFO);
  // Restore noisy tags for diagnosis sessions.
  esp_log_level_set("wifi", ESP_LOG_INFO);
  esp_log_level_set("vendor_ie", ESP_LOG_INFO);
  esp_log_level_set("Mesh-Lite", ESP_LOG_INFO);
  g_state.log_quiet = false;
}

/**
 * @brief Execute FramI2cReadAdapter.
 * @param context Parameter context.
 * @param addr Parameter addr.
 * @param out Parameter out.
 * @param len Parameter len.
 * @return Return the function result.
 */
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

/**
 * @brief Execute FramI2cWriteAdapter.
 * @param context Parameter context.
 * @param addr Parameter addr.
 * @param data Parameter data.
 * @param len Parameter len.
 * @return Return the function result.
 */
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

/**
 * @brief Execute FormatMacString.
 * @param mac Parameter mac.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
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

/**
 * @brief Execute BuildDateStringFromRecord.
 * @param record Parameter record.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
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
  strftime(out, out_size, "%Y-%m-%dZ", &time_info);
}

/**
 * @brief Execute CsvDataPortWriter.
 * @param bytes Parameter bytes.
 * @param len Parameter len.
 * @param context Parameter context.
 * @return Return the function result.
 */
static bool
CsvDataPortWriter(const char* bytes, size_t len, void* context)
{
  (void)context;
  size_t written = 0;
  return DataPortWrite(bytes, len, &written) == ESP_OK && written == len;
}

/**
 * @brief Execute BuildMqttTopic.
 * @param prefix Parameter prefix.
 * @param node_id Parameter node_id.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 * @return Return the function result.
 */
static bool
BuildMqttTopic(const char* prefix,
               const char* node_id,
               char* out,
               size_t out_size)
{
  if (node_id == NULL || out == NULL || out_size == 0) {
    return false;
  }
  const char* prefix_value = prefix;
  if (prefix_value == NULL || prefix_value[0] == '\0') {
    prefix_value = APP_SETTINGS_MQTT_TOPIC_PREFIX_DEFAULT;
  }
  const int written =
    snprintf(out, out_size, "%s/%s/record", prefix_value, node_id);
  return written > 0 && (size_t)written < out_size;
}

/**
 * @brief Execute BuildMqttPayload.
 * @param record Parameter record.
 * @param node_id Parameter node_id.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 * @param written_out Parameter written_out.
 * @return Return the function result.
 */
static bool
BuildMqttPayload(const log_record_t* record,
                 const char* node_id,
                 char* out,
                 size_t out_size,
                 size_t* written_out)
{
  return CsvFormatRow(record, node_id, out, out_size, written_out);
}

/**
 * @brief Execute TryEmitCsvHeader.
 * @param state Parameter state.
 * @return Return the function result.
 */
static bool
TryEmitCsvHeader(runtime_state_t* state)
{
  if (state == NULL) {
    return false;
  }
  if (state->csv_header_emitted) {
    return true;
  }
  state->csv_header_emitted = true;
  if (!CsvWriteHeader(CsvDataPortWriter, NULL)) {
    state->csv_header_emitted = false;
    state->export_write_fail_count++;
    UpdateCachedUint32(state,
                       &state->cached_status.export_write_fail_count,
                       state->export_write_fail_count);
    return false;
  }
  return true;
}

/**
 * @brief Execute TryEmitBridgeCsvHeader.
 * @param state Parameter state.
 * @return Return the function result.
 */
static bool
TryEmitBridgeCsvHeader(runtime_state_t* state)
{
  if (state == NULL) {
    return false;
  }
  if (state->root_bridge_header_emitted) {
    return true;
  }
  state->root_bridge_header_emitted = true;
  if (!CsvWriteHeader(CsvDataPortWriter, NULL)) {
    state->root_bridge_header_emitted = false;
    state->export_write_fail_count++;
    UpdateCachedUint32(state,
                       &state->cached_status.export_write_fail_count,
                       state->export_write_fail_count);
    return false;
  }
  return true;
}

/**
 * @brief Execute SnapshotActiveSettings.
 * @param state Parameter state.
 */
static void
SnapshotActiveSettings(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  state->node_role_active = state->settings.node_role;
  state->net_mode_active = (state->settings.node_role == APP_NODE_ROLE_ROOT)
                             ? APP_NET_MODE_MESH
                             : state->settings.net_mode;
  state->mqtt_enabled_active = state->settings.mqtt_enabled;
  strlcpy(state->mqtt_broker_uri_active,
          state->settings.mqtt_broker_uri,
          sizeof(state->mqtt_broker_uri_active));
  strlcpy(state->mqtt_topic_prefix_active,
          state->settings.mqtt_topic_prefix,
          sizeof(state->mqtt_topic_prefix_active));
  state->mqtt_qos_active = state->settings.mqtt_qos;
  state->mqtt_retain_active = state->settings.mqtt_retain;
  state->mqtt_bridge_mode_active = state->settings.mqtt_bridge_mode;
}

/**
 * @brief Execute EnqueueExportRecord.
 * @param state Parameter state.
 * @param node_id Parameter node_id.
 * @param record Parameter record.
 */
static void
EnqueueExportRecord(runtime_state_t* state,
                    const char* node_id,
                    const log_record_t* record)
{
  if (state == NULL || record == NULL || state->export_queue == NULL) {
    return;
  }

  export_item_t item;
  memset(&item, 0, sizeof(item));
  item.record = *record;
  if (node_id != NULL) {
    snprintf(item.node_id, sizeof(item.node_id), "%s", node_id);
  }

  if (xQueueSend(state->export_queue, &item, 0) != pdTRUE) {
    state->export_dropped_count++;
    UpdateCachedUint32(state,
                       &state->cached_status.export_dropped_count,
                       state->export_dropped_count);
  }
}

/**
 * @brief Execute EnqueueExportOutbox.
 * @param state Parameter state.
 * @param src_mac Parameter src_mac.
 * @param record Parameter record.
 */
static void
EnqueueExportOutbox(runtime_state_t* state,
                    const uint8_t src_mac[6],
                    const log_record_t* record)
{
  if (state == NULL || record == NULL || src_mac == NULL ||
      state->export_outbox == NULL) {
    return;
  }

  export_record_item_t item;
  memset(&item, 0, sizeof(item));
  memcpy(item.src_mac, src_mac, sizeof(item.src_mac));
  item.record = *record;

  if (xQueueSend(state->export_outbox, &item, 0) != pdTRUE) {
    state->export_drop_count++;
    UpdateCachedUint32(
      state, &state->cached_status.export_drop_count, state->export_drop_count);
    if (LogRateLimitAllow(&state->last_export_drop_log_ms,
                          kExportLogRateLimitMs)) {
      ESP_LOGW(kTag, "export outbox full; dropping network export");
    }
  }
}

/**
 * @brief Execute EnqueueBrokerPublish.
 * @param state Parameter state.
 * @param src_mac Parameter src_mac.
 * @param record Parameter record.
 */
static void
EnqueueBrokerPublish(runtime_state_t* state,
                     const uint8_t src_mac[6],
                     const log_record_t* record)
{
  if (state == NULL || src_mac == NULL || record == NULL ||
      state->broker_outbox == NULL) {
    return;
  }

  char node_id[32] = { 0 };
  FormatMacString(src_mac, node_id, sizeof(node_id));

  broker_publish_item_t publish_item;
  memset(&publish_item, 0, sizeof(publish_item));
  if (!BuildMqttTopic(state->mqtt_topic_prefix_active,
                      node_id,
                      publish_item.topic,
                      sizeof(publish_item.topic))) {
    state->broker_send_fail_count++;
    UpdateCachedUint32(state,
                       &state->cached_status.broker_send_fail_count,
                       state->broker_send_fail_count);
    return;
  }

  size_t payload_len = 0;
  if (!BuildMqttPayload(record,
                        node_id,
                        publish_item.payload,
                        sizeof(publish_item.payload),
                        &payload_len)) {
    state->broker_send_fail_count++;
    UpdateCachedUint32(state,
                       &state->cached_status.broker_send_fail_count,
                       state->broker_send_fail_count);
    return;
  }

  publish_item.payload_len = (uint16_t)payload_len;

  if (xQueueSend(state->broker_outbox, &publish_item, 0) != pdTRUE) {
    state->broker_drop_count++;
    UpdateCachedUint32(
      state, &state->cached_status.broker_drop_count, state->broker_drop_count);
    if (LogRateLimitAllow(&state->last_broker_drop_log_ms,
                          kExportLogRateLimitMs)) {
      ESP_LOGW(kTag, "broker outbox full; dropping publish");
    }
  }
}

/**
 * @brief Execute RootRecordRxCallback.
 * @param from Parameter from.
 * @param record Parameter record.
 * @param context Parameter context.
 */
static void
RootRecordRxCallback(const pt100_mesh_addr_t* from,
                     const log_record_t* record,
                     void* context)
{
  (void)context;
  char node_id[32];
  FormatMacString(from->addr, node_id, sizeof(node_id));
  EnqueueExportRecord(&g_state, node_id, record);
  if (g_state.settings.node_role == APP_NODE_ROLE_ROOT) {
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t now_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    uint64_t leaf_id = 0;
    for (int i = 0; i < 6; ++i) {
      leaf_id = (leaf_id << 8) | from->addr[i];
    }
    AlertManagerOnSample(
      &g_state.alert_manager, leaf_id, record, now_ms, now_epoch);
  }
}

/**
 * @brief Execute RootPublishRecordRxCallback.
 * @param src_mac Parameter src_mac.
 * @param record Parameter record.
 * @param context Parameter context.
 */
static void
RootPublishRecordRxCallback(const uint8_t src_mac[6],
                            const log_record_t* record,
                            void* context)
{
  (void)context;
  if (!g_state.root_publish_consumer_active) {
    g_state.root_publish_drop_no_consumer++;
    UpdateCachedUint32(&g_state,
                       &g_state.cached_status.root_publish_drop_no_consumer,
                       g_state.root_publish_drop_no_consumer);
    return;
  }
  EnqueueExportOutbox(&g_state, src_mac, record);
}

/**
 * @brief Execute UpdateMqttConnectionState.
 * @param state Parameter state.
 */
static void
UpdateMqttConnectionState(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  const bool connected = MqttClientWrapIsConnected(&state->mqtt_client);
  state->mqtt_client_connected = connected;
  UpdateCachedBool(state, &state->cached_status.mqtt_connected, connected);
}

/**
 * @brief Execute EnsureMqttClientState.
 * @param state Parameter state.
 * @param should_run Parameter should_run.
 */
static void
EnsureMqttClientState(runtime_state_t* state, bool should_run)
{
  if (state == NULL) {
    return;
  }
  if (!should_run) {
    MqttClientWrapStop(&state->mqtt_client);
    UpdateMqttConnectionState(state);
    return;
  }

  const bool wifi_connected = WifiManagerIsConnected();
  if (!wifi_connected) {
    MqttClientWrapStop(&state->mqtt_client);
    UpdateMqttConnectionState(state);
    return;
  }

  const esp_err_t start_result =
    MqttClientWrapStart(&state->mqtt_client, state->mqtt_broker_uri_active);
  if (start_result != ESP_OK) {
    if (LogRateLimitAllow(&state->last_mqtt_fail_log_ms,
                          kExportLogRateLimitMs)) {
      ESP_LOGW(kTag, "MQTT start failed: %s", esp_err_to_name(start_result));
    }
  }
  UpdateMqttConnectionState(state);
}

/**
 * @brief Execute EnsureSdSyncedForEpoch.
 * @param state Parameter state.
 * @param epoch_for_file Parameter epoch_for_file.
 * @return Return the function result.
 */
static esp_err_t
EnsureSdSyncedForEpoch(runtime_state_t* state, int64_t epoch_for_file)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (FramLogGetBufferedRecords(&state->fram_log) == 0) {
    return ESP_OK;
  }

  log_record_t oldest_record;
  esp_err_t peek_result = FramLogPeekOldest(&state->fram_log, &oldest_record);
  if (peek_result == ESP_ERR_INVALID_RESPONSE) {
    ESP_LOGE(kTag, "Cannot sync: corrupted FRAM record at head");
    return peek_result;
  }
  if (peek_result != ESP_OK) {
    return peek_result;
  }

  int64_t target_epoch = epoch_for_file;
  if (oldest_record.timestamp_epoch_sec > 0) {
    target_epoch = oldest_record.timestamp_epoch_sec;
  }
  esp_err_t sd_result =
    SdLoggerEnsureDailyFile(&state->sd_logger, target_epoch);
  if (sd_result != ESP_OK) {
    return sd_result;
  }

  uint32_t consumed = 0;
  esp_err_t consume_result = FramLogConsumeUpToRecordId(
    &state->fram_log, SdLoggerLastRecordIdOnSd(&state->sd_logger), &consumed);
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

/**
 * @brief Execute BuildBatchForDay.
 * @param state Parameter state.
 * @param target_date Parameter target_date.
 * @param buffer Parameter buffer.
 * @param buffer_size Parameter buffer_size.
 * @param max_records Parameter max_records.
 * @param start_ticks Parameter start_ticks.
 * @param max_ms Parameter max_ms.
 * @param records_used_out Parameter records_used_out.
 * @param last_record_id_out Parameter last_record_id_out.
 * @param bytes_used_out Parameter bytes_used_out.
 * @return Return the function result.
 */
static esp_err_t
BuildBatchForDay(runtime_state_t* state,
                 const char* target_date,
                 uint8_t* buffer,
                 size_t buffer_size,
                 uint32_t max_records,
                 TickType_t start_ticks,
                 uint32_t max_ms,
                 uint32_t* records_used_out,
                 uint64_t* last_record_id_out,
                 size_t* bytes_used_out)
{
  size_t used = 0;
  uint32_t records_used = 0;
  uint64_t last_record_id = 0;

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

    char line[256];
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
    last_record_id = record.record_id;

    if (used >= buffer_size - sizeof(line)) {
      break;
    }
  }

  *records_used_out = records_used;
  *last_record_id_out = last_record_id;
  *bytes_used_out = used;
  return ESP_OK;
}

/**
 * @brief Execute FlushFramToSd.
 * @param state Parameter state.
 * @param flush_all Parameter flush_all.
 * @return Return the function result.
 */
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

  const bool allow_full_flush = flush_all && state->stop_requested;
  TickType_t flush_start = xTaskGetTickCount();
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

    if (!RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    esp_err_t sync_result = EnsureSdSyncedForEpoch(state, epoch_for_file);
    if (sync_result != ESP_OK) {
      RuntimeDiagHeapCheck(state, "SD unmount (sync fail before)", false);
      (void)SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (sync fail after)", false);
      RuntimeSdIoUnlock(state);
      MarkSdFailure(state, "SD sync failed", "sync", sync_result, 0, true);
      return sync_result;
    }

    uint32_t records_used = 0;
    uint64_t last_record_id = 0;
    size_t bytes_used = 0;
    esp_err_t batch_result = BuildBatchForDay(state,
                                              day_string,
                                              state->batch_buffer,
                                              state->batch_buffer_size,
                                              0,
                                              xTaskGetTickCount(),
                                              0,
                                              &records_used,
                                              &last_record_id,
                                              &bytes_used);
    if (batch_result != ESP_OK) {
      RuntimeSdIoUnlock(state);
      return batch_result;
    }
    if (records_used == 0 || bytes_used == 0) {
      RuntimeSdIoUnlock(state);
      return ESP_ERR_INVALID_STATE;
    }

    sd_csv_append_stats_t append_stats = { 0 };
    sd_csv_append_scratch_t append_scratch = { 0 };
    const sd_csv_append_scratch_t* scratch =
      BuildSdAppendScratch(state, &append_scratch);
    esp_err_t write_result = ESP_OK;
    if (state->sd_force_unmount_on_append) {
      state->sd_force_unmount_on_append = false;
      RuntimeDiagHeapCheck(state, "SD unmount (inject before)", false);
      (void)SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (inject after)", false);
      append_stats.diag.operation = "inject-unmount";
      append_stats.diag.errno_value = 0;
      write_result = ESP_ERR_INVALID_STATE;
    } else {
      write_result =
        SdLoggerAppendBatchEx(&state->sd_logger,
                              state->batch_buffer,
                              bytes_used,
                              last_record_id,
                              ResolveSdVerifyMode(state),
                              SD_APPEND_FLUSH_ALWAYS,
                              scratch,
                              &append_stats);
    }
    if (write_result != ESP_OK) {
      ESP_LOGE(kTag,
               "SD append failed after %u records: %s",
               total_flushed + records_used,
               esp_err_to_name(write_result));
      RuntimeDiagHeapCheck(state, "SD unmount (append fail before)", false);
      (void)SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (append fail after)", false);
      RuntimeSdIoUnlock(state);
      const char* op =
        (append_stats.diag.operation != NULL) ? append_stats.diag.operation
                                              : "append";
      MarkSdFailure(state,
                    "SD append failed",
                    op,
                    write_result,
                    append_stats.diag.errno_value,
                    true);
      return write_result;
    }
    ClearSdIoError(state);
    RuntimeSdIoUnlock(state);

    for (uint32_t index = 0; index < records_used; ++index) {
      esp_err_t discard_result = FramLogDiscardOldest(&state->fram_log);
      if (discard_result != ESP_OK) {
        return discard_result;
      }
      if ((index % 16u) == 0u && (xTaskGetTickCount() - flush_start) >
                                   pdMS_TO_TICKS(kSdFlushTimeSliceMs)) {
        const TickType_t now_ticks = xTaskGetTickCount();
        if (state->last_sd_flush_warn_ticks == 0 ||
            (now_ticks - state->last_sd_flush_warn_ticks) >
              pdMS_TO_TICKS(kSdFlushWarnIntervalMs)) {
          ESP_LOGW(kTag, "SD flush time slice exceeded; yielding");
          state->last_sd_flush_warn_ticks = now_ticks;
        }
        vTaskDelay(1);
        flush_start = xTaskGetTickCount();
      }
    }

    total_flushed += records_used;
    ESP_LOGI(kTag,
             "Flushed %u records (%zu bytes) for %s (total=%u)",
             records_used,
             bytes_used,
             day_string,
             total_flushed);

    if (!allow_full_flush) {
      break;
    }

    if ((xTaskGetTickCount() - flush_start) >
        pdMS_TO_TICKS(kSdFlushTimeSliceMs)) {
      const TickType_t now_ticks = xTaskGetTickCount();
      if (state->last_sd_flush_warn_ticks == 0 ||
          (now_ticks - state->last_sd_flush_warn_ticks) >
            pdMS_TO_TICKS(kSdFlushWarnIntervalMs)) {
        ESP_LOGW(kTag, "SD flush time slice exceeded; yielding");
        state->last_sd_flush_warn_ticks = now_ticks;
      }
      vTaskDelay(1);
      flush_start = xTaskGetTickCount();
    }
  }

  return (total_flushed > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
 * @brief Execute MarkSdFailure.
 * @param state Parameter state.
 * @param context Parameter context.
 * @param operation Parameter operation.
 * @param error Parameter error.
 * @param errno_value Parameter errno_value.
 * @param did_unmount Parameter did_unmount.
 */
static void
MarkSdFailure(runtime_state_t* state,
              const char* context,
              const char* operation,
              esp_err_t error,
              int errno_value,
              bool did_unmount)
{
  if (state == NULL) {
    return;
  }
  const TickType_t now_ticks = xTaskGetTickCount();
  state->sd_degraded = true;
  state->sd_fail_count++;
  state->sd_backoff_until_ticks =
    now_ticks + pdMS_TO_TICKS(kSdFlushFailureBackoffMs);
  if (IsSdIoOperation(operation)) {
    state->sd_last_io_error_active = true;
    state->sd_last_io_err = error;
    state->sd_last_errno = errno_value;
    UpdateCachedBool(state, &state->cached_status.sd_io_error_active, true);
  }
  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  UpdateCachedBool(state, &state->cached_status.sd_degraded, true);
  UpdateCachedUint32(
    state, &state->cached_status.sd_fail_count, state->sd_fail_count);
  UpdateCachedUint32(state,
                     &state->cached_status.sd_backoff_remaining_ms,
                     ComputeSdBackoffRemainingMs(state, now_ticks));
  const char* op_label = (operation != NULL) ? operation : "unknown";
  const char* errno_str = (errno_value != 0) ? strerror(errno_value) : "n/a";
  const char* action_label = did_unmount ? "unmount+backoff" : "backoff";
  ESP_LOGW(kTag,
           "%s: op=%s err=%s (%d) errno=%d (%s) action=%s backoff_until=%u",
           context,
           op_label,
           esp_err_to_name(error),
           (int)error,
           errno_value,
           errno_str,
           action_label,
           (unsigned)state->sd_backoff_until_ticks);
}

/**
 * @brief Execute ResolveSdVerifyMode.
 * @param state Parameter state.
 * @return Return the function result.
 */
static sd_append_verify_t
ResolveSdVerifyMode(const runtime_state_t* state)
{
  if (state == NULL) {
    return SD_APPEND_VERIFY_NONE;
  }
  return state->sd_verify_readback ? SD_APPEND_VERIFY_READBACK_SHA256
                                   : SD_APPEND_VERIFY_NONE;
}

/**
 * @brief Execute SdFlushWorkerTick.
 * @param state Parameter state.
 * @param max_records Parameter max_records.
 * @param max_ms Parameter max_ms.
 * @param records_flushed_out Parameter records_flushed_out.
 * @param bytes_flushed_out Parameter bytes_flushed_out.
 * @param more_pending_out Parameter more_pending_out.
 * @return Return the function result.
 */
static esp_err_t
SdFlushWorkerTickEx(runtime_state_t* state,
                    uint32_t max_records,
                    uint32_t max_ms,
                    sd_append_verify_t verify_mode,
                    sd_append_flush_t flush_mode,
                    uint32_t* records_flushed_out,
                    size_t* bytes_flushed_out,
                    bool* more_pending_out)
{
  if (records_flushed_out != NULL) {
    *records_flushed_out = 0;
  }
  if (bytes_flushed_out != NULL) {
    *bytes_flushed_out = 0;
  }
  if (more_pending_out != NULL) {
    *more_pending_out = false;
  }
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (state->batch_buffer == NULL || state->batch_buffer_size == 0) {
    return ESP_ERR_NO_MEM;
  }

  const uint32_t buffered_records = FramLogGetBufferedRecords(&state->fram_log);
  if (buffered_records == 0) {
    return ESP_OK;
  }

  const TickType_t now_ticks = xTaskGetTickCount();
  if (state->sd_backoff_until_ticks != 0 &&
      now_ticks < state->sd_backoff_until_ticks) {
    if (more_pending_out != NULL) {
      *more_pending_out = true;
    }
    return ESP_OK;
  }

  if (!state->sd_logger.is_mounted) {
    if (more_pending_out != NULL) {
      *more_pending_out = true;
    }
    return ESP_OK;
  }

  log_record_t first_record;
  esp_err_t peek_result = FramLogPeekOldest(&state->fram_log, &first_record);
  if (peek_result == ESP_ERR_INVALID_RESPONSE) {
    ESP_LOGE(kTag, "Corrupted FRAM record detected during SD flush");
    (void)FramLogSkipCorruptedRecord(&state->fram_log);
    if (more_pending_out != NULL) {
      *more_pending_out = (FramLogGetBufferedRecords(&state->fram_log) > 0);
    }
    return ESP_OK;
  }
  if (peek_result != ESP_OK) {
    return peek_result;
  }

  const int64_t epoch_for_file = (first_record.timestamp_epoch_sec > 0)
                                   ? first_record.timestamp_epoch_sec
                                   : (int64_t)time(NULL);
  if (state->sd_flush_in_progress) {
    const TickType_t now_ticks = xTaskGetTickCount();
    if (state->last_sd_flush_wait_warn_ticks == 0 ||
        (now_ticks - state->last_sd_flush_wait_warn_ticks) >
          pdMS_TO_TICKS(kSdFlushWarnIntervalMs)) {
      ESP_LOGW(kTag, "SD flush already in progress; waiting for SD lock");
      state->last_sd_flush_wait_warn_ticks = now_ticks;
    }
  }
  if (!RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
    if (more_pending_out != NULL) {
      *more_pending_out = true;
    }
    return ESP_ERR_TIMEOUT;
  }
  state->sd_flush_in_progress = true;
  esp_err_t result = ESP_OK;
  esp_err_t sync_result = EnsureSdSyncedForEpoch(state, epoch_for_file);
  if (sync_result != ESP_OK) {
    RuntimeDiagHeapCheck(state, "SD unmount (flush sync before)", false);
    (void)SdLoggerUnmount(&state->sd_logger);
    RuntimeDiagHeapCheck(state, "SD unmount (flush sync after)", false);
    MarkSdFailure(state, "SD sync failed", "sync", sync_result, 0, true);
    result = sync_result;
    goto flush_done;
  }

  char day_string[16];
  BuildDateStringFromRecord(&first_record, day_string, sizeof(day_string));

  uint32_t records_used = 0;
  uint64_t last_record_id = 0;
  size_t bytes_used = 0;
  esp_err_t batch_result = BuildBatchForDay(state,
                                            day_string,
                                            state->batch_buffer,
                                            state->batch_buffer_size,
                                            max_records,
                                            now_ticks,
                                            max_ms,
                                            &records_used,
                                            &last_record_id,
                                            &bytes_used);
  if (batch_result != ESP_OK) {
    result = batch_result;
    goto flush_done;
  }
  if (records_used == 0 || bytes_used == 0) {
    // We still have buffered records, but couldn't build a batch this pass
    // (e.g., time jumped while formatting dates or a corrupted record was
    // skipped). Keep the flush pending so we retry soon instead of waiting for
    // watermark/periodic.
    if (more_pending_out != NULL) {
      *more_pending_out = true;
    }
    result = ESP_OK;
    goto flush_done;
  }

  sd_csv_append_stats_t append_stats = { 0 };
  sd_csv_append_scratch_t append_scratch = { 0 };
  const sd_csv_append_scratch_t* scratch =
    BuildSdAppendScratch(state, &append_scratch);
  esp_err_t write_result =
    SdLoggerAppendBatchEx(&state->sd_logger,
                          state->batch_buffer,
                          bytes_used,
                          last_record_id,
                          verify_mode,
                          flush_mode,
                          scratch,
                          &append_stats);
  if (write_result != ESP_OK) {
    RuntimeDiagHeapCheck(state, "SD unmount (flush append before)", false);
    (void)SdLoggerUnmount(&state->sd_logger);
    RuntimeDiagHeapCheck(state, "SD unmount (flush append after)", false);
    const char* op =
      (append_stats.diag.operation != NULL) ? append_stats.diag.operation
                                            : "append";
    MarkSdFailure(state,
                  "SD append failed",
                  op,
                  write_result,
                  append_stats.diag.errno_value,
                  true);
    result = write_result;
    goto flush_done;
  }
  ClearSdIoError(state);
  result = ESP_OK;

flush_done:
  state->sd_flush_in_progress = false;
  RuntimeSdIoUnlock(state);
  if (result != ESP_OK) {
    return result;
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
  if (bytes_flushed_out != NULL) {
    *bytes_flushed_out = bytes_used;
  }
  if (more_pending_out != NULL) {
    *more_pending_out = (FramLogGetBufferedRecords(&state->fram_log) > 0);
  }
  return ESP_OK;
}

/**
 * @brief Execute SdFlushWorkerTick.
 * @param state Parameter state.
 * @param max_records Parameter max_records.
 * @param max_ms Parameter max_ms.
 * @param records_flushed_out Parameter records_flushed_out.
 * @param bytes_flushed_out Parameter bytes_flushed_out.
 * @param more_pending_out Parameter more_pending_out.
 * @return Return the function result.
 */
static esp_err_t
SdFlushWorkerTick(runtime_state_t* state,
                  uint32_t max_records,
                  uint32_t max_ms,
                  uint32_t* records_flushed_out,
                  size_t* bytes_flushed_out,
                  bool* more_pending_out)
{
  return SdFlushWorkerTickEx(state,
                             max_records,
                             max_ms,
                             ResolveSdVerifyMode(state),
                             SD_APPEND_FLUSH_ALWAYS,
                             records_flushed_out,
                             bytes_flushed_out,
                             more_pending_out);
}

/**
 * @brief Execute SensorTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the SensorTask task.
 */
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
    int64_t epoch_sec = 0;
    int32_t millis = 0;
    TimeSyncGetNow(&epoch_sec, &millis);
    const bool time_valid = TimeSyncIsSystemTimeValid();
    UpdateTimeHealthState(state, time_valid);
    record.timestamp_epoch_sec = time_valid ? epoch_sec : (int64_t)0;
    record.timestamp_millis = time_valid ? millis : 0;

    if (result == ESP_OK) {
      const double cal_c = CalibrationModelEvaluateWithPoints(
        &state->settings.calibration,
        sample.temperature_c,
        state->settings.calibration_points,
        state->settings.calibration_points_count);
      record.raw_temp_milli_c = (int32_t)llround(sample.temperature_c * 1000.0);
      CalWindowPushRawSample(record.raw_temp_milli_c);
      record.temp_milli_c = (int32_t)llround(cal_c * 1000.0);
      record.resistance_milli_ohm =
        (int32_t)llround(sample.resistance_ohm * 1000.0);
      if (sample.fault_present) {
        record.flags |= LOG_RECORD_FLAG_SENSOR_FAULT;
      }
    } else {
      record.flags |= LOG_RECORD_FLAG_SENSOR_FAULT;
    }

    // Log sensor faults in a rate-limited way so operators see
    // wiring/open/short issues without flooding the console.
    const TickType_t now_ticks = xTaskGetTickCount();
    if (result == ESP_OK && sample.fault_present) {
      const bool changed =
        !state->last_sensor_fault_present ||
        state->last_sensor_fault_status != sample.fault_status;
      const bool rate_ok =
        (state->last_sensor_fault_log_ticks == 0) ||
        (pdTICKS_TO_MS(now_ticks - state->last_sensor_fault_log_ticks) >=
         5000u);
      if (changed || rate_ok) {
        // Avoid float formatting in logs. newlib's %f formatting is
        // stack-heavy.
        const int32_t res_milli_ohm =
          (int32_t)llround(sample.resistance_ohm * 1000.0);
        const int32_t temp_milli_c =
          (int32_t)llround(sample.temperature_c * 1000.0);
        ESP_LOGW(kTag,
                 "MAX31865 fault: status=0x%02X res_mohm=%" PRId32
                 " temp_mC=%" PRId32,
                 sample.fault_status,
                 res_milli_ohm,
                 temp_milli_c);
        state->last_sensor_fault_log_ticks = now_ticks;
      }
      state->last_sensor_fault_present = true;
      state->last_sensor_fault_status = sample.fault_status;
    } else if (result != ESP_OK) {
      const bool changed = !state->last_sensor_fault_present ||
                           state->last_sensor_fault_status != 0xFFu;
      const bool rate_ok =
        (state->last_sensor_fault_log_ticks == 0) ||
        (pdTICKS_TO_MS(now_ticks - state->last_sensor_fault_log_ticks) >=
         5000u);
      if (changed || rate_ok) {
        ESP_LOGW(kTag, "MAX31865 read failed: %s", esp_err_to_name(result));
        state->last_sensor_fault_log_ticks = now_ticks;
      }
      state->last_sensor_fault_present = true;
      state->last_sensor_fault_status = 0xFFu;
    } else {
      if (state->last_sensor_fault_present) {
        ESP_LOGW(kTag, "MAX31865 fault cleared");
      }
      state->last_sensor_fault_present = false;
      state->last_sensor_fault_status = 0;
    }
    UpdateCachedBool(state,
                     &state->cached_status.sensor_fault_present,
                     state->last_sensor_fault_present);

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
    const bool mesh_connected = MeshTransportIsConnected(&state->mesh);
    UpdateCachedBool(
      state, &state->cached_status.mesh_connected, mesh_connected);
    UpdateCachedInt32(
      state, &state->cached_status.mesh_level, state->mesh.last_level);
    if (mesh_connected) {
      record.flags |= LOG_RECORD_FLAG_MESH_CONNECTED;
    }

    int32_t temp_milli_c = record.temp_milli_c;
    if (!state->settings.calibration.is_valid) {
      temp_milli_c = record.raw_temp_milli_c;
    }
    const bool temp_valid = (result == ESP_OK && !sample.fault_present);

    taskENTER_CRITICAL(&state->last_temp_lock);
    state->last_temp_milli_c = temp_milli_c;
    state->last_temp_valid = temp_valid;
    state->last_flags = record.flags;
    state->last_update_ticks = xTaskGetTickCount();
    taskEXIT_CRITICAL(&state->last_temp_lock);

    (void)xQueueSend(state->log_queue, &record, 0);
    RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(period_ms));
  }

  state->sensor_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute ExportTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the ExportTask task.
 */
static void
ExportTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;

  while (!state->stop_requested ||
         (state->export_queue != NULL &&
          uxQueueMessagesWaiting(state->export_queue) > 0)) {
    export_item_t item;
    if (state->export_queue != NULL &&
        xQueueReceive(state->export_queue, &item, pdMS_TO_TICKS(500)) ==
          pdTRUE) {
      if (!state->data_streaming_enabled) {
        continue;
      }
      if (!TryEmitCsvHeader(state)) {
        RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(50));
        continue;
      }
      if (!CsvWriteRow(CsvDataPortWriter, NULL, &item.record, item.node_id)) {
        state->export_write_fail_count++;
        UpdateCachedUint32(state,
                           &state->cached_status.export_write_fail_count,
                           state->export_write_fail_count);
        RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(50));
      }
    }
  }

  state->export_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute ExportNetworkTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the ExportNetworkTask task.
 */
static void
ExportNetworkTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;

  while (!state->stop_requested ||
         (state->export_outbox != NULL &&
          uxQueueMessagesWaiting(state->export_outbox) > 0)) {
    if (state->export_outbox == NULL) {
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(200));
      continue;
    }
    export_record_item_t item;
    if (xQueuePeek(state->export_outbox, &item, pdMS_TO_TICKS(500)) != pdTRUE) {
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(100));
      continue;
    }

    if (state->net_mode_active == APP_NET_MODE_MESH) {
      if (!MeshTransportIsConnected(&state->mesh)) {
        RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(500));
        continue;
      }
      const esp_err_t send_result = MeshTransportSendPublishRecord(
        &state->mesh, item.src_mac, &item.record);
      if (send_result == ESP_OK) {
        (void)xQueueReceive(state->export_outbox, &item, 0);
      } else {
        state->export_send_fail_count++;
        UpdateCachedUint32(state,
                           &state->cached_status.export_send_fail_count,
                           state->export_send_fail_count);
        if (LogRateLimitAllow(&state->last_export_fail_log_ms,
                              kExportLogRateLimitMs)) {
          ESP_LOGW(
            kTag, "mesh export send failed: %s", esp_err_to_name(send_result));
        }
        RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(500));
      }
      continue;
    }

    const bool should_mqtt = state->mqtt_enabled_active &&
                             state->net_mode_active == APP_NET_MODE_DIRECT_WIFI;
    EnsureMqttClientState(state, should_mqtt);
    if (!state->mqtt_client_connected) {
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(500));
      continue;
    }

    char node_id[32] = { 0 };
    FormatMacString(item.src_mac, node_id, sizeof(node_id));
    char topic[128] = { 0 };
    if (!BuildMqttTopic(
          state->mqtt_topic_prefix_active, node_id, topic, sizeof(topic))) {
      state->export_send_fail_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.export_send_fail_count,
                         state->export_send_fail_count);
      (void)xQueueReceive(state->export_outbox, &item, 0);
      continue;
    }

    size_t payload_len = 0;
    char payload[256] = { 0 };
    if (!BuildMqttPayload(
          &item.record, node_id, payload, sizeof(payload), &payload_len)) {
      state->export_send_fail_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.export_send_fail_count,
                         state->export_send_fail_count);
      (void)xQueueReceive(state->export_outbox, &item, 0);
      continue;
    }

    const esp_err_t publish_result =
      MqttClientWrapPublish(&state->mqtt_client,
                            topic,
                            payload,
                            (int)payload_len,
                            state->mqtt_qos_active,
                            state->mqtt_retain_active ? 1 : 0);
    if (publish_result == ESP_OK) {
      (void)xQueueReceive(state->export_outbox, &item, 0);
    } else {
      state->export_send_fail_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.export_send_fail_count,
                         state->export_send_fail_count);
      if (LogRateLimitAllow(&state->last_export_fail_log_ms,
                            kExportLogRateLimitMs)) {
        ESP_LOGW(kTag, "MQTT publish failed");
      }
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(500));
    }
  }

  state->export_network_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute RootBridgeTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the RootBridgeTask task.
 */
static void
RootBridgeTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;

  while (!state->stop_requested ||
         (state->export_outbox != NULL &&
          uxQueueMessagesWaiting(state->export_outbox) > 0)) {
    export_record_item_t item;
    if (state->export_outbox != NULL &&
        xQueueReceive(state->export_outbox, &item, pdMS_TO_TICKS(500)) ==
          pdTRUE) {
      char node_id[32] = { 0 };
      FormatMacString(item.src_mac, node_id, sizeof(node_id));

      char payload[256] = { 0 };
      size_t payload_len = 0;
      if (!BuildMqttPayload(
            &item.record, node_id, payload, sizeof(payload), &payload_len)) {
        state->export_send_fail_count++;
        UpdateCachedUint32(state,
                           &state->cached_status.export_send_fail_count,
                           state->export_send_fail_count);
        continue;
      }

      if (BridgeModeUsesSerial(state->mqtt_bridge_mode_active)) {
        if (!TryEmitBridgeCsvHeader(state)) {
          vTaskDelay(pdMS_TO_TICKS(50));
          continue;
        }
        size_t written = 0;
        const esp_err_t write_result =
          DataPortWrite(payload, payload_len, &written);
        if (write_result != ESP_OK || written != payload_len) {
          state->export_send_fail_count++;
          UpdateCachedUint32(state,
                             &state->cached_status.export_send_fail_count,
                             state->export_send_fail_count);
          if (LogRateLimitAllow(&state->last_export_fail_log_ms,
                                kExportLogRateLimitMs)) {
            ESP_LOGW(kTag, "serial bridge write failed");
          }
        }
      }

      if (BridgeModeUsesBroker(state->mqtt_bridge_mode_active) &&
          state->mqtt_enabled_active) {
        broker_publish_item_t publish_item;
        memset(&publish_item, 0, sizeof(publish_item));
        if (!BuildMqttTopic(state->mqtt_topic_prefix_active,
                            node_id,
                            publish_item.topic,
                            sizeof(publish_item.topic))) {
          state->broker_send_fail_count++;
          UpdateCachedUint32(state,
                             &state->cached_status.broker_send_fail_count,
                             state->broker_send_fail_count);
          continue;
        }
        if (payload_len >= sizeof(publish_item.payload)) {
          state->broker_send_fail_count++;
          UpdateCachedUint32(state,
                             &state->cached_status.broker_send_fail_count,
                             state->broker_send_fail_count);
          continue;
        }
        memcpy(publish_item.payload, payload, payload_len);
        publish_item.payload_len = (uint16_t)payload_len;

        if (state->broker_outbox != NULL &&
            xQueueSend(state->broker_outbox, &publish_item, 0) != pdTRUE) {
          state->broker_drop_count++;
          UpdateCachedUint32(state,
                             &state->cached_status.broker_drop_count,
                             state->broker_drop_count);
          if (LogRateLimitAllow(&state->last_broker_drop_log_ms,
                                kExportLogRateLimitMs)) {
            ESP_LOGW(kTag, "broker outbox full; dropping publish");
          }
        }
      }
    }
  }

  state->bridge_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute BrokerPublishTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the BrokerPublishTask task.
 */
static void
BrokerPublishTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;

  while (!state->stop_requested ||
         (state->broker_outbox != NULL &&
          uxQueueMessagesWaiting(state->broker_outbox) > 0)) {
    if (state->broker_outbox == NULL) {
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(200));
      continue;
    }

    broker_publish_item_t item;
    if (xQueuePeek(state->broker_outbox, &item, pdMS_TO_TICKS(500)) != pdTRUE) {
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(100));
      continue;
    }

    const bool should_mqtt =
      state->mqtt_enabled_active &&
      BridgeModeUsesBroker(state->mqtt_bridge_mode_active);
    EnsureMqttClientState(state, should_mqtt);
    if (!state->mqtt_client_connected) {
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(500));
      continue;
    }

    const esp_err_t publish_result =
      MqttClientWrapPublish(&state->mqtt_client,
                            item.topic,
                            item.payload,
                            (int)item.payload_len,
                            state->mqtt_qos_active,
                            state->mqtt_retain_active ? 1 : 0);
    if (publish_result == ESP_OK) {
      (void)xQueueReceive(state->broker_outbox, &item, 0);
    } else {
      state->broker_send_fail_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.broker_send_fail_count,
                         state->broker_send_fail_count);
      if (LogRateLimitAllow(&state->last_broker_fail_log_ms,
                            kExportLogRateLimitMs)) {
        ESP_LOGW(kTag, "broker publish failed");
      }
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(500));
    }
  }

  state->broker_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute StorageTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the StorageTask task.
 */
static void
StorageTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  state->last_flush_ticks = xTaskGetTickCount();
  state->sd_next_flush_allowed_ticks = state->last_flush_ticks;
  TickType_t last_sd_detect_poll_ticks = 0;

  while (!state->stop_requested ||
         uxQueueMessagesWaiting(state->log_queue) > 0) {
    log_record_t record;
    const bool received =
      (xQueueReceive(state->log_queue, &record, pdMS_TO_TICKS(500)) == pdTRUE);
    if (received) {
      esp_err_t id_result = FramLogAssignRecordIds(&state->fram_log, &record);
      if (id_result != ESP_OK) {
        ESP_LOGE(
          kTag, "Failed to assign record id: %s", esp_err_to_name(id_result));
      }

      if (state->fram_i2c.initialized) {
        esp_err_t append_result = FramLogAppend(&state->fram_log, &record);
        if (append_result != ESP_OK) {
          ESP_LOGE(
            kTag, "FRAM append failed: %s", esp_err_to_name(append_result));
        } else {
          state->sd_flush_records_since++;
        }
        const uint64_t overrun_after =
          FramLogGetOverrunRecordsTotal(&state->fram_log);
        const size_t fram_count = FramLogGetCountRecords(&state->fram_log);
        const size_t fram_capacity =
          FramLogGetCapacityRecords(&state->fram_log);
        const TickType_t now_ticks = xTaskGetTickCount();
        LogFramOverrunWarning(
          state, overrun_after, fram_count, fram_capacity, now_ticks);
        state->last_overrun_records_total = overrun_after;
        UpdateCachedBool(state,
                         &state->cached_status.fram_overrun_active,
                         state->last_overrun_records_total >
                           state->fram_overrun_ack_total);
        UpdateFramFillState(state);
        if (state->fram_full) {
          record.flags |= LOG_RECORD_FLAG_FRAM_FULL;
        }
      }

      // if (!state->mesh.is_root && MeshTransportIsConnected(&state->mesh)) {
      //   (void)MeshTransportSendRecord(&state->mesh, &record);
      // }

      EnqueueExportRecord(state, state->node_id_string, &record);

      if (state->mqtt_enabled_active) {
        if (state->node_role_active == APP_NODE_ROLE_ROOT) {
          if (BridgeModeUsesBroker(state->mqtt_bridge_mode_active)) {
            EnqueueBrokerPublish(state, state->local_mac, &record);
          }
        } else {
          EnqueueExportOutbox(state, state->local_mac, &record);
        }
      }
    }

    const TickType_t now_ticks = xTaskGetTickCount();
    if (state->sd_card_detect.initialized &&
        (last_sd_detect_poll_ticks == 0 ||
         pdTICKS_TO_MS(now_ticks - last_sd_detect_poll_ticks) >=
           kSdDetectPollIntervalMs)) {
      last_sd_detect_poll_ticks = now_ticks;
      bool detect_changed = false;
      const bool present =
        SdCardDetectPoll(&state->sd_card_detect, &detect_changed);
      UpdateCachedBool(state, &state->cached_status.sd_card_present, present);
      if (detect_changed) {
        if (present) {
          ESP_LOGI(kTag, "SD card inserted");
        } else {
          ESP_LOGW(kTag, "SD card removed");
          if (state->sd_logger.is_mounted) {
            if (RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
              SdLoggerClose(&state->sd_logger);
              RuntimeDiagHeapCheck(state, "SD unmount (card removed before)", false);
              (void)SdLoggerUnmount(&state->sd_logger);
              RuntimeDiagHeapCheck(state, "SD unmount (card removed after)", false);
              RuntimeSdIoUnlock(state);
            }
            UpdateCachedBool(state,
                             &state->cached_status.sd_mounted,
                             state->sd_logger.is_mounted);
            ClearSdIoError(state);
            state->sd_was_mounted = false;
            state->sd_backoff_until_ticks = 0;
            UpdateCachedUint32(
              state, &state->cached_status.sd_backoff_remaining_ms, 0u);
          }
        }
      }
    }
    UpdateCachedUint32(state,
                       &state->cached_status.sd_backoff_remaining_ms,
                       ComputeSdBackoffRemainingMs(state, now_ticks));
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
    UpdateCachedBool(
      state, &state->cached_status.sd_degraded, state->sd_degraded);
    UpdateCachedUint32(
      state, &state->cached_status.sd_fail_count, state->sd_fail_count);
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

    if (!state->sd_logger.is_mounted) {
      SdMaintenanceTick(state);
    }

    const UBaseType_t queue_depth =
      (state->log_queue != NULL) ? uxQueueMessagesWaiting(state->log_queue) : 0;
    const bool queue_idle = (queue_depth <= 1u);
    const bool allow_flush_now = (!received) || state->sd_start_drain_pending;
    if (allow_flush_now && queue_idle && state->sd_flush_pending &&
        state->sd_logger.is_mounted &&
        now_ticks >= state->sd_next_flush_allowed_ticks) {
      uint32_t flushed = 0;
      bool more_pending = false;
      esp_err_t flush_result = SdFlushWorkerTick(state,
                                                 kSdFlushMaxRecordsPerPass,
                                                 kSdFlushMaxMsPerPass,
                                                 &flushed,
                                                 NULL,
                                                 &more_pending);
      if (flush_result == ESP_OK) {
        if (flushed > 0) {
          state->sd_flush_records_since = 0;
          UpdateFramFillState(state);
          if (!state->sd_was_mounted && state->sd_logger.is_mounted) {
            ESP_LOGI(kTag,
                     "SD recovered; resuming flush. FRAM overruns since boot: "
                     "%" PRIu64,
                     FramLogGetOverrunRecordsTotal(&state->fram_log));
          }
        }
        state->sd_flush_pending = more_pending;
        if (!more_pending) {
          state->sd_start_drain_pending = false;
        }
      }
      state->sd_next_flush_allowed_ticks =
        xTaskGetTickCount() + pdMS_TO_TICKS(kSdFlushMinIntervalMs);
    }

    if (!state->sd_logger.is_mounted) {
      state->sd_was_mounted = false;
    } else if (!state->sd_was_mounted) {
      state->sd_was_mounted = true;
    }
  }

  if (state->sd_logger.is_mounted) {
    (void)SdFlushWorkerTick(
      state, kSdFlushMaxRecordsPerPass, kSdFlushMaxMsPerPass, NULL, NULL, NULL);
  }

  state->storage_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute TimeSyncTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the TimeSyncTask task.
 */
static void
TimeSyncTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;

  if (state->settings.node_role == APP_NODE_ROLE_SENSOR) {
    while (!TimeSyncIsSystemTimeValid() && !state->stop_requested) {
      const bool time_valid = TimeSyncIsSystemTimeValid();
      UpdateTimeHealthState(state, time_valid);
      const bool mesh_connected = MeshTransportIsConnected(&state->mesh);
      UpdateCachedBool(
        state, &state->cached_status.mesh_connected, mesh_connected);
      UpdateCachedInt32(
        state, &state->cached_status.mesh_level, state->mesh.last_level);
      if (mesh_connected) {
        (void)MeshTransportRequestTime(&state->mesh);
      }
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(10 * 1000));
    }
  }

  if (state->settings.node_role == APP_NODE_ROLE_ROOT) {
    while (!state->stop_requested) {
      const bool time_valid = TimeSyncIsSystemTimeValid();
      UpdateTimeHealthState(state, time_valid);
      const bool mesh_connected = MeshTransportIsConnected(&state->mesh);
      UpdateCachedBool(
        state, &state->cached_status.mesh_connected, mesh_connected);
      UpdateCachedInt32(
        state, &state->cached_status.mesh_level, state->mesh.last_level);
      if (time_valid && mesh_connected) {
        const int64_t now_seconds = (int64_t)time(NULL);
        (void)MeshTransportBroadcastTime(&state->mesh, now_seconds);
      }
      uint64_t period_ms =
        (uint64_t)AppNetConfigGetTimeSyncPeriodSeconds() * 1000ULL;
      if (period_ms < 1000ULL) {
        period_ms = 1000ULL;
      }
      const TickType_t period_ticks = pdMS_TO_TICKS(period_ms);
      RuntimeInterruptibleDelayTicks(period_ticks);
    }
  }

  state->time_sync_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute DirectWifiTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the DirectWifiTask task.
 */
static void
DirectWifiTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  bool last_connected = WifiManagerIsConnected();
  bool last_time_valid = TimeSyncIsSystemTimeValid();
  uint32_t retry_delay_ms = 30 * 1000;
  const uint32_t max_delay_ms = 5 * 60 * 1000;

  // Track minimum stack high-water mark to confirm stack sizing under real
  // workloads. uxTaskGetStackHighWaterMark() returns words, not bytes.
  UBaseType_t min_stack_hwm_words = UINT32_MAX;

  while (!state->stop_requested) {
    const UBaseType_t hwm_words = uxTaskGetStackHighWaterMark(NULL);
    if (hwm_words < min_stack_hwm_words) {
      min_stack_hwm_words = hwm_words;
      ESP_LOGI(kTag,
               "wifi_direct stack watermark: %u words (%u bytes) free",
               (unsigned)min_stack_hwm_words,
               (unsigned)(min_stack_hwm_words * sizeof(StackType_t)));
    }

    wifi_credentials_t creds;
    WifiCredentialsLoad(&creds);

    bool connected = WifiManagerIsConnected();
    if (!connected && creds.has_ssid) {
      const esp_err_t connect_result =
        WifiManagerConnectSta(creds.ssid, creds.password, 10000);
      connected = (connect_result == ESP_OK);
      if (!connected) {
        retry_delay_ms = (retry_delay_ms < max_delay_ms / 2)
                           ? retry_delay_ms * 2
                           : max_delay_ms;
      } else {
        retry_delay_ms = 30 * 1000;
      }
    } else if (!connected) {
      retry_delay_ms = 30 * 1000;
    }

    // Schedule an immediate SNTP sync on each (re)connect, even if the current
    // system time was loaded from the RTC at boot. This keeps the system time
    // accurate and refreshes the RTC periodically.
    static TickType_t s_next_time_sync_ticks = 0;

    const bool connected_changed = (connected != last_connected);
    if (connected_changed) {
      if (connected) {
        ESP_LOGI(kTag, "Wi-Fi connected (direct)");
        s_next_time_sync_ticks = xTaskGetTickCount(); // immediate
      } else {
        ESP_LOGW(kTag, "Wi-Fi disconnected (direct)");
        s_next_time_sync_ticks = 0;
      }
      last_connected = connected;
    }

    const TickType_t now_ticks = xTaskGetTickCount();
    const uint32_t time_sync_period_s = AppNetConfigGetTimeSyncPeriodSeconds();

    bool time_valid = TimeSyncIsSystemTimeValid();
    UpdateTimeHealthState(state, time_valid);

    if (connected && s_next_time_sync_ticks != 0 &&
        now_ticks >= s_next_time_sync_ticks) {
      const char* sntp_server = AppNetConfigGetSntpServer();
      esp_err_t sntp_result = ESP_ERR_INVALID_STATE;
      if (sntp_server != NULL && sntp_server[0] != '\0') {
        sntp_result = TimeSyncStartSntpAndWait(sntp_server, 30 * 1000);
        if (sntp_result == ESP_OK) {
          (void)TimeSyncSetRtcFromSystem(&state->time_sync);
          state->wifi_direct_time_synced = true;
          ESP_LOGI(kTag, "Time synchronized (SNTP -> RTC UTC)");
        } else {
          ESP_LOGW(kTag, "SNTP sync failed: %s", esp_err_to_name(sntp_result));
        }
      }

      // Schedule next sync. If the configured period is zero, we only sync once
      // per connect (unless manually requested).
      if (time_sync_period_s == 0) {
        s_next_time_sync_ticks = 0;
      } else {
        TickType_t period_ticks =
          pdMS_TO_TICKS((uint64_t)time_sync_period_s * 1000ULL);
        if (period_ticks == 0) {
          period_ticks = pdMS_TO_TICKS(60 * 1000);
        }

        // If the sync failed, retry sooner (but not aggressively).
        if (sntp_result != ESP_OK) {
          const TickType_t retry_ticks = pdMS_TO_TICKS(30 * 1000);
          if (period_ticks > retry_ticks) {
            period_ticks = retry_ticks;
          }
        }

        s_next_time_sync_ticks = now_ticks + period_ticks;
      }

      time_valid = TimeSyncIsSystemTimeValid();
      UpdateTimeHealthState(state, time_valid);
      last_time_valid = time_valid;
    } else if (time_valid != last_time_valid) {
      last_time_valid = time_valid;
    }

    RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(retry_delay_ms));
  }

  state->wifi_direct_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute TopologyTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the TopologyTask task.
 */
static void
TopologyTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  const TickType_t interval_ticks = pdMS_TO_TICKS(30 * 1000);
  const TickType_t warn_period_ticks = pdMS_TO_TICKS(5 * 60 * 1000);
  TickType_t last_disconnected_warn_ticks = 0;

  // Only emit the topology status line when something changes. This avoids
  // spamming the CSV/console output with repeated identical lines.
  bool have_prev_status = false;
  char prev_role[16] = { 0 };
  bool prev_allow_children = false;
  int prev_layer = -9999;
  char prev_parent_str[20] = { 0 };
  uint32_t prev_child_count = 0;
  int prev_rssi = -9999;

  // const TickType_t watermark_log_period_ticks = pdMS_TO_TICKS(5 * 60 * 1000);
  // TickType_t last_watermark_log_ticks = 0;

  // const UBaseType_t initial_watermark_words =
  // uxTaskGetStackHighWaterMark(NULL); ESP_LOGI(kTag,
  //          "topology stack watermark: %u words (%u bytes) free",
  //          (unsigned)initial_watermark_words,
  //          (unsigned)(initial_watermark_words * sizeof(StackType_t)));

  while (!state->stop_requested) {
    const char* role = AppSettingsRoleToString(state->settings.node_role);
    const uint32_t child_count = esp_mesh_lite_get_mesh_node_number();
    int layer = -1;
    int rssi = 0;
    char parent_str[20] = "unknown";

    if (MeshTransportIsStarted(&state->mesh)) {
      layer = esp_mesh_lite_get_level();
      mesh_lite_ap_record_t ap_record = { 0 };
      if (esp_mesh_lite_get_ap_record(&ap_record) == ESP_OK) {
        FormatMacString(ap_record.bssid, parent_str, sizeof(parent_str));
        rssi = ap_record.rssi;
      }
    }
    UpdateCachedInt32(state, &state->cached_status.mesh_level, layer);
    UpdateCachedInt32(state, &state->cached_status.mesh_rssi, rssi);
    UpdateCachedBool(state, &state->cached_status.mesh_connected, (layer > 0));

    // Replace the vendor/wifi spam with a single rate-limited warning.
    const bool connected_now = (layer > 0);
    if (connected_now) {
      last_disconnected_warn_ticks = 0;
    } else if (MeshTransportIsStarted(&state->mesh)) {
      const TickType_t now_ticks = xTaskGetTickCount();
      const bool should_warn =
        (last_disconnected_warn_ticks == 0) ||
        ((now_ticks - last_disconnected_warn_ticks) >= warn_period_ticks);
      if (should_warn) {
        ESP_LOGW(kTag,
                 "Mesh not connected (layer=%d). Still scanning for AP/root...",
                 layer);
        last_disconnected_warn_ticks = now_ticks;
      }
    }

    if (!state->log_quiet) {
      const bool allow_children = state->settings.allow_children;
      const bool changed =
        (!have_prev_status) ||
        (strncmp(prev_role, role, sizeof(prev_role)) != 0) ||
        (prev_allow_children != allow_children) || (prev_layer != layer) ||
        (strncmp(prev_parent_str, parent_str, sizeof(prev_parent_str)) != 0) ||
        (prev_child_count != child_count) || (prev_rssi != rssi);

      if (changed) {
        printf("topology role=%s allow_children=%u layer=%d parent=%s "
               "children=%u rssi=%d\n",
               role,
               allow_children ? 1u : 0u,
               layer,
               parent_str,
               (unsigned)child_count,
               rssi);

        strlcpy(prev_role, role, sizeof(prev_role));
        prev_allow_children = allow_children;
        prev_layer = layer;
        strlcpy(prev_parent_str, parent_str, sizeof(prev_parent_str));
        prev_child_count = child_count;
        prev_rssi = rssi;
        have_prev_status = true;
      }
    }

    // const TickType_t now_ticks = xTaskGetTickCount();
    // if ((last_watermark_log_ticks == 0) ||
    //     ((now_ticks - last_watermark_log_ticks) >=
    //      watermark_log_period_ticks)) {
    //   const UBaseType_t watermark_words = uxTaskGetStackHighWaterMark(NULL);
    //   ESP_LOGI(kTag,
    //            "topology stack watermark: %u words (%u bytes) free",
    //            (unsigned)watermark_words,
    //            (unsigned)(watermark_words * sizeof(StackType_t)));
    //   last_watermark_log_ticks = now_ticks;
    // }
    RuntimeInterruptibleDelayTicks(interval_ticks);
  }

  state->topology_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute DrainFramToSd.
 * @param state Parameter state.
 * @param unmount_on_exit Parameter unmount_on_exit.
 * @param max_duration_ms Parameter max_duration_ms.
 * @param max_records_per_pass Parameter max_records_per_pass.
 * @param yield_every_records Parameter yield_every_records.
 * @param out_stats Parameter out_stats.
 * @return Return the function result.
 */
static esp_err_t
DrainFramToSd(runtime_state_t* state,
              bool unmount_on_exit,
              int32_t max_duration_ms,
              int32_t max_records_per_pass,
              int32_t yield_every_records,
              sd_drain_stats_t* out_stats)
{
  if (out_stats != NULL) {
    memset(out_stats, 0, sizeof(*out_stats));
    out_stats->result = ESP_OK;
  }

  if (state == NULL) {
    if (out_stats != NULL) {
      out_stats->result = ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_INVALID_ARG;
  }
  const TickType_t start_ticks = xTaskGetTickCount();
  const TickType_t saved_backoff = state->sd_backoff_until_ticks;
  state->sd_backoff_until_ticks = 0;

  bool mounted_here = false;
  int32_t flushed_records = 0;
  int32_t flushed_bytes = 0;
  esp_err_t result = ESP_OK;

  if (state->batch_buffer == NULL || state->batch_buffer_size == 0) {
    result = ESP_ERR_NO_MEM;
    goto drain_done;
  }

  if (!SdCardPresent(state)) {
    result = ESP_ERR_NOT_FOUND;
    goto drain_done;
  }

  if (!state->sd_logger.is_mounted) {
    if (state->sd_flush_in_progress) {
      const TickType_t now_ticks = xTaskGetTickCount();
      if (state->last_sd_flush_wait_warn_ticks == 0 ||
          (now_ticks - state->last_sd_flush_wait_warn_ticks) >
            pdMS_TO_TICKS(kSdFlushWarnIntervalMs)) {
        ESP_LOGW(kTag, "SD drain already in progress; waiting for SD lock");
        state->last_sd_flush_wait_warn_ticks = now_ticks;
      }
    }
    if (!RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
      result = ESP_ERR_TIMEOUT;
      goto drain_done;
    }
    state->sd_flush_in_progress = true;
    RuntimeDiagHeapCheck(state, "SD mount (drain before)", false);
    esp_err_t mount_result = SdLoggerTryRemount(&state->sd_logger, false);
    RuntimeDiagHeapCheck(state, "SD mount (drain after)", false);
    state->sd_flush_in_progress = false;
    RuntimeSdIoUnlock(state);
    if (mount_result != ESP_OK) {
      MarkSdFailure(state, "SD mount failed", "mount", mount_result, 0, false);
      UpdateCachedBool(
        state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
      result = mount_result;
      goto drain_done;
    }
    mounted_here = true;
    ClearSdIoError(state);
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  }

  const int32_t drain_records_per_pass = (max_records_per_pass > 0)
                                           ? max_records_per_pass
                                           : (int32_t)kSdFlushMaxRecordsPerPass;
  const int32_t yield_interval_records =
    (yield_every_records > 0) ? yield_every_records : drain_records_per_pass;

  int32_t records_since_yield = 0;

  RuntimeDiagHeapCheck(state, "DrainFramToSd loop (before)", false);
  while (FramLogGetBufferedRecords(&state->fram_log) > 0) {
    uint32_t flushed = 0;
    size_t bytes_flushed = 0;
    bool more_pending = false;
    result = SdFlushWorkerTickEx(state,
                                 (uint32_t)drain_records_per_pass,
                                 kSdFlushMaxMsPerPass,
                                 SD_APPEND_VERIFY_NONE,
                                 SD_APPEND_FLUSH_NEVER,
                                 &flushed,
                                 &bytes_flushed,
                                 &more_pending);
    if (result != ESP_OK) {
      break;
    }
    if (flushed > 0) {
      state->sd_flush_records_since = 0;
    }
    flushed_records += (int32_t)flushed;
    flushed_bytes += (int32_t)bytes_flushed;
    records_since_yield += (int32_t)flushed;
    UpdateFramFillState(state);
#if CONFIG_APP_DRAIN_LOG_PROGRESS
    if (flushed > 0) {
      ESP_LOGI(kTag,
               "Drain progress: flushed=%d remaining=%u",
               flushed_records,
               (unsigned)FramLogGetBufferedRecords(&state->fram_log));
    }
#endif
    if (!more_pending) {
      break;
    }

    if (yield_interval_records > 0 &&
        records_since_yield >= yield_interval_records) {
      vTaskDelay(1);
      records_since_yield = 0;
    }

    if (max_duration_ms >= 0 &&
        pdTICKS_TO_MS(xTaskGetTickCount() - start_ticks) >=
          (uint32_t)max_duration_ms) {
      result = ESP_ERR_TIMEOUT;
      break;
    }
  }
  RuntimeDiagHeapCheck(state, "DrainFramToSd loop (after)", false);

drain_done:
  if (state->sd_backoff_until_ticks == 0 && saved_backoff != 0) {
    state->sd_backoff_until_ticks = saved_backoff;
  }

  const uint32_t remaining =
    (uint32_t)FramLogGetBufferedRecords(&state->fram_log);
  const uint32_t duration_ms =
    (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - start_ticks);

  UpdateCachedInt32(state, &state->cached_status.last_drain_result, result);
  UpdateCachedUint32(
    state, &state->cached_status.last_drain_remaining, remaining);
  UpdateCachedUint32(
    state, &state->cached_status.last_drain_duration_ms, duration_ms);
  UpdateCachedInt32(
    state, &state->cached_status.last_drain_flushed_records, flushed_records);
  UpdateCachedInt32(
    state, &state->cached_status.last_drain_flushed_bytes, flushed_bytes);

  if (out_stats != NULL) {
    out_stats->flushed_records = flushed_records;
    out_stats->remaining_records = (int32_t)remaining;
    out_stats->flushed_bytes = flushed_bytes;
    out_stats->duration_ms = (int32_t)duration_ms;
    out_stats->result = result;
  }

  ESP_LOGI(kTag,
           "Drain FRAM->SD: flushed=%d remaining=%u duration=%u ms result=%s",
           flushed_records,
           (unsigned)remaining,
           (unsigned)duration_ms,
           esp_err_to_name(result));
  if (result == ESP_ERR_TIMEOUT) {
    ESP_LOGW(kTag, "Drain timed out; remaining=%u", (unsigned)remaining);
  }

  if (unmount_on_exit) {
    if (state->sd_flush_in_progress) {
      const TickType_t now_ticks = xTaskGetTickCount();
      if (state->last_sd_flush_wait_warn_ticks == 0 ||
          (now_ticks - state->last_sd_flush_wait_warn_ticks) >
            pdMS_TO_TICKS(kSdFlushWarnIntervalMs)) {
        ESP_LOGW(kTag, "SD drain already in progress; waiting for SD lock");
        state->last_sd_flush_wait_warn_ticks = now_ticks;
      }
    }
    if (RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
      state->sd_flush_in_progress = true;
      RuntimeDiagHeapCheck(state, "SD unmount (drain before)", false);
      (void)SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (drain after)", false);
      state->sd_flush_in_progress = false;
      RuntimeSdIoUnlock(state);
    }
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  } else if (mounted_here && !state->sd_logger.is_mounted) {
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  }

  return result;
}

static void
RuntimeStopForceSdUnmount(runtime_state_t* state,
                          const char* reason,
                          const sd_drain_stats_t* drain_stats)
{
  if (state == NULL) {
    return;
  }

  state->sd_flush_pending = false;
  state->sd_start_drain_pending = false;
  state->sd_degraded = true;

  if (drain_stats != NULL) {
    ESP_LOGW(kTag,
             "SD STOP INCOMPLETE: %s (remaining=%d duration=%d ms)",
             reason,
             drain_stats->remaining_records,
             drain_stats->duration_ms);
  } else {
    ESP_LOGW(kTag, "SD STOP INCOMPLETE: %s", reason);
  }

  SdCsvAppendDiagnostics diag = { 0 };
  const bool locked = RuntimeSdIoLock(state, kSdIoLockTimeoutTicks);
  if (!locked) {
    ESP_LOGW(kTag,
             "SD STOP INCOMPLETE: SD I/O lock timeout; attempting unmount without lock");
  } else {
    state->sd_flush_in_progress = true;
  }

  if (state->sd_logger.file != NULL) {
    esp_err_t flush_result = SdLoggerFlushAndSync(&state->sd_logger, &diag);
    if (flush_result != ESP_OK) {
      const char* op = (diag.operation != NULL) ? diag.operation : "flush";
      const char* errno_str =
        (diag.errno_value != 0) ? strerror(diag.errno_value) : "n/a";
      ESP_LOGW(kTag,
               "SD STOP INCOMPLETE: %s failed errno=%d (%s)",
               op,
               diag.errno_value,
               errno_str);
    }
  }

  (void)SdLoggerUnmount(&state->sd_logger);

  if (locked) {
    state->sd_flush_in_progress = false;
    RuntimeSdIoUnlock(state);
  }

  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  UpdateCachedBool(
    state, &state->cached_status.sd_degraded, state->sd_degraded);
}

/**
 * @brief Execute UpdateStartDrainCachedStatus.
 * @param state Parameter state.
 * @param stats Parameter stats.
 */
static void
UpdateStartDrainCachedStatus(runtime_state_t* state,
                             const sd_drain_stats_t* stats)
{
  if (state == NULL || stats == NULL) {
    return;
  }
  UpdateCachedInt32(
    state, &state->cached_status.last_drain_result, stats->result);
  UpdateCachedUint32(state,
                     &state->cached_status.last_drain_remaining,
                     (uint32_t)stats->remaining_records);
  UpdateCachedUint32(state,
                     &state->cached_status.last_drain_duration_ms,
                     (uint32_t)stats->duration_ms);
  UpdateCachedInt32(state,
                    &state->cached_status.last_drain_flushed_records,
                    stats->flushed_records);
  UpdateCachedInt32(state,
                    &state->cached_status.last_drain_flushed_bytes,
                    stats->flushed_bytes);
}

/**
 * @brief Execute DrainFramToSdOnStartBestEffort.
 * @param state Parameter state.
 * @param out_stats Parameter out_stats.
 * @return Return the function result.
 */
static esp_err_t
DrainFramToSdOnStartBestEffort(runtime_state_t* state,
                               sd_drain_stats_t* out_stats)
{
  sd_drain_stats_t local_stats = { 0 };
  sd_drain_stats_t* stats = (out_stats != NULL) ? out_stats : &local_stats;
  if (stats == &local_stats) {
    memset(stats, 0, sizeof(*stats));
  }

  if (state == NULL) {
    stats->result = ESP_ERR_INVALID_ARG;
    UpdateStartDrainCachedStatus(state, stats);
    ESP_LOGW(kTag,
             "start drain: flushed=%d remaining=%d duration=%d ms result=%s",
             stats->flushed_records,
             stats->remaining_records,
             stats->duration_ms,
             esp_err_to_name(stats->result));
    return ESP_ERR_INVALID_ARG;
  }

#if !CONFIG_APP_START_DRAIN_ENABLE
  const int32_t initial_remaining =
    (int32_t)FramLogGetBufferedRecords(&state->fram_log);
  EnsureSdMounted();
  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  stats->flushed_records = 0;
  stats->remaining_records = initial_remaining;
  stats->flushed_bytes = 0;
  stats->duration_ms = 0;
  stats->result = ESP_ERR_NOT_SUPPORTED;
  UpdateStartDrainCachedStatus(state, stats);
  UpdateFramFillState(state);
  ESP_LOGW(kTag,
           "start drain: flushed=%d remaining=%d duration=%d ms result=%s",
           stats->flushed_records,
           stats->remaining_records,
           stats->duration_ms,
           esp_err_to_name(stats->result));
  return stats->result;
#endif

  const int32_t initial_remaining =
    (int32_t)FramLogGetBufferedRecords(&state->fram_log);

  EnsureSdMounted();
  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);

  if (initial_remaining <= 0) {
    state->sd_start_drain_pending = false;
    stats->flushed_records = 0;
    stats->remaining_records = 0;
    stats->flushed_bytes = 0;
    stats->duration_ms = 0;
    stats->result = ESP_OK;
    UpdateStartDrainCachedStatus(state, stats);
    UpdateFramFillState(state);
    ESP_LOGW(kTag,
             "start drain: flushed=%d remaining=%d duration=%d ms result=%s",
             stats->flushed_records,
             stats->remaining_records,
             stats->duration_ms,
             esp_err_to_name(stats->result));
    return ESP_OK;
  }

  if (!state->sd_logger.is_mounted) {
    stats->flushed_records = 0;
    stats->remaining_records = initial_remaining;
    stats->flushed_bytes = 0;
    stats->duration_ms = 0;
    stats->result = ESP_ERR_INVALID_STATE;
    UpdateStartDrainCachedStatus(state, stats);
    UpdateFramFillState(state);
    // SD isn't available yet. Make sure the normal storage loop drains FRAM
    // as soon as the card mounts (don't wait for watermark/periodic flush).
    state->sd_start_drain_pending = true;
    state->sd_flush_pending = true;
    state->sd_next_flush_allowed_ticks = 0;
    ESP_LOGW(kTag, "Start drain skipped; SD not mounted");
    ESP_LOGW(kTag,
             "start drain: flushed=%d remaining=%d duration=%d ms result=%s",
             stats->flushed_records,
             stats->remaining_records,
             stats->duration_ms,
             esp_err_to_name(stats->result));
    return stats->result;
  }

  esp_err_t result = DrainFramToSd(state,
                                   false,
                                   CONFIG_APP_START_DRAIN_MAX_MS,
                                   CONFIG_APP_START_DRAIN_MAX_RECORDS_PER_PASS,
                                   CONFIG_APP_START_DRAIN_YIELD_EVERY_RECORDS,
                                   stats);

  if (result == ESP_ERR_TIMEOUT) {
    state->sd_flush_pending = true;
    state->sd_start_drain_pending = true;
  } else if (result == ESP_OK && stats->remaining_records <= 0) {
    state->sd_start_drain_pending = false;
  }

  UpdateFramFillState(state);
  UpdateStartDrainCachedStatus(state, stats);
  ESP_LOGW(kTag,
           "start drain: flushed=%d remaining=%d duration=%d ms result=%s",
           stats->flushed_records,
           stats->remaining_records,
           stats->duration_ms,
           esp_err_to_name(stats->result));
  return result;
}

/**
 * @brief Execute ControlTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the ControlTask task.
 */
static void
ControlTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;

  while (true) {
    bool request_start = false;
    bool request_stop = false;
    taskENTER_CRITICAL(&state->request_lock);
    request_start = state->request_run_start;
    request_stop = state->request_run_stop;
    state->request_run_start = false;
    state->request_run_stop = false;
    taskEXIT_CRITICAL(&state->request_lock);

    if (request_stop && RuntimeIsRunning()) {
      (void)EnterDiagMode();
    } else if (request_start && !RuntimeIsRunning()) {
      (void)EnterRunMode();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/**
 * @brief Execute RuntimeFlushToSd.
 * @param context Parameter context.
 * @return Return the function result.
 */
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

/**
 * @brief Execute InitSpiBus.
 * @param host Parameter host.
 * @return Return the function result.
 */
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
  esp_err_t result = spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO);
  if (result == ESP_ERR_INVALID_STATE) {
    return ESP_OK;
  }
  return result;
}

/**
 * @brief Execute GetSpiHost.
 * @return Return the function result.
 */
static spi_host_device_t
GetSpiHost(void)
{
  return (CONFIG_APP_SPI_HOST == 3) ? SPI3_HOST : SPI2_HOST;
}

/**
 * @brief Execute GetDisplaySpiHost.
 * @return Return the function result.
 */
static spi_host_device_t
GetDisplaySpiHost(void)
{
  return (CONFIG_APP_MAX7219_SPI_HOST == 3) ? SPI3_HOST : SPI2_HOST;
}

/**
 * @brief Execute InitializeMax31865Sensor.
 * @param state Parameter state.
 * @param spi_host Parameter spi_host.
 * @return Return the function result.
 */
static esp_err_t
InitializeMax31865Sensor(runtime_state_t* state, spi_host_device_t spi_host)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t sensor_result =
    Max31865ReaderInit(&state->sensor, spi_host, CONFIG_APP_MAX31865_CS_GPIO);
  if (sensor_result != ESP_OK) {
    ESP_LOGE(
      kTag, "Max31865ReaderInit failed: %s", esp_err_to_name(sensor_result));
    return sensor_result;
  }
  if (state->settings.calibration.is_valid) {
    calibration_context_t current_context;
    AppSettingsBuildCalibrationContextFromReader(&current_context,
                                                 &state->sensor);
    char reason[128];
    if (!CalibrationContextMatches(
          &state->settings, &current_context, reason, sizeof(reason))) {
      CalibrationModelInitIdentity(&state->settings.calibration);
      state->settings.calibration.is_valid = false;
      ESP_LOGW(kTag, "Calibration invalidated: %s", reason);
    }
  }
  return ESP_OK;
}

#if CONFIG_APP_MAX7219_ENABLE
/**
 * @brief Execute InitializeMax7219Display.
 * @param state Parameter state.
 * @return Return the function result.
 */
static esp_err_t
InitializeMax7219Display(runtime_state_t* state)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  max7219_display_config_t display_config = {
    .host = GetDisplaySpiHost(),
    .mosi_gpio = CONFIG_APP_MAX7219_MOSI_GPIO,
    .sclk_gpio = CONFIG_APP_MAX7219_SCLK_GPIO,
    .cs_gpio = CONFIG_APP_MAX7219_CS_GPIO,
    .chain_len = CONFIG_APP_MAX7219_CHAIN_LEN,
    .clock_hz = 2000000,
    .intensity = CONFIG_APP_MAX7219_INTENSITY,
  };
  esp_err_t display_result =
    Max7219DisplayInit(&state->display, &display_config);
  if (display_result == ESP_OK) {
    state->display_initialized = true;
  } else {
    state->display_initialized = false;
    ESP_LOGW(
      kTag, "Max7219 display init failed: %s", esp_err_to_name(display_result));
  }
  return display_result;
}
#endif

/**
 * @brief Execute InitializeRuntimeStruct.
 */
static void
InitializeRuntimeStruct(void)
{
  memset(&g_state, 0, sizeof(g_state));
  memset(&g_runtime, 0, sizeof(g_runtime));
  g_state.last_temp_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  g_state.request_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  MqttClientWrapInit(&g_state.mqtt_client);
  RuntimeHealthInit(&g_state.health_cache);
  RuntimeHealthPublisherInit(&g_state);
  g_state.cached_status.mesh_level = -1;

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
  g_runtime.alert_manager = &g_state.alert_manager;
  g_runtime.flush_callback = &RuntimeFlushToSd;
  g_runtime.flush_context = &g_state;
  g_runtime.fram_full = &g_state.fram_full;
  g_runtime.export_dropped_count = &g_state.export_dropped_count;
  g_runtime.export_write_fail_count = &g_state.export_write_fail_count;
  g_runtime.export_outbox = &g_state.export_outbox;
  g_runtime.broker_outbox = &g_state.broker_outbox;
  g_runtime.export_drop_count = &g_state.export_drop_count;
  g_runtime.export_send_fail_count = &g_state.export_send_fail_count;
  g_runtime.broker_drop_count = &g_state.broker_drop_count;
  g_runtime.broker_send_fail_count = &g_state.broker_send_fail_count;
  g_runtime.mqtt_client_connected = &g_state.mqtt_client_connected;
}

/**
 * @brief Execute RuntimeGetRuntime.
 * @return Return the function result.
 */
const app_runtime_t*
RuntimeGetRuntime(void)
{
  return g_state.initialized ? &g_runtime : NULL;
}

/**
 * @brief Execute RuntimeGetCachedStatus.
 * @return Return the function result.
 */
const runtime_cached_status_t*
RuntimeGetCachedStatus(void)
{
  return g_state.initialized ? &g_state.cached_status : NULL;
}

/**
 * @brief Execute RuntimeGetState.
 * @return Return the function result.
 */
runtime_state_t*
RuntimeGetState(void)
{
  return g_state.initialized ? &g_state : NULL;
}

/**
 * @brief Execute RuntimeManagerInit.
 * @return Return the function result.
 */
esp_err_t
RuntimeManagerInit(void)
{
  InitializeRuntimeStruct();
  esp_err_t first_error = ESP_OK;

  g_state.sd_io_mutex =
    xSemaphoreCreateMutexStatic(&g_state.sd_io_mutex_buf);
  if (g_state.sd_io_mutex == NULL) {
    ESP_LOGE(kTag, "Failed to create SD I/O mutex; SD marked degraded");
    g_state.sd_degraded = true;
    UpdateCachedBool(&g_state, &g_state.cached_status.sd_degraded, true);
  }

  uint8_t mac[6] = { 0 };
  esp_err_t mac_result = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if (mac_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = mac_result;
    }
    ESP_LOGE(kTag, "esp_read_mac failed: %s", esp_err_to_name(mac_result));
  }
  memcpy(g_state.local_mac, mac, sizeof(g_state.local_mac));
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

  esp_err_t net_config_result = AppNetConfigInit();
  if (net_config_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = net_config_result;
    }
    ESP_LOGE(
      kTag, "AppNetConfigInit failed: %s", esp_err_to_name(net_config_result));
  }
  UpdateCachedUint32(&g_state,
                     &g_state.cached_status.disp_attn_mask,
                     g_state.settings.display_attention_mask);
  UpdateCachedUint32(&g_state,
                     &g_state.cached_status.disp_attn_pol,
                     g_state.settings.display_attention_policy);
  UpdateCachedUint32(&g_state,
                     &g_state.cached_status.fram_flush_watermark_records,
                     g_state.settings.fram_flush_watermark_records);
  UpdateCachedBool(&g_state, &g_state.cached_status.runtime_running, false);
  UpdateCachedBool(&g_state, &g_state.cached_status.stop_requested, false);
  SdCardDetectInit(&g_state.sd_card_detect);
  const bool sd_card_present = SdCardDetectPoll(&g_state.sd_card_detect, NULL);
  UpdateCachedBool(
    &g_state, &g_state.cached_status.sd_card_present, sd_card_present);
  RuntimeHealthPublisherTick(&g_state);

  AlertManagerInit(&g_state.alert_manager, g_state.node_id_string);
  (void)AlertManagerLoadConfig(&g_state.alert_manager);

#if CONFIG_APP_MAX7219_ENABLE
  esp_err_t display_result = InitializeMax7219Display(&g_state);
  if (display_result == ESP_OK && g_state.display_initialized) {
    BaseType_t display_created = xTaskCreate(
      &DisplayTask, "display", 4096, &g_state, 2, &g_state.display_task);
    if (display_created != pdPASS) {
      g_state.display_initialized = false;
      g_state.display_task = NULL;
      ESP_LOGW(kTag, "Display task create failed");
    }
  }
#endif

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

  // Batch buffer is purely a staging buffer for file I/O; it is safe to place
  // in PSRAM and doing so preserves scarce internal heap for Wi-Fi/COEX.
  {
    size_t desired_bytes = g_state.sd_logger.config.batch_target_bytes;
    const size_t kMinBatchBytes = 4096;
    const size_t kMaxBatchBytes = 64 * 1024;
    if (desired_bytes < kMinBatchBytes) {
      desired_bytes = kMinBatchBytes;
    }
    if (desired_bytes > kMaxBatchBytes) {
      desired_bytes = kMaxBatchBytes;
    }

    g_state.batch_buffer_size = desired_bytes;
    g_state.batch_buffer =
      (uint8_t*)AllocatePreferPsram(g_state.batch_buffer_size);
    if (g_state.batch_buffer == NULL) {
      // As a last resort, try a smaller buffer rather than failing init.
      g_state.batch_buffer_size = kMinBatchBytes;
      g_state.batch_buffer =
        (uint8_t*)AllocatePreferPsram(g_state.batch_buffer_size);
    }
  }

  {
    const size_t kSdIoBounceBytes = 4096;
    g_state.sd_logger.io_bounce_bytes = (uint8_t*)heap_caps_malloc(
      kSdIoBounceBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (g_state.sd_logger.io_bounce_bytes != NULL) {
      g_state.sd_logger.io_bounce_capacity = kSdIoBounceBytes;
    } else {
      g_state.sd_logger.io_bounce_capacity = 0;
      ESP_LOGW(kTag, "SD I/O bounce buffer allocation failed");
    }

    size_t verify_bytes = g_state.sd_logger.config.batch_target_bytes;
    const size_t kVerifyReadbackMaxBytes = 64 * 1024;
    if (verify_bytes > kVerifyReadbackMaxBytes) {
      verify_bytes = kVerifyReadbackMaxBytes;
    }
    if (verify_bytes > 0) {
      g_state.sd_logger.verify_readback_bytes =
        (uint8_t*)AllocatePreferPsram(verify_bytes);
    }
    if (g_state.sd_logger.verify_readback_bytes != NULL) {
      g_state.sd_logger.verify_readback_capacity = verify_bytes;
    } else {
      g_state.sd_logger.verify_readback_capacity = 0;
    }
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
    UpdateTimeHealthState(&g_state, TimeSyncIsSystemTimeValid());
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

  if (g_state.sd_io_mutex != NULL && SdCardPresent(&g_state)) {
    if (RuntimeSdIoLock(&g_state, kSdIoLockTimeoutTicks)) {
      RuntimeDiagHeapCheck(&g_state, "SD mount (init before)", false);
      (void)SdLoggerMount(&g_state.sd_logger, spi_host, CONFIG_APP_SD_CS_GPIO);
      RuntimeDiagHeapCheck(&g_state, "SD mount (init after)", false);
      RuntimeSdIoUnlock(&g_state);
    }
  }
  UpdateCachedBool(
    &g_state, &g_state.cached_status.sd_mounted, g_state.sd_logger.is_mounted);

  esp_err_t sensor_result = InitializeMax31865Sensor(&g_state, spi_host);
  if (sensor_result != ESP_OK && first_error == ESP_OK) {
    first_error = sensor_result;
  }

  if (g_log_queue_storage == NULL) {
    g_log_queue_storage =
      (uint8_t*)AllocatePreferPsram(64 * sizeof(log_record_t));
  }
  g_state.log_queue = xQueueCreateStatic(
    64, sizeof(log_record_t), g_log_queue_storage, &g_log_queue_struct);
  if (g_state.log_queue == NULL) {
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to create log queue");
  }

  if (g_export_queue_storage == NULL) {
    g_export_queue_storage =
      (uint8_t*)AllocatePreferPsram(kExportQueueDepth * sizeof(export_item_t));
  }
  g_state.export_queue = xQueueCreateStatic(kExportQueueDepth,
                                            sizeof(export_item_t),
                                            g_export_queue_storage,
                                            &g_export_queue_struct);
  if (g_state.export_queue == NULL) {
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to create export queue");
  }

  if (g_export_outbox_storage == NULL) {
    g_export_outbox_storage = (uint8_t*)AllocatePreferPsram(
      kExportOutboxDepth * sizeof(export_record_item_t));
  }
  g_state.export_outbox = xQueueCreateStatic(kExportOutboxDepth,
                                             sizeof(export_record_item_t),
                                             g_export_outbox_storage,
                                             &g_export_outbox_queue_struct);
  if (g_state.export_outbox == NULL) {
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to create export outbox");
  }

  if (g_broker_outbox_storage == NULL) {
    g_broker_outbox_storage = (uint8_t*)AllocatePreferPsram(
      kBrokerOutboxDepth * sizeof(broker_publish_item_t));
  }
  g_state.broker_outbox = xQueueCreateStatic(kBrokerOutboxDepth,
                                             sizeof(broker_publish_item_t),
                                             g_broker_outbox_storage,
                                             &g_broker_outbox_queue_struct);
  if (g_state.broker_outbox == NULL) {
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to create broker outbox");
  }

  BaseType_t control_created = xTaskCreate(
    &ControlTask, "control", 3072, &g_state, 3, &g_state.control_task);
  if (control_created != pdPASS) {
    g_state.control_task = NULL;
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to create control task");
  }

  g_state.initialized = true;
  return first_error;
}

/**
 * @brief Execute EnsureSdMounted.
 */
static void
EnsureSdMounted(void)
{
  if (!g_state.sd_logger.is_mounted) {
    if (!SdCardPresent(&g_state)) {
      return;
    }
    if (!RuntimeSdIoLock(&g_state, kSdIoLockTimeoutTicks)) {
      return;
    }
    RuntimeDiagHeapCheck(&g_state, "SD mount (ensure before)", false);
    esp_err_t mount_result =
      SdLoggerMount(&g_state.sd_logger, GetSpiHost(), CONFIG_APP_SD_CS_GPIO);
    RuntimeDiagHeapCheck(&g_state, "SD mount (ensure after)", false);
    RuntimeSdIoUnlock(&g_state);
    if (mount_result != ESP_OK) {
      MarkSdFailure(
        &g_state, "SD mount failed", "mount", mount_result, 0, false);
    } else {
      ClearSdIoError(&g_state);
      UpdateCachedBool(&g_state,
                       &g_state.cached_status.sd_mounted,
                       g_state.sd_logger.is_mounted);
    }
  }
}

/**
 * @brief Execute SdWithTemporaryMount.
 * @param state Parameter state.
 * @param op Parameter op.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdWithTemporaryMount(runtime_state_t* state, runtime_sd_op_fn_t op, void* ctx)
{
  if (state == NULL || op == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!SdCardPresent(state)) {
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
    return ESP_ERR_NOT_FOUND;
  }

  bool mounted_here = false;
  if (!state->sd_logger.is_mounted) {
    if (!RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    RuntimeDiagHeapCheck(state, "SD mount (temp before)", false);
    esp_err_t mount_result = SdLoggerTryRemount(&state->sd_logger, false);
    RuntimeDiagHeapCheck(state, "SD mount (temp after)", false);
    RuntimeSdIoUnlock(state);
    if (mount_result != ESP_OK) {
      UpdateCachedBool(
        state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
      ESP_LOGW(kTag,
               "SD mount failed for diagnostics: %s",
               esp_err_to_name(mount_result));
      return mount_result;
    }
    mounted_here = true;
    ClearSdIoError(state);
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  }

  esp_err_t result = op(&g_runtime, ctx);

  if (mounted_here) {
    esp_err_t unmount_result = ESP_OK;
    if (RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
      RuntimeDiagHeapCheck(state, "SD unmount (temp before)", false);
      unmount_result = SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (temp after)", false);
      RuntimeSdIoUnlock(state);
    } else {
      unmount_result = ESP_ERR_TIMEOUT;
    }
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
    if (result == ESP_OK && unmount_result != ESP_OK) {
      result = unmount_result;
    }
  }

  return result;
}

/**
 * @brief Execute RuntimeWithTemporarySdMount.
 * @param op Parameter op.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
esp_err_t
RuntimeWithTemporarySdMount(runtime_sd_op_fn_t op, void* ctx)
{
  return SdWithTemporaryMount(&g_state, op, ctx);
}

/**
 * @brief Execute RuntimeStart.
 * @return Return the function result.
 */
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
      g_state.time_sync_task != NULL || g_state.topology_task != NULL ||
      g_state.health_publisher_task != NULL ||
      g_state.wifi_direct_task != NULL) {
    if (g_state.sensor_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: sensor_task still alive");
    }
    if (g_state.storage_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: storage_task still alive");
    }
    if (g_state.time_sync_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: time_sync_task still alive");
    }
    if (g_state.topology_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: topology_task still alive");
    }
    if (g_state.health_publisher_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: health_publisher_task still alive");
    }
    if (g_state.wifi_direct_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: wifi_direct_task still alive");
    }
    return ESP_ERR_INVALID_STATE;
  }
  if (g_state.log_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }
  if (g_state.export_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }
  if (g_state.batch_buffer == NULL || g_state.batch_buffer_size == 0) {
    return ESP_ERR_NO_MEM;
  }

  if (!g_state.sensor.is_initialized) {
    esp_err_t sensor_result =
      InitializeMax31865Sensor(&g_state, GetSpiHost());
    if (sensor_result != ESP_OK) {
      return sensor_result;
    }
  }

#if CONFIG_APP_MAX7219_ENABLE
  if (!g_state.display_initialized) {
    (void)InitializeMax7219Display(&g_state);
  }
#endif

  g_state.stop_requested = false;
  UpdateCachedBool(&g_state, &g_state.cached_status.stop_requested, false);
  g_state.fram_full = false;
  g_state.sd_degraded = false;
  g_state.sd_fail_count = 0;
  g_state.sd_backoff_until_ticks = 0;
  g_state.last_sd_flush_warn_ticks = 0;
  g_state.last_sd_flush_wait_warn_ticks = 0;
  g_state.sd_flush_records_since = 0;
  g_state.sd_flush_in_progress = false;
  g_state.sd_flush_pending = false;
  g_state.sd_start_drain_pending = false;
  g_state.sd_next_flush_allowed_ticks = 0;
  g_state.sd_last_io_error_active = false;
  g_state.sd_last_io_err = ESP_OK;
  g_state.sd_last_errno = 0;
  g_state.last_overrun_log_ticks = 0;
  g_state.last_overrun_records_total = 0;
  g_state.last_overrun_logged_total = 0;
  g_state.wifi_direct_started = false;
  g_state.wifi_direct_time_synced = false;
  g_state.csv_header_emitted = false;
  g_state.root_bridge_header_emitted = false;
  g_state.broker_bridge_requested_without_mqtt = false;
  UpdateCachedBool(&g_state, &g_state.cached_status.sd_degraded, false);
  UpdateCachedUint32(&g_state, &g_state.cached_status.sd_fail_count, 0);
  UpdateCachedUint32(
    &g_state, &g_state.cached_status.sd_backoff_remaining_ms, 0);
  UpdateCachedBool(&g_state, &g_state.cached_status.sd_io_error_active, false);
  UpdateCachedBool(&g_state, &g_state.cached_status.fram_full, false);
  UpdateCachedBool(&g_state, &g_state.cached_status.fram_overrun_active, false);

  sd_drain_stats_t drain_stats = { 0 };
  (void)DrainFramToSdOnStartBestEffort(&g_state, &drain_stats);
  g_state.sd_was_mounted = g_state.sd_logger.is_mounted;

  SnapshotActiveSettings(&g_state);
  const app_node_role_t role = g_state.node_role_active;
  const bool is_root = (role == APP_NODE_ROLE_ROOT);
  const bool allow_children = g_state.settings.allow_children;
  const app_net_mode_t effective_net_mode = g_state.net_mode_active;
  const bool bridge_uses_serial =
    BridgeModeUsesSerial(g_state.mqtt_bridge_mode_active);
  const bool bridge_uses_broker =
    BridgeModeUsesBroker(g_state.mqtt_bridge_mode_active);
  g_state.root_publish_consumer_active =
    is_root &&
    (bridge_uses_serial || (bridge_uses_broker && g_state.mqtt_enabled_active));
  UpdateCachedBool(&g_state,
                   &g_state.cached_status.root_publish_consumer_active,
                   g_state.root_publish_consumer_active);
  g_state.root_publish_drop_no_consumer = 0;
  UpdateCachedUint32(&g_state,
                     &g_state.cached_status.root_publish_drop_no_consumer,
                     g_state.root_publish_drop_no_consumer);
  if (is_root && !g_state.root_publish_consumer_active &&
      g_state.export_outbox != NULL) {
    (void)xQueueReset(g_state.export_outbox);
  }

  if (!g_state.log_quiet) {
    printf("role=%s allow_children=%u net_mode=%s\n",
           AppSettingsRoleToString(role),
           allow_children ? 1u : 0u,
           AppSettingsNetModeToString(effective_net_mode));
  }

  if (is_root && bridge_uses_broker && !g_state.mqtt_enabled_active) {
    g_state.broker_bridge_requested_without_mqtt = true;
    if (LogRateLimitAllow(&g_state.last_broker_bridge_disabled_log_ms,
                          kExportLogRateLimitMs)) {
      ESP_LOGW(kTag,
               "Bridge mode includes broker but mqtt is disabled; broker "
               "publish will be inactive.");
    }
  }

  const bool router_disabled = AppNetConfigGetMeshDisableRouter();

  RuntimeEnableDataStreaming(true);

  if (effective_net_mode == APP_NET_MODE_MESH) {
    // Only the root should ever be configured with upstream router credentials.
    // Non-root nodes should focus on joining the Mesh-Lite network.
    const char* router_ssid = "";
    const char* router_password = "";
    if (is_root && !router_disabled) {
      wifi_credentials_t creds;
      WifiCredentialsLoad(&creds);
      if (creds.has_ssid) {
        router_ssid = creds.ssid;
        router_password = creds.password;
      }
    }

    if (!g_state.mesh_started) {
      RuntimeDiagHeapCheck(&g_state, "Wi-Fi mesh start (before)", false);
      esp_err_t wifi_result = WifiServiceAcquire(WIFI_SERVICE_MODE_MESH);
      RuntimeDiagHeapCheck(&g_state, "Wi-Fi mesh start (after)", false);
      if (wifi_result != ESP_OK) {
        ESP_LOGE(
          kTag, "Wi-Fi service start failed: %s", esp_err_to_name(wifi_result));
        g_state.mesh_started = false;
        goto mesh_start_done;
      }

      esp_err_t mesh_result =
        MeshTransportStart(&g_state.mesh,
                           is_root,
                           allow_children,
                           router_ssid,
                           router_password,
                           is_root ? &RootRecordRxCallback : NULL,
                           NULL,
                           (is_root && g_state.root_publish_consumer_active)
                             ? &RootPublishRecordRxCallback
                             : NULL,
                           NULL,
                           &g_state.time_sync);
      if (mesh_result == ESP_OK) {
        g_state.mesh_started = true;
      } else {
        ESP_LOGE(kTag, "Mesh start failed: %s", esp_err_to_name(mesh_result));
        RuntimeDiagHeapCheck(&g_state, "Wi-Fi mesh stop (start fail before)", false);
        (void)WifiServiceRelease();
        RuntimeDiagHeapCheck(&g_state, "Wi-Fi mesh stop (start fail after)", false);
        g_state.mesh_started = false;
      }
    }
  }
mesh_start_done:
  if (effective_net_mode != APP_NET_MODE_MESH) {
    RuntimeDiagHeapCheck(&g_state, "Wi-Fi direct start (before)", false);
    esp_err_t wifi_result =
      WifiServiceAcquire(WIFI_SERVICE_MODE_DIAGNOSTIC_STA);
    RuntimeDiagHeapCheck(&g_state, "Wi-Fi direct start (after)", false);
    if (wifi_result != ESP_OK) {
      ESP_LOGE(
        kTag, "Wi-Fi service start failed: %s", esp_err_to_name(wifi_result));
      g_state.wifi_direct_started = false;
      goto wifi_direct_start_done;
    }
    g_state.wifi_direct_started = true;
  }
wifi_direct_start_done:

  if (g_state.mesh_started && g_state.mesh.is_root &&
      effective_net_mode == APP_NET_MODE_MESH) {
    esp_err_t sntp_result =
      TimeSyncStartSntpAndWait(AppNetConfigGetSntpServer(), 30 * 1000);
    if (sntp_result == ESP_OK) {
      (void)TimeSyncSetRtcFromSystem(&g_state.time_sync);
    } else {
      ESP_LOGW(kTag, "SNTP sync failed: %s", esp_err_to_name(sntp_result));
    }
  }

  if (g_state.sd_logger.is_mounted) {
    const int64_t epoch_for_file =
      TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : 0;
    if (RuntimeSdIoLock(&g_state, kSdIoLockTimeoutTicks)) {
      esp_err_t sync_result = EnsureSdSyncedForEpoch(&g_state, epoch_for_file);
      if (sync_result != ESP_OK) {
        RuntimeDiagHeapCheck(&g_state, "SD unmount (initial sync before)", false);
        (void)SdLoggerUnmount(&g_state.sd_logger);
        RuntimeDiagHeapCheck(&g_state, "SD unmount (initial sync after)", false);
        MarkSdFailure(
          &g_state, "Initial SD sync failed", "sync", sync_result, 0, true);
      }
      RuntimeSdIoUnlock(&g_state);
    } else {
      ESP_LOGE(kTag, "Initial SD sync skipped due to I/O lock timeout");
    }
  }

  g_state.is_running = true;
  UpdateCachedBool(&g_state, &g_state.cached_status.runtime_running, true);

  BaseType_t sensor_created = pdPASS;
  BaseType_t storage_created = pdPASS;
  BaseType_t export_created = pdPASS;
  BaseType_t export_network_created = pdPASS;
  BaseType_t time_created = pdPASS;
  BaseType_t topology_created = pdPASS;
  BaseType_t health_publish_created = pdPASS;
  BaseType_t wifi_direct_created = pdPASS;
  BaseType_t bridge_created = pdPASS;
  BaseType_t broker_created = pdPASS;

  health_publish_created = xTaskCreate(&HealthPublisherTask,
                                       "health_pub",
                                       4096,
                                       &g_state,
                                       3,
                                       &g_state.health_publisher_task);
  if (health_publish_created != pdPASS) {
    g_state.health_publisher_task = NULL;
    ESP_LOGE(kTag, "Failed to create task health_pub");
  }

  if (effective_net_mode == APP_NET_MODE_DIRECT_WIFI) {
    wifi_direct_created = xTaskCreate(&DirectWifiTask,
                                      "wifi_direct",
                                      4096,
                                      &g_state,
                                      3,
                                      &g_state.wifi_direct_task);
    if (wifi_direct_created != pdPASS) {
      g_state.wifi_direct_task = NULL;
      ESP_LOGE(kTag, "Failed to create task wifi_direct");
    }
  }

  if (role == APP_NODE_ROLE_SENSOR) {
    sensor_created = xTaskCreate(
      &SensorTask, "sensor", 6144, &g_state, 5, &g_state.sensor_task);
    if (sensor_created != pdPASS) {
      g_state.sensor_task = NULL;
      ESP_LOGE(kTag, "Failed to create task sensor");
    }
    storage_created = xTaskCreate(
      &StorageTask, "storage", 8192, &g_state, 6, &g_state.storage_task);
    if (storage_created != pdPASS) {
      g_state.storage_task = NULL;
      ESP_LOGE(kTag, "Failed to create task storage");
    }
  }

  if (role == APP_NODE_ROLE_SENSOR || role == APP_NODE_ROLE_ROOT) {
    export_created = xTaskCreate(
      &ExportTask, "export", 6144, &g_state, 4, &g_state.export_task);
    if (export_created != pdPASS) {
      g_state.export_task = NULL;
      ESP_LOGE(kTag, "Failed to create task export");
    }
  }

  if (role != APP_NODE_ROLE_ROOT && g_state.mqtt_enabled_active) {
    export_network_created = xTaskCreate(&ExportNetworkTask,
                                         "export_net",
                                         2048,
                                         &g_state,
                                         4,
                                         &g_state.export_network_task);
    if (export_network_created != pdPASS) {
      g_state.export_network_task = NULL;
      ESP_LOGE(kTag, "Failed to create task export_net");
    }
  }

  if (role == APP_NODE_ROLE_ROOT &&
      (bridge_uses_serial ||
       (bridge_uses_broker && g_state.mqtt_enabled_active))) {
    bridge_created = xTaskCreate(
      &RootBridgeTask, "bridge", 2048, &g_state, 4, &g_state.bridge_task);
    if (bridge_created != pdPASS) {
      g_state.bridge_task = NULL;
      ESP_LOGE(kTag, "Failed to create task bridge");
    }
  }

  if (role == APP_NODE_ROLE_ROOT && g_state.mqtt_enabled_active &&
      bridge_uses_broker) {
    broker_created = xTaskCreate(&BrokerPublishTask,
                                 "broker_pub",
                                 2048,
                                 &g_state,
                                 4,
                                 &g_state.broker_task);
    if (broker_created != pdPASS) {
      g_state.broker_task = NULL;
      ESP_LOGE(kTag, "Failed to create task broker_pub");
    }
  }

  if (role == APP_NODE_ROLE_SENSOR || role == APP_NODE_ROLE_ROOT) {
    time_created = xTaskCreate(
      &TimeSyncTask, "time_sync", 2048, &g_state, 4, &g_state.time_sync_task);
    if (time_created != pdPASS) {
      g_state.time_sync_task = NULL;
      ESP_LOGE(kTag, "Failed to create task time_sync");
    }
  }

  topology_created = xTaskCreate(
    &TopologyTask, "topology", 4096, &g_state, 3, &g_state.topology_task);
  if (topology_created != pdPASS) {
    g_state.topology_task = NULL;
    ESP_LOGE(kTag, "Failed to create task topology");
  }

  BaseType_t alert_monitor_created = pdPASS;
  BaseType_t alert_sender_created = pdPASS;
  if (role == APP_NODE_ROLE_ROOT) {
    g_state.alert_monitor_context =
      (alert_task_context_t){ .manager = &g_state.alert_manager,
                              .stop_requested = &g_state.stop_requested,
                              .task_handle = &g_state.alert_monitor_task };
    g_state.alert_sender_context =
      (alert_task_context_t){ .manager = &g_state.alert_manager,
                              .stop_requested = &g_state.stop_requested,
                              .task_handle = &g_state.alert_sender_task };
    alert_monitor_created = xTaskCreate(&AlertManagerMonitorTask,
                                        "alert_mon",
                                        2048,
                                        &g_state.alert_monitor_context,
                                        3,
                                        &g_state.alert_monitor_task);
    if (alert_monitor_created != pdPASS) {
      g_state.alert_monitor_task = NULL;
      ESP_LOGE(kTag, "Failed to create task alert_mon");
    }
    alert_sender_created = xTaskCreate(&AlertManagerSenderTask,
                                       "alert_send",
                                       2048,
                                       &g_state.alert_sender_context,
                                       3,
                                       &g_state.alert_sender_task);
    if (alert_sender_created != pdPASS) {
      g_state.alert_sender_task = NULL;
      ESP_LOGE(kTag, "Failed to create task alert_send");
    }
    AlertManagerEmitRootRestart(&g_state.alert_manager,
                                esp_timer_get_time() / 1000);
  }

  if (sensor_created != pdPASS || storage_created != pdPASS ||
      export_created != pdPASS || export_network_created != pdPASS ||
      time_created != pdPASS || topology_created != pdPASS ||
      health_publish_created != pdPASS || wifi_direct_created != pdPASS ||
      bridge_created != pdPASS || broker_created != pdPASS ||
      alert_monitor_created != pdPASS || alert_sender_created != pdPASS) {
    g_state.stop_requested = true;
    g_state.is_running = false;
    UpdateCachedBool(&g_state, &g_state.cached_status.stop_requested, true);
    UpdateCachedBool(&g_state, &g_state.cached_status.runtime_running, false);
    const TickType_t wait_start = xTaskGetTickCount();
    while ((g_state.sensor_task != NULL || g_state.storage_task != NULL ||
            g_state.export_task != NULL ||
            g_state.export_network_task != NULL ||
            g_state.time_sync_task != NULL || g_state.topology_task != NULL ||
            g_state.health_publisher_task != NULL ||
            g_state.wifi_direct_task != NULL || g_state.bridge_task != NULL ||
            g_state.broker_task != NULL || g_state.alert_monitor_task != NULL ||
            g_state.alert_sender_task != NULL) &&
           (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < 1000)) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (g_state.sensor_task == NULL && g_state.storage_task == NULL &&
        g_state.export_task == NULL && g_state.export_network_task == NULL &&
        g_state.time_sync_task == NULL && g_state.topology_task == NULL &&
        g_state.health_publisher_task == NULL &&
        g_state.wifi_direct_task == NULL && g_state.bridge_task == NULL &&
        g_state.broker_task == NULL && g_state.alert_monitor_task == NULL &&
        g_state.alert_sender_task == NULL) {
      g_state.stop_requested = false;
      UpdateCachedBool(&g_state, &g_state.cached_status.stop_requested, false);
    }
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(kTag,
           "Runtime started (node=%s role=%s)",
           g_state.node_id_string,
           AppSettingsRoleToString(role));
  return ESP_OK;
}

/**
 * @brief Execute RuntimeStopSamplingOnly.
 * @param state Parameter state.
 * @return Return the function result.
 */
static esp_err_t
RuntimeStopSamplingOnly(runtime_state_t* state)
{
  if (state == NULL || !state->is_running) {
    return ESP_OK;
  }

  state->csv_header_emitted = false;
  state->root_bridge_header_emitted = false;
  state->stop_requested = true;
  RuntimeNotifyAllRunTasks(state);
  state->is_running = false;
  UpdateCachedBool(state, &state->cached_status.stop_requested, true);
  UpdateCachedBool(state, &state->cached_status.runtime_running, false);

  const TickType_t wait_start = xTaskGetTickCount();
  while ((state->sensor_task != NULL || state->storage_task != NULL ||
          state->export_task != NULL || state->export_network_task != NULL ||
          state->time_sync_task != NULL || state->topology_task != NULL ||
          state->health_publisher_task != NULL ||
          state->wifi_direct_task != NULL || state->bridge_task != NULL ||
          state->broker_task != NULL || state->alert_monitor_task != NULL ||
          state->alert_sender_task != NULL) &&
         (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < 15000)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (state->sensor_task != NULL || state->storage_task != NULL ||
      state->export_task != NULL || state->export_network_task != NULL ||
      state->time_sync_task != NULL || state->topology_task != NULL ||
      state->health_publisher_task != NULL || state->wifi_direct_task != NULL ||
      state->bridge_task != NULL || state->broker_task != NULL ||
      state->alert_monitor_task != NULL || state->alert_sender_task != NULL) {
    if (state->sensor_task != NULL) {
      ESP_LOGW(kTag, "Stop timeout: sensor_task still running (%p)", state->sensor_task);
    }
    if (state->storage_task != NULL) {
      ESP_LOGW(kTag, "Stop timeout: storage_task still running (%p)", state->storage_task);
    }
    if (state->export_task != NULL) {
      ESP_LOGW(kTag, "Stop timeout: export_task still running (%p)", state->export_task);
    }
    if (state->export_network_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: export_network_task still running (%p)",
               state->export_network_task);
    }
    if (state->time_sync_task != NULL) {
      ESP_LOGW(kTag, "Stop timeout: time_sync_task still running (%p)", state->time_sync_task);
    }
    if (state->topology_task != NULL) {
      ESP_LOGW(kTag, "Stop timeout: topology_task still running (%p)", state->topology_task);
    }
    if (state->health_publisher_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: health_publisher_task still running (%p)",
               state->health_publisher_task);
    }
    if (state->wifi_direct_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: wifi_direct_task still running (%p)",
               state->wifi_direct_task);
    }
    if (state->bridge_task != NULL) {
      ESP_LOGW(kTag, "Stop timeout: bridge_task still running (%p)", state->bridge_task);
    }
    if (state->broker_task != NULL) {
      ESP_LOGW(kTag, "Stop timeout: broker_task still running (%p)", state->broker_task);
    }
    if (state->alert_monitor_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: alert_monitor_task still running (%p)",
               state->alert_monitor_task);
    }
    if (state->alert_sender_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: alert_sender_task still running (%p)",
               state->alert_sender_task);
    }
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

/**
 * @brief Execute RuntimeStopAllTasks.
 * @param state Parameter state.
 * @return Return the function result.
 * @note FreeRTOS task entry for the RuntimeStopAllTasks task.
 */
static esp_err_t
RuntimeStopAllTasks(runtime_state_t* state)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (state->mesh_started) {
    (void)MeshTransportStop(&state->mesh);
    state->mesh_started = false;
    RuntimeDiagHeapCheck(state, "Wi-Fi mesh stop (before)", false);
    (void)WifiServiceRelease();
    RuntimeDiagHeapCheck(state, "Wi-Fi mesh stop (after)", false);
  }
  if (state->wifi_direct_started) {
    RuntimeDiagHeapCheck(state, "Wi-Fi direct stop (before)", false);
    (void)WifiServiceRelease();
    RuntimeDiagHeapCheck(state, "Wi-Fi direct stop (after)", false);
    state->wifi_direct_started = false;
  }

  if (RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
    SdLoggerClose(&state->sd_logger);
    RuntimeSdIoUnlock(state);
  }
  if (state->log_queue != NULL) {
    (void)xQueueReset(state->log_queue);
  }
  if (state->export_queue != NULL) {
    (void)xQueueReset(state->export_queue);
  }
  if (state->export_outbox != NULL) {
    (void)xQueueReset(state->export_outbox);
  }
  if (state->broker_outbox != NULL) {
    (void)xQueueReset(state->broker_outbox);
  }
  MqttClientWrapStop(&state->mqtt_client);
  UpdateMqttConnectionState(state);
  state->stop_requested = false;
  state->spi_pause_requested = false;
  state->spi_paused = false;
  UpdateCachedBool(state, &state->cached_status.stop_requested, false);

#if CONFIG_APP_MAX7219_ENABLE
  if (state->display_initialized) {
    (void)Max7219DisplayDeinit(&state->display);
    state->display_initialized = false;
  }
#endif

  if (state->sensor.is_initialized) {
    (void)Max31865ReaderDeinit(&state->sensor);
  }
  return ESP_OK;
}

/**
 * @brief Execute RuntimeStop.
 * @return Return the function result.
 */
esp_err_t
RuntimeStop(void)
{
  esp_err_t stop_result = RuntimeStopSamplingOnly(&g_state);
  esp_err_t finalize_result = RuntimeStopAllTasks(&g_state);
  return (stop_result != ESP_OK) ? stop_result : finalize_result;
}

/**
 * @brief Execute RuntimeIsRunning.
 * @return Return the function result.
 */
bool
RuntimeIsRunning(void)
{
  return g_state.is_running;
}

/**
 * @brief Execute EnterRunMode.
 * @return Return the function result.
 */
esp_err_t
EnterRunMode(void)
{
  RuntimeSetLogPolicyRun();

  esp_err_t result = RuntimeStart();
  if (result != ESP_OK) {
    RuntimeSetLogPolicyDiag();
  }
  return result;
}

/**
 * @brief Execute EnterDiagMode.
 * @return Return the function result.
 */
esp_err_t
EnterDiagMode(void)
{
  RuntimeSetLogPolicyDiag();
  RuntimeEnableDataStreaming(false);
  ESP_LOGW(kTag, "Stop: sampling halt requested");
  esp_err_t stop_result = RuntimeStopSamplingOnly(&g_state);
  bool allow_drain = (stop_result == ESP_OK);
  if (allow_drain && (g_state.sensor_task != NULL || g_state.storage_task != NULL)) {
    if (g_state.sensor_task != NULL) {
      ESP_LOGW(kTag, "Diag entry guard: sensor_task still running (%p)",
               g_state.sensor_task);
    }
    if (g_state.storage_task != NULL) {
      ESP_LOGW(kTag, "Diag entry guard: storage_task still running (%p)",
               g_state.storage_task);
    }
    allow_drain = false;
    stop_result = ESP_ERR_TIMEOUT;
  }
  esp_err_t flush_result = ESP_OK;
  if (!allow_drain) {
    ESP_LOGW(kTag,
             "Diag entry: stop timeout; skipping FRAM->SD drain/unmount to avoid SD/SPI contention");
    g_state.sd_degraded = true;
  } else {
    ESP_LOGW(kTag, "Stop: pausing SPI users before drain");
    RuntimePauseSpiUsers(&g_state, 1000u);
    ESP_LOGW(kTag, "Stop: draining FRAM->SD (and unmounting)");
    sd_drain_stats_t drain_stats = { 0 };
    const int32_t stop_drain_max_ms = ResolveStopDrainMaxMs();
    flush_result = DrainFramToSd(&g_state,
                                 true,
                                 stop_drain_max_ms,
                                 CONFIG_APP_DRAIN_MAX_RECORDS_PER_PASS,
                                 CONFIG_APP_DRAIN_YIELD_EVERY_RECORDS,
                                 &drain_stats);
    if (flush_result == ESP_ERR_TIMEOUT) {
      ESP_LOGW(kTag,
               "Stop drain timed out: remaining=%d duration=%d ms",
               drain_stats.remaining_records,
               drain_stats.duration_ms);
      RuntimeStopForceSdUnmount(&g_state, "drain timeout", &drain_stats);
    } else if (flush_result != ESP_OK) {
      ESP_LOGW(kTag,
               "Stop drain failed: %s remaining=%d",
               esp_err_to_name(flush_result),
               drain_stats.remaining_records);
    }
  }
  ESP_LOGW(kTag, "Stop: finalize");
  esp_err_t finalize_result = RuntimeStopAllTasks(&g_state);

  if (stop_result != ESP_OK) {
    return stop_result;
  }
  if (flush_result != ESP_OK && flush_result != ESP_ERR_TIMEOUT) {
    return flush_result;
  }
  return finalize_result;
}

/**
 * @brief Execute RuntimeEnableDataStreaming.
 * @param enabled Parameter enabled.
 */
void
RuntimeEnableDataStreaming(bool enabled)
{
  const bool was_enabled = g_state.data_streaming_enabled;
  if (enabled && !was_enabled) {
    g_state.data_streaming_enabled = true;
    if (DataPortInit() != ESP_OK) {
      g_state.data_streaming_enabled = false;
      return;
    }
    (void)TryEmitCsvHeader(&g_state);
    return;
  }
  g_state.data_streaming_enabled = enabled;
}

/**
 * @brief Execute RuntimeIsDataStreamingEnabled.
 * @return Return the function result.
 */
bool
RuntimeIsDataStreamingEnabled(void)
{
  return g_state.data_streaming_enabled;
}

/**
 * @brief Execute RuntimeSetLogPolicyRun.
 */
void
RuntimeSetLogPolicyRun(void)
{
  SetRunLogPolicy();
}

/**
 * @brief Execute RuntimeSetLogPolicyDiag.
 */
void
RuntimeSetLogPolicyDiag(void)
{
  SetDiagLogPolicy();
}

/**
 * @brief Execute RuntimeRequestRunStart.
 */
void
RuntimeRequestRunStart(void)
{
  taskENTER_CRITICAL(&g_state.request_lock);
  g_state.request_run_start = true;
  taskEXIT_CRITICAL(&g_state.request_lock);
}

/**
 * @brief Execute RuntimeRequestRunStop.
 */
void
RuntimeRequestRunStop(void)
{
  taskENTER_CRITICAL(&g_state.request_lock);
  g_state.request_run_stop = true;
  taskEXIT_CRITICAL(&g_state.request_lock);
}

/**
 * @brief Execute RuntimeNudgeWifiDirectTask.
 */
void
RuntimeNudgeWifiDirectTask(void)
{
  RuntimeNotifyTask(g_state.wifi_direct_task);
}

/**
 * @brief Execute RuntimeSdUnmountNow.
 * @return Return the function result.
 */
esp_err_t
RuntimeSdUnmountNow(void)
{
  esp_err_t result = ESP_ERR_TIMEOUT;
  if (RuntimeSdIoLock(&g_state, kSdIoLockTimeoutTicks)) {
    RuntimeDiagHeapCheck(&g_state, "SD unmount (manual before)", false);
    result = SdLoggerUnmount(&g_state.sd_logger);
    RuntimeDiagHeapCheck(&g_state, "SD unmount (manual after)", false);
    RuntimeSdIoUnlock(&g_state);
  }
  UpdateCachedBool(
    &g_state, &g_state.cached_status.sd_mounted, g_state.sd_logger.is_mounted);
  return result;
}

/**
 * @brief Execute RuntimeSetSdAppendFailureOnce.
 * @param enabled Parameter enabled.
 */
void
RuntimeSetSdAppendFailureOnce(bool enabled)
{
  g_state.sd_force_unmount_on_append = enabled;
}

/**
 * @brief Execute RuntimeSdIsDegraded.
 * @return Return the function result.
 */
bool
RuntimeSdIsDegraded(void)
{
  return g_state.sd_degraded;
}

/**
 * @brief Execute RuntimeSdFailCount.
 * @return Return the function result.
 */
uint32_t
RuntimeSdFailCount(void)
{
  return g_state.sd_fail_count;
}

/**
 * @brief Execute RuntimeSdBackoffUntilTicks.
 * @return Return the function result.
 */
uint32_t
RuntimeSdBackoffUntilTicks(void)
{
  return (uint32_t)g_state.sd_backoff_until_ticks;
}

/**
 * @brief Execute RuntimeGetSpiDeviceCount.
 * @return Return the function result.
 */
uint32_t
RuntimeGetSpiDeviceCount(void)
{
  uint32_t count = 0;
  if (g_state.sensor.spi_device != NULL) {
    ++count;
  }
#if CONFIG_APP_MAX7219_ENABLE
  if (g_state.display.device != NULL) {
    ++count;
  }
#endif
  if (g_state.sd_logger.card != NULL) {
    ++count;
  }
  return count;
}

/**
 * @brief Execute RuntimeAcknowledgeDisplayAttention.
 * @param item Parameter item.
 * @return Return the function result.
 */
bool
RuntimeAcknowledgeDisplayAttention(display_attention_item_t item)
{
  if (!g_state.initialized) {
    return false;
  }
  if (item != kDispAttnItemFramOvr) {
    return false;
  }
  g_state.fram_overrun_ack_total =
    FramLogGetOverrunRecordsTotal(&g_state.fram_log);
  UpdateCachedBool(&g_state,
                   &g_state.cached_status.fram_overrun_active,
                   g_state.last_overrun_records_total >
                     g_state.fram_overrun_ack_total);
  return true;
}

/**
 * @brief Execute RuntimeSetDisplayAttentionPolicy.
 * @param policy Parameter policy.
 */
void
RuntimeSetDisplayAttentionPolicy(uint32_t policy)
{
  // Keep the settings copy updated (used by diagnostics/status printing), but
  // more importantly, update the cached status so the display task and health
  // publisher see the new policy immediately.
  g_state.settings.display_attention_policy = policy;
  UpdateCachedUint32(&g_state, &g_state.cached_status.disp_attn_pol, policy);
  UpdateCachedUint32(&g_state,
                     &g_state.cached_status.disp_attn_mask,
                     g_state.settings.display_attention_mask);
}

/**
 * @brief Execute RuntimeShowDisplayTestPattern.
 * @param duration_ms Parameter duration_ms.
 * @return Return the function result.
 */
esp_err_t
RuntimeShowDisplayTestPattern(uint32_t duration_ms)
{
  if (!g_state.initialized || !g_state.display_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (duration_ms == 0) {
    duration_ms = 2000u;
  }
  g_state.display_test_start_ticks = xTaskGetTickCount();
  g_state.display_test_until_ticks =
    g_state.display_test_start_ticks + pdMS_TO_TICKS(duration_ms);
  g_state.display_test_active = true;
  return ESP_OK;
}
