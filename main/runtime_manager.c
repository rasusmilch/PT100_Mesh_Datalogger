#include "runtime_manager.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdalign.h>
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
#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_port.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "fram_error_log.h"
#include "fram_i2c.h"
#include "fram_layout.h"
#include "fram_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gpio_buttons.h"
#include "heap_event_log.h"
#include "heap_monitor.h"
#include "heap_phase_log.h"
#include "i2c_bus.h"
#include "log_rate_limit.h"
#include "max31865_reader.h"
#include "max7219_display.h"
#include "mem_guard.h"
#include "mem_pool.h"
#include "mesh_transport.h"
#include "mqtt_client_wrap.h"
#include "net_supervisor.h"
#include "ptlog_format.h"
#include "run_gpio.h"
#include "runtime_health.h"
#include "runtime_health_publisher.h"
#include "runtime_markers.h"
#include "runtime_state.h"
#include "sd_logger.h"
#include "sdkconfig.h"
#include "stack_monitor.h"
#include "time_civil.h"
#include "time_sync.h"
#include "units_gpio.h"
#include "wifi_credentials.h"
#include "wifi_manager.h"
#include "wifi_service.h"

static const char* kTag = "runtime";

// String used to represent "no holder" for mutex/I2C operation state.
// This must be a compile-time constant so it can be used in static
// initializers.
static const char kMutexHolderNone[] = "<none>";

static const uint32_t kSdFlushMaxRecordsPerPass = 100;
static const uint32_t kSdFlushMaxMsPerPass = 50;
static const uint32_t kSdFlushTimeSliceMs = 50;
static const uint32_t kSdFlushWarnIntervalMs = 10000;
static const uint32_t kFramLogLockWarnIntervalMs = 10000;
static const uint32_t kI2cTimeoutWarnIntervalMs = 5000;
static const uint32_t kI2cLockDumpIntervalMs = 60000;
static const uint32_t kI2cHangWarnMs = 2000;
static const uint32_t kI2cHangRestartMs = 8000;
static const uint32_t kSdFlushFailureBackoffMs = 5000;
static const uint32_t kSdFlushFailureBackoffMaxMs = 300000;
static const uint32_t kSdFlushMinIntervalMs = 200;
static const uint32_t kSdDetectPollIntervalMs = 250;
static const uint32_t kExportQueueDepth = 64;
static const uint32_t kExportOutboxDepth = CONFIG_APP_EXPORT_OUTBOX_DEPTH;
static const uint32_t kSdIoErrorSetThreshold = 2;
static const uint32_t kSdIoErrorClearThreshold = 2;
static const uint32_t kFramOverrunActiveSetThreshold = 2;
static const uint32_t kFramOverrunActiveClearThreshold = 2;
static const int64_t kCalTimeStableDelaySec = 60;
static const uint32_t kBrokerOutboxDepth = CONFIG_APP_BROKER_OUTBOX_DEPTH;
static const uint32_t kExportLogRateLimitMs = CONFIG_APP_EXPORT_RATE_LIMIT_MS;
static const uint32_t kAlertHttpSendFailLogRateLimitMs = 60000;
static const uint32_t kAlertHttpRetryDelayMs = 1000;
static const uint32_t kAlertHttpMaxAttempts = 3;
static const TickType_t kSdIoLockTimeoutTicks = pdMS_TO_TICKS(2000);
static const TickType_t kSensorSpiBusLockTimeoutTicks = pdMS_TO_TICKS(20);
static const TickType_t kFramLogLockTimeoutTicks = pdMS_TO_TICKS(2000);
static const TickType_t kI2cIoLockTimeoutTicks = pdMS_TO_TICKS(150);
static const TickType_t kI2cRecoveryLockTimeoutTicks = pdMS_TO_TICKS(2000);
static const uint32_t kSensorSpiInvalidStreakThreshold = 3;
static const uint32_t kSensorSpiReinitAlertThreshold = 2;
static const int64_t kSensorSpiReinitWindowMs = 60000;
static const int64_t kSensorSpiStableResolveMs = 30000;
static const uint16_t kErrlogSensorSpiFault = 17;
static const int32_t kStopDrainHardMaxDefaultMs = 15000;
static const int64_t kNetTxStallDropMs = 30000;
static const uint32_t kI2cRecoveryTriggerCount = 3;
static const int64_t kFramRetryIntervalMs = 30000;
static const uint32_t kFramInvalidHeapLogEvery = 10;
static const uint32_t kDeferredDropWarnIntervalMs = 60000;
static const uint32_t kFramSuppressedWarnIntervalMs = 10000;
static const uint32_t kRebootAlertLatchMagic = 0x5254424C;
static const uint16_t kRebootAlertLatchVersion = 2;
static const uint32_t kRtcErrlogLatchMagic = 0x5245434C;
static const uint16_t kRtcErrlogLatchVersion = 1;
static const uint32_t kRebootAlertWifiWaitMs = 2500;
static const uint32_t kRebootAlertHttpTimeoutMs = 1500;
static const uint32_t kRebootAlertResolveDelayMs = 5000;
static const uint32_t kRebootAlertResolveCheckIntervalMs = 1000;
static const uint32_t kSafeHoldMaxMs = 120000;
static const uint32_t kSafeHoldMinRetryMs = 2000;
static const uint32_t kSafeHoldMaxRetryMs = 60000;
static const uint32_t kSafeHoldLogIntervalMs = 5000;
static const uint32_t kRtdFaultNtfyBatchHoldoffMs = 30000;
static const uint32_t kRtdFaultNtfyMinResendIntervalMs = 300000;
static const uint32_t kRtdFaultNtfyPendingThreshold = 5;
static const uint32_t kStorageStallThresholdMs =
  CONFIG_APP_ALERT_STORAGE_STALL_MS;
static char g_sd_csv_line_buffer[CONFIG_APP_MAX_CSV_LINE_BYTES];
static stack_monitor_t g_stack_monitor;
static runtime_state_t g_state;
static app_runtime_t g_runtime;
static runtime_sensor_sample_t g_runtime_sensor_sample;
static bool g_alert_http_psram_failure_logged = false;
static RTC_DATA_ATTR runtime_reboot_alert_latch_t g_reboot_alert_latch = {
  .magic = kRebootAlertLatchMagic,
  .version = kRebootAlertLatchVersion,
};
static RTC_DATA_ATTR runtime_rtc_errlog_latch_t g_rtc_errlog_latch;

enum
{
  SD_FLUSH_TRIGGER_WATERMARK = 1u << 0,
  SD_FLUSH_TRIGGER_PERIODIC = 1u << 1,
  SD_FLUSH_TRIGGER_BACKLOG = 1u << 2,
  SD_FLUSH_TRIGGER_RETRY = 1u << 3,
  SD_FLUSH_TRIGGER_MORE_PENDING = 1u << 4,
  SD_FLUSH_TRIGGER_MANUAL = 1u << 5,
};

typedef struct
{
  bool active;
  const char* op_name;
  uint32_t address;
  size_t length;
  TickType_t start_ticks;
  const char* caller;
} runtime_i2c_op_state_t;

static portMUX_TYPE g_i2c_op_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_rtc_errlog_latch_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_runtime_sensor_sample_lock = portMUX_INITIALIZER_UNLOCKED;
static runtime_i2c_op_state_t g_i2c_op_state = {
  .active = false,
  .op_name = kMutexHolderNone,
  .address = 0,
  .length = 0,
  .start_ticks = 0,
  .caller = kMutexHolderNone,
};

enum
{
  ERROR_FRAM_IO_FAIL = 100,
  ERROR_I2C_RECOVERY_START = 101,
  ERROR_I2C_RECOVERY_SUCCESS = 102,
  ERROR_I2C_RECOVERY_FAILED = 103,
  ERROR_I2C_HANG_RESTART = 104,
};

enum
{
  SPI_PAUSE_ACK_SENSOR = 1u << 0,
  SPI_PAUSE_ACK_DISPLAY = 1u << 1,
};

static esp_err_t
InitSpiBus(spi_host_device_t host);

static esp_err_t
InitializeMax7219Display(runtime_state_t* state);

static void
UpdateCalibrationDueState(runtime_state_t* state,
                          bool time_valid,
                          int64_t now_utc);

static spi_host_device_t
GetSpiHost(void);

static esp_err_t
InitializeMax31865Sensor(runtime_state_t* state, spi_host_device_t spi_host);

static void
AlertHttpTask(void* context);

static void
RuntimeLogNtfyBodyLines(const char* label, const char* text);

static void
RuntimeLogNtfyJob(const alert_ntfy_job_t* job, const char* stage);

static void
AppendStopStorageSummaryLine(runtime_state_t* state, alert_ntfy_job_t* job);

static void
RuntimeResetStorageStallTracking(runtime_state_t* state, int64_t now_ms);

static bool
RuntimeComputeStorageStallCondition(runtime_state_t* state, int64_t now_ms);

static bool
RuntimeRecoverI2cBus(runtime_state_t* state,
                     const char* reason,
                     esp_err_t last_error);

/**
 * @brief Compute effective sensor polling period for sampling/display updates.
 * @param settings Current app settings snapshot.
 * @return Effective polling period in milliseconds.
 */
static uint32_t
RuntimeEffectiveSensorPollPeriodMs(const app_settings_t* settings);

/**
 * @brief Request net_tx + alert_http pause state and wait for acknowledgements.
 * @param state Runtime state.
 * @param pause_enabled True to request pause, false to resume.
 * @param timeout_ms Maximum wait for acknowledgement.
 */
static void
RuntimeRequestNetPause(runtime_state_t* state,
                       bool pause_enabled,
                       uint32_t timeout_ms);

/**
 * @brief Create the alert_http task with a PSRAM-backed stack.
 * @param state Runtime state (owns the task memory).
 * @param stack_bytes Stack size in bytes.
 * @return pdPASS on success, or pdFAIL on failure.
 */
static BaseType_t
CreateAlertHttpTaskWithPsrStack(runtime_state_t* state, uint32_t stack_bytes);

static void
RootRecordRxCallback(const pt100_mesh_addr_t* from,
                     const log_record_t* record,
                     void* context);
static void
RootPublishRecordRxCallback(const uint8_t src_mac[6],
                            const log_record_t* record,
                            void* context);

static uint64_t
PackMacToLeafId(const uint8_t mac[6]);

static sd_append_verify_t
ResolveSdVerifyMode(const runtime_state_t* state);

static bool
RuntimeBuildPtlogHeaderInternal(const runtime_state_t* state,
                                int64_t epoch_utc,
                                ptlog_header_t* header_out,
                                uint32_t* signature_out);

static bool
RuntimeCalibrationApplied(const runtime_state_t* state);

static void
FormatUtcTimestamp(int64_t epoch_utc, char* out, size_t out_size);

static bool
RuntimeFramLogLock(runtime_state_t* state, TickType_t timeout_ticks);

static void
RuntimeFramLogUnlock(runtime_state_t* state);

static bool
RuntimeVerifyFram(runtime_state_t* state);

static void
UpdateDebouncedRtdFault(runtime_state_t* state,
                        const app_settings_t* settings,
                        bool raw_fault_present,
                        bool sd_flush_busy,
                        int64_t now_ms,
                        uint8_t raw_fault_status);

/**
 * @brief Perform one MAX31865 one-shot conversion/read cycle.
 * @param state Runtime state owning the MAX31865 reader.
 * @param sample_out Output sample buffer.
 * @param one_shot_out Optional one-shot state snapshot for diagnostics.
 * @return ESP_OK on read completion, or an ESP_ERR_* code on failure.
 */
static esp_err_t
RuntimePerformSingleMax31865Read(runtime_state_t* state,
                                 max31865_sample_t* sample_out,
                                 max31865_one_shot_state_t* one_shot_out);

/**
 * @brief Perform MAX31865 one-shot read with immediate fault confirmation
 * retry.
 * @param state Runtime state owning counters and MAX31865 reader.
 * @param sample_out Authoritative sample for this sensor loop.
 * @param used_retry_out True when a same-loop retry was attempted.
 * @param retry_faulted_out True when retry completed with a faulted sample.
 * @param first_fault_status_out Fault status from the first read fault.
 * @param retry_fault_status_out Fault status from the retry read, if any.
 * @param retry_result_out Retry read result (ESP_OK or hard error).
 * @return Result of the authoritative sample path.
 */
static esp_err_t
RuntimePerformMax31865ReadWithFaultRetry(runtime_state_t* state,
                                         max31865_sample_t* sample_out,
                                         bool* used_retry_out,
                                         bool* retry_faulted_out,
                                         uint8_t* first_fault_status_out,
                                         uint8_t* retry_fault_status_out,
                                         esp_err_t* retry_result_out);

/**
 * @brief Accumulate RTD retry diagnostics for rate-limited ntfy summary jobs.
 */
static void
RuntimeAccumulateRtdFaultNtfy(runtime_state_t* state,
                              bool first_fault,
                              bool retry_fault,
                              bool retry_clean,
                              uint8_t first_status,
                              uint8_t retry_status,
                              esp_err_t retry_err,
                              int64_t now_ms);

/**
 * @brief Check whether pending RTD retry diagnostics should be sent now.
 */
static bool
RuntimeShouldSendRtdFaultNtfy(const runtime_state_t* state, int64_t now_ms);

/**
 * @brief Build a custom ntfy summary job from pending RTD retry diagnostics.
 */
static bool
RuntimeBuildRtdFaultSummaryNtfyJob(runtime_state_t* state,
                                   int64_t now_ms,
                                   alert_ntfy_job_t* out_job);

/**
 * @brief Try to enqueue a pending RTD retry diagnostic ntfy summary job.
 */
static void
RuntimeMaybeSendRtdFaultSummaryNtfy(runtime_state_t* state, int64_t now_ms);

/**
 * @brief Apply EMA settings to a freshly captured RTD sample.
 * @param state Runtime state containing MAX31865 EMA storage.
 * @param ema_enabled True when EMA filtering is enabled.
 * @param alpha_permille EMA alpha in permille units [1, 1000].
 * @param sample Sample to fold into EMA state.
 * @param result Capture result associated with @p sample.
 */
static void
UpdateSensorEmaFromSample(runtime_state_t* state,
                          bool ema_enabled,
                          uint16_t alpha_permille,
                          const max31865_sample_t* sample,
                          esp_err_t result);

/**
 * @brief Publish latest runtime sensor snapshot from SensorTask.
 * @param sample Source MAX31865 sample.
 */
static void
RuntimePublishSensorSnapshot(const max31865_sample_t* sample);

static const char*
Max31865OneShotPhaseToString(max31865_one_shot_phase_t phase);

/**
 * @brief Execute RuntimeRecoverI2cBusCommon.
 * @param state Parameter state.
 * @param reason Parameter reason.
 * @param last_error Parameter last_error.
 * @param lock_held Parameter lock_held.
 * @return Return the function result.
 */
static bool
RuntimeRecoverI2cBusCommon(runtime_state_t* state,
                           const char* reason,
                           esp_err_t last_error,
                           bool lock_held);

static void
RuntimeRebootOnSdEioEscalation(runtime_state_t* state, const char* context);

/**
 * @brief Enqueue an ntfy job and optionally wait for AlertHttpTask completion.
 * @param state Runtime state.
 * @param job Ntfy job to enqueue.
 * @param wait_ms Maximum time to wait for send counters to change.
 * @param status_out Optional pointer receiving last HTTP status.
 * @param err_out Optional pointer receiving last esp_err_t.
 * @param retry_after_seconds_out Optional pointer receiving cooldown-derived
 * retry seconds.
 * @return Result indicating enqueue/wait/send outcome.
 */
static alert_ntfy_result_t
RuntimeEnqueueNtfyJobAndWait(runtime_state_t* state,
                             const alert_ntfy_job_t* job,
                             uint32_t wait_ms,
                             int* status_out,
                             esp_err_t* err_out,
                             int* retry_after_seconds_out);

static void
RuntimeDumpLocks(runtime_state_t* state, const char* reason);

static void
RuntimeRecordError(runtime_state_t* state,
                   const char* module,
                   esp_err_t err,
                   const char* phase);

static bool
IsRecoverableI2cErr(esp_err_t err);

/**
 * @brief Execute DiscardFramRecordsWithYield.
 * @param state Parameter state.
 * @param records_to_discard Parameter records_to_discard.
 * @param fram_log_timeout_ticks Parameter fram_log_timeout_ticks.
 * @param timeout_counter Parameter timeout_counter.
 * @param last_log_ms Parameter last_log_ms.
 * @param context Parameter context.
 * @return Return the function result.
 */
static esp_err_t
DiscardFramRecordsWithYield(runtime_state_t* state,
                            uint32_t records_to_discard,
                            TickType_t fram_log_timeout_ticks,
                            uint32_t* timeout_counter,
                            uint32_t* last_log_ms,
                            const char* context);

static void
UpdateFramFillState(runtime_state_t* state);

static void
UpdateCachedBool(runtime_state_t* state, bool* field, bool value);

static void
RuntimeLatchOperatorHold(runtime_state_t* state, uint32_t now_ms);

static void
RuntimeClearOperatorHold(runtime_state_t* state);

static void
UpdateCachedUint64(runtime_state_t* state, uint64_t* field, uint64_t value);

static bool
DeferredLogPush(runtime_state_t* state, const log_record_t* record);

static bool
DeferredLogPop(runtime_state_t* state, log_record_t* out);

static uint8_t
DeferredLogCount(runtime_state_t* state);

static uint32_t
DeferredLogDrops(runtime_state_t* state);

static void
DeferredLogReset(runtime_state_t* state);

static void
RuntimeBeginI2cQuiesce(runtime_state_t* state, const char* reason);

static void
RuntimeEndI2cQuiesce(runtime_state_t* state);

static esp_err_t
RuntimeDrainDeferredToFram(runtime_state_t* state, const char* context);

static void
RuntimeAssignFallbackRecordIds(runtime_state_t* state, log_record_t* record);

static void
RuntimeFramRetryTick(runtime_state_t* state, int64_t now_ms);

static void
RestoreTimeJumpBackPendingFromFram(runtime_state_t* state);

static esp_err_t
LogFramErrorEvent(runtime_state_t* state,
                  uint16_t code,
                  bool resolved,
                  int32_t detail0,
                  int32_t detail1);

static void
RuntimeRtcErrlogLatchInitIfNeeded(void);

static void
RuntimeRtcErrlogLatchPush(uint16_t code,
                          bool resolved,
                          int32_t detail0,
                          int32_t detail1,
                          uint32_t epoch_sec,
                          uint16_t millis);

static void
RuntimeRtcErrlogLatchFlushToFram(runtime_state_t* state);

static uint32_t
GetStackMonitorMinBytes(const char* name);

static void
LogDrainPreflight(const runtime_state_t* state, const char* reason);

static void
LogDrainPostflight(const runtime_state_t* state,
                   const char* reason,
                   int32_t flushed_records,
                   int32_t remaining_records,
                   uint32_t duration_ms);

/**
 * @brief Build a printable SD flush trigger list.
 * @param trigger_flags Bitset of SD flush trigger flags.
 * @param buffer Destination buffer for the printable trigger list.
 * @param buffer_len Size of @p buffer in bytes.
 */
static void
SdFlushTriggerFlagsToString(uint32_t trigger_flags,
                            char* buffer,
                            size_t buffer_len);

/**
 * @brief Start an SD flush session if one is not active.
 * @param state Runtime state.
 */
static void
SdFlushMaybeStartSession(runtime_state_t* state);

/**
 * @brief Log end-of-session drain metrics.
 * @param state Runtime state.
 * @param records_flushed Number of records flushed in the session.
 * @param duration_ms Session duration in milliseconds.
 */
static void
SdFlushLogSessionEnd(runtime_state_t* state,
                     uint32_t records_flushed,
                     uint32_t duration_ms);

static uint32_t
RuntimeI2cLockHeldMsNoLock(const runtime_state_t* state);

static void
RuntimeDumpI2cLockState(runtime_state_t* state, const char* reason);

static void
RuntimeI2cOpBegin(const char* op_name,
                  uint32_t address,
                  size_t length,
                  const char* caller);

static void
RuntimeI2cOpEnd(void);

static void
RuntimeDumpI2cOpState(const runtime_state_t* state, const char* reason);

static bool
ReadFramBufferedRecords(runtime_state_t* state, uint32_t* buffered_out);

static void
RuntimeNotifyTask(TaskHandle_t handle)
{
  if (handle == NULL) {
    return;
  }
  xTaskNotifyGive(handle);
}

static bool
RuntimeFramLogLockWithWarn(runtime_state_t* state,
                           TickType_t timeout_ticks,
                           uint32_t* timeout_counter,
                           uint32_t* last_log_ms,
                           const char* context)
{
  if (RuntimeFramLogLock(state, timeout_ticks)) {
    return true;
  }
  if (timeout_counter != NULL) {
    (*timeout_counter)++;
  }
  if (LogRateLimitAllow(last_log_ms, kFramLogLockWarnIntervalMs)) {
    ESP_LOGW(kTag, "FRAM log lock timeout in %s", context);
    RuntimeDumpI2cLockState(state, "fram_log_lock_timeout");
    RuntimeDumpI2cOpState(state, "fram_log_lock_timeout");
    I2cBusLogState(&state->i2c_bus, "fram_log_lock_timeout");
  }
  return false;
}

static void
RegisterStackMonitorTask(const char* name,
                         TaskHandle_t* handle_ptr,
                         uint32_t stack_alloc_bytes)
{
  if (!StackMonitorRegister(
        &g_stack_monitor, name, handle_ptr, stack_alloc_bytes)) {
    ESP_LOGW(kTag, "Stack monitor registration failed; skipping %s", name);
  }
}

/**
 * @brief Update EMA state with an already-read MAX31865 sample.
 * @param state Runtime state containing sensor EMA fields.
 * @param ema_enabled True when EMA filtering is enabled.
 * @param alpha_permille EMA alpha in permille units [1, 1000].
 * @param sample Sample captured in the current sensor loop iteration.
 * @param result Capture status associated with @p sample.
 */
static void
UpdateSensorEmaFromSample(runtime_state_t* state,
                          bool ema_enabled,
                          uint16_t alpha_permille,
                          const max31865_sample_t* sample,
                          esp_err_t result)
{
  if (state == NULL || sample == NULL || !ema_enabled || result != ESP_OK) {
    return;
  }

  const double alpha = (double)alpha_permille / 1000.0;
  (void)Max31865ApplyEmaSample(&state->sensor, alpha, sample, NULL);
}

static void
RuntimePublishSensorSnapshot(const max31865_sample_t* sample)
{
  if (sample == NULL) {
    return;
  }

  taskENTER_CRITICAL(&g_runtime_sensor_sample_lock);
  g_runtime_sensor_sample.timestamp_us = esp_timer_get_time();
  g_runtime_sensor_sample.temp_mC =
    (int32_t)llround(sample->temperature_c * 1000.0);
  g_runtime_sensor_sample.ohm_mohm =
    (int32_t)llround(sample->resistance_ohm * 1000.0);
  g_runtime_sensor_sample.fault = sample->fault_status;
  g_runtime_sensor_sample.valid = !sample->fault_present;
  g_runtime_sensor_sample.sample_id++;
  taskEXIT_CRITICAL(&g_runtime_sensor_sample_lock);
}

static const char*
Max31865OneShotPhaseToString(max31865_one_shot_phase_t phase)
{
  switch (phase) {
    case kMax31865OneShotIdle:
      return "Idle";
    case kMax31865OneShotBiasSettling:
      return "BiasSettling";
    case kMax31865OneShotConverting:
      return "Converting";
    default:
      return "Unknown";
  }
}

static uint32_t
GetStackMonitorMinBytes(const char* name)
{
  uint32_t min_bytes = 0;
  if (!StackMonitorGetMinFreeBytes(&g_stack_monitor, name, &min_bytes)) {
    return 0;
  }
  return min_bytes;
}

static const char*
RuntimeGetCurrentTaskName(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
    return "scheduler-not-started";
  }
  const char* name = pcTaskGetName(NULL);
  return (name != NULL) ? name : "unknown";
}

static uint32_t
RuntimeElapsedMs(TickType_t start_ticks, TickType_t now_ticks)
{
  if (start_ticks == 0) {
    return 0;
  }
  return (uint32_t)pdTICKS_TO_MS(now_ticks - start_ticks);
}

static void
RuntimeUpdateHoldMax(uint32_t hold_ms, uint32_t* max_observed_ms)
{
  if (max_observed_ms == NULL) {
    return;
  }
  if (hold_ms > *max_observed_ms) {
    *max_observed_ms = hold_ms;
  }
}

static void
RuntimeRecordError(runtime_state_t* state,
                   const char* module,
                   esp_err_t err,
                   const char* phase)
{
  if (state == NULL) {
    return;
  }
  runtime_error_entry_t* entry = &state->error_ring[state->error_ring_head];
  entry->uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
  entry->module = (module != NULL) ? module : "unknown";
  entry->err = err;
  entry->phase = (phase != NULL) ? phase : kMutexHolderNone;

  state->error_ring_head =
    (uint8_t)((state->error_ring_head + 1u) % RUNTIME_ERROR_RING_SIZE);
  if (state->error_ring_count < RUNTIME_ERROR_RING_SIZE) {
    state->error_ring_count++;
  }
}

/**
 * @brief Return I2C lock hold duration without taking locks.
 * @param state Parameter state.
 * @return Return the function result.
 */
static uint32_t
RuntimeI2cLockHeldMsNoLock(const runtime_state_t* state)
{
  if (state == NULL) {
    return 0;
  }
  if (state->i2c_mutex_holder == kMutexHolderNone ||
      state->i2c_mutex_hold_start_ticks == 0) {
    return 0;
  }
  return RuntimeElapsedMs(state->i2c_mutex_hold_start_ticks,
                          xTaskGetTickCount());
}

/**
 * @brief Log I2C lock state for debugging.
 * @param state Parameter state.
 * @param reason Parameter reason.
 */
static void
RuntimeDumpI2cLockState(runtime_state_t* state, const char* reason)
{
  if (state == NULL) {
    return;
  }
  const TickType_t now_ticks = xTaskGetTickCount();
  const char* holder = (state->i2c_mutex_holder != NULL)
                         ? state->i2c_mutex_holder
                         : kMutexHolderNone;
  const uint32_t held_ms =
    (holder != kMutexHolderNone)
      ? RuntimeElapsedMs(state->i2c_mutex_hold_start_ticks, now_ticks)
      : 0u;
  const char* phase =
    (state->sd_flush_phase != NULL) ? state->sd_flush_phase : kMutexHolderNone;
  const char* reason_label = (reason != NULL) ? reason : "unknown";
  ESP_LOGW(kTag,
           "I2C lock (%s): holder=%s held_ms=%" PRIu32 " timeouts=%" PRIu32
           " max_ms=%" PRIu32 " sd_phase=%s last_err=%s (%d)",
           reason_label,
           holder,
           held_ms,
           state->i2c_mutex_timeouts,
           state->i2c_mutex_hold_max_ms_observed,
           phase,
           esp_err_to_name(state->sd_flush_last_err),
           (int)state->sd_flush_last_err);
}

/**
 * @brief Begin tracking an I2C operation.
 * @param op_name Parameter op_name.
 * @param address Parameter address.
 * @param length Parameter length.
 * @param caller Parameter caller.
 */
static void
RuntimeI2cOpBegin(const char* op_name,
                  uint32_t address,
                  size_t length,
                  const char* caller)
{
  taskENTER_CRITICAL(&g_i2c_op_lock);
  g_i2c_op_state.active = true;
  g_i2c_op_state.op_name = (op_name != NULL) ? op_name : kMutexHolderNone;
  g_i2c_op_state.address = address;
  g_i2c_op_state.length = length;
  g_i2c_op_state.start_ticks = xTaskGetTickCount();
  g_i2c_op_state.caller = (caller != NULL) ? caller : kMutexHolderNone;
  taskEXIT_CRITICAL(&g_i2c_op_lock);
}

/**
 * @brief End tracking an I2C operation.
 */
static void
RuntimeI2cOpEnd(void)
{
  taskENTER_CRITICAL(&g_i2c_op_lock);
  g_i2c_op_state.active = false;
  g_i2c_op_state.op_name = kMutexHolderNone;
  g_i2c_op_state.address = 0;
  g_i2c_op_state.length = 0;
  g_i2c_op_state.start_ticks = 0;
  g_i2c_op_state.caller = kMutexHolderNone;
  taskEXIT_CRITICAL(&g_i2c_op_lock);
}

/**
 * @brief Log current I2C operation state.
 * @param state Parameter state.
 * @param reason Parameter reason.
 */
static void
RuntimeDumpI2cOpState(const runtime_state_t* state, const char* reason)
{
  runtime_i2c_op_state_t snapshot;
  taskENTER_CRITICAL(&g_i2c_op_lock);
  snapshot = g_i2c_op_state;
  taskEXIT_CRITICAL(&g_i2c_op_lock);

  const TickType_t now_ticks = xTaskGetTickCount();
  const char* holder = kMutexHolderNone;
  const uint32_t held_ms =
    (state != NULL && state->i2c_mutex_holder != NULL &&
     state->i2c_mutex_holder != kMutexHolderNone)
      ? RuntimeElapsedMs(state->i2c_mutex_hold_start_ticks, now_ticks)
      : 0u;
  if (state != NULL) {
    holder = (state->i2c_mutex_holder != NULL) ? state->i2c_mutex_holder
                                               : kMutexHolderNone;
  }
  const char* phase = (state != NULL && state->sd_flush_phase != NULL)
                        ? state->sd_flush_phase
                        : kMutexHolderNone;
  const char* reason_label = (reason != NULL) ? reason : "unknown";
  const uint32_t op_ms = (snapshot.active && snapshot.start_ticks != 0)
                           ? RuntimeElapsedMs(snapshot.start_ticks, now_ticks)
                           : 0u;
  ESP_LOGW(kTag,
           "I2C op (%s): lock_holder=%s held_ms=%" PRIu32
           " op=%s addr=0x%08" PRIx32 " len=%zu op_ms=%" PRIu32
           " caller=%s sd_phase=%s last_err=%s (%d)",
           reason_label,
           holder,
           held_ms,
           snapshot.active ? snapshot.op_name : kMutexHolderNone,
           snapshot.address,
           snapshot.length,
           op_ms,
           snapshot.active ? snapshot.caller : kMutexHolderNone,
           phase,
           (state != NULL) ? esp_err_to_name(state->sd_flush_last_err)
                           : kMutexHolderNone,
           (state != NULL) ? (int)state->sd_flush_last_err : 0);

  if (state != NULL) {
    I2cBusLogState(&state->i2c_bus, reason_label);
  }
}

static void
RuntimeDumpLocks(runtime_state_t* state, const char* reason)
{
  if (state == NULL) {
    return;
  }
  const TickType_t now_ticks = xTaskGetTickCount();
  const char* fram_holder = (state->fram_log_mutex_holder != NULL)
                              ? state->fram_log_mutex_holder
                              : kMutexHolderNone;
  const char* i2c_holder = (state->i2c_mutex_holder != NULL)
                             ? state->i2c_mutex_holder
                             : kMutexHolderNone;
  const char* sd_holder = (state->sd_io_mutex_holder != NULL)
                            ? state->sd_io_mutex_holder
                            : kMutexHolderNone;
  const uint32_t fram_held_ms =
    (fram_holder != kMutexHolderNone)
      ? RuntimeElapsedMs(state->fram_log_mutex_hold_start_ticks, now_ticks)
      : 0u;
  const uint32_t i2c_held_ms =
    (i2c_holder != kMutexHolderNone)
      ? RuntimeElapsedMs(state->i2c_mutex_hold_start_ticks, now_ticks)
      : 0u;
  const uint32_t sd_held_ms =
    (sd_holder != kMutexHolderNone)
      ? RuntimeElapsedMs(state->sd_io_mutex_hold_start_ticks, now_ticks)
      : 0u;
  const char* phase =
    (state->sd_flush_phase != NULL) ? state->sd_flush_phase : kMutexHolderNone;
  const char* reason_label = (reason != NULL) ? reason : "unknown";

  ESP_LOGW(kTag, "Lock dump (%s):", reason_label);
  ESP_LOGW(kTag,
           "  fram_log holder=%s held_ms=%" PRIu32 " timeouts=%" PRIu32
           " max_ms=%" PRIu32,
           fram_holder,
           fram_held_ms,
           state->fram_log_mutex_timeouts,
           state->fram_log_mutex_hold_max_ms_observed);
  ESP_LOGW(kTag,
           "  i2c holder=%s held_ms=%" PRIu32 " timeouts=%" PRIu32
           " max_ms=%" PRIu32,
           i2c_holder,
           i2c_held_ms,
           state->i2c_mutex_timeouts,
           state->i2c_mutex_hold_max_ms_observed);
  ESP_LOGW(kTag,
           "  sd_io holder=%s held_ms=%" PRIu32 " timeouts=%" PRIu32
           " max_ms=%" PRIu32,
           sd_holder,
           sd_held_ms,
           state->sd_io_mutex_timeouts,
           state->sd_io_mutex_hold_max_ms_observed);
  ESP_LOGW(kTag,
           "  sd_flush phase=%s last_err=%s (%d) i2c_errs=%" PRIu32
           " sd_errs=%" PRIu32,
           phase,
           esp_err_to_name(state->sd_flush_last_err),
           (int)state->sd_flush_last_err,
           state->sd_flush_i2c_errs,
           state->sd_flush_sd_errs);

  if (state->error_ring_count > 0u) {
    const uint8_t count = state->error_ring_count;
    const uint8_t start =
      (uint8_t)((state->error_ring_head + RUNTIME_ERROR_RING_SIZE - count) %
                RUNTIME_ERROR_RING_SIZE);
    for (uint8_t i = 0; i < count; ++i) {
      const uint8_t idx = (uint8_t)((start + i) % RUNTIME_ERROR_RING_SIZE);
      const runtime_error_entry_t* entry = &state->error_ring[idx];
      const char* err_phase =
        (entry->phase != NULL) ? entry->phase : kMutexHolderNone;
      ESP_LOGW(kTag,
               "  err[%u] uptime_ms=%" PRIu32 " module=%s err=%s (%d) phase=%s",
               (unsigned)i,
               entry->uptime_ms,
               (entry->module != NULL) ? entry->module : "unknown",
               esp_err_to_name(entry->err),
               (int)entry->err,
               err_phase);
    }
  }
}

void
RuntimeDumpLocksManual(const char* reason)
{
  RuntimeDumpLocks(&g_state, reason);
}

/**
 * @brief Dump current I2C operation diagnostics to log output.
 * @param reason Reason string for the dump.
 */
void
RuntimeDumpI2cOpStateManual(const char* reason)
{
  RuntimeDumpI2cOpState(&g_state, reason);
}

static bool
IsRecoverableI2cErr(esp_err_t err)
{
  return err == ESP_ERR_TIMEOUT || err == ESP_FAIL ||
         err == ESP_ERR_INVALID_STATE || err == ESP_ERR_INVALID_RESPONSE;
}

static void
LogDrainPreflight(const runtime_state_t* state, const char* reason)
{
  if (state == NULL) {
    return;
  }

  StackMonitorMaybeSample(&g_stack_monitor);
  const runtime_cached_status_t* cached = &state->cached_status;
  const uint32_t fram_count = cached->fram_count;
  const uint32_t fram_capacity = cached->fram_capacity;
  const uint32_t watermark = cached->fram_flush_watermark_records;
  const uint32_t fram_pct =
    (fram_capacity > 0u) ? (fram_count * 100u) / fram_capacity : 0u;

  const char* reason_label = (reason != NULL) ? reason : "unknown";

  ESP_LOGI(kTag,
           "Drain preflight(%s): fram=%" PRIu32 "/%" PRIu32 " wm=%" PRIu32
           " (%" PRIu32 "%%) sd=mounted:%u degraded:%u backoff:%" PRIu32
           "ms heap=int:%" PRIu32 "/%" PRIu32 " psram:%" PRIu32 "/%" PRIu32
           " stack_min(ctrl:%" PRIu32 " stor:%" PRIu32 " flush:%" PRIu32
           " net:%" PRIu32 " http:%" PRIu32 ")",
           reason_label,
           fram_count,
           fram_capacity,
           watermark,
           fram_pct,
           cached->sd_mounted ? 1u : 0u,
           cached->sd_degraded ? 1u : 0u,
           cached->sd_backoff_remaining_ms,
           cached->heap_internal_free_bytes,
           cached->heap_internal_largest_free_block_bytes,
           cached->heap_psram_free_bytes,
           cached->heap_psram_largest_free_block_bytes,
           GetStackMonitorMinBytes("control"),
           GetStackMonitorMinBytes("storage"),
           GetStackMonitorMinBytes("sd_flush"),
           GetStackMonitorMinBytes("net_tx"),
           GetStackMonitorMinBytes("alert_http"));
}

static void
LogDrainPostflight(const runtime_state_t* state,
                   const char* reason,
                   int32_t flushed_records,
                   int32_t remaining_records,
                   uint32_t duration_ms)
{
  if (state == NULL) {
    return;
  }

  StackMonitorMaybeSample(&g_stack_monitor);
  ESP_LOGI(kTag,
           "Drain post(%s): flushed=%" PRIi32 " remaining=%" PRIi32
           " duration=%" PRIu32 "ms stack_min(ctrl:%" PRIu32 " stor:%" PRIu32
           " flush:%" PRIu32 " net:%" PRIu32 " http:%" PRIu32 ")",
           (reason != NULL) ? reason : "unknown",
           flushed_records,
           remaining_records,
           duration_ms,
           GetStackMonitorMinBytes("control"),
           GetStackMonitorMinBytes("storage"),
           GetStackMonitorMinBytes("sd_flush"),
           GetStackMonitorMinBytes("net_tx"),
           GetStackMonitorMinBytes("alert_http"));
}

static void
SdFlushTriggerFlagsToString(uint32_t trigger_flags,
                            char* buffer,
                            size_t buffer_len)
{
  if (buffer == NULL || buffer_len == 0u) {
    return;
  }

  buffer[0] = '\0';
  const struct
  {
    uint32_t flag;
    const char* label;
  } entries[] = {
    { SD_FLUSH_TRIGGER_WATERMARK, "watermark" },
    { SD_FLUSH_TRIGGER_PERIODIC, "periodic" },
    { SD_FLUSH_TRIGGER_BACKLOG, "backlog" },
    { SD_FLUSH_TRIGGER_RETRY, "retry" },
    { SD_FLUSH_TRIGGER_MORE_PENDING, "more_pending" },
    { SD_FLUSH_TRIGGER_MANUAL, "manual" },
  };

  size_t used = 0u;
  for (size_t i = 0; i < (sizeof(entries) / sizeof(entries[0])); ++i) {
    if ((trigger_flags & entries[i].flag) == 0u) {
      continue;
    }
    int written = snprintf(buffer + used,
                           buffer_len - used,
                           "%s%s",
                           (used > 0u) ? "+" : "",
                           entries[i].label);
    if (written < 0) {
      buffer[0] = '\0';
      return;
    }
    if ((size_t)written >= (buffer_len - used)) {
      used = buffer_len - 1u;
      break;
    }
    used += (size_t)written;
  }

  if (used == 0u) {
    (void)snprintf(buffer, buffer_len, "unknown");
  }
}

static void
SdFlushMaybeStartSession(runtime_state_t* state)
{
  if (state == NULL || state->sd_flush_quiesce_session_active) {
    return;
  }

  uint32_t trigger_flags = state->sd_flush_pending_trigger_flags;
  if (state->sd_start_drain_pending) {
    trigger_flags |= SD_FLUSH_TRIGGER_BACKLOG;
  }
  if (state->sd_manual_drain_active) {
    trigger_flags |= SD_FLUSH_TRIGGER_MANUAL;
  }
  if (trigger_flags == 0u) {
    trigger_flags = SD_FLUSH_TRIGGER_PERIODIC;
  }

  state->sd_flush_trigger_flags = trigger_flags;
  state->sd_flush_pending_trigger_flags = 0u;
  state->sd_flush_session_records_flushed = 0u;
  state->sd_flush_session_start_ticks = xTaskGetTickCount();
  state->sd_flush_session_id++;

  char trigger_string[64];
  SdFlushTriggerFlagsToString(
    state->sd_flush_trigger_flags, trigger_string, sizeof(trigger_string));
  (void)snprintf(state->sd_flush_session_label,
                 sizeof(state->sd_flush_session_label),
                 "sd_flush/%s#%" PRIu32,
                 trigger_string,
                 state->sd_flush_session_id);
  LogDrainPreflight(state, state->sd_flush_session_label);
}

static void
SdFlushLogSessionEnd(runtime_state_t* state,
                     uint32_t records_flushed,
                     uint32_t duration_ms)
{
  if (state == NULL) {
    return;
  }

  char session_label[RUNTIME_SD_FLUSH_SESSION_LABEL_LEN];
  const char* label = state->sd_flush_session_label;
  if (label == NULL || label[0] == '\0') {
    label = "sd_flush/unknown#0";
  }
  (void)snprintf(session_label, sizeof(session_label), "%s", label);

  uint32_t remaining = 0u;
  (void)ReadFramBufferedRecords(state, &remaining);
  LogDrainPostflight(state,
                     session_label,
                     (int32_t)records_flushed,
                     (int32_t)remaining,
                     duration_ms);

  state->sd_flush_trigger_flags = 0u;
  state->sd_flush_session_records_flushed = 0u;
  state->sd_flush_session_start_ticks = 0;
  state->sd_flush_session_label[0] = '\0';
}

static void
RuntimeNotifyAllRunTasks(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  RuntimeNotifyTask(state->sensor_task);
  RuntimeNotifyTask(state->storage_task);
  RuntimeNotifyTask(state->sd_flush_task);
  RuntimeNotifyTask(state->export_task);
  RuntimeNotifyTask(state->net_tx_task);
  RuntimeNotifyTask(state->alert_http_task);
  RuntimeNotifyTask(state->wifi_direct_task);
}

static bool
AlertManagerEligible(const runtime_state_t* state)
{
  if (state == NULL) {
    return false;
  }
  if (state->node_role_active == APP_NODE_ROLE_ROOT) {
    return true;
  }
  if (state->node_role_active != APP_NODE_ROLE_SENSOR) {
    return false;
  }
  if (state->net_mode_active != APP_NET_MODE_DIRECT_WIFI) {
    return false;
  }
  if (!WifiManagerIsConnected()) {
    return false;
  }
  return !MeshTransportIsConnected(&state->mesh);
}

static bool
AlertManagerSuppressed(const runtime_state_t* state)
{
  if (state == NULL) {
    return true;
  }
  if (state->node_role_active == APP_NODE_ROLE_ROOT) {
    return false;
  }
  if (state->node_role_active != APP_NODE_ROLE_SENSOR) {
    return true;
  }
  if (state->net_mode_active != APP_NET_MODE_DIRECT_WIFI) {
    return true;
  }
  return MeshTransportIsConnected(&state->mesh);
}

static bool
AlertManagerCanSend(const runtime_state_t* state)
{
  if (!AlertManagerEligible(state)) {
    return false;
  }
  if (state->node_role_active == APP_NODE_ROLE_ROOT) {
    return true;
  }
  return WifiManagerIsConnected();
}

/**
 * @brief Reset storage stall tracking baselines.
 * @param state Runtime state.
 * @param now_ms Current uptime in milliseconds.
 */
static void
RuntimeResetStorageStallTracking(runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL) {
    return;
  }

  if (now_ms < 0) {
    now_ms = 0;
  }

  state->storage_stall_last_progress_ms = (uint32_t)now_ms;
  state->storage_stall_last_fram_count = state->cached_status.fram_count;
  state->storage_stall_last_sd_last_record_id =
    SdLoggerLastRecordIdOnSd(&state->sd_logger);
  state->storage_stall_last_sd_fail_count = state->cached_status.sd_fail_count;
  state->storage_stall_active = false;
}

/**
 * @brief Evaluate if storage drain appears stalled.
 * @param state Runtime state.
 * @param now_ms Current uptime in milliseconds.
 * @return True when stall condition is active.
 */
static bool
RuntimeComputeStorageStallCondition(runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL || now_ms < 0) {
    return false;
  }

  const uint32_t now_ms_u32 = (uint32_t)now_ms;
  const uint32_t fram_count = state->cached_status.fram_count;
  const uint32_t watermark = state->cached_status.fram_flush_watermark_records;
  const uint64_t sd_last_record_id =
    SdLoggerLastRecordIdOnSd(&state->sd_logger);
  const uint32_t sd_fail_count = state->cached_status.sd_fail_count;

  const bool progress =
    (sd_last_record_id > state->storage_stall_last_sd_last_record_id) ||
    (fram_count < state->storage_stall_last_fram_count);
  if (progress) {
    state->storage_stall_last_progress_ms = now_ms_u32;
  }

  state->storage_stall_last_fram_count = fram_count;
  state->storage_stall_last_sd_last_record_id = sd_last_record_id;

  const bool fram_pressure = state->cached_status.fram_overrun_active ||
                             (watermark > 0u && fram_count >= watermark);
  const bool sd_issue =
    state->cached_status.sd_degraded ||
    (!state->cached_status.sd_mounted &&
     state->cached_status.sd_card_present) ||
    (state->cached_status.sd_backoff_remaining_ms > 0u) ||
    (sd_fail_count > state->storage_stall_last_sd_fail_count) ||
    state->cached_status.sd_io_error_active ||
    state->cached_status.sd_out_of_space_active;

  state->storage_stall_last_sd_fail_count = sd_fail_count;

  if (!state->cached_status.runtime_running || !fram_pressure || !sd_issue) {
    state->storage_stall_active = false;
    return false;
  }

  const uint32_t elapsed_ms =
    now_ms_u32 - state->storage_stall_last_progress_ms;
  state->storage_stall_active = elapsed_ms >= kStorageStallThresholdMs;
  return state->storage_stall_active;
}

static bool
RuntimeRebootAlertLatchValid(void)
{
  return g_reboot_alert_latch.magic == kRebootAlertLatchMagic &&
         (g_reboot_alert_latch.version == 1 ||
          g_reboot_alert_latch.version == kRebootAlertLatchVersion);
}

static void
RuntimeRebootAlertLatchUpgradeIfNeeded(void)
{
  if (g_reboot_alert_latch.magic != kRebootAlertLatchMagic ||
      g_reboot_alert_latch.version != 1) {
    return;
  }
  g_reboot_alert_latch.version = kRebootAlertLatchVersion;
  g_reboot_alert_latch.send_attempt_count = 0;
  g_reboot_alert_latch.last_attempt_epoch = 0;
  g_reboot_alert_latch.last_attempt_uptime_ms = 0;
  g_reboot_alert_latch.last_gate_reason = RUNTIME_REBOOT_ALERT_GATE_UNKNOWN;
  g_reboot_alert_latch.last_send_result = RUNTIME_REBOOT_ALERT_SEND_NONE;
  g_reboot_alert_latch.last_http_status = 0;
  g_reboot_alert_latch.last_ntfy_err = 0;
  g_reboot_alert_latch.last_retry_after_seconds = 0;
  g_reboot_alert_latch.sent_successfully = false;
}

static void
RuntimeRebootAlertLatchSet(alert_system_code_t code,
                           int64_t epoch,
                           uint32_t uptime_ms)
{
  g_reboot_alert_latch.magic = kRebootAlertLatchMagic;
  g_reboot_alert_latch.version = kRebootAlertLatchVersion;
  g_reboot_alert_latch.pending_is_active = true;
  g_reboot_alert_latch.pending_system_code = (uint32_t)code;
  g_reboot_alert_latch.pending_epoch = epoch;
  g_reboot_alert_latch.pending_uptime_ms = uptime_ms;
  g_reboot_alert_latch.send_attempt_count = 0;
  g_reboot_alert_latch.last_attempt_epoch = 0;
  g_reboot_alert_latch.last_attempt_uptime_ms = 0;
  g_reboot_alert_latch.last_gate_reason = RUNTIME_REBOOT_ALERT_GATE_UNKNOWN;
  g_reboot_alert_latch.last_send_result = RUNTIME_REBOOT_ALERT_SEND_NONE;
  g_reboot_alert_latch.last_http_status = 0;
  g_reboot_alert_latch.last_ntfy_err = 0;
  g_reboot_alert_latch.last_retry_after_seconds = 0;
  g_reboot_alert_latch.sent_successfully = false;
}

static void
RuntimeRebootAlertLatchClear(void)
{
  g_reboot_alert_latch.magic = kRebootAlertLatchMagic;
  g_reboot_alert_latch.version = kRebootAlertLatchVersion;
  g_reboot_alert_latch.pending_is_active = false;
  g_reboot_alert_latch.pending_system_code = 0;
  g_reboot_alert_latch.pending_epoch = 0;
  g_reboot_alert_latch.pending_uptime_ms = 0;
  g_reboot_alert_latch.send_attempt_count = 0;
  g_reboot_alert_latch.last_attempt_epoch = 0;
  g_reboot_alert_latch.last_attempt_uptime_ms = 0;
  g_reboot_alert_latch.last_gate_reason = RUNTIME_REBOOT_ALERT_GATE_UNKNOWN;
  g_reboot_alert_latch.last_send_result = RUNTIME_REBOOT_ALERT_SEND_NONE;
  g_reboot_alert_latch.last_http_status = 0;
  g_reboot_alert_latch.last_ntfy_err = 0;
  g_reboot_alert_latch.last_retry_after_seconds = 0;
  g_reboot_alert_latch.sent_successfully = false;
}

static void
RuntimeLoadRebootAlertLatch(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  state->reboot_alert_pending = false;
  state->reboot_alert_active_sent = false;
  state->reboot_alert_active_sent_ms = 0;
  state->reboot_alert_event_epoch = 0;
  state->reboot_alert_event_uptime_ms = 0;
  state->reboot_alert_code = ALERT_SYSTEM_CODE_NONE;
  state->reboot_alert_next_check_ms = 0;

  if (!RuntimeRebootAlertLatchValid()) {
    return;
  }
  RuntimeRebootAlertLatchUpgradeIfNeeded();
  if (!g_reboot_alert_latch.pending_is_active ||
      g_reboot_alert_latch.pending_system_code == 0) {
    return;
  }

  state->reboot_alert_pending = true;
  state->reboot_alert_code =
    (alert_system_code_t)g_reboot_alert_latch.pending_system_code;
  state->reboot_alert_event_epoch = g_reboot_alert_latch.pending_epoch;
  state->reboot_alert_event_uptime_ms = g_reboot_alert_latch.pending_uptime_ms;
}

static void
RuntimeRtcErrlogLatchInitIfNeeded(void)
{
  taskENTER_CRITICAL(&g_rtc_errlog_latch_lock);
  const bool needs_reset =
    g_rtc_errlog_latch.magic != kRtcErrlogLatchMagic ||
    g_rtc_errlog_latch.version != kRtcErrlogLatchVersion ||
    g_rtc_errlog_latch.count > RUNTIME_RTC_ERRLOG_RING_SIZE ||
    g_rtc_errlog_latch.head >= RUNTIME_RTC_ERRLOG_RING_SIZE;
  if (needs_reset) {
    memset(&g_rtc_errlog_latch, 0, sizeof(g_rtc_errlog_latch));
    g_rtc_errlog_latch.magic = kRtcErrlogLatchMagic;
    g_rtc_errlog_latch.version = kRtcErrlogLatchVersion;
  }
  taskEXIT_CRITICAL(&g_rtc_errlog_latch_lock);
}

static void
RuntimeRtcErrlogLatchPush(uint16_t code,
                          bool resolved,
                          int32_t detail0,
                          int32_t detail1,
                          uint32_t epoch_sec,
                          uint16_t millis)
{
  RuntimeRtcErrlogLatchInitIfNeeded();
  taskENTER_CRITICAL(&g_rtc_errlog_latch_lock);
  uint16_t write_index = 0;
  if (g_rtc_errlog_latch.count >= RUNTIME_RTC_ERRLOG_RING_SIZE) {
    write_index = g_rtc_errlog_latch.head;
    g_rtc_errlog_latch.head =
      (g_rtc_errlog_latch.head + 1u) % RUNTIME_RTC_ERRLOG_RING_SIZE;
    g_rtc_errlog_latch.count = RUNTIME_RTC_ERRLOG_RING_SIZE;
  } else {
    write_index = (g_rtc_errlog_latch.head + g_rtc_errlog_latch.count) %
                  RUNTIME_RTC_ERRLOG_RING_SIZE;
    g_rtc_errlog_latch.count++;
  }
  runtime_rtc_errlog_entry_t* entry = &g_rtc_errlog_latch.entries[write_index];
  entry->code = code;
  entry->resolved = resolved;
  entry->detail0 = detail0;
  entry->detail1 = detail1;
  entry->epoch_sec = epoch_sec;
  entry->millis = millis;
  taskEXIT_CRITICAL(&g_rtc_errlog_latch_lock);
}

static void
RuntimeRtcErrlogLatchFlushToFram(runtime_state_t* state)
{
  if (state == NULL || !state->fram_error_log.initialized) {
    return;
  }
  RuntimeRtcErrlogLatchInitIfNeeded();
  while (true) {
    runtime_rtc_errlog_entry_t entry = { 0 };
    uint16_t head_snapshot = 0;
    bool has_entry = false;
    taskENTER_CRITICAL(&g_rtc_errlog_latch_lock);
    if (g_rtc_errlog_latch.count > 0) {
      head_snapshot = g_rtc_errlog_latch.head;
      entry = g_rtc_errlog_latch.entries[head_snapshot];
      has_entry = true;
    }
    taskEXIT_CRITICAL(&g_rtc_errlog_latch_lock);
    if (!has_entry) {
      break;
    }
    bool logged = false;
    const esp_err_t result =
      entry.resolved ? FramErrorLogAppendResolved(&state->fram_error_log,
                                                  entry.code,
                                                  entry.detail0,
                                                  entry.detail1,
                                                  entry.epoch_sec,
                                                  entry.millis,
                                                  &logged)
                     : FramErrorLogAppendActive(&state->fram_error_log,
                                                entry.code,
                                                entry.detail0,
                                                entry.detail1,
                                                entry.epoch_sec,
                                                entry.millis,
                                                &logged);
    if (result != ESP_OK) {
      break;
    }
    taskENTER_CRITICAL(&g_rtc_errlog_latch_lock);
    if (g_rtc_errlog_latch.count > 0 &&
        g_rtc_errlog_latch.head == head_snapshot) {
      g_rtc_errlog_latch.head =
        (g_rtc_errlog_latch.head + 1u) % RUNTIME_RTC_ERRLOG_RING_SIZE;
      g_rtc_errlog_latch.count--;
    }
    taskEXIT_CRITICAL(&g_rtc_errlog_latch_lock);
  }
}

static const char*
RuntimeRebootAlertGateReasonToString(runtime_reboot_alert_gate_reason_t reason)
{
  switch (reason) {
    case RUNTIME_REBOOT_ALERT_GATE_NOT_CONFIGURED:
      return "not_configured";
    case RUNTIME_REBOOT_ALERT_GATE_DISABLED_BY_MASK:
      return "disabled_by_mask";
    case RUNTIME_REBOOT_ALERT_GATE_NOT_ELIGIBLE_NET_MODE:
      return "not_eligible_net_mode";
    case RUNTIME_REBOOT_ALERT_GATE_NOT_ELIGIBLE_ROLE:
      return "not_eligible_role";
    case RUNTIME_REBOOT_ALERT_GATE_WIFI_DISCONNECTED:
      return "wifi_disconnected";
    case RUNTIME_REBOOT_ALERT_GATE_MESH_CONNECTED_BLOCKING_DIRECT:
      return "mesh_connected";
    case RUNTIME_REBOOT_ALERT_GATE_COOLDOWN_ACTIVE:
      return "cooldown_active";
    case RUNTIME_REBOOT_ALERT_GATE_QUEUE_FULL:
      return "queue_full";
    case RUNTIME_REBOOT_ALERT_GATE_UNKNOWN:
    default:
      return "unknown";
  }
}

static runtime_reboot_alert_gate_reason_t
RuntimeRebootAlertGateReason(const runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL) {
    return RUNTIME_REBOOT_ALERT_GATE_UNKNOWN;
  }
  if (!AlertManagerIsConfigured(&state->alert_manager)) {
    return RUNTIME_REBOOT_ALERT_GATE_NOT_CONFIGURED;
  }
  const uint32_t enable_mask = state->alert_manager.config.enable_mask;
  if ((enable_mask & (1u << ALERT_SYSTEM_ERROR)) == 0u) {
    return RUNTIME_REBOOT_ALERT_GATE_DISABLED_BY_MASK;
  }
  if (state->node_role_active != APP_NODE_ROLE_ROOT &&
      state->node_role_active != APP_NODE_ROLE_SENSOR) {
    return RUNTIME_REBOOT_ALERT_GATE_NOT_ELIGIBLE_ROLE;
  }
  if (state->node_role_active == APP_NODE_ROLE_SENSOR &&
      state->net_mode_active != APP_NET_MODE_DIRECT_WIFI) {
    return RUNTIME_REBOOT_ALERT_GATE_NOT_ELIGIBLE_NET_MODE;
  }
  if (!WifiManagerIsConnected()) {
    return RUNTIME_REBOOT_ALERT_GATE_WIFI_DISCONNECTED;
  }
  if (state->node_role_active == APP_NODE_ROLE_SENSOR &&
      MeshTransportIsConnected(&state->mesh)) {
    return RUNTIME_REBOOT_ALERT_GATE_MESH_CONNECTED_BLOCKING_DIRECT;
  }
  if (state->alert_manager.ntfy.cooldown_until_ms > now_ms) {
    return RUNTIME_REBOOT_ALERT_GATE_COOLDOWN_ACTIVE;
  }
  return RUNTIME_REBOOT_ALERT_GATE_UNKNOWN;
}

static void
RuntimeRebootAlertLatchRecordGate(runtime_reboot_alert_gate_reason_t reason)
{
  if (!RuntimeRebootAlertLatchValid()) {
    return;
  }
  RuntimeRebootAlertLatchUpgradeIfNeeded();
  if (!g_reboot_alert_latch.pending_is_active) {
    return;
  }
  g_reboot_alert_latch.last_gate_reason = (uint32_t)reason;
  g_reboot_alert_latch.last_send_result = RUNTIME_REBOOT_ALERT_SEND_SKIPPED;
}

static void
RuntimeRebootAlertLatchRecordAttempt(runtime_reboot_alert_send_result_t result,
                                     runtime_reboot_alert_gate_reason_t reason,
                                     int64_t attempt_epoch,
                                     uint32_t attempt_uptime_ms,
                                     int http_status,
                                     esp_err_t err,
                                     int retry_after_seconds)
{
  if (!RuntimeRebootAlertLatchValid()) {
    return;
  }
  RuntimeRebootAlertLatchUpgradeIfNeeded();
  if (!g_reboot_alert_latch.pending_is_active) {
    return;
  }
  g_reboot_alert_latch.send_attempt_count++;
  g_reboot_alert_latch.last_attempt_epoch = attempt_epoch;
  g_reboot_alert_latch.last_attempt_uptime_ms = attempt_uptime_ms;
  g_reboot_alert_latch.last_gate_reason = (uint32_t)reason;
  g_reboot_alert_latch.last_send_result = (uint32_t)result;
  g_reboot_alert_latch.last_http_status = (int32_t)http_status;
  g_reboot_alert_latch.last_ntfy_err = (int32_t)err;
  g_reboot_alert_latch.last_retry_after_seconds = (int32_t)retry_after_seconds;
  if (result == RUNTIME_REBOOT_ALERT_SEND_OK) {
    g_reboot_alert_latch.sent_successfully = true;
  }
}

static bool
RuntimeWaitForWifiConnected(uint32_t wait_ms)
{
  if (WifiManagerIsConnected()) {
    return true;
  }
  const int64_t start_ms = esp_timer_get_time() / 1000;
  while ((esp_timer_get_time() / 1000) - start_ms < (int64_t)wait_ms) {
    if (WifiManagerIsConnected()) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return WifiManagerIsConnected();
}

static bool
RuntimeEnqueueSystemErrorNote(runtime_state_t* state,
                              alert_system_code_t code,
                              bool resolved,
                              int64_t event_epoch,
                              int64_t event_uptime_ms)
{
  if (state == NULL || !AlertManagerIsConfigured(&state->alert_manager)) {
    return false;
  }
  if (state->alert_manager.ntfy.queue == NULL) {
    return false;
  }
  const uint32_t enable_mask = state->alert_manager.config.enable_mask;
  if ((enable_mask & (1u << ALERT_SYSTEM_ERROR)) == 0u) {
    return false;
  }

  alert_notification_t note = { 0 };
  note.type = ALERT_SYSTEM_ERROR;
  note.severity = ALERT_SEV_CRIT;
  note.resolved = resolved;
  note.leaf_id = state->local_leaf_id;
  note.payload.event_code = (uint32_t)code;
  note.payload.event_epoch = (event_epoch > 0) ? event_epoch : -1;
  note.payload.event_uptime_ms = event_uptime_ms;
  return AlertNtfyEnqueue(&state->alert_manager.ntfy, &note);
}

/**
 * @brief Enqueue an ntfy job and optionally wait for AlertHttpTask completion.
 * @param state Runtime state.
 * @param job Ntfy job to enqueue.
 * @param wait_ms Maximum time to wait for send counters to change.
 * @param status_out Optional pointer receiving last HTTP status.
 * @param err_out Optional pointer receiving last esp_err_t.
 * @param retry_after_seconds_out Optional pointer receiving cooldown-derived
 * retry seconds.
 * @return Result indicating enqueue/wait/send outcome.
 */
static alert_ntfy_result_t
RuntimeEnqueueNtfyJobAndWait(runtime_state_t* state,
                             const alert_ntfy_job_t* job,
                             uint32_t wait_ms,
                             int* status_out,
                             esp_err_t* err_out,
                             int* retry_after_seconds_out)
{
  if (status_out != NULL) {
    *status_out = 0;
  }
  if (err_out != NULL) {
    *err_out = ESP_OK;
  }
  if (retry_after_seconds_out != NULL) {
    *retry_after_seconds_out = -1;
  }
  if (state == NULL) {
    if (err_out != NULL) {
      *err_out = ESP_ERR_INVALID_ARG;
    }
    return ALERT_NTFY_FAILED;
  }
  if (state->alert_manager.ntfy.job_queue == NULL) {
    if (err_out != NULL) {
      *err_out = ESP_ERR_INVALID_STATE;
    }
    return ALERT_NTFY_SKIPPED;
  }

  const uint32_t success_before = state->alert_manager.ntfy.send_success;
  const uint32_t fail_before = state->alert_manager.ntfy.send_fail;
  if (job != NULL && !AlertNtfyEnqueueJob(&state->alert_manager.ntfy, job)) {
    if (err_out != NULL) {
      *err_out = ESP_FAIL;
    }
    return ALERT_NTFY_FAILED;
  }

  const int64_t start_ms = esp_timer_get_time() / 1000;
  while (!state->stop_requested) {
    const bool sent_ok =
      (state->alert_manager.ntfy.send_success != success_before);
    const bool sent_fail = (state->alert_manager.ntfy.send_fail != fail_before);
    if (sent_ok || sent_fail) {
      break;
    }
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if ((uint32_t)(now_ms - start_ms) >= wait_ms) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(25));
  }

  const int64_t now_ms = esp_timer_get_time() / 1000;
  if (status_out != NULL) {
    *status_out = state->alert_manager.ntfy.last_http_status;
  }
  if (err_out != NULL) {
    *err_out = state->alert_manager.ntfy.last_err;
  }
  if (retry_after_seconds_out != NULL) {
    *retry_after_seconds_out = -1;
    if (state->alert_manager.ntfy.cooldown_until_ms > now_ms) {
      *retry_after_seconds_out =
        (int)((state->alert_manager.ntfy.cooldown_until_ms - now_ms + 999) /
              1000);
    }
  }

  if (state->alert_manager.ntfy.send_success != success_before) {
    return ALERT_NTFY_OK;
  }
  if (state->alert_manager.ntfy.send_fail != fail_before) {
    return ALERT_NTFY_FAILED;
  }
  if (err_out != NULL && *err_out == ESP_OK) {
    *err_out = ESP_ERR_TIMEOUT;
  }
  return ALERT_NTFY_SKIPPED;
}

static alert_ntfy_result_t
RuntimeAttemptPreRebootAlertSendDetailed(runtime_state_t* state,
                                         alert_system_code_t code,
                                         int64_t event_epoch,
                                         int64_t event_uptime_ms,
                                         int* retry_after_seconds_out,
                                         int* status_out,
                                         esp_err_t* err_out)
{
  if (retry_after_seconds_out != NULL) {
    *retry_after_seconds_out = -1;
  }
  if (status_out != NULL) {
    *status_out = 0;
  }
  if (err_out != NULL) {
    *err_out = ESP_OK;
  }
  if (state == NULL) {
    if (err_out != NULL) {
      *err_out = ESP_ERR_INVALID_ARG;
    }
    return ALERT_NTFY_FAILED;
  }
  if (!AlertManagerIsConfigured(&state->alert_manager)) {
    if (err_out != NULL) {
      *err_out = ESP_ERR_INVALID_STATE;
    }
    return ALERT_NTFY_SKIPPED;
  }
  const uint32_t enable_mask = state->alert_manager.config.enable_mask;
  if ((enable_mask & (1u << ALERT_SYSTEM_ERROR)) == 0u) {
    if (err_out != NULL) {
      *err_out = ESP_ERR_INVALID_STATE;
    }
    return ALERT_NTFY_SKIPPED;
  }

  bool wifi_connected = WifiManagerIsConnected();
  if (!wifi_connected) {
    wifi_connected = RuntimeWaitForWifiConnected(kRebootAlertWifiWaitMs);
  }
  if (!wifi_connected) {
    if (err_out != NULL) {
      *err_out = ESP_ERR_TIMEOUT;
    }
    return ALERT_NTFY_SKIPPED;
  }

  const int64_t now_ms = esp_timer_get_time() / 1000;
  if (!RuntimeEnqueueSystemErrorNote(
        state, code, false, event_epoch, event_uptime_ms)) {
    if (err_out != NULL) {
      *err_out = ESP_FAIL;
    }
    return ALERT_NTFY_FAILED;
  }

  int64_t next_attempt_ms = now_ms;
  (void)AlertManagerPumpNtfy(&state->alert_manager, now_ms, &next_attempt_ms);

  return RuntimeEnqueueNtfyJobAndWait(state,
                                      NULL,
                                      kRebootAlertHttpTimeoutMs,
                                      status_out,
                                      err_out,
                                      retry_after_seconds_out);
}

esp_err_t
RuntimeManagerRunSafeHoldIfNeeded(runtime_state_t* state)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!RuntimeRebootAlertLatchIsPending()) {
    return ESP_OK;
  }

  ESP_LOGW(kTag, "SAFE HOLD active: pending reboot alert latched");
  const int64_t start_ms = esp_timer_get_time() / 1000;
  uint32_t backoff_ms = kSafeHoldMinRetryMs;
  uint32_t last_log_ms = 0;

  while (RuntimeRebootAlertLatchIsPending()) {
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if ((uint32_t)(now_ms - start_ms) >= kSafeHoldMaxMs) {
      ESP_LOGW(kTag, "SAFE HOLD timeout after %u ms", kSafeHoldMaxMs);
      return ESP_ERR_TIMEOUT;
    }

    runtime_reboot_alert_gate_reason_t gate_reason =
      RuntimeRebootAlertGateReason(state, now_ms);
    if (gate_reason != RUNTIME_REBOOT_ALERT_GATE_UNKNOWN) {
      RuntimeRebootAlertLatchRecordGate(gate_reason);
      if (LogRateLimitAllow(&last_log_ms, kSafeHoldLogIntervalMs)) {
        ESP_LOGI(kTag,
                 "SAFE HOLD waiting: gate=%s",
                 RuntimeRebootAlertGateReasonToString(gate_reason));
      }
      vTaskDelay(pdMS_TO_TICKS(backoff_ms));
      backoff_ms = (backoff_ms < kSafeHoldMaxRetryMs) ? (backoff_ms * 2)
                                                      : kSafeHoldMaxRetryMs;
      continue;
    }

    int64_t event_epoch = state->reboot_alert_event_epoch;
    uint32_t event_uptime_ms = state->reboot_alert_event_uptime_ms;
    if (event_uptime_ms == 0) {
      event_uptime_ms = (uint32_t)now_ms;
    }
    if (event_epoch <= 0 && TimeSyncIsSystemTimeValid()) {
      event_epoch = (int64_t)time(NULL);
    }

    int status = 0;
    int retry_after_seconds = -1;
    esp_err_t err = ESP_OK;
    alert_ntfy_result_t result =
      RuntimeAttemptPreRebootAlertSendDetailed(state,
                                               state->reboot_alert_code,
                                               event_epoch,
                                               event_uptime_ms,
                                               &retry_after_seconds,
                                               &status,
                                               &err);
    runtime_reboot_alert_send_result_t send_result =
      (result == ALERT_NTFY_OK)
        ? RUNTIME_REBOOT_ALERT_SEND_OK
        : (result == ALERT_NTFY_FAILED ? RUNTIME_REBOOT_ALERT_SEND_FAIL
                                       : RUNTIME_REBOOT_ALERT_SEND_SKIPPED);
    const int64_t attempt_epoch =
      TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    RuntimeRebootAlertLatchRecordAttempt(send_result,
                                         gate_reason,
                                         attempt_epoch,
                                         (uint32_t)now_ms,
                                         status,
                                         err,
                                         retry_after_seconds);

    if (result == ALERT_NTFY_OK) {
      state->reboot_alert_pending = false;
      state->reboot_alert_active_sent = true;
      state->reboot_alert_active_sent_ms = now_ms;
      state->reboot_alert_next_check_ms =
        now_ms + (int64_t)kRebootAlertResolveDelayMs;
      RuntimeRebootAlertLatchClear();
      ESP_LOGI(kTag, "SAFE HOLD alert sent");
      return ESP_OK;
    }

    if (LogRateLimitAllow(&last_log_ms, kSafeHoldLogIntervalMs)) {
      ESP_LOGW(kTag,
               "SAFE HOLD send failed: result=%d err=%s status=%d retry=%d",
               (int)result,
               esp_err_to_name(err),
               status,
               retry_after_seconds);
    }
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    backoff_ms = (backoff_ms < kSafeHoldMaxRetryMs) ? (backoff_ms * 2)
                                                    : kSafeHoldMaxRetryMs;
  }

  return ESP_OK;
}

static uint32_t
RuntimeExpectedSpiPauseMask(runtime_state_t* state)
{
  if (state == NULL) {
    return 0;
  }
  uint32_t mask = 0;
  if (state->sensor_task != NULL) {
    mask |= SPI_PAUSE_ACK_SENSOR;
  }
  if (state->display_task != NULL) {
    mask |= SPI_PAUSE_ACK_DISPLAY;
  }
  return mask;
}

static bool
RuntimePauseSpiUsers(runtime_state_t* state, uint32_t timeout_ms)
{
  if (state == NULL) {
    return false;
  }

  const uint32_t expected_mask = RuntimeExpectedSpiPauseMask(state);
  if (expected_mask == 0) {
    return true;
  }

  state->spi_pause_requested = true;
  state->spi_pause_ack_mask = 0;
  RuntimeNotifyTask(state->display_task);
  RuntimeNotifyTask(state->sensor_task);

  const TickType_t wait_start = xTaskGetTickCount();
  while (((state->spi_pause_ack_mask & expected_mask) != expected_mask) &&
         (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < timeout_ms)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if ((state->spi_pause_ack_mask & expected_mask) != expected_mask) {
    ESP_LOGW(kTag,
             "SPI pause timed out; acked=0x%02" PRIx32 " expected=0x%02" PRIx32,
             state->spi_pause_ack_mask,
             expected_mask);
    return false;
  }
  return true;
}

static void
RuntimeResumeSpiUsers(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  state->spi_pause_requested = false;
  state->spi_pause_ack_mask = 0;
  RuntimeNotifyTask(state->display_task);
  RuntimeNotifyTask(state->sensor_task);
}

/**
 * @brief Request net_tx + alert_http pause state and wait for acknowledgements.
 * @param state Runtime state.
 * @param pause_enabled True to request pause, false to resume.
 * @param timeout_ms Maximum wait for acknowledgement.
 */
static void
RuntimeRequestNetPause(runtime_state_t* state,
                       bool pause_enabled,
                       uint32_t timeout_ms)
{
  if (state == NULL) {
    return;
  }

  state->net_tx_pause_requested = pause_enabled;
  state->alert_http_pause_requested = pause_enabled;
  RuntimeNotifyTask(state->net_tx_task);
  RuntimeNotifyTask(state->alert_http_task);

  const TickType_t wait_start = xTaskGetTickCount();
  while (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < timeout_ms) {
    const bool net_ok =
      (state->net_tx_task == NULL) || (state->net_tx_paused == pause_enabled);
    const bool alert_ok = (state->alert_http_task == NULL) ||
                          (state->alert_http_paused == pause_enabled);
    if (net_ok && alert_ok) {
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_LOGW(kTag,
           "net pause timeout: requested=%d net_paused=%d alert_paused=%d",
           (int)pause_enabled,
           (int)state->net_tx_paused,
           (int)state->alert_http_paused);
}

static void
RuntimeInterruptibleDelayTicks(TickType_t delay_ticks)
{
  (void)ulTaskNotifyTake(pdTRUE, delay_ticks);
}

static bool
ManualDrainTimedOutTicks(TickType_t now_ticks, TickType_t deadline_ticks)
{
  if (deadline_ticks == 0) {
    return false;
  }
  return ((int32_t)(now_ticks - deadline_ticks) >= 0);
}

static void
RuntimeSyncFramFallbackCounters(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  const uint32_t next_sequence = FramLogNextSequence(&state->fram_log);
  const uint64_t next_record_id = FramLogNextRecordId(&state->fram_log);
  if (next_sequence != 0) {
    state->fram_fallback_sequence = next_sequence;
  }
  if (next_record_id != 0) {
    state->fram_fallback_record_id = next_record_id;
  }
}

static void
RuntimeSetFramUnavailable(runtime_state_t* state,
                          const char* context,
                          esp_err_t error,
                          int64_t now_ms)
{
  if (state == NULL) {
    return;
  }
  state->fram_available = false;
  state->fram_next_retry_ms = now_ms + kFramRetryIntervalMs;
  if (LogRateLimitAllow(&state->last_fram_retry_log_ms,
                        (uint32_t)kFramRetryIntervalMs)) {
    ESP_LOGW(kTag,
             "FRAM unavailable (%s): %s; retry in %u ms",
             (context != NULL) ? context : "init",
             esp_err_to_name(error),
             (unsigned)kFramRetryIntervalMs);
  }
  if (!state->cached_status.fram_io_error_active) {
    LogFramErrorEvent(state, ERROR_FRAM_IO_FAIL, false, (int32_t)error, 0);
    UpdateCachedBool(state, &state->cached_status.fram_io_error_active, true);
  }
  if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    UpdateFramFillState(state);
    RuntimeFramLogUnlock(state);
  }
}

static bool
ReadFramBufferedRecords(runtime_state_t* state, uint32_t* buffered_out)
{
  if (buffered_out == NULL) {
    return false;
  }
  *buffered_out = 0;
  if (state == NULL) {
    return false;
  }
  if (!state->fram_available) {
    return true;
  }
  if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    return false;
  }
  *buffered_out = FramLogGetBufferedRecords(&state->fram_log);
  RuntimeFramLogUnlock(state);
  return true;
}

static esp_err_t
RuntimeAssignRecordIds(runtime_state_t* state, log_record_t* record)
{
  if (state == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!state->fram_available) {
    if (state->fram_fallback_sequence == 0) {
      state->fram_fallback_sequence = 1;
    }
    if (state->fram_fallback_record_id == 0) {
      state->fram_fallback_record_id = 1;
    }
    record->sequence = state->fram_fallback_sequence++;
    record->record_id = state->fram_fallback_record_id++;
    record->schema_version = LOG_RECORD_SCHEMA_VER;
    return ESP_OK;
  }
  const esp_err_t result = FramLogAssignRecordIds(&state->fram_log, record);
  if (result == ESP_OK) {
    state->fram_fallback_sequence = record->sequence + 1u;
    state->fram_fallback_record_id = record->record_id + 1u;
  }
  return result;
}

/**
 * @brief Assign monotonic fallback record IDs without FRAM access.
 * @param state Runtime state.
 * @param record Record to update.
 */
static void
RuntimeAssignFallbackRecordIds(runtime_state_t* state, log_record_t* record)
{
  if (state == NULL || record == NULL) {
    return;
  }
  if (state->fram_fallback_sequence == 0u) {
    state->fram_fallback_sequence = 1u;
  }
  if (state->fram_fallback_record_id == 0u) {
    state->fram_fallback_record_id = 1u;
  }
  record->sequence = state->fram_fallback_sequence++;
  record->record_id = state->fram_fallback_record_id++;
  record->schema_version = LOG_RECORD_SCHEMA_VER;
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
 * @brief Latch operator hold state for immediate display feedback.
 * @param state Runtime state.
 * @param now_ms Button/event uptime in milliseconds.
 */
static void
RuntimeLatchOperatorHold(runtime_state_t* state, uint32_t now_ms)
{
  if (state == NULL) {
    return;
  }

  (void)now_ms;
  UpdateCachedBool(state, &state->cached_status.operator_hold_latched, true);
}

/**
 * @brief Clear operator hold latch when returning to RUN mode.
 * @param state Runtime state.
 */
static void
RuntimeClearOperatorHold(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }

  UpdateCachedBool(state, &state->cached_status.operator_hold_latched, false);
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

static void
UpdateCachedUint64(runtime_state_t* state, uint64_t* field, uint64_t value)
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
DeferredLogPush(runtime_state_t* state, const log_record_t* record)
{
  if (state == NULL || record == NULL) {
    return false;
  }
  bool pushed = false;
  taskENTER_CRITICAL(&state->deferred_log.lock);
  if (state->deferred_log.count < DEFERRED_LOG_CAPACITY) {
    state->deferred_log.records[state->deferred_log.head] = *record;
    state->deferred_log.head =
      (uint8_t)((state->deferred_log.head + 1u) % DEFERRED_LOG_CAPACITY);
    state->deferred_log.count++;
    pushed = true;
  }
  taskEXIT_CRITICAL(&state->deferred_log.lock);
  return pushed;
}

static bool
DeferredLogPop(runtime_state_t* state, log_record_t* out)
{
  if (state == NULL || out == NULL) {
    return false;
  }
  bool popped = false;
  taskENTER_CRITICAL(&state->deferred_log.lock);
  if (state->deferred_log.count > 0u) {
    *out = state->deferred_log.records[state->deferred_log.tail];
    state->deferred_log.tail =
      (uint8_t)((state->deferred_log.tail + 1u) % DEFERRED_LOG_CAPACITY);
    state->deferred_log.count--;
    popped = true;
  }
  taskEXIT_CRITICAL(&state->deferred_log.lock);
  return popped;
}

static uint8_t
DeferredLogCount(runtime_state_t* state)
{
  if (state == NULL) {
    return 0;
  }
  uint8_t count = 0;
  taskENTER_CRITICAL(&state->deferred_log.lock);
  count = state->deferred_log.count;
  taskEXIT_CRITICAL(&state->deferred_log.lock);
  return count;
}

static uint32_t
DeferredLogDrops(runtime_state_t* state)
{
  if (state == NULL) {
    return 0;
  }
  uint32_t drops = 0;
  taskENTER_CRITICAL(&state->deferred_log.lock);
  drops = state->deferred_log.drops;
  taskEXIT_CRITICAL(&state->deferred_log.lock);
  return drops;
}

static void
DeferredLogReset(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  taskENTER_CRITICAL(&state->deferred_log.lock);
  state->deferred_log.head = 0;
  state->deferred_log.tail = 0;
  state->deferred_log.count = 0;
  state->deferred_log.drops = 0;
  taskEXIT_CRITICAL(&state->deferred_log.lock);
  UpdateCachedUint32(state, &state->cached_status.deferred_count, 0u);
  UpdateCachedUint32(state, &state->cached_status.deferred_drops, 0u);
  UpdateCachedBool(state, &state->cached_status.deferred_active, false);
}

static void
RuntimeBeginI2cQuiesce(runtime_state_t* state, const char* reason)
{
  if (state == NULL) {
    return;
  }

  if (state->i2c_quiesce_refcount > 0u) {
    state->i2c_quiesce_refcount++;
    state->i2c_quiesce_reason = (reason != NULL) ? reason : "unknown";
    return;
  }

  state->i2c_quiesce_active = true;
  state->i2c_quiesce_refcount = 1u;
  state->i2c_quiesce_enter_count++;
  state->i2c_quiesce_reason = (reason != NULL) ? reason : "unknown";
  UpdateCachedBool(state, &state->cached_status.i2c_quiesce_active, true);
  UpdateCachedUint32(
    state, &state->cached_status.deferred_count, DeferredLogCount(state));
  UpdateCachedUint32(
    state, &state->cached_status.deferred_drops, DeferredLogDrops(state));
  UpdateCachedBool(state,
                   &state->cached_status.deferred_active,
                   (DeferredLogCount(state) > 0u));
  if (strncmp(state->i2c_quiesce_reason, "sd_flush", 8) == 0) {
    char trigger_string[64];
    SdFlushTriggerFlagsToString(
      state->sd_flush_trigger_flags, trigger_string, sizeof(trigger_string));
    ESP_LOGI(kTag,
             "I2C quiesce begin (%s) sd_session_id=%" PRIu32
             " trigger=%s flags=0x%08" PRIx32
             " pending=%u start_drain=%u fram=%" PRIu32 " wm=%" PRIu32
             " deferred=%u drops=%u",
             state->i2c_quiesce_reason,
             state->sd_flush_session_id,
             trigger_string,
             state->sd_flush_trigger_flags,
             state->sd_flush_pending ? 1u : 0u,
             state->sd_start_drain_pending ? 1u : 0u,
             state->cached_status.fram_count,
             state->cached_status.fram_flush_watermark_records,
             (unsigned)state->cached_status.deferred_count,
             (unsigned)state->cached_status.deferred_drops);
  } else {
    ESP_LOGI(kTag,
             "I2C quiesce begin (%s) deferred=%u drops=%u",
             state->i2c_quiesce_reason,
             (unsigned)state->cached_status.deferred_count,
             (unsigned)state->cached_status.deferred_drops);
  }
}

static void
RuntimeEndI2cQuiesce(runtime_state_t* state)
{
  if (state == NULL || state->i2c_quiesce_refcount == 0u) {
    return;
  }

  state->i2c_quiesce_refcount--;
  if (state->i2c_quiesce_refcount > 0u) {
    return;
  }

  state->i2c_quiesce_active = false;
  state->i2c_quiesce_reason = NULL;
  UpdateCachedBool(state, &state->cached_status.i2c_quiesce_active, false);
  UpdateCachedUint32(
    state, &state->cached_status.deferred_count, DeferredLogCount(state));
  UpdateCachedUint32(
    state, &state->cached_status.deferred_drops, DeferredLogDrops(state));
  UpdateCachedBool(state,
                   &state->cached_status.deferred_active,
                   (DeferredLogCount(state) > 0u));
}

static esp_err_t
RuntimeDrainDeferredToFram(runtime_state_t* state, const char* context)
{
  if (state == NULL || !state->fram_available) {
    return ESP_OK;
  }
  log_record_t record;
  while (DeferredLogPop(state, &record)) {
    if (!RuntimeFramLogLockWithWarn(
          state,
          kFramLogLockTimeoutTicks,
          &state->fram_log_lock_timeout_count_sdflush,
          &state->last_fram_log_lock_timeout_sdflush_log_ms,
          context)) {
      (void)DeferredLogPush(state, &record);
      return ESP_ERR_TIMEOUT;
    }
    if (record.sequence == 0u || record.record_id == 0u) {
      const esp_err_t id_result = RuntimeAssignRecordIds(state, &record);
      if (id_result != ESP_OK) {
        RuntimeFramLogUnlock(state);
        (void)DeferredLogPush(state, &record);
        return id_result;
      }
    }
    const esp_err_t append_result = FramLogAppend(&state->fram_log, &record);
    RuntimeFramLogUnlock(state);
    if (append_result != ESP_OK) {
      (void)DeferredLogPush(state, &record);
      return append_result;
    }
    state->sd_flush_records_since++;
  }
  return ESP_OK;
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
    if (!state->logger_running) {
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
 * @brief Execute RuntimeSdFsLock.
 * @param state Parameter state.
 * @param timeout_ticks Parameter timeout_ticks.
 * @return Return the function result.
 */
bool
RuntimeSdFsLock(runtime_state_t* state, TickType_t timeout_ticks)
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
    const char* phase = (state->stop_requested || !state->logger_running)
                          ? "stop/diag"
                          : "runtime";
    state->sd_io_mutex_timeouts++;
    const TickType_t now_ticks = xTaskGetTickCount();
    const uint32_t held_ms =
      RuntimeElapsedMs(state->sd_io_mutex_hold_start_ticks, now_ticks);
    const char* holder = (state->sd_io_mutex_holder != NULL)
                           ? state->sd_io_mutex_holder
                           : kMutexHolderNone;
    const char* flush_phase = (state->sd_flush_phase != NULL)
                                ? state->sd_flush_phase
                                : kMutexHolderNone;
    ESP_LOGE(kTag,
             "SD I/O mutex timeout during %s; holder=%s held_ms=%" PRIu32
             " task=%s sd_phase=%s",
             phase,
             holder,
             held_ms,
             RuntimeGetCurrentTaskName(),
             flush_phase);
    const TickType_t dump_elapsed = now_ticks - state->last_lock_dump_ticks;
    if (state->last_lock_dump_ticks == 0 ||
        pdTICKS_TO_MS(dump_elapsed) >= kI2cLockDumpIntervalMs) {
      state->last_lock_dump_ticks = now_ticks;
      RuntimeDumpLocks(state, "sd_io_timeout");
    }
    MarkSdIoLockFailure(state);
    return false;
  }
  state->sd_io_mutex_holder = RuntimeGetCurrentTaskName();
  state->sd_io_mutex_hold_start_ticks = xTaskGetTickCount();
  return true;
}

/**
 * @brief Execute RuntimeSdFsUnlock.
 * @param state Parameter state.
 */
void
RuntimeSdFsUnlock(runtime_state_t* state)
{
  if (state == NULL || state->sd_io_mutex == NULL) {
    return;
  }
  const TickType_t now_ticks = xTaskGetTickCount();
  const uint32_t held_ms =
    RuntimeElapsedMs(state->sd_io_mutex_hold_start_ticks, now_ticks);
  RuntimeUpdateHoldMax(held_ms, &state->sd_io_mutex_hold_max_ms_observed);
  state->sd_io_mutex_holder = kMutexHolderNone;
  state->sd_io_mutex_hold_start_ticks = 0;
  (void)xSemaphoreGive(state->sd_io_mutex);
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
  return RuntimeSdFsLock(state, timeout_ticks);
}

/**
 * @brief Execute RuntimeSdIoUnlock.
 * @param state Parameter state.
 */
void
RuntimeSdIoUnlock(runtime_state_t* state)
{
  RuntimeSdFsUnlock(state);
}

bool
RuntimeSpiBusLockForSharedDevices(runtime_state_t* state,
                                  TickType_t timeout_ticks)
{
  if (state == NULL) {
    return false;
  }
  if (state->spi_bus_mutex == NULL) {
    ESP_LOGE(kTag, "SPI bus mutex unavailable");
    return false;
  }
  if (xSemaphoreTake(state->spi_bus_mutex, timeout_ticks) != pdTRUE) {
    ESP_LOGW(kTag, "SPI bus mutex timeout");
    return false;
  }
  return true;
}

void
RuntimeSpiBusUnlockForSharedDevices(runtime_state_t* state)
{
  if (state == NULL || state->spi_bus_mutex == NULL) {
    return;
  }
  (void)xSemaphoreGive(state->spi_bus_mutex);
}

static bool
RuntimeSpiBusLock(void* context, TickType_t timeout_ticks)
{
  return RuntimeSpiBusLockForSharedDevices((runtime_state_t*)context,
                                           timeout_ticks);
}

static void
RuntimeSpiBusUnlock(void* context)
{
  RuntimeSpiBusUnlockForSharedDevices((runtime_state_t*)context);
}

static bool
RuntimeFramLogLock(runtime_state_t* state, TickType_t timeout_ticks)
{
  if (state == NULL) {
    return false;
  }
  if (state->fram_log_mutex == NULL) {
    ESP_LOGE(kTag, "FRAM log mutex unavailable");
    return false;
  }
  if (xSemaphoreTake(state->fram_log_mutex, timeout_ticks) != pdTRUE) {
    state->fram_log_mutex_timeouts++;
    const TickType_t now_ticks = xTaskGetTickCount();
    const uint32_t held_ms =
      RuntimeElapsedMs(state->fram_log_mutex_hold_start_ticks, now_ticks);
    const char* holder_name = (state->fram_log_mutex_holder != NULL)
                                ? state->fram_log_mutex_holder
                                : kMutexHolderNone;
    const char* flush_phase = (state->sd_flush_phase != NULL)
                                ? state->sd_flush_phase
                                : kMutexHolderNone;
    ESP_LOGW(kTag,
             "FRAM log mutex timeout (holder=%s held_ms=%" PRIu32
             " task=%s sd_phase=%s)",
             holder_name,
             held_ms,
             RuntimeGetCurrentTaskName(),
             flush_phase);
    const TickType_t dump_elapsed = now_ticks - state->last_lock_dump_ticks;
    if (state->last_lock_dump_ticks == 0 ||
        pdTICKS_TO_MS(dump_elapsed) >= kI2cLockDumpIntervalMs) {
      state->last_lock_dump_ticks = now_ticks;
      RuntimeDumpLocks(state, "fram_log_timeout");
    }
    return false;
  }
  state->fram_log_mutex_holder = RuntimeGetCurrentTaskName();
  state->fram_log_mutex_hold_start_ticks = xTaskGetTickCount();
  return true;
}

static void
RuntimeFramLogUnlock(runtime_state_t* state)
{
  if (state == NULL || state->fram_log_mutex == NULL) {
    return;
  }
  const TickType_t now_ticks = xTaskGetTickCount();
  const uint32_t held_ms =
    RuntimeElapsedMs(state->fram_log_mutex_hold_start_ticks, now_ticks);
  RuntimeUpdateHoldMax(held_ms, &state->fram_log_mutex_hold_max_ms_observed);
  state->fram_log_mutex_holder = kMutexHolderNone;
  state->fram_log_mutex_hold_start_ticks = 0;
  (void)xSemaphoreGive(state->fram_log_mutex);
}

/**
 * @brief Execute RuntimeI2cLock.
 * @param timeout_ticks Parameter timeout_ticks.
 * @return Return the function result.
 */
bool
RuntimeI2cLock(TickType_t timeout_ticks)
{
  if (g_state.i2c_mutex == NULL) {
    ESP_LOGE(kTag, "I2C mutex unavailable");
    return false;
  }

  const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  if (g_state.i2c_mutex_owner_task != NULL &&
      g_state.i2c_mutex_owner_task == current_task) {
    g_state.i2c_mutex_same_task_reentry_count++;
    const TickType_t now_ticks = xTaskGetTickCount();
    const uint32_t held_ms =
      RuntimeElapsedMs(g_state.i2c_mutex_hold_start_ticks, now_ticks);
    const char* holder = (g_state.i2c_mutex_holder != NULL)
                           ? g_state.i2c_mutex_holder
                           : kMutexHolderNone;
    if (LogRateLimitAllow(&g_state.last_i2c_reentry_warn_ms,
                          kI2cTimeoutWarnIntervalMs)) {
      const uint32_t suppressed = g_state.i2c_mutex_reentry_suppressed_count;
      g_state.i2c_mutex_reentry_suppressed_count = 0;
      ESP_LOGW(kTag,
               "I2C mutex same-task reentry denied (holder=%s held_ms=%" PRIu32
               " task=%s reentry_count=%" PRIu32 " suppressed=%" PRIu32 ")",
               holder,
               held_ms,
               RuntimeGetCurrentTaskName(),
               g_state.i2c_mutex_same_task_reentry_count,
               suppressed);
    } else {
      g_state.i2c_mutex_reentry_suppressed_count++;
    }
    const TickType_t dump_elapsed =
      now_ticks - g_state.last_i2c_lock_dump_ticks;
    if (g_state.last_i2c_lock_dump_ticks == 0 ||
        pdTICKS_TO_MS(dump_elapsed) >= kI2cLockDumpIntervalMs) {
      g_state.last_i2c_lock_dump_ticks = now_ticks;
      RuntimeDumpLocks(&g_state, "i2c_same_task_reentry");
    }
    return false;
  }

  if (xSemaphoreTake(g_state.i2c_mutex, timeout_ticks) != pdTRUE) {
    g_state.i2c_mutex_timeouts++;
    const TickType_t now_ticks = xTaskGetTickCount();
    const uint32_t held_ms =
      RuntimeElapsedMs(g_state.i2c_mutex_hold_start_ticks, now_ticks);
    const char* holder = (g_state.i2c_mutex_holder != NULL)
                           ? g_state.i2c_mutex_holder
                           : kMutexHolderNone;
    const char* flush_phase = (g_state.sd_flush_phase != NULL)
                                ? g_state.sd_flush_phase
                                : kMutexHolderNone;
    if (LogRateLimitAllow(&g_state.last_i2c_timeout_warn_ms,
                          kI2cTimeoutWarnIntervalMs)) {
      const uint32_t suppressed = g_state.i2c_mutex_timeout_suppressed_count;
      g_state.i2c_mutex_timeout_suppressed_count = 0;
      ESP_LOGW(kTag,
               "I2C mutex timeout (holder=%s held_ms=%" PRIu32 " task=%s "
               "sd_phase=%s suppressed=%" PRIu32 ")",
               holder,
               held_ms,
               RuntimeGetCurrentTaskName(),
               flush_phase,
               suppressed);
    } else {
      g_state.i2c_mutex_timeout_suppressed_count++;
    }
    const TickType_t dump_elapsed =
      now_ticks - g_state.last_i2c_lock_dump_ticks;
    if (g_state.last_i2c_lock_dump_ticks == 0 ||
        pdTICKS_TO_MS(dump_elapsed) >= kI2cLockDumpIntervalMs) {
      g_state.last_i2c_lock_dump_ticks = now_ticks;
      RuntimeDumpLocks(&g_state, "i2c_timeout");
    }
    return false;
  }
  g_state.i2c_mutex_holder = RuntimeGetCurrentTaskName();
  g_state.i2c_mutex_owner_task = current_task;
  g_state.i2c_mutex_hold_start_ticks = xTaskGetTickCount();
  return true;
}

/**
 * @brief Execute RuntimeI2cUnlock.
 */
void
RuntimeI2cUnlock(void)
{
  if (g_state.i2c_mutex == NULL) {
    return;
  }
  const TickType_t now_ticks = xTaskGetTickCount();
  const uint32_t held_ms =
    RuntimeElapsedMs(g_state.i2c_mutex_hold_start_ticks, now_ticks);
  RuntimeUpdateHoldMax(held_ms, &g_state.i2c_mutex_hold_max_ms_observed);
  g_state.i2c_mutex_holder = kMutexHolderNone;
  g_state.i2c_mutex_owner_task = NULL;
  g_state.i2c_mutex_hold_start_ticks = 0;
  (void)xSemaphoreGive(g_state.i2c_mutex);
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

static StaticQueue_t g_export_outbox_queue_struct;
static StaticQueue_t g_broker_outbox_queue_struct;
static uint8_t* g_export_outbox_queue_storage = NULL;
static uint8_t* g_broker_outbox_queue_storage = NULL;
static uint8_t* g_export_outbox_pool_storage = NULL;
static uint8_t* g_broker_outbox_pool_storage = NULL;
static mem_pool_t g_export_outbox_pool;
static mem_pool_t g_broker_outbox_pool;

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

static void
DrainExportOutboxQueue(runtime_state_t* state)
{
  if (state == NULL || state->export_outbox == NULL) {
    return;
  }

  export_record_item_t* item = NULL;
  while (xQueueReceive(state->export_outbox, &item, 0) == pdTRUE) {
    if (item != NULL) {
      MemPoolFree(&g_export_outbox_pool, item);
    }
  }
}

static void
DrainBrokerOutboxQueue(runtime_state_t* state)
{
  if (state == NULL || state->broker_outbox == NULL) {
    return;
  }

  broker_publish_item_t* item = NULL;
  while (xQueueReceive(state->broker_outbox, &item, 0) == pdTRUE) {
    if (item != NULL) {
      MemPoolFree(&g_broker_outbox_pool, item);
    }
  }
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
ControlTickTimeSync(runtime_state_t* state);
static void
ControlTickRtcResync(runtime_state_t* state);
static void
ControlTickTopology(runtime_state_t* state, int64_t now_ms);
static void
NetTxTask(void* context);

static void
EnsureSdMounted(void);
static void
EnsureSdMountedLocked(runtime_state_t* state);

static void
ReduceSdMaxFreqOnEio(runtime_state_t* state, int errno_value);

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
    HeapEventLog("sd_recovered", "mounted", 0);
  }

  if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    if (FramLogGetBufferedRecords(&state->fram_log) > 0) {
      // We have backlog and SD is now available. Start draining immediately.
      state->sd_flush_pending = true;
      state->sd_start_drain_pending = true;
      state->sd_next_flush_allowed_ticks = 0;
    }
    RuntimeFramLogUnlock(state);
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
  state->sd_io_success_streak++;
  state->sd_io_error_streak = 0;
  if (state->sd_last_io_error_active &&
      state->sd_io_success_streak < kSdIoErrorClearThreshold) {
    return;
  }
  state->sd_last_io_error_active = false;
  state->sd_last_io_err = ESP_OK;
  state->sd_last_errno = 0;
  UpdateCachedBool(state, &state->cached_status.sd_io_error_active, false);
}

static void
UpdateFramOverrunActive(runtime_state_t* state, uint64_t overrun_total)
{
  if (state == NULL) {
    return;
  }
  const bool overrun_active = (overrun_total > state->fram_overrun_ack_total);
  if (overrun_active) {
    state->fram_overrun_active_streak++;
    state->fram_overrun_clear_streak = 0;
  } else {
    state->fram_overrun_clear_streak++;
    state->fram_overrun_active_streak = 0;
  }
  const bool cached_active = state->cached_status.fram_overrun_active;
  if (!cached_active && overrun_active &&
      state->fram_overrun_active_streak >= kFramOverrunActiveSetThreshold) {
    UpdateCachedBool(state, &state->cached_status.fram_overrun_active, true);
  } else if (cached_active && !overrun_active &&
             state->fram_overrun_clear_streak >=
               kFramOverrunActiveClearThreshold) {
    UpdateCachedBool(state, &state->cached_status.fram_overrun_active, false);
  }
}

/**
 * @brief Execute UpdateFramFillState.
 * @param state Parameter state.
 * @note Caller must hold fram_log_mutex.
 */
static void
UpdateFramFillState(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  if (!state->fram_available) {
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
  if ((mask & kDispAttnNtpFail) != 0u && health->ntp_fail_alert_active) {
    active |= kDispAttnNtpFail;
  }
  if ((mask & kDispAttnMeshDown) != 0u && !health->mesh_connected) {
    active |= kDispAttnMeshDown;
  }
  if ((mask & kDispAttnHeap) != 0u &&
      (health->heap_internal_warn || health->heap_internal_crit)) {
    active |= kDispAttnHeap;
  }
  if ((mask & kDispAttnSdSpace) != 0u &&
      (health->sd_space_reclaim_active || health->sd_out_of_space_active)) {
    active |= kDispAttnSdSpace;
  }

  return active;
}

/**
 * @brief Execute AttentionBitToCode.
 * @param bit Parameter bit.
 * @return Return the function result.
 */
static const char*
AttentionBitToCode(display_attention_bit_t bit,
                   const runtime_health_snapshot_t* snapshot)
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
    case kDispAttnNtpFail:
      return "NTP  ";
    case kDispAttnMeshDown:
      return "MESH ";
    case kDispAttnHeap:
      return "HEAP ";
    case kDispAttnSdSpace:
      if (snapshot != NULL && snapshot->sd_out_of_space_active) {
        return "SDFUL";
      }
      return "SDROT";
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
    const int32_t whole = abs_tenths / 10;
    const int32_t frac = abs_tenths % 10;
    const char* sign = (tenths < 0) ? "-" : "";
    snprintf(
      out, out_len, "%s%ld.%01ld%c", sign, (long)whole, (long)frac, unit_char);
    return;
  }

  const int32_t whole = abs_tenths / 10;
  const int32_t frac = abs_tenths % 10;
  if (tenths < 0) {
    snprintf(out, out_len, "-%ld.%ld%c", (long)whole, (long)frac, unit_char);
  } else {
    snprintf(out, out_len, "%ld.%ld%c", (long)whole, (long)frac, unit_char);
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
  const int64_t now_utc = time_valid ? (int64_t)time(NULL) : 0;
  UpdateCalibrationDueState(state, time_valid, now_utc);
  if (!time_valid) {
    UpdateCachedInt32(state, &state->cached_status.utc_offset_sec, 0);
    UpdateCachedBool(state, &state->cached_status.dst_in_effect, false);
    return;
  }

  const time_t now = (time_t)now_utc;
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

static void
UpdateCalibrationDueState(runtime_state_t* state,
                          bool time_valid,
                          int64_t now_utc)
{
  if (state == NULL) {
    return;
  }

  if (time_valid) {
    if (!state->cal_time_stable) {
      if (state->cal_last_time_valid_utc == 0) {
        state->cal_last_time_valid_utc = now_utc;
      }
      if (now_utc - state->cal_last_time_valid_utc >= kCalTimeStableDelaySec) {
        state->cal_time_stable = true;
      }
    } else {
      state->cal_last_time_valid_utc = now_utc;
    }
  } else {
    state->cal_time_stable = false;
    state->cal_last_time_valid_utc = 0;
  }

  const bool cal_due_check_suspended = !time_valid;
  bool cal_overdue = false;
  if (time_valid) {
    const int64_t last_cal = (state->settings.cal_last_override_utc != 0)
                               ? state->settings.cal_last_override_utc
                               : state->settings.cal_last_utc;
    const uint16_t freq_count = (state->settings.cal_due_override_count != 0)
                                  ? state->settings.cal_due_override_count
                                  : state->settings.cal_due_count;
    const uint8_t freq_unit = (state->settings.cal_due_override_unit != 0)
                                ? state->settings.cal_due_override_unit
                                : state->settings.cal_due_unit;
    if (last_cal != 0 && freq_count != 0) {
      const int64_t due_utc =
        CalComputeDueDateUtc(last_cal, freq_count, (cal_due_unit_t)freq_unit);
      const int64_t today_midnight = GetUtcMidnightEpochNow();
      cal_overdue = (due_utc != 0 && today_midnight > due_utc);
    }
  }

  state->cal_due_check_suspended = cal_due_check_suspended;
  state->cal_overdue = cal_overdue;
  UpdateCachedBool(state,
                   &state->cached_status.cal_due_check_suspended,
                   cal_due_check_suspended);
  UpdateCachedBool(
    state, &state->cached_status.cal_overdue, state->cal_overdue);
  UpdateCachedBool(
    state, &state->cached_status.cal_time_stable, state->cal_time_stable);
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
  // SensorTask is the only owner of MAX31865 conversions. DisplayTask
  // must only read RuntimeHealth/cached values; never trigger sensor reads.
  // Guardrail: grep -R
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
        state->spi_pause_ack_mask |= SPI_PAUSE_ACK_DISPLAY;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (state->spi_pause_requested) {
      state->spi_pause_ack_mask |= SPI_PAUSE_ACK_DISPLAY;
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(50));
      continue;
    }
    state->spi_pause_ack_mask &= ~SPI_PAUSE_ACK_DISPLAY;

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
    display_attention_bit_t error_bits[kDispAttnItemCount];
    display_attention_bit_t warn_bits[kDispAttnItemCount];
    size_t error_count = 0;
    size_t warn_count = 0;
    display_attention_mask_t error_mask = 0;
    display_attention_mask_t warn_mask = 0;
    const display_attention_item_t items[] = {
      kDispAttnItemSdOut,    kDispAttnItemSdIo,    kDispAttnItemFramOvr,
      kDispAttnItemRtdFault, kDispAttnItemTimeBad, kDispAttnItemNtpFail,
      kDispAttnItemMeshDown, kDispAttnItemHeap,    kDispAttnItemSdSpace,
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
        const char* text = AttentionBitToCode(error_bits[code_index], &health);
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

    const bool operator_hold = state->cached_status.operator_hold_latched;
    const bool sd_safe_to_remove = state->cached_status.sd_safe_to_remove;
    if (operator_hold) {
      const char* text = sd_safe_to_remove ? "SAFE " : "HOLD ";
      if (strncmp(last_text, text, sizeof(last_text)) != 0) {
        Max7219DisplaySetText(&state->display, text);
        snprintf(last_text, sizeof(last_text), "%s", text);
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
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

      const char* text =
        AttentionBitToCode(warn_bits[warn_code_index], &health);
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
      if (stop_requested) {
        const bool unsafe = !sd_safe_to_remove;
        if (unsafe && ((now_ms / 500u) % 2u) == 0u) {
          if (last_text[0] != '\0') {
            Max7219DisplayClear(&state->display);
            last_text[0] = '\0';
          }
        } else {
          const char* text = unsafe ? "HOLD " : "SAFE ";
          if (strncmp(last_text, text, sizeof(last_text)) != 0) {
            Max7219DisplaySetText(&state->display, text);
            snprintf(last_text, sizeof(last_text), "%s", text);
          }
        }
      } else {
        const char* text = "IDLE ";
        if (strncmp(last_text, text, sizeof(last_text)) != 0) {
          Max7219DisplaySetText(&state->display, text);
          snprintf(last_text, sizeof(last_text), "%s", text);
        }
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
    const bool show_cal_overdue = state->cached_status.time_valid &&
                                  state->cached_status.cal_overdue &&
                                  ((now_ms / 750u) % 2u) == 0u;
    if (show_cal_overdue) {
      snprintf(text, sizeof(text), "CAL ");
    } else {
      FormatTemperatureText(text,
                            sizeof(text),
                            temp_milli_c,
                            RuntimeGetEffectiveDisplayUnits(),
                            temp_valid);
    }
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
  runtime_state_t* state = &g_state;
  if (context == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (addr > 0xFFFFu) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
    return ESP_ERR_TIMEOUT;
  }
  RuntimeI2cOpBegin("fram_read", addr, len, "FramI2cReadAdapter");
  if (state->i2c_bus.initialized && !I2cBusLooksIdle(&state->i2c_bus)) {
    RuntimeI2cOpEnd();
    RuntimeI2cUnlock();
    (void)RuntimeRecoverI2cBus(
      state, "fram preflight bus busy", ESP_ERR_TIMEOUT);
    if (!RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    RuntimeI2cOpBegin("fram_read", addr, len, "FramI2cReadAdapter");
  }
  esp_err_t result =
    FramI2cRead((const fram_i2c_t*)context, (uint16_t)addr, out, len);
  if (result == ESP_ERR_TIMEOUT || result == ESP_ERR_INVALID_STATE ||
      result == ESP_ERR_INVALID_RESPONSE) {
    RuntimeI2cOpEnd();
    RuntimeI2cUnlock();
    (void)RuntimeRecoverI2cBus(state, "fram io error", result);
    if (!RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    RuntimeI2cOpBegin("fram_read", addr, len, "FramI2cReadAdapter");
    result = FramI2cRead((const fram_i2c_t*)context, (uint16_t)addr, out, len);
  }
  RuntimeI2cOpEnd();
  RuntimeI2cUnlock();
  return result;
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
  runtime_state_t* state = &g_state;
  if (context == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (addr > 0xFFFFu) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
    return ESP_ERR_TIMEOUT;
  }
  RuntimeI2cOpBegin("fram_write", addr, len, "FramI2cWriteAdapter");
  if (state->i2c_bus.initialized && !I2cBusLooksIdle(&state->i2c_bus)) {
    RuntimeI2cOpEnd();
    RuntimeI2cUnlock();
    (void)RuntimeRecoverI2cBus(
      state, "fram preflight bus busy", ESP_ERR_TIMEOUT);
    if (!RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    RuntimeI2cOpBegin("fram_write", addr, len, "FramI2cWriteAdapter");
  }
  esp_err_t result =
    FramI2cWrite((const fram_i2c_t*)context, (uint16_t)addr, data, len);
  if (result == ESP_ERR_TIMEOUT || result == ESP_ERR_INVALID_STATE ||
      result == ESP_ERR_INVALID_RESPONSE) {
    RuntimeI2cOpEnd();
    RuntimeI2cUnlock();
    (void)RuntimeRecoverI2cBus(state, "fram io error", result);
    if (!RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    RuntimeI2cOpBegin("fram_write", addr, len, "FramI2cWriteAdapter");
    result =
      FramI2cWrite((const fram_i2c_t*)context, (uint16_t)addr, data, len);
  }
  RuntimeI2cOpEnd();
  RuntimeI2cUnlock();
  return result;
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
 * @brief Execute BuildDateStringFromEpoch.
 * @param epoch Parameter epoch.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
static void
BuildDateStringFromEpoch(int64_t epoch, char* out, size_t out_size)
{
  time_t time_seconds = (time_t)epoch;
  struct tm time_info;
  gmtime_r(&time_seconds, &time_info);
  strftime(out, out_size, "%Y-%m-%dZ", &time_info);
}

static void
FormatUtcTimestamp(int64_t epoch_utc, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  time_t time_seconds = (time_t)epoch_utc;
  struct tm time_info;
  gmtime_r(&time_seconds, &time_info);
  strftime(out, out_size, "%Y-%m-%dT%H:%M:%SZ", &time_info);
}

static bool
RuntimeCalibrationApplied(const runtime_state_t* state)
{
  if (state == NULL) {
    return false;
  }
  const app_settings_t* settings = &state->settings;
  return (settings->calibration_points_count > 0u) &&
         settings->calibration.is_valid && !state->cal_overdue;
}

static bool
RuntimeBuildPtlogHeaderInternal(const runtime_state_t* state,
                                int64_t epoch_utc,
                                ptlog_header_t* header_out,
                                uint32_t* signature_out)
{
  if (state == NULL || header_out == NULL) {
    return false;
  }
  memset(header_out, 0, sizeof(*header_out));

  const app_settings_t* settings = &state->settings;
  const esp_app_desc_t* app_desc = esp_app_get_description();
  const int64_t now_utc = (epoch_utc > 0) ? epoch_utc : (int64_t)time(NULL);
  FormatUtcTimestamp(
    now_utc, header_out->created_utc, sizeof(header_out->created_utc));

  snprintf(header_out->device_serial,
           sizeof(header_out->device_serial),
           "%s",
           settings->unit_serial);
  FormatMacString(
    state->local_mac, header_out->device_mac, sizeof(header_out->device_mac));
  snprintf(header_out->device_role,
           sizeof(header_out->device_role),
           "%s",
           AppSettingsRoleToString(state->node_role_active));
  snprintf(header_out->firmware_project,
           sizeof(header_out->firmware_project),
           "%s",
           app_desc->project_name);
  snprintf(header_out->firmware_version,
           sizeof(header_out->firmware_version),
           "%s",
           app_desc->version);
  snprintf(header_out->firmware_build_date,
           sizeof(header_out->firmware_build_date),
           "%s",
           app_desc->date);
  snprintf(header_out->firmware_build_time,
           sizeof(header_out->firmware_build_time),
           "%s",
           app_desc->time);
  snprintf(header_out->esp_idf_ver,
           sizeof(header_out->esp_idf_ver),
           "%s",
           esp_get_idf_version());
  snprintf(header_out->timezone_posix,
           sizeof(header_out->timezone_posix),
           "%s",
           settings->tz_posix);
  header_out->dst_enabled = settings->dst_enabled;

  const int64_t last_cal_utc = (settings->cal_last_override_utc != 0)
                                 ? settings->cal_last_override_utc
                                 : settings->cal_last_utc;
  if (last_cal_utc > 0) {
    FormatUtcTimestamp(
      last_cal_utc, header_out->cal_last_utc, sizeof(header_out->cal_last_utc));
  } else {
    snprintf(
      header_out->cal_last_utc, sizeof(header_out->cal_last_utc), "unknown");
  }

  const uint16_t due_count = (settings->cal_due_override_count != 0)
                               ? settings->cal_due_override_count
                               : settings->cal_due_count;
  const uint8_t due_unit = (settings->cal_due_override_unit != 0)
                             ? settings->cal_due_override_unit
                             : settings->cal_due_unit;
  const char* due_unit_text = "unknown";
  if (due_unit == (uint8_t)CAL_DUE_UNIT_DAYS) {
    due_unit_text = "days";
  } else if (due_unit == (uint8_t)CAL_DUE_UNIT_MONTHS) {
    due_unit_text = "months";
  } else if (due_unit == (uint8_t)CAL_DUE_UNIT_YEARS) {
    due_unit_text = "years";
  }
  snprintf(header_out->cal_due_rule,
           sizeof(header_out->cal_due_rule),
           "%u_%s",
           (unsigned)due_count,
           due_unit_text);

  const int64_t due_utc =
    (last_cal_utc > 0 && due_count > 0 &&
     due_unit >= (uint8_t)CAL_DUE_UNIT_DAYS &&
     due_unit <= (uint8_t)CAL_DUE_UNIT_YEARS)
      ? CalComputeDueDateUtc(last_cal_utc, due_count, (cal_due_unit_t)due_unit)
      : 0;
  if (due_utc > 0) {
    FormatUtcTimestamp(
      due_utc, header_out->cal_due_utc, sizeof(header_out->cal_due_utc));
  } else {
    snprintf(
      header_out->cal_due_utc, sizeof(header_out->cal_due_utc), "unknown");
  }

  snprintf(header_out->cal_points_count,
           sizeof(header_out->cal_points_count),
           "%u",
           (unsigned)settings->calibration_points_count);
  snprintf(header_out->cal_applied,
           sizeof(header_out->cal_applied),
           "%u",
           RuntimeCalibrationApplied(state) ? 1u : 0u);
  const char* cal_method =
    (settings->cal_method[0] != '\0') ? settings->cal_method : "<unset>";
  snprintf(
    header_out->cal_method, sizeof(header_out->cal_method), "%s", cal_method);
  snprintf(header_out->cal_context,
           sizeof(header_out->cal_context),
           "net_mode=%d",
           (int)state->net_mode_active);

  if (signature_out != NULL) {
    *signature_out = PtlogComputeHeaderSignature(header_out);
  }
  return true;
}

bool
RuntimeBuildPtlogHeader(int64_t epoch_utc,
                        ptlog_header_t* header_out,
                        uint32_t* signature_out)
{
  runtime_state_t* state = RuntimeGetState();
  return RuntimeBuildPtlogHeaderInternal(
    state, epoch_utc, header_out, signature_out);
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
  state->net_mode_active =
    AppSettingsGetEffectiveNetMode(state->settings.node_role,
                                   state->settings.net_mode);
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

static esp_err_t
RuntimeStartMeshTransport(runtime_state_t* state)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (state->mesh_started && MeshTransportIsStarted(&state->mesh)) {
    return ESP_OK;
  }

  const bool is_root = (state->settings.node_role == APP_NODE_ROLE_ROOT);
  const bool allow_children = state->settings.allow_children;
  const bool router_disabled = AppNetConfigGetMeshDisableRouter();
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

  esp_err_t mesh_result =
    MeshTransportStart(&state->mesh,
                       is_root,
                       allow_children,
                       router_ssid,
                       router_password,
                       is_root ? &RootRecordRxCallback : NULL,
                       NULL,
                       (is_root && state->root_publish_consumer_active)
                         ? &RootPublishRecordRxCallback
                         : NULL,
                       NULL,
                       &state->time_sync);
  if (mesh_result == ESP_OK) {
    state->mesh_started = true;
  } else {
    state->mesh_started = false;
  }
  return mesh_result;
}

static void
RuntimeStopMeshTransport(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  if (!state->mesh_started && !MeshTransportIsStarted(&state->mesh)) {
    return;
  }
  (void)MeshTransportStop(&state->mesh);
  state->mesh_started = false;
}

esp_err_t
RuntimeApplyNetMode(app_net_mode_t mode)
{
  const app_net_mode_t effective_mode =
    AppSettingsGetEffectiveNetMode(g_state.settings.node_role, mode);
  g_state.net_mode_active = effective_mode;
  if (effective_mode == APP_NET_MODE_MESH) {
    return RuntimeStartMeshTransport(&g_state);
  }
  RuntimeStopMeshTransport(&g_state);
  return ESP_OK;
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

  export_record_item_t* item =
    (export_record_item_t*)MemPoolAlloc(&g_export_outbox_pool);
  if (item == NULL) {
    state->export_drop_count++;
    UpdateCachedUint32(
      state, &state->cached_status.export_drop_count, state->export_drop_count);
    if (LogRateLimitAllow(&state->last_export_drop_log_ms,
                          kExportLogRateLimitMs)) {
      ESP_LOGW(kTag, "export outbox pool empty; dropping network export");
    }
    return;
  }

  memset(item, 0, sizeof(*item));
  memcpy(item->src_mac, src_mac, sizeof(item->src_mac));
  item->record = *record;

  if (xQueueSend(state->export_outbox, &item, 0) != pdTRUE) {
    state->export_drop_count++;
    UpdateCachedUint32(
      state, &state->cached_status.export_drop_count, state->export_drop_count);
    if (LogRateLimitAllow(&state->last_export_drop_log_ms,
                          kExportLogRateLimitMs)) {
      ESP_LOGW(kTag, "export outbox full; dropping network export");
    }
    MemPoolFree(&g_export_outbox_pool, item);
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

  broker_publish_item_t* publish_item =
    (broker_publish_item_t*)MemPoolAlloc(&g_broker_outbox_pool);
  if (publish_item == NULL) {
    state->broker_drop_count++;
    UpdateCachedUint32(
      state, &state->cached_status.broker_drop_count, state->broker_drop_count);
    if (LogRateLimitAllow(&state->last_broker_drop_log_ms,
                          kExportLogRateLimitMs)) {
      ESP_LOGW(kTag, "broker outbox pool empty; dropping publish");
    }
    return;
  }

  memset(publish_item, 0, sizeof(*publish_item));
  if (!BuildMqttTopic(state->mqtt_topic_prefix_active,
                      node_id,
                      publish_item->topic,
                      sizeof(publish_item->topic))) {
    state->broker_send_fail_count++;
    UpdateCachedUint32(state,
                       &state->cached_status.broker_send_fail_count,
                       state->broker_send_fail_count);
    MemPoolFree(&g_broker_outbox_pool, publish_item);
    return;
  }

  size_t payload_len = 0;
  if (!BuildMqttPayload(record,
                        node_id,
                        publish_item->payload,
                        sizeof(publish_item->payload),
                        &payload_len)) {
    state->broker_send_fail_count++;
    UpdateCachedUint32(state,
                       &state->cached_status.broker_send_fail_count,
                       state->broker_send_fail_count);
    MemPoolFree(&g_broker_outbox_pool, publish_item);
    return;
  }

  publish_item->payload_len = (uint16_t)payload_len;

  if (xQueueSend(state->broker_outbox, &publish_item, 0) != pdTRUE) {
    state->broker_drop_count++;
    UpdateCachedUint32(
      state, &state->cached_status.broker_drop_count, state->broker_drop_count);
    if (LogRateLimitAllow(&state->last_broker_drop_log_ms,
                          kExportLogRateLimitMs)) {
      ESP_LOGW(kTag, "broker outbox full; dropping publish");
    }
    MemPoolFree(&g_broker_outbox_pool, publish_item);
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
    uint64_t leaf_id = PackMacToLeafId(from->addr);
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
 * @brief Execute PackMacToLeafId.
 * @param mac Parameter mac.
 * @return Return the function result.
 */
static uint64_t
PackMacToLeafId(const uint8_t mac[6])
{
  uint64_t leaf_id = 0;
  for (int i = 0; i < 6; ++i) {
    leaf_id = (leaf_id << 8) | mac[i];
  }
  return leaf_id;
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

static void
RecordFramCorruption(runtime_state_t* state,
                     uint32_t offset,
                     uint32_t slot,
                     uint32_t addr,
                     fram_log_validate_result_t reason,
                     const log_record_t* record,
                     uint16_t actual_crc)
{
  if (state == NULL || record == NULL) {
    return;
  }
  state->fram_corrupt_last_offset = offset;
  state->fram_corrupt_last_slot = slot;
  state->fram_corrupt_last_addr = addr;
  state->fram_corrupt_last_magic = record->magic;
  state->fram_corrupt_last_schema = record->schema_version;
  state->fram_corrupt_last_exp_crc = record->crc16_ccitt;
  state->fram_corrupt_last_act_crc = actual_crc;
  state->fram_corrupt_last_reason = reason;
}

static void
RuntimeErrlogLatchEvent(runtime_state_t* state, uint16_t code, bool resolved)
{
  if (state == NULL) {
    return;
  }
  uint64_t mask = 0;
  if (!FramErrorLogGetCodeMask(code, &mask)) {
    return;
  }
  taskENTER_CRITICAL(&state->errlog_latch_lock);
  if (resolved) {
    state->errlog_pending_resolved_mask |= mask;
  } else {
    state->errlog_pending_active_mask |= mask;
  }
  taskEXIT_CRITICAL(&state->errlog_latch_lock);
}

static void
RuntimeFlushPendingErrlog(runtime_state_t* state)
{
  if (state == NULL || !state->fram_error_log.initialized) {
    return;
  }
  uint64_t pending_active = 0;
  uint64_t pending_resolved = 0;
  taskENTER_CRITICAL(&state->errlog_latch_lock);
  pending_active = state->errlog_pending_active_mask;
  pending_resolved = state->errlog_pending_resolved_mask;
  taskEXIT_CRITICAL(&state->errlog_latch_lock);

  if (pending_active == 0 && pending_resolved == 0) {
    return;
  }

  int64_t epoch_seconds = 0;
  int32_t millis = 0;
  TimeSyncGetNow(&epoch_seconds, &millis);
  uint32_t epoch_u32 = (epoch_seconds > 0) ? (uint32_t)epoch_seconds : 0u;
  uint16_t millis_u16 = (millis >= 0) ? (uint16_t)millis : 0u;

  for (uint16_t code = 0; code < 64u; ++code) {
    uint64_t mask = 0;
    if (!FramErrorLogGetCodeMask(code, &mask)) {
      continue;
    }
    if ((pending_active & mask) != 0) {
      bool logged = false;
      const esp_err_t result = FramErrorLogAppendActive(
        &state->fram_error_log, code, 0, 0, epoch_u32, millis_u16, &logged);
      if (result == ESP_OK) {
        taskENTER_CRITICAL(&state->errlog_latch_lock);
        state->errlog_pending_active_mask &= ~mask;
        taskEXIT_CRITICAL(&state->errlog_latch_lock);
      }
    }
    if ((pending_resolved & mask) != 0) {
      bool logged = false;
      const esp_err_t result = FramErrorLogAppendResolved(
        &state->fram_error_log, code, 0, 0, epoch_u32, millis_u16, &logged);
      if (result == ESP_OK && logged) {
        taskENTER_CRITICAL(&state->errlog_latch_lock);
        state->errlog_pending_resolved_mask &= ~mask;
        taskEXIT_CRITICAL(&state->errlog_latch_lock);
      }
    }
  }
}

static esp_err_t
LogFramErrorEvent(runtime_state_t* state,
                  uint16_t code,
                  bool resolved,
                  int32_t detail0,
                  int32_t detail1)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!state->fram_error_log.initialized) {
    if (code < 64u) {
      RuntimeErrlogLatchEvent(state, code, resolved);
      return ESP_OK;
    }
    int64_t epoch_seconds = 0;
    int32_t millis = 0;
    TimeSyncGetNow(&epoch_seconds, &millis);
    const uint32_t epoch_u32 =
      (epoch_seconds > 0) ? (uint32_t)epoch_seconds : 0;
    const uint16_t millis_u16 = (millis >= 0) ? (uint16_t)millis : 0u;
    RuntimeRtcErrlogLatchPush(
      code, resolved, detail0, detail1, epoch_u32, millis_u16);
    return ESP_OK;
  }
  int64_t epoch_seconds = 0;
  int32_t millis = 0;
  TimeSyncGetNow(&epoch_seconds, &millis);
  uint32_t epoch_u32 = (epoch_seconds > 0) ? (uint32_t)epoch_seconds : 0u;
  uint16_t millis_u16 = (millis >= 0) ? (uint16_t)millis : 0u;
  bool logged = false;
  const esp_err_t result =
    resolved ? FramErrorLogAppendResolved(&state->fram_error_log,
                                          code,
                                          detail0,
                                          detail1,
                                          epoch_u32,
                                          millis_u16,
                                          &logged)
             : FramErrorLogAppendActive(&state->fram_error_log,
                                        code,
                                        detail0,
                                        detail1,
                                        epoch_u32,
                                        millis_u16,
                                        &logged);
  if (result != ESP_OK) {
    RuntimeRtcErrlogLatchPush(
      code, resolved, detail0, detail1, epoch_u32, millis_u16);
  }
  return result;
}

esp_err_t
RuntimeMaybeInitFramErrorLog(runtime_state_t* state)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (state->fram_error_log.initialized) {
    return ESP_OK;
  }
  if (!state->fram_available) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!state->i2c_bus.initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
    return ESP_ERR_TIMEOUT;
  }
  const bool fram_ok = RuntimeVerifyFram(state);
  RuntimeI2cUnlock();
  if (!fram_ok) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  const esp_err_t result = FramErrorLogInit(&state->fram_error_log,
                                            state->fram_io,
                                            FRAM_ERRLOG_BASE,
                                            FRAM_ERRLOG_BYTES);
  if (result != ESP_OK) {
    return result;
  }
  if (state->fram_error_log.state == FRAM_ERRLOG_STATE_CORRUPT &&
      !state->errlog_corrupt_alerted) {
    state->errlog_corrupt_alerted = true;
    ESP_LOGW(kTag, "FRAM errlog corrupt; manual clear required");
    int64_t now_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    int64_t now_ms = esp_timer_get_time() / 1000;
    (void)RuntimeEnqueueSystemErrorNote(
      state, ALERT_SYSTEM_CODE_ERROR_FRAM_ERRLOG, false, now_epoch, now_ms);
  }
  RuntimeRtcErrlogLatchFlushToFram(state);
  RuntimeFlushPendingErrlog(state);
  return ESP_OK;
}

static void
RuntimeFramRetryTick(runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL || state->fram_available) {
    return;
  }
  if (state->fram_next_retry_ms != 0 && now_ms < state->fram_next_retry_ms) {
    return;
  }
  if (!state->i2c_bus.initialized) {
    state->fram_next_retry_ms = now_ms + kFramRetryIntervalMs;
    return;
  }
  if (!RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
    state->fram_next_retry_ms = now_ms + kFramRetryIntervalMs;
    return;
  }

  esp_err_t i2c_result = FramI2cInit(&state->fram_i2c,
                                     state->i2c_bus.handle,
                                     &state->i2c_bus,
                                     (uint8_t)CONFIG_APP_FRAM_I2C_ADDR,
                                     CONFIG_APP_FRAM_SIZE_BYTES,
                                     state->i2c_bus.frequency_hz);
  if (i2c_result == ESP_OK && !RuntimeVerifyFram(state)) {
    i2c_result = ESP_ERR_INVALID_RESPONSE;
  }
  RuntimeI2cUnlock();

  if (i2c_result != ESP_OK) {
    RuntimeSetFramUnavailable(state, "retry", i2c_result, now_ms);
    return;
  }

  if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    state->fram_next_retry_ms = now_ms + kFramRetryIntervalMs;
    return;
  }

  esp_err_t fram_log_result =
    FramLogInit(&state->fram_log, state->fram_io, FRAM_DATA_BYTES);
  RuntimeFramLogUnlock(state);
  if (fram_log_result != ESP_OK) {
    RuntimeSetFramUnavailable(state, "retry log init", fram_log_result, now_ms);
    return;
  }

  state->fram_available = true;
  state->fram_next_retry_ms = 0;
  RuntimeSyncFramFallbackCounters(state);
  if (state->cached_status.fram_io_error_active) {
    LogFramErrorEvent(state, ERROR_FRAM_IO_FAIL, true, 0, 0);
    UpdateCachedBool(state, &state->cached_status.fram_io_error_active, false);
  }

  if (!state->fram_error_log.initialized) {
    esp_err_t errlog_result = FramErrorLogInit(&state->fram_error_log,
                                               state->fram_io,
                                               FRAM_ERRLOG_BASE,
                                               FRAM_ERRLOG_BYTES);
    if (errlog_result != ESP_OK) {
      ESP_LOGW(kTag,
               "FramErrorLogInit retry failed: %s",
               esp_err_to_name(errlog_result));
    } else {
      RuntimeRtcErrlogLatchFlushToFram(state);
      RuntimeFlushPendingErrlog(state);
      if (state->fram_error_log.state == FRAM_ERRLOG_STATE_CORRUPT &&
          !state->errlog_corrupt_alerted) {
        state->errlog_corrupt_alerted = true;
        ESP_LOGW(kTag, "FRAM errlog corrupt; manual clear required");
        int64_t now_epoch =
          TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
        int64_t now_ms = esp_timer_get_time() / 1000;
        (void)RuntimeEnqueueSystemErrorNote(
          state, ALERT_SYSTEM_CODE_ERROR_FRAM_ERRLOG, false, now_epoch, now_ms);
      }
    }
  }

  if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    UpdateFramFillState(state);
    RuntimeFramLogUnlock(state);
  }
  RestoreTimeJumpBackPendingFromFram(state);
  ESP_LOGI(kTag, "FRAM retry succeeded");
}

static bool
RuntimeVerifyFram(runtime_state_t* state)
{
  if (state == NULL || !state->fram_i2c.initialized) {
    return false;
  }
  uint8_t before[4] = { 0 };
  uint8_t after[4] = { 0 };
  esp_err_t read_result =
    FramI2cRead(&state->fram_i2c, 0, before, sizeof(before));
  if (read_result != ESP_OK) {
    return false;
  }
  esp_err_t write_result =
    FramI2cWrite(&state->fram_i2c, 0, before, sizeof(before));
  if (write_result != ESP_OK) {
    return false;
  }
  esp_err_t verify_result =
    FramI2cRead(&state->fram_i2c, 0, after, sizeof(after));
  if (verify_result != ESP_OK) {
    return false;
  }
  return memcmp(before, after, sizeof(before)) == 0;
}

/**
 * @brief Execute RuntimeRecoverI2cBusCommon.
 * @param state Parameter state.
 * @param reason Parameter reason.
 * @param last_error Parameter last_error.
 * @param lock_held Parameter lock_held.
 * @return Return the function result.
 */
static bool
RuntimeRecoverI2cBusCommon(runtime_state_t* state,
                           const char* reason,
                           esp_err_t last_error,
                           bool lock_held)
{
  if (state == NULL) {
    return false;
  }
  if (state->i2c_recovery_in_progress) {
    return false;
  }
  state->i2c_recovery_in_progress = true;

  if (!lock_held && !RuntimeI2cLock(kI2cRecoveryLockTimeoutTicks)) {
    ESP_LOGW(kTag, "I2C recovery lock timeout");
    state->i2c_recovery_in_progress = false;
    return false;
  }

  ESP_LOGW(kTag,
           "I2C recovery start (%s): %s",
           (reason != NULL) ? reason : "unknown",
           esp_err_to_name(last_error));
  HeapEventLog(
    "i2c_recovery_start", (reason != NULL) ? reason : "unknown", last_error);
  UpdateCachedBool(state, &state->cached_status.i2c_recovery_active, true);

  const i2c_port_t port = state->i2c_bus.port;
  const int sda_gpio = state->i2c_bus.sda_gpio;
  const int scl_gpio = state->i2c_bus.scl_gpio;
  const uint32_t frequency_hz = state->i2c_bus.frequency_hz;

  esp_err_t recover_result = I2cBusRecoverLines(sda_gpio, scl_gpio);
  if (recover_result != ESP_OK) {
    ESP_LOGW(
      kTag, "I2C line recovery failed: %s", esp_err_to_name(recover_result));
  }

  (void)TimeSyncDeinit(&state->time_sync);
  (void)FramI2cDeinit(&state->fram_i2c);

  esp_err_t result = I2cBusDeinit(&state->i2c_bus);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "I2C bus deinit failed: %s", esp_err_to_name(result));
    goto recovery_failed;
  }

  result = I2cBusInit(&state->i2c_bus, port, sda_gpio, scl_gpio, frequency_hz);
  if (result != ESP_OK) {
    goto recovery_failed;
  }

  result = TimeSyncInit(
    &state->time_sync, &state->i2c_bus, (uint8_t)CONFIG_APP_DS3231_I2C_ADDR);
  if (result != ESP_OK) {
    goto recovery_failed;
  }

  result = FramI2cInit(&state->fram_i2c,
                       state->i2c_bus.handle,
                       &state->i2c_bus,
                       (uint8_t)CONFIG_APP_FRAM_I2C_ADDR,
                       CONFIG_APP_FRAM_SIZE_BYTES,
                       state->i2c_bus.frequency_hz);
  if (result != ESP_OK) {
    goto recovery_failed;
  }

  if (!RuntimeVerifyFram(state)) {
    result = ESP_ERR_INVALID_RESPONSE;
    goto recovery_failed;
  }

  state->fram_available = true;
  state->fram_next_retry_ms = 0;
  RuntimeSyncFramFallbackCounters(state);
  UpdateCachedBool(state, &state->cached_status.i2c_recovery_active, false);
  UpdateCachedBool(state, &state->cached_status.fram_io_error_active, false);
  state->fram_append_fail_streak = 0;
  state->fram_crc_fail_streak = 0;
  state->i2c_recovery_in_progress = false;
  if (!lock_held) {
    RuntimeI2cUnlock();
  }
  LogFramErrorEvent(
    state, ERROR_I2C_RECOVERY_START, false, (int32_t)last_error, 0);
  LogFramErrorEvent(state, ERROR_I2C_RECOVERY_START, true, 0, 0);
  LogFramErrorEvent(state, ERROR_FRAM_IO_FAIL, true, 0, 0);
  LogFramErrorEvent(state, ERROR_I2C_RECOVERY_SUCCESS, true, 0, 0);
  ESP_LOGI(kTag, "I2C recovery succeeded");
  HeapEventLog("i2c_recovery_ok", (reason != NULL) ? reason : "unknown", 0);
  return true;

recovery_failed:
  HeapEventLog(
    "i2c_recovery_fail", (reason != NULL) ? reason : "unknown", result);
  ESP_LOGE(kTag, "I2C recovery failed: %s", esp_err_to_name(result));
  if (!lock_held) {
    RuntimeI2cUnlock();
  }
  state->i2c_recovery_in_progress = false;
  LogFramErrorEvent(
    state, ERROR_I2C_RECOVERY_START, false, (int32_t)last_error, 0);
  LogFramErrorEvent(
    state, ERROR_I2C_RECOVERY_FAILED, false, (int32_t)result, 0);
  const int64_t event_uptime_ms = esp_timer_get_time() / 1000;
  const int64_t event_epoch =
    TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
  int status = 0;
  int retry_after_seconds = -1;
  esp_err_t ntfy_err = ESP_OK;
  alert_ntfy_result_t ntfy_result = RuntimeAttemptPreRebootAlertSendDetailed(
    state,
    ALERT_SYSTEM_CODE_ERROR_I2C_RECOVERY,
    event_epoch,
    event_uptime_ms,
    &retry_after_seconds,
    &status,
    &ntfy_err);
  vTaskDelay(pdMS_TO_TICKS(200));
  if (ntfy_result == ALERT_NTFY_OK) {
    RuntimeRebootAlertLatchClear();
  } else {
    RuntimeRebootAlertLatchSet(ALERT_SYSTEM_CODE_ERROR_I2C_RECOVERY,
                               event_epoch,
                               (uint32_t)event_uptime_ms);
    const runtime_reboot_alert_send_result_t send_result =
      (ntfy_result == ALERT_NTFY_FAILED) ? RUNTIME_REBOOT_ALERT_SEND_FAIL
                                         : RUNTIME_REBOOT_ALERT_SEND_SKIPPED;
    const int64_t attempt_epoch =
      TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    RuntimeRebootAlertLatchRecordAttempt(send_result,
                                         RUNTIME_REBOOT_ALERT_GATE_UNKNOWN,
                                         attempt_epoch,
                                         (uint32_t)event_uptime_ms,
                                         status,
                                         ntfy_err,
                                         retry_after_seconds);
  }
  esp_restart();
  return false;
}

static bool
RuntimeRecoverI2cBus(runtime_state_t* state,
                     const char* reason,
                     esp_err_t last_error)
{
  return RuntimeRecoverI2cBusCommon(state, reason, last_error, false);
}

static void
TrackFramInvalidResponse(runtime_state_t* state, const char* context)
{
  if (state == NULL) {
    return;
  }
  if (state->i2c_quiesce_active) {
    state->fram_crc_fail_suppressed_count++;
    if (LogRateLimitAllow(&state->last_fram_crc_suppressed_log_ms,
                          kFramSuppressedWarnIntervalMs)) {
      ESP_LOGW(kTag,
               "Suppressing FRAM invalid-response escalation during I2C "
               "quiesce (%s); count=%" PRIu32,
               (context != NULL) ? context : "unknown",
               state->fram_crc_fail_suppressed_count);
    }
    return;
  }
  state->fram_crc_fail_streak++;
  ESP_LOGW(kTag,
           "FRAM invalid response (%s); streak=%" PRIu32,
           (context != NULL) ? context : "unknown",
           state->fram_crc_fail_streak);
  if (state->fram_crc_fail_streak == 1 ||
      (state->fram_crc_fail_streak % kFramInvalidHeapLogEvery) == 0u) {
    HeapEventLog("fram_invalid_response",
                 (context != NULL) ? context : "unknown",
                 ESP_ERR_INVALID_RESPONSE);
  }
  if (state->fram_crc_fail_streak >= kI2cRecoveryTriggerCount) {
    state->fram_crc_fail_streak = 0;
    if (!state->cached_status.fram_io_error_active) {
      LogFramErrorEvent(state,
                        ERROR_FRAM_IO_FAIL,
                        false,
                        (int32_t)ESP_ERR_INVALID_RESPONSE,
                        (int32_t)kI2cRecoveryTriggerCount);
      UpdateCachedBool(state, &state->cached_status.fram_io_error_active, true);
    }
    (void)RuntimeRecoverI2cBus(state, context, ESP_ERR_INVALID_RESPONSE);
  }
}

static void
ResetFramInvalidResponseStreak(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  state->fram_crc_fail_streak = 0;
}

static void
TrackFramAppendFailure(runtime_state_t* state, esp_err_t error)
{
  if (state == NULL) {
    return;
  }
  if (state->i2c_quiesce_active) {
    state->fram_append_fail_suppressed_count++;
    if (LogRateLimitAllow(&state->last_fram_append_suppressed_log_ms,
                          kFramSuppressedWarnIntervalMs)) {
      ESP_LOGW(kTag,
               "Suppressing FRAM append failure during I2C quiesce; err=%s "
               "count=%" PRIu32,
               esp_err_to_name(error),
               state->fram_append_fail_suppressed_count);
    }
    return;
  }
  state->fram_append_fail_streak++;
  ESP_LOGW(kTag,
           "FRAM append failure streak=%" PRIu32 " err=%s",
           state->fram_append_fail_streak,
           esp_err_to_name(error));
  if (state->fram_append_fail_streak >= kI2cRecoveryTriggerCount) {
    state->fram_append_fail_streak = 0;
    if (!state->cached_status.fram_io_error_active) {
      LogFramErrorEvent(state,
                        ERROR_FRAM_IO_FAIL,
                        false,
                        (int32_t)error,
                        (int32_t)kI2cRecoveryTriggerCount);
      UpdateCachedBool(state, &state->cached_status.fram_io_error_active, true);
    }
    (void)RuntimeRecoverI2cBus(state, "append", error);
  }
}

/**
 * @brief Execute DiscardFramRecordsWithYield.
 * @param state Parameter state.
 * @param records_to_discard Parameter records_to_discard.
 * @param fram_log_timeout_ticks Parameter fram_log_timeout_ticks.
 * @param timeout_counter Parameter timeout_counter.
 * @param last_log_ms Parameter last_log_ms.
 * @param context Parameter context.
 * @return Return the function result.
 */
static esp_err_t
DiscardFramRecordsWithYield(runtime_state_t* state,
                            uint32_t records_to_discard,
                            TickType_t fram_log_timeout_ticks,
                            uint32_t* timeout_counter,
                            uint32_t* last_log_ms,
                            const char* context)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  const uint32_t kDiscardChunkRecords = 16;
  uint32_t remaining = records_to_discard;
  while (remaining > 0) {
    if (!RuntimeFramLogLockWithWarn(state,
                                    fram_log_timeout_ticks,
                                    timeout_counter,
                                    last_log_ms,
                                    context)) {
      return ESP_ERR_TIMEOUT;
    }
    const uint32_t chunk =
      (remaining < kDiscardChunkRecords) ? remaining : kDiscardChunkRecords;
    const bool final_chunk = (chunk == remaining);

    for (uint32_t index = 0; index < chunk; ++index) {
      esp_err_t discard_result = FramLogDiscardOldest(&state->fram_log);
      if (discard_result != ESP_OK) {
        RuntimeFramLogUnlock(state);
        return discard_result;
      }
    }

    if (final_chunk) {
      // Ensure the updated read index/count are durably persisted once the bulk
      // discard completes.
      esp_err_t persist_result = FramLogPersistHeader(&state->fram_log);
      if (persist_result != ESP_OK) {
        ESP_LOGW(kTag,
                 "FRAM header persist after discard failed: %s",
                 esp_err_to_name(persist_result));
      }
    }
    RuntimeFramLogUnlock(state);
    remaining -= chunk;
    if (remaining > 0) {
      vTaskDelay(1);
    }
  }
  return ESP_OK;
}

/**
 * @brief Execute EnsureSdSyncedForEpoch.
 * @param state Parameter state.
 * @param epoch_for_file Parameter epoch_for_file.
 * @return Return the function result.
 */
/**
 * @brief Request an SD flush without blocking acquisition.
 * @param state Parameter state.
 */
static void
RequestSdFlush(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  state->sd_flush_pending = true;
  state->sd_next_flush_allowed_ticks = 0;
}

/**
 * @brief Arm the one-shot time jump marker for the next record.
 * @param state Parameter state.
 */
static void
ArmTimeJumpBackNextRecord(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  if (state->time_jump_back_arm_next) {
    return;
  }
  const TickType_t now_ticks = xTaskGetTickCount();
  if (state->time_jump_back_last_arm_ticks == now_ticks) {
    return;
  }
  state->time_jump_back_arm_next = true;
  state->time_jump_back_last_arm_ticks = now_ticks;
}

/**
 * @brief Handle a backward RTC resync and request a one-shot marker.
 * @param state Parameter state.
 * @param delta_seconds Signed delta (rtc - system).
 * @param system_epoch_before System epoch (seconds) before resync.
 */
static void
HandleRtcBackwardJump(runtime_state_t* state,
                      int64_t delta_seconds,
                      int64_t system_epoch_before)
{
  if (state == NULL || delta_seconds >= 0) {
    return;
  }

  const int64_t rtc_epoch = system_epoch_before + delta_seconds;
  ESP_LOGW(kTag,
           "RTC resync stepped system time backward by %" PRId64
           " s (sys=%" PRId64 " rtc=%" PRId64 "); will flag next record",
           delta_seconds,
           system_epoch_before,
           rtc_epoch);
  state->last_time_jump_back_delta_sec = delta_seconds;

  if (state->time_jump_back_pending_confirm) {
    ESP_LOGI(kTag,
             "Time jump marker already pending SD confirm; not re-arming");
    return;
  }
  ArmTimeJumpBackNextRecord(state);
  RequestSdFlush(state);
}

/**
 * @brief Clear pending time jump marker after SD confirmation.
 * @param state Parameter state.
 */
static void
ConfirmTimeJumpBackMarker(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  if (!state->time_jump_back_pending_confirm) {
    return;
  }
  state->time_jump_back_pending_confirm = false;
  state->time_jump_back_attempt_record_id = 0;
  ESP_LOGI(kTag, "Time jump marker written and verified on SD");
}

/**
 * @brief Retry time jump marker after a proven loss.
 * @param state Parameter state.
 */
static void
RetryTimeJumpBackMarker(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  ESP_LOGW(kTag,
           "Time jump marker record lost before SD confirm; flagging next "
           "record");
  state->time_jump_back_pending_confirm = false;
  state->time_jump_back_attempt_record_id = 0;
  ArmTimeJumpBackNextRecord(state);
  RequestSdFlush(state);
}

/**
 * @brief Evaluate a batch write for time jump confirmation or retry.
 * @param state Parameter state.
 * @param batch_includes_time_jump_flag Whether the batch has the marker.
 * @param last_record_id Last record id written in the batch.
 */
static void
HandleTimeJumpBackBatchWritten(runtime_state_t* state,
                               bool batch_includes_time_jump_flag,
                               uint64_t last_record_id)
{
  if (state == NULL || !state->time_jump_back_pending_confirm) {
    return;
  }
  if (batch_includes_time_jump_flag) {
    ConfirmTimeJumpBackMarker(state);
    return;
  }
  if (state->time_jump_back_attempt_record_id != 0 &&
      last_record_id >= state->time_jump_back_attempt_record_id) {
    RetryTimeJumpBackMarker(state);
  }
}

/**
 * @brief Evaluate a corrupted FRAM skip for time jump retry.
 * @param state Parameter state.
 * @param record Corrupted record data (best effort).
 */
static void
HandleTimeJumpBackCorruptSkip(runtime_state_t* state,
                              const log_record_t* record)
{
  if (state == NULL || record == NULL ||
      !state->time_jump_back_pending_confirm) {
    return;
  }
  if (record->record_id == 0) {
    return;
  }
  if (state->time_jump_back_attempt_record_id == 0) {
    return;
  }
  if (record->record_id >= state->time_jump_back_attempt_record_id) {
    RetryTimeJumpBackMarker(state);
  }
}

/**
 * @brief Restore pending time jump marker state from FRAM on boot.
 * @param state Parameter state.
 */
static void
RestoreTimeJumpBackPendingFromFram(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  uint32_t buffered = 0;
  if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    return;
  }
  buffered = FramLogGetBufferedRecords(&state->fram_log);
  RuntimeFramLogUnlock(state);
  if (buffered == 0) {
    return;
  }

  uint64_t sd_last_record_id_on_boot = 0;
  if (state->sd_logger.is_mounted && TimeSyncIsSystemTimeValid()) {
    const int64_t epoch_now = (int64_t)time(NULL);
    ptlog_header_t header;
    uint32_t header_signature = 0;
    if (RuntimeBuildPtlogHeaderInternal(
          state, epoch_now, &header, &header_signature) &&
        SdLoggerEnsureDailyFileWithHeader(
          &state->sd_logger, epoch_now, &header, header_signature) == ESP_OK) {
      sd_last_record_id_on_boot = SdLoggerLastRecordIdOnSd(&state->sd_logger);
    }
  }

  uint32_t max_scan = buffered;
  const uint32_t watermark = state->settings.fram_flush_watermark_records;
  if (watermark > 0) {
    const uint32_t scan_limit = watermark * 2u;
    if (scan_limit < max_scan) {
      max_scan = scan_limit;
    }
  }

  if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    for (uint32_t offset = 0; offset < max_scan; ++offset) {
      log_record_t record;
      const esp_err_t peek_result =
        FramLogPeekOffset(&state->fram_log, offset, &record);
      if (peek_result == ESP_ERR_NOT_FOUND) {
        break;
      }
      if (peek_result == ESP_ERR_INVALID_RESPONSE) {
        continue;
      }
      if (peek_result != ESP_OK) {
        break;
      }
      if ((record.flags & LOG_RECORD_FLAG_TIME_JUMP_BACK) == 0) {
        continue;
      }
      if (sd_last_record_id_on_boot == 0 ||
          record.record_id > sd_last_record_id_on_boot) {
        state->time_jump_back_pending_confirm = true;
        state->time_jump_back_attempt_record_id = record.record_id;
        RequestSdFlush(state);
        ESP_LOGI(
          kTag,
          "Restored pending time jump marker from FRAM: record_id=%" PRIu64,
          record.record_id);
      }
      break;
    }
    RuntimeFramLogUnlock(state);
  }
}

static esp_err_t
EnsureSdSyncedForEpoch(runtime_state_t* state, int64_t epoch_for_file)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    return ESP_ERR_TIMEOUT;
  }
  if (FramLogGetBufferedRecords(&state->fram_log) == 0) {
    RuntimeFramLogUnlock(state);
    return ESP_OK;
  }

  log_record_t oldest_record;
  esp_err_t peek_result = FramLogPeekOldest(&state->fram_log, &oldest_record);
  if (peek_result == ESP_ERR_INVALID_RESPONSE) {
    const uint32_t record_index = state->fram_log.read_index;
    const uint32_t slot = record_index % state->fram_log.capacity_records;
    const uint32_t addr =
      state->fram_log.record_region_offset + slot * sizeof(log_record_t);
    uint16_t actual_crc = 0;
    const fram_log_validate_result_t validate_result =
      FramLogValidateRecord(&oldest_record, &actual_crc);
    ESP_LOGE(kTag,
             "Skipping corrupted FRAM record at head index=%u slot=%u "
             "addr=0x%04x reason=%s magic=0x%08" PRIx32 " schema=%" PRIu32
             " exp_crc=0x%04x act_crc=0x%04x",
             (unsigned)record_index,
             (unsigned)slot,
             (unsigned)addr,
             FramLogValidateResultToString(validate_result),
             oldest_record.magic,
             oldest_record.schema_version,
             (unsigned)oldest_record.crc16_ccitt,
             (unsigned)actual_crc);
    state->fram_log.saw_corruption = true;
    state->fram_corrupt_skip_count++;
    RecordFramCorruption(
      state, 0, slot, addr, validate_result, &oldest_record, actual_crc);
    HandleTimeJumpBackCorruptSkip(state, &oldest_record);
    TrackFramInvalidResponse(state, "sd_sync");
    esp_err_t skip_result = FramLogDiscardOldest(&state->fram_log);
    RuntimeFramLogUnlock(state);
    if (skip_result != ESP_OK) {
      return skip_result;
    }
    return ESP_OK;
  }
  if (peek_result != ESP_OK) {
    RuntimeFramLogUnlock(state);
    return peek_result;
  }
  ResetFramInvalidResponseStreak(state);
  RuntimeFramLogUnlock(state);

  int64_t target_epoch = epoch_for_file;
  if (oldest_record.timestamp_epoch_sec > 0) {
    target_epoch = oldest_record.timestamp_epoch_sec;
  }
  int64_t target_epoch_for_file = target_epoch;
  if (state->sd_logger.current_date[0] != '\0') {
    char target_date[16];
    BuildDateStringFromEpoch(target_epoch, target_date, sizeof(target_date));
    const int date_cmp = strcmp(target_date, state->sd_logger.current_date);
    if (date_cmp > 0) {
      const TickType_t now_ticks = xTaskGetTickCount();
      const uint32_t ms_since_force =
        (state->last_rtc_force_before_roll_ticks == 0)
          ? UINT32_MAX
          : (uint32_t)pdTICKS_TO_MS(now_ticks -
                                    state->last_rtc_force_before_roll_ticks);
      if (ms_since_force >= 5000u) {
        int64_t delta_seconds = 0;
        bool jumped_back = false;
        const int64_t system_epoch_before = (int64_t)time(NULL);
        const esp_err_t resync_result = TimeSyncResyncSystemFromRtc(
          &state->time_sync, &delta_seconds, &jumped_back);
        if (resync_result == ESP_OK && jumped_back) {
          HandleRtcBackwardJump(state, delta_seconds, system_epoch_before);
        }
        if (resync_result == ESP_OK && TimeSyncIsSystemTimeValid()) {
          target_epoch_for_file = (int64_t)time(NULL);
        }
        state->last_rtc_force_before_roll_ticks = now_ticks;
      }
    }
  }
  ptlog_header_t header;
  uint32_t header_signature = 0;
  if (!RuntimeBuildPtlogHeaderInternal(
        state, target_epoch_for_file, &header, &header_signature)) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t sd_result = SdLoggerEnsureDailyFileWithHeader(
    &state->sd_logger, target_epoch_for_file, &header, header_signature);
  if (sd_result != ESP_OK) {
    return sd_result;
  }

  uint32_t consumed = 0;
  if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t consume_result = FramLogConsumeUpToRecordId(
    &state->fram_log, SdLoggerLastRecordIdOnSd(&state->sd_logger), &consumed);
  RuntimeFramLogUnlock(state);
  if (consume_result == ESP_ERR_INVALID_RESPONSE) {
    ESP_LOGE(kTag, "FRAM corruption while aligning with SD contents");
    HeapEventLog("fram_sd_align_fail", "consume_up_to_id", consume_result);
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
 * @param fram_log Parameter fram_log.
 * @param buffered Parameter buffered.
 * @param target_date Parameter target_date.
 * @param buffer Parameter buffer.
 * @param buffer_size Parameter buffer_size.
 * @param max_records Parameter max_records.
 * @param start_ticks Parameter start_ticks.
 * @param max_ms Parameter max_ms.
 * @param records_used_out Parameter records_used_out.
 * @param last_record_id_out Parameter last_record_id_out.
 * @param bytes_used_out Parameter bytes_used_out.
 * @param batch_includes_time_jump_flag_out Parameter
 * batch_includes_time_jump_flag_out.
 * @return Return the function result.
 */
static esp_err_t
BuildBatchForDay(runtime_state_t* state,
                 const fram_log_t* fram_log,
                 uint32_t buffered,
                 const char* target_date,
                 uint8_t* buffer,
                 size_t buffer_size,
                 uint32_t max_records,
                 TickType_t start_ticks,
                 uint32_t max_ms,
                 uint32_t* records_used_out,
                 uint64_t* last_record_id_out,
                 size_t* bytes_used_out,
                 bool* batch_includes_time_jump_flag_out)
{
  size_t used = 0;
  uint32_t records_used = 0;
  uint64_t last_record_id = 0;
  bool batch_includes_time_jump_flag = false;
  TickType_t last_yield_ticks = xTaskGetTickCount();

  for (uint32_t offset = 0; offset < buffered; ++offset) {
    if (max_records > 0 && records_used >= max_records) {
      break;
    }
    if (max_ms > 0 &&
        pdTICKS_TO_MS(xTaskGetTickCount() - start_ticks) >= max_ms) {
      break;
    }
    log_record_t record;
    state->sd_flush_phase = "build:peek_offset";
    esp_err_t peek_result = FramLogPeekOffset(fram_log, offset, &record);
    if (peek_result == ESP_ERR_NOT_FOUND) {
      break;
    }
    if (peek_result == ESP_ERR_INVALID_RESPONSE) {
      const uint32_t record_index = fram_log->read_index + offset;
      const uint32_t slot = record_index % fram_log->capacity_records;
      const uint32_t addr =
        fram_log->record_region_offset + slot * sizeof(log_record_t);
      uint16_t actual_crc = 0;
      const fram_log_validate_result_t validate_result =
        FramLogValidateRecord(&record, &actual_crc);
      ESP_LOGE(kTag,
               "Corrupted FRAM record during batch build offset=%u "
               "index=%u slot=%u addr=0x%04x reason=%s magic=0x%08" PRIx32
               " schema=%" PRIu32 " exp_crc=0x%04x act_crc=0x%04x",
               (unsigned)offset,
               (unsigned)record_index,
               (unsigned)slot,
               (unsigned)addr,
               FramLogValidateResultToString(validate_result),
               record.magic,
               record.schema_version,
               (unsigned)record.crc16_ccitt,
               (unsigned)actual_crc);
      state->fram_log.saw_corruption = true;
      state->fram_corrupt_detect_count++;
      RecordFramCorruption(
        state, offset, slot, addr, validate_result, &record, actual_crc);
      TrackFramInvalidResponse(state, "batch");
      state->sd_flush_last_err = peek_result;
      state->sd_flush_phase = "build:peek_invalid";
      RuntimeRecordError(state, "fram", peek_result, state->sd_flush_phase);
      break;
    }
    if (peek_result != ESP_OK) {
      state->sd_flush_last_err = peek_result;
      state->sd_flush_i2c_errs++;
      state->sd_flush_phase = "build:peek_offset_failed";
      RuntimeRecordError(state, "sd_flush", peek_result, state->sd_flush_phase);
      return peek_result;
    }
    ResetFramInvalidResponseStreak(state);

    char record_date[16];
    BuildDateStringFromRecord(&record, record_date, sizeof(record_date));
    if (strcmp(record_date, target_date) != 0) {
      break; // stop at day rollover; flush current batch first
    }

    size_t line_len = 0;
    state->sd_flush_phase = "build:format_csv";
    if (!CsvFormatRow(&record,
                      state->node_id_string,
                      g_sd_csv_line_buffer,
                      sizeof(g_sd_csv_line_buffer),
                      &line_len)) {
      return ESP_ERR_NO_MEM;
    }
    if (used + line_len > buffer_size) {
      // Buffer full; flush what we have so far.
      break;
    }

    memcpy(buffer + used, g_sd_csv_line_buffer, line_len);
    used += line_len;
    records_used++;
    last_record_id = record.record_id;
    if ((record.flags & LOG_RECORD_FLAG_TIME_JUMP_BACK) != 0) {
      batch_includes_time_jump_flag = true;
    }

    if (used >= buffer_size - sizeof(g_sd_csv_line_buffer)) {
      break;
    }

    const TickType_t now_ticks = xTaskGetTickCount();
    if ((now_ticks - last_yield_ticks) >= pdMS_TO_TICKS(25)) {
      vTaskDelay(1);
      last_yield_ticks = xTaskGetTickCount();
    }
  }

  state->sd_flush_phase = "build:done";
  *records_used_out = records_used;
  *last_record_id_out = last_record_id;
  *bytes_used_out = used;
  if (batch_includes_time_jump_flag_out != NULL) {
    *batch_includes_time_jump_flag_out = batch_includes_time_jump_flag;
  }
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
  if (!state->fram_available) {
    return ESP_ERR_INVALID_STATE;
  }
  if (state->batch_buffer == NULL || state->batch_buffer_size == 0) {
    return ESP_ERR_NO_MEM;
  }

  const bool allow_full_flush = flush_all && state->stop_requested;
  TickType_t flush_start = xTaskGetTickCount();
  uint32_t total_flushed = 0;
  uint32_t fram_lock_timeout_counter = 0;
  uint32_t fram_lock_last_log_ms = 0;
  uint32_t buffered = 0;
  while (true) {
    if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    buffered = FramLogGetBufferedRecords(&state->fram_log);
    RuntimeFramLogUnlock(state);
    if (buffered == 0) {
      break;
    }

    log_record_t first_record;
    if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    esp_err_t peek_result = FramLogPeekOldest(&state->fram_log, &first_record);
    if (peek_result == ESP_ERR_INVALID_RESPONSE) {
      ESP_LOGW(kTag,
               "FRAM invalid response persists after retries; discarding "
               "oldest record");
      esp_err_t skip_result = FramLogSkipCorruptedRecord(&state->fram_log);
      RuntimeFramLogUnlock(state);
      TrackFramInvalidResponse(state, "sd_sync");
      if (skip_result != ESP_OK) {
        return skip_result;
      }
      return ESP_ERR_INVALID_RESPONSE;
    }
    if (peek_result != ESP_OK) {
      RuntimeFramLogUnlock(state);
      return peek_result;
    }
    RuntimeFramLogUnlock(state);

    char day_string[16];
    BuildDateStringFromRecord(&first_record, day_string, sizeof(day_string));

    const int64_t epoch_for_file = (first_record.timestamp_epoch_sec > 0)
                                     ? first_record.timestamp_epoch_sec
                                     : (int64_t)time(NULL);

    if (!RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    state->sd_flush_phase = "sync:pre";
    EnsureSdMountedLocked(state);
    if (!state->sd_logger.is_mounted) {
      RuntimeSdIoUnlock(state);
      return ESP_ERR_INVALID_STATE;
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
    state->sd_flush_phase = "sync:ok";
    RuntimeSdIoUnlock(state);

    uint32_t records_used = 0;
    uint64_t last_record_id = 0;
    size_t bytes_used = 0;
    bool batch_includes_time_jump_flag = false;
    fram_log_t fram_log_snapshot;
    if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    buffered = FramLogGetBufferedRecords(&state->fram_log);
    fram_log_snapshot = state->fram_log;
    RuntimeFramLogUnlock(state);
    esp_err_t batch_result = BuildBatchForDay(state,
                                              &fram_log_snapshot,
                                              buffered,
                                              day_string,
                                              state->batch_buffer,
                                              state->batch_buffer_size,
                                              0,
                                              xTaskGetTickCount(),
                                              kSdFlushMaxMsPerPass,
                                              &records_used,
                                              &last_record_id,
                                              &bytes_used,
                                              &batch_includes_time_jump_flag);
    if (batch_result != ESP_OK) {
      return batch_result;
    }
    if (records_used == 0 || bytes_used == 0) {
      return ESP_ERR_INVALID_STATE;
    }

    sd_csv_append_stats_t append_stats = { 0 };
    sd_csv_append_scratch_t append_scratch = { 0 };
    const sd_csv_append_scratch_t* scratch =
      BuildSdAppendScratch(state, &append_scratch);
    esp_err_t write_result = ESP_OK;
    if (!RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
      return ESP_ERR_TIMEOUT;
    }
    state->sd_flush_phase = "append:sync";
    EnsureSdMountedLocked(state);
    if (!state->sd_logger.is_mounted) {
      RuntimeSdIoUnlock(state);
      return ESP_ERR_INVALID_STATE;
    }
    sync_result = EnsureSdSyncedForEpoch(state, epoch_for_file);
    if (sync_result != ESP_OK) {
      RuntimeDiagHeapCheck(state, "SD unmount (sync fail before)", false);
      (void)SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (sync fail after)", false);
      RuntimeSdIoUnlock(state);
      MarkSdFailure(state, "SD sync failed", "sync", sync_result, 0, true);
      return sync_result;
    }
    state->sd_flush_phase = "append:begin";
    if (state->sd_force_unmount_on_append) {
      state->sd_force_unmount_on_append = false;
      RuntimeDiagHeapCheck(state, "SD unmount (inject before)", false);
      (void)SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (inject after)", false);
      append_stats.diag.operation = "inject-unmount";
      append_stats.diag.errno_value = 0;
      write_result = ESP_ERR_INVALID_STATE;
    } else {
      write_result = SdLoggerAppendBatchEx(&state->sd_logger,
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
      const char* op = (append_stats.diag.operation != NULL)
                         ? append_stats.diag.operation
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
    state->sd_flush_phase = "append:ok";
    HandleTimeJumpBackBatchWritten(
      state, batch_includes_time_jump_flag, last_record_id);
    RuntimeSdIoUnlock(state);

    esp_err_t discard_result =
      DiscardFramRecordsWithYield(state,
                                  records_used,
                                  kFramLogLockTimeoutTicks,
                                  &fram_lock_timeout_counter,
                                  &fram_lock_last_log_ms,
                                  "flush discard");
    if (discard_result != ESP_OK) {
      return discard_result;
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

static void
RuntimeRebootOnSdEioEscalation(runtime_state_t* state, const char* context)
{
  const int64_t event_epoch = (int64_t)time(NULL);
  const uint32_t event_uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
  int status = 0;
  int retry_after_seconds = -1;
  esp_err_t ntfy_err = ESP_OK;
  alert_ntfy_result_t ntfy_result = ALERT_NTFY_SKIPPED;
  if (RuntimeEnqueueSystemErrorNote(state,
                                    ALERT_SYSTEM_CODE_ERROR_SD_IO,
                                    false,
                                    event_epoch,
                                    event_uptime_ms)) {
    const int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t next_attempt_ms = now_ms;
    (void)AlertManagerPumpNtfy(&state->alert_manager, now_ms, &next_attempt_ms);

    ntfy_result = RuntimeEnqueueNtfyJobAndWait(state,
                                               NULL,
                                               kRebootAlertHttpTimeoutMs,
                                               &status,
                                               &ntfy_err,
                                               &retry_after_seconds);
  }
  if (ntfy_result == ALERT_NTFY_OK) {
    RuntimeRebootAlertLatchClear();
  } else {
    RuntimeRebootAlertLatchSet(
      ALERT_SYSTEM_CODE_ERROR_SD_IO, event_epoch, event_uptime_ms);
    const runtime_reboot_alert_send_result_t send_result =
      (ntfy_result == ALERT_NTFY_FAILED) ? RUNTIME_REBOOT_ALERT_SEND_FAIL
                                         : RUNTIME_REBOOT_ALERT_SEND_SKIPPED;
    const int64_t attempt_epoch =
      TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    RuntimeRebootAlertLatchRecordAttempt(send_result,
                                         RUNTIME_REBOOT_ALERT_GATE_UNKNOWN,
                                         attempt_epoch,
                                         event_uptime_ms,
                                         status,
                                         ntfy_err,
                                         retry_after_seconds);
  }
  ESP_LOGE(kTag,
           "%s: SD I/O escalation detected; rebooting to avoid SPI reset path",
           (context != NULL) ? context : "SD");
  vTaskDelay(pdMS_TO_TICKS(1000));
  esp_restart();
}

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
  if (errno_value == EIO && did_unmount && state->sd_fail_count >= 3u) {
    if (!state->sd_reset_pending) {
      // SD hard reset recovery tears down the SPI bus and can corrupt heap
      // state under IPC; reboot to return to a known-good state.
      state->sd_reset_pending = true;
      state->sd_reset_pending_since_ticks = now_ticks;
      state->sd_reset_pending_context = context;
      state->sd_reset_pending_errno = errno_value;
      ESP_LOGW(kTag,
               "%s: SD reset requested after EIO escalation",
               (context != NULL) ? context : "SD");
      RuntimeRebootOnSdEioEscalation(state, context);
      return;
    }
  }
  ReduceSdMaxFreqOnEio(state, errno_value);
  uint64_t backoff_ms = 500;
  backoff_ms = kSdFlushFailureBackoffMs;
  if (state->sd_fail_count > 1u) {
    const uint32_t shift = state->sd_fail_count - 1u;
    if (shift >= 31u) {
      backoff_ms = kSdFlushFailureBackoffMaxMs;
    } else {
      backoff_ms = (uint64_t)kSdFlushFailureBackoffMs << shift;
    }
  }
  if (backoff_ms < kSdFlushFailureBackoffMs) {
    backoff_ms = kSdFlushFailureBackoffMs;
  }
  if (backoff_ms > kSdFlushFailureBackoffMaxMs) {
    backoff_ms = kSdFlushFailureBackoffMaxMs;
  }
  state->sd_backoff_until_ticks = now_ticks + pdMS_TO_TICKS(backoff_ms);
  if (IsSdIoOperation(operation)) {
    state->sd_io_error_streak++;
    state->sd_io_success_streak = 0;
    state->sd_last_io_err = error;
    state->sd_last_errno = errno_value;
    if (!state->sd_last_io_error_active &&
        state->sd_io_error_streak >= kSdIoErrorSetThreshold) {
      state->sd_last_io_error_active = true;
      UpdateCachedBool(state, &state->cached_status.sd_io_error_active, true);
    }
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
  const char* context_label = (context != NULL) ? context : "SD";
  if (did_unmount) {
    HeapEventLog("sd_unmount_backoff", op_label, error);
  }
  ESP_LOGW(kTag,
           "%s: op=%s err=%s (%d) errno=%d (%s) action=%s backoff_until=%u",
           context_label,
           op_label,
           esp_err_to_name(error),
           (int)error,
           errno_value,
           errno_str,
           action_label,
           (unsigned)state->sd_backoff_until_ticks);
}

static void
SdResetPendingTick(runtime_state_t* state)
{
  if (state == NULL || !state->sd_reset_pending) {
    return;
  }
  const char* context = (state->sd_reset_pending_context != NULL)
                          ? state->sd_reset_pending_context
                          : "SD reset";
  RuntimeRebootOnSdEioEscalation(state, context);
}

static void
ReduceSdMaxFreqOnEio(runtime_state_t* state, int errno_value)
{
  if (state == NULL || errno_value != EIO) {
    return;
  }

  uint32_t current_khz = state->sd_logger.config.max_freq_khz;
  uint32_t next_khz = current_khz;
  if (current_khz > 5000u) {
    next_khz = 5000u;
  } else if (current_khz > 2000u) {
    next_khz = 2000u;
  }

  if (next_khz != current_khz && next_khz != 0u) {
    state->sd_logger.config.max_freq_khz = next_khz;
    ESP_LOGW(kTag,
             "SD I/O error (EIO); reducing SD SPI max freq to %u kHz",
             (unsigned)next_khz);
  }
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
// Lock order rule: SdFlushTask/SdFlushWorkerTickEx takes sd_io_mutex first,
// then fram_log_mutex. StorageTask never takes sd_io_mutex.
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

  RuntimeMarkersSetSdFlush(state, SD_040_FLUSH_WORKER_ENTER);
  state->sd_flush_phase = "flush:enter";
  state->sd_flush_last_err = ESP_OK;
  const TickType_t start_ticks = xTaskGetTickCount();
  bool recover_i2c = false;
  bool more_pending = false;
  esp_err_t recover_err = ESP_OK;
  if (state->sd_backoff_until_ticks != 0 &&
      start_ticks < state->sd_backoff_until_ticks) {
    if (more_pending_out != NULL) {
      *more_pending_out = true;
    }
    RuntimeMarkersSetSdFlush(state, SD_050_FLUSH_WORKER_EXIT);
    return ESP_OK;
  }

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
    RuntimeMarkersSetSdFlush(state, SD_050_FLUSH_WORKER_EXIT);
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t result = ESP_OK;
  state->sd_flush_in_progress = true;
  if (!state->sd_flush_quiesce_session_active) {
    SdFlushMaybeStartSession(state);
    const char* session_reason = (state->sd_flush_session_label[0] != '\0')
                                   ? state->sd_flush_session_label
                                   : "sd_flush/unknown#0";
    RuntimeBeginI2cQuiesce(state, session_reason);
    state->sd_flush_quiesce_session_active = true;
  }
  uint32_t total_records_flushed = 0;
  size_t total_bytes_flushed = 0;
  uint32_t corrupt_skips_this_call = 0;
  const uint32_t kMaxCorruptSkipsThisCall = 8;
  const TickType_t fram_log_timeout_ticks = pdMS_TO_TICKS(250);
  if (!RuntimeSpiBusLockForSharedDevices(state, kSdIoLockTimeoutTicks)) {
    result = ESP_ERR_TIMEOUT;
    goto flush_done;
  }
  EnsureSdMountedLocked(state);
  RuntimeSpiBusUnlockForSharedDevices(state);
  if (!state->sd_logger.is_mounted) {
    result = ESP_OK;
    goto flush_done;
  }

  while (true) {
    if (max_records > 0 && total_records_flushed >= max_records) {
      break;
    }
    if (max_ms > 0 &&
        pdTICKS_TO_MS(xTaskGetTickCount() - start_ticks) >= max_ms) {
      break;
    }

    uint32_t buffered_records = 0;
    log_record_t first_record;
    char day_string[16];
    state->sd_flush_phase = "flush:peek_oldest";
    if (!RuntimeFramLogLockWithWarn(
          state,
          fram_log_timeout_ticks,
          &state->fram_log_lock_timeout_count_sdflush,
          &state->last_fram_log_lock_timeout_sdflush_log_ms,
          "sd_flush snapshot")) {
      result = ESP_ERR_TIMEOUT;
      goto flush_done;
    }
    buffered_records = FramLogGetBufferedRecords(&state->fram_log);
    if (buffered_records == 0) {
      RuntimeFramLogUnlock(state);
      break;
    }
    esp_err_t peek_result = FramLogPeekOldest(&state->fram_log, &first_record);
    if (peek_result == ESP_ERR_INVALID_RESPONSE) {
      ESP_LOGW(
        kTag,
        "FRAM invalid response persists after retries; discarding oldest "
        "record during SD flush");
      esp_err_t skip_result = FramLogSkipCorruptedRecord(&state->fram_log);
      RuntimeFramLogUnlock(state);
      if (skip_result != ESP_OK) {
        result = skip_result;
        goto flush_done;
      }
      state->fram_corrupt_skip_count++;
      corrupt_skips_this_call++;
      TrackFramInvalidResponse(state, "flush");
      if (corrupt_skips_this_call >= kMaxCorruptSkipsThisCall) {
        break;
      }
      continue;
    }
    if (peek_result != ESP_OK) {
      RuntimeFramLogUnlock(state);
      state->sd_flush_last_err = peek_result;
      state->sd_flush_i2c_errs++;
      state->sd_flush_phase = "flush:peek_oldest_failed";
      RuntimeRecordError(state, "sd_flush", peek_result, state->sd_flush_phase);
      if (!recover_i2c && IsRecoverableI2cErr(peek_result)) {
        recover_i2c = true;
        recover_err = peek_result;
        ESP_LOGW(kTag,
                 "Recoverable I2C error during flush peek: %s (%d)",
                 esp_err_to_name(peek_result),
                 (int)peek_result);
      }
      result = peek_result;
      goto flush_done;
    }
    ResetFramInvalidResponseStreak(state);

    BuildDateStringFromRecord(&first_record, day_string, sizeof(day_string));
    fram_log_t fram_log_snapshot = state->fram_log;
    RuntimeFramLogUnlock(state);
    uint32_t max_batch_records = 0;
    if (max_records > 0) {
      max_batch_records = max_records - total_records_flushed;
    }
    uint32_t records_used = 0;
    uint64_t last_record_id = 0;
    size_t bytes_used = 0;
    bool batch_includes_time_jump_flag = false;
    esp_err_t batch_result = BuildBatchForDay(state,
                                              &fram_log_snapshot,
                                              buffered_records,
                                              day_string,
                                              state->batch_buffer,
                                              state->batch_buffer_size,
                                              max_batch_records,
                                              start_ticks,
                                              max_ms,
                                              &records_used,
                                              &last_record_id,
                                              &bytes_used,
                                              &batch_includes_time_jump_flag);
    if (batch_result != ESP_OK) {
      state->sd_flush_last_err = batch_result;
      state->sd_flush_phase = "flush:build_failed";
      RuntimeRecordError(
        state, "sd_flush", batch_result, state->sd_flush_phase);
      if (!recover_i2c && IsRecoverableI2cErr(batch_result)) {
        state->sd_flush_i2c_errs++;
        recover_i2c = true;
        recover_err = batch_result;
        ESP_LOGW(kTag,
                 "Recoverable I2C error during batch build: %s (%d)",
                 esp_err_to_name(batch_result),
                 (int)batch_result);
      }
      result = batch_result;
      goto flush_done;
    }
    if (records_used == 0 || bytes_used == 0) {
      result = ESP_ERR_NO_MEM;
      goto flush_done;
    }

    const int64_t epoch_for_file = (first_record.timestamp_epoch_sec > 0)
                                     ? first_record.timestamp_epoch_sec
                                     : (int64_t)time(NULL);
    if (!RuntimeSpiBusLockForSharedDevices(state, kSdIoLockTimeoutTicks)) {
      result = ESP_ERR_TIMEOUT;
      goto flush_done;
    }
    state->sd_flush_phase = "flush:sync";
    esp_err_t sync_result = EnsureSdSyncedForEpoch(state, epoch_for_file);
    if (sync_result != ESP_OK) {
      RuntimeDiagHeapCheck(state, "SD unmount (flush sync before)", false);
      (void)SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (flush sync after)", false);
      RuntimeSpiBusUnlockForSharedDevices(state);
      MarkSdFailure(state, "SD sync failed", "sync", sync_result, 0, true);
      state->sd_flush_last_err = sync_result;
      state->sd_flush_sd_errs++;
      state->sd_flush_phase = "flush:sync_failed";
      RuntimeRecordError(state, "sd_flush", sync_result, state->sd_flush_phase);
      result = sync_result;
      goto flush_done;
    }

    sd_csv_append_stats_t append_stats = { 0 };
    sd_csv_append_scratch_t append_scratch = { 0 };
    const sd_csv_append_scratch_t* scratch =
      BuildSdAppendScratch(state, &append_scratch);
    state->sd_flush_phase = "flush:append";
    esp_err_t write_result = SdLoggerAppendBatchEx(&state->sd_logger,
                                                   state->batch_buffer,
                                                   bytes_used,
                                                   last_record_id,
                                                   verify_mode,
                                                   flush_mode,
                                                   scratch,
                                                   &append_stats);
    uint64_t sd_total_bytes = 0;
    uint64_t sd_free_bytes = 0;
    if (SdLoggerGetSpaceInfo(
          &state->sd_logger, &sd_total_bytes, &sd_free_bytes) == ESP_OK) {
      UpdateCachedUint64(
        state, &state->cached_status.sd_total_bytes, sd_total_bytes);
      UpdateCachedUint64(
        state, &state->cached_status.sd_free_bytes, sd_free_bytes);
    }
    UpdateCachedBool(state,
                     &state->cached_status.sd_space_reclaim_active,
                     SdLoggerSpaceReclaimActive(&state->sd_logger));
    UpdateCachedUint32(state,
                       &state->cached_status.sd_space_reclaim_deleted_total,
                       SdLoggerSpaceReclaimDeletedTotal(&state->sd_logger));

    RuntimeSpiBusUnlockForSharedDevices(state);

    if (write_result != ESP_OK) {
      if (append_stats.diag.errno_value == ENOSPC) {
        UpdateCachedBool(
          state, &state->cached_status.sd_out_of_space_active, true);
        state->sd_flush_last_err = write_result;
        state->sd_flush_phase = "flush:append_enospc";
        RuntimeRecordError(
          state, "sd_flush", write_result, state->sd_flush_phase);
        result = write_result;
        goto flush_done;
      }
      RuntimeDiagHeapCheck(state, "SD unmount (flush append before)", false);
      (void)SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (flush append after)", false);
      const char* op = (append_stats.diag.operation != NULL)
                         ? append_stats.diag.operation
                         : "append";
      MarkSdFailure(state,
                    "SD append failed",
                    op,
                    write_result,
                    append_stats.diag.errno_value,
                    true);
      state->sd_flush_last_err = write_result;
      state->sd_flush_sd_errs++;
      state->sd_flush_phase = "flush:append_failed";
      RuntimeRecordError(
        state, "sd_flush", write_result, state->sd_flush_phase);
      result = write_result;
      goto flush_done;
    }
    UpdateCachedBool(
      state, &state->cached_status.sd_out_of_space_active, false);
    ClearSdIoError(state);
    HandleTimeJumpBackBatchWritten(
      state, batch_includes_time_jump_flag, last_record_id);

    state->sd_flush_phase = "flush:discard";
    esp_err_t discard_result = DiscardFramRecordsWithYield(
      state,
      records_used,
      fram_log_timeout_ticks,
      &state->fram_log_lock_timeout_count_sdflush,
      &state->last_fram_log_lock_timeout_sdflush_log_ms,
      "sd_flush discard");
    if (discard_result != ESP_OK) {
      result = discard_result;
      goto flush_done;
    }

    total_records_flushed += records_used;
    total_bytes_flushed += bytes_used;
  }

flush_done:
  state->sd_flush_in_progress = false;
  RuntimeSdIoUnlock(state);
  if (recover_i2c && IsRecoverableI2cErr(recover_err)) {
    state->sd_flush_phase = "i2c:recover";
    (void)RuntimeRecoverI2cBus(state, "sd_flush_fram_err", recover_err);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  RuntimeMarkersSetSdFlush(state, SD_050_FLUSH_WORKER_EXIT);

  if (RuntimeFramLogLockWithWarn(
        state,
        fram_log_timeout_ticks,
        &state->fram_log_lock_timeout_count_sdflush,
        &state->last_fram_log_lock_timeout_sdflush_log_ms,
        "sd_flush pending check")) {
    more_pending = (FramLogGetBufferedRecords(&state->fram_log) > 0);
    RuntimeFramLogUnlock(state);
  } else {
    more_pending = true;
  }
  if (more_pending_out != NULL) {
    *more_pending_out = more_pending;
  }

  state->sd_flush_session_records_flushed += total_records_flushed;

  if (result != ESP_OK || !more_pending) {
    const uint32_t duration_ms =
      (state->sd_flush_session_start_ticks > 0)
        ? (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() -
                                  state->sd_flush_session_start_ticks)
        : 0u;
    SdFlushLogSessionEnd(
      state, state->sd_flush_session_records_flushed, duration_ms);
    state->sd_flush_quiesce_session_active = false;
    RuntimeEndI2cQuiesce(state);
    if (!state->i2c_quiesce_active) {
      const esp_err_t deferred_result =
        RuntimeDrainDeferredToFram(state, "sd_flush deferred reconcile");
      if (deferred_result != ESP_OK) {
        ESP_LOGW(kTag,
                 "Deferred RAM->FRAM reconcile paused: %s",
                 esp_err_to_name(deferred_result));
      }
      UpdateCachedUint32(
        state, &state->cached_status.deferred_count, DeferredLogCount(state));
      UpdateCachedUint32(
        state, &state->cached_status.deferred_drops, DeferredLogDrops(state));
      UpdateCachedBool(state,
                       &state->cached_status.deferred_active,
                       (DeferredLogCount(state) > 0u));
    }
  }

  if (result != ESP_OK) {
    return result;
  }
  if (records_flushed_out != NULL) {
    *records_flushed_out = total_records_flushed;
  }
  if (bytes_flushed_out != NULL) {
    *bytes_flushed_out = total_bytes_flushed;
  }
  state->sd_flush_phase = "flush:done";
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

static void
SensorSpiFaultSetActive(runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL || state->sensor_spi_fault_active) {
    return;
  }
  LogFramErrorEvent(state, kErrlogSensorSpiFault, false, 0, 0);
  const int64_t event_epoch =
    TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
  (void)RuntimeEnqueueSystemErrorNote(
    state, ALERT_SYSTEM_CODE_ERROR_SENSOR_SPI, false, event_epoch, now_ms);
  state->sensor_spi_fault_active = true;
}

static void
SensorSpiFaultResolveIfStable(runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL || !state->sensor_spi_fault_active) {
    return;
  }
  if (state->sensor_spi_last_invalid_ms == 0 ||
      now_ms - state->sensor_spi_last_invalid_ms < kSensorSpiStableResolveMs) {
    return;
  }
  LogFramErrorEvent(state, kErrlogSensorSpiFault, true, 0, 0);
  const int64_t event_epoch =
    TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
  (void)RuntimeEnqueueSystemErrorNote(
    state, ALERT_SYSTEM_CODE_ERROR_SENSOR_SPI, true, event_epoch, now_ms);
  state->sensor_spi_fault_active = false;
  state->sensor_spi_reinit_count = 0;
  state->sensor_spi_reinit_window_start_ms = 0;
}

static void
TrackSensorSpiInvalid(runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL) {
    return;
  }
  state->sensor_spi_invalid_streak++;
  state->sensor_spi_last_invalid_ms = now_ms;
  if (state->sensor_spi_invalid_streak < kSensorSpiInvalidStreakThreshold) {
    return;
  }
  state->sensor_spi_invalid_streak = 0;
  const bool locked = RuntimeSpiBusLock(state, kSdIoLockTimeoutTicks);
  if (!locked) {
    ESP_LOGW(kTag, "SPI bus lock timeout; skipping MAX31865 reinit");
    return;
  }
  const esp_err_t init_result = InitializeMax31865Sensor(state, GetSpiHost());
  RuntimeSpiBusUnlock(state);
  if (init_result != ESP_OK) {
    ESP_LOGW(kTag,
             "MAX31865 reinit failed after SPI invalid streak: %s",
             esp_err_to_name(init_result));
    return;
  }
  if (state->sensor_spi_reinit_window_start_ms == 0 ||
      (now_ms - state->sensor_spi_reinit_window_start_ms) >
        kSensorSpiReinitWindowMs) {
    state->sensor_spi_reinit_window_start_ms = now_ms;
    state->sensor_spi_reinit_count = 1;
  } else {
    state->sensor_spi_reinit_count++;
  }
  if (state->sensor_spi_reinit_count >= kSensorSpiReinitAlertThreshold) {
    SensorSpiFaultSetActive(state, now_ms);
  }
}

/**
 * @brief Update the debounced RTD fault state from raw MAX31865 indications.
 * @param state Runtime state to update.
 * @param settings Settings that contain the debounce timings.
 * @param raw_fault_present True when a raw fault is detected.
 * @param sd_flush_busy True when SD flush contention should freeze updates.
 * @param now_ms Current monotonic time in milliseconds.
 * @param raw_fault_status Raw MAX31865 fault status when available.
 */
static void
UpdateDebouncedRtdFault(runtime_state_t* state,
                        const app_settings_t* settings,
                        bool raw_fault_present,
                        bool sd_flush_busy,
                        int64_t now_ms,
                        uint8_t raw_fault_status)
{
  if (state == NULL || settings == NULL) {
    return;
  }
  if (sd_flush_busy) {
    if (state->rtd_fault_pending_since_ms != 0) {
      state->rtd_fault_pending_since_ms = now_ms;
    }
    if (state->rtd_clear_pending_since_ms != 0) {
      state->rtd_clear_pending_since_ms = now_ms;
    }
    return;
  }

  if (raw_fault_present) {
    state->rtd_clear_pending_since_ms = 0;
    state->rtd_fault_last_status = raw_fault_status;
    if (!state->last_sensor_fault_present) {
      if (state->rtd_fault_pending_since_ms == 0) {
        state->rtd_fault_pending_since_ms = now_ms;
      }
      if ((now_ms - state->rtd_fault_pending_since_ms) >=
          (int64_t)settings->rtd_fault_assert_ms) {
        char fault_desc[128] = { 0 };
        Max31865FormatFault(
          state->rtd_fault_last_status, fault_desc, sizeof(fault_desc));
        state->last_sensor_fault_present = true;
        state->rtd_fault_pending_since_ms = 0;
        ESP_LOGI(kTag,
                 "RTD fault active: status=0x%02X (%s) assert_ms=%" PRIu32,
                 state->rtd_fault_last_status,
                 fault_desc,
                 settings->rtd_fault_assert_ms);
      }
    } else {
      state->rtd_fault_pending_since_ms = 0;
    }
    return;
  }

  if (state->rtd_fault_pending_since_ms != 0 &&
      !state->last_sensor_fault_present) {
    state->rtd_fault_suppressed_count++;
  }
  state->rtd_fault_pending_since_ms = 0;

  if (state->last_sensor_fault_present) {
    if (state->rtd_clear_pending_since_ms == 0) {
      state->rtd_clear_pending_since_ms = now_ms;
    }
    if ((now_ms - state->rtd_clear_pending_since_ms) >=
        (int64_t)settings->rtd_fault_clear_ms) {
      state->last_sensor_fault_present = false;
      state->rtd_clear_pending_since_ms = 0;
      ESP_LOGI(kTag,
               "RTD fault resolved: clear_ms=%" PRIu32,
               settings->rtd_fault_clear_ms);
    }
  } else {
    state->rtd_clear_pending_since_ms = 0;
  }
}

/**
 * @brief Compute effective sensor polling period from display/log intervals.
 * @param settings Current app settings snapshot.
 * @return Polling period in milliseconds, clamped to [100, 3600000].
 */
static uint32_t
RuntimeEffectiveSensorPollPeriodMs(const app_settings_t* settings)
{
  uint32_t display_period_ms = 1000u;
  uint32_t log_period_ms = 1000u;

  if (settings != NULL) {
    display_period_ms = settings->display_sample_period_ms;
    log_period_ms = settings->log_period_ms;
  }

  if (display_period_ms == 0u) {
    display_period_ms = 1000u;
  }
  if (display_period_ms < 100u) {
    display_period_ms = 100u;
  } else if (display_period_ms > 3600000u) {
    display_period_ms = 3600000u;
  }

  if (log_period_ms >= 100u && log_period_ms <= 3600000u &&
      log_period_ms < display_period_ms) {
    display_period_ms = log_period_ms;
  }

  return display_period_ms;
}
/**
 * @brief Perform one MAX31865 one-shot conversion/read cycle.
 * @param state Runtime state owning the MAX31865 reader.
 * @param sample_out Output sample buffer.
 * @param one_shot_out Optional one-shot state snapshot for diagnostics.
 * @return ESP_OK on read completion, or an ESP_ERR_* code on failure.
 */
static esp_err_t
RuntimePerformSingleMax31865Read(runtime_state_t* state,
                                 max31865_sample_t* sample_out,
                                 max31865_one_shot_state_t* one_shot_out)
{
  if (state == NULL || sample_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  max31865_one_shot_state_t one_shot = {
    .phase = kMax31865OneShotIdle,
  };
  esp_err_t result = Max31865StartOneShot(&state->sensor, &one_shot);
  if (result == ESP_OK) {
    while (!state->stop_requested && !state->spi_pause_requested) {
      result = Max31865TryReadOneShot(&state->sensor, &one_shot, sample_out);
      if (result == ESP_OK || result != ESP_ERR_TIMEOUT) {
        break;
      }

      const int64_t now_us = esp_timer_get_time();
      int64_t delay_ms =
        (one_shot.next_action_deadline_us > now_us)
          ? ((one_shot.next_action_deadline_us - now_us + 999) / 1000)
          : 1;
      if (delay_ms < 1) {
        delay_ms = 1;
      } else if (delay_ms > 20) {
        delay_ms = 20;
      }
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS((uint32_t)delay_ms));
    }
  }

  if ((state->stop_requested || state->spi_pause_requested) &&
      one_shot.phase != kMax31865OneShotIdle) {
    (void)Max31865AbortOneShot(&state->sensor, &one_shot);
  }

  if (one_shot_out != NULL) {
    *one_shot_out = one_shot;
  }
  return result;
}

/**
 * @brief Perform MAX31865 one-shot read with immediate fault confirmation
 * retry.
 * @param state Runtime state owning counters and MAX31865 reader.
 * @param sample_out Authoritative sample for this sensor loop.
 * @param used_retry_out True when a same-loop retry was attempted.
 * @param retry_faulted_out True when retry completed with a faulted sample.
 * @param first_fault_status_out Fault status from the first read fault.
 * @param retry_fault_status_out Fault status from the retry read, if any.
 * @param retry_result_out Retry read result (ESP_OK or hard error).
 * @return Result of the authoritative sample path.
 */
static esp_err_t
RuntimePerformMax31865ReadWithFaultRetry(runtime_state_t* state,
                                         max31865_sample_t* sample_out,
                                         bool* used_retry_out,
                                         bool* retry_faulted_out,
                                         uint8_t* first_fault_status_out,
                                         uint8_t* retry_fault_status_out,
                                         esp_err_t* retry_result_out)
{
  if (state == NULL || sample_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (used_retry_out != NULL) {
    *used_retry_out = false;
  }
  if (retry_faulted_out != NULL) {
    *retry_faulted_out = false;
  }
  if (first_fault_status_out != NULL) {
    *first_fault_status_out = 0u;
  }
  if (retry_fault_status_out != NULL) {
    *retry_fault_status_out = 0u;
  }
  if (retry_result_out != NULL) {
    *retry_result_out = ESP_OK;
  }

  esp_err_t first_result =
    RuntimePerformSingleMax31865Read(state, sample_out, NULL);
  if (first_result != ESP_OK || !sample_out->fault_present) {
    return first_result;
  }

  state->rtd_fault_first_read_count++;
  state->rtd_fault_last_first_status = sample_out->fault_status;
  if (first_fault_status_out != NULL) {
    *first_fault_status_out = sample_out->fault_status;
  }

  if (used_retry_out != NULL) {
    *used_retry_out = true;
  }

  max31865_sample_t retry_sample = { 0 };
  esp_err_t retry_result =
    RuntimePerformSingleMax31865Read(state, &retry_sample, NULL);
  state->rtd_fault_last_retry_err = retry_result;
  if (retry_result_out != NULL) {
    *retry_result_out = retry_result;
  }

  if (retry_result != ESP_OK) {
    return first_result;
  }

  state->rtd_fault_last_retry_status = retry_sample.fault_status;
  if (retry_fault_status_out != NULL) {
    *retry_fault_status_out = retry_sample.fault_status;
  }

  if (!retry_sample.fault_present) {
    state->rtd_fault_retry_clean_count++;
    *sample_out = retry_sample;
    return ESP_OK;
  }

  state->rtd_fault_retry_fault_count++;
  if (retry_faulted_out != NULL) {
    *retry_faulted_out = true;
  }
  *sample_out = retry_sample;
  return ESP_OK;
}

/**
 * @brief Accumulate RTD retry diagnostics for rate-limited ntfy summary jobs.
 */
static void
RuntimeAccumulateRtdFaultNtfy(runtime_state_t* state,
                              bool first_fault,
                              bool retry_fault,
                              bool retry_clean,
                              uint8_t first_status,
                              uint8_t retry_status,
                              esp_err_t retry_err,
                              int64_t now_ms)
{
  if (state == NULL || !first_fault) {
    return;
  }

  state->rtd_fault_ntfy_pending_first_read_count++;
  if (retry_fault) {
    state->rtd_fault_ntfy_pending_retry_fault_count++;
  }
  if (retry_clean) {
    state->rtd_fault_ntfy_pending_retry_clean_count++;
  }
  if (state->rtd_fault_ntfy_first_pending_ms == 0) {
    state->rtd_fault_ntfy_first_pending_ms = now_ms;
  }
  state->rtd_fault_ntfy_last_first_status = first_status;
  state->rtd_fault_ntfy_last_retry_status = retry_status;
  state->rtd_fault_last_retry_err = retry_err;
}

/**
 * @brief Check whether pending RTD retry diagnostics should be sent now.
 */
static bool
RuntimeShouldSendRtdFaultNtfy(const runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL) {
    return false;
  }

  const uint32_t pending_total =
    state->rtd_fault_ntfy_pending_first_read_count +
    state->rtd_fault_ntfy_pending_retry_fault_count +
    state->rtd_fault_ntfy_pending_retry_clean_count;
  if (pending_total == 0u) {
    return false;
  }

  if (pending_total >= kRtdFaultNtfyPendingThreshold) {
    return true;
  }

  if (state->rtd_fault_ntfy_first_pending_ms > 0 &&
      (now_ms - state->rtd_fault_ntfy_first_pending_ms) >=
        (int64_t)kRtdFaultNtfyBatchHoldoffMs) {
    return true;
  }

  if (state->rtd_fault_ntfy_last_sent_ms == 0 ||
      (now_ms - state->rtd_fault_ntfy_last_sent_ms) >=
        (int64_t)kRtdFaultNtfyMinResendIntervalMs) {
    return true;
  }

  return false;
}

/**
 * @brief Build a custom ntfy summary job from pending RTD retry diagnostics.
 */
static bool
RuntimeBuildRtdFaultSummaryNtfyJob(runtime_state_t* state,
                                   int64_t now_ms,
                                   alert_ntfy_job_t* out_job)
{
  if (state == NULL || out_job == NULL ||
      !AlertManagerIsConfigured(&state->alert_manager)) {
    return false;
  }

  memset(out_job, 0, sizeof(*out_job));
  snprintf(out_job->url,
           sizeof(out_job->url),
           "%s",
           state->alert_manager.config.ntfy_url);
  snprintf(out_job->topic,
           sizeof(out_job->topic),
           "%s",
           state->alert_manager.config.ntfy_topic);
  snprintf(out_job->token,
           sizeof(out_job->token),
           "%s",
           state->alert_manager.config.ntfy_token);
  snprintf(
    out_job->root_id, sizeof(out_job->root_id), "%s", state->node_id_string);
  snprintf(out_job->sequence_id,
           sizeof(out_job->sequence_id),
           "rtd-summary-%" PRId64 "-%" PRIu32,
           now_ms,
           state->rtd_fault_first_read_count);
  snprintf(
    out_job->title, sizeof(out_job->title), "RTD transient faults summary");
  out_job->http_timeout_ms = 1500;

  char first_desc[128] = { 0 };
  char retry_desc[128] = { 0 };
  Max31865FormatFault(
    state->rtd_fault_ntfy_last_first_status, first_desc, sizeof(first_desc));
  Max31865FormatFault(
    state->rtd_fault_ntfy_last_retry_status, retry_desc, sizeof(retry_desc));

  const int64_t batch_start_ms = state->rtd_fault_ntfy_first_pending_ms;
  const bool has_retry_status = state->rtd_fault_ntfy_last_retry_status != 0u;
  const bool has_retry_err = state->rtd_fault_last_retry_err != ESP_OK;

  snprintf(out_job->body,
           sizeof(out_job->body),
           "Raw MAX31865 RTD fault retry summary\n"
           "Batch window start: %" PRId64 " ms uptime\n"
           "Batch counts: first-read=%" PRIu32 " retry-fault=%" PRIu32
           " retry-clean=%" PRIu32 "\n"
           "Cumulative counts: first-read=%" PRIu32 " retry-fault=%" PRIu32
           " retry-clean=%" PRIu32 "\n"
           "Last first fault status: 0x%02X (%s)\n"
           "Last retry fault status: 0x%02X (%s)\n"
           "Last retry error: %s\n"
           "Meaning: retry-clean indicates a transient fault that cleared on "
           "immediate reread",
           batch_start_ms,
           state->rtd_fault_ntfy_pending_first_read_count,
           state->rtd_fault_ntfy_pending_retry_fault_count,
           state->rtd_fault_ntfy_pending_retry_clean_count,
           state->rtd_fault_first_read_count,
           state->rtd_fault_retry_fault_count,
           state->rtd_fault_retry_clean_count,
           state->rtd_fault_ntfy_last_first_status,
           first_desc,
           has_retry_status ? state->rtd_fault_ntfy_last_retry_status : 0u,
           has_retry_status ? retry_desc : "n/a",
           has_retry_err ? esp_err_to_name(state->rtd_fault_last_retry_err)
                         : "ESP_OK");

  return true;
}

/**
 * @brief Try to enqueue a pending RTD retry diagnostic ntfy summary job.
 */
static void
RuntimeMaybeSendRtdFaultSummaryNtfy(runtime_state_t* state, int64_t now_ms)
{
  if (!RuntimeShouldSendRtdFaultNtfy(state, now_ms)) {
    return;
  }

  alert_ntfy_job_t job = { 0 };
  if (!RuntimeBuildRtdFaultSummaryNtfyJob(state, now_ms, &job)) {
    return;
  }

  if (state->alert_manager.ntfy.cooldown_until_ms > now_ms) {
    state->rtd_fault_ntfy_suppressed_count++;
    return;
  }

  if (!AlertNtfyEnqueueJob(&state->alert_manager.ntfy, &job)) {
    state->rtd_fault_ntfy_suppressed_count++;
    return;
  }

  state->rtd_fault_ntfy_pending_first_read_count = 0;
  state->rtd_fault_ntfy_pending_retry_fault_count = 0;
  state->rtd_fault_ntfy_pending_retry_clean_count = 0;
  state->rtd_fault_ntfy_first_pending_ms = 0;
  state->rtd_fault_ntfy_last_sent_ms = now_ms;
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
  int64_t next_log_due_ms = 0;
  uint32_t last_log_period_ms = 0;
  bool first_log = true;

  while (!state->stop_requested) {
    if (state->spi_pause_requested) {
      state->spi_pause_ack_mask |= SPI_PAUSE_ACK_SENSOR;
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(50));
      continue;
    }
    state->spi_pause_ack_mask &= ~SPI_PAUSE_ACK_SENSOR;
    uint32_t log_period_ms = state->settings.log_period_ms;
    if (log_period_ms < 100u || log_period_ms > 3600000u) {
      log_period_ms = 1000u;
    }
    const uint32_t poll_period_ms =
      RuntimeEffectiveSensorPollPeriodMs(&state->settings);
    if (log_period_ms != last_log_period_ms) {
      first_log = true;
      last_log_period_ms = log_period_ms;
    }
    const int64_t loop_start_ms = esp_timer_get_time() / 1000;

    max31865_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    const bool ema_enabled = state->settings.rtd_ema_enabled;
    uint16_t alpha_permille = state->settings.rtd_ema_alpha_permille;
    if (alpha_permille < 1) {
      alpha_permille = 1;
    } else if (alpha_permille > 1000) {
      alpha_permille = 1000;
    }
    if (ema_enabled != state->last_rtd_ema_enabled) {
      state->sensor.ema_valid = false;
      state->last_rtd_ema_enabled = ema_enabled;
    }

    esp_err_t result = ESP_OK;
    double raw_temp_c = 0.0;
    double raw_res_ohm = 0.0;
    bool used_retry = false;
    bool retry_faulted = false;
    bool retry_clean = false;
    uint8_t first_fault_status = 0u;
    uint8_t retry_fault_status = 0u;
    esp_err_t retry_result = ESP_OK;

    // SensorTask is the sole owner of MAX31865 conversions. DisplayTask and
    // logging consume only this loop's cached sample/result.
    result = RuntimePerformMax31865ReadWithFaultRetry(state,
                                                      &sample,
                                                      &used_retry,
                                                      &retry_faulted,
                                                      &first_fault_status,
                                                      &retry_fault_status,
                                                      &retry_result);
    retry_clean = (used_retry && retry_result == ESP_OK && !retry_faulted);
    if (state->stop_requested) {
      break;
    }
    if (state->spi_pause_requested) {
      state->spi_pause_ack_mask |= SPI_PAUSE_ACK_SENSOR;
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(50));
      continue;
    }

    if (result == ESP_OK) {
      UpdateSensorEmaFromSample(
        state, ema_enabled, alpha_permille, &sample, result);
      raw_temp_c = sample.temperature_c;
      raw_res_ohm = sample.resistance_ohm;
      RuntimePublishSensorSnapshot(&sample);
    }

    const int64_t now_ms = esp_timer_get_time() / 1000;
    const bool sd_flush_spi_busy =
      (result == ESP_ERR_TIMEOUT && state->sd_flush_in_progress);
    if (result == ESP_ERR_INVALID_RESPONSE) {
      TrackSensorSpiInvalid(state, now_ms);
    } else {
      state->sensor_spi_invalid_streak = 0;
      if (result == ESP_OK) {
        SensorSpiFaultResolveIfStable(state, now_ms);
      }
    }

    const bool raw_rtd_fault_present =
      (result == ESP_OK && sample.fault_present);
    const uint8_t raw_rtd_fault_status =
      (result == ESP_OK) ? sample.fault_status : 0u;
    RuntimeAccumulateRtdFaultNtfy(state,
                                  used_retry,
                                  retry_faulted,
                                  retry_clean,
                                  first_fault_status,
                                  retry_fault_status,
                                  retry_result,
                                  now_ms);
    if (result == ESP_OK) {
      UpdateDebouncedRtdFault(state,
                              &state->settings,
                              raw_rtd_fault_present,
                              sd_flush_spi_busy,
                              now_ms,
                              raw_rtd_fault_status);
    }

    double disp_raw_temp_c = raw_temp_c;
    double disp_raw_res_ohm = raw_res_ohm;
    if (result == ESP_OK && !sample.fault_present && ema_enabled &&
        state->sensor.ema_valid) {
      disp_raw_temp_c = state->sensor.ema_temp_c;
      disp_raw_res_ohm = state->sensor.ema_resistance_ohm;
    }

    log_record_t record;
    memset(&record, 0, sizeof(record));
    record.fault_status = 0u;
    record.reserved0 = 0u;
    int64_t epoch_sec = 0;
    int32_t millis = 0;
    TimeSyncGetNow(&epoch_sec, &millis);
    const bool time_valid = TimeSyncIsSystemTimeValid();
    UpdateTimeHealthState(state, time_valid);
    UpdateCachedBool(state,
                     &state->cached_status.ntp_fail_alert_active,
                     NetSupervisorIsSntpFailureAlertActive());
    record.timestamp_epoch_sec = time_valid ? epoch_sec : (int64_t)0;
    record.timestamp_millis = time_valid ? millis : 0;

    int32_t disp_raw_milli_c = 0;
    int32_t disp_cal_milli_c = 0;
    int32_t disp_selected_milli_c = 0;
    const bool calibration_applied = RuntimeCalibrationApplied(state);
    if (result == ESP_OK) {
      record.raw_temp_milli_c = (int32_t)llround(raw_temp_c * 1000.0);
      record.temp_milli_c = record.raw_temp_milli_c;
      record.resistance_milli_ohm = (int32_t)llround(raw_res_ohm * 1000.0);

      disp_raw_milli_c = (int32_t)llround(disp_raw_temp_c * 1000.0);
      disp_cal_milli_c = disp_raw_milli_c;
      disp_selected_milli_c = disp_raw_milli_c;

      if (calibration_applied) {
        if (state->settings.calibration_domain == CAL_DOMAIN_RESISTANCE_OHM) {
          const double corrected_raw_res_ohm =
            CalibrationModelEvaluateWithPoints(
              &state->settings.calibration,
              raw_res_ohm,
              state->settings.calibration_points,
              state->settings.calibration_points_count);
          const double corrected_disp_res_ohm =
            CalibrationModelEvaluateWithPoints(
              &state->settings.calibration,
              disp_raw_res_ohm,
              state->settings.calibration_points,
              state->settings.calibration_points_count);
          const double raw_cal_c = Max31865ResistanceToTemperature(
            &state->sensor, corrected_raw_res_ohm);
          const double disp_cal_c = Max31865ResistanceToTemperature(
            &state->sensor, corrected_disp_res_ohm);
          record.temp_milli_c = (int32_t)llround(raw_cal_c * 1000.0);
          disp_cal_milli_c = (int32_t)llround(disp_cal_c * 1000.0);
        } else {
          const double raw_cal_c = CalibrationModelEvaluateWithPoints(
            &state->settings.calibration,
            raw_temp_c,
            state->settings.calibration_points,
            state->settings.calibration_points_count);
          const double disp_cal_c = CalibrationModelEvaluateWithPoints(
            &state->settings.calibration,
            disp_raw_temp_c,
            state->settings.calibration_points,
            state->settings.calibration_points_count);
          record.temp_milli_c = (int32_t)llround(raw_cal_c * 1000.0);
          disp_cal_milli_c = (int32_t)llround(disp_cal_c * 1000.0);
        }
        disp_selected_milli_c = disp_cal_milli_c;
      }

      // Intentionally do not push runtime/display samples into the calibration
      // capture window. `cal live` / `cal livecal` own that window and rely on
      // a single writer so generation/sample-count snapshots stay coherent
      // while calibrated cache refresh and validation run.
      if (sample.fault_present) {
        record.flags |= LOG_RECORD_FLAG_SENSOR_FAULT;
        record.fault_status = sample.fault_status;
      }
    } else {
      record.flags |= LOG_RECORD_FLAG_SENSOR_FAULT;
    }

    // Log sensor faults in a rate-limited way so operators see
    // wiring/open/short issues without flooding the console.
    const TickType_t now_ticks = xTaskGetTickCount();
    if (retry_clean) {
      const bool retry_rate_ok =
        (state->last_sensor_fault_log_ticks == 0) ||
        (pdTICKS_TO_MS(now_ticks - state->last_sensor_fault_log_ticks) >=
         5000u);
      if (retry_rate_ok) {
        char first_desc[128] = { 0 };
        Max31865FormatFault(first_fault_status, first_desc, sizeof(first_desc));
        ESP_LOGW(kTag,
                 "MAX31865 transient fault recovered by retry: first=0x%02X "
                 "(%s) first_count=%" PRIu32 " retry_clean_count=%" PRIu32,
                 first_fault_status,
                 first_desc,
                 state->rtd_fault_first_read_count,
                 state->rtd_fault_retry_clean_count);
        state->last_sensor_fault_log_ticks = now_ticks;
      }
    }
    if (result == ESP_OK && sample.fault_present) {
      const bool changed = !state->rtd_fault_pending_was_raw ||
                           state->rtd_fault_last_status != sample.fault_status;
      const bool rate_ok =
        (state->last_sensor_fault_log_ticks == 0) ||
        (pdTICKS_TO_MS(now_ticks - state->last_sensor_fault_log_ticks) >=
         5000u);
      if (changed || rate_ok) {
        char fault_desc[128] = { 0 };
        Max31865FormatFault(
          sample.fault_status, fault_desc, sizeof(fault_desc));
        // Avoid float formatting in logs. newlib's %f formatting is
        // stack-heavy.
        const int32_t res_milli_ohm =
          (int32_t)llround(sample.resistance_ohm * 1000.0);
        const int32_t temp_milli_c =
          (int32_t)llround(sample.temperature_c * 1000.0);
        ESP_LOGW(kTag,
                 "MAX31865 fault%s: status=0x%02X (%s) res_mohm=%" PRId32
                 " temp_mC=%" PRId32,
                 (used_retry ? " (confirmed on retry)" : ""),
                 sample.fault_status,
                 fault_desc,
                 res_milli_ohm,
                 temp_milli_c);
        state->last_sensor_fault_log_ticks = now_ticks;
      }
      state->rtd_fault_pending_was_raw = true;
      state->rtd_fault_last_status = sample.fault_status;
    } else if (result == ESP_ERR_INVALID_RESPONSE) {
      const bool rate_ok =
        (state->last_sensor_spi_log_ticks == 0) ||
        (pdTICKS_TO_MS(now_ticks - state->last_sensor_spi_log_ticks) >= 5000u);
      if (rate_ok) {
        ESP_LOGW(kTag, "MAX31865 SPI invalid response");
        state->last_sensor_spi_log_ticks = now_ticks;
      }
    } else if (sd_flush_spi_busy) {
      // Best-effort: skip fault logging/state changes when SD flush holds the
      // shared SPI bus.
    } else if (result != ESP_OK) {
      const bool changed = !state->sensor_read_err_pending ||
                           state->last_sensor_read_err != result;
      state->max31865_read_err_count++;
      state->max31865_last_read_err = result;
      state->last_sensor_read_err = result;
      state->sensor_read_err_pending = true;
      if (result == ESP_ERR_INVALID_STATE) {
        state->max31865_invalid_state_count++;
      }
      const bool rate_ok =
        (state->last_sensor_fault_log_ticks == 0) ||
        (pdTICKS_TO_MS(now_ticks - state->last_sensor_fault_log_ticks) >=
         5000u);
      if (changed || rate_ok) {
        if (result == ESP_ERR_INVALID_STATE) {
          ESP_LOGW(kTag,
                   "MAX31865 read failed: %s (init=%u phase=%s)",
                   esp_err_to_name(result),
                   state->sensor.is_initialized ? 1u : 0u,
                   Max31865OneShotPhaseToString(kMax31865OneShotIdle));
        } else {
          ESP_LOGW(kTag, "MAX31865 read failed: %s", esp_err_to_name(result));
        }
        state->last_sensor_fault_log_ticks = now_ticks;
      }

      if (result == ESP_ERR_INVALID_STATE && !sd_flush_spi_busy &&
          !state->sd_flush_in_progress) {
        esp_err_t recover_result = Max31865RecoverToBaseConfig(&state->sensor);
        if (recover_result == ESP_OK) {
          if (changed || rate_ok) {
            ESP_LOGW(kTag, "MAX31865 recover-to-base succeeded");
          }
        } else if (changed || rate_ok) {
          ESP_LOGW(kTag,
                   "MAX31865 recover-to-base failed: %s",
                   esp_err_to_name(recover_result));
        }
      }
    } else {
      if (state->rtd_fault_pending_was_raw &&
          state->rtd_fault_last_status != 0) {
        char fault_desc[128] = { 0 };
        Max31865FormatFault(
          state->rtd_fault_last_status, fault_desc, sizeof(fault_desc));
        ESP_LOGW(kTag,
                 "MAX31865 fault cleared (last=0x%02X (%s))",
                 state->rtd_fault_last_status,
                 fault_desc);
      }
      if (state->sensor_read_err_pending) {
        ESP_LOGW(kTag,
                 "MAX31865 read recovered (last_err=%s)",
                 esp_err_to_name(state->last_sensor_read_err));
      }
      state->rtd_fault_pending_was_raw = false;
      state->sensor_read_err_pending = false;
    }
    UpdateCachedBool(state,
                     &state->cached_status.sensor_fault_present,
                     state->last_sensor_fault_present);

    if (time_valid) {
      record.flags |= LOG_RECORD_FLAG_TIME_VALID;
    }
    const bool cal_valid = RuntimeCalibrationApplied(state);
    if (cal_valid) {
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

    const bool temp_valid = (result == ESP_OK && !sample.fault_present);

    taskENTER_CRITICAL(&state->last_temp_lock);
    state->last_temp_milli_c = disp_selected_milli_c;
    state->last_temp_valid = temp_valid;
    state->last_flags = record.flags;
    state->last_update_ticks = xTaskGetTickCount();
    taskEXIT_CRITICAL(&state->last_temp_lock);

    bool log_due = false;
    if (first_log) {
      log_due = true;
      next_log_due_ms = now_ms + (int64_t)log_period_ms;
      first_log = false;
    } else if (now_ms >= next_log_due_ms) {
      log_due = true;
      while (next_log_due_ms <= now_ms) {
        next_log_due_ms += (int64_t)log_period_ms;
      }
    }

    if (log_due) {
      sensor_sample_msg_t msg = {
        .record = record,
        .disp_raw_temp_milli_c = disp_raw_milli_c,
        .disp_cal_temp_milli_c = disp_cal_milli_c,
      };
      (void)xQueueSend(state->log_queue, &msg, 0);
    }

    const int64_t elapsed_ms = (esp_timer_get_time() / 1000) - loop_start_ms;
    const int64_t remaining_ms = (int64_t)poll_period_ms - elapsed_ms;
    if (remaining_ms > 0) {
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS((uint32_t)remaining_ms));
    }
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

static bool
NetTxDropExportHead(runtime_state_t* state)
{
  if (state == NULL || state->export_outbox == NULL) {
    return false;
  }
  export_record_item_t* dequeued = NULL;
  if (xQueueReceive(state->export_outbox, &dequeued, 0) == pdTRUE) {
    if (dequeued != NULL) {
      MemPoolFree(&g_export_outbox_pool, dequeued);
    }
    return true;
  }
  return false;
}

static bool
NetTxDropBrokerHead(runtime_state_t* state)
{
  if (state == NULL || state->broker_outbox == NULL) {
    return false;
  }
  broker_publish_item_t* dequeued = NULL;
  if (xQueueReceive(state->broker_outbox, &dequeued, 0) == pdTRUE) {
    if (dequeued != NULL) {
      MemPoolFree(&g_broker_outbox_pool, dequeued);
    }
    return true;
  }
  return false;
}

static void
NetTxStallDropExport(runtime_state_t* state)
{
  if (!NetTxDropExportHead(state)) {
    return;
  }
  state->export_drop_count++;
  UpdateCachedUint32(
    state, &state->cached_status.export_drop_count, state->export_drop_count);
  if (LogRateLimitAllow(&state->last_export_drop_log_ms,
                        kExportLogRateLimitMs)) {
    ESP_LOGW(kTag, "export outbox stalled; dropping oldest record");
  }
}

static void
NetTxStallDropBroker(runtime_state_t* state)
{
  if (!NetTxDropBrokerHead(state)) {
    return;
  }
  state->broker_drop_count++;
  UpdateCachedUint32(
    state, &state->cached_status.broker_drop_count, state->broker_drop_count);
  if (LogRateLimitAllow(&state->last_broker_drop_log_ms,
                        kExportLogRateLimitMs)) {
    ESP_LOGW(kTag, "broker outbox stalled; dropping oldest record");
  }
}

static void
NetTxBackoffUpdate(uint32_t* backoff_ms,
                   int64_t* next_attempt_ms,
                   int64_t now_ms)
{
  if (backoff_ms == NULL || next_attempt_ms == NULL) {
    return;
  }
  if (*backoff_ms == 0) {
    *backoff_ms = 200;
  } else {
    *backoff_ms = (*backoff_ms < 2000) ? (*backoff_ms * 2) : 2000;
  }
  *next_attempt_ms = now_ms + (int64_t)(*backoff_ms);
}

static bool
NetTxHandleExportLeaf(runtime_state_t* state,
                      int64_t now_ms,
                      int64_t* fail_start_ms,
                      int64_t* next_attempt_ms,
                      uint32_t* backoff_ms)
{
  if (state == NULL || state->export_outbox == NULL) {
    return false;
  }
  if (next_attempt_ms != NULL && now_ms < *next_attempt_ms) {
    return false;
  }

  export_record_item_t* item = NULL;
  if (xQueuePeek(state->export_outbox, &item, 0) != pdTRUE) {
    return false;
  }
  if (item == NULL) {
    (void)xQueueReceive(state->export_outbox, &item, 0);
    return true;
  }

  bool sent = false;
  if (state->net_mode_active == APP_NET_MODE_MESH) {
    if (MeshTransportIsConnected(&state->mesh)) {
      const esp_err_t send_result = MeshTransportSendPublishRecord(
        &state->mesh, item->src_mac, &item->record);
      if (send_result == ESP_OK) {
        sent = true;
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
      }
    }
  } else {
    const bool should_mqtt = state->mqtt_enabled_active &&
                             state->net_mode_active == APP_NET_MODE_DIRECT_WIFI;
    EnsureMqttClientState(state, should_mqtt);
    if (state->mqtt_client_connected) {
      char node_id[32] = { 0 };
      FormatMacString(item->src_mac, node_id, sizeof(node_id));
      char topic[128] = { 0 };
      if (!BuildMqttTopic(
            state->mqtt_topic_prefix_active, node_id, topic, sizeof(topic))) {
        state->export_send_fail_count++;
        UpdateCachedUint32(state,
                           &state->cached_status.export_send_fail_count,
                           state->export_send_fail_count);
        NetTxDropExportHead(state);
        if (fail_start_ms != NULL) {
          *fail_start_ms = 0;
        }
        if (backoff_ms != NULL) {
          *backoff_ms = 0;
        }
        if (next_attempt_ms != NULL) {
          *next_attempt_ms = 0;
        }
        return true;
      }

      size_t payload_len = 0;
      char payload[256] = { 0 };
      if (!BuildMqttPayload(
            &item->record, node_id, payload, sizeof(payload), &payload_len)) {
        state->export_send_fail_count++;
        UpdateCachedUint32(state,
                           &state->cached_status.export_send_fail_count,
                           state->export_send_fail_count);
        NetTxDropExportHead(state);
        if (fail_start_ms != NULL) {
          *fail_start_ms = 0;
        }
        if (backoff_ms != NULL) {
          *backoff_ms = 0;
        }
        if (next_attempt_ms != NULL) {
          *next_attempt_ms = 0;
        }
        return true;
      }

      const esp_err_t publish_result =
        MqttClientWrapPublish(&state->mqtt_client,
                              topic,
                              payload,
                              (int)payload_len,
                              state->mqtt_qos_active,
                              state->mqtt_retain_active ? 1 : 0);
      if (publish_result == ESP_OK) {
        sent = true;
      } else {
        state->export_send_fail_count++;
        UpdateCachedUint32(state,
                           &state->cached_status.export_send_fail_count,
                           state->export_send_fail_count);
        if (LogRateLimitAllow(&state->last_export_fail_log_ms,
                              kExportLogRateLimitMs)) {
          ESP_LOGW(kTag, "MQTT publish failed");
        }
      }
    }
  }

  if (sent) {
    NetTxDropExportHead(state);
    if (fail_start_ms != NULL) {
      *fail_start_ms = 0;
    }
    if (backoff_ms != NULL) {
      *backoff_ms = 0;
    }
    if (next_attempt_ms != NULL) {
      *next_attempt_ms = 0;
    }
    return true;
  }

  if (fail_start_ms != NULL) {
    if (*fail_start_ms == 0) {
      *fail_start_ms = now_ms;
    } else if (now_ms - *fail_start_ms >= kNetTxStallDropMs) {
      NetTxStallDropExport(state);
      *fail_start_ms = 0;
    }
  }
  NetTxBackoffUpdate(backoff_ms, next_attempt_ms, now_ms);
  return true;
}

static bool
NetTxHandleBridgeRoot(runtime_state_t* state)
{
  if (state == NULL || state->export_outbox == NULL ||
      !state->root_publish_consumer_active) {
    return false;
  }

  export_record_item_t* item = NULL;
  if (xQueueReceive(state->export_outbox, &item, 0) != pdTRUE) {
    return false;
  }
  if (item == NULL) {
    return true;
  }

  char node_id[32] = { 0 };
  FormatMacString(item->src_mac, node_id, sizeof(node_id));

  char payload[256] = { 0 };
  size_t payload_len = 0;
  if (!BuildMqttPayload(
        &item->record, node_id, payload, sizeof(payload), &payload_len)) {
    state->export_send_fail_count++;
    UpdateCachedUint32(state,
                       &state->cached_status.export_send_fail_count,
                       state->export_send_fail_count);
    MemPoolFree(&g_export_outbox_pool, item);
    return true;
  }

  if (BridgeModeUsesSerial(state->mqtt_bridge_mode_active)) {
    if (!TryEmitBridgeCsvHeader(state)) {
      MemPoolFree(&g_export_outbox_pool, item);
      return true;
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
    if (state->broker_outbox == NULL) {
      state->broker_drop_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.broker_drop_count,
                         state->broker_drop_count);
      if (LogRateLimitAllow(&state->last_broker_drop_log_ms,
                            kExportLogRateLimitMs)) {
        ESP_LOGW(kTag, "broker outbox unavailable; dropping publish");
      }
      MemPoolFree(&g_export_outbox_pool, item);
      return true;
    }
    broker_publish_item_t* publish_item =
      (broker_publish_item_t*)MemPoolAlloc(&g_broker_outbox_pool);
    if (publish_item == NULL) {
      state->broker_drop_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.broker_drop_count,
                         state->broker_drop_count);
      if (LogRateLimitAllow(&state->last_broker_drop_log_ms,
                            kExportLogRateLimitMs)) {
        ESP_LOGW(kTag, "broker outbox pool empty; dropping publish");
      }
      MemPoolFree(&g_export_outbox_pool, item);
      return true;
    }
    memset(publish_item, 0, sizeof(*publish_item));
    if (!BuildMqttTopic(state->mqtt_topic_prefix_active,
                        node_id,
                        publish_item->topic,
                        sizeof(publish_item->topic))) {
      state->broker_send_fail_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.broker_send_fail_count,
                         state->broker_send_fail_count);
      MemPoolFree(&g_broker_outbox_pool, publish_item);
      MemPoolFree(&g_export_outbox_pool, item);
      return true;
    }
    if (payload_len >= sizeof(publish_item->payload)) {
      state->broker_send_fail_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.broker_send_fail_count,
                         state->broker_send_fail_count);
      MemPoolFree(&g_broker_outbox_pool, publish_item);
      MemPoolFree(&g_export_outbox_pool, item);
      return true;
    }
    memcpy(publish_item->payload, payload, payload_len);
    publish_item->payload_len = (uint16_t)payload_len;

    if (xQueueSend(state->broker_outbox, &publish_item, 0) != pdTRUE) {
      state->broker_drop_count++;
      UpdateCachedUint32(state,
                         &state->cached_status.broker_drop_count,
                         state->broker_drop_count);
      if (LogRateLimitAllow(&state->last_broker_drop_log_ms,
                            kExportLogRateLimitMs)) {
        ESP_LOGW(kTag, "broker outbox full; dropping publish");
      }
      MemPoolFree(&g_broker_outbox_pool, publish_item);
    }
  }
  MemPoolFree(&g_export_outbox_pool, item);
  return true;
}

static bool
NetTxHandleBrokerPublish(runtime_state_t* state,
                         int64_t now_ms,
                         int64_t* fail_start_ms,
                         int64_t* next_attempt_ms,
                         uint32_t* backoff_ms)
{
  if (state == NULL || state->broker_outbox == NULL) {
    return false;
  }
  if (next_attempt_ms != NULL && now_ms < *next_attempt_ms) {
    return false;
  }

  broker_publish_item_t* item = NULL;
  if (xQueuePeek(state->broker_outbox, &item, 0) != pdTRUE) {
    return false;
  }
  if (item == NULL) {
    NetTxDropBrokerHead(state);
    return true;
  }

  const bool should_mqtt = state->mqtt_enabled_active &&
                           BridgeModeUsesBroker(state->mqtt_bridge_mode_active);
  EnsureMqttClientState(state, should_mqtt);
  if (!state->mqtt_client_connected) {
    if (fail_start_ms != NULL) {
      if (*fail_start_ms == 0) {
        *fail_start_ms = now_ms;
      } else if (now_ms - *fail_start_ms >= kNetTxStallDropMs) {
        NetTxStallDropBroker(state);
        *fail_start_ms = 0;
      }
    }
    NetTxBackoffUpdate(backoff_ms, next_attempt_ms, now_ms);
    return true;
  }

  const esp_err_t publish_result =
    MqttClientWrapPublish(&state->mqtt_client,
                          item->topic,
                          item->payload,
                          (int)item->payload_len,
                          state->mqtt_qos_active,
                          state->mqtt_retain_active ? 1 : 0);
  if (publish_result == ESP_OK) {
    NetTxDropBrokerHead(state);
    if (fail_start_ms != NULL) {
      *fail_start_ms = 0;
    }
    if (backoff_ms != NULL) {
      *backoff_ms = 0;
    }
    if (next_attempt_ms != NULL) {
      *next_attempt_ms = 0;
    }
    return true;
  }

  state->broker_send_fail_count++;
  UpdateCachedUint32(state,
                     &state->cached_status.broker_send_fail_count,
                     state->broker_send_fail_count);
  if (LogRateLimitAllow(&state->last_broker_fail_log_ms,
                        kExportLogRateLimitMs)) {
    ESP_LOGW(kTag, "broker publish failed");
  }
  if (fail_start_ms != NULL) {
    if (*fail_start_ms == 0) {
      *fail_start_ms = now_ms;
    } else if (now_ms - *fail_start_ms >= kNetTxStallDropMs) {
      NetTxStallDropBroker(state);
      *fail_start_ms = 0;
    }
  }
  NetTxBackoffUpdate(backoff_ms, next_attempt_ms, now_ms);
  return true;
}

static bool
NetTxHandleAlertSend(runtime_state_t* state,
                     int64_t now_ms,
                     int64_t* next_attempt_ms)
{
  if (state == NULL || !AlertManagerIsConfigured(&state->alert_manager)) {
    return false;
  }
  if (state->alert_manager.ntfy.queue == NULL) {
    return false;
  }
  if (next_attempt_ms != NULL && now_ms < *next_attempt_ms) {
    return false;
  }
  const uint32_t dropped_before = state->alert_manager.ntfy.job_dropped;
  const bool did_work =
    AlertManagerPumpNtfy(&state->alert_manager, now_ms, next_attempt_ms);
  if (state->alert_manager.ntfy.job_dropped != dropped_before) {
    int64_t now_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    if (LogRateLimitAllow(&state->last_ntfy_queue_full_log_ms,
                          kExportLogRateLimitMs)) {
      (void)RuntimeEnqueueSystemErrorNote(state,
                                          ALERT_SYSTEM_CODE_ERROR_NTFY_QUEUE,
                                          false,
                                          now_epoch,
                                          (int64_t)now_ms);
    }
  }
  return did_work;
}

static void
NetTxDrainAlertQueue(runtime_state_t* state)
{
  if (state == NULL || state->alert_manager.ntfy.queue == NULL) {
    return;
  }
  alert_notification_t note;
  while (xQueueReceive(state->alert_manager.ntfy.queue, &note, 0) == pdTRUE) {
    (void)note;
  }
}

/**
 * @brief Create the alert_http task with a PSRAM-backed stack.
 * @param state Runtime state (owns the task memory).
 * @param stack_bytes Stack size in bytes.
 * @return pdPASS on success, or pdFAIL on failure.
 */
static BaseType_t
CreateAlertHttpTaskWithPsrStack(runtime_state_t* state, uint32_t stack_bytes)
{
  if (state == NULL) {
    return pdFAIL;
  }
  const size_t stack_words =
    (stack_bytes + sizeof(StackType_t) - 1) / sizeof(StackType_t);
  const size_t stack_alloc_bytes = stack_words * sizeof(StackType_t);
  if (state->alert_http_task_stack == NULL) {
    state->alert_http_task_stack =
      heap_caps_aligned_alloc(alignof(StackType_t),
                              stack_alloc_bytes,
                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (state->alert_http_task_tcb == NULL) {
    state->alert_http_task_tcb = heap_caps_calloc(
      1, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (state->alert_http_task_stack == NULL) {
    if (!g_alert_http_psram_failure_logged) {
      ESP_LOGE(kTag, "Failed to allocate PSRAM for alert_http task");
      g_alert_http_psram_failure_logged = true;
    }
    return pdFAIL;
  }
  if (state->alert_http_task_tcb == NULL) {
    return pdFAIL;
  }
  TaskHandle_t task =
    xTaskCreateStaticPinnedToCore(AlertHttpTask,
                                  "alert_http",
                                  stack_words,
                                  state,
                                  3,
                                  state->alert_http_task_stack,
                                  state->alert_http_task_tcb,
                                  tskNO_AFFINITY);
  if (task == NULL) {
    if (!g_alert_http_psram_failure_logged) {
      ESP_LOGE(kTag, "Failed to create alert_http task with PSRAM stack");
      g_alert_http_psram_failure_logged = true;
    }
    return pdFAIL;
  }
  state->alert_http_task = task;
  return pdPASS;
}

/**
 * @brief Execute AlertHttpTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the alert_http task.
 */
static void
RuntimeLogNtfyBodyLines(const char* label, const char* text)
{
  const char* line_label = (label != NULL) ? label : "unknown";
  if (text == NULL || text[0] == '\0') {
    ESP_LOGI(kTag, "runtime: ntfy body %s: <empty>", line_label);
    return;
  }

  const size_t raw_len = strnlen(text, ALERT_NTFY_JOB_BODY_LEN);
  const bool truncated = (raw_len == ALERT_NTFY_JOB_BODY_LEN);
  char cleaned[ALERT_NTFY_JOB_BODY_LEN + 1];
  size_t cleaned_len = 0;
  for (size_t i = 0; i < raw_len && cleaned_len < ALERT_NTFY_JOB_BODY_LEN;
       ++i) {
    if (text[i] == '\r') {
      continue;
    }
    cleaned[cleaned_len++] = text[i];
  }
  cleaned[cleaned_len] = '\0';

  ESP_LOGI(kTag,
           "runtime: ntfy body %s BEGIN\n%s\nruntime: ntfy body %s END%s",
           line_label,
           cleaned,
           line_label,
           truncated ? " (truncated)" : "");
}

static void
RuntimeLogNtfyJob(const alert_ntfy_job_t* job, const char* stage)
{
  if (job == NULL) {
    return;
  }
  const char* stage_name = (stage != NULL) ? stage : "unknown";
  ESP_LOGI(kTag,
           "runtime: ntfy job %s seq_id=\"%s\" title=\"%s\" attempt=%" PRIu32,
           stage_name,
           job->sequence_id,
           job->title,
           job->attempt + 1);
  RuntimeLogNtfyBodyLines(stage_name, job->body);
}

static void
AppendLineBounded(char* dest, size_t dest_size, const char* line)
{
  if (dest == NULL || line == NULL || dest_size == 0u) {
    return;
  }

  const size_t used = strnlen(dest, dest_size);
  if (used >= dest_size - 1u) {
    dest[dest_size - 1u] = '\0';
    return;
  }

  size_t write_pos = used;
  if (write_pos > 0u && dest[write_pos - 1u] != '\n') {
    dest[write_pos++] = '\n';
  }

  const size_t remaining = dest_size - write_pos;
  if (remaining == 0u) {
    dest[dest_size - 1u] = '\0';
    return;
  }

  const size_t line_len = strlen(line);
  const size_t copy_len =
    (line_len < (remaining - 1u)) ? line_len : (remaining - 1u);
  if (copy_len > 0u) {
    memcpy(dest + write_pos, line, copy_len);
    write_pos += copy_len;
  }
  dest[write_pos] = '\0';
}

static void
AppendStopStorageSummaryLine(runtime_state_t* state, alert_ntfy_job_t* job)
{
  if (state == NULL || job == NULL) {
    return;
  }

  const runtime_cached_status_t* cached = &state->cached_status;
  const bool sd_unhealthy = cached->sd_io_error_active || cached->sd_degraded ||
                            cached->sd_out_of_space_active ||
                            !cached->sd_card_present;
  const unsigned int sd_io = cached->sd_io_error_active ? 1U : 0U;
  const unsigned int degraded = cached->sd_degraded ? 1U : 0U;
  const unsigned int mounted = cached->sd_mounted ? 1U : 0U;
  const unsigned int present = cached->sd_card_present ? 1U : 0U;
  const unsigned int oos = cached->sd_out_of_space_active ? 1U : 0U;
  const unsigned int overrun = cached->fram_overrun_active ? 1U : 0U;

  char storage_line[256];
  if (sd_unhealthy) {
    (void)snprintf(storage_line,
                   sizeof(storage_line),
                   "CRITICAL STORAGE: SD logging FAILED (sd_io=%u degraded=%u "
                   "mounted=%u present=%u fail=%" PRIu32 " backoff_ms=%" PRIu32
                   " oos=%u) -> FRAM %" PRIu32 "/%" PRIu32
                   " overrun=%u (DATA LOSS LIKELY)",
                   sd_io,
                   degraded,
                   mounted,
                   present,
                   cached->sd_fail_count,
                   cached->sd_backoff_remaining_ms,
                   oos,
                   cached->fram_count,
                   cached->fram_capacity,
                   overrun);
  } else {
    (void)snprintf(storage_line,
                   sizeof(storage_line),
                   "STORAGE: SD OK (mounted=%u present=%u free=%" PRIu64 ") -> "
                   "FRAM %" PRIu32 "/%" PRIu32 " overrun=%u",
                   mounted,
                   present,
                   (uint64_t)cached->sd_free_bytes,
                   cached->fram_count,
                   cached->fram_capacity,
                   overrun);
  }

  AppendLineBounded(job->body, sizeof(job->body), storage_line);
}

static void
AlertHttpTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  if (state == NULL) {
    vTaskDelete(NULL);
    return;
  }

  uint32_t min_stack_hwm_bytes = UINT32_MAX;
  while (!state->stop_requested) {
    if (state->alert_http_pause_requested) {
      state->alert_http_paused = true;
      while (state->alert_http_pause_requested && !state->stop_requested) {
        RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(20));
      }
      state->alert_http_paused = false;
      if (state->stop_requested) {
        ESP_LOGW(kTag, "alert_http pause interrupted by stop request");
        break;
      }
    }

    const uint32_t hwm_bytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    if (hwm_bytes < min_stack_hwm_bytes) {
      min_stack_hwm_bytes = hwm_bytes;
      ESP_LOGI(kTag,
               "alert_http stack watermark: %u bytes free",
               (unsigned)min_stack_hwm_bytes);
    }

    alert_ntfy_job_t job = { 0 };
    if (state->alert_manager.ntfy.job_queue == NULL ||
        xQueueReceive(state->alert_manager.ntfy.job_queue,
                      &job,
                      pdMS_TO_TICKS(250)) != pdTRUE) {
      continue;
    }
    RuntimeLogNtfyJob(&job, "dequeued");

    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t ready_ms = job.next_attempt_ms;
    if (state->alert_manager.ntfy.cooldown_until_ms > ready_ms) {
      ready_ms = state->alert_manager.ntfy.cooldown_until_ms;
    }
    if (now_ms < ready_ms) {
      RuntimeInterruptibleDelayTicks(
        pdMS_TO_TICKS((uint32_t)(ready_ms - now_ms)));
      if (state->stop_requested) {
        ESP_LOGW(
          kTag,
          "alert_http stop requested before send; job retained title=\"%s\"",
          job.title);
        break;
      }
    }

    now_ms = esp_timer_get_time() / 1000;
    if (!AlertManagerIsConfigured(&state->alert_manager) ||
        !AlertManagerCanSend(state)) {
      job.next_attempt_ms = now_ms + kAlertHttpRetryDelayMs;
      (void)AlertNtfyEnqueueJob(&state->alert_manager.ntfy, &job);
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(kAlertHttpRetryDelayMs));
      continue;
    }

    alert_ntfy_config_t cfg = {
      .url = job.url,
      .topic = job.topic,
      .token = job.token,
      .root_id = job.root_id,
      .http_timeout_ms = job.http_timeout_ms,
      .attempt = job.attempt + 1u,
      .is_retry = (job.attempt > 0u),
      .queue_depth =
        (state->alert_manager.ntfy.job_queue != NULL)
          ? (uint32_t)uxQueueMessagesWaiting(state->alert_manager.ntfy.job_queue)
          : 0u,
      .task_stack_free_bytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL),
      .task_name = pcTaskGetName(NULL),
    };

    int status = 0;
    int retry_after_seconds = -1;
    esp_err_t err = ESP_OK;
    RuntimeLogNtfyJob(&job, "attempt");
    alert_ntfy_result_t result = AlertNtfySendText(&state->alert_manager.ntfy,
                                                   &cfg,
                                                   job.title,
                                                   job.body,
                                                   job.sequence_id,
                                                   &retry_after_seconds,
                                                   &status,
                                                   &err);
    AlertManagerUpdateNtfySendState(&state->alert_manager,
                                    result,
                                    status,
                                    retry_after_seconds,
                                    err,
                                    now_ms,
                                    NULL);

    const char* result_name = "failed";
    if (result == ALERT_NTFY_OK) {
      result_name = "ok";
    } else if (result == ALERT_NTFY_SKIPPED) {
      result_name = "skipped";
    }

    ESP_LOGI(kTag,
             "ntfy job %s: status=%d err=%s (%d) attempt=%" PRIu32
             " title=\"%s\"",
             result_name,
             status,
             esp_err_to_name(err),
             (int)err,
             job.attempt + 1,
             job.title);

    if (result == ALERT_NTFY_FAILED) {
      if (LogRateLimitAllow(&state->alert_manager.ntfy.last_send_fail_log_ms,
                            kAlertHttpSendFailLogRateLimitMs)) {
        ESP_LOGW(kTag,
                 "ntfy send failed: err=%s status=%d retry_after=%d",
                 esp_err_to_name(err),
                 status,
                 retry_after_seconds);
      }
      if (job.attempt + 1 < kAlertHttpMaxAttempts) {
        job.attempt++;
        job.next_attempt_ms = state->alert_manager.ntfy.cooldown_until_ms;
        (void)AlertNtfyEnqueueJob(&state->alert_manager.ntfy, &job);
      }
    }
  }

  state->alert_http_paused = false;
  state->alert_http_task = NULL;
  vTaskDelete(NULL);
}

static void
NetTxTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  int64_t export_fail_start_ms = 0;
  int64_t broker_fail_start_ms = 0;
  int64_t export_next_attempt_ms = 0;
  int64_t broker_next_attempt_ms = 0;
  int64_t alert_next_attempt_ms = 0;
  uint32_t export_backoff_ms = 0;
  uint32_t broker_backoff_ms = 0;
  uint32_t min_stack_hwm_bytes = UINT32_MAX;

  while (!state->stop_requested) {
    if (state->net_tx_pause_requested) {
      state->net_tx_paused = true;
      while (state->net_tx_pause_requested && !state->stop_requested) {
        RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(20));
      }
      state->net_tx_paused = false;
      if (state->stop_requested) {
        break;
      }
    }

    const uint32_t hwm_bytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    if (hwm_bytes < min_stack_hwm_bytes) {
      min_stack_hwm_bytes = hwm_bytes;
      ESP_LOGI(kTag,
               "net_tx stack watermark: %u bytes free",
               (unsigned)min_stack_hwm_bytes);
    }

    const int64_t now_ms = esp_timer_get_time() / 1000;
    const bool alert_can_send = AlertManagerCanSend(state);
    const bool alert_suppressed = AlertManagerSuppressed(state);
    bool did_work = false;

    if (state->node_role_active == APP_NODE_ROLE_ROOT) {
      did_work |= NetTxHandleBridgeRoot(state);
      if (BridgeModeUsesBroker(state->mqtt_bridge_mode_active) &&
          state->mqtt_enabled_active) {
        did_work |= NetTxHandleBrokerPublish(state,
                                             now_ms,
                                             &broker_fail_start_ms,
                                             &broker_next_attempt_ms,
                                             &broker_backoff_ms);
      }
      if (alert_can_send && AlertManagerIsConfigured(&state->alert_manager)) {
        did_work |= NetTxHandleAlertSend(state, now_ms, &alert_next_attempt_ms);
      } else if (alert_suppressed) {
        NetTxDrainAlertQueue(state);
      }
    } else {
      did_work |= NetTxHandleExportLeaf(state,
                                        now_ms,
                                        &export_fail_start_ms,
                                        &export_next_attempt_ms,
                                        &export_backoff_ms);
      if (alert_can_send && AlertManagerIsConfigured(&state->alert_manager)) {
        did_work |= NetTxHandleAlertSend(state, now_ms, &alert_next_attempt_ms);
      } else if (alert_suppressed) {
        NetTxDrainAlertQueue(state);
      }
    }

    if (!did_work) {
      RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(50));
    }
  }

  DrainExportOutboxQueue(state);
  DrainBrokerOutboxQueue(state);
  NetTxDrainAlertQueue(state);
  state->net_tx_paused = false;
  state->net_tx_task = NULL;
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
  // SD flush snapshots legitimately hold the FRAM log mutex while scanning and
  // batching records. Ensure storage can wait out a snapshot and still commit
  // samples to FRAM instead of skipping writes.
  const TickType_t fram_log_timeout_ticks =
    pdMS_TO_TICKS(kSdFlushMaxMsPerPass * 4u);

  while (!state->stop_requested ||
         uxQueueMessagesWaiting(state->log_queue) > 0) {
    if (state->sd_reset_pending) {
      SdResetPendingTick(state);
    }
    sensor_sample_msg_t msg;
    RuntimeMarkersSetStorage(state, STORAGE_010_BEFORE_QUEUE_RECV);
    const bool received =
      (xQueueReceive(state->log_queue, &msg, pdMS_TO_TICKS(500)) == pdTRUE);
    if (received) {
      RuntimeMarkersSetStorage(state, STORAGE_020_AFTER_QUEUE_RECV);
      log_record_t record = msg.record;
      if (state->i2c_quiesce_active) {
        if (record.sequence == 0u || record.record_id == 0u) {
          RuntimeAssignFallbackRecordIds(state, &record);
        }
        if (!DeferredLogPush(state, &record)) {
          taskENTER_CRITICAL(&state->deferred_log.lock);
          state->deferred_log.drops++;
          taskEXIT_CRITICAL(&state->deferred_log.lock);
          if (LogRateLimitAllow(&state->deferred_drop_last_log_ms,
                                kDeferredDropWarnIntervalMs)) {
            ESP_LOGW(
              kTag,
              "Deferred log full during SD flush; dropping samples (drops=%u)",
              (unsigned)DeferredLogDrops(state));
          }
        }
        UpdateCachedUint32(
          state, &state->cached_status.deferred_count, DeferredLogCount(state));
        UpdateCachedUint32(
          state, &state->cached_status.deferred_drops, DeferredLogDrops(state));
        UpdateCachedBool(state,
                         &state->cached_status.deferred_active,
                         (DeferredLogCount(state) > 0u));
        const int64_t now_ms = esp_timer_get_time() / 1000;
        const int64_t now_epoch =
          TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
        log_record_t alert_record = record;
        alert_record.raw_temp_milli_c = msg.disp_raw_temp_milli_c;
        alert_record.temp_milli_c = msg.disp_cal_temp_milli_c;
        RuntimeMarkersSetStorage(state, STORAGE_040_ALERT_MANAGER);
        AlertManagerOnSample(&state->alert_manager,
                             state->local_leaf_id,
                             &alert_record,
                             now_ms,
                             now_epoch);
        RuntimeMarkersSetStorage(state, STORAGE_060_EXPORT_UART_ENQUEUE);
        EnqueueExportRecord(state, state->node_id_string, &record);
        if (state->mqtt_enabled_active) {
          RuntimeMarkersSetStorage(state, STORAGE_070_MQTT_ENQUEUE);
          if (state->node_role_active == APP_NODE_ROLE_ROOT) {
            if (BridgeModeUsesBroker(state->mqtt_bridge_mode_active)) {
              EnqueueBrokerPublish(state, state->local_mac, &record);
            }
          } else {
            EnqueueExportOutbox(state, state->local_mac, &record);
          }
        }
        continue;
      }
      const bool fram_available = state->fram_available;
      bool fram_lock_ok = false;
      if (fram_available) {
        fram_lock_ok = RuntimeFramLogLockWithWarn(
          state,
          fram_log_timeout_ticks,
          &state->fram_log_lock_timeout_count_storage,
          &state->last_fram_log_lock_timeout_storage_log_ms,
          "storage");
      }
      esp_err_t id_result = fram_available
                              ? ESP_ERR_TIMEOUT
                              : RuntimeAssignRecordIds(state, &record);
      if (fram_available && fram_lock_ok) {
        id_result = RuntimeAssignRecordIds(state, &record);
      }
      RuntimeMarkersSetStorage(state, STORAGE_030_ASSIGN_IDS);
      if (id_result != ESP_OK) {
        if (fram_available && fram_lock_ok) {
          ESP_LOGE(
            kTag, "Failed to assign record id: %s", esp_err_to_name(id_result));
        }
      } else if (state->time_jump_back_arm_next) {
        record.flags |= LOG_RECORD_FLAG_TIME_JUMP_BACK;
        state->time_jump_back_attempt_record_id = record.record_id;
        state->time_jump_back_pending_confirm = true;
        state->time_jump_back_arm_next = false;
      }
      const int64_t now_ms = esp_timer_get_time() / 1000;
      const int64_t now_epoch =
        TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
      log_record_t alert_record = record;
      alert_record.raw_temp_milli_c = msg.disp_raw_temp_milli_c;
      alert_record.temp_milli_c = msg.disp_cal_temp_milli_c;
      RuntimeMarkersSetStorage(state, STORAGE_040_ALERT_MANAGER);
      AlertManagerOnSample(&state->alert_manager,
                           state->local_leaf_id,
                           &alert_record,
                           now_ms,
                           now_epoch);

      if (fram_available && fram_lock_ok) {
        RuntimeMarkersSetStorage(state, STORAGE_050_FRAM_APPEND);
        esp_err_t append_result = FramLogAppend(&state->fram_log, &record);
        if (append_result != ESP_OK) {
          ESP_LOGE(
            kTag, "FRAM append failed: %s", esp_err_to_name(append_result));
          TrackFramAppendFailure(state, append_result);
        } else {
          state->fram_append_fail_streak = 0;
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
        UpdateFramOverrunActive(state, state->last_overrun_records_total);
        UpdateFramFillState(state);
        if (state->fram_full) {
          record.flags |= LOG_RECORD_FLAG_FRAM_FULL;
        }
      }
      if (fram_available && fram_lock_ok) {
        RuntimeFramLogUnlock(state);
      }

      // if (!state->mesh.is_root && MeshTransportIsConnected(&state->mesh)) {
      //   (void)MeshTransportSendRecord(&state->mesh, &record);
      // }

      RuntimeMarkersSetStorage(state, STORAGE_060_EXPORT_UART_ENQUEUE);
      EnqueueExportRecord(state, state->node_id_string, &record);

      if (state->mqtt_enabled_active) {
        RuntimeMarkersSetStorage(state, STORAGE_070_MQTT_ENQUEUE);
        if (state->node_role_active == APP_NODE_ROLE_ROOT) {
          if (BridgeModeUsesBroker(state->mqtt_bridge_mode_active)) {
            EnqueueBrokerPublish(state, state->local_mac, &record);
          }
        } else {
          EnqueueExportOutbox(state, state->local_mac, &record);
        }
      }
    }
  }

  state->storage_task = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Execute SdFlushTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the SdFlushTask task.
 */
static void
SdFlushTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  state->last_flush_ticks = xTaskGetTickCount();
  state->sd_next_flush_allowed_ticks = state->last_flush_ticks;
  TickType_t last_sd_detect_poll_ticks = 0;
  bool last_watermark_hit = false;

  while (!state->stop_requested) {
    const TickType_t now_ticks = xTaskGetTickCount();
    if (state->sd_card_detect.initialized &&
        (last_sd_detect_poll_ticks == 0 ||
         pdTICKS_TO_MS(now_ticks - last_sd_detect_poll_ticks) >=
           kSdDetectPollIntervalMs)) {
      last_sd_detect_poll_ticks = now_ticks;
      bool detect_changed = false;
      RuntimeMarkersSetSdFlush(state, SD_010_DETECT_POLL);
      const bool present =
        SdCardDetectPoll(&state->sd_card_detect, &detect_changed);
      UpdateCachedBool(state, &state->cached_status.sd_card_present, present);
      if (detect_changed) {
        if (present) {
          ESP_LOGI(kTag, "SD card inserted");
          UpdateCachedBool(state, &state->cached_status.sd_safe_to_remove, false);
        } else {
          ESP_LOGW(kTag, "SD card removed");
          if (RuntimeSdIoLock(state, kSdIoLockTimeoutTicks)) {
            RuntimeDiagHeapCheck(
              state, "SD unmount (card removed before)", false);
            (void)SdLoggerUnmount(&state->sd_logger);
            SdLoggerResetMountState(&state->sd_logger);
            RuntimeDiagHeapCheck(
              state, "SD unmount (card removed after)", false);
            RuntimeSdIoUnlock(state);
          }
          UpdateCachedBool(state,
                           &state->cached_status.sd_mounted,
                           state->sd_logger.is_mounted);
          UpdateCachedBool(state, &state->cached_status.sd_safe_to_remove, false);
          ClearSdIoError(state);
          state->sd_was_mounted = false;
          state->sd_backoff_until_ticks = 0;
          UpdateCachedUint32(
            state, &state->cached_status.sd_backoff_remaining_ms, 0u);
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
    if (!state->fram_available) {
      state->sd_flush_pending = false;
      state->sd_start_drain_pending = false;
      state->sd_flush_pending_trigger_flags = 0u;
      if (state->sd_manual_drain_active) {
        state->sd_manual_drain_active = false;
        state->sd_manual_drain_deadline_ticks = 0;
      }
    }
    const bool periodic_due =
      (pdTICKS_TO_MS(now_ticks - state->last_flush_ticks) >=
       state->settings.sd_flush_period_ms);
    uint32_t buffered = 0;
    if (state->fram_available &&
        RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      buffered = FramLogGetBufferedRecords(&state->fram_log);
      RuntimeFramLogUnlock(state);
    }
    const bool watermark_hit =
      buffered >= state->settings.fram_flush_watermark_records;

    if (periodic_due && state->fram_available) {
      state->sd_flush_pending = true;
      state->sd_flush_pending_trigger_flags |= SD_FLUSH_TRIGGER_PERIODIC;
      state->last_flush_ticks = now_ticks;
    }
    if (watermark_hit && state->fram_available) {
      state->sd_flush_pending = true;
      state->sd_flush_pending_trigger_flags |= SD_FLUSH_TRIGGER_WATERMARK;
      if (!last_watermark_hit) {
        const uint32_t watermark = state->settings.fram_flush_watermark_records;
        ESP_LOGI(kTag,
                 "Watermark hit (edge): buffered=%" PRIu32 " wm=%" PRIu32,
                 buffered,
                 watermark);
      }
    }
    last_watermark_hit = watermark_hit;
    if (state->sd_start_drain_pending) {
      state->sd_flush_pending_trigger_flags |= SD_FLUSH_TRIGGER_BACKLOG;
    }

    if (!state->sd_logger.is_mounted) {
      RuntimeMarkersSetSdFlush(state, SD_020_MAINTENANCE);
      SdMaintenanceTick(state);
    }

    if (state->sd_manual_drain_active) {
      uint32_t empty_checks = 0;
      uint32_t pass_count = 0;
      while (state->sd_manual_drain_active && !state->stop_requested) {
        const TickType_t now_ticks = xTaskGetTickCount();
        if (ManualDrainTimedOutTicks(now_ticks,
                                     state->sd_manual_drain_deadline_ticks)) {
          state->sd_manual_drain_active = false;
          state->sd_manual_drain_deadline_ticks = 0;
          break;
        }

        uint32_t remaining = 0;
        const bool remaining_ok = ReadFramBufferedRecords(state, &remaining);
        if (remaining_ok) {
          if (remaining == 0u) {
            empty_checks++;
          } else {
            empty_checks = 0;
          }
          if (empty_checks >= 2u) {
            state->sd_manual_drain_active = false;
            state->sd_manual_drain_deadline_ticks = 0;
            state->sd_flush_pending = false;
            break;
          }
          if (remaining == 0u) {
            taskYIELD();
            continue;
          }
        }

        state->sd_flush_pending_trigger_flags |= SD_FLUSH_TRIGGER_MANUAL;
        uint32_t flushed = 0;
        bool more_pending = false;
        const esp_err_t flush_result =
          SdFlushWorkerTick(state,
                            kSdFlushMaxRecordsPerPass,
                            kSdFlushMaxMsPerPass,
                            &flushed,
                            NULL,
                            &more_pending);
        state->sd_manual_drain_passes++;
        if (flush_result == ESP_OK && flushed > 0u) {
          state->sd_flush_records_since = 0;
        }

        taskYIELD();
        pass_count++;
        if ((pass_count % 4u) == 0u) {
          vTaskDelay(1);
        }
      }
    }

    const UBaseType_t queue_depth =
      (state->log_queue != NULL) ? uxQueueMessagesWaiting(state->log_queue) : 0;
    const bool queue_idle = (queue_depth <= 1u);
    const bool allow_flush_now =
      state->sd_start_drain_pending || (queue_depth == 0u);
    RuntimeMarkersSetSdFlush(state, SD_030_SCHEDULER);
    if (allow_flush_now && queue_idle && state->sd_flush_pending &&
        state->sd_logger.is_mounted && state->fram_available &&
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
          if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
            UpdateFramFillState(state);
            if (!state->sd_was_mounted && state->sd_logger.is_mounted) {
              ESP_LOGI(kTag,
                       "SD recovered; resuming flush. FRAM overruns since "
                       "boot: %" PRIu64,
                       FramLogGetOverrunRecordsTotal(&state->fram_log));
            }
            RuntimeFramLogUnlock(state);
          }
        }
        state->sd_flush_pending = more_pending;
        if (more_pending) {
          state->sd_flush_pending_trigger_flags |=
            SD_FLUSH_TRIGGER_MORE_PENDING;
        } else {
          state->sd_start_drain_pending = false;
        }
      } else {
        state->sd_flush_pending = true;
        state->sd_flush_pending_trigger_flags |= SD_FLUSH_TRIGGER_RETRY;
      }
      state->sd_next_flush_allowed_ticks =
        xTaskGetTickCount() + pdMS_TO_TICKS(kSdFlushMinIntervalMs);
    }

    if (!state->sd_logger.is_mounted) {
      state->sd_was_mounted = false;
    } else if (!state->sd_was_mounted) {
      state->sd_was_mounted = true;
    }

    RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(50));
  }

  state->sd_flush_task = NULL;
  vTaskDelete(NULL);
}

static void
ControlTickTimeSync(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }

  const bool time_valid = TimeSyncIsSystemTimeValid();
  UpdateTimeHealthState(state, time_valid);
  UpdateCachedBool(state,
                   &state->cached_status.ntp_fail_alert_active,
                   NetSupervisorIsSntpFailureAlertActive());
  const bool mesh_connected = MeshTransportIsConnected(&state->mesh);
  UpdateCachedBool(state, &state->cached_status.mesh_connected, mesh_connected);
  UpdateCachedInt32(
    state, &state->cached_status.mesh_level, state->mesh.last_level);

  if (state->settings.node_role == APP_NODE_ROLE_SENSOR) {
    if (!time_valid && mesh_connected) {
      (void)MeshTransportRequestTime(&state->mesh);
    }
    return;
  }

  if (state->settings.node_role == APP_NODE_ROLE_ROOT) {
    if (time_valid && mesh_connected) {
      const int64_t now_seconds = (int64_t)time(NULL);
      (void)MeshTransportBroadcastTime(&state->mesh, now_seconds);
    }
  }
}

static void
ControlTickRtcResync(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }

  const uint32_t period_ms = state->settings.rtc_resync_period_ms;
  if (period_ms == 0) {
    return;
  }

  if (state->i2c_quiesce_active) {
    return;
  }

  const TickType_t now_ticks = xTaskGetTickCount();
  if (state->rtc_resync_last_ticks != 0 &&
      pdTICKS_TO_MS(now_ticks - state->rtc_resync_last_ticks) < period_ms) {
    return;
  }
  state->rtc_resync_last_ticks = now_ticks;

  int64_t delta_seconds = 0;
  bool jumped_back = false;
  const int64_t system_epoch_before = (int64_t)time(NULL);
  const esp_err_t result = TimeSyncResyncSystemFromRtc(
    &state->time_sync, &delta_seconds, &jumped_back);
  if (result != ESP_OK) {
    const uint32_t warn_elapsed_ms =
      (state->last_rtc_resync_warn_ticks == 0)
        ? UINT32_MAX
        : (uint32_t)pdTICKS_TO_MS(now_ticks -
                                  state->last_rtc_resync_warn_ticks);
    if (warn_elapsed_ms >= 60000u) {
      ESP_LOGW(kTag, "RTC resync failed: %s", esp_err_to_name(result));
      state->last_rtc_resync_warn_ticks = now_ticks;
    }
    return;
  }

  if (jumped_back) {
    HandleRtcBackwardJump(state, delta_seconds, system_epoch_before);
  }
}

/**
 * @brief Execute DirectWifiTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the DirectWifiTask task.
 */
static void __attribute__((unused))
DirectWifiTask(void* context)
{
  runtime_state_t* state = (runtime_state_t*)context;
  bool last_connected = WifiManagerIsConnected();
  bool last_time_valid = TimeSyncIsSystemTimeValid();
  uint32_t retry_delay_ms = 30 * 1000;
  const uint32_t max_delay_ms = 5 * 60 * 1000;

  // Track minimum stack high-water mark to confirm stack sizing under real
  // workloads. uxTaskGetStackHighWaterMark() returns bytes in ESP-IDF.
  uint32_t min_stack_hwm_bytes = UINT32_MAX;

  while (!state->stop_requested) {
    const uint32_t hwm_bytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    if (hwm_bytes < min_stack_hwm_bytes) {
      min_stack_hwm_bytes = hwm_bytes;
      ESP_LOGI(kTag,
               "wifi_direct stack watermark: %u bytes free",
               (unsigned)min_stack_hwm_bytes);
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
    UpdateCachedBool(state,
                     &state->cached_status.ntp_fail_alert_active,
                     NetSupervisorIsSntpFailureAlertActive());

    if (connected && s_next_time_sync_ticks != 0 &&
        now_ticks >= s_next_time_sync_ticks) {
      const char* sntp_server = AppNetConfigGetSntpServer();
      esp_err_t sntp_result = ESP_ERR_INVALID_STATE;
      if (sntp_server != NULL && sntp_server[0] != '\0') {
        sntp_result = TimeSyncStartSntpAndWait(sntp_server, 30 * 1000);
        if (sntp_result == ESP_OK) {
          const esp_err_t rtc_result =
            TimeSyncSetRtcFromSystem(&state->time_sync);
          state->wifi_direct_time_synced = true;
          if (rtc_result == ESP_OK) {
            ESP_LOGI(kTag, "Time synchronized (SNTP -> RTC UTC)");
          } else {
            ESP_LOGW(kTag,
                     "Time synchronized (SNTP), but RTC update failed: %s",
                     esp_err_to_name(rtc_result));
          }
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
      UpdateCachedBool(state,
                       &state->cached_status.ntp_fail_alert_active,
                       NetSupervisorIsSntpFailureAlertActive());
      last_time_valid = time_valid;
    } else if (time_valid != last_time_valid) {
      last_time_valid = time_valid;
    }

    RuntimeInterruptibleDelayTicks(pdMS_TO_TICKS(retry_delay_ms));
  }

  state->wifi_direct_task = NULL;
  vTaskDelete(NULL);
}

static void
ControlTickTopology(runtime_state_t* state, int64_t now_ms)
{
  if (state == NULL) {
    return;
  }

  static int64_t s_last_disconnected_warn_ms = 0;
  static bool s_have_prev_status = false;
  static char s_prev_role[16] = { 0 };
  static bool s_prev_allow_children = false;
  static int s_prev_layer = -9999;
  static char s_prev_parent_str[20] = { 0 };
  static uint32_t s_prev_child_count = 0;
  static int s_prev_rssi = -9999;

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

  const bool connected_now = (layer > 0);
  if (connected_now) {
    s_last_disconnected_warn_ms = 0;
  } else if (MeshTransportIsStarted(&state->mesh)) {
    const bool should_warn =
      (s_last_disconnected_warn_ms == 0) ||
      ((now_ms - s_last_disconnected_warn_ms) >= (5 * 60 * 1000));
    if (should_warn) {
      ESP_LOGW(kTag,
               "Mesh not connected (layer=%d). Still scanning for AP/root...",
               layer);
      s_last_disconnected_warn_ms = now_ms;
    }
  }

  if (!state->log_quiet) {
    const bool allow_children = state->settings.allow_children;
    const bool changed =
      (!s_have_prev_status) ||
      (strncmp(s_prev_role, role, sizeof(s_prev_role)) != 0) ||
      (s_prev_allow_children != allow_children) || (s_prev_layer != layer) ||
      (strncmp(s_prev_parent_str, parent_str, sizeof(s_prev_parent_str)) !=
       0) ||
      (s_prev_child_count != child_count) || (s_prev_rssi != rssi);

    if (changed) {
      printf("topology role=%s allow_children=%u layer=%d parent=%s "
             "children=%u rssi=%d\n",
             role,
             allow_children ? 1u : 0u,
             layer,
             parent_str,
             (unsigned)child_count,
             rssi);

      strlcpy(s_prev_role, role, sizeof(s_prev_role));
      s_prev_allow_children = allow_children;
      s_prev_layer = layer;
      strlcpy(s_prev_parent_str, parent_str, sizeof(s_prev_parent_str));
      s_prev_child_count = child_count;
      s_prev_rssi = rssi;
      s_have_prev_status = true;
    }
  }
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

  if (!state->fram_available) {
    result = ESP_ERR_INVALID_STATE;
    goto drain_done;
  }

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
    UpdateCachedBool(state, &state->cached_status.sd_safe_to_remove, false);
  }

  const int32_t drain_records_per_pass = (max_records_per_pass > 0)
                                           ? max_records_per_pass
                                           : (int32_t)kSdFlushMaxRecordsPerPass;
  const int32_t yield_interval_records =
    (yield_every_records > 0) ? yield_every_records : drain_records_per_pass;

  int32_t records_since_yield = 0;
  TickType_t last_progress_log_ticks = 0;

  RuntimeDiagHeapCheck(state, "DrainFramToSd loop (before)", false);
  while (true) {
    uint32_t buffered = 0;
    if (!RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      result = ESP_ERR_TIMEOUT;
      break;
    }
    buffered = FramLogGetBufferedRecords(&state->fram_log);
    RuntimeFramLogUnlock(state);
    if (buffered == 0) {
      break;
    }
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
    if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      UpdateFramFillState(state);
      RuntimeFramLogUnlock(state);
    }
#if CONFIG_APP_DRAIN_LOG_PROGRESS
    if (flushed > 0) {
      uint32_t remaining = 0;
      if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
        remaining = FramLogGetBufferedRecords(&state->fram_log);
        RuntimeFramLogUnlock(state);
      }
      const TickType_t now_ticks = xTaskGetTickCount();
      const bool should_log =
        (last_progress_log_ticks == 0) ||
        (pdTICKS_TO_MS(now_ticks - last_progress_log_ticks) >= 1000u);
      if (should_log) {
        ESP_LOGI(kTag,
                 "Drain progress: flushed=%d remaining=%u",
                 flushed_records,
                 (unsigned)remaining);
        last_progress_log_ticks = now_ticks;
      }
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

  uint32_t remaining = 0;
  if (state->fram_available &&
      RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    remaining = (uint32_t)FramLogGetBufferedRecords(&state->fram_log);
    RuntimeFramLogUnlock(state);
  }
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
    esp_err_t unmount_result = ESP_ERR_INVALID_STATE;
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
      unmount_result = SdLoggerUnmount(&state->sd_logger);
      RuntimeDiagHeapCheck(state, "SD unmount (drain after)", false);
      state->sd_flush_in_progress = false;
      RuntimeSdIoUnlock(state);
    } else {
      unmount_result = ESP_ERR_TIMEOUT;
    }
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
    if (result == ESP_OK && unmount_result == ESP_OK &&
        !state->sd_logger.is_mounted) {
      UpdateCachedBool(state, &state->cached_status.sd_safe_to_remove, true);
      printf("SD: unmounted; safe to remove\n");
    } else {
      UpdateCachedBool(state, &state->cached_status.sd_safe_to_remove, false);
    }
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
    ESP_LOGW(
      kTag, "SD STOP INCOMPLETE: SD I/O lock timeout; skipping forced unmount");
  } else {
    state->sd_flush_in_progress = true;

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
  if (!state->fram_available) {
    stats->result = ESP_ERR_INVALID_STATE;
    UpdateStartDrainCachedStatus(state, stats);
    ESP_LOGW(kTag,
             "start drain: flushed=%d remaining=%d duration=%d ms result=%s",
             stats->flushed_records,
             stats->remaining_records,
             stats->duration_ms,
             esp_err_to_name(stats->result));
    return stats->result;
  }

#if !CONFIG_APP_START_DRAIN_ENABLE
  int32_t initial_remaining = 0;
  if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    initial_remaining = (int32_t)FramLogGetBufferedRecords(&state->fram_log);
    RuntimeFramLogUnlock(state);
  }
  EnsureSdMounted();
  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  stats->flushed_records = 0;
  stats->remaining_records = initial_remaining;
  stats->flushed_bytes = 0;
  stats->duration_ms = 0;
  stats->result = ESP_ERR_NOT_SUPPORTED;
  UpdateStartDrainCachedStatus(state, stats);
  if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    UpdateFramFillState(state);
    RuntimeFramLogUnlock(state);
  }
  ESP_LOGW(kTag,
           "start drain: flushed=%d remaining=%d duration=%d ms result=%s",
           stats->flushed_records,
           stats->remaining_records,
           stats->duration_ms,
           esp_err_to_name(stats->result));
  return stats->result;
#endif

  int32_t initial_remaining = 0;
  if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    initial_remaining = (int32_t)FramLogGetBufferedRecords(&state->fram_log);
    RuntimeFramLogUnlock(state);
  }

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
    if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      UpdateFramFillState(state);
      RuntimeFramLogUnlock(state);
    }
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
    if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      UpdateFramFillState(state);
      RuntimeFramLogUnlock(state);
    }
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

  if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
    UpdateFramFillState(state);
    RuntimeFramLogUnlock(state);
  }
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
  static int64_t next_topology_ms = 0;
  static int64_t next_time_sync_ms = 0;
  static int64_t next_alert_tick_ms = 0;
  static int64_t next_errlog_init_ms = 0;
  static uint32_t last_i2c_hang_log_ms = 0;
  static bool alert_boot_pending = true;
  static bool startup_mode_pending = true;
  static alert_system_code_t pending_mode_code = ALERT_SYSTEM_CODE_NONE;
  static runtime_phase_t last_phase = RUNTIME_PHASE_DIAGNOSTICS;
  uint32_t min_stack_hwm_bytes = UINT32_MAX;

  while (true) {
    const int64_t now_ms = esp_timer_get_time() / 1000;
    bool request_start = false;
    bool request_stop = false;
    taskENTER_CRITICAL(&state->request_lock);
    request_start = state->request_run_start;
    request_stop = state->request_run_stop;
    state->request_run_start = false;
    state->request_run_stop = false;
    taskEXIT_CRITICAL(&state->request_lock);

    const uint32_t i2c_held_ms = RuntimeI2cLockHeldMsNoLock(state);
    if (i2c_held_ms >= kI2cHangWarnMs) {
      if (LogRateLimitAllow(&last_i2c_hang_log_ms, kI2cLockDumpIntervalMs)) {
        RuntimeDumpI2cLockState(state, "i2c_hang_warn");
        RuntimeDumpI2cOpState(state, "i2c_hang_warn");
      }
      if (i2c_held_ms >= kI2cHangRestartMs) {
        ESP_LOGE(kTag, "HARD I2C HANG - restarting");
        RuntimeDumpI2cLockState(state, "i2c_hang_restart");
        RuntimeDumpI2cOpState(state, "i2c_hang_restart");
        const int64_t event_uptime_ms = esp_timer_get_time() / 1000;
        const int64_t event_epoch =
          TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
        int status = 0;
        int retry_after_seconds = -1;
        esp_err_t ntfy_err = ESP_OK;
        alert_ntfy_result_t ntfy_result =
          RuntimeAttemptPreRebootAlertSendDetailed(
            state,
            ALERT_SYSTEM_CODE_ERROR_I2C_HANG,
            event_epoch,
            event_uptime_ms,
            &retry_after_seconds,
            &status,
            &ntfy_err);
        vTaskDelay(pdMS_TO_TICKS(200));
        if (ntfy_result == ALERT_NTFY_OK) {
          RuntimeRebootAlertLatchClear();
        } else {
          RuntimeRebootAlertLatchSet(ALERT_SYSTEM_CODE_ERROR_I2C_HANG,
                                     event_epoch,
                                     (uint32_t)event_uptime_ms);
          const runtime_reboot_alert_send_result_t send_result =
            (ntfy_result == ALERT_NTFY_FAILED)
              ? RUNTIME_REBOOT_ALERT_SEND_FAIL
              : RUNTIME_REBOOT_ALERT_SEND_SKIPPED;
          const int64_t attempt_epoch =
            TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
          RuntimeRebootAlertLatchRecordAttempt(
            send_result,
            RUNTIME_REBOOT_ALERT_GATE_UNKNOWN,
            attempt_epoch,
            (uint32_t)event_uptime_ms,
            status,
            ntfy_err,
            retry_after_seconds);
        }
        int64_t err_epoch = 0;
        int32_t err_millis = 0;
        TimeSyncGetNow(&err_epoch, &err_millis);
        const uint32_t err_epoch_u32 =
          (err_epoch > 0) ? (uint32_t)err_epoch : 0u;
        const uint16_t err_millis_u16 =
          (err_millis >= 0) ? (uint16_t)err_millis : 0u;
        RuntimeRtcErrlogLatchPush(ERROR_I2C_HANG_RESTART,
                                  false,
                                  (int32_t)i2c_held_ms,
                                  (int32_t)state->i2c_mutex_timeouts,
                                  err_epoch_u32,
                                  err_millis_u16);
        esp_restart();
      }
    }

    gpio_button_event_t button_event = { 0 };
    while (GpioButtonsReceive(&button_event, 0)) {
      if (button_event.event_type != GPIO_BUTTON_EVENT_PRESS) {
        continue;
      }
      if (button_event.id == GPIO_BUTTON_RUN_START) {
        request_start = true;
        ESP_LOGI(kTag,
                 "button run start (uptime=%" PRIu32 " ms)",
                 button_event.uptime_ms);
      } else if (button_event.id == GPIO_BUTTON_RUN_STOP) {
        request_stop = true;
        RuntimeLatchOperatorHold(state, button_event.uptime_ms);
        ESP_LOGI(kTag,
                 "button run stop (uptime=%" PRIu32 " ms)",
                 button_event.uptime_ms);
      } else if (button_event.id == GPIO_BUTTON_UNITS_TOGGLE) {
        UnitsGpioHandleButtonPress();
        ESP_LOGI(kTag,
                 "button units toggle (uptime=%" PRIu32 " ms)",
                 button_event.uptime_ms);
      }
    }

    if (request_start) {
      state->pending_start = true;
    }
    if (request_stop) {
      state->pending_stop = true;
    }

    if (state->runtime_phase == RUNTIME_PHASE_RUNNING) {
      state->pending_start = false;
      if (state->pending_stop) {
        state->pending_stop = false;
        (void)EnterDiagMode();
      }
    }

    if (state->runtime_phase == RUNTIME_PHASE_DIAGNOSTICS) {
      state->pending_stop = false;
      if (state->pending_start && !RunGpioStopActive()) {
        state->pending_start = false;
        (void)EnterRunMode();
      }
    }

    RuntimeFramRetryTick(state, now_ms);

    if (!state->fram_error_log.initialized) {
      if (next_errlog_init_ms == 0 || now_ms >= next_errlog_init_ms) {
        (void)RuntimeMaybeInitFramErrorLog(state);
        next_errlog_init_ms = now_ms + 1000;
      }
    } else {
      next_errlog_init_ms = 0;
    }

    const runtime_phase_t current_phase = state->runtime_phase;
    if (current_phase != last_phase) {
      if (current_phase == RUNTIME_PHASE_RUNNING) {
        pending_mode_code = ALERT_SYSTEM_CODE_MODE_RUN;
        RuntimeResetStorageStallTracking(state, now_ms);
      } else if (current_phase == RUNTIME_PHASE_DIAGNOSTICS) {
        pending_mode_code = ALERT_SYSTEM_CODE_MODE_DIAG;
        RuntimeResetStorageStallTracking(state, now_ms);
      }
      last_phase = current_phase;
    }

    const bool alert_eligible = AlertManagerEligible(state);
    if (state->reboot_alert_pending) {
      runtime_reboot_alert_gate_reason_t gate_reason =
        RuntimeRebootAlertGateReason(state, now_ms);
      if (gate_reason == RUNTIME_REBOOT_ALERT_GATE_UNKNOWN) {
        int64_t event_epoch = state->reboot_alert_event_epoch;
        int64_t event_uptime_ms = state->reboot_alert_event_uptime_ms;
        if (event_uptime_ms <= 0) {
          event_uptime_ms = now_ms;
        }
        if (event_epoch <= 0 && TimeSyncIsSystemTimeValid()) {
          event_epoch = (int64_t)time(NULL);
        }
        if (RuntimeEnqueueSystemErrorNote(state,
                                          state->reboot_alert_code,
                                          false,
                                          event_epoch,
                                          event_uptime_ms)) {
          state->reboot_alert_pending = false;
          state->reboot_alert_active_sent = true;
          state->reboot_alert_active_sent_ms = now_ms;
          state->reboot_alert_next_check_ms =
            now_ms + (int64_t)kRebootAlertResolveDelayMs;
          RuntimeRebootAlertLatchClear();
        } else {
          RuntimeRebootAlertLatchRecordGate(
            RUNTIME_REBOOT_ALERT_GATE_QUEUE_FULL);
        }
      } else {
        RuntimeRebootAlertLatchRecordGate(gate_reason);
      }
    }
    if (state->reboot_alert_active_sent && alert_eligible &&
        WifiManagerIsConnected()) {
      if (state->reboot_alert_next_check_ms == 0 ||
          now_ms >= state->reboot_alert_next_check_ms) {
        bool stable = false;
        if (state->i2c_bus.initialized && state->fram_available &&
            RuntimeI2cLock(kI2cIoLockTimeoutTicks)) {
          stable = RuntimeVerifyFram(state);
          RuntimeI2cUnlock();
        }
        if (stable && (now_ms - state->reboot_alert_active_sent_ms) >=
                        (int64_t)kRebootAlertResolveDelayMs) {
          const int64_t resolve_epoch =
            TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
          if (RuntimeEnqueueSystemErrorNote(
                state, state->reboot_alert_code, true, resolve_epoch, now_ms)) {
            state->reboot_alert_active_sent = false;
          } else {
            state->reboot_alert_next_check_ms =
              now_ms + (int64_t)kRebootAlertResolveCheckIntervalMs;
          }
        } else {
          state->reboot_alert_next_check_ms =
            now_ms + (int64_t)kRebootAlertResolveCheckIntervalMs;
        }
      }
    }
    if (alert_eligible) {
      const int64_t now_epoch =
        TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
      const bool storage_stall_active =
        RuntimeComputeStorageStallCondition(state, now_ms);
      if (alert_boot_pending) {
        AlertManagerStartupNtfyBegin(&state->alert_manager, now_ms, now_epoch);
        alert_boot_pending = false;
      }
      if (pending_mode_code != ALERT_SYSTEM_CODE_NONE) {
        if (startup_mode_pending) {
          AlertManagerStartupNtfyUpdateMode(
            &state->alert_manager, pending_mode_code, now_ms, now_epoch);
          startup_mode_pending = false;
        } else {
          AlertManagerEmitSystemMode(
            &state->alert_manager, pending_mode_code, now_ms, now_epoch);
        }
        pending_mode_code = ALERT_SYSTEM_CODE_NONE;
      }
      AlertManagerStartupNtfyTick(&state->alert_manager, now_ms, now_epoch);
      AlertManagerProcessSystemError(&state->alert_manager,
                                     ALERT_SYSTEM_CODE_ERROR_SD_IO,
                                     state->cached_status.sd_io_error_active,
                                     now_ms,
                                     now_epoch);
      AlertManagerProcessSystemError(
        &state->alert_manager,
        ALERT_SYSTEM_CODE_ERROR_SD_OOS,
        state->cached_status.sd_out_of_space_active,
        now_ms,
        now_epoch);
      AlertManagerProcessSystemError(&state->alert_manager,
                                     ALERT_SYSTEM_CODE_ERROR_STORAGE_STALL,
                                     storage_stall_active,
                                     now_ms,
                                     now_epoch);
      AlertManagerProcessSystemError(&state->alert_manager,
                                     ALERT_SYSTEM_CODE_ERROR_FRAM_OVERRUN,
                                     state->cached_status.fram_overrun_active,
                                     now_ms,
                                     now_epoch);
      AlertManagerProcessSystemError(&state->alert_manager,
                                     ALERT_SYSTEM_CODE_ERROR_FRAM_IO,
                                     state->cached_status.fram_io_error_active,
                                     now_ms,
                                     now_epoch);
      AlertManagerProcessSystemError(&state->alert_manager,
                                     ALERT_SYSTEM_CODE_ERROR_I2C_RECOVERY,
                                     state->cached_status.i2c_recovery_active,
                                     now_ms,
                                     now_epoch);
      AlertManagerProcessSystemError(&state->alert_manager,
                                     ALERT_SYSTEM_CODE_ERROR_RTD_FAULT,
                                     state->cached_status.sensor_fault_present,
                                     now_ms,
                                     now_epoch);
      RuntimeMaybeSendRtdFaultSummaryNtfy(state, now_ms);
    } else {
      RuntimeResetStorageStallTracking(state, now_ms);
    }

    if (state->runtime_phase == RUNTIME_PHASE_RUNNING &&
        !state->stop_requested) {
      RuntimeHealthPublisherTick(state);

      if (next_topology_ms == 0 || now_ms >= next_topology_ms) {
        ControlTickTopology(state, now_ms);
        next_topology_ms = now_ms + (30 * 1000);
      }

      if (next_time_sync_ms == 0 || now_ms >= next_time_sync_ms) {
        ControlTickTimeSync(state);
        next_time_sync_ms = now_ms + 2000;
      }

      ControlTickRtcResync(state);

      if (alert_eligible &&
          (next_alert_tick_ms == 0 || now_ms >= next_alert_tick_ms)) {
        const int64_t now_epoch =
          TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
        const int64_t alert_now_ms = esp_timer_get_time() / 1000;
        AlertManagerTick(&state->alert_manager, alert_now_ms, now_epoch);
        next_alert_tick_ms = alert_now_ms + 1000;
      } else if (!alert_eligible) {
        next_alert_tick_ms = 0;
      }
    } else {
      next_topology_ms = 0;
      next_time_sync_ms = 0;
      next_alert_tick_ms = 0;
    }

    const uint32_t hwm_bytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    if (hwm_bytes < min_stack_hwm_bytes) {
      min_stack_hwm_bytes = hwm_bytes;
      ESP_LOGI(
        kTag, "control stack watermark: %u bytes free", (unsigned)hwm_bytes);
    }

    StackMonitorMaybeSample(&g_stack_monitor);
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
    uint32_t remaining = 0;
    if (RuntimeFramLogLock(state, kFramLogLockTimeoutTicks)) {
      remaining = (uint32_t)FramLogGetBufferedRecords(&state->fram_log);
      RuntimeFramLogUnlock(state);
    }
    ESP_LOGI(kTag, "flush complete; remaining=%u", (unsigned)remaining);
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
    ESP_LOGE(kTag, "spi_bus_initialize called twice for host %d", (int)host);
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
spi_host_device_t
RuntimeGetDisplaySpiHost(void)
{
#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
  return GetSpiHost();
#else
  return (CONFIG_APP_MAX7219_SPI_HOST == 3) ? SPI3_HOST : SPI2_HOST;
#endif
}

/**
 * @brief Execute GetDisplayMosiGpio.
 * @return Return the function result.
 */
int
RuntimeGetDisplayMosiGpio(void)
{
#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
  return CONFIG_APP_SPI_MOSI_GPIO;
#else
  return CONFIG_APP_MAX7219_MOSI_GPIO;
#endif
}

/**
 * @brief Execute GetDisplaySclkGpio.
 * @return Return the function result.
 */
int
RuntimeGetDisplaySclkGpio(void)
{
#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
  return CONFIG_APP_SPI_SCLK_GPIO;
#else
  return CONFIG_APP_MAX7219_SCLK_GPIO;
#endif
}

/**
 * @brief Execute GetDisplayCsGpio.
 * @return Return the function result.
 */
int
RuntimeGetDisplayCsGpio(void)
{
  return CONFIG_APP_MAX7219_CS_GPIO;
}

#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
static bool
DisplayShareConfigMatchesApp(void)
{
  return RuntimeGetDisplaySpiHost() == GetSpiHost() &&
         RuntimeGetDisplayMosiGpio() == CONFIG_APP_SPI_MOSI_GPIO &&
         RuntimeGetDisplaySclkGpio() == CONFIG_APP_SPI_SCLK_GPIO;
}
#endif // CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS

static int
SpiHostToId(spi_host_device_t host)
{
  return (host == SPI3_HOST) ? 3 : 2;
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
  state->sensor.spi_bus_lock = RuntimeSpiBusLock;
  state->sensor.spi_bus_unlock = RuntimeSpiBusUnlock;
  state->sensor.spi_bus_lock_context = state;
  state->sensor.spi_bus_lock_timeout_ticks = kSensorSpiBusLockTimeoutTicks;

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

#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
  if (!DisplayShareConfigMatchesApp()) {
    ESP_LOGE(kTag,
             "MAX7219 share enabled but pins/host differ "
             "(app host=%d mosi=%d sclk=%d, display host=%d mosi=%d sclk=%d); "
             "disable display or unshare SPI buses",
             SpiHostToId(GetSpiHost()),
             CONFIG_APP_SPI_MOSI_GPIO,
             CONFIG_APP_SPI_SCLK_GPIO,
             SpiHostToId(RuntimeGetDisplaySpiHost()),
             RuntimeGetDisplayMosiGpio(),
             RuntimeGetDisplaySclkGpio());
    return ESP_ERR_INVALID_STATE;
  }
#endif

  max7219_display_config_t display_config = {
    .host = RuntimeGetDisplaySpiHost(),
    .mosi_gpio = RuntimeGetDisplayMosiGpio(),
    .sclk_gpio = RuntimeGetDisplaySclkGpio(),
    .cs_gpio = RuntimeGetDisplayCsGpio(),
    .chain_len = CONFIG_APP_MAX7219_CHAIN_LEN,
    .clock_hz = 2000000,
    .intensity = CONFIG_APP_MAX7219_INTENSITY,
    .spi_bus_mutex =
#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
      state->spi_bus_mutex,
#else
      NULL,
#endif
    .spi_bus_mutex_timeout_ticks =
#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
      kSdIoLockTimeoutTicks,
#else
      0,
#endif
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
  g_state.errlog_latch_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  g_state.deferred_log.lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  MqttClientWrapInit(&g_state.mqtt_client);
  RuntimeHealthInit(&g_state.health_cache);
  RuntimeHealthPublisherInit(&g_state);
  HeapMonitorInit(&g_state.heap_monitor);
  g_state.cached_status.mesh_level = -1;
  g_state.fram_log_mutex_holder = kMutexHolderNone;
  g_state.i2c_mutex_holder = kMutexHolderNone;
  g_state.i2c_mutex_owner_task = NULL;
  g_state.sd_io_mutex_holder = kMutexHolderNone;
  g_state.sd_flush_phase = "idle";
  g_state.sd_flush_last_err = ESP_OK;
  g_state.i2c_quiesce_active = false;
  g_state.i2c_quiesce_refcount = 0u;
  g_state.i2c_quiesce_reason = NULL;
  g_state.data_stream_init_err = ESP_OK;

  g_runtime.settings = &g_state.settings;
  g_runtime.fram_i2c = &g_state.fram_i2c;
  g_runtime.fram_io = &g_state.fram_io;
  g_runtime.fram_log = &g_state.fram_log;
  g_runtime.fram_error_log = &g_state.fram_error_log;
  g_runtime.sd_logger = &g_state.sd_logger;
  g_runtime.sensor = &g_state.sensor;
  g_runtime.mesh = &g_state.mesh;
  g_runtime.time_sync = &g_state.time_sync;
  g_runtime.cal_due_check_suspended = &g_state.cal_due_check_suspended;
  g_runtime.cal_overdue = &g_state.cal_overdue;
  g_runtime.cal_time_stable = &g_state.cal_time_stable;
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
 * @brief Execute RuntimeGetEffectiveDisplayUnits.
 * @return Return the function result.
 */
app_display_units_t
RuntimeGetEffectiveDisplayUnits(void)
{
  return AppDisplayUnitsGetEffective();
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

bool
RuntimeRebootAlertLatchIsPending(void)
{
  if (!RuntimeRebootAlertLatchValid()) {
    return false;
  }
  RuntimeRebootAlertLatchUpgradeIfNeeded();
  return g_reboot_alert_latch.pending_is_active &&
         g_reboot_alert_latch.pending_system_code != 0;
}

void
RuntimeRebootAlertLatchClearSticky(void)
{
  RuntimeRebootAlertLatchClear();
}

void
RuntimeRebootAlertLatchCopy(runtime_reboot_alert_latch_t* out)
{
  if (out == NULL) {
    return;
  }
  if (!RuntimeRebootAlertLatchValid()) {
    memset(out, 0, sizeof(*out));
    return;
  }
  RuntimeRebootAlertLatchUpgradeIfNeeded();
  memcpy(out, &g_reboot_alert_latch, sizeof(*out));
}

uint16_t
RuntimeRtcErrlogLatchPendingCount(void)
{
  RuntimeRtcErrlogLatchInitIfNeeded();
  uint16_t count = 0;
  taskENTER_CRITICAL(&g_rtc_errlog_latch_lock);
  count = g_rtc_errlog_latch.count;
  taskEXIT_CRITICAL(&g_rtc_errlog_latch_lock);
  return count;
}

/**
 * @brief Execute RuntimeManagerInitMinimal.
 * @return Return the function result.
 */
esp_err_t
RuntimeManagerInitMinimal(void)
{
  MemGuardInit();
  InitializeRuntimeStruct();
  StackMonitorInit(&g_stack_monitor, 10000);
  esp_err_t first_error = ESP_OK;

  g_state.sd_io_mutex = xSemaphoreCreateMutexStatic(&g_state.sd_io_mutex_buf);
  if (g_state.sd_io_mutex == NULL) {
    ESP_LOGE(kTag, "Failed to create SD I/O mutex; SD marked degraded");
    g_state.sd_degraded = true;
    UpdateCachedBool(&g_state, &g_state.cached_status.sd_degraded, true);
  }
  g_state.spi_bus_mutex =
    xSemaphoreCreateMutexStatic(&g_state.spi_bus_mutex_buf);
  if (g_state.spi_bus_mutex == NULL) {
    ESP_LOGE(kTag, "Failed to create SPI bus mutex");
  }
  g_state.fram_log_mutex = xSemaphoreCreateMutex();
  if (g_state.fram_log_mutex == NULL) {
    ESP_LOGE(kTag, "Failed to create FRAM log mutex");
  }
  g_state.i2c_mutex = xSemaphoreCreateMutexStatic(&g_state.i2c_mutex_buf);
  if (g_state.i2c_mutex == NULL) {
    ESP_LOGE(kTag, "Failed to create I2C mutex");
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
  g_state.local_leaf_id = PackMacToLeafId(mac);
  FormatMacString(mac, g_state.node_id_string, sizeof(g_state.node_id_string));

  esp_err_t settings_result = AppSettingsLoad(&g_state.settings);
  if (settings_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = settings_result;
    }
    ESP_LOGE(
      kTag, "AppSettingsLoad failed: %s", esp_err_to_name(settings_result));
  }
  g_state.sd_verify_readback = g_state.settings.sd_verify_readback;
  g_state.last_rtd_ema_enabled = g_state.settings.rtd_ema_enabled;
  AppSettingsApplyTimeZone(&g_state.settings);
  UnitsGpioInit(&g_state.settings);

#if CONFIG_APP_WIFI_EARLY_RESERVE
  if (g_state.settings.net_mode != APP_NET_MODE_NONE) {
    HeapLogPhase("wifi_reserve_before");
    esp_err_t reserve_result = WifiServiceReserveEarly();
    if (reserve_result == ESP_OK) {
      ESP_LOGI(kTag, "wifi early reserve ok");
    } else {
      ESP_LOGW(
        kTag, "wifi early reserve failed: %s", esp_err_to_name(reserve_result));
    }
    HeapLogPhase("wifi_reserve_after");
  }
#endif

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
  UpdateCachedBool(&g_state, &g_state.cached_status.system_running, false);
  UpdateCachedBool(&g_state, &g_state.cached_status.data_stream_enabled, false);
  UpdateCachedUint32(&g_state,
                     &g_state.cached_status.data_stream_backend,
                     (uint32_t)DataPortGetBackend());
  UpdateCachedInt32(&g_state,
                    &g_state.cached_status.data_stream_init_err,
                    (int32_t)g_state.data_stream_init_err);
  g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
  g_state.pending_start = false;
  g_state.pending_stop = false;
  RuntimeResetStorageStallTracking(&g_state, 0);
  SdCardDetectInit(&g_state.sd_card_detect);
  const bool sd_card_present = SdCardDetectPoll(&g_state.sd_card_detect, NULL);
  UpdateCachedBool(
    &g_state, &g_state.cached_status.sd_card_present, sd_card_present);
  RuntimeHealthPublisherTick(&g_state);

  AlertManagerInit(
    &g_state.alert_manager, g_state.node_id_string, g_state.local_leaf_id);
  (void)AlertManagerLoadConfig(&g_state.alert_manager);
  RuntimeLoadRebootAlertLatch(&g_state);

  g_state.initialized = true;
  g_state.full_initialized = false;
  return first_error;
}

/**
 * @brief Execute RuntimeManagerInitFull.
 * @return Return the function result.
 */
esp_err_t
RuntimeManagerInitFull(void)
{
  esp_err_t first_error = ESP_OK;
  if (!g_state.initialized) {
    esp_err_t minimal_result = RuntimeManagerInitMinimal();
    if (minimal_result != ESP_OK && first_error == ESP_OK) {
      first_error = minimal_result;
    }
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

  ESP_LOGI(kTag,
           "APP SPI: host=%d mosi=%d miso=%d sclk=%d",
           CONFIG_APP_SPI_HOST,
           CONFIG_APP_SPI_MOSI_GPIO,
           CONFIG_APP_SPI_MISO_GPIO,
           CONFIG_APP_SPI_SCLK_GPIO);
  ESP_LOGI(kTag,
           "DISP SPI: host=%d mosi=%d sclk=%d cs=%d (share=%d)",
           SpiHostToId(RuntimeGetDisplaySpiHost()),
           RuntimeGetDisplayMosiGpio(),
           RuntimeGetDisplaySclkGpio(),
           RuntimeGetDisplayCsGpio(),
#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
           1
#else
           0
#endif
  );

#if CONFIG_APP_MAX7219_ENABLE
  esp_err_t display_result = InitializeMax7219Display(&g_state);
  if (display_result == ESP_OK && g_state.display_initialized) {
    const uint32_t kDisplayTaskStackBytes = 4608;
    BaseType_t display_created = xTaskCreate(&DisplayTask,
                                             "display",
                                             kDisplayTaskStackBytes,
                                             &g_state,
                                             2,
                                             &g_state.display_task);
    if (display_created != pdPASS) {
      g_state.display_initialized = false;
      g_state.display_task = NULL;
      ESP_LOGW(kTag, "Display task create failed");
    } else {
      RegisterStackMonitorTask(
        "display", &g_state.display_task, kDisplayTaskStackBytes);
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
    .max_freq_khz = CONFIG_APP_SD_SPI_MAX_FREQ_KHZ,
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
    UpdateCachedBool(&g_state,
                     &g_state.cached_status.ntp_fail_alert_active,
                     NetSupervisorIsSntpFailureAlertActive());
  }

  g_state.fram_available = false;
  esp_err_t fram_i2c_result = ESP_ERR_INVALID_STATE;
  bool i2c_lines_ok = false;
  if (g_state.i2c_bus.initialized) {
    i2c_lines_ok = I2cBusLinesLookIdle(&g_state.i2c_bus);
    if (!i2c_lines_ok) {
      (void)I2cBusRecoverLines(g_state.i2c_bus.sda_gpio,
                               g_state.i2c_bus.scl_gpio);
      i2c_lines_ok = I2cBusLinesLookIdle(&g_state.i2c_bus);
    }
  }
  if (!g_state.i2c_bus.initialized) {
    fram_i2c_result = ESP_ERR_INVALID_STATE;
  } else if (!i2c_lines_ok) {
    fram_i2c_result = ESP_ERR_TIMEOUT;
    ESP_LOGW(kTag, "FRAM init skipped: I2C lines not idle");
  } else {
    fram_i2c_result = FramI2cInit(&g_state.fram_i2c,
                                  g_state.i2c_bus.handle,
                                  &g_state.i2c_bus,
                                  (uint8_t)CONFIG_APP_FRAM_I2C_ADDR,
                                  CONFIG_APP_FRAM_SIZE_BYTES,
                                  g_state.i2c_bus.frequency_hz);
  }
  const int64_t fram_init_now_ms = esp_timer_get_time() / 1000;
  if (fram_i2c_result != ESP_OK) {
    if (first_error == ESP_OK) {
      first_error = fram_i2c_result;
    }
    ESP_LOGE(kTag, "FramI2cInit failed: %s", esp_err_to_name(fram_i2c_result));
    RuntimeSetFramUnavailable(
      &g_state, "init", fram_i2c_result, fram_init_now_ms);
  } else {
    esp_err_t fram_log_result =
      FramLogInit(&g_state.fram_log, g_state.fram_io, FRAM_DATA_BYTES);
    if (fram_log_result != ESP_OK) {
      if (first_error == ESP_OK) {
        first_error = fram_log_result;
      }
      ESP_LOGE(
        kTag, "FramLogInit failed: %s", esp_err_to_name(fram_log_result));
      RuntimeSetFramUnavailable(
        &g_state, "log init", fram_log_result, fram_init_now_ms);
    } else {
      g_state.fram_available = true;
      g_state.fram_next_retry_ms = 0;
      RuntimeSyncFramFallbackCounters(&g_state);
      UpdateCachedBool(
        &g_state, &g_state.cached_status.fram_io_error_active, false);
      esp_err_t fram_errlog_result = FramErrorLogInit(&g_state.fram_error_log,
                                                      g_state.fram_io,
                                                      FRAM_ERRLOG_BASE,
                                                      FRAM_ERRLOG_BYTES);
      if (fram_errlog_result != ESP_OK) {
        if (first_error == ESP_OK) {
          first_error = fram_errlog_result;
        }
        ESP_LOGE(kTag,
                 "FramErrorLogInit failed: %s",
                 esp_err_to_name(fram_errlog_result));
      } else {
        fram_error_log_stats_t stats = { 0 };
        if (FramErrorLogGetStats(&g_state.fram_error_log, &stats) == ESP_OK &&
            stats.count > 0) {
          (void)FramErrorLogDump(&g_state.fram_error_log, 0);
        }
      }
    }
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
  if (g_state.sd_logger.is_mounted) {
    UpdateCachedBool(&g_state, &g_state.cached_status.sd_safe_to_remove, false);
  }
  if (g_state.fram_available) {
    RestoreTimeJumpBackPendingFromFram(&g_state);
  }

  esp_err_t sensor_result = InitializeMax31865Sensor(&g_state, spi_host);
  if (sensor_result != ESP_OK && first_error == ESP_OK) {
    first_error = sensor_result;
  }

  g_state.log_queue = xQueueCreateStatic(kLogQueueDepth,
                                         sizeof(sensor_sample_msg_t),
                                         g_state.log_queue_storage,
                                         &g_state.log_queue_struct);
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

  bool export_pool_ready = false;
  if (g_export_outbox_pool_storage == NULL) {
    const size_t export_pool_bytes =
      MemPoolGetRequiredBytes(sizeof(export_record_item_t), kExportOutboxDepth);
    g_export_outbox_pool_storage = (uint8_t*)AppMalloc(export_pool_bytes);
  }
  if (g_export_outbox_pool_storage != NULL &&
      MemPoolInit(&g_export_outbox_pool,
                  g_export_outbox_pool_storage,
                  sizeof(export_record_item_t),
                  kExportOutboxDepth)) {
    export_pool_ready = true;
  } else {
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to initialize export outbox pool");
  }

  if (export_pool_ready) {
    if (g_export_outbox_queue_storage == NULL) {
      g_export_outbox_queue_storage = (uint8_t*)AllocatePreferPsram(
        kExportOutboxDepth * sizeof(export_record_item_t*));
    }
    g_state.export_outbox = xQueueCreateStatic(kExportOutboxDepth,
                                               sizeof(export_record_item_t*),
                                               g_export_outbox_queue_storage,
                                               &g_export_outbox_queue_struct);
    if (g_state.export_outbox == NULL) {
      if (first_error == ESP_OK) {
        first_error = ESP_ERR_NO_MEM;
      }
      ESP_LOGE(kTag, "Failed to create export outbox");
    }
  }

  bool broker_pool_ready = false;
  if (g_broker_outbox_pool_storage == NULL) {
    const size_t broker_pool_bytes = MemPoolGetRequiredBytes(
      sizeof(broker_publish_item_t), kBrokerOutboxDepth);
    g_broker_outbox_pool_storage = (uint8_t*)AppMalloc(broker_pool_bytes);
  }
  if (g_broker_outbox_pool_storage != NULL &&
      MemPoolInit(&g_broker_outbox_pool,
                  g_broker_outbox_pool_storage,
                  sizeof(broker_publish_item_t),
                  kBrokerOutboxDepth)) {
    broker_pool_ready = true;
  } else {
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to initialize broker outbox pool");
  }

  if (broker_pool_ready) {
    if (g_broker_outbox_queue_storage == NULL) {
      g_broker_outbox_queue_storage = (uint8_t*)AllocatePreferPsram(
        kBrokerOutboxDepth * sizeof(broker_publish_item_t*));
    }
    g_state.broker_outbox = xQueueCreateStatic(kBrokerOutboxDepth,
                                               sizeof(broker_publish_item_t*),
                                               g_broker_outbox_queue_storage,
                                               &g_broker_outbox_queue_struct);
    if (g_state.broker_outbox == NULL) {
      if (first_error == ESP_OK) {
        first_error = ESP_ERR_NO_MEM;
      }
      ESP_LOGE(kTag, "Failed to create broker outbox");
    }
  }

  const uint32_t kControlTaskStackBytes = 8192;
  BaseType_t control_created = xTaskCreate(&ControlTask,
                                           "control",
                                           kControlTaskStackBytes,
                                           &g_state,
                                           3,
                                           &g_state.control_task);
  if (control_created != pdPASS) {
    g_state.control_task = NULL;
    if (first_error == ESP_OK) {
      first_error = ESP_ERR_NO_MEM;
    }
    ESP_LOGE(kTag, "Failed to create control task");
  } else {
    RegisterStackMonitorTask(
      "control", &g_state.control_task, kControlTaskStackBytes);
  }

  g_state.system_running = true;
  UpdateCachedBool(&g_state, &g_state.cached_status.system_running, true);
  g_state.full_initialized = true;
  g_state.initialized = true;
  return first_error;
}

/**
 * @brief Execute RuntimeManagerInit.
 * @return Return the function result.
 */
esp_err_t
RuntimeManagerInit(void)
{
  const esp_err_t minimal_result = RuntimeManagerInitMinimal();
  const esp_err_t full_result = RuntimeManagerInitFull();
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  const char* console_backend = "usb_jtag";
#else
  const char* console_backend = "uart";
#endif
  ESP_LOGI(kTag,
           "Stream routing: console=%s data=%s",
           console_backend,
           DataPortBackendToString(DataPortGetBackend()));
  if (minimal_result != ESP_OK) {
    return minimal_result;
  }
  return full_result;
}

/**
 * @brief Execute EnsureSdMountedLocked.
 * @param state Parameter state.
 */
static void
EnsureSdMountedLocked(runtime_state_t* state)
{
  if (state == NULL || state->sd_logger.is_mounted) {
    return;
  }
  if (!SdCardPresent(state)) {
    UpdateCachedBool(
      state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
    return;
  }
  SdLoggerResetMountState(&state->sd_logger);
  RuntimeDiagHeapCheck(state, "SD mount (ensure before)", false);
  esp_err_t mount_result =
    SdLoggerMount(&state->sd_logger, GetSpiHost(), CONFIG_APP_SD_CS_GPIO);
  RuntimeDiagHeapCheck(state, "SD mount (ensure after)", false);
  if (mount_result != ESP_OK) {
    MarkSdFailure(state, "SD mount failed", "mount", mount_result, 0, false);
  } else {
    ClearSdIoError(state);
    if (state->sd_logger.is_mounted) {
      UpdateCachedBool(state, &state->cached_status.sd_safe_to_remove, false);
    }
  }
  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
}

/**
 * @brief Execute EnsureSdMounted.
 */
static void
EnsureSdMounted(void)
{
  if (!RuntimeSdIoLock(&g_state, kSdIoLockTimeoutTicks)) {
    return;
  }
  EnsureSdMountedLocked(&g_state);
  RuntimeSdIoUnlock(&g_state);
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
    UpdateCachedBool(state, &state->cached_status.sd_safe_to_remove, false);
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
  if (!g_state.initialized || !g_state.full_initialized) {
    g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
    return ESP_ERR_INVALID_STATE;
  }
  if (g_state.runtime_phase == RUNTIME_PHASE_STOPPING) {
    g_state.pending_start = true;
    return ESP_ERR_INVALID_STATE;
  }
  g_state.runtime_phase = RUNTIME_PHASE_STARTING;
  if (g_state.logger_running) {
    g_state.runtime_phase = RUNTIME_PHASE_RUNNING;
    return ESP_OK;
  }
  if (g_state.sensor_task != NULL || g_state.storage_task != NULL ||
      g_state.net_tx_task != NULL || g_state.alert_http_task != NULL ||
      g_state.wifi_direct_task != NULL) {
    if (g_state.sensor_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: sensor_task still alive");
    }
    if (g_state.storage_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: storage_task still alive");
    }
    if (g_state.net_tx_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: net_tx_task still alive");
    }
    if (g_state.alert_http_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: alert_http_task still alive");
    }
    if (g_state.wifi_direct_task != NULL) {
      ESP_LOGW(kTag, "Start blocked: wifi_direct_task still alive");
    }
    g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
    return ESP_ERR_INVALID_STATE;
  }
  if (g_state.log_queue == NULL) {
    g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
    return ESP_ERR_NO_MEM;
  }
  if (g_state.export_queue == NULL) {
    g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
    return ESP_ERR_NO_MEM;
  }
  if (g_state.batch_buffer == NULL || g_state.batch_buffer_size == 0) {
    g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
    return ESP_ERR_NO_MEM;
  }

  if (!g_state.sensor.is_initialized) {
    esp_err_t sensor_result = InitializeMax31865Sensor(&g_state, GetSpiHost());
    if (sensor_result != ESP_OK) {
      g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
      return sensor_result;
    }
  }

#if CONFIG_APP_MAX7219_ENABLE
  if (!g_state.display_initialized) {
    (void)InitializeMax7219Display(&g_state);
  }
#endif

  g_state.stop_requested = false;
  g_state.net_tx_pause_requested = false;
  g_state.net_tx_paused = false;
  g_state.alert_http_pause_requested = false;
  g_state.alert_http_paused = false;
  UpdateCachedBool(&g_state, &g_state.cached_status.stop_requested, false);
  UpdateCachedBool(&g_state, &g_state.cached_status.sd_safe_to_remove, false);
  g_state.fram_full = false;
  g_state.sd_degraded = false;
  g_state.sd_fail_count = 0;
  g_state.sd_backoff_until_ticks = 0;
  g_state.last_sd_flush_warn_ticks = 0;
  g_state.last_sd_flush_wait_warn_ticks = 0;
  g_state.sd_flush_records_since = 0;
  g_state.sd_flush_in_progress = false;
  g_state.sd_flush_pending = false;
  g_state.sd_flush_quiesce_session_active = false;
  g_state.sd_flush_pending_trigger_flags = 0u;
  g_state.sd_flush_trigger_flags = 0u;
  g_state.sd_flush_session_id = 0u;
  g_state.sd_flush_session_records_flushed = 0u;
  g_state.sd_flush_session_start_ticks = 0;
  g_state.sd_flush_session_label[0] = '\0';
  g_state.sd_start_drain_pending = false;
  g_state.sd_next_flush_allowed_ticks = 0;
  g_state.i2c_quiesce_active = false;
  g_state.deferred_drop_last_log_ms = 0;
  DeferredLogReset(&g_state);
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
  UpdateCachedBool(&g_state, &g_state.cached_status.i2c_quiesce_active, false);
  UpdateCachedBool(&g_state, &g_state.cached_status.data_stream_enabled, false);
  UpdateCachedUint32(&g_state,
                     &g_state.cached_status.data_stream_backend,
                     (uint32_t)DataPortGetBackend());
  UpdateCachedInt32(
    &g_state, &g_state.cached_status.data_stream_init_err, (int32_t)ESP_OK);

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
    DrainExportOutboxQueue(&g_state);
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

  RuntimeEnableDataStreaming(true);

  esp_err_t net_apply_result = RuntimeApplyNetMode(g_state.settings.net_mode);
  if (net_apply_result != ESP_OK && effective_net_mode == APP_NET_MODE_MESH) {
    ESP_LOGE(kTag, "Mesh start failed: %s", esp_err_to_name(net_apply_result));
  }

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
        RuntimeDiagHeapCheck(
          &g_state, "SD unmount (initial sync before)", false);
        (void)SdLoggerUnmount(&g_state.sd_logger);
        RuntimeDiagHeapCheck(
          &g_state, "SD unmount (initial sync after)", false);
        MarkSdFailure(
          &g_state, "Initial SD sync failed", "sync", sync_result, 0, true);
      }
      RuntimeSdIoUnlock(&g_state);
    } else {
      ESP_LOGE(kTag, "Initial SD sync skipped due to I/O lock timeout");
    }
  }

  g_state.logger_running = true;
  UpdateCachedBool(&g_state, &g_state.cached_status.runtime_running, true);
  RuntimeResetStorageStallTracking(&g_state, esp_timer_get_time() / 1000);

  BaseType_t sensor_created = pdPASS;
  BaseType_t storage_created = pdPASS;
  BaseType_t sd_flush_created = pdPASS;
  BaseType_t export_created = pdPASS;
  BaseType_t net_tx_created = pdPASS;
  BaseType_t alert_http_created = pdPASS;
  BaseType_t wifi_direct_created = pdPASS;

  const uint32_t kSensorStackBytes = 4608;
  const uint32_t kExportStackBytes = 3584;
  const uint32_t kNetTxTaskStackBytes = 8192;

  if (effective_net_mode == APP_NET_MODE_DIRECT_WIFI) {
    ESP_LOGI(kTag, "Wi-Fi direct handled by net supervisor");
  }

  if (role == APP_NODE_ROLE_SENSOR) {
    sensor_created = xTaskCreate(&SensorTask,
                                 "sensor",
                                 kSensorStackBytes,
                                 &g_state,
                                 5,
                                 &g_state.sensor_task);
    if (sensor_created != pdPASS) {
      g_state.sensor_task = NULL;
      ESP_LOGE(kTag, "Failed to create task sensor");
    } else {
      RegisterStackMonitorTask(
        "sensor", &g_state.sensor_task, kSensorStackBytes);
    }
    g_state.storage_task =
      xTaskCreateStatic(&StorageTask,
                        "storage",
                        kStorageTaskStackBytes / sizeof(StackType_t),
                        &g_state,
                        6,
                        g_state.storage_task_stack,
                        &g_state.storage_task_tcb);
    storage_created = (g_state.storage_task != NULL) ? pdPASS : pdFAIL;
    if (storage_created != pdPASS) {
      g_state.storage_task = NULL;
      ESP_LOGE(kTag, "Failed to create task storage");
    } else {
      RegisterStackMonitorTask(
        "storage", &g_state.storage_task, kStorageTaskStackBytes);
    }
    g_state.sd_flush_task =
      xTaskCreateStatic(&SdFlushTask,
                        "sd_flush",
                        kSdFlushTaskStackBytes / sizeof(StackType_t),
                        &g_state,
                        5,
                        g_state.sd_flush_task_stack,
                        &g_state.sd_flush_task_tcb);
    sd_flush_created = (g_state.sd_flush_task != NULL) ? pdPASS : pdFAIL;
    if (sd_flush_created != pdPASS) {
      g_state.sd_flush_task = NULL;
      ESP_LOGE(kTag, "Failed to create task sd_flush");
    } else {
      RegisterStackMonitorTask(
        "sd_flush", &g_state.sd_flush_task, kSdFlushTaskStackBytes);
    }
  }

  if (role == APP_NODE_ROLE_SENSOR || role == APP_NODE_ROLE_ROOT) {
    export_created = xTaskCreate(&ExportTask,
                                 "export",
                                 kExportStackBytes,
                                 &g_state,
                                 4,
                                 &g_state.export_task);
    if (export_created != pdPASS) {
      g_state.export_task = NULL;
      ESP_LOGE(kTag, "Failed to create task export");
    } else {
      RegisterStackMonitorTask(
        "export", &g_state.export_task, kExportStackBytes);
    }
  }

  if (role == APP_NODE_ROLE_SENSOR || role == APP_NODE_ROLE_ROOT) {
    net_tx_created = xTaskCreate(&NetTxTask,
                                 "net_tx",
                                 kNetTxTaskStackBytes,
                                 &g_state,
                                 3,
                                 &g_state.net_tx_task);
    if (net_tx_created != pdPASS) {
      g_state.net_tx_task = NULL;
      ESP_LOGE(kTag, "Failed to create task net_tx");
    } else {
      RegisterStackMonitorTask(
        "net_tx", &g_state.net_tx_task, kNetTxTaskStackBytes);
    }
  }

  if (role == APP_NODE_ROLE_SENSOR || role == APP_NODE_ROLE_ROOT) {
    alert_http_created =
      CreateAlertHttpTaskWithPsrStack(&g_state, kAlertHttpTaskStackBytes);
    if (alert_http_created != pdPASS) {
      g_state.alert_http_task = NULL;
      if (!g_alert_http_psram_failure_logged) {
        ESP_LOGW(kTag, "alert_http task deferred (PSRAM alloc failed)");
        g_alert_http_psram_failure_logged = true;
      }
    } else {
      RegisterStackMonitorTask(
        "alert_http", &g_state.alert_http_task, kAlertHttpTaskStackBytes);
    }
  }

  if (role == APP_NODE_ROLE_ROOT) {
    AlertManagerEmitRootRestart(&g_state.alert_manager,
                                esp_timer_get_time() / 1000);
  }

  if (sensor_created != pdPASS || storage_created != pdPASS ||
      sd_flush_created != pdPASS || export_created != pdPASS ||
      net_tx_created != pdPASS || wifi_direct_created != pdPASS) {
    g_state.stop_requested = true;
    g_state.logger_running = false;
    UpdateCachedBool(&g_state, &g_state.cached_status.stop_requested, true);
    UpdateCachedBool(&g_state, &g_state.cached_status.runtime_running, false);
    const TickType_t wait_start = xTaskGetTickCount();
    while ((g_state.sensor_task != NULL || g_state.storage_task != NULL ||
            g_state.sd_flush_task != NULL || g_state.export_task != NULL ||
            g_state.net_tx_task != NULL || g_state.alert_http_task != NULL ||
            g_state.wifi_direct_task != NULL) &&
           (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < 1000)) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (g_state.sensor_task == NULL && g_state.storage_task == NULL &&
        g_state.sd_flush_task == NULL && g_state.export_task == NULL &&
        g_state.net_tx_task == NULL && g_state.alert_http_task == NULL &&
        g_state.wifi_direct_task == NULL) {
      g_state.stop_requested = false;
      UpdateCachedBool(&g_state, &g_state.cached_status.stop_requested, false);
    }
    g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
    return ESP_ERR_NO_MEM;
  }

  const int64_t now_ms = esp_timer_get_time() / 1000;
  const int64_t now_epoch =
    TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
  AlertManagerOnLoggingSessionStart(&g_state.alert_manager, now_ms, now_epoch);

  ESP_LOGI(kTag,
           "Runtime started (node=%s role=%s)",
           g_state.node_id_string,
           AppSettingsRoleToString(role));
  g_state.runtime_phase = RUNTIME_PHASE_RUNNING;
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
  if (state == NULL || !state->logger_running) {
    return ESP_OK;
  }

  state->csv_header_emitted = false;
  state->root_bridge_header_emitted = false;
  state->stop_requested = true;
  RuntimeNotifyAllRunTasks(state);
  state->logger_running = false;
  UpdateCachedBool(state, &state->cached_status.stop_requested, true);
  UpdateCachedBool(state, &state->cached_status.runtime_running, false);
  RuntimeResetStorageStallTracking(state, esp_timer_get_time() / 1000);

  const TickType_t wait_start = xTaskGetTickCount();
  while ((state->sensor_task != NULL || state->storage_task != NULL ||
          state->sd_flush_task != NULL || state->export_task != NULL ||
          state->net_tx_task != NULL || state->alert_http_task != NULL ||
          state->wifi_direct_task != NULL) &&
         (pdTICKS_TO_MS(xTaskGetTickCount() - wait_start) < 15000)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (state->sensor_task != NULL || state->storage_task != NULL ||
      state->sd_flush_task != NULL || state->export_task != NULL ||
      state->net_tx_task != NULL || state->alert_http_task != NULL ||
      state->wifi_direct_task != NULL) {
    if (state->sensor_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: sensor_task still running (%p)",
               state->sensor_task);
    }
    if (state->storage_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: storage_task still running (%p)",
               state->storage_task);
    }
    if (state->sd_flush_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: sd_flush_task still running (%p)",
               state->sd_flush_task);
    }
    if (state->export_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: export_task still running (%p)",
               state->export_task);
    }
    if (state->net_tx_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: net_tx_task still running (%p)",
               state->net_tx_task);
    }
    if (state->alert_http_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: alert_http_task still running (%p)",
               state->alert_http_task);
    }
    if (state->wifi_direct_task != NULL) {
      ESP_LOGW(kTag,
               "Stop timeout: wifi_direct_task still running (%p)",
               state->wifi_direct_task);
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
    RuntimeStopMeshTransport(state);
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
    DrainExportOutboxQueue(state);
    (void)xQueueReset(state->export_outbox);
  }
  if (state->broker_outbox != NULL) {
    DrainBrokerOutboxQueue(state);
    (void)xQueueReset(state->broker_outbox);
  }
  MqttClientWrapStop(&state->mqtt_client);
  UpdateMqttConnectionState(state);
  state->stop_requested = false;
  state->net_tx_pause_requested = false;
  state->net_tx_paused = false;
  state->alert_http_pause_requested = false;
  state->alert_http_paused = false;
  state->spi_pause_requested = false;
  state->spi_pause_ack_mask = 0;
  UpdateCachedBool(state, &state->cached_status.stop_requested, false);

  // Leave the MAX7219 display initialized during stop; only deinit on shutdown.

  // Keep MAX31865 initialized in diagnostics mode so calibration commands can
  // read the RTD when logging is stopped. Full deinit is only done on reboot or
  // an explicit shutdown path (if added later).
  return ESP_OK;
}

/**
 * @brief Execute RuntimeStop.
 * @return Return the function result.
 */
esp_err_t
RuntimeStop(void)
{
  g_state.runtime_phase = RUNTIME_PHASE_STOPPING;
  esp_err_t stop_result = RuntimeStopSamplingOnly(&g_state);
  esp_err_t finalize_result = RuntimeStopAllTasks(&g_state);
  if (!heap_caps_check_integrity_all(true)) {
    ESP_LOGE(kTag, "Heap corruption detected after RuntimeStop");
    abort();
  }
  g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
  return (stop_result != ESP_OK) ? stop_result : finalize_result;
}

/**
 * @brief Execute RuntimeIsRunning.
 * @return Return the function result.
 */
bool
RuntimeIsRunning(void)
{
  return g_state.logger_running;
}

bool
RuntimeCopyLatestSensorSample(runtime_sensor_sample_t* out_sample)
{
  if (out_sample == NULL) {
    return false;
  }

  taskENTER_CRITICAL(&g_runtime_sensor_sample_lock);
  *out_sample = g_runtime_sensor_sample;
  taskEXIT_CRITICAL(&g_runtime_sensor_sample_lock);
  return out_sample->sample_id > 0u;
}

/**
 * @brief Execute EnterRunMode.
 * @return Return the function result.
 */
esp_err_t
EnterRunMode(void)
{
  RuntimeClearOperatorHold(&g_state);
  RuntimeSetLogPolicyRun();

  esp_err_t result = RuntimeStart();
  if (result != ESP_OK) {
    RuntimeSetLogPolicyDiag();
  } else {
    MemGuardSetPhase(MEM_GUARD_PHASE_RUN);
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
  const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
  RuntimeLatchOperatorHold(&g_state, now_ms);
  g_state.runtime_phase = RUNTIME_PHASE_STOPPING;
  RuntimeSetLogPolicyDiag();
  RuntimeEnableDataStreaming(false);
  ESP_LOGW(kTag, "Stop requested (pre-flush)");
  uint32_t drained_notes = 0;
  uint32_t drained_jobs = 0;
  int stop_http_status = 0;
  esp_err_t stop_ntfy_err = ESP_OK;
  bool stop_flush_attempted = false;
  if (AlertManagerIsConfigured(&g_state.alert_manager)) {
    alert_ntfy_job_t stop_job = { 0 };
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (AlertManagerBuildStopFlushNtfyJob(
          &g_state.alert_manager,
          "operator requested STOP (RUN -> DIAG)",
          now_ms,
          &stop_job,
          &drained_notes,
          &drained_jobs)) {
      AppendStopStorageSummaryLine(&g_state, &stop_job);
      int retry_after_seconds = -1;
      stop_flush_attempted = true;
      alert_ntfy_result_t stop_result_ntfy =
        RuntimeEnqueueNtfyJobAndWait(&g_state,
                                     &stop_job,
                                     1500u,
                                     &stop_http_status,
                                     &stop_ntfy_err,
                                     &retry_after_seconds);
      ESP_LOGW(
        kTag,
        "ntfy stop flush: drained_notes=%u drained_jobs=%u send_result=%s "
        "http_status=%d err=%s",
        (unsigned)drained_notes,
        (unsigned)drained_jobs,
        (stop_result_ntfy == ALERT_NTFY_OK)
          ? "OK"
          : ((stop_result_ntfy == ALERT_NTFY_SKIPPED) ? "SKIPPED" : "FAIL"),
        stop_http_status,
        esp_err_to_name(stop_ntfy_err));
    }
  }
  if (!stop_flush_attempted) {
    ESP_LOGW(kTag,
             "ntfy stop flush: drained_notes=%u drained_jobs=%u "
             "send_result=SKIPPED http_status=%d",
             (unsigned)drained_notes,
             (unsigned)drained_jobs,
             stop_http_status);
  }
  RuntimeRequestNetPause(&g_state, true, 1500u);
  ESP_LOGW(kTag, "Stop: sampling halt requested");
  esp_err_t stop_result = RuntimeStopSamplingOnly(&g_state);
  bool allow_drain = (stop_result == ESP_OK);
  if (allow_drain &&
      (g_state.sensor_task != NULL || g_state.storage_task != NULL ||
       g_state.sd_flush_task != NULL)) {
    if (g_state.sensor_task != NULL) {
      ESP_LOGW(kTag,
               "Diag entry guard: sensor_task still running (%p)",
               g_state.sensor_task);
    }
    if (g_state.storage_task != NULL) {
      ESP_LOGW(kTag,
               "Diag entry guard: storage_task still running (%p)",
               g_state.storage_task);
    }
    if (g_state.sd_flush_task != NULL) {
      ESP_LOGW(kTag,
               "Diag entry guard: sd_flush_task still running (%p)",
               g_state.sd_flush_task);
    }
    allow_drain = false;
    stop_result = ESP_ERR_TIMEOUT;
  }
  esp_err_t flush_result = ESP_OK;
  if (!allow_drain) {
    ESP_LOGW(kTag,
             "Diag entry: stop timeout; skipping FRAM->SD drain/unmount to "
             "avoid SD/SPI contention");
    g_state.sd_degraded = true;
  } else {
    ESP_LOGW(kTag, "Stop: pausing SPI users before drain");
    RuntimePauseSpiUsers(&g_state, 1000u);
    (void)RuntimeDrainDeferredToFram(&g_state, "stop deferred reconcile");
    UpdateCachedUint32(&g_state,
                       &g_state.cached_status.deferred_count,
                       DeferredLogCount(&g_state));
    UpdateCachedUint32(&g_state,
                       &g_state.cached_status.deferred_drops,
                       DeferredLogDrops(&g_state));
    UpdateCachedBool(&g_state,
                     &g_state.cached_status.deferred_active,
                     (DeferredLogCount(&g_state) > 0u));
    ESP_LOGW(kTag, "Stop: draining FRAM->SD (and unmounting)");
    sd_drain_stats_t drain_stats = { 0 };
    const int32_t stop_drain_max_ms = ResolveStopDrainMaxMs();
    flush_result = DrainFramToSd(&g_state,
                                 true,
                                 stop_drain_max_ms,
                                 CONFIG_APP_DRAIN_MAX_RECORDS_PER_PASS,
                                 CONFIG_APP_DRAIN_YIELD_EVERY_RECORDS,
                                 &drain_stats);
    RuntimeResumeSpiUsers(&g_state);
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
    g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
    return stop_result;
  }
  if (flush_result != ESP_OK && flush_result != ESP_ERR_TIMEOUT) {
    g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
    return flush_result;
  }
  g_state.runtime_phase = RUNTIME_PHASE_DIAGNOSTICS;
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
    g_state.data_stream_init_err = DataPortInit();
    UpdateCachedInt32(&g_state,
                      &g_state.cached_status.data_stream_init_err,
                      (int32_t)g_state.data_stream_init_err);
    if (g_state.data_stream_init_err != ESP_OK) {
      g_state.data_streaming_enabled = false;
      UpdateCachedBool(
        &g_state, &g_state.cached_status.data_stream_enabled, false);
      return;
    }
    (void)TryEmitCsvHeader(&g_state);
    UpdateCachedBool(
      &g_state, &g_state.cached_status.data_stream_enabled, true);
    return;
  }
  g_state.data_streaming_enabled = enabled;
  UpdateCachedBool(
    &g_state, &g_state.cached_status.data_stream_enabled, enabled);
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

DataPortBackend
RuntimeGetDataStreamBackend(void)
{
  return DataPortGetBackend();
}

esp_err_t
RuntimeGetDataStreamInitError(void)
{
  return g_state.data_stream_init_err;
}

bool
RuntimeIsI2cQuiesceActive(void)
{
  return g_state.i2c_quiesce_active;
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
 * @brief Notify SensorTask to wake immediately and re-evaluate settings.
 */
void
RuntimeNudgeSensorTask(void)
{
  RuntimeNotifyTask(g_state.sensor_task);
}

/**
 * @brief Execute RuntimeSdUnmountLocked.
 * @param state Parameter state.
 * @return Return the function result.
 */
static esp_err_t
RuntimeSdUnmountLocked(runtime_state_t* state)
{
  if (state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  RuntimeDiagHeapCheck(state, "SD unmount (manual before)", false);
  esp_err_t result = SdLoggerUnmount(&state->sd_logger);
  RuntimeDiagHeapCheck(state, "SD unmount (manual after)", false);
  UpdateCachedBool(
    state, &state->cached_status.sd_mounted, state->sd_logger.is_mounted);
  return result;
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
    result = RuntimeSdUnmountLocked(&g_state);
    RuntimeSdIoUnlock(&g_state);
  }
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
 * @brief Execute RuntimePrintStackMonitor.
 * @param headroom_bytes Parameter headroom_bytes.
 */
void
RuntimePrintStackMonitor(uint32_t headroom_bytes)
{
  StackMonitorPrint(&g_stack_monitor, headroom_bytes);
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
  if (RuntimeFramLogLock(&g_state, kFramLogLockTimeoutTicks)) {
    g_state.fram_overrun_ack_total =
      FramLogGetOverrunRecordsTotal(&g_state.fram_log);
    RuntimeFramLogUnlock(&g_state);
  }
  g_state.fram_overrun_active_streak = 0;
  g_state.fram_overrun_clear_streak = 0;
  UpdateCachedBool(&g_state, &g_state.cached_status.fram_overrun_active, false);
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
