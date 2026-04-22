#include "console_commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "app_net_config.h"
#include "app_settings.h"
#include "argtable3/argtable3.h"
#include "boot_mode.h"
#include "calibration.h"
#include "diagnostics/diag_fram.h"
#include "diagnostics/diag_mesh.h"
#include "diagnostics/diag_rtc.h"
#include "diagnostics/diag_rtd.h"
#include "diagnostics/diag_sd.h"
#include "diagnostics/diag_storage.h"
#include "diagnostics/diag_wifi.h"
#include "display_attention.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_console.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "fram_error_log.h"
#include "net_supervisor.h"
#include "runtime_health.h"
#include "time_civil.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include <fcntl.h>
#include <unistd.h>
#endif

#include "console_alerts.h"
#include "console_help.h"
#include "console_registry.h"
#include "esp_log.h"
#include "esp_system.h"
#include "i2c_bus.h"
#include "max31865_reader.h"
#include "mem_guard.h"
#include "runtime_manager.h"
#include "runtime_markers.h"
#include "sd_card_detect.h"
#include "sdkconfig.h"
#include "time_sync.h"
#include "units_gpio.h"
#include "wifi_credentials.h"
#include "wifi_manager.h"
#include "wifi_service.h"

static const char* kTag = "console";
static const TickType_t kConsoleFramLogLockTimeoutTicks = pdMS_TO_TICKS(2000);
static const int32_t kFlushTimeoutDefaultMs = 15000;
static const size_t kConsoleMaxCmdlineLength = 1024;
static const int kConsoleTransportRxBufferSize = 2048;
static const int kConsoleTransportTxBufferSize = 2048;
static const double kDefaultCaptureDriftLimitCPerMin = 0.020;
static char* s_console_cmdline_buffer = NULL;
static char* s_console_cmdline_snapshot = NULL;
static bool s_console_cmdline_buffer_in_psram = false;
static bool s_console_cmdline_snapshot_in_psram = false;
static void
FormatFileTime(const time_t* timestamp, char* buffer, size_t buffer_size);
static void
FormatUtcEpochIso8601(int64_t epoch_utc, char* buffer, size_t buffer_size);
static void
FormatLocalEpochIso8601(int64_t epoch_utc, char* buffer, size_t buffer_size);
static void
FormatErrlogFlags(uint16_t flags, char* buffer, size_t buffer_size);
static bool
CalConsoleOpIsActive(void);
static esp_err_t
CalConsoleOpStartLive(uint32_t every_ms,
                      int seconds,
                      int live_output_mode,
                      bool drift_notify_armed,
                      double drift_notify_threshold_c_per_min);
static esp_err_t
CalConsoleOpStartCapture(double actual_temp_c,
                         double stable_stddev_c,
                         bool drift_limit_enabled,
                         double drift_limit_c_per_min,
                         calibration_drift_limit_source_t drift_limit_source,
                         int min_seconds,
                         int timeout_seconds);
static void
CalConsoleOpRequestCancel(void);
static void
CalConsoleOpTask(void* task_arg);
static void
PrintCalWindowLine_(const char* prefix,
                    const char* timestamp_utc,
                    size_t sample_count,
                    int32_t last_raw_mC,
                    int32_t last_raw_mOhm,
                    int32_t mean_raw_mC,
                    int32_t stddev_mC,
                    double drift_c_per_min,
                    double delta_c,
                    bool drift_target_enabled,
                    const char* drift_eta_to_target_text);
static void
PrintCalWindowCalibratedLine_(const char* timestamp_utc,
                              size_t sample_count,
                              int32_t last_raw_mOhm,
                              int32_t mean_raw_mOhm,
                              int32_t stddev_mOhm,
                              double calibrated_temp_c,
                              double mean_calibrated_temp_c,
                              double stddev_calibrated_temp_c,
                              double drift_c_per_min,
                              double delta_c,
                              uint8_t fault_status);
static bool
ParseCalLiveArguments_(int argc,
                       char** argv,
                       const char* usage,
                       int* every_ms_out,
                       int* seconds_out,
                       bool* drift_notify_armed_out,
                       double* drift_notify_threshold_out);
static void
PrintCalLiveUsage_(const char* command_name);
static bool
FormatCalLiveTimestampNow_(char* buffer, size_t buffer_size);
static bool
EvaluateCalibratedTemperature_(double raw_temp_c,
                               double raw_resistance_ohm,
                               double* calibrated_temp_c_out);
static double
ConsoleResistanceToTemperatureFitCallback_(double resistance_ohm,
                                           void* context);
static void
PrintCalibrationFitLabelAndValueOrNa_(const char* label,
                                      bool available,
                                      double value);
static const char*
CalibrationFitDomainDisplayString_(calibration_domain_t fit_domain);
static const char*
CalibrationFitCorrectionLabel_(calibration_domain_t fit_domain);
static const char*
CalibrationFitCorrectionColumnHeader_(calibration_domain_t fit_domain);
static const char*
CalibrationFitDegreeLabel_(calibration_fit_mode_t fit_mode);
static void
PrintCalibrationFitValueOrNa_(const char* label,
                              bool available,
                              double value,
                              const char* unit_suffix);
static void
PrintCalibrationFitDiagnosticsBlock_(
  const calibration_fit_diagnostics_t* diagnostics);
static void
PrintCalibrationModelFitTable_(const calibration_fit_report_t* fit_report);
static void
PrintCalibrationModelFitReport_(const app_settings_t* settings);
static bool
RefreshCalibratedWindowTempStatsCache_(size_t sample_count,
                                       uint32_t window_generation,
                                       int32_t last_raw_mC,
                                       int32_t last_raw_mOhm,
                                       int32_t mean_raw_mC,
                                       int32_t mean_raw_mOhm);
static bool
CalHistoryEnsureStorage_(void);
static void
CalLogMemorySnapshot_(const char* stage, const char* mode_label);
static void
CalTrackLiveLowWatermark_(void);
static void
CalTrackLiveStackAndTempUsage_(size_t temp_bytes);
static void
CalLogLiveRetainedResources_(const char* mode_label);
/**
 * @brief Queue a one-shot cal live drift-ready ntfy message via alert queue.
 * @param drift_c_per_min Current gated drift value (C/min).
 * @param threshold_c_per_min Armed absolute drift threshold (C/min).
 * @param sustained_seconds Required sustained-under-threshold duration.
 * @return true when an ntfy job was queued successfully.
 */
static bool
QueueCalLiveDriftReadyNtfy_(double drift_c_per_min,
                            double threshold_c_per_min,
                            uint32_t sustained_seconds);
static void
PrintCalCaptureWindowLine_(double elapsed_s,
                           size_t sample_count,
                           int32_t last_raw_mC,
                           int32_t last_raw_mOhm,
                           int32_t mean_raw_mC,
                           double stddev_c,
                           double drift_c_per_min,
                           double delta_c,
                           double stable_elapsed_s,
                           int min_seconds,
                           double stable_stddev_c,
                           bool drift_limit_enabled,
                           double drift_limit_c_per_min,
                           calibration_drift_limit_source_t drift_limit_source,
                           double actual_temp_c);
static const char*
DriftLimitSourceToString_(calibration_drift_limit_source_t source);
static const char*
FindOptionValue_(int argc, char** argv, const char* name);
static bool
OptionPresent_(int argc, char** argv, const char* name);
static bool
ParseOptionInt_(int argc, char** argv, const char* name, int* value_out);
static bool
ParseOptionDouble_(int argc, char** argv, const char* name, double* value_out);
static bool
ParseTempC_(const char* text, double* value_out);
static bool
ParseCalAddPositionalArgs_(int argc,
                           char** argv,
                           double* raw_c_out,
                           double* actual_c_out);
static bool
ParseCalImportArgs_(int argc,
                    char** argv,
                    double* raw_ohm_out,
                    double* actual_c_out,
                    bool* raw_c_supplied_out,
                    double* raw_c_out,
                    bool* stddev_c_supplied_out,
                    double* stddev_c_out);
static bool
ParseCalibrationImportStatusLineSpan_(const char* line_start,
                                      size_t line_len,
                                      calibration_point_t* point_out,
                                      size_t* token_count_out);
static bool
ParseCalibrationImportStatusTokens_(int argc,
                                    char** argv,
                                    int start_index,
                                    calibration_point_t* point_out,
                                    size_t* token_count_out);
static bool
CalibrationImportPayloadFromConsoleLine_(const char* action,
                                         const char** payload_start_out,
                                         size_t* payload_len_out,
                                         bool* payload_quoted_out,
                                         bool* unterminated_quote_out);
static bool
NextCalibrationImportTokenSpan_(const char* line_start,
                                size_t line_len,
                                size_t* cursor_inout,
                                const char** token_start_out,
                                size_t* token_len_out);
static bool
ParseCalibrationImportDoubleSpan_(const char* value_start,
                                  size_t value_len,
                                  double* parsed_out);
static bool
ParseCalibrationImportDriftLimitSourceSpan_(
  const char* value_start,
  size_t value_len,
  calibration_drift_limit_source_t* source_out);
static bool
CalibrationImportKeyEquals_(const char* key_start,
                            size_t key_len,
                            const char* expected);
static int
FindCalibrationPointIndexByActualMc_(const app_settings_t* settings,
                                     int32_t actual_mC);
static esp_err_t
SaveCalibrationPointsAndDomain_(app_settings_t* settings);
static esp_err_t
DeleteCalibrationPointByDisplayIndex_(app_settings_t* settings,
                                      size_t display_index,
                                      calibration_point_t* deleted_point_out);
static bool
JoinArgsWithSpaces(int argc,
                   char** argv,
                   int start_index,
                   char* out,
                   size_t out_size);
static int
CalTopicHelp(const char* topic);
static int
SdTopicHelp(const char* topic);
static void
PrintModeHelpBody(void);
static void
PrintNetHelpBody(void);
static int
ModeTopicHelp(const char* topic);
/**
 * @brief Compute effective sensor poll period shown by console diagnostics.
 * @param settings Current settings snapshot.
 * @return Effective poll period in milliseconds.
 */
static uint32_t
ConsoleEffectiveSensorPollPeriodMs(const app_settings_t* settings);
static bool
ConsoleInputTooLongOrUnterminatedQuote_(const char* line,
                                        size_t max_cmdline_length);
static bool
ConsoleCmdlineBufferInit_(size_t max_cmdline_length);
static bool
ConsoleCmdlineSnapshotCapture_(const char* line, size_t max_cmdline_length);
static char*
ConsoleReadLine_(const char* prompt);

/**
 * @brief Print combined calibration status including window stats, points, and
 * schedule metadata.
 * @param settings Active settings snapshot.
 * @param state Runtime state for time stability / due-check flags.
 */
static void
PrintCalibrationStatusUnified(const app_settings_t* settings,
                              const runtime_state_t* state);
/**
 * @brief Print human-readable calibration equation and named coefficients.
 * @param model Active stored calibration model.
 */
static void
PrintCalibrationEquationBlock_(const calibration_model_t* model);
/**
 * @brief Print polynomial coefficients using symbolic names that map from
 * highest-order term down to constant term.
 * @param coefficients Stored polynomial coefficients in constant-first order.
 * @param degree Polynomial degree to print.
 */
static void
PrintNamedPolynomialCoefficients_(const double* coefficients, uint8_t degree);
/**
 * @brief Summary values for rendering one calibration point in status/report
 * output.
 */
typedef struct
{
  bool ideal_ref_res_available;
  bool captured_raw_res_available;
  bool captured_raw_temp_avg_available;
  bool captured_raw_temp_stddev_available;
  bool captured_raw_res_stddev_available;
  bool captured_drift_available;
  bool captured_delta_available;
  bool drift_limit_available;
  bool captured_window_available;
  bool captured_ema_alpha_available;
  bool residual_res_available;
  bool residual_temp_available;
  double ideal_ref_res_ohm;
  double captured_raw_res_avg_ohm;
  double captured_raw_temp_avg_c;
  double captured_raw_temp_stddev_c;
  double captured_raw_res_stddev_ohm;
  double captured_drift_c_per_min;
  double captured_delta_c;
  double drift_limit_c_per_min;
  double captured_ema_alpha;
  int captured_window_s;
  calibration_drift_limit_source_t drift_limit_source;
  double residual_res_ohm;
  double residual_temp_c;
} cal_point_print_summary_t;

/**
 * @brief Convert reference temperature to ideal PT100 resistance using the
 * shared CVD helper used by calibration apply.
 * @param reference_temp_c Reference temperature in Celsius.
 * @param out_ideal_ref_res_ohm Output ideal resistance in ohms.
 * @return true when conversion succeeds.
 */
static bool
ComputeIdealReferenceResistanceOhm_(double reference_temp_c,
                                    double* out_ideal_ref_res_ohm);

/**
 * @brief Build printable summary values for one calibration point.
 * @param point Calibration point.
 * @param summary_out Receives derived/availability fields for output.
 */
static void
BuildCalibrationPointPrintSummary_(const calibration_point_t* point,
                                   cal_point_print_summary_t* summary_out);

/**
 * @brief Print one concise calibration-point status line.
 * @param point Calibration point.
 * @param summary Precomputed printable summary values.
 * @param display_index 1-based display index.
 * @param point_is_resistance_domain True for captured/imported resistance
 * domain points.
 */
static void
PrintCalibrationPointStatusLine_(const calibration_point_t* point,
                                 const cal_point_print_summary_t* summary,
                                 uint32_t display_index,
                                 bool point_is_resistance_domain);

/**
 * @brief Print one report-friendly calibration-point block.
 * @param point Calibration point.
 * @param summary Precomputed printable summary values.
 * @param display_index 1-based display index.
 * @param point_is_resistance_domain True for captured/imported resistance
 * domain points.
 */
static void
PrintCalibrationPointReportBlock_(const calibration_point_t* point,
                                  const cal_point_print_summary_t* summary,
                                  uint32_t display_index,
                                  bool point_is_resistance_domain);

/**
 * @brief Force PTLOG daily file re-evaluation after calibration metadata
 * changes that affect PTLOG header content.
 * @param reason Human-readable trigger label for operator output.
 */
static void
ForcePtlogRevisionForCalibrationMetadata(const char* reason);

/**
 * @brief Print the compact calibration schedule and due status block.
 * @param settings Current settings.
 * @param state Current runtime state (may be NULL).
 */
static void
PrintCalibrationStatusBlock(const app_settings_t* settings,
                            const runtime_state_t* state);

/**
 * @brief Evaluate whether calibration is currently applied/valid for
 * reporting.
 * @param settings Current settings.
 * @param state Current runtime state (may be NULL).
 * @param reason_out Optional output reason string.
 * @return true when calibration is applied.
 */
static bool
ConsoleCalibrationIsApplied(const app_settings_t* settings,
                            const runtime_state_t* state,
                            const char** reason_out);

/**
 * @brief Parse a UTC date string in YYYY-MM-DD format into an epoch seconds
 * value.
 * @param value Input string.
 * @param epoch_out Output epoch seconds (UTC midnight).
 * @return true on success.
 */
static bool
ParseUtcDateString(const char* value, int64_t* epoch_out);

/**
 * @brief Parse calibration due unit string (days|months|years).
 * @param value Input string.
 * @param unit_out Output unit enum value.
 * @return true on success.
 */
static bool
ParseCalDueUnit(const char* value, uint8_t* unit_out);
static bool
ParseMetarAltimeterInHg(const char* token, float* altimeter_inHg_out);
static bool
MetarTokenLooksLikeObservation_(const char* token);
static bool
MetarTokenLooksLikeStation_(const char* token);
static bool
MetarTokenLooksLikeTempDew_(const char* token);
static bool
ResolveMetarObservationIsoUtc_(const char* token,
                               bool system_time_valid,
                               int64_t now_epoch_utc,
                               char* iso_out,
                               size_t iso_out_size,
                               bool* resolved_out);
static bool
ParseMetarLine_(const char* metar_line,
                calibration_metar_reference_t* metar_out,
                char* error_out,
                size_t error_out_size);
static void
PrintCalibrationMetarBlock_(const calibration_metar_reference_t* metar,
                            const char* heading,
                            bool include_report_block);
static bool
ParseElevationFt_(const char* token, uint16_t* elevation_ft_out);
static float
SaturationTempCFromStationInHg(float station_inHg);
static float
StationPressureFromSlpInHg(float slp_inHg, float elev_ft);
static void
PrintSatPtUsage(void);
/**
 * @brief Print system time in UTC and local time using current TZ/DST settings.
 * @param runtime Runtime context.
 */
static void
PrintTimeShow(const app_runtime_t* runtime);

static bool
ConsoleFramLogLock(const runtime_state_t* state)
{
  if (state == NULL || state->fram_log_mutex == NULL) {
    return false;
  }
  return xSemaphoreTake(state->fram_log_mutex,
                        kConsoleFramLogLockTimeoutTicks) == pdTRUE;
}

static void
ConsoleFramLogUnlock(const runtime_state_t* state)
{
  if (state == NULL || state->fram_log_mutex == NULL) {
    return;
  }
  (void)xSemaphoreGive(state->fram_log_mutex);
}
/**
 * @brief Execute MaybePushCalRawSampleFromSensor.
 */
typedef struct
{
  bool read_ok;
  bool sample_valid_for_window;
  bool sample_skipped_duplicate;
  bool sample_skipped_invalid_or_fault;
  uint8_t fault_status;
} cal_live_sensor_status_t;
static uint32_t g_cal_runtime_last_sample_id = 0u;

static cal_live_sensor_status_t
MaybePushCalRawSampleFromSensorWithStatus_(void)
{
  cal_live_sensor_status_t status = { 0 };

  const app_runtime_t* runtime = RuntimeGetRuntime();
  if (runtime == NULL || runtime->sensor == NULL) {
    return status;
  }

  if (RuntimeIsRunning()) {
    runtime_sensor_sample_t runtime_sample = { 0 };
    if (!RuntimeCopyLatestSensorSample(&runtime_sample)) {
      return status;
    }
    status.read_ok = true;
    status.fault_status = (uint8_t)runtime_sample.fault;
    if (runtime_sample.sample_id == g_cal_runtime_last_sample_id) {
      status.sample_skipped_duplicate = true;
      ESP_LOGD(kTag,
               "cal sample duplicate skipped: sample_id=%" PRIu32,
               runtime_sample.sample_id);
      return status;
    }
    g_cal_runtime_last_sample_id = runtime_sample.sample_id;

    if (!runtime_sample.valid || runtime_sample.fault != 0u) {
      status.sample_skipped_invalid_or_fault = true;
      ESP_LOGD(kTag,
               "cal sample skipped: sample_id=%" PRIu32
               " valid=%u fault=0x%02X",
               runtime_sample.sample_id,
               runtime_sample.valid ? 1u : 0u,
               (unsigned)runtime_sample.fault);
      return status;
    }

    // Calibration window is single-writer (console ops only).
    // Runtime produces sensor samples but MUST NOT write to the window.
    // Console consumes runtime samples via snapshot + sample_id tracking.
    CalWindowPushRawSample(runtime_sample.temp_mC, runtime_sample.ohm_mohm);
    status.sample_valid_for_window = true;
    ESP_LOGD(kTag,
             "cal sample consumed: sample_id=%" PRIu32,
             runtime_sample.sample_id);
    return status;
  }

  max31865_sample_t sample = { 0 };
  esp_err_t read_result = Max31865ReadOnce(runtime->sensor, &sample);
  if (read_result != ESP_OK) {
    return status;
  }

  status.read_ok = true;
  status.fault_status = sample.fault_status;
  if (sample.fault_present) {
    return status;
  }

  // Run mode can remain active while a console calibration operation is in
  // progress; however, this console path remains the sole writer to the
  // calibration window. Runtime/background/display/logging flows must not push
  // samples into this window.
  int32_t raw_milli_c = (int32_t)llround(sample.temperature_c * 1000.0);
  int32_t raw_milli_ohm = (int32_t)llround(sample.resistance_ohm * 1000.0);
  CalWindowPushRawSample(raw_milli_c, raw_milli_ohm);
  status.sample_valid_for_window = true;
  return status;
}

typedef enum
{
  CAL_CONSOLE_OP_NONE = 0,
  CAL_CONSOLE_OP_LIVE,
  CAL_CONSOLE_OP_CAPTURE
} cal_console_op_mode_t;

typedef enum
{
  CAL_CONSOLE_LIVE_OUTPUT_RAW = 0,
  CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED
} cal_console_live_output_mode_t;

typedef struct
{
  cal_console_op_mode_t mode;
  TaskHandle_t task_handle;
  volatile bool cancel_requested;
  uint32_t print_every_ms;
  int live_seconds;
  double capture_actual_temp_c;
  double capture_stable_stddev_c;
  bool capture_drift_limit_enabled;
  double capture_drift_limit_c_per_min;
  calibration_drift_limit_source_t capture_drift_limit_source;
  int capture_min_seconds;
  int capture_timeout_seconds;
  bool capture_armed;
  int64_t capture_armed_start_us;
  bool live_drift_notify_armed;
  double live_drift_notify_threshold_c_per_min;
  cal_console_live_output_mode_t live_output_mode;
  int64_t live_drift_under_start_us;
  bool live_drift_notify_sent;
  int64_t start_us;
  size_t last_sample_count;
  uint32_t start_internal_free;
  uint32_t start_internal_largest;
  uint32_t start_psram_free;
  uint32_t start_psram_largest;
  uint32_t low_internal_free;
  uint32_t low_internal_largest;
  uint32_t min_stack_free_bytes;
  uint32_t max_temp_buffer_bytes;
} cal_console_op_state_t;

static const uint32_t kCalOpTaskStackBytes = 6144u;
static const uint32_t kCalDriftReadyBodyMaxBytes = 160u;

static cal_console_op_state_t g_cal_console_op = { 0 };
static SemaphoreHandle_t g_cal_console_op_mutex = NULL;

static app_runtime_t* g_runtime = NULL;
static app_boot_mode_t g_boot_mode = APP_BOOT_MODE_DIAGNOSTICS;

/**
 * @brief Execute BootModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
BootModeToString(app_boot_mode_t mode)
{
  return (mode == APP_BOOT_MODE_RUN) ? "run" : "diagnostics";
}

/**
 * @brief Execute DisplayAttentionItemToName.
 * @param item Parameter item.
 * @return Return the function result.
 */
static const char*
DisplayAttentionItemToName(display_attention_item_t item)
{
  switch (item) {
    case kDispAttnItemSdOut:
      return "sdout";
    case kDispAttnItemSdIo:
      return "sdio";
    case kDispAttnItemFramOvr:
      return "framovr";
    case kDispAttnItemRtdFault:
      return "rtd";
    case kDispAttnItemTimeBad:
      return "time";
    case kDispAttnItemNtpFail:
      return "ntp";
    case kDispAttnItemMeshDown:
      return "mesh";
    case kDispAttnItemHeap:
      return "heap";
    case kDispAttnItemSdSpace:
      return "sdspace";
    default:
      return "unknown";
  }
}

static void
NotifyNetSupervisor(void)
{
  NetSupervisorNotifyUpdate();
}

static void
FormatPermille(uint16_t permille, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  const unsigned int whole = permille / 1000u;
  const unsigned int frac = permille % 1000u;
  snprintf(buffer, buffer_size, "%u.%03u", whole, frac);
}

static void
PrintRtdEmaSettings(const app_settings_t* settings,
                    const max31865_reader_t* reader)
{
  if (settings == NULL) {
    return;
  }
  char alpha_buffer[8] = { 0 };
  FormatPermille(
    settings->rtd_ema_alpha_permille, alpha_buffer, sizeof(alpha_buffer));
  printf("rtd_ema_enabled: %s\n", settings->rtd_ema_enabled ? "yes" : "no");
  printf("rtd_ema_alpha: %s\n", alpha_buffer);
  if (reader != NULL) {
    printf("rtd_ema_valid: %s\n", reader->ema_valid ? "yes" : "no");
  }
}

static void
PrintRtdFaultSettings(const app_settings_t* settings,
                      const runtime_state_t* state)
{
  if (settings == NULL) {
    return;
  }
  printf("rtd_fault_assert_ms: %u\n", (unsigned)settings->rtd_fault_assert_ms);
  printf("rtd_fault_clear_ms: %u\n", (unsigned)settings->rtd_fault_clear_ms);
  if (state != NULL) {
    char fault_desc[128] = { 0 };
    Max31865FormatFault(
      state->rtd_fault_last_status, fault_desc, sizeof(fault_desc));
    printf("rtd_fault_debounced: %s\n",
           state->last_sensor_fault_present ? "yes" : "no");
    printf("rtd_fault_last_status: 0x%02X\n", state->rtd_fault_last_status);
    printf("rtd_fault_last_desc: %s\n", fault_desc);
    printf("rtd_fault_suppressed_count: %u\n",
           (unsigned)state->rtd_fault_suppressed_count);
    printf("rtd_fault_first_read_count: %u\n",
           (unsigned)state->rtd_fault_first_read_count);
    printf("rtd_fault_retry_fault_count: %u\n",
           (unsigned)state->rtd_fault_retry_fault_count);
    printf("rtd_fault_retry_clean_count: %u\n",
           (unsigned)state->rtd_fault_retry_clean_count);
    printf("rtd_fault_last_first_status: 0x%02X\n",
           state->rtd_fault_last_first_status);
    printf("rtd_fault_last_retry_status: 0x%02X\n",
           state->rtd_fault_last_retry_status);
    printf("rtd_fault_last_retry_err: %s\n",
           esp_err_to_name(state->rtd_fault_last_retry_err));
    printf("rtd_fault_ntfy_pending_first_read_count: %u\n",
           (unsigned)state->rtd_fault_ntfy_pending_first_read_count);
    printf("rtd_fault_ntfy_pending_retry_fault_count: %u\n",
           (unsigned)state->rtd_fault_ntfy_pending_retry_fault_count);
    printf("rtd_fault_ntfy_pending_retry_clean_count: %u\n",
           (unsigned)state->rtd_fault_ntfy_pending_retry_clean_count);
    printf("rtd_fault_ntfy_suppressed_count: %u\n",
           (unsigned)state->rtd_fault_ntfy_suppressed_count);
    printf("rtd_read_err_last: %s\n",
           esp_err_to_name(state->max31865_last_read_err));
    printf("rtd_read_err_count: %u\n",
           (unsigned)state->max31865_read_err_count);
    printf("rtd_invalid_state_count: %u\n",
           (unsigned)state->max31865_invalid_state_count);
  }
}

/**
 * @brief Execute ParseDisplayAttentionName.
 * @param value Parameter value.
 * @param item_out Parameter item_out.
 * @return Return the function result.
 */
static bool
ParseDisplayAttentionName(const char* value, display_attention_item_t* item_out)
{
  if (value == NULL || item_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "sdout") == 0) {
    *item_out = kDispAttnItemSdOut;
    return true;
  }
  if (strcasecmp(value, "sdio") == 0) {
    *item_out = kDispAttnItemSdIo;
    return true;
  }
  if (strcasecmp(value, "framovr") == 0) {
    *item_out = kDispAttnItemFramOvr;
    return true;
  }
  if (strcasecmp(value, "rtd") == 0) {
    *item_out = kDispAttnItemRtdFault;
    return true;
  }
  if (strcasecmp(value, "time") == 0) {
    *item_out = kDispAttnItemTimeBad;
    return true;
  }
  if (strcasecmp(value, "ntp") == 0) {
    *item_out = kDispAttnItemNtpFail;
    return true;
  }
  if (strcasecmp(value, "mesh") == 0) {
    *item_out = kDispAttnItemMeshDown;
    return true;
  }
  if (strcasecmp(value, "heap") == 0) {
    *item_out = kDispAttnItemHeap;
    return true;
  }
  if (strcasecmp(value, "sdspace") == 0) {
    *item_out = kDispAttnItemSdSpace;
    return true;
  }
  return false;
}

/**
 * @brief Execute WifiServiceModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
WifiServiceModeToString(wifi_service_mode_t mode)
{
  switch (mode) {
    case WIFI_SERVICE_MODE_NONE:
      return "none";
    case WIFI_SERVICE_MODE_DIAGNOSTIC_STA:
      return "diagnostic_sta";
    case WIFI_SERVICE_MODE_MESH:
      return "mesh";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute WifiAuthModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
WifiAuthModeToString(wifi_auth_mode_t mode)
{
  switch (mode) {
    case WIFI_AUTH_OPEN:
      return "open";
    case WIFI_AUTH_WEP:
      return "wep";
    case WIFI_AUTH_WPA_PSK:
      return "wpa_psk";
    case WIFI_AUTH_WPA2_PSK:
      return "wpa2_psk";
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "wpa_wpa2";
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "wpa2_ent";
    case WIFI_AUTH_WPA3_PSK:
      return "wpa3_psk";
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "wpa2_wpa3";
    case WIFI_AUTH_WAPI_PSK:
      return "wapi_psk";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute WifiDisconnectReasonToString.
 * @param reason Parameter reason.
 * @return Return the function result.
 */
static const char*
WifiDisconnectReasonToString(wifi_err_reason_t reason)
{
  switch (reason) {
    case WIFI_REASON_AUTH_EXPIRE:
      return "auth_expire";
    case WIFI_REASON_AUTH_LEAVE:
      return "auth_leave";
    case WIFI_REASON_ASSOC_EXPIRE:
      return "assoc_expire";
    case WIFI_REASON_ASSOC_TOOMANY:
      return "assoc_toomany";
    case WIFI_REASON_NOT_AUTHED:
      return "not_authed";
    case WIFI_REASON_NOT_ASSOCED:
      return "not_assoc";
    case WIFI_REASON_ASSOC_LEAVE:
      return "assoc_leave";
    case WIFI_REASON_ASSOC_NOT_AUTHED:
      return "assoc_not_authed";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:
      return "disassoc_pwrcap";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
      return "disassoc_supchan";
    case WIFI_REASON_IE_INVALID:
      return "ie_invalid";
    case WIFI_REASON_MIC_FAILURE:
      return "mic_failure";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
      return "4way_timeout";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
      return "gk_timeout";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:
      return "ie_4way_diff";
    case WIFI_REASON_GROUP_CIPHER_INVALID:
      return "group_cipher";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
      return "pairwise_cipher";
    case WIFI_REASON_AKMP_INVALID:
      return "akmp_invalid";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
      return "rsn_ver";
    case WIFI_REASON_INVALID_RSN_IE_CAP:
      return "rsn_cap";
    case WIFI_REASON_802_1X_AUTH_FAILED:
      return "8021x_failed";
    case WIFI_REASON_BEACON_TIMEOUT:
      return "beacon_timeout";
    case WIFI_REASON_AUTH_FAIL:
      return "auth_fail";
    case WIFI_REASON_NO_AP_FOUND:
      return "no_ap";
    case WIFI_REASON_CONNECTION_FAIL:
      return "conn_fail";
    case WIFI_REASON_ASSOC_FAIL:
      return "assoc_fail";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
      return "handshake_timeout";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute PrintWifiUsage.
 */
static void
PrintWifiUsage(void)
{
  printf(
    "usage:\n"
    "  wifi help\n"
    "  wifi show\n"
    "  wifi set <ssid> [password]\n"
    "  wifi clear\n"
    "  wifi scan [--max N]\n"
    "  wifi status\n"
    "  wifi connect [--timeout_ms T]\n"
    "  wifi disconnect\n"
    "  wifi cfg show\n"
    "  wifi cfg defaults\n"
    "  wifi cfg set sntp <primary[, secondary[, tertiary]]>\n"
    "  wifi cfg set sntp_fail_n <1..20>\n"
    "  wifi cfg set mesh_chan <1..13>\n"
    "  wifi cfg set mesh_id <XX:XX:XX:XX:XX:XX>\n"
    "  wifi cfg set mesh_ap_pass <password>\n"
    "  wifi cfg set no_router 0|1\n"
    "  wifi cfg set time_sync_s <seconds>\n"
    "  wifi ntp status\n"
    "  wifi ntp sync [--server host] [--timeout_ms T] [--update-rtc 0|1]\n"
    "\n"
    "notes:\n"
    "  - wifi cfg set ... writes to NVS and persists across reboot.\n"
    "  - wifi cfg defaults clears NVS overrides (revert to Kconfig defaults).\n"
    "  - Changes are applied by the network supervisor when possible.\n"
    "  - time_sync_s is clamped to 5..3600 seconds.\n"
    "  - wifi ntp sync performs a one-off sync; it does not change NVS\n"
    "    unless you also use wifi cfg set sntp/time_sync_s.\n"
    "  - sntp CSV allows optional spaces after commas; whitespace is "
    "stripped.\n");
}

// In diagnostics/console mode, Wi-Fi may not be initialized yet. Most Wi-Fi
// manager operations (scan/connect/disconnect) require the Wi-Fi service to be
// acquired so the netif/event loop/Wi-Fi driver are ready.
//
// This helper acquires the diagnostic STA mode if Wi-Fi is currently idle.
// If the system is running Wi-Fi for another mode (e.g., mesh), we refuse to
// mutate it from the console.
static esp_err_t
AcquireWifiForConsole(bool* did_acquire)
{
  if (did_acquire != NULL) {
    *did_acquire = false;
  }

  (void)WifiServiceInitOnce();

  const wifi_service_mode_t active_mode = WifiServiceActiveMode();
  if (active_mode != WIFI_SERVICE_MODE_NONE &&
      active_mode != WIFI_SERVICE_MODE_DIAGNOSTIC_STA) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t result = WifiServiceAcquire(WIFI_SERVICE_MODE_DIAGNOSTIC_STA);
  if (result == ESP_OK && did_acquire != NULL) {
    *did_acquire = true;
  }
  return result;
}

static void
ReleaseWifiForConsoleIfNeeded(bool did_acquire)
{
  if (did_acquire) {
    (void)WifiServiceRelease();
  }
}

/**
 * @brief Execute PrintWifiConfig.
 */
static void
PrintWifiConfig(void)
{
  const uint8_t mesh_channel = AppNetConfigGetMeshChannel();
  const char* mesh_id = AppNetConfigGetMeshIdString();
  const bool mesh_id_valid = (mesh_id != NULL && mesh_id[0] != '\0');
  const char* ap_password = AppNetConfigGetMeshApPassword();
  const size_t ap_password_len = strlen(ap_password);
  const char* sntp_server_csv = AppNetConfigGetSntpServersCsv();
  const uint8_t sntp_server_count = AppNetConfigGetSntpServerCount();
  const char* sntp_primary = AppNetConfigGetSntpServerAt(0);
  const char* sntp_secondary = AppNetConfigGetSntpServerAt(1);
  const char* sntp_tertiary = AppNetConfigGetSntpServerAt(2);

  printf("mesh_channel: %u\n", (unsigned)mesh_channel);
  printf("mesh_channel_source: %s\n",
         AppNetConfigMeshChannelIsOverridden() ? "nvs" : "kconfig");
  printf("mesh_id: %s\n", mesh_id_valid ? mesh_id : "<invalid>");
  printf("mesh_id_source: %s\n",
         AppNetConfigMeshIdIsOverridden() ? "nvs" : "kconfig");
  if (ap_password_len == 0) {
    printf("mesh_ap_password: <empty>\n");
  } else {
    printf("mesh_ap_password: <redacted>\n");
  }
  printf("mesh_ap_password_len: %u\n", (unsigned)ap_password_len);
  printf("mesh_ap_password_source: %s\n",
         AppNetConfigMeshApPasswordIsOverridden() ? "nvs" : "kconfig");
  printf("mesh_no_router: %u\n", AppNetConfigGetMeshDisableRouter() ? 1u : 0u);
  printf("mesh_no_router_source: %s\n",
         AppNetConfigMeshDisableRouterIsOverridden() ? "nvs" : "kconfig");

  printf("sntp_servers_csv: %s\n",
         (sntp_server_csv[0] != '\0') ? sntp_server_csv : "<empty>");
  printf("sntp_server_count: %u\n", (unsigned)sntp_server_count);
  printf("sntp_primary: %s\n",
         (sntp_primary[0] != '\0') ? sntp_primary : "<empty>");
  printf("sntp_secondary: %s\n",
         (sntp_secondary[0] != '\0') ? sntp_secondary : "<empty>");
  printf("sntp_tertiary: %s\n",
         (sntp_tertiary[0] != '\0') ? sntp_tertiary : "<empty>");
  printf("sntp_server_source: %s\n",
         AppNetConfigSntpServerIsOverridden() ? "nvs" : "kconfig");
  printf("sntp_fail_n: %u\n", (unsigned)AppNetConfigGetSntpFailThresholdN());
  printf("sntp_fail_n_source: %s\n",
         AppNetConfigSntpFailThresholdIsOverridden() ? "nvs" : "kconfig");
  printf("time_sync_s: %u\n", (unsigned)AppNetConfigGetTimeSyncPeriodSeconds());
  printf("time_sync_s_source: %s\n",
         AppNetConfigTimeSyncPeriodIsOverridden() ? "nvs" : "kconfig");
}

/**
 * @brief Execute PickDefaultSntpServer.
 * @return Return the function result.
 */
static const char*
PickDefaultSntpServer(void)
{
  const char* server = AppNetConfigGetSntpServer();
  return (server[0] != '\0') ? server : "pool.ntp.org";
}

/**
 * @brief Execute PrintDisplayAttentionPolicy.
 * @param policy Parameter policy.
 */
static void
PrintDisplayAttentionPolicy(uint32_t policy)
{
  const display_attention_item_t items[] = {
    kDispAttnItemSdOut,    kDispAttnItemSdIo,    kDispAttnItemFramOvr,
    kDispAttnItemRtdFault, kDispAttnItemTimeBad, kDispAttnItemNtpFail,
    kDispAttnItemMeshDown, kDispAttnItemHeap,    kDispAttnItemSdSpace,
  };
  printf("display_attention_policy: 0x%08" PRIX32 "\n", policy);
  for (size_t idx = 0; idx < sizeof(items) / sizeof(items[0]); ++idx) {
    const display_attention_item_t item = items[idx];
    const display_attention_severity_t severity =
      DisplayAttentionPolicyGet(policy, item);
    const char* label = "off";
    if (severity == DISP_SEV_WARN) {
      label = "warn";
    } else if (severity == DISP_SEV_ERROR) {
      label = "error";
    }
    printf("  %s: %s\n", DisplayAttentionItemToName(item), label);
  }
}

/**
 * @brief Execute SaveDisplayAttentionPolicy.
 * @param policy Parameter policy.
 * @return Return the function result.
 */
static int
SaveDisplayAttentionPolicy(uint32_t policy)
{
  if (g_runtime == NULL) {
    return 1;
  }
  const esp_err_t result = AppSettingsSaveDisplayAttentionPolicy(policy);
  if (result != ESP_OK) {
    printf("save failed: %s\n", esp_err_to_name(result));
    return 1;
  }
  g_runtime->settings->display_attention_policy = policy;
  g_runtime->settings->display_attention_mask =
    AppSettingsGetDisplayAttentionMask();

  // Ensure the display task reflects policy changes immediately.
  RuntimeSetDisplayAttentionPolicy(policy);
  printf("OK\n");
  PrintDisplayAttentionPolicy(policy);
  return 0;
}

/**
 * @brief Execute ParseDisplayAttentionSeverity.
 * @param value Parameter value.
 * @param severity_out Parameter severity_out.
 * @return Return the function result.
 */
static bool
ParseDisplayAttentionSeverity(const char* value,
                              display_attention_severity_t* severity_out)
{
  if (value == NULL || severity_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "off") == 0) {
    *severity_out = DISP_SEV_OFF;
    return true;
  }
  if (strcasecmp(value, "warn") == 0) {
    *severity_out = DISP_SEV_WARN;
    return true;
  }
  if (strcasecmp(value, "error") == 0) {
    *severity_out = DISP_SEV_ERROR;
    return true;
  }
  return false;
}

/**
 * @brief Execute CalibrationModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
CalibrationModeToString(calibration_fit_mode_t mode)
{
  switch (mode) {
    case CAL_FIT_MODE_LINEAR:
      return "linear";
    case CAL_FIT_MODE_PIECEWISE:
      return "piecewise";
    case CAL_FIT_MODE_POLY:
      return "poly";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute SaveCalibrationWithContext.
 * @param model Parameter model.
 * @return Return the function result.
 */
static esp_err_t
SaveCalibrationWithContext(const calibration_model_t* model)
{
  if (g_runtime == NULL || g_runtime->sensor == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  calibration_context_t context;
  AppSettingsBuildCalibrationContextFromReader(&context, g_runtime->sensor);

  if (g_runtime->settings != NULL) {
    g_runtime->settings->calibration = *model;
    g_runtime->settings->calibration_context = context;
    g_runtime->settings->calibration_context_valid = true;
  }

  esp_err_t result = AppSettingsSaveCalibrationWithContext(model, &context);
  if (result != ESP_OK) {
    return result;
  }
  if (TimeSyncIsSystemTimeValid() && g_runtime->settings != NULL) {
    g_runtime->settings->cal_last_utc = (int64_t)time(NULL);
    result = AppSettingsSaveCalibrationSchedule(g_runtime->settings);
  }
  return result;
}

/**
 * @brief Compute effective sensor polling period from display and log periods.
 * @param settings Current settings snapshot.
 * @return Poll period in milliseconds clamped to [100, 3600000].
 */
static uint32_t
ConsoleEffectiveSensorPollPeriodMs(const app_settings_t* settings)
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
 * @brief Execute CommandStatus.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandStatus(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  if (g_runtime == NULL) {
    return 1;
  }

  const app_settings_t* settings = g_runtime->settings;
  const runtime_state_t* state = RuntimeGetState();

  printf("node_id: %s\n", g_runtime->node_id_string);
  printf("runtime_running: %s\n", RuntimeIsRunning() ? "yes" : "no");
  printf("time_valid: %s\n", TimeSyncIsSystemTimeValid() ? "yes" : "no");
  printf("log_period_ms: %u\n", (unsigned)settings->log_period_ms);
  printf("display_sample_period_ms: %u\n",
         (unsigned)settings->display_sample_period_ms);
  printf("rtc_resync_period_ms: %u\n",
         (unsigned)settings->rtc_resync_period_ms);
  printf("sd_flush_period_ms: %u\n", (unsigned)settings->sd_flush_period_ms);
  printf("sd_batch_target_bytes: %u\n",
         (unsigned)settings->sd_batch_bytes_target);
  printf("sd_verify_readback: %s\n",
         (state != NULL && state->sd_verify_readback) ? "on" : "off");
  printf("node_role: %s\n", AppSettingsRoleToString(settings->node_role));
  const app_net_mode_t effective_net_mode =
    AppSettingsGetEffectiveNetMode(settings->node_role, settings->net_mode);
  printf("configured_net_mode: %s\n",
         AppSettingsNetModeToString(settings->net_mode));
  printf("effective_net_mode: %s\n",
         AppSettingsNetModeToString(effective_net_mode));
  const char* net_override_reason = AppSettingsGetNetModeOverrideReason(
    settings->node_role, settings->net_mode);
  if (net_override_reason != NULL) {
    printf("effective_override_reason: %s\n", net_override_reason);
  }
  printf("allow_children: %s\n", settings->allow_children ? "yes" : "no");
  printf("tz_posix: %s\n", settings->tz_posix);
  printf("dst_enabled: %s\n", settings->dst_enabled ? "yes" : "no");
  printf("mqtt_enabled: %s\n", settings->mqtt_enabled ? "yes" : "no");
  printf("mqtt_broker_uri: %s\n", settings->mqtt_broker_uri);
  printf("mqtt_topic_prefix: %s\n", settings->mqtt_topic_prefix);
  printf("mqtt_qos: %u\n", (unsigned)settings->mqtt_qos);
  printf("mqtt_retain: %s\n", settings->mqtt_retain ? "yes" : "no");
  if (settings->node_role == APP_NODE_ROLE_ROOT) {
    printf("bridge_mode: %s\n",
           AppSettingsMqttBridgeModeToString(settings->mqtt_bridge_mode));
    const bool broker_bridge_active =
      settings->mqtt_enabled &&
      (settings->mqtt_bridge_mode == MQTT_BRIDGE_BROKER ||
       settings->mqtt_bridge_mode == MQTT_BRIDGE_BOTH);
    printf("broker_bridge_active: %s\n", broker_bridge_active ? "yes" : "no");
  }
  PrintRtdEmaSettings(settings, g_runtime->sensor);
  PrintRtdFaultSettings(settings, RuntimeGetState());

  // Ensure the TZ rules are loaded before formatting local time.
  // (TZ is applied via AppSettingsApplyTimeZone() at boot and by the tz/dst
  // commands.)
  tzset();

  const time_t now = time(NULL);

  struct tm utc_time;
  char utc_buffer[48] = { 0 };
  if (gmtime_r(&now, &utc_time) != NULL) {
    strftime(utc_buffer, sizeof(utc_buffer), "%Y-%m-%d %H:%M:%SZ", &utc_time);
  }
  printf("utc_time: %s (epoch=%ld)\n",
         (utc_buffer[0] != '\0') ? utc_buffer : "unknown",
         (long)now);

  struct tm local_time;
  char local_buffer[48] = { 0 };
  if (localtime_r(&now, &local_time) != NULL) {
    strftime(
      local_buffer, sizeof(local_buffer), "%Y-%m-%d %H:%M:%S", &local_time);
  }

  // Compute UTC offset in seconds (local = UTC + offset).
  // Avoid relying on non-portable tm_gmtoff.
  struct tm utc_as_local = utc_time;
  utc_as_local.tm_isdst = -1;
  const time_t utc_epoch_as_local = mktime(&utc_as_local);
  long utc_offset_sec = 0;
  if (utc_epoch_as_local != (time_t)-1) {
    utc_offset_sec = (long)difftime(now, utc_epoch_as_local);
  }

  printf("local_time: %s (utc_offset_sec=%ld dst_in_effect=%d)\n",
         (local_buffer[0] != '\0') ? local_buffer : "unknown",
         utc_offset_sec,
         local_time.tm_isdst);
  size_t fram_count = 0;
  size_t fram_capacity = 0;
  uint64_t fram_overrun_total = 0;
  uint32_t fram_buffered = 0;
  uint32_t fram_next_sequence = 0;
  const bool fram_locked = ConsoleFramLogLock(state);
  if (fram_locked) {
    fram_count = FramLogGetCountRecords(g_runtime->fram_log);
    fram_capacity = FramLogGetCapacityRecords(g_runtime->fram_log);
    fram_overrun_total = FramLogGetOverrunRecordsTotal(g_runtime->fram_log);
    fram_buffered = FramLogGetBufferedRecords(g_runtime->fram_log);
    fram_next_sequence = FramLogNextSequence(g_runtime->fram_log);
    ConsoleFramLogUnlock(state);
  } else {
    printf("fram_stats: unavailable (lock timeout)\n");
  }
  const uint32_t fram_fill_pct =
    (fram_capacity > 0) ? (uint32_t)((fram_count * 100u) / fram_capacity) : 0u;
  printf("fram_count: %zu\n", fram_count);
  printf("fram_capacity: %zu\n", fram_capacity);
  printf("fram_fill_pct: %u\n", (unsigned)fram_fill_pct);
  printf("fram_overrun_records_total: %" PRIu64 "\n", fram_overrun_total);
  printf("fram_flush_watermark_records: %u\n",
         (unsigned)settings->fram_flush_watermark_records);
  const bool fram_full =
    (g_runtime->fram_full != NULL) ? *g_runtime->fram_full : false;
  printf("fram_full: %s\n", fram_full ? "yes" : "no");
  printf("fram_count/seq: %u/%u\n",
         (unsigned)fram_buffered,
         (unsigned)fram_next_sequence);
  if (state != NULL) {
    printf("fram_corrupt_detect_count: %u\n",
           (unsigned)state->fram_corrupt_detect_count);
    printf("fram_corrupt_skip_count: %u\n",
           (unsigned)state->fram_corrupt_skip_count);
    printf("fram_corrupt_last_reason: %s\n",
           FramLogValidateResultToString(state->fram_corrupt_last_reason));
    printf("fram_corrupt_last_slot: %u\n",
           (unsigned)state->fram_corrupt_last_slot);
    printf("fram_corrupt_last_addr: 0x%04x\n",
           (unsigned)state->fram_corrupt_last_addr);
    printf("fram_corrupt_last_magic: 0x%08" PRIx32 "\n",
           state->fram_corrupt_last_magic);
    printf("fram_corrupt_last_schema: %u\n",
           (unsigned)state->fram_corrupt_last_schema);
    printf("fram_corrupt_last_exp_crc: 0x%04x\n",
           (unsigned)state->fram_corrupt_last_exp_crc);
    printf("fram_corrupt_last_act_crc: 0x%04x\n",
           (unsigned)state->fram_corrupt_last_act_crc);
  }
  const uint32_t export_dropped = (g_runtime->export_dropped_count != NULL)
                                    ? *g_runtime->export_dropped_count
                                    : 0u;
  const uint32_t export_write_fail =
    (g_runtime->export_write_fail_count != NULL)
      ? *g_runtime->export_write_fail_count
      : 0u;
  printf("data_csv_drop_count: %u\n", (unsigned)export_dropped);
  printf("data_csv_write_fail_count: %u\n", (unsigned)export_write_fail);
  printf("data_stream_enabled: %s\n",
         RuntimeIsDataStreamingEnabled() ? "yes" : "no");
  DataPortBackend data_stream_backend = RuntimeGetDataStreamBackend();
  printf("data_stream_backend: %s\n",
         DataPortBackendToString(data_stream_backend));
  if (data_stream_backend == DATA_PORT_BACKEND_UART0) {
    int32_t uart0_tx_gpio = -1;
    int32_t uart0_rx_gpio = -1;
    DataPortGetUart0Pins(&uart0_tx_gpio, &uart0_rx_gpio);
    printf("data_stream_uart0_tx_gpio: %ld\n", (long)uart0_tx_gpio);
    printf("data_stream_uart0_rx_gpio: %ld\n", (long)uart0_rx_gpio);
  }
  printf("data_stream_init_err: %s\n",
         esp_err_to_name(RuntimeGetDataStreamInitError()));
  const uint32_t export_drop_count =
    (g_runtime->export_drop_count != NULL) ? *g_runtime->export_drop_count : 0u;
  const uint32_t export_send_fail_count =
    (g_runtime->export_send_fail_count != NULL)
      ? *g_runtime->export_send_fail_count
      : 0u;
  const uint32_t broker_drop_count =
    (g_runtime->broker_drop_count != NULL) ? *g_runtime->broker_drop_count : 0u;
  const uint32_t broker_send_fail_count =
    (g_runtime->broker_send_fail_count != NULL)
      ? *g_runtime->broker_send_fail_count
      : 0u;
  const runtime_cached_status_t* cached_status = RuntimeGetCachedStatus();
  runtime_health_snapshot_t health = { 0 };
  if (state != NULL) {
    RuntimeHealthRead(&state->health_cache, &health);
  }
  const bool mqtt_connected = (g_runtime->mqtt_client_connected != NULL)
                                ? *g_runtime->mqtt_client_connected
                                : false;
  UBaseType_t export_outbox_used = 0;
  if (g_runtime->export_outbox != NULL && *g_runtime->export_outbox != NULL) {
    export_outbox_used = uxQueueMessagesWaiting(*g_runtime->export_outbox);
  }
  printf("mqtt_connected: %s\n", mqtt_connected ? "yes" : "no");
  printf("export_outbox_depth/used: %u/%u\n",
         (unsigned)CONFIG_APP_EXPORT_OUTBOX_DEPTH,
         (unsigned)export_outbox_used);
  printf("export_drop_count: %u\n", (unsigned)export_drop_count);
  printf("export_send_fail_count: %u\n", (unsigned)export_send_fail_count);
  if (settings->node_role == APP_NODE_ROLE_ROOT) {
    UBaseType_t broker_outbox_used = 0;
    if (g_runtime->broker_outbox != NULL && *g_runtime->broker_outbox != NULL) {
      broker_outbox_used = uxQueueMessagesWaiting(*g_runtime->broker_outbox);
    }
    printf("broker_outbox_depth/used: %u/%u\n",
           (unsigned)CONFIG_APP_BROKER_OUTBOX_DEPTH,
           (unsigned)broker_outbox_used);
    printf("broker_drop_count: %u\n", (unsigned)broker_drop_count);
    printf("broker_send_fail_count: %u\n", (unsigned)broker_send_fail_count);
    if (cached_status != NULL) {
      printf("root_publish_consumer_active: %s\n",
             cached_status->root_publish_consumer_active ? "yes" : "no");
      printf("root_publish_drop_no_consumer: %u\n",
             (unsigned)cached_status->root_publish_drop_no_consumer);
    }
  }

  printf("calibration: mode=%s degree=%u coeffs=[%.9g, %.9g, %.9g, %.9g]\n",
         CalibrationModeToString(settings->calibration.mode),
         (unsigned)settings->calibration.degree,
         settings->calibration.coefficients[0],
         settings->calibration.coefficients[1],
         settings->calibration.coefficients[2],
         settings->calibration.coefficients[3]);
  PrintCalibrationStatusBlock(settings, state);

  const bool sd_mounted = g_runtime->sd_logger->is_mounted;
  const bool sd_degraded = RuntimeSdIsDegraded();
  const uint32_t sd_fail_count = RuntimeSdFailCount();
  const TickType_t now_ticks = xTaskGetTickCount();
  const uint32_t sd_backoff_until = RuntimeSdBackoffUntilTicks();
  uint32_t sd_backoff_remaining_ms = 0;
  if (sd_backoff_until != 0 && now_ticks < (TickType_t)sd_backoff_until) {
    sd_backoff_remaining_ms =
      (uint32_t)pdTICKS_TO_MS((TickType_t)sd_backoff_until - now_ticks);
  }
  const char* sd_card_present = "unknown";
  const char* sd_safe_to_remove = "unknown";
  if (cached_status != NULL) {
    sd_card_present = cached_status->sd_card_present ? "yes" : "no";
    sd_safe_to_remove = cached_status->sd_safe_to_remove ? "yes" : "no";
  }
  printf("sd_mounted: %s\n", sd_mounted ? "yes" : "no");
  printf("sd_card_present: %s\n", sd_card_present);
  printf("sd_safe_to_remove: %s\n", sd_safe_to_remove);
  printf("sd_card_detect_gpio: %d\n", CONFIG_APP_SD_CARD_DETECT_GPIO);
  printf("sd_card_detect_present_level: %s\n",
#if CONFIG_APP_SD_CARD_DETECT_PRESENT_HIGH
         "high"
#else
         "low"
#endif
  );
  printf("sd_degraded: %s\n", sd_degraded ? "yes" : "no");
  if (cached_status != NULL) {
    printf("deferred_count: %u\n", (unsigned)cached_status->deferred_count);
    printf("deferred_drops: %u\n", (unsigned)cached_status->deferred_drops);
    printf("deferred_active: %s\n",
           cached_status->deferred_active ? "yes" : "no");
    printf("i2c_quiesce_active: %s\n",
           cached_status->i2c_quiesce_active ? "yes" : "no");
  }
  printf("sd_fail_count: %u\n", (unsigned)sd_fail_count);
  printf("sd_backoff_remaining_ms: %u\n", (unsigned)sd_backoff_remaining_ms);
  if (cached_status != NULL) {
    const uint64_t total = cached_status->sd_total_bytes;
    const uint64_t free = cached_status->sd_free_bytes;
    printf("sd_total_bytes: %" PRIu64 " (%" PRIu64 " MiB)\n",
           total,
           (total / (1024u * 1024u)));
    printf("sd_free_bytes: %" PRIu64 " (%" PRIu64 " MiB)\n",
           free,
           (free / (1024u * 1024u)));
    printf("sd_space_reclaim_active: %s\n",
           cached_status->sd_space_reclaim_active ? "yes" : "no");
    printf("sd_space_reclaim_deleted_total: %u\n",
           (unsigned)cached_status->sd_space_reclaim_deleted_total);
    printf("sd_out_of_space_active: %s\n",
           cached_status->sd_out_of_space_active ? "yes" : "no");
  }
  printf("sd_last_record_id: %" PRIu64 "\n",
         SdLoggerLastRecordIdOnSd(g_runtime->sd_logger));
#if CONFIG_APP_HEAP_MONITOR_ENABLE
  printf("heap_internal_free_bytes: %u\n",
         (unsigned)health.heap_internal_free_bytes);
  printf("heap_internal_largest_free_block_bytes: %u\n",
         (unsigned)health.heap_internal_largest_free_block_bytes);
  printf("heap_internal_min_free_bytes: %u\n",
         (unsigned)health.heap_internal_min_free_bytes);
  printf("heap_internal_min_largest_free_block_bytes: %u\n",
         (unsigned)health.heap_internal_min_largest_free_block_bytes);
  printf("heap_internal_frag_percent: %u\n",
         (unsigned)health.heap_internal_frag_percent);
  printf("heap_internal_warn: %s\n", health.heap_internal_warn ? "yes" : "no");
  printf("heap_internal_crit: %s\n", health.heap_internal_crit ? "yes" : "no");
#if CONFIG_APP_HEAP_PSRAM_MONITOR_ENABLE
  printf("heap_psram_free_bytes: %u\n", (unsigned)health.heap_psram_free_bytes);
  printf("heap_psram_largest_free_block_bytes: %u\n",
         (unsigned)health.heap_psram_largest_free_block_bytes);
  printf("heap_psram_min_free_bytes: %u\n",
         (unsigned)health.heap_psram_min_free_bytes);
  printf("heap_psram_min_largest_free_block_bytes: %u\n",
         (unsigned)health.heap_psram_min_largest_free_block_bytes);
  printf("heap_psram_frag_percent: %u\n",
         (unsigned)health.heap_psram_frag_percent);
  printf("heap_psram_warn: %s\n", health.heap_psram_warn ? "yes" : "no");
  printf("heap_psram_crit: %s\n", health.heap_psram_crit ? "yes" : "no");
#endif
#endif
  printf("allocs_since_run: %" PRIu64 "\n", MemGuardGetAllocCountSinceRun());
  printf("mesh_connected: %s\n",
         MeshTransportIsConnected(g_runtime->mesh) ? "yes" : "no");
  const bool wifi_connected = WifiManagerIsConnected();
  printf("wifi_connected: %s\n", wifi_connected ? "yes" : "no");
  if (wifi_connected) {
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    if (WifiManagerGetIpInfo(&ip_info) == ESP_OK) {
      char ip[16] = { 0 };
      esp_ip4addr_ntoa(&ip_info.ip, ip, sizeof(ip));
      printf("wifi_ip: %s\n", ip);
    } else {
      printf("wifi_ip: n/a\n");
    }
  } else {
    printf("wifi_ip: n/a\n");
  }
  const bool calibration_applied =
    (g_runtime->settings->calibration_points_count > 0u) &&
    g_runtime->settings->calibration.is_valid && !(*g_runtime->cal_overdue) &&
    !(*g_runtime->cal_due_check_suspended);
  printf("calibration_applied: %s (points=%u)\n",
         calibration_applied ? "yes" : "no",
         (unsigned)g_runtime->settings->calibration_points_count);
  return 0;
}

/**
 * @brief Execute CommandRtd.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRtd(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: rtd show | rtd ema show | rtd ema on|off | rtd ema alpha "
           "<0.0..1.0> | rtd fault [assert_ms clear_ms]\n");
    return 1;
  }

  app_settings_t* settings = g_runtime->settings;
  const char* action = argv[1];

  if (strcmp(action, "show") == 0) {
    PrintRtdEmaSettings(settings, g_runtime->sensor);
    return 0;
  }

  if (strcmp(action, "fault") == 0) {
    if (argc == 2) {
      PrintRtdFaultSettings(settings, RuntimeGetState());
      return 0;
    }
    if (argc != 4) {
      printf("usage: rtd fault [assert_ms clear_ms]\n");
      return 1;
    }
    char* end = NULL;
    const uint32_t max_ms = 60000u;
    long long assert_value = strtoll(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0' || assert_value < 0) {
      printf("assert_ms must be a non-negative integer\n");
      return 1;
    }
    long long clear_value = strtoll(argv[3], &end, 10);
    if (end == argv[3] || *end != '\0' || clear_value < 0) {
      printf("clear_ms must be a non-negative integer\n");
      return 1;
    }
    uint32_t assert_ms = (uint32_t)assert_value;
    uint32_t clear_ms = (uint32_t)clear_value;
    if (assert_value > (long long)max_ms) {
      assert_ms = max_ms;
      printf("assert_ms clamped to %u\n", (unsigned)max_ms);
    }
    if (clear_value > (long long)max_ms) {
      clear_ms = max_ms;
      printf("clear_ms clamped to %u\n", (unsigned)max_ms);
    }
    settings->rtd_fault_assert_ms = assert_ms;
    settings->rtd_fault_clear_ms = clear_ms;
    const esp_err_t result =
      AppSettingsSaveRtdFaultDebounceMs(assert_ms, clear_ms);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("rtd_fault_assert_ms set to %u\n", (unsigned)assert_ms);
    printf("rtd_fault_clear_ms set to %u\n", (unsigned)clear_ms);
    return 0;
  }

  if (strcmp(action, "ema") != 0) {
    printf("usage: rtd show | rtd ema show | rtd ema on|off | rtd ema alpha "
           "<0.0..1.0> | rtd fault [assert_ms clear_ms]\n");
    return 1;
  }

  if (argc < 3) {
    printf("usage: rtd ema show | rtd ema on|off | rtd ema alpha <0.0..1.0>\n");
    return 1;
  }

  const char* subaction = argv[2];
  if (strcmp(subaction, "show") == 0) {
    PrintRtdEmaSettings(settings, g_runtime->sensor);
    return 0;
  }

  if (strcmp(subaction, "on") == 0 || strcmp(subaction, "off") == 0) {
    if (argc != 3) {
      printf("usage: rtd ema on|off\n");
      return 1;
    }
    const bool enabled = (strcmp(subaction, "on") == 0);
    settings->rtd_ema_enabled = enabled;
    esp_err_t result = AppSettingsSaveRtdEmaEnabled(enabled);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("rtd_ema_enabled set to %s\n", enabled ? "on" : "off");
    return 0;
  }

  if (strcmp(subaction, "alpha") == 0) {
    if (argc != 4) {
      printf("usage: rtd ema alpha <0.0..1.0>\n");
      return 1;
    }
    char* end = NULL;
    const double value = strtod(argv[3], &end);
    if (end == argv[3] || *end != '\0' || value <= 0.0 || value > 1.0) {
      printf("usage: rtd ema alpha <0.0..1.0>\n");
      return 1;
    }
    long long permille = llround(value * 1000.0);
    if (permille < 1) {
      permille = 1;
    } else if (permille > 1000) {
      permille = 1000;
    }
    const uint16_t permille_u16 = (uint16_t)permille;
    settings->rtd_ema_alpha_permille = permille_u16;
    esp_err_t result = AppSettingsSaveRtdEmaAlphaPermille(permille_u16);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    char alpha_buffer[8] = { 0 };
    FormatPermille(permille_u16, alpha_buffer, sizeof(alpha_buffer));
    printf("rtd_ema_alpha set to %s\n", alpha_buffer);
    return 0;
  }

  printf("usage: rtd show | rtd ema show | rtd ema on|off | rtd ema alpha "
         "<0.0..1.0> | rtd fault [assert_ms clear_ms]\n");
  return 1;
}

static void
PrintStackUsage(void)
{
  printf("usage: stack [show] [--headroom BYTES]\n");
  printf("       Reports monitored tasks from a static registry; includes\n");
  printf("       alloc/min_free/peak/recommended history.\n");
  printf("       recommended = round_up(peak_used + headroom, 256).\n");
}

/**
 * @brief Execute CommandStack.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandStack(int argc, char** argv)
{
  uint32_t headroom_bytes = 1024;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "show") == 0) {
      continue;
    }
    if (strcmp(argv[i], "--headroom") == 0) {
      if ((i + 1) >= argc) {
        printf("--headroom requires a byte count\n");
        PrintStackUsage();
        return 2;
      }
      headroom_bytes = (uint32_t)strtoul(argv[++i], NULL, 10);
      continue;
    }

    printf("unknown option: %s\n", argv[i]);
    PrintStackUsage();
    return 2;
  }

  RuntimePrintStackMonitor(headroom_bytes);
  return 0;
}

/**
 * @brief Execute CommandDisplay.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandDisplay(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: disp show | disp units C|F | disp interval <ms> | disp attn "
           "... | disp test [ms]\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "show") == 0) {
    const spi_host_device_t display_host = RuntimeGetDisplaySpiHost();
    const int display_host_id = (display_host == SPI3_HOST) ? 3 : 2;
#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
    const bool display_shared = true;
#else
    const bool display_shared = false;
#endif
#if CONFIG_APP_MAX7219_ENABLE
    const bool display_enabled = true;
#else
    const bool display_enabled = false;
#endif
    const app_display_units_t effective_units =
      RuntimeGetEffectiveDisplayUnits();
    printf("display_units: %s (effective: %s)\n",
           AppSettingsDisplayUnitsToString(g_runtime->settings->display_units),
           AppSettingsDisplayUnitsToString(effective_units));
    printf("display_sample_period_ms: %u\n",
           (unsigned)g_runtime->settings->display_sample_period_ms);
    printf("sensor_poll_period_ms (effective): %u\n",
           (unsigned)ConsoleEffectiveSensorPollPeriodMs(g_runtime->settings));
    printf("max7219_enabled: %s\n", display_enabled ? "yes" : "no");
    printf("max7219_spi_host: %d%s\n",
           display_host_id,
           display_shared ? " (shared)" : "");
    printf("max7219_chain_len: %d\n", CONFIG_APP_MAX7219_CHAIN_LEN);
    printf("max7219_intensity: %d\n", CONFIG_APP_MAX7219_INTENSITY);
    printf("max7219_pins: mosi=%d sclk=%d cs=%d%s\n",
           RuntimeGetDisplayMosiGpio(),
           RuntimeGetDisplaySclkGpio(),
           RuntimeGetDisplayCsGpio(),
           display_shared ? " (shared)" : "");
    PrintDisplayAttentionPolicy(AppSettingsGetDisplayAttentionPolicy());
    return 0;
  }

  if (strcmp(action, "units") == 0) {
    if (argc < 3) {
      printf("usage: disp units C|F\n");
      return 1;
    }
    app_display_units_t units = APP_DISPLAY_UNITS_F;
    if (!AppSettingsParseDisplayUnits(argv[2], &units)) {
      printf("usage: disp units C|F\n");
      return 1;
    }
    if (!UnitsGpioSetTemporaryUnits(units)) {
      printf("failed to set temporary units\n");
      return 1;
    }
    printf("display_units temporary override set to %s\n",
           AppSettingsDisplayUnitsToString(units));
    return 0;
  }

  if (strcmp(action, "interval") == 0) {
    if (argc != 3) {
      printf("usage: disp interval <ms>\n");
      return 1;
    }
    char* end = NULL;
    long interval_ms_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid interval\n");
      return 1;
    }
    if (interval_ms_long < 100 || interval_ms_long > 3600000) {
      printf("invalid interval\n");
      return 1;
    }

    const uint32_t interval_ms = (uint32_t)interval_ms_long;
    g_runtime->settings->display_sample_period_ms = interval_ms;
    esp_err_t result = AppSettingsSaveDisplaySamplePeriodMs(interval_ms);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    RuntimeNudgeSensorTask();
    printf("display_sample_period_ms set to %u\n", (unsigned)interval_ms);
    return 0;
  }

  if (strcmp(action, "attn") == 0) {
    if (argc < 3) {
      printf("usage: disp attn show|set|defaults|ack\n");
      return 1;
    }
    const char* attn_action = argv[2];
    if (strcmp(attn_action, "show") == 0) {
      PrintDisplayAttentionPolicy(AppSettingsGetDisplayAttentionPolicy());
      return 0;
    }
    if (strcmp(attn_action, "set") == 0) {
      if (argc < 5) {
        printf("usage: disp attn set <name> <off|warn|error>\n");
        return 1;
      }
      display_attention_item_t item = kDispAttnItemSdOut;
      if (!ParseDisplayAttentionName(argv[3], &item)) {
        printf(
          "unknown name. valid: sdout, sdio, framovr, rtd, time, mesh, heap\n");
        return 1;
      }
      display_attention_severity_t severity = DISP_SEV_OFF;
      if (!ParseDisplayAttentionSeverity(argv[4], &severity)) {
        printf("unknown severity. valid: off, warn, error\n");
        return 1;
      }
      uint32_t policy = AppSettingsGetDisplayAttentionPolicy();
      policy = DisplayAttentionPolicySet(policy, item, severity);
      return SaveDisplayAttentionPolicy(policy);
    }
    if (strcmp(attn_action, "defaults") == 0) {
      const uint32_t policy = AppSettingsDefaultDisplayAttentionPolicy();
      return SaveDisplayAttentionPolicy(policy);
    }
    if (strcmp(attn_action, "ack") == 0) {
      if (RuntimeAcknowledgeDisplayAttention(kDispAttnItemFramOvr)) {
        printf("OK\n");
        return 0;
      }
      printf("ack failed\n");
      return 1;
    }
    printf("unknown action. usage: disp attn show|set|defaults|ack\n");
    return 1;
  }

  if (strcmp(action, "test") == 0) {
    uint32_t duration_ms = 2000u;
    if (argc >= 3) {
      char* end = NULL;
      unsigned long parsed = strtoul(argv[2], &end, 10);
      if (end == argv[2] || *end != '\0') {
        printf("usage: disp test [ms]\n");
        return 1;
      }
      duration_ms = (uint32_t)parsed;
    }
    esp_err_t result = RuntimeShowDisplayTestPattern(duration_ms);
    if (result != ESP_OK) {
      printf("display test failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("display test pattern for %u ms\n", (unsigned)duration_ms);
    return 0;
  }

  printf("unknown action. usage: disp show | disp units C|F | disp interval "
         "<ms> | disp attn ... | disp "
         "test [ms]\n");
  return 1;
}

static void
PrintUnitsUsage(void)
{
  printf("usage: units set C|F\n");
}

/**
 * @brief Execute CommandUnits.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandUnits(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    PrintUnitsUsage();
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "set") == 0) {
    if (argc < 3) {
      PrintUnitsUsage();
      return 1;
    }
    app_display_units_t units = APP_DISPLAY_UNITS_F;
    if (!AppSettingsParseDisplayUnits(argv[2], &units)) {
      PrintUnitsUsage();
      return 1;
    }
    if (!UnitsGpioSetTemporaryUnits(units)) {
      printf("failed to set temporary units\n");
      return 1;
    }
    printf("display_units temporary override set to %s\n",
           AppSettingsDisplayUnitsToString(units));
    return 0;
  }

  PrintUnitsUsage();
  return 1;
}

/**
 * @brief Execute CommandRaw.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRaw(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  if (g_runtime == NULL) {
    return 1;
  }

  max31865_sample_t sample;
  esp_err_t result = Max31865ReadOnce(g_runtime->sensor, &sample);
  if (result != ESP_OK) {
    printf("read failed: %s\n", esp_err_to_name(result));
    return 1;
  }

  const calibration_domain_t domain = g_runtime->settings->calibration_domain;
  double corrected_resistance_ohm = sample.resistance_ohm;
  if (domain == CAL_DOMAIN_RESISTANCE_OHM) {
    corrected_resistance_ohm = CalibrationModelEvaluateWithPoints(
      &g_runtime->settings->calibration,
      sample.resistance_ohm,
      g_runtime->settings->calibration_points,
      g_runtime->settings->calibration_points_count);
  }
  double calibrated_temp_c = sample.temperature_c;
  if (!EvaluateCalibratedTemperature_(
        sample.temperature_c, sample.resistance_ohm, &calibrated_temp_c)) {
    printf("calibration evaluate failed\n");
    return 1;
  }

  char fault[64] = { 0 };
  Max31865FormatFault(sample.fault_status, fault, sizeof(fault));

  printf("adc_code_15: %u\n", (unsigned)sample.adc_code);
  printf("resistance_ohm: %.3f\n", sample.resistance_ohm);
  if (domain == CAL_DOMAIN_RESISTANCE_OHM) {
    printf("corrected_resistance_ohm: %.3f\n", corrected_resistance_ohm);
  }
  printf("temp_raw_c: %.3f\n", sample.temperature_c);
  printf("temp_cal_c: %.3f\n", calibrated_temp_c);
  printf("fault: %s (0x%02x)\n", fault, (unsigned)sample.fault_status);
  return 0;
}

/**
 * @brief Execute FlushAllRecordsToSd.
 * @param runtime Parameter runtime.
 * @return Return the function result.
 */
static esp_err_t
FlushAllRecordsToSd(app_runtime_t* runtime)
{
  if (runtime->flush_callback == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return runtime->flush_callback(runtime->flush_context);
}

/**
 * @brief Execute FlushOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
FlushOp(app_runtime_t* runtime, void* ctx)
{
  (void)ctx;
  return FlushAllRecordsToSd(runtime);
}

/**
 * @brief Execute FormatRecordFlags.
 * @param flags Parameter flags.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
static void
FormatRecordFlags(uint16_t flags, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  out[0] = '\0';
  const struct
  {
    uint16_t flag;
    const char* name;
  } entries[] = {
    { LOG_RECORD_FLAG_TIME_VALID, "time_valid" },
    { LOG_RECORD_FLAG_CAL_VALID, "cal_valid" },
    { LOG_RECORD_FLAG_SD_ERROR, "sd_error" },
    { LOG_RECORD_FLAG_MESH_CONNECTED, "mesh_connected" },
    { LOG_RECORD_FLAG_SENSOR_FAULT, "sensor_fault" },
    { LOG_RECORD_FLAG_FRAM_FULL, "fram_full" },
    { LOG_RECORD_FLAG_RTD_EMA, "rtd_ema" },
  };

  size_t used = 0;
  bool first = true;
  for (size_t i = 0; i < sizeof(entries) / sizeof(entries[0]); ++i) {
    if ((flags & entries[i].flag) == 0) {
      continue;
    }
    const char* separator = first ? "" : ",";
    const int written =
      snprintf(out + used, out_size - used, "%s%s", separator, entries[i].name);
    if (written < 0 || (size_t)written >= out_size - used) {
      out[out_size - 1] = '\0';
      return;
    }
    used += (size_t)written;
    first = false;
  }

  if (first) {
    snprintf(out, out_size, "none");
  }
}

/**
 * @brief Execute CommandFlush.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandFlush(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }

  bool async = false;
  int32_t timeout_ms = kFlushTimeoutDefaultMs;
  for (int index = 1; index < argc; ++index) {
    const char* arg = argv[index];
    if (strcmp(arg, "--async") == 0) {
      async = true;
      continue;
    }
    if (strcmp(arg, "--timeout") == 0) {
      if ((index + 1) >= argc) {
        printf("usage: flush [--async] [--timeout <ms>]\n");
        return 1;
      }
      char* end = NULL;
      const long timeout_long = strtol(argv[index + 1], &end, 10);
      if (end == argv[index + 1] || *end != '\0') {
        printf("invalid timeout\n");
        return 1;
      }
      timeout_ms = (int32_t)timeout_long;
      if (timeout_ms < 0) {
        printf("invalid timeout\n");
        return 1;
      }
      index++;
      continue;
    }
    printf("usage: flush [--async] [--timeout <ms>]\n");
    return 1;
  }

  if (RuntimeIsRunning()) {
    runtime_state_t* state = RuntimeGetState();
    if (state == NULL || state->sd_flush_task == NULL) {
      printf("flush unavailable; SD flush task not running\n");
      return 1;
    }
    const TickType_t now_ticks = xTaskGetTickCount();
    const TickType_t deadline_ticks =
      (timeout_ms > 0) ? (now_ticks + pdMS_TO_TICKS(timeout_ms)) : 0;
    state->sd_manual_drain_active = true;
    state->sd_manual_drain_deadline_ticks = deadline_ticks;
    state->sd_manual_drain_passes = 0;
    state->sd_flush_pending = true;
    state->sd_next_flush_allowed_ticks = 0;
    xTaskNotifyGive(state->sd_flush_task);

    if (async) {
      printf("flush armed (timeout=%d ms)\n", (int)timeout_ms);
      return 0;
    }

    uint32_t remaining = 0;
    while (true) {
      if (ConsoleFramLogLock(state)) {
        remaining = FramLogGetBufferedRecords(&state->fram_log);
        ConsoleFramLogUnlock(state);
      }

      if (remaining == 0u) {
        break;
      }
      if (!state->sd_manual_drain_active) {
        break;
      }
      if (deadline_ticks != 0 &&
          (int32_t)(xTaskGetTickCount() - deadline_ticks) >= 0) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (remaining == 0u) {
      printf("flush complete; remaining=0\n");
      return 0;
    }
    if (deadline_ticks != 0 &&
        (int32_t)(xTaskGetTickCount() - deadline_ticks) >= 0) {
      printf("flush timed out; remaining=%u\n", (unsigned)remaining);
      return 1;
    }
    printf("flush stopped; remaining=%u\n", (unsigned)remaining);
    return 1;
  }

  if (async) {
    printf("flush async unsupported while stopped\n");
    return 1;
  }

  const esp_err_t result = RuntimeWithTemporarySdMount(&FlushOp, NULL);
  if (result != ESP_OK) {
    printf("flush failed: %s\n", esp_err_to_name(result));
    return 1;
  }
  return 0;
}

/**
 * @brief Execute CommandFram.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandFram(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: fram status | fram show | fram clear\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "status") != 0 && strcmp(action, "show") != 0 &&
      strcmp(action, "clear") != 0) {
    printf(
      "unknown fram command. try 'fram status | fram show | fram clear'\n");
    return 1;
  }

  runtime_state_t* state = RuntimeGetState();
  if (!ConsoleFramLogLock(state)) {
    printf("fram: busy (lock timeout)\n");
    return 1;
  }
  fram_log_status_t status;
  esp_err_t result = FramLogGetStatus(g_runtime->fram_log, &status);
  ConsoleFramLogUnlock(state);
  if (result != ESP_OK) {
    printf("fram: not initialized\n");
    return 1;
  }
  status.flush_watermark_records =
    g_runtime->settings->fram_flush_watermark_records;

  printf("fram: mounted=%s full=%s\n",
         status.mounted ? "yes" : "no",
         status.full ? "yes" : "no");
  printf("fram: cap=%u rec_size=%u watermark=%u\n",
         (unsigned)status.capacity_records,
         (unsigned)status.record_size_bytes,
         (unsigned)status.flush_watermark_records);
  printf("fram: write=%u read=%u count=%u seq=%u id=%" PRIu64 "\n",
         (unsigned)status.write_index_abs,
         (unsigned)status.read_index_abs,
         (unsigned)status.buffered_count,
         (unsigned)status.next_sequence,
         status.next_record_id);
  if (strcmp(action, "clear") == 0) {
    if (!ConsoleFramLogLock(state)) {
      printf("fram clear skipped: lock timeout\n");
      return 1;
    }
    result = FramLogReset(g_runtime->fram_log);
    ConsoleFramLogUnlock(state);
    if (result != ESP_OK) {
      printf("fram clear failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    if (g_runtime->fram_full != NULL) {
      *g_runtime->fram_full = false;
    }
    printf("fram: cleared\n");
    return 0;
  }
  if (strcmp(action, "show") == 0) {
    if (!ConsoleFramLogLock(state)) {
      printf("fram show skipped: lock timeout\n");
      return 1;
    }
    const uint32_t buffered = FramLogGetBufferedRecords(g_runtime->fram_log);
    const uint64_t last_sd_id = SdLoggerLastRecordIdOnSd(g_runtime->sd_logger);
    printf("fram: buffered=%u last_sd_record_id=%" PRIu64 "\n",
           (unsigned)buffered,
           last_sd_id);
    for (uint32_t offset = 0; offset < buffered; ++offset) {
      log_record_t record;
      const esp_err_t peek_result =
        FramLogPeekOffset(g_runtime->fram_log, offset, &record);
      if (peek_result == ESP_ERR_NOT_FOUND) {
        break;
      }
      const bool corrupted = (peek_result == ESP_ERR_INVALID_RESPONSE);
      if (peek_result != ESP_OK && !corrupted) {
        printf("fram show failed at offset %u: %s\n",
               (unsigned)offset,
               esp_err_to_name(peek_result));
        ConsoleFramLogUnlock(state);
        return 1;
      }

      if (corrupted) {
        uint16_t actual_crc = 0;
        const fram_log_validate_result_t validate_result =
          FramLogValidateRecord(&record, &actual_crc);
        printf(
          "record[%u]: corrupt=yes reason=%s exp_crc=0x%04x act_crc=0x%04x\n",
          (unsigned)offset,
          FramLogValidateResultToString(validate_result),
          (unsigned)record.crc16_ccitt,
          (unsigned)actual_crc);
        continue;
      }

      char time_string[32] = "unknown";
      if (record.timestamp_epoch_sec > 0) {
        const time_t epoch = (time_t)record.timestamp_epoch_sec;
        FormatFileTime(&epoch, time_string, sizeof(time_string));
      }

      char flags_string[128];
      FormatRecordFlags(record.flags, flags_string, sizeof(flags_string));
      const bool pending = record.record_id > last_sd_id;

      printf("record[%u]: id=%" PRIu64 " seq=%u pending=%s corrupt=no\n",
             (unsigned)offset,
             record.record_id,
             (unsigned)record.sequence,
             pending ? "yes" : "no");
      printf("  time: %s.%03d epoch=%" PRIi64 "\n",
             time_string,
             (int)record.timestamp_millis,
             record.timestamp_epoch_sec);
      printf("  temp: raw=%.3fC cal=%.3fC resistance=%.3f ohm\n",
             record.raw_temp_milli_c / 1000.0,
             record.temp_milli_c / 1000.0,
             record.resistance_milli_ohm / 1000.0);
      printf("  flags: 0x%04x [%s]\n", (unsigned)record.flags, flags_string);
      printf("  fault_status: 0x%02x\n", (unsigned)record.fault_status);
    }
    ConsoleFramLogUnlock(state);
  }
  return 0;
}

/**
 * @brief Execute PrintSdStatus.
 * @param runtime Parameter runtime.
 */
static void
PrintSdStatus(const app_runtime_t* runtime)
{
  const sd_logger_t* logger = runtime->sd_logger;
  const bool sd_mounted = logger->is_mounted;
  const bool sd_degraded = RuntimeSdIsDegraded();
  const uint32_t sd_fail_count = RuntimeSdFailCount();
  const TickType_t now_ticks = xTaskGetTickCount();
  const uint32_t sd_backoff_until = RuntimeSdBackoffUntilTicks();
  const runtime_cached_status_t* cached_status = RuntimeGetCachedStatus();
  uint32_t sd_backoff_remaining_ms = 0;
  if (sd_backoff_until != 0 && now_ticks < (TickType_t)sd_backoff_until) {
    sd_backoff_remaining_ms =
      (uint32_t)pdTICKS_TO_MS((TickType_t)sd_backoff_until - now_ticks);
  }

  printf("sd_mounted: %s\n", sd_mounted ? "yes" : "no");
  printf("sd_degraded: %s\n", sd_degraded ? "yes" : "no");
  if (cached_status != NULL) {
    printf("deferred_count: %u\n", (unsigned)cached_status->deferred_count);
    printf("deferred_drops: %u\n", (unsigned)cached_status->deferred_drops);
    printf("deferred_active: %s\n",
           cached_status->deferred_active ? "yes" : "no");
    printf("i2c_quiesce_active: %s\n",
           cached_status->i2c_quiesce_active ? "yes" : "no");
  }
  printf("sd_fail_count: %u\n", (unsigned)sd_fail_count);
  printf("sd_backoff_remaining_ms: %u\n", (unsigned)sd_backoff_remaining_ms);
  runtime_state_t* state = RuntimeGetState();
  if (state != NULL) {
    printf("sd_verify_readback: %s\n",
           state->sd_verify_readback ? "on" : "off");
  }
  printf("sd_last_record_id: %" PRIu64 "\n", SdLoggerLastRecordIdOnSd(logger));
  if (sd_mounted) {
    printf("sd_mount_point: %s\n", logger->mount_point);
    if (logger->card != NULL) {
      const uint64_t size_bytes = (uint64_t)logger->card->csd.capacity *
                                  (uint64_t)logger->card->csd.sector_size;
      printf("sd_card_name: %s\n", logger->card->cid.name);
      printf("sd_card_size_mb: %llu\n",
             (unsigned long long)(size_bytes / (1024ULL * 1024ULL)));
    }
  }
}

/**
 * @brief Execute FormatFileTime.
 * @param timestamp Parameter timestamp.
 * @param buffer Parameter buffer.
 * @param buffer_size Parameter buffer_size.
 */
static void
FormatFileTime(const time_t* timestamp, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  struct tm time_info;
  if (timestamp == NULL || gmtime_r(timestamp, &time_info) == NULL) {
    buffer[0] = '\0';
    return;
  }
  strftime(buffer, buffer_size, "%Y-%m-%d %H:%M:%SZ", &time_info);
}

/**
 * @brief Execute CommandSdView.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
static int
CommandSdView(const sd_logger_t* logger)
{
  if (!logger->is_mounted) {
    printf("sd not mounted\n");
    return 1;
  }

  runtime_state_t* state = RuntimeGetState();
  if (state == NULL || !RuntimeSdIoLock(state, pdMS_TO_TICKS(2000))) {
    printf("sd view failed: sd io lock timeout\n");
    return 1;
  }

  if (!RuntimeSpiBusLockForSharedDevices(state, pdMS_TO_TICKS(2000))) {
    RuntimeSdIoUnlock(state);
    printf("sd view failed: spi bus lock timeout\n");
    return 1;
  }

  DIR* dir = opendir(logger->mount_point);
  if (dir == NULL) {
    RuntimeSpiBusUnlockForSharedDevices(state);
    RuntimeSdIoUnlock(state);
    printf("sd view failed: %s\n", strerror(errno));
    return 1;
  }

  printf("sd files in %s:\n", logger->mount_point);
  struct dirent* entry = NULL;
  int file_count = 0;
  while ((entry = readdir(dir)) != NULL) {
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
      continue;
    }
    // FATFS long file names may be up to 255 chars. Include mount point, '/',
    // and NUL terminator. Keep this comfortably above worst-case.
    char path[512];
    int path_length =
      snprintf(path, sizeof(path), "%s/%s", logger->mount_point, entry->d_name);
    if (path_length < 0 || path_length >= (int)sizeof(path)) {
      printf("  %s (path too long)\n", entry->d_name);
      continue;
    }
    struct stat info;
    if (stat(path, &info) != 0) {
      printf("  %s (stat failed: %s)\n", entry->d_name, strerror(errno));
      continue;
    }
    char time_buffer[32];
    FormatFileTime(&info.st_mtime, time_buffer, sizeof(time_buffer));
    const char* kind = S_ISDIR(info.st_mode) ? "dir" : "file";
    printf("  %s %-4s size=%llu mtime=%s\n",
           entry->d_name,
           kind,
           (unsigned long long)info.st_size,
           time_buffer);
    ++file_count;
  }
  closedir(dir);
  RuntimeSpiBusUnlockForSharedDevices(state);
  RuntimeSdIoUnlock(state);
  printf("sd file count: %d\n", file_count);
  return 0;
}

/**
 * @brief Execute SdStatusOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdStatusOp(app_runtime_t* runtime, void* ctx)
{
  (void)ctx;
  PrintSdStatus(runtime);
  return ESP_OK;
}

/**
 * @brief Execute SdViewOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdViewOp(app_runtime_t* runtime, void* ctx)
{
  (void)ctx;
  return (CommandSdView(runtime->sd_logger) == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Execute SdFormatOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdFormatOp(app_runtime_t* runtime, void* ctx)
{
  (void)ctx;
  runtime_state_t* state = RuntimeGetState();
  if (state == NULL || !RuntimeSdIoLock(state, pdMS_TO_TICKS(2000))) {
    return ESP_ERR_TIMEOUT;
  }
  if (!RuntimeSpiBusLockForSharedDevices(state, pdMS_TO_TICKS(2000))) {
    RuntimeSdIoUnlock(state);
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t result = SdLoggerFormatDestructive(runtime->sd_logger);
  RuntimeSpiBusUnlockForSharedDevices(state);
  RuntimeSdIoUnlock(state);
  return result;
}

/**
 * @brief Execute SdMountOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdMountOp(app_runtime_t* runtime, void* ctx)
{
  (void)runtime;
  (void)ctx;
  return ESP_OK;
}

/**
 * @brief Execute CommandSd.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandSd(int argc, char** argv)
{
  if (g_runtime == NULL || g_runtime->sd_logger == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: sd status|mount|unmount|format|view|verify\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "status") == 0) {
    if (!heap_caps_check_integrity_all(true)) {
      printf("heap integrity check failed (heap corrupted)\r\n");
      return 1;
    }

    if (!RuntimeIsRunning()) {
      esp_err_t result = RuntimeWithTemporarySdMount(&SdStatusOp, NULL);
      if (result != ESP_OK) {
        printf("sd status failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      return 0;
    }
    PrintSdStatus(g_runtime);
    return 0;
  }

  if (strcmp(action, "view") == 0) {
    if (!RuntimeIsRunning()) {
      esp_err_t result = RuntimeWithTemporarySdMount(&SdViewOp, NULL);
      if (result != ESP_OK) {
        printf("sd view failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      return 0;
    }
    return CommandSdView(g_runtime->sd_logger);
  }

  if (strcmp(action, "verify") == 0) {
    runtime_state_t* state = RuntimeGetState();
    if (state == NULL) {
      printf("sd verify failed: runtime unavailable\n");
      return 1;
    }
    if (argc < 3) {
      printf("sd verify readback: %s\n",
             state->sd_verify_readback ? "on" : "off");
      return 0;
    }
    const char* value = argv[2];
    if (strcmp(value, "on") == 0) {
      const esp_err_t save_result = AppSettingsSaveSdVerifyReadback(true);
      if (save_result != ESP_OK) {
        printf("sd verify readback save failed: %s\n",
               esp_err_to_name(save_result));
        return 1;
      }
      state->sd_verify_readback = true;
      g_runtime->settings->sd_verify_readback = true;
      printf("sd verify readback: on\n");
      return 0;
    }
    if (strcmp(value, "off") == 0) {
      const esp_err_t save_result = AppSettingsSaveSdVerifyReadback(false);
      if (save_result != ESP_OK) {
        printf("sd verify readback save failed: %s\n",
               esp_err_to_name(save_result));
        return 1;
      }
      state->sd_verify_readback = false;
      g_runtime->settings->sd_verify_readback = false;
      printf("sd verify readback: off\n");
      return 0;
    }
    printf("usage: sd verify\n");
    printf("       sd verify on|off\n");
    return 1;
  }

  if (RuntimeIsRunning()) {
    printf("Stop run mode first: run stop\n");
    return 1;
  }

  if (strcmp(action, "mount") == 0) {
    esp_err_t result = RuntimeWithTemporarySdMount(&SdMountOp, NULL);
    if (result != ESP_OK) {
      printf("sd mount failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd mounted (temporary)\n");
    return 0;
  }

  if (strcmp(action, "unmount") == 0) {
    esp_err_t result = RuntimeSdUnmountNow();
    if (result != ESP_OK) {
      printf("sd unmount failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd unmounted\n");
    return 0;
  }

  if (strcmp(action, "format") == 0) {
    esp_err_t result = RuntimeWithTemporarySdMount(&SdFormatOp, NULL);
    if (result != ESP_OK) {
      printf("sd format failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd format complete (FAT filesystem recreated)\n");
    return 0;
  }

  printf(
    "unknown sd command. try 'sd status|mount|unmount|format|view|verify'\n");
  return 1;
}

// NOTE: The "log" command uses manual argv parsing.
//
// We intentionally do NOT use argtable for this command because argtable's
// positional parsing cannot express "subcommand + single positional value"
// without accidentally binding the value to the wrong field.
//
// Example of the problem with argtable positional arguments:
//   "log flush_period 300000"
// would bind "300000" to the first positional integer field, not the one
// associated with "flush_period".
//
// Manual parsing keeps the CLI simple and predictable.

static struct
{
  struct arg_str* action;
  struct arg_str* mode_value;
  struct arg_end* end;
} g_mode_args;

static struct
{
  struct arg_str* action;
  struct arg_end* end;
} g_data_args;

static struct
{
  struct arg_str* action;
  struct arg_end* end;
} g_run_args;

static struct
{
  struct arg_str* action;
  struct arg_str* posix;
  struct arg_end* end;
} g_tz_args;

static struct
{
  struct arg_str* action;
  struct arg_str* local_time;
  struct arg_int* is_dst;
  struct arg_end* end;
} g_time_args;

static struct
{
  struct arg_str* action;
  struct arg_int* enabled;
  struct arg_end* end;
} g_dst_args;

static struct
{
  struct arg_str* action;
  struct arg_str* role;
  struct arg_end* end;
} g_role_args;

static struct
{
  struct arg_str* action;
  struct arg_str* mode;
  struct arg_end* end;
} g_net_args;

static struct
{
  struct arg_str* action;
  struct arg_int* enabled;
  struct arg_end* end;
} g_children_args;

/**
 * @brief Execute CommandLog.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandLog(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }

  if (argc < 2) {
    printf("usage: log interval <ms> | log watermark <records> | log "
           "flush_period <ms> | log batch <bytes> | log show\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "flush_ms") == 0) {
    // Backwards/typo-friendly alias.
    action = "flush_period";
  }
  if (strcmp(action, "interval") == 0) {
    if (argc != 3) {
      printf("usage: log interval <ms>\n");
      return 1;
    }
    char* end = NULL;
    long interval_ms_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid interval\n");
      return 1;
    }
    const int interval_ms = (int)interval_ms_long;
    if (interval_ms < 100 || interval_ms > 3600000) {
      printf("invalid interval\n");
      return 1;
    }
    g_runtime->settings->log_period_ms = (uint32_t)interval_ms;
    esp_err_t result = AppSettingsSaveLogPeriodMs((uint32_t)interval_ms);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    RuntimeNudgeSensorTask();
    printf("log_period_ms set to %d\n", interval_ms);
    return 0;
  }

  if (strcmp(action, "watermark") == 0) {
    if (argc != 3) {
      printf("usage: log watermark <records>\n");
      return 1;
    }
    char* end = NULL;
    long watermark_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid watermark\n");
      return 1;
    }
    const int watermark = (int)watermark_long;
    if (watermark < 1) {
      printf("invalid watermark\n");
      return 1;
    }
    g_runtime->settings->fram_flush_watermark_records = (uint32_t)watermark;
    esp_err_t result =
      AppSettingsSaveFramFlushWatermarkRecords((uint32_t)watermark);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("fram flush watermark set to %d\n", watermark);
    return 0;
  }

  if (strcmp(action, "flush_period") == 0) {
    if (argc != 3) {
      printf("usage: log flush_period <ms>\n");
      return 1;
    }
    char* end = NULL;
    long period_ms_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid period\n");
      return 1;
    }
    const int period_ms = (int)period_ms_long;
    if (period_ms < 1000) {
      printf("invalid period\n");
      return 1;
    }
    g_runtime->settings->sd_flush_period_ms = (uint32_t)period_ms;
    esp_err_t result = AppSettingsSaveSdFlushPeriodMs((uint32_t)period_ms);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd_flush_period_ms set to %d\n", period_ms);
    return 0;
  }

  if (strcmp(action, "batch") == 0) {
    if (argc != 3) {
      printf("usage: log batch <bytes>\n");
      return 1;
    }
    char* end = NULL;
    long batch_bytes_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid batch size\n");
      return 1;
    }
    const int batch_bytes = (int)batch_bytes_long;
    if (batch_bytes < 4096) {
      printf("invalid batch size\n");
      return 1;
    }
    g_runtime->settings->sd_batch_bytes_target = (uint32_t)batch_bytes;
    esp_err_t result = AppSettingsSaveSdBatchBytes((uint32_t)batch_bytes);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd batch target set to %d bytes\n", batch_bytes);
    return 0;
  }

  if (strcmp(action, "show") == 0) {
    printf("log_period_ms: %u\n", (unsigned)g_runtime->settings->log_period_ms);
    printf("fram_flush_watermark_records: %u\n",
           (unsigned)g_runtime->settings->fram_flush_watermark_records);
    printf("sd_flush_period_ms: %u\n",
           (unsigned)g_runtime->settings->sd_flush_period_ms);
    printf("sd_batch_target_bytes: %u\n",
           (unsigned)g_runtime->settings->sd_batch_bytes_target);
    return 0;
  }

  printf("unknown action. usage: log interval <ms> | log watermark <records> | "
         "log flush_period <ms> | log batch <bytes> | log show\n");
  return 1;
}

/**
 * @brief Execute CommandErrlog.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandErrlog(int argc, char** argv)
{
  runtime_state_t* state = RuntimeGetState();
  if (state != NULL) {
    (void)RuntimeMaybeInitFramErrorLog(state);
  }
  if (g_runtime == NULL || g_runtime->fram_error_log == NULL) {
    printf("runtime not ready\n");
    return 1;
  }
  if (!g_runtime->fram_error_log->initialized) {
    uint64_t pending_active = 0;
    uint64_t pending_resolved = 0;
    if (state != NULL) {
      taskENTER_CRITICAL(&state->errlog_latch_lock);
      pending_active = state->errlog_pending_active_mask;
      pending_resolved = state->errlog_pending_resolved_mask;
      taskEXIT_CRITICAL(&state->errlog_latch_lock);
    }
    printf("FRAM errlog unavailable; pending_latches=0x%016" PRIx64
           "/0x%016" PRIx64 "\n",
           pending_active,
           pending_resolved);
    return 1;
  }
  if (argc < 2) {
    printf(
      "usage: errlog show [--last N] | errlog stats | errlog status | errlog "
      "clear\n");
    return 1;
  }

  const char* action = argv[1];
  fram_error_log_status_t status = { 0 };
  const esp_err_t status_result =
    FramErrorLogGetStatus(g_runtime->fram_error_log, &status);
  if (status_result != ESP_OK) {
    printf("errlog status failed: %s\n", esp_err_to_name(status_result));
    return 1;
  }
  if (strcmp(action, "stats") == 0) {
    if (status.state == FRAM_ERRLOG_STATE_CORRUPT) {
      printf("FRAM errlog corrupt; run `errlog clear` to reinitialize.\n");
      return 1;
    }
    fram_error_log_stats_t stats = { 0 };
    const esp_err_t result =
      FramErrorLogGetStats(g_runtime->fram_error_log, &stats);
    if (result != ESP_OK) {
      printf("errlog stats failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("errlog count=%" PRIu32 " capacity=%" PRIu32 " write_index=%" PRIu32
           "\n",
           stats.count,
           stats.capacity,
           stats.write_index);
    printf("errlog active_bitmap=0x%08" PRIx32 "%08" PRIx32 "\n",
           stats.active_bitmap_high,
           stats.active_bitmap_low);
    return 0;
  }

  if (strcmp(action, "status") == 0) {
    printf("errlog state=%s blank=%s\n",
           FramErrorLogStateToString(status.state),
           status.region_blank ? "yes" : "no");
    printf("header copy0: valid=%s reason=%s\n",
           status.copy0.valid ? "yes" : "no",
           FramErrorLogHeaderReasonToString(status.copy0.reason));
    printf("header copy1: valid=%s reason=%s\n",
           status.copy1.valid ? "yes" : "no",
           FramErrorLogHeaderReasonToString(status.copy1.reason));
    printf("rtc_pending=%" PRIu16 "\n", RuntimeRtcErrlogLatchPendingCount());
    return 0;
  }

  if (strcmp(action, "clear") == 0) {
    const esp_err_t result = FramErrorLogClear(g_runtime->fram_error_log);
    if (result != ESP_OK) {
      printf("errlog clear failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("errlog cleared\n");
    return 0;
  }

  if (strcmp(action, "show") == 0) {
    if (status.state == FRAM_ERRLOG_STATE_CORRUPT) {
      printf("FRAM errlog corrupt; run `errlog clear` to reinitialize.\n");
      printf("header copy0: valid=%s reason=%s\n",
             status.copy0.valid ? "yes" : "no",
             FramErrorLogHeaderReasonToString(status.copy0.reason));
      printf("header copy1: valid=%s reason=%s\n",
             status.copy1.valid ? "yes" : "no",
             FramErrorLogHeaderReasonToString(status.copy1.reason));
      return 0;
    }
    int last = 0;
    for (int i = 2; i < argc; ++i) {
      if (strcmp(argv[i], "--last") == 0 && (i + 1) < argc) {
        last = atoi(argv[++i]);
      } else {
        printf("usage: errlog show [--last N]\n");
        return 1;
      }
    }
    if (last < 0) {
      printf("last must be >= 0\n");
      return 1;
    }
    fram_error_log_stats_t stats = { 0 };
    const esp_err_t stats_result =
      FramErrorLogGetStats(g_runtime->fram_error_log, &stats);
    if (stats_result != ESP_OK) {
      printf("errlog show failed: %s\n", esp_err_to_name(stats_result));
      return 1;
    }
    if (stats.count == 0) {
      printf("errlog empty\n");
      return 0;
    }
    uint32_t start_index = 0;
    if (last > 0 && (uint32_t)last < stats.count) {
      start_index = stats.count - (uint32_t)last;
    }
    printf("errlog entries: showing %" PRIu32 " of %" PRIu32 "\n",
           stats.count - start_index,
           stats.count);
    for (uint32_t i = start_index; i < stats.count; ++i) {
      fram_error_log_entry_t entry = { 0 };
      bool crc_ok = false;
      const esp_err_t result =
        FramErrorLogReadEntry(g_runtime->fram_error_log, i, &entry, &crc_ok);
      if (result != ESP_OK) {
        printf(
          "entry[%" PRIu32 "] read failed: %s\n", i, esp_err_to_name(result));
        continue;
      }
      char utc_buffer[32];
      char local_buffer[32];
      char flags_buffer[32];
      FormatUtcEpochIso8601(entry.epoch_sec, utc_buffer, sizeof(utc_buffer));
      FormatLocalEpochIso8601(
        entry.epoch_sec, local_buffer, sizeof(local_buffer));
      FormatErrlogFlags(entry.flags, flags_buffer, sizeof(flags_buffer));
      printf("entry[%" PRIu32 "]: utc=%s local=%s.%03u code=%u flags=%s "
             "detail0=%" PRId32 " detail1=%" PRId32 " crc=%s\n",
             i,
             utc_buffer,
             local_buffer,
             (unsigned)entry.millis,
             (unsigned)entry.code,
             flags_buffer,
             entry.detail0,
             entry.detail1,
             crc_ok ? "ok" : "bad");
    }
    return 0;
  }

  printf(
    "unknown action. usage: errlog show [--last N] | errlog stats | errlog "
    "status | errlog clear\n");
  return 1;
}

static struct
{
  struct arg_str* action;
  struct arg_dbl* raw_c;
  struct arg_dbl* actual_c;
  struct arg_int* every_ms;
  struct arg_int* seconds;
  struct arg_dbl* stable_stddev_c;
  struct arg_int* min_seconds;
  struct arg_int* timeout_seconds;
  struct arg_dbl* drift_c_per_min;
  struct arg_lit* no_drift_limit;
  struct arg_str* mode;
  struct arg_lit* allow_wide_slope;
  struct arg_end* end;
} g_cal_args;

static bool
CalConsoleOpLock_(TickType_t ticks)
{
  if (g_cal_console_op_mutex == NULL) {
    return false;
  }
  return xSemaphoreTake(g_cal_console_op_mutex, ticks) == pdTRUE;
}

static void
CalConsoleOpUnlock_(void)
{
  if (g_cal_console_op_mutex == NULL) {
    return;
  }
  (void)xSemaphoreGive(g_cal_console_op_mutex);
}

static bool
CalConsoleOpIsActive(void)
{
  bool active = false;
  if (!CalConsoleOpLock_(pdMS_TO_TICKS(50))) {
    return false;
  }
  active = (g_cal_console_op.mode != CAL_CONSOLE_OP_NONE);
  CalConsoleOpUnlock_();
  return active;
}

static esp_err_t
CalConsoleOpStartLive(uint32_t every_ms,
                      int seconds,
                      int live_output_mode,
                      bool drift_notify_armed,
                      double drift_notify_threshold_c_per_min)
{
  if (!CalConsoleOpLock_(pdMS_TO_TICKS(100))) {
    return ESP_ERR_INVALID_STATE;
  }
  if (g_cal_console_op.mode != CAL_CONSOLE_OP_NONE) {
    CalConsoleOpUnlock_();
    return ESP_ERR_INVALID_STATE;
  }
  g_cal_console_op.mode = CAL_CONSOLE_OP_LIVE;
  g_cal_console_op.task_handle = NULL;
  g_cal_console_op.cancel_requested = false;
  g_cal_console_op.print_every_ms = every_ms;
  g_cal_console_op.live_seconds = seconds;
  g_cal_console_op.capture_actual_temp_c = 0.0;
  g_cal_console_op.capture_stable_stddev_c = 0.0;
  g_cal_console_op.capture_drift_limit_enabled = true;
  g_cal_console_op.capture_drift_limit_c_per_min = 0.0;
  g_cal_console_op.capture_drift_limit_source = CAL_DRIFT_LIMIT_SOURCE_DEFAULT;
  g_cal_console_op.capture_min_seconds = 0;
  g_cal_console_op.capture_timeout_seconds = 0;
  g_cal_console_op.capture_armed = false;
  g_cal_console_op.capture_armed_start_us = 0;
  g_cal_console_op.live_drift_notify_armed = drift_notify_armed;
  g_cal_console_op.live_drift_notify_threshold_c_per_min =
    drift_notify_threshold_c_per_min;
  g_cal_console_op.live_output_mode =
    (live_output_mode == (int)CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED)
      ? CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED
      : CAL_CONSOLE_LIVE_OUTPUT_RAW;
  g_cal_console_op.live_drift_under_start_us = -1;
  g_cal_console_op.live_drift_notify_sent = false;
  g_cal_console_op.start_us = esp_timer_get_time();
  g_cal_console_op.last_sample_count = 0;
  g_cal_console_op.start_internal_free =
    heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  g_cal_console_op.start_internal_largest =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  g_cal_console_op.start_psram_free =
    heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_cal_console_op.start_psram_largest =
    heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_cal_console_op.low_internal_free = g_cal_console_op.start_internal_free;
  g_cal_console_op.low_internal_largest =
    g_cal_console_op.start_internal_largest;
  g_cal_console_op.min_stack_free_bytes = UINT32_MAX;
  g_cal_console_op.max_temp_buffer_bytes = 0u;
  g_cal_runtime_last_sample_id = 0u;
  if (!CalHistoryEnsureStorage_()) {
    g_cal_console_op.mode = CAL_CONSOLE_OP_NONE;
    g_cal_console_op.task_handle = NULL;
    CalConsoleOpUnlock_();
    printf("cal live: requires PSRAM-backed metric history\n");
    return ESP_ERR_NO_MEM;
  }
  BaseType_t created = xTaskCreate(&CalConsoleOpTask,
                                   "cal_op",
                                   kCalOpTaskStackBytes,
                                   NULL,
                                   2,
                                   &g_cal_console_op.task_handle);
  if (created != pdPASS) {
    g_cal_console_op.mode = CAL_CONSOLE_OP_NONE;
    g_cal_console_op.task_handle = NULL;
    CalConsoleOpUnlock_();
    return ESP_ERR_NO_MEM;
  }
  const char* mode_label =
    (g_cal_console_op.live_output_mode == CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED)
      ? "cal livecal"
      : "cal live";
  CalLogMemorySnapshot_("start-pre", mode_label);
  CalLogLiveRetainedResources_(mode_label);
  CalConsoleOpUnlock_();
  return ESP_OK;
}

static esp_err_t
CalConsoleOpArmCaptureWhileLive(
  double actual_temp_c,
  double stable_stddev_c,
  bool drift_limit_enabled,
  double drift_limit_c_per_min,
  calibration_drift_limit_source_t drift_limit_source,
  int min_seconds,
  int timeout_seconds,
  bool* replaced_out)
{
  if (replaced_out != NULL) {
    *replaced_out = false;
  }
  if (!CalConsoleOpLock_(pdMS_TO_TICKS(100))) {
    return ESP_ERR_INVALID_STATE;
  }
  if (g_cal_console_op.mode != CAL_CONSOLE_OP_LIVE) {
    CalConsoleOpUnlock_();
    return ESP_ERR_INVALID_STATE;
  }
  if (replaced_out != NULL) {
    *replaced_out = g_cal_console_op.capture_armed;
  }
  g_cal_console_op.capture_actual_temp_c = actual_temp_c;
  g_cal_console_op.capture_stable_stddev_c = stable_stddev_c;
  g_cal_console_op.capture_drift_limit_enabled = drift_limit_enabled;
  g_cal_console_op.capture_drift_limit_c_per_min = drift_limit_c_per_min;
  g_cal_console_op.capture_drift_limit_source = drift_limit_source;
  g_cal_console_op.capture_min_seconds = min_seconds;
  g_cal_console_op.capture_timeout_seconds = timeout_seconds;
  g_cal_console_op.capture_armed = true;
  g_cal_console_op.capture_armed_start_us = esp_timer_get_time();
  CalConsoleOpUnlock_();
  return ESP_OK;
}

static esp_err_t
CalConsoleOpStartCapture(double actual_temp_c,
                         double stable_stddev_c,
                         bool drift_limit_enabled,
                         double drift_limit_c_per_min,
                         calibration_drift_limit_source_t drift_limit_source,
                         int min_seconds,
                         int timeout_seconds)
{
  if (!CalConsoleOpLock_(pdMS_TO_TICKS(100))) {
    return ESP_ERR_INVALID_STATE;
  }
  if (g_cal_console_op.mode != CAL_CONSOLE_OP_NONE) {
    CalConsoleOpUnlock_();
    return ESP_ERR_INVALID_STATE;
  }
  g_cal_console_op.mode = CAL_CONSOLE_OP_CAPTURE;
  g_cal_console_op.task_handle = NULL;
  g_cal_console_op.cancel_requested = false;
  g_cal_console_op.print_every_ms = 1000;
  g_cal_console_op.live_seconds = 0;
  g_cal_console_op.capture_actual_temp_c = actual_temp_c;
  g_cal_console_op.capture_stable_stddev_c = stable_stddev_c;
  g_cal_console_op.capture_drift_limit_enabled = drift_limit_enabled;
  g_cal_console_op.capture_drift_limit_c_per_min = drift_limit_c_per_min;
  g_cal_console_op.capture_drift_limit_source = drift_limit_source;
  g_cal_console_op.capture_min_seconds = min_seconds;
  g_cal_console_op.capture_timeout_seconds = timeout_seconds;
  g_cal_console_op.capture_armed = true;
  g_cal_console_op.capture_armed_start_us = esp_timer_get_time();
  g_cal_console_op.live_drift_notify_armed = false;
  g_cal_console_op.live_drift_notify_threshold_c_per_min = 0.0;
  g_cal_console_op.live_output_mode = CAL_CONSOLE_LIVE_OUTPUT_RAW;
  g_cal_console_op.live_drift_under_start_us = -1;
  g_cal_console_op.live_drift_notify_sent = false;
  g_cal_console_op.start_us = esp_timer_get_time();
  g_cal_console_op.last_sample_count = 0;
  g_cal_console_op.start_internal_free =
    heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  g_cal_console_op.start_internal_largest =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  g_cal_console_op.start_psram_free =
    heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_cal_console_op.start_psram_largest =
    heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_cal_console_op.low_internal_free = g_cal_console_op.start_internal_free;
  g_cal_console_op.low_internal_largest =
    g_cal_console_op.start_internal_largest;
  g_cal_console_op.min_stack_free_bytes = UINT32_MAX;
  g_cal_console_op.max_temp_buffer_bytes = 0u;
  g_cal_runtime_last_sample_id = 0u;
  if (!CalHistoryEnsureStorage_()) {
    g_cal_console_op.mode = CAL_CONSOLE_OP_NONE;
    g_cal_console_op.task_handle = NULL;
    CalConsoleOpUnlock_();
    printf("cal capture: requires PSRAM-backed metric history\n");
    return ESP_ERR_NO_MEM;
  }
  BaseType_t created = xTaskCreate(&CalConsoleOpTask,
                                   "cal_op",
                                   kCalOpTaskStackBytes,
                                   NULL,
                                   2,
                                   &g_cal_console_op.task_handle);
  if (created != pdPASS) {
    g_cal_console_op.mode = CAL_CONSOLE_OP_NONE;
    g_cal_console_op.task_handle = NULL;
    CalConsoleOpUnlock_();
    return ESP_ERR_NO_MEM;
  }
  CalLogMemorySnapshot_("start-pre", "cal capture");
  CalLogLiveRetainedResources_("cal capture");
  CalConsoleOpUnlock_();
  return ESP_OK;
}

static void
CalConsoleOpRequestCancel(void)
{
  if (!CalConsoleOpLock_(pdMS_TO_TICKS(50))) {
    return;
  }
  if (g_cal_console_op.mode != CAL_CONSOLE_OP_NONE) {
    g_cal_console_op.cancel_requested = true;
  }
  CalConsoleOpUnlock_();
}

static void
PrintCalWindowLine_(const char* prefix,
                    const char* timestamp_utc,
                    size_t sample_count,
                    int32_t last_raw_mC,
                    int32_t last_raw_mOhm,
                    int32_t mean_raw_mC,
                    int32_t stddev_mC,
                    double drift_c_per_min,
                    double delta_c,
                    bool drift_target_enabled,
                    const char* drift_eta_to_target_text)
{
  if (prefix == NULL) {
    return;
  }
  const char* ts_text = (timestamp_utc != NULL) ? timestamp_utc : "n/a";
  const char* eta_text =
    (drift_eta_to_target_text != NULL) ? drift_eta_to_target_text : "n/a";
  if (drift_target_enabled) {
    printf("%s: ts=%s n=%u last=%.3fC last_ohm=%.3f mean=%.3fC std=%.3fC "
           "drift=%.3fC/min delta=%.3fC drift_eta_to_target=%s\n",
           prefix,
           ts_text,
           (unsigned)sample_count,
           last_raw_mC / 1000.0,
           last_raw_mOhm / 1000.0,
           mean_raw_mC / 1000.0,
           stddev_mC / 1000.0,
           drift_c_per_min,
           delta_c,
           eta_text);
    return;
  }

  printf("%s: ts=%s n=%u last=%.3fC last_ohm=%.3f mean=%.3fC std=%.3fC "
         "drift=%.3fC/min delta=%.3fC\n",
         prefix,
         ts_text,
         (unsigned)sample_count,
         last_raw_mC / 1000.0,
         last_raw_mOhm / 1000.0,
         mean_raw_mC / 1000.0,
         stddev_mC / 1000.0,
         drift_c_per_min,
         delta_c);
}

static void
PrintCalWindowCalibratedLine_(const char* timestamp_utc,
                              size_t sample_count,
                              int32_t last_raw_mOhm,
                              int32_t mean_raw_mOhm,
                              int32_t stddev_mOhm,
                              double calibrated_temp_c,
                              double mean_calibrated_temp_c,
                              double stddev_calibrated_temp_c,
                              double drift_c_per_min,
                              double delta_c,
                              uint8_t fault_status)
{
  char fault_desc[64] = { 0 };
  Max31865FormatFault(fault_status, fault_desc, sizeof(fault_desc));
  printf("cal livecal: ts=%s n=%u raw_ohm=%.3f std_ohm=%.3f mean_ohm=%.3f "
         "cal_temp=%.3fC std_temp=%.3fC mean_temp=%.3fC drift=%.3fC/min "
         "delta=%.3fC fault=%s (0x%02X)\n",
         (timestamp_utc != NULL) ? timestamp_utc : "n/a",
         (unsigned)sample_count,
         last_raw_mOhm / 1000.0,
         stddev_mOhm / 1000.0,
         mean_raw_mOhm / 1000.0,
         calibrated_temp_c,
         stddev_calibrated_temp_c,
         mean_calibrated_temp_c,
         drift_c_per_min,
         delta_c,
         fault_desc,
         (unsigned)fault_status);
}

static void
PrintCalCaptureWindowLine_(double elapsed_s,
                           size_t sample_count,
                           int32_t last_raw_mC,
                           int32_t last_raw_mOhm,
                           int32_t mean_raw_mC,
                           double stddev_c,
                           double drift_c_per_min,
                           double delta_c,
                           double stable_elapsed_s,
                           int min_seconds,
                           double stable_stddev_c,
                           bool drift_limit_enabled,
                           double drift_limit_c_per_min,
                           calibration_drift_limit_source_t drift_limit_source,
                           double actual_temp_c)
{
  const char* drift_source = DriftLimitSourceToString_(drift_limit_source);
  char drift_limit_buf[40] = { 0 };
  if (drift_limit_enabled) {
    snprintf(drift_limit_buf,
             sizeof(drift_limit_buf),
             "%.3fC/min(%s)",
             drift_limit_c_per_min,
             drift_source);
  } else {
    snprintf(drift_limit_buf, sizeof(drift_limit_buf), "disabled");
  }
  if (sample_count == 0u) {
    printf("cal capture: t=%.1fs n=%u last=n/a last_ohm=n/a mean=%.3fC "
           "std=%.3fC drift=%.3fC/min delta=%.3fC stable=%.1f/%ds thr=%.3fC "
           "drift_thr=%s actual=%.3fC\n",
           elapsed_s,
           (unsigned)sample_count,
           mean_raw_mC / 1000.0,
           stddev_c,
           drift_c_per_min,
           delta_c,
           stable_elapsed_s,
           min_seconds,
           stable_stddev_c,
           drift_limit_buf,
           actual_temp_c);
    return;
  }

  printf("cal capture: t=%.1fs n=%u last=%.3fC last_ohm=%.3f mean=%.3fC "
         "std=%.3fC drift=%.3fC/min delta=%.3fC stable=%.1f/%ds thr=%.3fC "
         "drift_thr=%s actual=%.3fC\n",
         elapsed_s,
         (unsigned)sample_count,
         last_raw_mC / 1000.0,
         last_raw_mOhm / 1000.0,
         mean_raw_mC / 1000.0,
         stddev_c,
         drift_c_per_min,
         delta_c,
         stable_elapsed_s,
         min_seconds,
         stable_stddev_c,
         drift_limit_buf,
         actual_temp_c);
}

static const char*
DriftLimitSourceToString_(calibration_drift_limit_source_t source)
{
  switch (source) {
    case CAL_DRIFT_LIMIT_SOURCE_DEFAULT:
      return "DEFAULT";
    case CAL_DRIFT_LIMIT_SOURCE_USER:
      return "USER";
    case CAL_DRIFT_LIMIT_SOURCE_DISABLED:
      return "DISABLED";
    case CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE:
      return "LEGACY_UNAVAILABLE";
    default:
      return "UNKNOWN";
  }
}

static const char*
FindOptionValue_(int argc, char** argv, const char* name)
{
  if (argv == NULL || name == NULL) {
    return NULL;
  }
  const size_t name_len = strlen(name);
  for (int i = 2; i < argc; ++i) {
    const char* arg = argv[i];
    if (arg == NULL || strncmp(arg, "--", 2) != 0) {
      continue;
    }
    const char* option = arg + 2;
    if (strncmp(option, name, name_len) != 0) {
      continue;
    }
    if (option[name_len] == '=') {
      return option + name_len + 1;
    }
    if (option[name_len] == '\0') {
      if (i + 1 < argc) {
        return argv[i + 1];
      }
      return NULL;
    }
  }
  return NULL;
}

static bool
OptionPresent_(int argc, char** argv, const char* name)
{
  if (argv == NULL || name == NULL) {
    return false;
  }
  const size_t name_len = strlen(name);
  for (int i = 2; i < argc; ++i) {
    const char* arg = argv[i];
    if (arg == NULL || strncmp(arg, "--", 2) != 0) {
      continue;
    }
    const char* option = arg + 2;
    if (strncmp(option, name, name_len) != 0) {
      continue;
    }
    if (option[name_len] == '\0' || option[name_len] == '=') {
      return true;
    }
  }
  return false;
}

static bool
ParseOptionInt_(int argc, char** argv, const char* name, int* value_out)
{
  if (value_out == NULL) {
    return false;
  }
  const char* value = FindOptionValue_(argc, argv, name);
  if (value == NULL) {
    return false;
  }
  char* end = NULL;
  long parsed = strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed < INT_MIN || parsed > INT_MAX) {
    return false;
  }
  *value_out = (int)parsed;
  return true;
}

static bool
NextCalibrationImportTokenSpan_(const char* line_start,
                                size_t line_len,
                                size_t* cursor_inout,
                                const char** token_start_out,
                                size_t* token_len_out)
{
  if (line_start == NULL || cursor_inout == NULL || token_start_out == NULL ||
      token_len_out == NULL) {
    return false;
  }

  size_t cursor = *cursor_inout;
  while (cursor < line_len && isspace((unsigned char)line_start[cursor])) {
    cursor++;
  }
  if (cursor >= line_len) {
    *cursor_inout = cursor;
    return false;
  }

  const char* token_start = line_start + cursor;
  while (cursor < line_len && !isspace((unsigned char)line_start[cursor])) {
    cursor++;
  }

  *token_start_out = token_start;
  *token_len_out = (size_t)((line_start + cursor) - token_start);
  *cursor_inout = cursor;
  return true;
}

static bool
ParseCalibrationImportDoubleSpan_(const char* value_start,
                                  size_t value_len,
                                  double* parsed_out)
{
  if (value_start == NULL || parsed_out == NULL || value_len == 0u) {
    return false;
  }
  if (value_len == 3u && strncmp(value_start, "n/a", 3) == 0) {
    return false;
  }

  // Parse only a bounded numeric token. We intentionally avoid copying the
  // whole status line while still keeping strtod() usage deterministic.
  char numeric_buf[48];
  if (value_len >= sizeof(numeric_buf)) {
    return false;
  }
  memcpy(numeric_buf, value_start, value_len);
  numeric_buf[value_len] = '\0';

  char* end = NULL;
  const double parsed = strtod(numeric_buf, &end);
  if (end == numeric_buf || *end != '\0') {
    return false;
  }
  *parsed_out = parsed;
  return true;
}

static bool
CalibrationImportKeyEquals_(const char* key_start,
                            size_t key_len,
                            const char* expected)
{
  return (key_start != NULL && expected != NULL &&
          strlen(expected) == key_len &&
          strncmp(key_start, expected, key_len) == 0);
}

static bool
ParseCalibrationImportDriftLimitSourceSpan_(
  const char* value_start,
  size_t value_len,
  calibration_drift_limit_source_t* source_out)
{
  if (value_start == NULL || source_out == NULL) {
    return false;
  }
  if (value_len == strlen("DEFAULT") &&
      strncmp(value_start, "DEFAULT", value_len) == 0) {
    *source_out = CAL_DRIFT_LIMIT_SOURCE_DEFAULT;
    return true;
  }
  if (value_len == strlen("USER") &&
      strncmp(value_start, "USER", value_len) == 0) {
    *source_out = CAL_DRIFT_LIMIT_SOURCE_USER;
    return true;
  }
  if (value_len == strlen("DISABLED") &&
      strncmp(value_start, "DISABLED", value_len) == 0) {
    *source_out = CAL_DRIFT_LIMIT_SOURCE_DISABLED;
    return true;
  }
  if (value_len == strlen("LEGACY_UNAVAILABLE") &&
      strncmp(value_start, "LEGACY_UNAVAILABLE", value_len) == 0) {
    *source_out = CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE;
    return true;
  }
  return false;
}

static bool
CalibrationImportTokenLooksLikeDisplayIndex_(const char* token_start,
                                             size_t token_len)
{
  if (token_start == NULL || token_len < 2u ||
      token_start[token_len - 1u] != ':') {
    return false;
  }
  for (size_t i = 0; i + 1u < token_len; ++i) {
    if (!isdigit((unsigned char)token_start[i])) {
      return false;
    }
  }
  return true;
}

static bool
CalibrationImportTokenIsKnownKeyPrefix_(const char* token_start,
                                        size_t token_len)
{
  if (token_start == NULL || token_len == 0u) {
    return false;
  }
  size_t effective_len = token_len;
  while (effective_len > 0u && (token_start[effective_len - 1u] == '"' ||
                                token_start[effective_len - 1u] == '\'')) {
    effective_len--;
  }
  if (effective_len == 0u) {
    return false;
  }
  static const char* kKnownKeys[] = {
    "reference_temp_C",         "ideal_ref_res_Ohm",
    "captured_raw_temp_avg_C",  "captured_raw_temp_stddev_C",
    "captured_raw_res_avg_Ohm", "captured_raw_res_stddev_Ohm",
    "captured_drift_C_per_min", "captured_delta_C",
    "drift_limit_C_per_min",    "drift_limit_source",
    "captured_window_s",        "captured_ema_alpha",
  };
  for (size_t i = 0; i < (sizeof(kKnownKeys) / sizeof(kKnownKeys[0])); ++i) {
    const char* known_key = kKnownKeys[i];
    const size_t known_len = strlen(known_key);
    if (effective_len < known_len &&
        strncmp(token_start, known_key, effective_len) == 0) {
      return true;
    }
  }
  return false;
}

static bool
CalibrationImportTokenLooksMangledKey_(const char* token_start,
                                       size_t token_len)
{
  if (token_start == NULL || token_len == 0u) {
    return false;
  }
  static const char* kLikelyPrefixes[] = {
    "captured_",
    "drift_limit_",
    "reference_",
    "ideal_",
  };
  for (size_t i = 0; i < (sizeof(kLikelyPrefixes) / sizeof(kLikelyPrefixes[0]));
       ++i) {
    const size_t prefix_len = strlen(kLikelyPrefixes[i]);
    if (token_len >= prefix_len &&
        strncmp(token_start, kLikelyPrefixes[i], prefix_len) == 0) {
      return true;
    }
  }
  return false;
}

typedef struct
{
  bool found_reference_temp_c;
  bool found_ideal_ref_res_ohm;
  bool found_captured_raw_temp_avg_c;
  bool found_captured_raw_temp_stddev_c;
  bool found_captured_raw_res_avg_ohm;
  bool found_captured_raw_res_stddev_ohm;
  bool found_captured_drift_c_per_min;
  bool found_captured_delta_c;
  bool found_drift_limit_c_per_min;
  bool found_drift_limit_source;
  bool found_captured_window_s;
  bool found_captured_ema_alpha;
} cal_import_field_presence_t;

static bool
CalibrationImportValueIsNaSpan_(const char* value_start, size_t value_len)
{
  return (value_start != NULL && value_len == 3u &&
          strncmp(value_start, "n/a", 3) == 0);
}

static void
LogCalibrationImportFieldParsed_(const char* field_name,
                                 double parsed_value,
                                 bool available)
{
  if (available) {
    ESP_LOGI(kTag, "cal import debug: field %s=%.6f", field_name, parsed_value);
  } else {
    ESP_LOGI(kTag, "cal import debug: field %s=n/a", field_name);
  }
}

static void
LogCalibrationImportFieldAbsent_(const char* field_name)
{
  ESP_LOGI(kTag, "cal import debug: field %s absent", field_name);
}

static void
LogCalibrationImportFieldParseFailed_(const char* field_name,
                                      const char* value_start,
                                      size_t value_len)
{
  if (field_name == NULL || value_start == NULL) {
    return;
  }
  ESP_LOGI(kTag,
           "cal import debug: field %s parse failed for value '%.*s'",
           field_name,
           (int)value_len,
           value_start);
}

static void
LogCalibrationImportDriftSource_(calibration_drift_limit_source_t source)
{
  ESP_LOGI(kTag,
           "cal import debug: field drift_limit_source=%s",
           DriftLimitSourceToString_(source));
}

static void
LogCalibrationImportSummary_(const calibration_point_t* point)
{
  if (point == NULL) {
    return;
  }
  ESP_LOGI(kTag,
           "cal import debug: summary reference_temp_C=%.3f "
           "captured_raw_res_avg_Ohm=%.3f",
           point->actual_mC / 1000.0,
           point->raw_avg_mOhm / 1000.0);
  ESP_LOGI(
    kTag,
    "cal import debug: summary raw_temp_avg_present=%s "
    "raw_temp_stddev_present=%s raw_res_stddev_present=%s "
    "captured_drift_present=%s captured_delta_present=%s "
    "drift_limit_present=%s drift_source_present=%s "
    "captured_window_present=%s captured_ema_alpha_present=%s",
    (point->raw_avg_mC != INT32_MIN) ? "yes" : "no",
    (point->raw_stddev_mC >= 0) ? "yes" : "no",
    (point->raw_stddev_mOhm >= 0) ? "yes" : "no",
    (point->captured_drift_mC_per_min !=
     CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN)
      ? "yes"
      : "no",
    (point->captured_delta_mC != CAL_CAPTURE_DELTA_UNAVAILABLE_MC) ? "yes"
                                                                   : "no",
    (point->capture_drift_limit_mC_per_min !=
     CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN)
      ? "yes"
      : "no",
    (point->drift_limit_source !=
     (uint8_t)CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE)
      ? "yes"
      : "no",
    (point->captured_window_s != CAL_CAPTURE_WINDOW_S_UNAVAILABLE) ? "yes"
                                                                   : "no",
    (point->captured_ema_alpha_permille !=
     CAL_CAPTURE_EMA_ALPHA_UNAVAILABLE_PERMILLE)
      ? "yes"
      : "no");
}

static void
LogCalibrationImportOptionalAbsence_(const cal_import_field_presence_t* fields)
{
  if (fields == NULL) {
    return;
  }
  if (!fields->found_ideal_ref_res_ohm) {
    LogCalibrationImportFieldAbsent_("ideal_ref_res_Ohm");
  }
  if (!fields->found_captured_raw_temp_avg_c) {
    LogCalibrationImportFieldAbsent_("captured_raw_temp_avg_C");
  }
  if (!fields->found_captured_raw_temp_stddev_c) {
    LogCalibrationImportFieldAbsent_("captured_raw_temp_stddev_C");
  }
  if (!fields->found_captured_raw_res_stddev_ohm) {
    LogCalibrationImportFieldAbsent_("captured_raw_res_stddev_Ohm");
  }
  if (!fields->found_captured_drift_c_per_min) {
    LogCalibrationImportFieldAbsent_("captured_drift_C_per_min");
  }
  if (!fields->found_captured_delta_c) {
    LogCalibrationImportFieldAbsent_("captured_delta_C");
  }
  if (!fields->found_drift_limit_c_per_min) {
    LogCalibrationImportFieldAbsent_("drift_limit_C_per_min");
  }
  if (!fields->found_drift_limit_source) {
    LogCalibrationImportFieldAbsent_("drift_limit_source");
  }
  if (!fields->found_captured_window_s) {
    LogCalibrationImportFieldAbsent_("captured_window_s");
  }
  if (!fields->found_captured_ema_alpha) {
    LogCalibrationImportFieldAbsent_("captured_ema_alpha");
  }
}

static void
ParseCalibrationImportTokenSpan_(const char* token_start,
                                 size_t token_len,
                                 calibration_point_t* point,
                                 bool* have_actual,
                                 bool* have_raw_ohm,
                                 cal_import_field_presence_t* fields,
                                 bool* truncated_payload_out)
{
  if (token_start == NULL || point == NULL || have_actual == NULL ||
      have_raw_ohm == NULL || fields == NULL) {
    return;
  }
  const char* equals = memchr(token_start, '=', token_len);
  if (equals == NULL || equals == token_start ||
      equals == (token_start + token_len - 1u)) {
    if (equals == NULL &&
        !CalibrationImportTokenLooksLikeDisplayIndex_(token_start, token_len) &&
        CalibrationImportTokenIsKnownKeyPrefix_(token_start, token_len)) {
      if (truncated_payload_out != NULL) {
        *truncated_payload_out = true;
      }
      ESP_LOGI(kTag,
               "cal import debug: truncation detected near token '%.*s'",
               (int)token_len,
               token_start);
      return;
    }
    ESP_LOGI(kTag,
             "cal import debug: ignored token '%.*s'",
             (int)token_len,
             token_start);
    return;
  }
  const char* key_start = token_start;
  const size_t key_len = (size_t)(equals - token_start);
  const char* value_start = equals + 1;
  const size_t value_len = (size_t)((token_start + token_len) - value_start);
  const bool value_is_na =
    CalibrationImportValueIsNaSpan_(value_start, value_len);

  if (CalibrationImportKeyEquals_(key_start, key_len, "reference_temp_C")) {
    fields->found_reference_temp_c = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("reference_temp_C", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->actual_mC = (int32_t)llround(parsed * 1000.0);
      *have_actual = true;
      LogCalibrationImportFieldParsed_("reference_temp_C", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "reference_temp_C", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(key_start, key_len, "ideal_ref_res_Ohm")) {
    fields->found_ideal_ref_res_ohm = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("ideal_ref_res_Ohm", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      LogCalibrationImportFieldParsed_("ideal_ref_res_Ohm", parsed, true);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(
        key_start, key_len, "captured_raw_res_avg_Ohm")) {
    fields->found_captured_raw_res_avg_ohm = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("captured_raw_res_avg_Ohm", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->raw_avg_mOhm = (int32_t)llround(parsed * 1000.0);
      *have_raw_ohm = true;
      LogCalibrationImportFieldParsed_(
        "captured_raw_res_avg_Ohm", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "captured_raw_res_avg_Ohm", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(
        key_start, key_len, "captured_raw_temp_avg_C")) {
    fields->found_captured_raw_temp_avg_c = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("captured_raw_temp_avg_C", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->raw_avg_mC = (int32_t)llround(parsed * 1000.0);
      LogCalibrationImportFieldParsed_("captured_raw_temp_avg_C", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "captured_raw_temp_avg_C", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(
        key_start, key_len, "captured_raw_temp_stddev_C")) {
    fields->found_captured_raw_temp_stddev_c = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_(
        "captured_raw_temp_stddev_C", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->raw_stddev_mC = (int32_t)llround(parsed * 1000.0);
      LogCalibrationImportFieldParsed_(
        "captured_raw_temp_stddev_C", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "captured_raw_temp_stddev_C", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(
        key_start, key_len, "captured_raw_res_stddev_Ohm")) {
    fields->found_captured_raw_res_stddev_ohm = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_(
        "captured_raw_res_stddev_Ohm", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->raw_stddev_mOhm = (int32_t)llround(parsed * 1000.0);
      LogCalibrationImportFieldParsed_(
        "captured_raw_res_stddev_Ohm", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "captured_raw_res_stddev_Ohm", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(
        key_start, key_len, "captured_drift_C_per_min")) {
    fields->found_captured_drift_c_per_min = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("captured_drift_C_per_min", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->captured_drift_mC_per_min = (int32_t)llround(parsed * 1000.0);
      LogCalibrationImportFieldParsed_(
        "captured_drift_C_per_min", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "captured_drift_C_per_min", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(key_start, key_len, "captured_delta_C")) {
    fields->found_captured_delta_c = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("captured_delta_C", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->captured_delta_mC = (int32_t)llround(parsed * 1000.0);
      LogCalibrationImportFieldParsed_("captured_delta_C", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "captured_delta_C", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(
        key_start, key_len, "drift_limit_C_per_min")) {
    fields->found_drift_limit_c_per_min = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("drift_limit_C_per_min", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->capture_drift_limit_mC_per_min = (int32_t)llround(parsed * 1000.0);
      LogCalibrationImportFieldParsed_("drift_limit_C_per_min", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "drift_limit_C_per_min", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(key_start, key_len, "drift_limit_source")) {
    fields->found_drift_limit_source = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("drift_limit_source", 0.0, false);
      return;
    }
    calibration_drift_limit_source_t source = CAL_DRIFT_LIMIT_SOURCE_USER;
    if (ParseCalibrationImportDriftLimitSourceSpan_(
          value_start, value_len, &source)) {
      point->drift_limit_source = (uint8_t)source;
      LogCalibrationImportDriftSource_(source);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(key_start, key_len, "captured_window_s")) {
    fields->found_captured_window_s = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("captured_window_s", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->captured_window_s = (int16_t)llround(parsed);
      LogCalibrationImportFieldParsed_("captured_window_s", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "captured_window_s", value_start, value_len);
    }
    return;
  }

  if (CalibrationImportKeyEquals_(key_start, key_len, "captured_ema_alpha")) {
    fields->found_captured_ema_alpha = true;
    if (value_is_na) {
      LogCalibrationImportFieldParsed_("captured_ema_alpha", 0.0, false);
      return;
    }
    double parsed = 0.0;
    if (ParseCalibrationImportDoubleSpan_(value_start, value_len, &parsed)) {
      point->captured_ema_alpha_permille = (int16_t)llround(parsed * 1000.0);
      LogCalibrationImportFieldParsed_("captured_ema_alpha", parsed, true);
    } else {
      LogCalibrationImportFieldParseFailed_(
        "captured_ema_alpha", value_start, value_len);
    }
    return;
  }

  ESP_LOGI(
    kTag, "cal import debug: ignored key '%.*s'", (int)key_len, key_start);
}

static bool
ParseCalibrationImportStatusLineSpan_(const char* line_start,
                                      size_t line_len,
                                      calibration_point_t* point_out,
                                      size_t* token_count_out)
{
  if (line_start == NULL || point_out == NULL) {
    return false;
  }
  calibration_point_t point = { 0 };
  point.raw_avg_mC = INT32_MIN;
  point.raw_stddev_mC = -1;
  point.raw_stddev_mOhm = -1;
  point.captured_drift_mC_per_min = CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN;
  point.captured_delta_mC = CAL_CAPTURE_DELTA_UNAVAILABLE_MC;
  point.capture_drift_limit_mC_per_min =
    CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN;
  point.drift_limit_source = (uint8_t)CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE;
  point.captured_window_s = CAL_CAPTURE_WINDOW_S_UNAVAILABLE;
  point.captured_ema_alpha_permille =
    CAL_CAPTURE_EMA_ALPHA_UNAVAILABLE_PERMILLE;
  bool have_actual = false;
  bool have_raw_ohm = false;
  bool truncated_payload = false;
  cal_import_field_presence_t fields = { 0 };

  // Parse directly from the immutable pre-dispatch command snapshot without
  // mutating it. This keeps quoted console input handling stable while avoiding
  // whole-line
  // temporary copies (only tiny bounded per-token numeric buffers are used).
  size_t cursor = 0u;
  const char* token_start = NULL;
  size_t token_len = 0u;
  size_t token_count = 0u;
  while (NextCalibrationImportTokenSpan_(
    line_start, line_len, &cursor, &token_start, &token_len)) {
    token_count++;
    ParseCalibrationImportTokenSpan_(token_start,
                                     token_len,
                                     &point,
                                     &have_actual,
                                     &have_raw_ohm,
                                     &fields,
                                     &truncated_payload);
    if (truncated_payload) {
      break;
    }
  }

  if (!fields.found_reference_temp_c) {
    LogCalibrationImportFieldAbsent_("reference_temp_C");
  }
  if (!fields.found_captured_raw_res_avg_ohm) {
    LogCalibrationImportFieldAbsent_("captured_raw_res_avg_Ohm");
  }
  LogCalibrationImportOptionalAbsence_(&fields);
  if (truncated_payload) {
    return false;
  }
  if (!have_actual || !have_raw_ohm) {
    return false;
  }
  if (token_count_out != NULL) {
    *token_count_out = token_count;
  }
  LogCalibrationImportSummary_(&point);
  *point_out = point;
  return true;
}

static bool
ParseCalibrationImportStatusTokens_(int argc,
                                    char** argv,
                                    int start_index,
                                    calibration_point_t* point_out,
                                    size_t* token_count_out)
{
  if (argv == NULL || point_out == NULL || start_index < 0 ||
      start_index >= argc) {
    return false;
  }
  calibration_point_t point = { 0 };
  point.raw_avg_mC = INT32_MIN;
  point.raw_stddev_mC = -1;
  point.raw_stddev_mOhm = -1;
  point.captured_drift_mC_per_min = CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN;
  point.captured_delta_mC = CAL_CAPTURE_DELTA_UNAVAILABLE_MC;
  point.capture_drift_limit_mC_per_min =
    CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN;
  point.drift_limit_source = (uint8_t)CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE;
  point.captured_window_s = CAL_CAPTURE_WINDOW_S_UNAVAILABLE;
  point.captured_ema_alpha_permille =
    CAL_CAPTURE_EMA_ALPHA_UNAVAILABLE_PERMILLE;
  bool have_actual = false;
  bool have_raw_ohm = false;
  bool truncated_payload = false;
  size_t token_count = 0u;
  cal_import_field_presence_t fields = { 0 };

  for (int i = start_index; i < argc; ++i) {
    const char* token = argv[i];
    if (token == NULL || token[0] == '\0') {
      continue;
    }
    token_count++;
    ParseCalibrationImportTokenSpan_(token,
                                     strlen(token),
                                     &point,
                                     &have_actual,
                                     &have_raw_ohm,
                                     &fields,
                                     &truncated_payload);
    if (truncated_payload) {
      break;
    }
  }

  if (!fields.found_reference_temp_c) {
    LogCalibrationImportFieldAbsent_("reference_temp_C");
  }
  if (!fields.found_captured_raw_res_avg_ohm) {
    LogCalibrationImportFieldAbsent_("captured_raw_res_avg_Ohm");
  }
  LogCalibrationImportOptionalAbsence_(&fields);
  if (truncated_payload) {
    return false;
  }
  if (!have_actual || !have_raw_ohm) {
    return false;
  }
  if (token_count_out != NULL) {
    *token_count_out = token_count;
  }
  LogCalibrationImportSummary_(&point);
  *point_out = point;
  return true;
}

static bool
CalibrationImportPayloadFromConsoleLine_(const char* action,
                                         const char** payload_start_out,
                                         size_t* payload_len_out,
                                         bool* payload_quoted_out,
                                         bool* unterminated_quote_out)
{
  if (action == NULL || payload_start_out == NULL || payload_len_out == NULL ||
      payload_quoted_out == NULL || unterminated_quote_out == NULL ||
      s_console_cmdline_snapshot == NULL) {
    return false;
  }
  *unterminated_quote_out = false;

  const char* cursor = s_console_cmdline_snapshot;
  while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
    cursor++;
  }
  const char* cmd_start = cursor;
  while (*cursor != '\0' && !isspace((unsigned char)*cursor)) {
    cursor++;
  }
  if ((size_t)(cursor - cmd_start) != 3u || strncmp(cmd_start, "cal", 3) != 0) {
    return false;
  }
  while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
    cursor++;
  }
  const char* action_start = cursor;
  while (*cursor != '\0' && !isspace((unsigned char)*cursor)) {
    cursor++;
  }
  const size_t action_len = (size_t)(cursor - action_start);
  if (action_len != strlen(action) ||
      strncmp(action_start, action, action_len) != 0) {
    return false;
  }

  while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
    cursor++;
  }
  if (*cursor == '\0') {
    return false;
  }

  const char* payload_start = cursor;
  const char* payload_end =
    s_console_cmdline_snapshot + strlen(s_console_cmdline_snapshot);
  while (payload_end > payload_start &&
         isspace((unsigned char)payload_end[-1])) {
    payload_end--;
  }
  if (payload_end <= payload_start) {
    return false;
  }

  bool quoted = false;
  if (payload_start[0] == '"') {
    if ((size_t)(payload_end - payload_start) >= 2u && payload_end[-1] == '"') {
      quoted = true;
    } else {
      *unterminated_quote_out = true;
    }
  }
  if (quoted) {
    payload_start++;
    payload_end--;
  }
  if (payload_end <= payload_start) {
    return false;
  }

  *payload_start_out = payload_start;
  *payload_len_out = (size_t)(payload_end - payload_start);
  *payload_quoted_out = quoted;
  return true;
}

static bool
ParseOptionDouble_(int argc, char** argv, const char* name, double* value_out)
{
  if (value_out == NULL) {
    return false;
  }
  const char* value = FindOptionValue_(argc, argv, name);
  if (value == NULL) {
    return false;
  }
  char* end = NULL;
  double parsed = strtod(value, &end);
  if (end == value || *end != '\0') {
    return false;
  }
  *value_out = parsed;
  return true;
}

static void
PrintCalLiveUsage_(const char* command_name)
{
  if (command_name == NULL) {
    command_name = "live";
  }
  printf(
    "usage: cal %s [seconds] [--every_ms 1000] [--drift_c_per_min 0.020]\n",
    command_name);
}

static bool
ParseCalLiveArguments_(int argc,
                       char** argv,
                       const char* usage,
                       int* every_ms_out,
                       int* seconds_out,
                       bool* drift_notify_armed_out,
                       double* drift_notify_threshold_out)
{
  if (every_ms_out == NULL || seconds_out == NULL ||
      drift_notify_armed_out == NULL || drift_notify_threshold_out == NULL) {
    return false;
  }
  int every_ms = 1000;
  int seconds = 0;
  bool drift_notify_armed = false;
  double drift_notify_threshold_c_per_min = 0.0;
  bool seconds_positional_set = false;
  bool seconds_option_set = false;
  for (int i = 2; i < argc; ++i) {
    const char* arg = argv[i];
    if (arg == NULL) {
      continue;
    }
    if (strncmp(arg, "--", 2) == 0) {
      const char* option = arg + 2;
      const char* value_eq = strchr(option, '=');
      if (value_eq == NULL && (strncmp(option, "every_ms", 8) == 0 ||
                               strncmp(option, "seconds", 7) == 0 ||
                               strncmp(option, "drift_c_per_min", 15) == 0)) {
        if (i + 1 < argc) {
          ++i;
        }
      }
      continue;
    }
    if (seconds_positional_set) {
      PrintCalLiveUsage_(usage);
      return false;
    }
    char* end = NULL;
    long parsed = strtol(arg, &end, 10);
    if (end == arg || *end != '\0' || parsed > INT_MAX || parsed < INT_MIN) {
      PrintCalLiveUsage_(usage);
      return false;
    }
    seconds = (int)parsed;
    seconds_positional_set = true;
  }

  int option_value = 0;
  if (ParseOptionInt_(argc, argv, "every_ms", &option_value)) {
    every_ms = option_value;
  } else if (OptionPresent_(argc, argv, "every_ms")) {
    PrintCalLiveUsage_(usage);
    return false;
  }
  if (ParseOptionInt_(argc, argv, "seconds", &option_value)) {
    seconds = option_value;
    seconds_option_set = true;
  } else if (OptionPresent_(argc, argv, "seconds")) {
    PrintCalLiveUsage_(usage);
    return false;
  }
  double drift_option = 0.0;
  if (ParseOptionDouble_(argc, argv, "drift_c_per_min", &drift_option)) {
    drift_notify_armed = true;
    drift_notify_threshold_c_per_min = fabs(drift_option);
  } else if (OptionPresent_(argc, argv, "drift_c_per_min")) {
    PrintCalLiveUsage_(usage);
    return false;
  }

  if (seconds_positional_set && seconds_option_set) {
    PrintCalLiveUsage_(usage);
    return false;
  }
  if ((every_ms <= 0 || (seconds_positional_set || seconds_option_set)) &&
      (seconds <= 0)) {
    PrintCalLiveUsage_(usage);
    return false;
  }
  if (drift_notify_armed && drift_notify_threshold_c_per_min <= 0.0) {
    PrintCalLiveUsage_(usage);
    return false;
  }

  *every_ms_out = every_ms;
  *seconds_out = seconds;
  *drift_notify_armed_out = drift_notify_armed;
  *drift_notify_threshold_out = drift_notify_threshold_c_per_min;
  return true;
}

static bool
FormatCalLiveTimestampNow_(char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0u) {
    return false;
  }
  if (!TimeSyncIsSystemTimeValid()) {
    snprintf(buffer, buffer_size, "1970-01-01T00:00:00Z(time_invalid)");
    return false;
  }
  const int64_t now_epoch_utc = (int64_t)time(NULL);
  FormatUtcEpochIso8601(now_epoch_utc, buffer, buffer_size);
  return strcmp(buffer, "n/a") != 0;
}

static bool
EvaluateCalibratedTemperature_(double raw_temp_c,
                               double raw_resistance_ohm,
                               double* calibrated_temp_c_out)
{
  if (calibrated_temp_c_out == NULL || g_runtime == NULL ||
      g_runtime->settings == NULL || g_runtime->sensor == NULL) {
    return false;
  }
  const app_settings_t* settings = g_runtime->settings;
  const calibration_domain_t domain = settings->calibration_domain;
  if (domain == CAL_DOMAIN_RESISTANCE_OHM) {
    const double corrected_resistance_ohm =
      CalibrationModelEvaluateWithPoints(&settings->calibration,
                                         raw_resistance_ohm,
                                         settings->calibration_points,
                                         settings->calibration_points_count);
    *calibrated_temp_c_out = Max31865ResistanceToTemperature(
      g_runtime->sensor, corrected_resistance_ohm);
    return true;
  }
  *calibrated_temp_c_out =
    CalibrationModelEvaluateWithPoints(&settings->calibration,
                                       raw_temp_c,
                                       settings->calibration_points,
                                       settings->calibration_points_count);
  return true;
}

static bool
RefreshCalibratedWindowTempStatsCache_(size_t sample_count,
                                       uint32_t window_generation,
                                       int32_t last_raw_mC,
                                       int32_t last_raw_mOhm,
                                       int32_t mean_raw_mC,
                                       int32_t mean_raw_mOhm)
{
  double last_calibrated_temp_c = 0.0;
  double mean_calibrated_temp_c = 0.0;
  double stddev_calibrated_temp_c = 0.0;
  if (sample_count == 0u) {
    CalWindowSetCalibratedTempStats(
      0.0, 0.0, 0.0, true, sample_count, window_generation);
    return true;
  }

  if (!EvaluateCalibratedTemperature_(last_raw_mC / 1000.0,
                                      last_raw_mOhm / 1000.0,
                                      &last_calibrated_temp_c)) {
    CalWindowSetCalibratedTempStats(
      0.0, 0.0, 0.0, false, sample_count, window_generation);
    return false;
  }

  if (!EvaluateCalibratedTemperature_(mean_raw_mC / 1000.0,
                                      mean_raw_mOhm / 1000.0,
                                      &mean_calibrated_temp_c)) {
    CalWindowSetCalibratedTempStats(
      0.0, 0.0, 0.0, false, sample_count, window_generation);
    return false;
  }

  double sum = 0.0;
  double sum_sq = 0.0;
  size_t valid_count = 0u;
  for (size_t i = 0; i < sample_count; ++i) {
    int32_t raw_mC = 0;
    int32_t raw_mOhm = 0;
    if (!CalWindowGetActiveSampleByIndex(i, &raw_mC, &raw_mOhm, NULL)) {
      continue;
    }
    double calibrated_temp_c = 0.0;
    if (!EvaluateCalibratedTemperature_(
          raw_mC / 1000.0, raw_mOhm / 1000.0, &calibrated_temp_c)) {
      continue;
    }
    sum += calibrated_temp_c;
    sum_sq += calibrated_temp_c * calibrated_temp_c;
    ++valid_count;
  }
  if (valid_count == 0u) {
    CalWindowSetCalibratedTempStats(last_calibrated_temp_c,
                                    mean_calibrated_temp_c,
                                    0.0,
                                    true,
                                    sample_count,
                                    window_generation);
    return true;
  }
  const double inv_count = 1.0 / (double)valid_count;
  const double mean = sum * inv_count;
  const double variance = fmax(0.0, (sum_sq * inv_count) - (mean * mean));
  stddev_calibrated_temp_c = sqrt(variance);
  CalWindowSetCalibratedTempStats(last_calibrated_temp_c,
                                  mean_calibrated_temp_c,
                                  stddev_calibrated_temp_c,
                                  true,
                                  sample_count,
                                  window_generation);
  return true;
}

static bool
ParseTempC_(const char* text, double* value_out)
{
  if (text == NULL || value_out == NULL) {
    return false;
  }
  char* end = NULL;
  double parsed = strtod(text, &end);
  if (end == text) {
    return false;
  }
  if (*end == '\0') {
    *value_out = parsed;
    return true;
  }
  if (end[1] == '\0' && (end[0] == 'C' || end[0] == 'c')) {
    *value_out = parsed;
    return true;
  }
  return false;
}

/**
 * @brief Parse positional arguments for `cal add <raw_c> <actual_c>`.
 * @param argc Command argument count.
 * @param argv Command argument values.
 * @param raw_c_out Parsed raw Celsius value.
 * @param actual_c_out Parsed actual/reference Celsius value.
 * @return true when exactly two valid positional Celsius values are present.
 */
static bool
ParseCalAddPositionalArgs_(int argc,
                           char** argv,
                           double* raw_c_out,
                           double* actual_c_out)
{
  if (argv == NULL || raw_c_out == NULL || actual_c_out == NULL) {
    return false;
  }

  const char* raw_text = NULL;
  const char* actual_text = NULL;
  for (int i = 2; i < argc; ++i) {
    const char* arg = argv[i];
    if (arg == NULL) {
      continue;
    }
    if (strncmp(arg, "--", 2) == 0) {
      return false;
    }
    if (raw_text == NULL) {
      raw_text = arg;
      continue;
    }
    if (actual_text == NULL) {
      actual_text = arg;
      continue;
    }
    return false;
  }

  if (raw_text == NULL || actual_text == NULL) {
    return false;
  }

  double raw_c = 0.0;
  double actual_c = 0.0;
  if (!ParseTempC_(raw_text, &raw_c) || !ParseTempC_(actual_text, &actual_c)) {
    return false;
  }

  *raw_c_out = raw_c;
  *actual_c_out = actual_c;
  return true;
}

/**
 * @brief Parse arguments for `cal import <raw_ohm> <actual_c> [options]`.
 * @param argc Command argument count.
 * @param argv Command argument values.
 * @param raw_ohm_out Parsed raw resistance value in ohms.
 * @param actual_c_out Parsed actual/reference Celsius value.
 * @param raw_c_supplied_out True when --raw_c is provided.
 * @param raw_c_out Parsed optional captured raw Celsius average.
 * @param stddev_c_supplied_out True when --stddev_c is provided.
 * @param stddev_c_out Parsed optional captured raw Celsius stddev.
 * @return true when all required/optional values are valid.
 */
static bool
ParseCalImportArgs_(int argc,
                    char** argv,
                    double* raw_ohm_out,
                    double* actual_c_out,
                    bool* raw_c_supplied_out,
                    double* raw_c_out,
                    bool* stddev_c_supplied_out,
                    double* stddev_c_out)
{
  if (argv == NULL || raw_ohm_out == NULL || actual_c_out == NULL ||
      raw_c_supplied_out == NULL || raw_c_out == NULL ||
      stddev_c_supplied_out == NULL || stddev_c_out == NULL) {
    return false;
  }

  const char* positional_args[2] = { 0 };
  size_t positional_count = 0;
  for (int i = 2; i < argc; ++i) {
    const char* arg = argv[i];
    if (arg == NULL) {
      continue;
    }
    if (strncmp(arg, "--", 2) == 0) {
      const char* option = arg + 2;
      const char* option_value = strchr(option, '=');
      const size_t option_name_len = (option_value == NULL)
                                       ? strlen(option)
                                       : (size_t)(option_value - option);
      const bool is_raw_c = (option_name_len == strlen("raw_c")) &&
                            (strncmp(option, "raw_c", option_name_len) == 0);
      const bool is_stddev_c =
        (option_name_len == strlen("stddev_c")) &&
        (strncmp(option, "stddev_c", option_name_len) == 0);
      if (!is_raw_c && !is_stddev_c) {
        return false;
      }
      if (option_value == NULL) {
        if ((i + 1) >= argc) {
          return false;
        }
        ++i;
      }
      continue;
    }

    if (positional_count >= 2u) {
      return false;
    }
    positional_args[positional_count++] = arg;
  }

  if (positional_count != 2u) {
    return false;
  }

  char* end = NULL;
  double parsed_raw_ohm = strtod(positional_args[0], &end);
  if (end == positional_args[0] || *end != '\0' || parsed_raw_ohm <= 0.0) {
    return false;
  }
  double parsed_actual_c = 0.0;
  if (!ParseTempC_(positional_args[1], &parsed_actual_c)) {
    return false;
  }

  bool raw_c_supplied = false;
  bool stddev_c_supplied = false;
  double parsed_raw_c = 0.0;
  double parsed_stddev_c = 0.0;

  if (ParseOptionDouble_(argc, argv, "raw_c", &parsed_raw_c)) {
    raw_c_supplied = true;
  } else if (OptionPresent_(argc, argv, "raw_c")) {
    return false;
  }

  if (ParseOptionDouble_(argc, argv, "stddev_c", &parsed_stddev_c)) {
    if (parsed_stddev_c < 0.0) {
      return false;
    }
    stddev_c_supplied = true;
  } else if (OptionPresent_(argc, argv, "stddev_c")) {
    return false;
  }

  *raw_ohm_out = parsed_raw_ohm;
  *actual_c_out = parsed_actual_c;
  *raw_c_supplied_out = raw_c_supplied;
  *raw_c_out = parsed_raw_c;
  *stddev_c_supplied_out = stddev_c_supplied;
  *stddev_c_out = parsed_stddev_c;
  return true;
}

/**
 * @brief Find point index by exact millidegree actual/reference temperature.
 * @param settings Active settings with calibration points.
 * @param actual_mC Reference temperature in millidegrees Celsius.
 * @return Zero-based index when found; -1 when not found/invalid input.
 */
static int
FindCalibrationPointIndexByActualMc_(const app_settings_t* settings,
                                     int32_t actual_mC)
{
  if (settings == NULL) {
    return -1;
  }
  for (size_t i = 0; i < settings->calibration_points_count; ++i) {
    if (settings->calibration_points[i].actual_mC == actual_mC) {
      return (int)i;
    }
  }
  return -1;
}

/**
 * @brief Save calibration points and domain to NVS.
 * @param settings Active settings.
 * @return ESP_OK on success or error from underlying save routine.
 */
static esp_err_t
SaveCalibrationPointsAndDomain_(app_settings_t* settings)
{
  if (settings == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t result = AppSettingsSaveCalibrationPoints(
    settings->calibration_points, settings->calibration_points_count);
  if (result != ESP_OK) {
    return result;
  }
  return AppSettingsSaveCalibrationDomain(settings->calibration_domain);
}

/**
 * @brief Delete one calibration point by 1-based display index and compact.
 * @param settings Active settings.
 * @param display_index 1-based point index shown in cal list/status.
 * @param deleted_point_out Optional removed point snapshot.
 * @return ESP_OK on success; ESP_ERR_NOT_FOUND for out-of-range index.
 */
static esp_err_t
DeleteCalibrationPointByDisplayIndex_(app_settings_t* settings,
                                      size_t display_index,
                                      calibration_point_t* deleted_point_out)
{
  if (settings == NULL || display_index == 0u) {
    return ESP_ERR_INVALID_ARG;
  }
  if (display_index > settings->calibration_points_count) {
    return ESP_ERR_NOT_FOUND;
  }

  const size_t index = display_index - 1u;
  if (deleted_point_out != NULL) {
    *deleted_point_out = settings->calibration_points[index];
  }

  for (size_t i = index; (i + 1u) < settings->calibration_points_count; ++i) {
    settings->calibration_points[i] = settings->calibration_points[i + 1u];
  }
  settings->calibration_points_count--;
  memset(
    &settings->calibration_points[settings->calibration_points_count],
    0,
    sizeof(settings->calibration_points[settings->calibration_points_count]));
  return ESP_OK;
}

static bool
JoinArgsWithSpaces(int argc,
                   char** argv,
                   int start_index,
                   char* out,
                   size_t out_size)
{
  if (argv == NULL || out == NULL || out_size == 0 || start_index < 0 ||
      start_index >= argc) {
    return false;
  }

  out[0] = '\0';
  size_t used = 0;
  for (int i = start_index; i < argc; ++i) {
    const size_t arg_len = strlen(argv[i]);
    const size_t sep_len = (i > start_index) ? 1u : 0u;
    if (used + sep_len + arg_len >= out_size) {
      out[0] = '\0';
      return false;
    }
    if (sep_len == 1u) {
      out[used++] = ' ';
    }
    memcpy(&out[used], argv[i], arg_len);
    used += arg_len;
    out[used] = '\0';
  }
  return true;
}

/**
 * @brief Queue one ntfy message when cal live drift settle criteria are met.
 * @param drift_c_per_min Current gated drift value in C/min.
 * @param threshold_c_per_min Armed absolute threshold in C/min.
 * @param sustained_seconds Required continuous under-threshold duration.
 * @return true when queued to the existing alert ntfy pipeline.
 */
static bool
QueueCalLiveDriftReadyNtfy_(double drift_c_per_min,
                            double threshold_c_per_min,
                            uint32_t sustained_seconds)
{
  if (g_runtime == NULL || g_runtime->alert_manager == NULL) {
    return false;
  }
  alert_manager_t* manager = g_runtime->alert_manager;
  if (!AlertManagerIsConfigured(manager)) {
    return false;
  }

  alert_ntfy_job_t job = { 0 };
  const uint32_t internal_free =
    heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t internal_largest =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  snprintf(job.url, sizeof(job.url), "%s", manager->config.ntfy_url);
  snprintf(job.topic, sizeof(job.topic), "%s", manager->config.ntfy_topic);
  snprintf(job.token, sizeof(job.token), "%s", manager->config.ntfy_token);
  snprintf(job.root_id,
           sizeof(job.root_id),
           "%s",
           manager->root_id_string != NULL ? manager->root_id_string : "");
  snprintf(job.title, sizeof(job.title), "PT100 Calibration Live Ready");
  int body_len = snprintf(job.body,
                          sizeof(job.body),
                          "cal live drift threshold met\n"
                          "Drift: %.3f C/min\n"
                          "Threshold: %.3f C/min\n"
                          "Sustained for: %" PRIu32 " s",
                          drift_c_per_min,
                          threshold_c_per_min,
                          sustained_seconds);
  snprintf(job.sequence_id,
           sizeof(job.sequence_id),
           "cal-live-ready-%" PRId64,
           esp_timer_get_time() / 1000);
  job.http_timeout_ms = 15000;
  job.attempt = 0u;
  job.next_attempt_ms = esp_timer_get_time() / 1000;
  if (body_len < 0) {
    body_len = 0;
  }
  printf("cal live drift ntfy enqueue: body_len=%d internal_free=%" PRIu32
         " internal_largest=%" PRIu32 "\n",
         body_len,
         internal_free,
         internal_largest);
  return AlertNtfyEnqueueJob(&manager->ntfy, &job);
}

typedef struct
{
  int64_t timestamp_us;
  bool window_ready;
  double stddev_c;
  double drift_c_per_min;
} cal_window_metric_sample_t;

#define CAL_WINDOW_METRIC_HISTORY_CAPACITY 300u

typedef struct
{
  cal_window_metric_sample_t buffer[CAL_WINDOW_METRIC_HISTORY_CAPACITY];
  size_t head;
  size_t count;
} cal_window_metric_history_t;

// NOTE:
// Static ring buffer used intentionally:
// - Prevents FreeRTOS task stack overflow (~28KB prior)
// - Eliminates heap allocation / fragmentation risk
// - Removes O(n) memmove() operations
// - Provides deterministic memory and timing behavior
static cal_window_metric_history_t* s_cal_history = NULL;
static bool s_cal_history_initialized = false;
static bool s_cal_history_in_psram = false;

static bool
CalWindowMetricSampleMeetsCriteria_(const cal_window_metric_sample_t* sample,
                                    double stable_stddev_c,
                                    bool drift_limit_enabled,
                                    double drift_limit_c_per_min)
{
  if (sample == NULL || !sample->window_ready) {
    return false;
  }
  if (sample->stddev_c > stable_stddev_c) {
    return false;
  }
  if (drift_limit_enabled &&
      fabs(sample->drift_c_per_min) > drift_limit_c_per_min) {
    return false;
  }
  return true;
}

static bool
CalHistoryEnsureStorage_(void)
{
  if (s_cal_history_initialized) {
    return (s_cal_history != NULL && s_cal_history_in_psram);
  }
  const size_t bytes = sizeof(cal_window_metric_history_t);
  cal_window_metric_history_t* history =
    (cal_window_metric_history_t*)heap_caps_calloc(
      1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (history != NULL && esp_ptr_external_ram(history)) {
    s_cal_history = history;
    s_cal_history_in_psram = true;
  } else {
    heap_caps_free(history);
    s_cal_history = NULL;
    s_cal_history_in_psram = false;
    ESP_LOGE(kTag,
             "cal metric history PSRAM allocation failed (%u bytes); "
             "live/livecal/capture disabled",
             (unsigned)bytes);
  }
  s_cal_history_initialized = true;
  return (s_cal_history != NULL && s_cal_history_in_psram);
}

static void
CalLogMemorySnapshot_(const char* stage, const char* mode_label)
{
  const char* phase = (stage != NULL) ? stage : "snapshot";
  const char* mode = (mode_label != NULL) ? mode_label : "cal";
  const uint32_t internal_free =
    heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t internal_largest =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t psram_free =
    heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  const uint32_t psram_largest =
    heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  const UBaseType_t stack_words = uxTaskGetStackHighWaterMark(NULL);
  printf("%s mem[%s]: internal_free=%" PRIu32 " internal_largest=%" PRIu32
         " psram_free=%" PRIu32 " psram_largest=%" PRIu32 " stack_free=%" PRIu32
         "\n",
         mode,
         phase,
         internal_free,
         internal_largest,
         psram_free,
         psram_largest,
         (uint32_t)(stack_words * sizeof(StackType_t)));
}

static void
CalTrackLiveLowWatermark_(void)
{
  const uint32_t internal_free =
    heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t internal_largest =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (CalConsoleOpLock_(pdMS_TO_TICKS(5))) {
    if (g_cal_console_op.low_internal_free == 0u ||
        internal_free < g_cal_console_op.low_internal_free) {
      g_cal_console_op.low_internal_free = internal_free;
    }
    if (g_cal_console_op.low_internal_largest == 0u ||
        internal_largest < g_cal_console_op.low_internal_largest) {
      g_cal_console_op.low_internal_largest = internal_largest;
    }
    CalConsoleOpUnlock_();
  }
}

static void
CalTrackLiveStackAndTempUsage_(size_t temp_bytes)
{
  const uint32_t stack_free_bytes =
    (uint32_t)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
  if (!CalConsoleOpLock_(pdMS_TO_TICKS(5))) {
    return;
  }
  if (g_cal_console_op.min_stack_free_bytes == UINT32_MAX ||
      stack_free_bytes < g_cal_console_op.min_stack_free_bytes) {
    g_cal_console_op.min_stack_free_bytes = stack_free_bytes;
  }
  if ((uint32_t)temp_bytes > g_cal_console_op.max_temp_buffer_bytes) {
    g_cal_console_op.max_temp_buffer_bytes = (uint32_t)temp_bytes;
  }
  CalConsoleOpUnlock_();
}

static void
CalLogLiveRetainedResources_(const char* mode_label)
{
  cal_window_storage_layout_t layout = { 0 };
  CalWindowGetStorageLayout(&layout);
  const size_t ntfy_note_queue_bytes =
    (size_t)ALERT_NTFY_QUEUE_LEN * sizeof(alert_notification_t);
  const size_t ntfy_job_queue_bytes =
    (size_t)ALERT_NTFY_JOB_QUEUE_LEN * sizeof(alert_ntfy_job_t);
  const bool ntfy_note_queue_in_psram =
    (g_runtime != NULL && g_runtime->alert_manager != NULL &&
     g_runtime->alert_manager->ntfy.queue_storage != NULL &&
     esp_ptr_external_ram(g_runtime->alert_manager->ntfy.queue_storage));
  const bool ntfy_job_queue_in_psram =
    (g_runtime != NULL && g_runtime->alert_manager != NULL &&
     g_runtime->alert_manager->ntfy.job_queue_storage != NULL &&
     esp_ptr_external_ram(g_runtime->alert_manager->ntfy.job_queue_storage));
  printf("%s retained: cal_op_task_stack=%uB(internal) "
         "cal_console_state=%uB(internal) "
         "cal_metric_history=%uB(%s) "
         "cal_window_temp=%uB(%s) cal_window_ohm=%uB(%s) "
         "cal_window_time=%uB(%s) "
         "ntfy_note_queue_payload=%uB(%s) "
         "ntfy_job_queue_payload=%uB(%s) "
         "drift_notify_job_payload=%uB(transient-stack,max_body~%uB)\n",
         (mode_label != NULL) ? mode_label : "cal",
         (unsigned)kCalOpTaskStackBytes,
         (unsigned)sizeof(cal_console_op_state_t),
         (unsigned)sizeof(cal_window_metric_history_t),
         s_cal_history_in_psram ? "psram" : "internal",
         (unsigned)layout.samples_milli_c_bytes,
         layout.samples_milli_c_in_psram ? "psram" : "internal",
         (unsigned)layout.samples_milli_ohm_bytes,
         layout.samples_milli_ohm_in_psram ? "psram" : "internal",
         (unsigned)layout.samples_time_us_bytes,
         layout.samples_time_us_in_psram ? "psram" : "internal",
         (unsigned)ntfy_note_queue_bytes,
         ntfy_note_queue_in_psram ? "psram" : "internal",
         (unsigned)ntfy_job_queue_bytes,
         ntfy_job_queue_in_psram ? "psram" : "internal",
         (unsigned)sizeof(alert_ntfy_job_t),
         (unsigned)kCalDriftReadyBodyMaxBytes);
}

static size_t
CalHistoryGet(const cal_window_metric_history_t* history,
              size_t index,
              cal_window_metric_sample_t* out)
{
  if (history == NULL || out == NULL || index >= history->count) {
    return 0u;
  }

  const size_t start =
    (history->head + CAL_WINDOW_METRIC_HISTORY_CAPACITY - history->count) %
    CAL_WINDOW_METRIC_HISTORY_CAPACITY;
  const size_t real_index =
    (start + index) % CAL_WINDOW_METRIC_HISTORY_CAPACITY;
  *out = history->buffer[real_index];
  return 1u;
}

static double
CalWindowStableSuffixSeconds_(const cal_window_metric_history_t* history,
                              double stable_stddev_c,
                              bool drift_limit_enabled,
                              double drift_limit_c_per_min)
{
  if (history == NULL || history->count == 0u) {
    return 0.0;
  }

  cal_window_metric_sample_t newest = { 0 };
  if (CalHistoryGet(history, history->count - 1u, &newest) == 0u) {
    return 0.0;
  }
  if (!CalWindowMetricSampleMeetsCriteria_(
        &newest, stable_stddev_c, drift_limit_enabled, drift_limit_c_per_min)) {
    return 0.0;
  }
  size_t first_ok_index = history->count - 1u;
  while (first_ok_index > 0u) {
    const size_t prev = first_ok_index - 1u;
    cal_window_metric_sample_t prev_sample = { 0 };
    if (CalHistoryGet(history, prev, &prev_sample) == 0u ||
        !CalWindowMetricSampleMeetsCriteria_(&prev_sample,
                                             stable_stddev_c,
                                             drift_limit_enabled,
                                             drift_limit_c_per_min)) {
      break;
    }
    first_ok_index = prev;
  }

  cal_window_metric_sample_t first_ok_sample = { 0 };
  if (CalHistoryGet(history, first_ok_index, &first_ok_sample) == 0u) {
    return 0.0;
  }
  const int64_t stable_us = newest.timestamp_us - first_ok_sample.timestamp_us;
  if (stable_us <= 0) {
    return 0.0;
  }
  return stable_us / 1000000.0;
}

/**
 * @brief Estimate ETA until |gated drift| reaches |target| using linear
 * regression over ready-history samples.
 *
 * Regression model:
 *   x = elapsed seconds from oldest selected ready sample
 *   y = fabs(drift_c_per_min) (abs(C/min))
 *   slope units = abs(C/min) per second
 */
static bool
EstimateCalLiveDriftEtaToTarget_(const cal_window_metric_history_t* history,
                                 double target_abs_drift_c_per_min,
                                 double current_abs_drift_c_per_min,
                                 double* out_eta_seconds)
{
  if (out_eta_seconds == NULL || history == NULL || history->count == 0u ||
      !isfinite(target_abs_drift_c_per_min) ||
      !isfinite(current_abs_drift_c_per_min)) {
    return false;
  }
  if (current_abs_drift_c_per_min <= target_abs_drift_c_per_min) {
    *out_eta_seconds = 0.0;
    return true;
  }

  static const size_t kMinSamples = 8u;
  static const double kMinSpanSeconds = 20.0;
  static const double kMinConvergingSlopeAbsPerSec = 5e-6;

  size_t ready_count = 0u;
  int64_t oldest_ready_ts_us = 0;
  int64_t newest_ready_ts_us = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;

  for (size_t i = 0u; i < history->count; ++i) {
    cal_window_metric_sample_t sample = { 0 };
    if (CalHistoryGet(history, i, &sample) == 0u || !sample.window_ready ||
        sample.timestamp_us <= 0 || !isfinite(sample.drift_c_per_min)) {
      continue;
    }

    if (ready_count == 0u) {
      oldest_ready_ts_us = sample.timestamp_us;
    }
    newest_ready_ts_us = sample.timestamp_us;
    const double x_seconds =
      (sample.timestamp_us - oldest_ready_ts_us) / 1000000.0;
    const double y_abs_drift_c_per_min = fabs(sample.drift_c_per_min);

    sum_x += x_seconds;
    sum_y += y_abs_drift_c_per_min;
    sum_xx += (x_seconds * x_seconds);
    sum_xy += (x_seconds * y_abs_drift_c_per_min);
    ready_count++;
  }

  if (ready_count < kMinSamples) {
    return false;
  }
  const double span_seconds =
    (newest_ready_ts_us - oldest_ready_ts_us) / 1000000.0;
  if (!isfinite(span_seconds) || span_seconds < kMinSpanSeconds) {
    return false;
  }

  const double n = (double)ready_count;
  const double denom = (n * sum_xx) - (sum_x * sum_x);
  if (!isfinite(denom) || fabs(denom) < 1e-12) {
    return false;
  }
  const double slope_abs_drift_per_sec =
    ((n * sum_xy) - (sum_x * sum_y)) / denom;
  if (!isfinite(slope_abs_drift_per_sec) ||
      slope_abs_drift_per_sec >= -kMinConvergingSlopeAbsPerSec) {
    return false;
  }

  double eta_seconds =
    (current_abs_drift_c_per_min - target_abs_drift_c_per_min) /
    (-slope_abs_drift_per_sec);
  if (!isfinite(eta_seconds)) {
    return false;
  }
  if (eta_seconds < 0.0) {
    eta_seconds = 0.0;
  }
  *out_eta_seconds = eta_seconds;
  return true;
}

static void
FormatCalLiveDriftEta_(double eta_seconds, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0u || !isfinite(eta_seconds)) {
    return;
  }

  if (eta_seconds < 0.0) {
    eta_seconds = 0.0;
  }
  const uint32_t eta_s = (uint32_t)ceil(eta_seconds);
  if (eta_s < 60u) {
    (void)snprintf(buffer, buffer_size, "%" PRIu32 "s", eta_s);
    return;
  }
  if (eta_s < 3600u) {
    const uint32_t minutes = eta_s / 60u;
    const uint32_t seconds = eta_s % 60u;
    (void)snprintf(
      buffer, buffer_size, "%" PRIu32 "m %02" PRIu32 "s", minutes, seconds);
    return;
  }

  const uint32_t hours = eta_s / 3600u;
  const uint32_t minutes = (eta_s % 3600u) / 60u;
  (void)snprintf(
    buffer, buffer_size, "%" PRIu32 "h %02" PRIu32 "m", hours, minutes);
}

static bool
CalCaptureSavePoint_(double actual_temp_c,
                     bool drift_limit_enabled,
                     double drift_limit_c_per_min,
                     calibration_drift_limit_source_t drift_limit_source,
                     int32_t mean_raw_mC,
                     int32_t stddev_mC,
                     int32_t mean_raw_mOhm,
                     int32_t stddev_mOhm,
                     size_t sample_count,
                     double drift_c_per_min,
                     double delta_c)
{
  if (g_runtime == NULL || g_runtime->settings == NULL) {
    printf("cal capture failed: runtime unavailable\n");
    return false;
  }
  app_settings_t* settings = g_runtime->settings;
  if (settings->calibration_domain == CAL_DOMAIN_TEMP_C &&
      settings->calibration_points_count > 0u) {
    printf("cal capture failed: existing temp-domain points detected; run "
           "'cal clear' first\n");
    return false;
  }
  const int32_t actual_mC = (int32_t)llround(actual_temp_c * 1000.0);
  int point_index = FindCalibrationPointIndexByActualMc_(settings, actual_mC);
  const bool updating_existing = (point_index >= 0);
  if (!updating_existing &&
      settings->calibration_points_count >= CALIBRATION_MAX_POINTS) {
    printf("cal capture failed: already have %u points\n",
           (unsigned)settings->calibration_points_count);
    return false;
  }
  if (!updating_existing) {
    point_index = (int)settings->calibration_points_count;
    settings->calibration_points_count++;
  }
  calibration_point_t* point = &settings->calibration_points[point_index];
  point->raw_avg_mC = mean_raw_mC;
  point->actual_mC = actual_mC;
  point->raw_stddev_mC = stddev_mC;
  point->raw_avg_mOhm = mean_raw_mOhm;
  point->raw_stddev_mOhm = stddev_mOhm;
  point->sample_count = (uint16_t)sample_count;
  point->time_valid = TimeSyncIsSystemTimeValid() ? 1u : 0u;
  point->timestamp_epoch_sec = point->time_valid ? (int64_t)time(NULL) : 0;
  point->captured_drift_mC_per_min = (int32_t)llround(drift_c_per_min * 1000.0);
  point->captured_delta_mC = (int32_t)llround(delta_c * 1000.0);
  point->capture_drift_limit_mC_per_min =
    drift_limit_enabled ? (int32_t)llround(drift_limit_c_per_min * 1000.0)
                        : CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN;
  point->drift_limit_source =
    (uint8_t)(drift_limit_enabled ? drift_limit_source
                                  : CAL_DRIFT_LIMIT_SOURCE_DISABLED);
  point->captured_window_s = (int16_t)CalWindowGetDurationSeconds();
  point->captured_ema_alpha_permille =
    (int16_t)CalWindowGetTrendEmaAlphaPermille();
  settings->calibration_domain = CAL_DOMAIN_RESISTANCE_OHM;
  esp_err_t save_result = SaveCalibrationPointsAndDomain_(settings);
  if (save_result != ESP_OK) {
    printf("save failed: %s\n", esp_err_to_name(save_result));
    return false;
  }
  if (updating_existing) {
    printf("cal capture OK: updated point #%u actual=%.3fC mean_raw=%.3fC "
           "std=%.3fC drift=%.3fC/min delta=%.3fC mean_raw_ohm=%.6f saved\n",
           (unsigned)(point_index + 1),
           actual_temp_c,
           mean_raw_mC / 1000.0,
           stddev_mC / 1000.0,
           drift_c_per_min,
           delta_c,
           mean_raw_mOhm / 1000.0);
  } else {
    printf("cal capture OK: added point #%u actual=%.3fC mean_raw=%.3fC "
           "std=%.3fC drift=%.3fC/min delta=%.3fC mean_raw_ohm=%.6f saved\n",
           (unsigned)(point_index + 1),
           actual_temp_c,
           mean_raw_mC / 1000.0,
           stddev_mC / 1000.0,
           drift_c_per_min,
           delta_c,
           mean_raw_mOhm / 1000.0);
  }
  return true;
}

static void
CalConsoleOpTask(void* task_arg)
{
  (void)task_arg;
  if (!CalConsoleOpLock_(pdMS_TO_TICKS(200))) {
    vTaskDelete(NULL);
    return;
  }
  g_cal_console_op.start_us = esp_timer_get_time();
  g_cal_console_op.last_sample_count = 0;
  g_cal_runtime_last_sample_id = 0u;
  const cal_console_op_mode_t mode = g_cal_console_op.mode;
  const cal_console_live_output_mode_t live_output_mode =
    g_cal_console_op.live_output_mode;
  CalConsoleOpUnlock_();

  if (mode == CAL_CONSOLE_OP_CAPTURE || mode == CAL_CONSOLE_OP_LIVE) {
    CalWindowClear();
    if (g_runtime != NULL && g_runtime->settings != NULL) {
      CalWindowSetDurationSeconds(g_runtime->settings->cal_window_duration_s);
      CalWindowSetTrendEmaAlphaPermille(
        g_runtime->settings->cal_trend_ema_alpha_permille);
    }
  }

  const int64_t start_us = esp_timer_get_time();
  const uint32_t window_duration_s = CalWindowGetDurationSeconds();
  int64_t last_print_us = 0;
  size_t last_sample_count = 0;
  uint32_t last_calibrated_stats_generation = UINT32_MAX;
  if (!CalHistoryEnsureStorage_()) {
    printf("cal: history storage unavailable\n");
    goto cal_op_cleanup;
  }
  s_cal_history->head = 0u;
  s_cal_history->count = 0u;

  while (true) {
    bool cancel_requested = false;
    uint32_t print_every_ms = 1000u;
    int live_seconds = 0;
    bool live_drift_notify_armed = false;
    double live_drift_notify_threshold_c_per_min = 0.0;
    bool live_drift_notify_sent = false;
    int64_t live_drift_under_start_us = -1;
    bool capture_armed = false;
    int64_t capture_armed_start_us = 0;
    double actual_temp_c = 0.0;
    double stable_stddev_c = 0.0;
    bool drift_limit_enabled = true;
    double drift_limit_c_per_min = 0.0;
    calibration_drift_limit_source_t drift_limit_source =
      CAL_DRIFT_LIMIT_SOURCE_DEFAULT;
    int min_seconds = 0;
    int timeout_seconds = 0;

    if (CalConsoleOpLock_(pdMS_TO_TICKS(50))) {
      cancel_requested = g_cal_console_op.cancel_requested;
      print_every_ms = g_cal_console_op.print_every_ms;
      live_seconds = g_cal_console_op.live_seconds;
      live_drift_notify_armed = g_cal_console_op.live_drift_notify_armed;
      live_drift_notify_threshold_c_per_min =
        g_cal_console_op.live_drift_notify_threshold_c_per_min;
      live_drift_notify_sent = g_cal_console_op.live_drift_notify_sent;
      live_drift_under_start_us = g_cal_console_op.live_drift_under_start_us;
      capture_armed = g_cal_console_op.capture_armed;
      capture_armed_start_us = g_cal_console_op.capture_armed_start_us;
      actual_temp_c = g_cal_console_op.capture_actual_temp_c;
      stable_stddev_c = g_cal_console_op.capture_stable_stddev_c;
      drift_limit_enabled = g_cal_console_op.capture_drift_limit_enabled;
      drift_limit_c_per_min = g_cal_console_op.capture_drift_limit_c_per_min;
      drift_limit_source = g_cal_console_op.capture_drift_limit_source;
      min_seconds = g_cal_console_op.capture_min_seconds;
      timeout_seconds = g_cal_console_op.capture_timeout_seconds;
      CalConsoleOpUnlock_();
    }

    if (cancel_requested) {
      if (capture_armed || mode == CAL_CONSOLE_OP_CAPTURE) {
        printf("cal capture aborted\n");
      }
      printf("cal: stopped\n");
      break;
    }

    const cal_live_sensor_status_t sensor_status =
      MaybePushCalRawSampleFromSensorWithStatus_();

    const size_t sample_count = CalWindowGetSampleCount();
    const uint32_t active_window_generation = CalWindowGetActiveGeneration();
    const int64_t now_us = esp_timer_get_time();
    const bool new_sample_arrived =
      sensor_status.sample_valid_for_window && (sample_count > 0u);
    const bool window_ready = CalWindowIsReady();
    const bool print_due =
      (sample_count != last_sample_count) || (last_print_us == 0) ||
      (now_us - last_print_us >= (int64_t)print_every_ms * 1000LL);

    int32_t last_raw_mC = 0;
    int32_t mean_raw_mC = 0;
    int32_t stddev_mC = 0;
    int32_t last_raw_mOhm = 0;
    int32_t mean_raw_mOhm = 0;
    int32_t stddev_mOhm = 0;
    int32_t delta_mC = 0;
    double drift_c_per_min_raw = 0.0;
    double drift_c_per_min_ema = 0.0;
    double delta_c_ema = 0.0;
    bool trend_ema_initialized = false;
    CalWindowGetStats(&last_raw_mC, &mean_raw_mC, &stddev_mC);
    CalWindowGetResistanceStats(&last_raw_mOhm, &mean_raw_mOhm, &stddev_mOhm);
    CalWindowGetTrendStats(
      NULL, NULL, &delta_mC, NULL, &drift_c_per_min_raw, NULL);
    CalWindowGetTrendEmaStats(
      &delta_c_ema, &drift_c_per_min_ema, &trend_ema_initialized);
    const double gated_drift_c_per_min =
      trend_ema_initialized ? drift_c_per_min_ema : drift_c_per_min_raw;
    const double stddev_c = stddev_mC / 1000.0;
    const double delta_c =
      trend_ema_initialized ? delta_c_ema : (delta_mC / 1000.0);

    if (new_sample_arrived) {
      CalTrackLiveLowWatermark_();
      s_cal_history->buffer[s_cal_history->head] =
        (cal_window_metric_sample_t){ .timestamp_us = now_us,
                                      .window_ready = window_ready,
                                      .stddev_c = stddev_c,
                                      .drift_c_per_min =
                                        gated_drift_c_per_min };
      s_cal_history->head =
        (s_cal_history->head + 1u) % CAL_WINDOW_METRIC_HISTORY_CAPACITY;
      if (s_cal_history->count < CAL_WINDOW_METRIC_HISTORY_CAPACITY) {
        s_cal_history->count++;
      }
    }

    if (live_output_mode == CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED &&
        active_window_generation != last_calibrated_stats_generation) {
      // The calibrated cache is keyed by active-window generation. Correctness
      // depends on single-owner updates to the calibration live/capture window.
      if (!RefreshCalibratedWindowTempStatsCache_(sample_count,
                                                  active_window_generation,
                                                  last_raw_mC,
                                                  last_raw_mOhm,
                                                  mean_raw_mC,
                                                  mean_raw_mOhm)) {
        printf("cal livecal blocked: calibration evaluation failed\n");
        break;
      }
      // Diagnostic: calibrated-window scan runs once per new window state,
      // not once per print tick.
      ESP_LOGD(kTag,
               "cal livecal cache refreshed for generation=%u sample_count=%u",
               (unsigned)active_window_generation,
               (unsigned)sample_count);
      last_calibrated_stats_generation = active_window_generation;
    }

    CalTrackLiveStackAndTempUsage_(0u);

    if (mode == CAL_CONSOLE_OP_LIVE) {
      if (print_due) {
        char timestamp_buf[40] = { 0 };
        (void)FormatCalLiveTimestampNow_(timestamp_buf, sizeof(timestamp_buf));
        char drift_eta_buf[24] = { 0 };
        size_t temp_buffer_bytes =
          sizeof(timestamp_buf) + sizeof(drift_eta_buf);
        const char* drift_eta_text = NULL;
        if (live_drift_notify_armed) {
          const double target_abs_drift_c_per_min =
            fabs(live_drift_notify_threshold_c_per_min);
          const double current_abs_drift_c_per_min =
            fabs(gated_drift_c_per_min);
          double drift_eta_seconds = 0.0;
          if (EstimateCalLiveDriftEtaToTarget_(s_cal_history,
                                               target_abs_drift_c_per_min,
                                               current_abs_drift_c_per_min,
                                               &drift_eta_seconds)) {
            FormatCalLiveDriftEta_(
              drift_eta_seconds, drift_eta_buf, sizeof(drift_eta_buf));
            drift_eta_text = drift_eta_buf;
          } else {
            drift_eta_text = "n/a";
          }
        }
        if (live_output_mode == CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED) {
          double calibrated_temp_c = 0.0;
          double mean_calibrated_temp_c = 0.0;
          double stddev_calibrated_temp_c = 0.0;
          bool calibrated_stats_valid = false;
          size_t calibrated_stats_sample_count = 0u;
          uint32_t calibrated_stats_generation = 0u;
          temp_buffer_bytes += 64u;
          CalWindowGetCalibratedTempStats(&calibrated_temp_c,
                                          &mean_calibrated_temp_c,
                                          &stddev_calibrated_temp_c,
                                          &calibrated_stats_valid,
                                          &calibrated_stats_sample_count,
                                          &calibrated_stats_generation);
          if (!calibrated_stats_valid ||
              calibrated_stats_generation != active_window_generation ||
              calibrated_stats_sample_count != sample_count) {
            printf("cal livecal blocked: calibration evaluation failed\n");
            break;
          }
          PrintCalWindowCalibratedLine_(timestamp_buf,
                                        sample_count,
                                        last_raw_mOhm,
                                        mean_raw_mOhm,
                                        stddev_mOhm,
                                        calibrated_temp_c,
                                        mean_calibrated_temp_c,
                                        stddev_calibrated_temp_c,
                                        gated_drift_c_per_min,
                                        delta_c,
                                        sensor_status.fault_status);
        } else {
          PrintCalWindowLine_("cal live",
                              timestamp_buf,
                              sample_count,
                              last_raw_mC,
                              last_raw_mOhm,
                              mean_raw_mC,
                              stddev_mC,
                              gated_drift_c_per_min,
                              delta_c,
                              live_drift_notify_armed,
                              drift_eta_text);
        }
        CalTrackLiveStackAndTempUsage_(temp_buffer_bytes);
        last_print_us = now_us;
        last_sample_count = sample_count;
      }
      if (live_drift_notify_armed && !live_drift_notify_sent) {
        const bool drift_under_threshold =
          window_ready && (fabs(gated_drift_c_per_min) <=
                           live_drift_notify_threshold_c_per_min);
        if (drift_under_threshold) {
          if (live_drift_under_start_us < 0) {
            live_drift_under_start_us = now_us;
          }
          const int64_t elapsed_us = now_us - live_drift_under_start_us;
          if (elapsed_us >= (int64_t)window_duration_s * 1000000LL) {
            const bool queued =
              QueueCalLiveDriftReadyNtfy_(gated_drift_c_per_min,
                                          live_drift_notify_threshold_c_per_min,
                                          window_duration_s);
            if (queued) {
              printf("cal live: drift notify sent\n");
            } else {
              printf("cal live: drift notify condition met, but ntfy not sent "
                     "(not configured or queue unavailable)\n");
            }
            live_drift_notify_sent = true;
            if (CalConsoleOpLock_(pdMS_TO_TICKS(20))) {
              g_cal_console_op.live_drift_notify_sent = true;
              CalConsoleOpUnlock_();
            }
          }
        } else {
          live_drift_under_start_us = -1;
        }
        if (CalConsoleOpLock_(pdMS_TO_TICKS(20))) {
          g_cal_console_op.live_drift_under_start_us =
            live_drift_under_start_us;
          CalConsoleOpUnlock_();
        }
      }
      if (live_seconds > 0 &&
          now_us - start_us >= (int64_t)live_seconds * 1000000LL) {
        printf("%s complete\n",
               (live_output_mode == CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED)
                 ? "cal livecal"
                 : "cal live");
        break;
      }
    }

    if (capture_armed || mode == CAL_CONSOLE_OP_CAPTURE) {
      const double stable_elapsed_s =
        CalWindowStableSuffixSeconds_(s_cal_history,
                                      stable_stddev_c,
                                      drift_limit_enabled,
                                      drift_limit_c_per_min);
      const double elapsed_s = (now_us - capture_armed_start_us) / 1000000.0;
      if (print_due && mode != CAL_CONSOLE_OP_LIVE) {
        PrintCalCaptureWindowLine_(elapsed_s,
                                   sample_count,
                                   last_raw_mC,
                                   last_raw_mOhm,
                                   mean_raw_mC,
                                   stddev_c,
                                   gated_drift_c_per_min,
                                   delta_c,
                                   stable_elapsed_s,
                                   min_seconds,
                                   stable_stddev_c,
                                   drift_limit_enabled,
                                   drift_limit_c_per_min,
                                   drift_limit_source,
                                   actual_temp_c);
        last_print_us = now_us;
        last_sample_count = sample_count;
      }

      if (stable_elapsed_s >= (double)min_seconds) {
        if (mode == CAL_CONSOLE_OP_LIVE) {
          printf("cal capture: capture criteria already satisfied; capturing "
                 "immediately from live buffer\n");
        }
        const bool saved = CalCaptureSavePoint_(actual_temp_c,
                                                drift_limit_enabled,
                                                drift_limit_c_per_min,
                                                drift_limit_source,
                                                mean_raw_mC,
                                                stddev_mC,
                                                mean_raw_mOhm,
                                                stddev_mOhm,
                                                sample_count,
                                                gated_drift_c_per_min,
                                                delta_c);
        if (mode == CAL_CONSOLE_OP_CAPTURE) {
          break;
        }
        if (CalConsoleOpLock_(pdMS_TO_TICKS(50))) {
          g_cal_console_op.capture_armed = false;
          g_cal_console_op.capture_armed_start_us = 0;
          CalConsoleOpUnlock_();
        }
        if (!saved) {
          printf("cal capture: disarmed after failure; cal live continues\n");
        }
      } else if (elapsed_s >= (double)timeout_seconds) {
        printf("cal capture failed: timeout after %d seconds\n",
               timeout_seconds);
        if (mode == CAL_CONSOLE_OP_CAPTURE) {
          break;
        }
        if (CalConsoleOpLock_(pdMS_TO_TICKS(50))) {
          g_cal_console_op.capture_armed = false;
          g_cal_console_op.capture_armed_start_us = 0;
          CalConsoleOpUnlock_();
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(print_every_ms));
  }

cal_op_cleanup:
  CalLogMemorySnapshot_(
    "stop-post",
    (live_output_mode == CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED)
      ? "cal livecal"
      : ((mode == CAL_CONSOLE_OP_CAPTURE) ? "cal capture" : "cal live"));
  if (CalConsoleOpLock_(pdMS_TO_TICKS(200))) {
    const uint32_t stop_internal_free =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const uint32_t stop_internal_largest =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const uint32_t low_internal_free = g_cal_console_op.low_internal_free;
    const uint32_t low_internal_largest = g_cal_console_op.low_internal_largest;
    const uint32_t min_stack_free = g_cal_console_op.min_stack_free_bytes;
    const uint32_t max_temp_buffer = g_cal_console_op.max_temp_buffer_bytes;
    printf("cal heap[start]: internal_free=%" PRIu32
           " internal_largest=%" PRIu32 "\n",
           g_cal_console_op.start_internal_free,
           g_cal_console_op.start_internal_largest);
    printf("cal heap[low]: internal_free=%" PRIu32 " internal_largest=%" PRIu32
           " delta_internal_free=%" PRId32 " delta_internal_largest=%" PRId32
           "\n",
           low_internal_free,
           low_internal_largest,
           (int32_t)low_internal_free -
             (int32_t)g_cal_console_op.start_internal_free,
           (int32_t)low_internal_largest -
             (int32_t)g_cal_console_op.start_internal_largest);
    printf("cal heap[stop]: internal_free=%" PRIu32 " internal_largest=%" PRIu32
           " delta_internal_free=%" PRId32 " delta_internal_largest=%" PRId32
           "\n",
           stop_internal_free,
           stop_internal_largest,
           (int32_t)stop_internal_free -
             (int32_t)g_cal_console_op.start_internal_free,
           (int32_t)stop_internal_largest -
             (int32_t)g_cal_console_op.start_internal_largest);
    if (min_stack_free != UINT32_MAX) {
      printf("cal_op stack: alloc=%" PRIu32 " min_free=%" PRIu32
             " peak_used=%" PRIu32 "\n",
             kCalOpTaskStackBytes,
             min_stack_free,
             kCalOpTaskStackBytes - min_stack_free);
    }
    printf("cal_op temp buffers: max=%" PRIu32 "B\n", max_temp_buffer);
    g_cal_console_op.mode = CAL_CONSOLE_OP_NONE;
    g_cal_console_op.task_handle = NULL;
    g_cal_console_op.cancel_requested = false;
    g_cal_console_op.start_us = 0;
    g_cal_console_op.last_sample_count = 0;
    g_cal_console_op.capture_armed = false;
    g_cal_console_op.capture_armed_start_us = 0;
    g_cal_console_op.capture_drift_limit_enabled = true;
    g_cal_console_op.capture_drift_limit_c_per_min = 0.0;
    g_cal_console_op.capture_drift_limit_source =
      CAL_DRIFT_LIMIT_SOURCE_DEFAULT;
    g_cal_console_op.live_drift_notify_armed = false;
    g_cal_console_op.live_drift_notify_threshold_c_per_min = 0.0;
    g_cal_console_op.live_output_mode = CAL_CONSOLE_LIVE_OUTPUT_RAW;
    g_cal_console_op.live_drift_under_start_us = -1;
    g_cal_console_op.live_drift_notify_sent = false;
    g_cal_console_op.start_internal_free = 0u;
    g_cal_console_op.start_internal_largest = 0u;
    g_cal_console_op.start_psram_free = 0u;
    g_cal_console_op.start_psram_largest = 0u;
    g_cal_console_op.low_internal_free = 0u;
    g_cal_console_op.low_internal_largest = 0u;
    g_cal_console_op.min_stack_free_bytes = UINT32_MAX;
    g_cal_console_op.max_temp_buffer_bytes = 0u;
    CalConsoleOpUnlock_();
  }

  vTaskDelete(NULL);
}

/**
 * @brief Execute CommandCal.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandCal(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }

  app_settings_t* settings = g_runtime->settings;
  if (argc >= 2) {
    const char* action = argv[1];
    if (strcmp(action, "status") == 0) {
      PrintCalibrationStatusUnified(settings, RuntimeGetState());
      return 0;
    }
    if (strcmp(action, "cfg") == 0) {
      if (argc == 3 && strcmp(argv[2], "show") == 0) {
        printf("cal_cfg_window_s: %u\n",
               (unsigned)settings->cal_window_duration_s);
        printf("cal_cfg_ema_alpha: %.3f\n",
               settings->cal_trend_ema_alpha_permille / 1000.0);
        printf("cal_cfg_drift_limit_default_c_per_min: %.3f\n",
               kDefaultCaptureDriftLimitCPerMin);
        return 0;
      }
      if (argc == 5 && strcmp(argv[2], "set") == 0) {
        if (strcmp(argv[3], "ema_alpha") == 0) {
          char* end = NULL;
          const double alpha = strtod(argv[4], &end);
          if (end == argv[4] || *end != '\0' || alpha <= 0.0 || alpha > 1.0) {
            printf("invalid ema_alpha; expected >0 and <=1.0\n");
            return 1;
          }
          const uint16_t alpha_permille = (uint16_t)llround(alpha * 1000.0);
          esp_err_t result = AppSettingsSaveCalibrationConfig(
            settings->cal_window_duration_s, alpha_permille);
          if (result != ESP_OK) {
            printf("save failed: %s\n", esp_err_to_name(result));
            return 1;
          }
          settings->cal_trend_ema_alpha_permille = alpha_permille;
          CalWindowSetTrendEmaAlphaPermille(alpha_permille);
          printf("cal cfg set ema_alpha: %.3f\n", alpha_permille / 1000.0);
          return 0;
        }
        if (strcmp(argv[3], "window_s") == 0) {
          char* end = NULL;
          long window_s = strtol(argv[4], &end, 10);
          if (end == argv[4] || *end != '\0' ||
              window_s < CAL_WINDOW_DURATION_MIN_S ||
              window_s > CAL_WINDOW_DURATION_MAX_S) {
            printf("invalid window_s; expected %u..%u\n",
                   (unsigned)CAL_WINDOW_DURATION_MIN_S,
                   (unsigned)CAL_WINDOW_DURATION_MAX_S);
            return 1;
          }
          esp_err_t result = AppSettingsSaveCalibrationConfig(
            (uint16_t)window_s, settings->cal_trend_ema_alpha_permille);
          if (result != ESP_OK) {
            printf("save failed: %s\n", esp_err_to_name(result));
            return 1;
          }
          settings->cal_window_duration_s = (uint16_t)window_s;
          CalWindowSetDurationSeconds((uint16_t)window_s);
          printf("cal cfg set window_s: %ld\n", window_s);
          return 0;
        }
      }
      printf("usage: cal cfg show | cal cfg set ema_alpha <value> | "
             "cal cfg set window_s <seconds>\n");
      return 1;
    }
    if (strcmp(action, "metar") == 0) {
      if (argc < 3) {
        printf("usage: cal metar set <elevation_ft> \"<raw METAR>\" | "
               "cal metar show | cal metar clear\n");
        return 1;
      }
      const char* metar_action = argv[2];
      if (strcmp(metar_action, "show") == 0) {
        PrintCalibrationMetarBlock_(
          &settings->cal_metar, "Calibration Steam Reference", false);
        return 0;
      }
      if (strcmp(metar_action, "clear") == 0) {
        calibration_metar_reference_t cleared = { 0 };
        esp_err_t save_result = AppSettingsSaveCalibrationMetar(&cleared);
        if (save_result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(save_result));
          return 1;
        }
        memset(&settings->cal_metar, 0, sizeof(settings->cal_metar));
        printf("cal metar cleared\n");
        return 0;
      }
      if (strcmp(metar_action, "set") == 0) {
        if (argc != 5) {
          printf("usage: cal metar set <elevation_ft> \"<raw METAR>\"\n");
          return 1;
        }
        uint16_t elevation_ft = 0;
        if (!ParseElevationFt_(argv[3], &elevation_ft)) {
          printf("ERR invalid elevation_ft (expected 0..20000)\n");
          return 1;
        }
        calibration_metar_reference_t metar = { 0 };
        char parse_error[96] = { 0 };
        if (!ParseMetarLine_(
              argv[4], &metar, parse_error, sizeof(parse_error))) {
          printf("ERR %s\n",
                 parse_error[0] != '\0' ? parse_error
                                        : "unable to parse METAR");
          return 1;
        }
        metar.elevation_ft = elevation_ft;
        metar.station_pressure_inhg =
          StationPressureFromSlpInHg(metar.altimeter_inhg, (float)elevation_ft);
        if (!isfinite(metar.station_pressure_inhg) ||
            metar.station_pressure_inhg <= 0.0f) {
          printf("ERR invalid station pressure from altimeter/elevation\n");
          return 1;
        }
        metar.satpt_c =
          SaturationTempCFromStationInHg(metar.station_pressure_inhg);
        if (!isfinite(metar.satpt_c)) {
          printf("ERR invalid saturation-point result\n");
          return 1;
        }
        const bool time_valid = TimeSyncIsSystemTimeValid();
        const int64_t now_epoch = time_valid ? (int64_t)time(NULL) : 0;
        metar.stored_at_utc_epoch = now_epoch;
        bool obs_resolved = false;
        (void)ResolveMetarObservationIsoUtc_(metar.observation_token,
                                             time_valid,
                                             now_epoch,
                                             metar.observation_iso_utc,
                                             sizeof(metar.observation_iso_utc),
                                             &obs_resolved);
        metar.observation_resolved = obs_resolved ? 1u : 0u;
        snprintf(metar.method_note,
                 sizeof(metar.method_note),
                 "%s",
                 "steam column attached above beaker; vented system");
        metar.valid = 1u;
        esp_err_t save_result = AppSettingsSaveCalibrationMetar(&metar);
        if (save_result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(save_result));
          return 1;
        }
        settings->cal_metar = metar;
        printf("cal metar set: station=%s obs=%s altimeter=%.2f inHg "
               "station=%.2f inHg satpt=%.3f C\n",
               metar.station_id,
               metar.observation_token,
               (double)metar.altimeter_inhg,
               (double)metar.station_pressure_inhg,
               (double)metar.satpt_c);
        if (metar.observation_resolved == 0u) {
          printf("note: observation token retained as %s (absolute UTC "
                 "unresolved)\n",
                 metar.observation_token);
        }
        printf("Use this for steam-point capture: cal capture %.3f\n",
               (double)metar.satpt_c);
        return 0;
      }
      printf("usage: cal metar set <elevation_ft> \"<raw METAR>\" | "
             "cal metar show | cal metar clear\n");
      return 1;
    }
    if (strcmp(action, "set") == 0) {
      if (argc < 4) {
        printf("usage: cal set date <YYYY-MM-DD> | cal set due <count> "
               "<days|months|years> | cal set due_override <count> "
               "<days|months|years> | cal set method <string...>\n");
        return 1;
      }
      const char* target = argv[2];
      if (strcmp(target, "date") == 0) {
        if (argc != 4) {
          printf("usage: cal set date <YYYY-MM-DD>\n");
          return 1;
        }
        int64_t epoch = 0;
        if (!ParseUtcDateString(argv[3], &epoch)) {
          printf("invalid date; expected YYYY-MM-DD\n");
          return 1;
        }
        settings->cal_last_override_utc = epoch;
        esp_err_t result = AppSettingsSaveCalibrationSchedule(settings);
        if (result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(result));
          return 1;
        }
        printf("calibration override date set to %s\n", argv[3]);
        ForcePtlogRevisionForCalibrationMetadata("cal set date");
        return 0;
      }
      if (strcmp(target, "method") == 0) {
        if (argc < 4) {
          printf("usage: cal set method <string...>\n");
          return 1;
        }
        char method_buffer[APP_SETTINGS_CAL_METHOD_MAX_LEN] = { 0 };
        if (!JoinArgsWithSpaces(
              argc, argv, 3, method_buffer, sizeof(method_buffer))) {
          printf("method too long (max %u chars)\n",
                 (unsigned)(APP_SETTINGS_CAL_METHOD_MAX_LEN - 1));
          return 1;
        }
        esp_err_t result = AppSettingsSaveCalibrationMethod(method_buffer);
        if (result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(result));
          return 1;
        }
        (void)snprintf(settings->cal_method,
                       sizeof(settings->cal_method),
                       "%s",
                       method_buffer);
        printf("calibration method set to: %s\n", method_buffer);
        ForcePtlogRevisionForCalibrationMetadata("cal set method");
        return 0;
      }
      if (strcmp(target, "due") == 0 || strcmp(target, "due_override") == 0) {
        if (argc != 5) {
          printf("usage: cal set %s <count> <days|months|years>\n", target);
          return 1;
        }
        char* end = NULL;
        long count_long = strtol(argv[3], &end, 10);
        if (end == argv[3] || *end != '\0' || count_long <= 0 ||
            count_long > UINT16_MAX) {
          printf("invalid count\n");
          return 1;
        }
        uint8_t unit = 0;
        if (!ParseCalDueUnit(argv[4], &unit)) {
          printf("invalid unit; use days|months|years\n");
          return 1;
        }
        if (strcmp(target, "due") == 0) {
          settings->cal_due_count = (uint16_t)count_long;
          settings->cal_due_unit = unit;
        } else {
          settings->cal_due_override_count = (uint16_t)count_long;
          settings->cal_due_override_unit = unit;
        }
        esp_err_t result = AppSettingsSaveCalibrationSchedule(settings);
        if (result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(result));
          return 1;
        }
        printf("calibration %s set to %ld %s\n",
               (strcmp(target, "due") == 0) ? "due" : "due override",
               count_long,
               argv[4]);
        ForcePtlogRevisionForCalibrationMetadata((strcmp(target, "due") == 0)
                                                   ? "cal set due"
                                                   : "cal set due_override");
        return 0;
      }
      printf("usage: cal set date <YYYY-MM-DD> | cal set due <count> "
             "<days|months|years> | cal set due_override <count> "
             "<days|months|years> | cal set method <string...>\n");
      return 1;
    }
    if (strcmp(action, "clear") == 0 && argc >= 3) {
      const char* target = argv[2];
      if (strcmp(target, "date") == 0) {
        settings->cal_last_override_utc = 0;
      } else if (strcmp(target, "due") == 0) {
        settings->cal_due_count = 0;
        settings->cal_due_unit = 0;
      } else if (strcmp(target, "due_override") == 0) {
        settings->cal_due_override_count = 0;
        settings->cal_due_override_unit = 0;
      } else if (strcmp(target, "method") == 0) {
        esp_err_t result = AppSettingsSaveCalibrationMethod("");
        if (result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(result));
          return 1;
        }
        settings->cal_method[0] = '\0';
        printf("calibration method cleared\n");
        ForcePtlogRevisionForCalibrationMetadata("cal clear method");
        return 0;
      } else {
        printf(
          "usage: cal clear date | cal clear due | cal clear due_override | "
          "cal clear method\n");
        return 1;
      }
      esp_err_t result = AppSettingsSaveCalibrationSchedule(settings);
      if (result != ESP_OK) {
        printf("save failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      printf("calibration %s cleared\n", target);
      ForcePtlogRevisionForCalibrationMetadata(
        (strcmp(target, "date") == 0)
          ? "cal clear date"
          : ((strcmp(target, "due") == 0) ? "cal clear due"
                                          : "cal clear due_override"));
      return 0;
    }

    if (strcmp(action, "stop") == 0) {
      if (!CalConsoleOpIsActive()) {
        printf("cal: no active operation\n");
        return 0;
      }
      CalLogMemorySnapshot_("stop-request", "cal");
      CalConsoleOpRequestCancel();
      printf("cal: cancel requested (waiting for stop)\n");
      return 0;
    }

    if (strcmp(action, "live") == 0 || strcmp(action, "livecal") == 0) {
      int every_ms = 0;
      int seconds = 0;
      bool drift_notify_armed = false;
      double drift_notify_threshold_c_per_min = 0.0;
      if (!ParseCalLiveArguments_(argc,
                                  argv,
                                  action,
                                  &every_ms,
                                  &seconds,
                                  &drift_notify_armed,
                                  &drift_notify_threshold_c_per_min)) {
        return 1;
      }

      const bool calibrated_live = (strcmp(action, "livecal") == 0);
      if (calibrated_live) {
        const runtime_state_t* state = RuntimeGetState();
        const char* applied_reason = NULL;
        if (!ConsoleCalibrationIsApplied(settings, state, &applied_reason)) {
          printf("cal livecal blocked: calibration not applied (%s)\n",
                 (applied_reason != NULL) ? applied_reason : "unknown");
          return 1;
        }
      }

      esp_err_t result = CalConsoleOpStartLive(
        (uint32_t)every_ms,
        seconds,
        calibrated_live ? (int)CAL_CONSOLE_LIVE_OUTPUT_CALIBRATED
                        : (int)CAL_CONSOLE_LIVE_OUTPUT_RAW,
        drift_notify_armed,
        drift_notify_threshold_c_per_min);
      if (result == ESP_ERR_INVALID_STATE) {
        printf("cal: operation already active\n");
        return 1;
      }
      if (result != ESP_OK) {
        printf("cal %s failed: %s\n",
               calibrated_live ? "livecal" : "live",
               esp_err_to_name(result));
        return 1;
      }
      CalLogMemorySnapshot_("start-post",
                            calibrated_live ? "cal livecal" : "cal live");
      printf("cal %s started (use 'cal stop' to abort)\n",
             calibrated_live ? "livecal" : "live");
      if (drift_notify_armed) {
        const uint32_t sustained_s = (settings != NULL)
                                       ? settings->cal_window_duration_s
                                       : CAL_WINDOW_DURATION_DEFAULT_S;
        printf("drift notify armed: |drift| <= %.3f C/min for %" PRIu32 " s\n",
               drift_notify_threshold_c_per_min,
               sustained_s);
      }
      return 0;
    }

    if (strcmp(action, "capture") == 0) {
      const char* temp_arg = NULL;
      for (int i = 2; i < argc; ++i) {
        const char* arg = argv[i];
        if (arg == NULL) {
          continue;
        }
        if (strncmp(arg, "--", 2) == 0) {
          const char* option = arg + 2;
          const char* value_eq = strchr(option, '=');
          if (value_eq == NULL &&
              (strncmp(option, "stable_stddev_c", 15) == 0 ||
               strncmp(option, "min_seconds", 11) == 0 ||
               strncmp(option, "timeout_seconds", 15) == 0 ||
               strncmp(option, "drift_c_per_min", 15) == 0)) {
            if (i + 1 < argc) {
              ++i;
            }
          }
          continue;
        }
        if (temp_arg != NULL) {
          printf("usage: cal capture <actual_temp_c> "
                 "[--stable_stddev_c 0.05] [--min_seconds 5] "
                 "[--timeout_seconds 120] [--drift_c_per_min 0.020] "
                 "[--no-drift-limit]\n");
          return 1;
        }
        temp_arg = arg;
      }

      if (temp_arg == NULL) {
        printf("usage: cal capture <actual_temp_c> "
               "[--stable_stddev_c 0.05] [--min_seconds 5] "
               "[--timeout_seconds 120] [--drift_c_per_min 0.020] "
               "[--no-drift-limit]\n");
        return 1;
      }

      double actual_temp_c = 0.0;
      if (!ParseTempC_(temp_arg, &actual_temp_c)) {
        const size_t len = strlen(temp_arg);
        if (len > 0 && (temp_arg[len - 1] == 'F' || temp_arg[len - 1] == 'f')) {
          printf("invalid temperature: Fahrenheit suffix not supported\n");
        } else {
          printf("invalid temperature: expected Celsius value (e.g., 98.5C)\n");
        }
        return 1;
      }
      if (settings->calibration_domain == CAL_DOMAIN_TEMP_C &&
          settings->calibration_points_count > 0u) {
        printf("cal capture failed: existing temp-domain points detected; run "
               "'cal clear' first\n");
        return 1;
      }

      double stable_stddev_c = 0.05;
      int min_seconds = 5;
      int timeout_seconds = 120;
      bool drift_limit_enabled = !OptionPresent_(argc, argv, "no-drift-limit");
      double drift_limit_c_per_min = kDefaultCaptureDriftLimitCPerMin;
      calibration_drift_limit_source_t drift_limit_source =
        CAL_DRIFT_LIMIT_SOURCE_DEFAULT;

      double option_double = 0.0;
      int option_int = 0;

      if (ParseOptionDouble_(argc, argv, "stable_stddev_c", &option_double)) {
        stable_stddev_c = option_double;
      } else if (OptionPresent_(argc, argv, "stable_stddev_c")) {
        printf("usage: cal capture <actual_temp_c> "
               "[--stable_stddev_c 0.05] [--min_seconds 5] "
               "[--timeout_seconds 120] [--drift_c_per_min 0.020] "
               "[--no-drift-limit]\n");
        return 1;
      }

      if (ParseOptionInt_(argc, argv, "min_seconds", &option_int)) {
        min_seconds = option_int;
      } else if (OptionPresent_(argc, argv, "min_seconds")) {
        printf("usage: cal capture <actual_temp_c> "
               "[--stable_stddev_c 0.05] [--min_seconds 5] "
               "[--timeout_seconds 120] [--drift_c_per_min 0.020] "
               "[--no-drift-limit]\n");
        return 1;
      }

      if (ParseOptionInt_(argc, argv, "timeout_seconds", &option_int)) {
        timeout_seconds = option_int;
      } else if (OptionPresent_(argc, argv, "timeout_seconds")) {
        printf("usage: cal capture <actual_temp_c> "
               "[--stable_stddev_c 0.05] [--min_seconds 5] "
               "[--timeout_seconds 120] [--drift_c_per_min 0.020] "
               "[--no-drift-limit]\n");
        return 1;
      }
      if (ParseOptionDouble_(argc, argv, "drift_c_per_min", &option_double)) {
        drift_limit_enabled = true;
        drift_limit_c_per_min = fabs(option_double);
        drift_limit_source = CAL_DRIFT_LIMIT_SOURCE_USER;
      } else if (OptionPresent_(argc, argv, "drift_c_per_min")) {
        printf("usage: cal capture <actual_temp_c> "
               "[--stable_stddev_c 0.05] [--min_seconds 5] "
               "[--timeout_seconds 120] [--drift_c_per_min 0.020] "
               "[--no-drift-limit]\n");
        return 1;
      }
      if (drift_limit_enabled &&
          drift_limit_source == CAL_DRIFT_LIMIT_SOURCE_DEFAULT) {
        drift_limit_c_per_min = fabs(drift_limit_c_per_min);
      }
      if (!drift_limit_enabled) {
        drift_limit_source = CAL_DRIFT_LIMIT_SOURCE_DISABLED;
      }

      if (stable_stddev_c <= 0.0 || min_seconds <= 0 || timeout_seconds <= 0 ||
          (drift_limit_enabled && drift_limit_c_per_min <= 0.0)) {
        printf("usage: cal capture <actual_temp_c> "
               "[--stable_stddev_c 0.05] [--min_seconds 5] "
               "[--timeout_seconds 120] [--drift_c_per_min 0.020] "
               "[--no-drift-limit]\n");
        return 1;
      }

      esp_err_t result = ESP_ERR_INVALID_STATE;
      if (CalConsoleOpLock_(pdMS_TO_TICKS(50))) {
        const cal_console_op_mode_t active_mode = g_cal_console_op.mode;
        CalConsoleOpUnlock_();
        if (active_mode == CAL_CONSOLE_OP_LIVE) {
          bool replaced = false;
          result = CalConsoleOpArmCaptureWhileLive(actual_temp_c,
                                                   stable_stddev_c,
                                                   drift_limit_enabled,
                                                   drift_limit_c_per_min,
                                                   drift_limit_source,
                                                   min_seconds,
                                                   timeout_seconds,
                                                   &replaced);
          if (result == ESP_OK) {
            if (replaced) {
              printf("cal capture replaced existing armed request\n");
            }
            printf("cal capture attached to running cal live session\n");
            printf("cal capture: evaluating existing buffered live data\n");
            printf("cal capture: if criteria are already satisfied, capture "
                   "will complete immediately\n");
            printf("cal capture: otherwise waiting continues using live "
                   "session data\n");
            return 0;
          }
        } else if (active_mode == CAL_CONSOLE_OP_NONE) {
          result = CalConsoleOpStartCapture(actual_temp_c,
                                            stable_stddev_c,
                                            drift_limit_enabled,
                                            drift_limit_c_per_min,
                                            drift_limit_source,
                                            min_seconds,
                                            timeout_seconds);
        } else {
          result = ESP_ERR_INVALID_STATE;
        }
      }
      if (result == ESP_ERR_INVALID_STATE) {
        printf("cal: operation already active\n");
        return 1;
      }
      if (result != ESP_OK) {
        printf("cal capture failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      printf("cal capture started (use 'cal stop' to abort)\n");
      return 0;
    }

    if (strcmp(action, "add") == 0) {
      double raw_c = 0.0;
      double actual_c = 0.0;
      if (!ParseCalAddPositionalArgs_(argc, argv, &raw_c, &actual_c)) {
        printf("usage: cal add <raw_c> <actual_c>\n");
        return 1;
      }
      if (settings->calibration_domain == CAL_DOMAIN_RESISTANCE_OHM) {
        printf(
          "cal add is legacy temp-domain only; use 'cal capture <actual_c>'\n");
        return 1;
      }

      const int32_t actual_mC = (int32_t)llround(actual_c * 1000.0);
      int point_index =
        FindCalibrationPointIndexByActualMc_(settings, actual_mC);
      const bool updating_existing = (point_index >= 0);
      if (!updating_existing &&
          settings->calibration_points_count >= CALIBRATION_MAX_POINTS) {
        printf("already have %u points; run 'cal apply' or 'cal clear'\n",
               (unsigned)settings->calibration_points_count);
        return 1;
      }

      if (!updating_existing) {
        point_index = (int)settings->calibration_points_count;
        settings->calibration_points_count++;
      }
      calibration_point_t* point = &settings->calibration_points[point_index];
      point->raw_avg_mC = (int32_t)llround(raw_c * 1000.0);
      point->actual_mC = actual_mC;
      point->raw_stddev_mC = 0;
      point->raw_avg_mOhm = 0;
      point->raw_stddev_mOhm = 0;
      point->sample_count = 0;
      point->time_valid = TimeSyncIsSystemTimeValid() ? 1u : 0u;
      point->timestamp_epoch_sec = point->time_valid ? (int64_t)time(NULL) : 0;
      point->captured_drift_mC_per_min =
        CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN;
      point->captured_delta_mC = CAL_CAPTURE_DELTA_UNAVAILABLE_MC;
      point->capture_drift_limit_mC_per_min =
        CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN;
      point->drift_limit_source =
        (uint8_t)CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE;
      point->captured_window_s = CAL_CAPTURE_WINDOW_S_UNAVAILABLE;
      point->captured_ema_alpha_permille =
        CAL_CAPTURE_EMA_ALPHA_UNAVAILABLE_PERMILLE;
      settings->calibration_domain = CAL_DOMAIN_TEMP_C;
      esp_err_t result = SaveCalibrationPointsAndDomain_(settings);
      if (result != ESP_OK) {
        printf("save failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      if (updating_existing) {
        printf("updated point %u: raw=%.6f actual=%.6f\n",
               (unsigned)(point_index + 1),
               raw_c,
               actual_c);
      } else {
        printf("added point %u: raw=%.6f actual=%.6f\n",
               (unsigned)(point_index + 1),
               raw_c,
               actual_c);
      }
      return 0;
    }

    if (strcmp(action, "import") == 0 || strcmp(action, "restore") == 0) {
      double raw_ohm = 0.0;
      double actual_c = 0.0;
      bool raw_c_supplied = false;
      bool stddev_c_supplied = false;
      double raw_c = 0.0;
      double stddev_c = 0.0;
      calibration_point_t imported_point = { 0 };
      bool parsed_status_line = false;
      const char* status_source = NULL;
      const bool parsed_numeric_mode = ParseCalImportArgs_(argc,
                                                           argv,
                                                           &raw_ohm,
                                                           &actual_c,
                                                           &raw_c_supplied,
                                                           &raw_c,
                                                           &stddev_c_supplied,
                                                           &stddev_c);
      if (parsed_numeric_mode) {
        ESP_LOGI(kTag, "cal import: mode=numeric argc=%d", argc);
      } else {
        size_t status_token_count = 0u;
        const char* payload_start = NULL;
        size_t payload_len = 0u;
        bool payload_quoted = false;
        bool unterminated_payload_quote = false;
        const size_t snapshot_len = (s_console_cmdline_snapshot != NULL)
                                      ? strlen(s_console_cmdline_snapshot)
                                      : 0u;
        const size_t mutable_len = (s_console_cmdline_buffer != NULL)
                                     ? strlen(s_console_cmdline_buffer)
                                     : 0u;
        bool raw_tail_available =
          CalibrationImportPayloadFromConsoleLine_(action,
                                                   &payload_start,
                                                   &payload_len,
                                                   &payload_quoted,
                                                   &unterminated_payload_quote);
        if (raw_tail_available && s_console_cmdline_snapshot != NULL) {
          ESP_LOGI(kTag,
                   "cal import debug: snapshot_line='%s'",
                   s_console_cmdline_snapshot);
          ESP_LOGI(kTag,
                   "cal import debug: payload='%.*s'",
                   (int)payload_len,
                   payload_start);
          ESP_LOGI(kTag,
                   "cal import debug: payload_len=%u quoted=%s",
                   (unsigned)payload_len,
                   payload_quoted ? "yes" : "no");
        }
        if (unterminated_payload_quote) {
          ESP_LOGI(kTag,
                   "cal import debug: unterminated quote in snapshot payload");
          printf("cal import failed: input appears truncated before parser "
                 "(transport RX limit or incomplete paste)\n");
          return 1;
        }
        bool parsed_raw_tail = false;
        if (raw_tail_available) {
          parsed_raw_tail = ParseCalibrationImportStatusLineSpan_(
            payload_start, payload_len, &imported_point, &status_token_count);
        }
        const bool parsed_argv_stream =
          (argc > 3) && ParseCalibrationImportStatusTokens_(
                          argc, argv, 2, &imported_point, &status_token_count);
        if (parsed_raw_tail || parsed_argv_stream) {
          parsed_status_line = true;
          status_source = parsed_raw_tail ? "snapshot-raw-tail" : "argv-stream";
          ESP_LOGI(kTag,
                   "cal import: mode=status-line source=%s quoted=%s argc=%d",
                   status_source,
                   (raw_tail_available && payload_quoted) ? "yes" : "no",
                   argc);
          ESP_LOGI(kTag,
                   "cal import debug: snapshot_available=%s snapshot_len=%u "
                   "mutable_len=%u",
                   (s_console_cmdline_snapshot != NULL) ? "yes" : "no",
                   (unsigned)snapshot_len,
                   (unsigned)mutable_len);
          ESP_LOGI(kTag,
                   "cal import debug: source=%s token_count=%u payload_len=%u",
                   status_source,
                   (unsigned)status_token_count,
                   (unsigned)payload_len);
          if (raw_tail_available) {
            ESP_LOGI(kTag,
                     "cal import debug: snapshot-raw-tail payload_len=%u "
                     "quoted=%s",
                     (unsigned)payload_len,
                     payload_quoted ? "yes" : "no");
          } else {
            ESP_LOGI(kTag,
                     "cal import debug: snapshot raw-tail unavailable; "
                     "falling back to argv-stream");
          }
          raw_ohm = imported_point.raw_avg_mOhm / 1000.0;
          actual_c = imported_point.actual_mC / 1000.0;
          raw_c_supplied = (imported_point.raw_avg_mC != INT32_MIN);
          stddev_c_supplied = (imported_point.raw_stddev_mC >= 0);
          raw_c = raw_c_supplied ? (imported_point.raw_avg_mC / 1000.0) : 0.0;
          stddev_c =
            stddev_c_supplied ? (imported_point.raw_stddev_mC / 1000.0) : 0.0;
        } else {
          ESP_LOGI(kTag,
                   "cal import debug: snapshot_available=%s snapshot_len=%u "
                   "mutable_len=%u",
                   (s_console_cmdline_snapshot != NULL) ? "yes" : "no",
                   (unsigned)snapshot_len,
                   (unsigned)mutable_len);
          if (raw_tail_available) {
            size_t cursor = 0u;
            const char* token_start = NULL;
            size_t token_len = 0u;
            bool truncation_detected = false;
            bool mangled_token_detected = false;
            while (NextCalibrationImportTokenSpan_(
              payload_start, payload_len, &cursor, &token_start, &token_len)) {
              const char* equals = memchr(token_start, '=', token_len);
              if (equals == NULL &&
                  !CalibrationImportTokenLooksLikeDisplayIndex_(token_start,
                                                                token_len) &&
                  CalibrationImportTokenIsKnownKeyPrefix_(token_start,
                                                          token_len)) {
                printf("cal import failed: status line appears truncated near "
                       "'%.*s'\n",
                       (int)token_len,
                       token_start);
                printf("cal import failed: input appears truncated before "
                       "parser (transport RX limit or incomplete paste)\n");
                ESP_LOGI(kTag,
                         "cal import debug: result=rejected reason=truncated "
                         "token='%.*s'",
                         (int)token_len,
                         token_start);
                truncation_detected = true;
                break;
              }
              if (equals == NULL &&
                  !CalibrationImportTokenLooksLikeDisplayIndex_(token_start,
                                                                token_len) &&
                  CalibrationImportTokenLooksMangledKey_(token_start,
                                                         token_len)) {
                printf("cal import failed: input appears truncated before "
                       "parser (transport RX limit or incomplete paste)\n");
                ESP_LOGI(kTag,
                         "cal import debug: parse failure near mangled token "
                         "'%.*s'",
                         (int)token_len,
                         token_start);
                mangled_token_detected = true;
                break;
              }
            }
            if (truncation_detected || mangled_token_detected) {
              return 1;
            }
          }
          ESP_LOGI(kTag,
                   "cal import debug: result=rejected reason=parse-failed "
                   "source=%s argc=%d",
                   raw_tail_available ? "raw-tail" : "argv-stream",
                   argc);
          printf("usage: cal import <raw_ohm> <actual_c> [--raw_c <value>] "
                 "[--stddev_c <value>] OR cal import <cal status point line> "
                 "(quoted or unquoted)\n");
          return 1;
        }
      }

      if (settings->calibration_domain == CAL_DOMAIN_TEMP_C &&
          settings->calibration_points_count > 0u) {
        printf("cal import failed: existing temp-domain points detected; run "
               "'cal clear' first\n");
        return 1;
      }

      const int32_t actual_mC = (int32_t)llround(actual_c * 1000.0);
      int point_index =
        FindCalibrationPointIndexByActualMc_(settings, actual_mC);
      const bool updating_existing = (point_index >= 0);
      if (!updating_existing &&
          settings->calibration_points_count >= CALIBRATION_MAX_POINTS) {
        printf("cal import failed: already have %u points\n",
               (unsigned)settings->calibration_points_count);
        return 1;
      }

      if (!updating_existing) {
        point_index = (int)settings->calibration_points_count;
        settings->calibration_points_count++;
      }
      calibration_point_t* point = &settings->calibration_points[point_index];
      point->raw_avg_mC =
        raw_c_supplied ? (int32_t)llround(raw_c * 1000.0) : INT32_MIN;
      point->actual_mC = actual_mC;
      point->raw_stddev_mC =
        stddev_c_supplied ? (int32_t)llround(stddev_c * 1000.0) : -1;
      point->raw_avg_mOhm = (int32_t)llround(raw_ohm * 1000.0);
      point->raw_stddev_mOhm = -1;
      point->sample_count = 0;
      point->time_valid = TimeSyncIsSystemTimeValid() ? 1u : 0u;
      point->timestamp_epoch_sec = point->time_valid ? (int64_t)time(NULL) : 0;
      point->captured_drift_mC_per_min =
        CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN;
      point->captured_delta_mC = CAL_CAPTURE_DELTA_UNAVAILABLE_MC;
      point->capture_drift_limit_mC_per_min =
        CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN;
      point->drift_limit_source =
        (uint8_t)CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE;
      point->captured_window_s = CAL_CAPTURE_WINDOW_S_UNAVAILABLE;
      point->captured_ema_alpha_permille =
        CAL_CAPTURE_EMA_ALPHA_UNAVAILABLE_PERMILLE;
      if (parsed_status_line) {
        point->raw_stddev_mOhm = imported_point.raw_stddev_mOhm;
        point->captured_drift_mC_per_min =
          imported_point.captured_drift_mC_per_min;
        point->captured_delta_mC = imported_point.captured_delta_mC;
        point->capture_drift_limit_mC_per_min =
          imported_point.capture_drift_limit_mC_per_min;
        point->drift_limit_source = imported_point.drift_limit_source;
        point->captured_window_s = imported_point.captured_window_s;
        point->captured_ema_alpha_permille =
          imported_point.captured_ema_alpha_permille;
      }
      settings->calibration_domain = CAL_DOMAIN_RESISTANCE_OHM;
      esp_err_t result = SaveCalibrationPointsAndDomain_(settings);
      if (result != ESP_OK) {
        printf("save failed: %s\n", esp_err_to_name(result));
        return 1;
      }

      if (updating_existing) {
        printf(
          "cal import OK: updated point #%u actual=%.3fC raw_avg_Ohm=%.3f\n",
          (unsigned)(point_index + 1),
          actual_c,
          raw_ohm);
        if (parsed_status_line) {
          ESP_LOGI(kTag,
                   "cal import debug: result=updated point_index=%u source=%s",
                   (unsigned)(point_index + 1),
                   (status_source != NULL) ? status_source : "unknown");
        }
      } else {
        printf("cal import OK: added point #%u actual=%.3fC raw_avg_Ohm=%.3f\n",
               (unsigned)(point_index + 1),
               actual_c,
               raw_ohm);
        if (parsed_status_line) {
          ESP_LOGI(kTag,
                   "cal import debug: result=added point_index=%u source=%s",
                   (unsigned)(point_index + 1),
                   (status_source != NULL) ? status_source : "unknown");
        }
      }
      if (!raw_c_supplied) {
        printf("cal import note: --raw_c not supplied (stored as n/a)\n");
      }
      if (!stddev_c_supplied) {
        printf("cal import note: --stddev_c not supplied (stored as n/a)\n");
      }
      return 0;
    }

    if (strcmp(action, "del") == 0 || strcmp(action, "remove") == 0) {
      if (argc != 3) {
        printf("usage: cal del <index>\n");
        return 1;
      }
      char* end = NULL;
      long display_index_long = strtol(argv[2], &end, 10);
      if (end == argv[2] || *end != '\0' || display_index_long <= 0 ||
          display_index_long > INT_MAX) {
        printf("usage: cal del <index>\n");
        return 1;
      }
      calibration_point_t deleted_point = { 0 };
      esp_err_t delete_result = DeleteCalibrationPointByDisplayIndex_(
        settings, (size_t)display_index_long, &deleted_point);
      if (delete_result == ESP_ERR_NOT_FOUND) {
        printf("cal del failed: index %ld out of range (1..%u)\n",
               display_index_long,
               (unsigned)settings->calibration_points_count);
        return 1;
      }
      if (delete_result != ESP_OK) {
        printf("cal del failed: %s\n", esp_err_to_name(delete_result));
        return 1;
      }
      esp_err_t save_result = SaveCalibrationPointsAndDomain_(settings);
      if (save_result != ESP_OK) {
        printf("save failed: %s\n", esp_err_to_name(save_result));
        return 1;
      }
      printf("deleted point %ld: raw=%.6f actual=%.6f; %u point(s) remain\n",
             display_index_long,
             deleted_point.raw_avg_mC / 1000.0,
             deleted_point.actual_mC / 1000.0,
             (unsigned)settings->calibration_points_count);
      return 0;
    }
  }

  int errors = arg_parse(argc, argv, (void**)&g_cal_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_cal_args.end, argv[0]);
    return 1;
  }
  const char* action = g_cal_args.action->sval[0];

  if (strcmp(action, "clear") == 0) {
    CalibrationModelInitIdentity(&settings->calibration);
    settings->calibration_points_count = 0;
    settings->calibration_domain = CAL_DOMAIN_TEMP_C;
    memset(
      settings->calibration_points, 0, sizeof(settings->calibration_points));
    esp_err_t result = SaveCalibrationWithContext(&settings->calibration);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    result = AppSettingsSaveCalibrationPoints(
      settings->calibration_points, settings->calibration_points_count);
    if (result == ESP_OK) {
      result = AppSettingsSaveCalibrationDomain(settings->calibration_domain);
    }
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("calibration reset to identity (y=x)\n");
    return 0;
  }

  if (strcmp(action, "list") == 0) {
    printf("calibration points (%u):\n",
           (unsigned)settings->calibration_points_count);
    for (size_t index = 0; index < settings->calibration_points_count;
         ++index) {
      const calibration_point_t* point = &settings->calibration_points[index];
      char raw_avg_c_buf[24] = { 0 };
      char stddev_c_buf[24] = { 0 };
      if (point->raw_avg_mC == INT32_MIN) {
        snprintf(raw_avg_c_buf, sizeof(raw_avg_c_buf), "n/a");
      } else {
        snprintf(raw_avg_c_buf,
                 sizeof(raw_avg_c_buf),
                 "%.6f",
                 point->raw_avg_mC / 1000.0);
      }
      if (point->raw_stddev_mC < 0) {
        snprintf(stddev_c_buf, sizeof(stddev_c_buf), "n/a");
      } else {
        snprintf(stddev_c_buf,
                 sizeof(stddev_c_buf),
                 "%.6f",
                 point->raw_stddev_mC / 1000.0);
      }
      printf("  %u: raw_avg_C=%s actual_C=%.6f stddev_C=%s raw_avg_Ohm=%.6f "
             "stddev_Ohm=%.6f samples=%u\n",
             (unsigned)(index + 1),
             raw_avg_c_buf,
             point->actual_mC / 1000.0,
             stddev_c_buf,
             point->raw_avg_mOhm / 1000.0,
             point->raw_stddev_mOhm / 1000.0,
             (unsigned)point->sample_count);
    }
    return 0;
  }

  if (strcmp(action, "apply") == 0) {
    if (settings->calibration_points_count < 1) {
      printf("no points; use 'cal capture <actual_c>' or "
             "'cal import <raw_ohm> <actual_c>' first\n");
      return 1;
    }
    if (settings->cal_method[0] == '\0') {
      printf("warning: calibration method is unset; PTLOG headers will record "
             "<unset>.\n");
      printf(
        "         set it now with: cal set method \"ice + satpt steam\"\n");
    }
    if (!TimeSyncIsSystemTimeValid()) {
      printf("warning: system time is invalid; apply will not update Last UTC "
             "until time is valid.\n");
    }

    bool have_resistance_points = false;
    bool have_temp_only_points = false;
    for (size_t i = 0; i < settings->calibration_points_count; ++i) {
      const calibration_point_t* point = &settings->calibration_points[i];
      if (point->raw_avg_mOhm > 0) {
        have_resistance_points = true;
      } else {
        have_temp_only_points = true;
      }
    }
    if (have_resistance_points && have_temp_only_points) {
      printf("cal apply failed: mixed-domain point set (temp + resistance). "
             "Use cal clear and rebuild a single-domain set.\n");
      return 1;
    }

    calibration_domain_t apply_domain =
      have_resistance_points ? CAL_DOMAIN_RESISTANCE_OHM : CAL_DOMAIN_TEMP_C;

    calibration_point_t fit_points[CALIBRATION_MAX_POINTS] = { 0 };
    esp_err_t result =
      CalibrationBuildFitDomainPoints(settings->calibration_points,
                                      settings->calibration_points_count,
                                      apply_domain,
                                      g_runtime->sensor->rtd_nominal_ohm,
                                      fit_points);
    if (result != ESP_OK) {
      printf("cal apply failed: could not build fit-domain points (%s)\n",
             esp_err_to_name(result));
      return 1;
    }

    calibration_model_t model;
    calibration_fit_options_t options;
    CalibrationFitOptionsInitDefault(&options);
    if (g_cal_args.mode->count > 0) {
      const char* mode = g_cal_args.mode->sval[0];
      if (strcmp(mode, "linear") == 0) {
        options.mode = CAL_FIT_MODE_LINEAR;
      } else if (strcmp(mode, "piecewise") == 0) {
        options.mode = CAL_FIT_MODE_PIECEWISE;
      } else if (strncmp(mode, "poly", 4) == 0) {
        const int degree = atoi(mode + 4);
        if (degree < 1 || degree > CALIBRATION_MAX_DEGREE) {
          printf("invalid poly degree; use poly1..poly%u\n",
                 CALIBRATION_MAX_DEGREE);
          return 1;
        }
        options.mode = CAL_FIT_MODE_POLY;
        options.poly_degree = (uint8_t)degree;
      } else {
        printf("invalid mode; use linear|piecewise|polyN\n");
        return 1;
      }
    }
    options.allow_wide_slope = (g_cal_args.allow_wide_slope->count > 0);
    if (apply_domain == CAL_DOMAIN_RESISTANCE_OHM) {
      options.min_slope = 0.9;
      options.max_slope = 1.1;
      options.guard_min_c = 80.0;
      options.guard_max_c = 180.0;
      options.max_abs_correction_c = 10.0;
    }

    calibration_fit_diagnostics_t diagnostics = { 0 };
    result = CalibrationModelFitFromPointsWithOptions(
      fit_points,
      settings->calibration_points_count,
      &options,
      &model,
      &diagnostics);
    if (result != ESP_OK) {
      printf("fit failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    calibration_fit_report_t fit_report;
    result =
      CalibrationBuildFitReport(settings->calibration_points,
                                settings->calibration_points_count,
                                apply_domain,
                                &model,
                                g_runtime->sensor->rtd_nominal_ohm,
                                ConsoleResistanceToTemperatureFitCallback_,
                                g_runtime->sensor,
                                &fit_report);
    if (result != ESP_OK) {
      printf("fit report failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    diagnostics = fit_report.summary;

    settings->calibration = model;
    settings->calibration_domain = apply_domain;
    result = SaveCalibrationWithContext(&model);
    if (result == ESP_OK) {
      result = AppSettingsSaveCalibrationDomain(settings->calibration_domain);
    }
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }

    printf("calibration applied: domain=%s mode=%s degree=%u coeffs=[%.9g, "
           "%.9g, %.9g, "
           "%.9g]\n",
           (apply_domain == CAL_DOMAIN_RESISTANCE_OHM) ? "resistance"
                                                       : "legacy_temp",
           CalibrationModeToString(model.mode),
           (unsigned)model.degree,
           model.coefficients[0],
           model.coefficients[1],
           model.coefficients[2],
           model.coefficients[3]);
    printf("fit diagnostics:\n");
    PrintCalibrationFitDiagnosticsBlock_(&diagnostics);
    ForcePtlogRevisionForCalibrationMetadata("cal apply");

    return 0;
  }

  printf("unknown action. usage: cal status | cal cfg show | cal cfg set "
         "ema_alpha <value> | cal cfg set window_s <seconds> | "
         "cal set date <YYYY-MM-DD> | "
         "cal clear date | cal set due <count> <days|months|years> | "
         "cal clear due | cal set due_override <count> <days|months|years> | "
         "cal clear due_override | cal set method <string...> | "
         "cal clear method | cal clear | cal add <raw_c> <actual_c> | "
         "cal metar set <elevation_ft> \"<raw METAR>\" | cal metar show | "
         "cal metar clear | "
         "cal import <raw_ohm> <actual_c> [--raw_c <value>] "
         "[--stddev_c <value>] | cal import \"<cal status point line>\" | "
         "cal restore ... | "
         "cal del <index> | cal remove <index> | "
         "cal list | cal apply | cal live [seconds] "
         "[--every_ms 1000] [--drift_c_per_min 0.020] | "
         "cal livecal [seconds] [--every_ms 1000] [--drift_c_per_min 0.020] | "
         "cal capture <actual_temp_c> "
         "[--stable_stddev_c 0.05] [--min_seconds 5] "
         "[--timeout_seconds 120] [--drift_c_per_min 0.020] "
         "[--no-drift-limit] | cal stop\n");
  return 1;
}

/**
 * @brief Execute CommandMode.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandMode(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_mode_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_mode_args.end, argv[0]);
    return 1;
  }

  const char* action = g_mode_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    const app_boot_mode_t stored = BootModeReadFromNvsOrDefault();
    const app_boot_mode_t running = RuntimeIsDataStreamingEnabled()
                                      ? APP_BOOT_MODE_RUN
                                      : APP_BOOT_MODE_DIAGNOSTICS;
    printf("nvs_boot_mode: %s\n", BootModeToString(stored));
    printf("current_mode: %s\n", BootModeToString(running));
    printf("data_streaming: %s\n",
           RuntimeIsDataStreamingEnabled() ? "on" : "off");
    return 0;
  }

  if (strcmp(action, "run") == 0 || strcmp(action, "diag") == 0) {
    const bool run_mode = strcmp(action, "run") == 0;
    if (run_mode) {
      RuntimeSetLogPolicyRun();
      RuntimeEnableDataStreaming(true);
    } else {
      RuntimeSetLogPolicyDiag();
      RuntimeEnableDataStreaming(false);
    }
    printf("mode set to %s\n", run_mode ? "run" : "diag");
    return 0;
  }

  app_boot_mode_t target = APP_BOOT_MODE_DIAGNOSTICS;
  if (strcmp(action, "set") == 0 && g_mode_args.mode_value->count == 1) {
    const char* mode = g_mode_args.mode_value->sval[0];
    if (strcmp(mode, "diag") == 0) {
      target = APP_BOOT_MODE_DIAGNOSTICS;
    } else if (strcmp(mode, "run") == 0) {
      target = APP_BOOT_MODE_RUN;
    } else {
      printf("usage: mode set diag|run\n");
      return 1;
    }
    esp_err_t result = BootModeWriteToNvs(target);
    if (result != ESP_OK) {
      printf("write failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("set; reboot required\n");
    return 0;
  }

  printf("unknown action. usage: mode show | mode run | mode diag | mode set "
         "diag|run\n");
  return 1;
}

/**
 * @brief Execute CommandData.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandData(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_data_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_data_args.end, argv[0]);
    return 1;
  }

  const char* action = g_data_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("data_streaming: %s\n",
           RuntimeIsDataStreamingEnabled() ? "on" : "off");
    DataPortBackend data_stream_backend = RuntimeGetDataStreamBackend();
    printf("data_stream_backend: %s\n",
           DataPortBackendToString(data_stream_backend));
    if (data_stream_backend == DATA_PORT_BACKEND_UART0) {
      int32_t uart0_tx_gpio = -1;
      int32_t uart0_rx_gpio = -1;
      DataPortGetUart0Pins(&uart0_tx_gpio, &uart0_rx_gpio);
      printf("data_stream_uart0_tx_gpio: %ld\n", (long)uart0_tx_gpio);
      printf("data_stream_uart0_rx_gpio: %ld\n", (long)uart0_rx_gpio);
    }
    printf("data_stream_init_err: %s\n",
           esp_err_to_name(RuntimeGetDataStreamInitError()));
    return 0;
  }

  if (strcmp(action, "on") == 0) {
    RuntimeEnableDataStreaming(true);
    printf("data streaming enabled\n");
    return 0;
  }

  if (strcmp(action, "off") == 0) {
    RuntimeEnableDataStreaming(false);
    printf("data streaming disabled\n");
    return 0;
  }

  printf("unknown action. usage: data show | data on | data off\n");
  return 1;
}

/**
 * @brief Execute CommandRun.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRun(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_run_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_run_args.end, argv[0]);
    return 1;
  }

  const char* action = g_run_args.action->sval[0];

  if (strcmp(action, "status") == 0) {
    printf("running: %s\n", RuntimeIsRunning() ? "yes" : "no");
    return 0;
  }

  if (strcmp(action, "start") == 0) {
    if (RuntimeIsRunning()) {
      printf("already running\n");
      return 0;
    }
    esp_err_t result = EnterRunMode();
    if (result != ESP_OK) {
      printf("start failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    const runtime_cached_status_t* status = RuntimeGetCachedStatus();
    if (status != NULL) {
      printf("drain: flushed=%ld remaining=%u duration=%u ms result=%s\n",
             status->last_drain_flushed_records,
             (unsigned)status->last_drain_remaining,
             (unsigned)status->last_drain_duration_ms,
             esp_err_to_name((esp_err_t)status->last_drain_result));
      if (status->last_drain_result == ESP_ERR_TIMEOUT) {
        printf("drain timed out; remaining=%u\n",
               (unsigned)status->last_drain_remaining);
      }
    }
    printf("runtime started\n");
    return 0;
  }

  if (strcmp(action, "stop") == 0) {
    if (!RuntimeIsRunning()) {
      printf("already stopped\n");
      return 0;
    }
    esp_err_t result = EnterDiagMode();
    if (result != ESP_OK) {
      printf("stop failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    const runtime_cached_status_t* status = RuntimeGetCachedStatus();
    if (status != NULL) {
      printf("drain: flushed=%ld remaining=%u duration=%u ms result=%s\n",
             status->last_drain_flushed_records,
             (unsigned)status->last_drain_remaining,
             (unsigned)status->last_drain_duration_ms,
             esp_err_to_name((esp_err_t)status->last_drain_result));
      if (status->last_drain_result == ESP_ERR_TIMEOUT) {
        printf("drain timed out; remaining=%u\n",
               (unsigned)status->last_drain_remaining);
      }
    }
    printf("runtime stopped\n");
    return 0;
  }

  printf("unknown action. usage: run status | run start | run stop\n");
  return 1;
}

/**
 * @brief Execute CommandTz.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandTz(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_tz_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_tz_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_tz_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("tz_posix: %s\n", g_runtime->settings->tz_posix);
    printf("dst_enabled: %s\n",
           g_runtime->settings->dst_enabled ? "yes" : "no");
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_tz_args.posix->count != 1) {
      printf("usage: tz set \"<posix>\"\n");
      return 1;
    }
    const char* tz_posix = g_tz_args.posix->sval[0];
    if (tz_posix[0] == '\0' ||
        strlen(tz_posix) >= sizeof(g_runtime->settings->tz_posix)) {
      printf("invalid tz string\n");
      return 1;
    }
    snprintf(g_runtime->settings->tz_posix,
             sizeof(g_runtime->settings->tz_posix),
             "%s",
             tz_posix);
    g_runtime->settings->dst_enabled = (strchr(tz_posix, ',') != NULL);
    esp_err_t result = AppSettingsSaveTimeZone(
      g_runtime->settings->tz_posix, g_runtime->settings->dst_enabled);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    AppSettingsApplyTimeZone(g_runtime->settings);
    printf("tz_posix set to %s\n", g_runtime->settings->tz_posix);
    return 0;
  }

  printf("unknown action. usage: tz show | tz set \"<posix>\"\n");
  return 1;
}

static void
PrintTimeShow(const app_runtime_t* runtime)
{
  if (runtime == NULL || runtime->settings == NULL) {
    return;
  }

  tzset();

  const time_t now_seconds = time(NULL);
  struct tm utc_tm = { 0 };
  char utc_buffer[32] = { 0 };
  if (gmtime_r(&now_seconds, &utc_tm) != NULL) {
    strftime(utc_buffer, sizeof(utc_buffer), "%Y-%m-%d %H:%M:%SZ", &utc_tm);
  }

  struct tm local_tm = { 0 };
  int local_isdst = -1;
  char local_buffer[48] = { 0 };
  if (localtime_r(&now_seconds, &local_tm) != NULL) {
    strftime(local_buffer,
             sizeof(local_buffer),
             "%Y-%m-%d %H:%M:%S %Z (%z)",
             &local_tm);
    local_isdst = local_tm.tm_isdst;
  }

  printf("utc_time: %s (epoch=%ld)\n",
         (utc_buffer[0] != '\0') ? utc_buffer : "unknown",
         (long)now_seconds);
  printf("local_time: %s\n",
         (local_buffer[0] != '\0') ? local_buffer : "unknown");
  printf("time_valid: %s\n", TimeSyncIsSystemTimeValid() ? "yes" : "no");
  printf("tz_posix: %s\n", runtime->settings->tz_posix);
  printf("dst_enabled: %s\n", runtime->settings->dst_enabled ? "yes" : "no");
  printf("local_isdst: %d\n", local_isdst);

  time_sntp_status_t sntp_status;
  TimeSyncGetSntpStatus(&sntp_status);
  printf("sntp_server: %s\n",
         (sntp_status.last_server[0] != '\0') ? sntp_status.last_server
                                              : "n/a");
  if (sntp_status.last_attempt_epoch > 0) {
    char attempt_buffer[32] = { 0 };
    FormatUtcEpochIso8601(
      sntp_status.last_attempt_epoch, attempt_buffer, sizeof(attempt_buffer));
    printf("sntp_last_attempt: %s (epoch=%" PRId64 ")\n",
           attempt_buffer,
           sntp_status.last_attempt_epoch);
  } else {
    printf("sntp_last_attempt: never\n");
  }
  printf("sntp_last_result: %s (%d)\n",
         esp_err_to_name(sntp_status.last_result),
         (int)sntp_status.last_result);
  if (sntp_status.last_success_epoch > 0) {
    char success_buffer[32] = { 0 };
    FormatUtcEpochIso8601(
      sntp_status.last_success_epoch, success_buffer, sizeof(success_buffer));
    printf("sntp_last_success: %s (epoch=%" PRId64 ")\n",
           success_buffer,
           sntp_status.last_success_epoch);
  } else {
    printf("sntp_last_success: never\n");
  }

  const bool rtc_present =
    (runtime->time_sync != NULL && runtime->time_sync->is_ds3231_ready);
  printf("rtc_present: %s\n", rtc_present ? "yes" : "no");

  int64_t rtc_epoch = 0;
  if (rtc_present &&
      TimeSyncReadRtcEpoch(runtime->time_sync, &rtc_epoch) == ESP_OK) {
    char rtc_buffer[32] = { 0 };
    FormatUtcEpochIso8601(rtc_epoch, rtc_buffer, sizeof(rtc_buffer));
    const int64_t drift_seconds = (int64_t)now_seconds - rtc_epoch;
    printf("rtc_time_utc: %s (epoch=%" PRId64 ")\n", rtc_buffer, rtc_epoch);
    printf("rtc_drift_seconds: %" PRId64 "\n", drift_seconds);
  } else {
    printf("rtc_time_utc: n/a\n");
    printf("rtc_drift_seconds: n/a\n");
  }

  const int64_t rtc_last_set_epoch = TimeSyncGetLastRtcSetEpoch();
  if (rtc_last_set_epoch > 0) {
    char set_buffer[32] = { 0 };
    FormatUtcEpochIso8601(rtc_last_set_epoch, set_buffer, sizeof(set_buffer));
    printf(
      "rtc_last_set: %s (epoch=%" PRId64 ")\n", set_buffer, rtc_last_set_epoch);
  } else {
    printf("rtc_last_set: never\n");
  }
}

/**
 * @brief Execute PrintTimeUsage.
 */
static void
PrintTimeUsage(void)
{
  printf("time show\n");
  printf("time setlocal \"YYYY-MM-DD HH:MM:SS\" [--is_dst 0|1]\n");
  printf(
    "  input is LOCAL wall time; converted to UTC epoch + RTC stored as UTC\n");
  printf("  local formatting uses current TZ rules (POSIX TZ string via "
         "settings)\n");
  printf("  use --is_dst to disambiguate fall-back hour\n");
}

/**
 * @brief Execute CommandTime.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandTime(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_time_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_time_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_time_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    if (g_time_args.local_time->count != 0 || g_time_args.is_dst->count != 0) {
      PrintTimeUsage();
      return 1;
    }
    PrintTimeShow(g_runtime);
    return 0;
  }

  if (strcmp(action, "setlocal") != 0) {
    PrintTimeUsage();
    return 1;
  }

  if (g_time_args.local_time->count != 1) {
    PrintTimeUsage();
    return 1;
  }

  struct tm tm_local;
  esp_err_t result =
    TimeParseLocalIso(g_time_args.local_time->sval[0], &tm_local);
  if (result != ESP_OK) {
    printf("invalid time format (use YYYY-MM-DD HH:MM:SS)\n");
    return 1;
  }

  if (g_time_args.is_dst->count == 1) {
    const int is_dst = g_time_args.is_dst->ival[0];
    if (is_dst != 0 && is_dst != 1) {
      PrintTimeUsage();
      return 1;
    }
    tm_local.tm_isdst = is_dst;
  }

  time_t epoch_utc = 0;
  bool ambiguous = false;
  result = TimeLocalTmToEpochUtc(&tm_local, &epoch_utc, &ambiguous);
  if (result == ESP_ERR_NOT_SUPPORTED && ambiguous) {
    printf("ambiguous local time; use --is_dst 0|1\n");
    return 1;
  }
  if (result == ESP_ERR_INVALID_STATE) {
    printf("invalid local time (DST gap)\n");
    return 1;
  }
  if (result != ESP_OK) {
    printf("time conversion failed: %s\n", esp_err_to_name(result));
    return 1;
  }

  (void)TimeSyncSetSystemEpoch((int64_t)epoch_utc, false, g_runtime->time_sync);

  bool rtc_ok = false;
  if (g_runtime->time_sync != NULL) {
    rtc_ok = (TimeSyncSetRtcFromSystem(g_runtime->time_sync) == ESP_OK);
  }

  if (g_runtime->mesh != NULL &&
      g_runtime->settings->node_role == APP_NODE_ROLE_ROOT) {
    const esp_err_t mesh_result =
      MeshTransportBroadcastTime(g_runtime->mesh, (int64_t)epoch_utc);
    if (mesh_result != ESP_OK) {
      ESP_LOGW(
        kTag, "mesh time broadcast failed: %s", esp_err_to_name(mesh_result));
    }
  }

  struct tm local_time;
  char local_buffer[32] = { 0 };
  if (localtime_r(&epoch_utc, &local_time) != NULL) {
    strftime(
      local_buffer, sizeof(local_buffer), "%Y-%m-%d %H:%M:%S", &local_time);
  }
  printf("time setlocal ok: local=%s utc_epoch=%" PRId64 " rtc=%s\n",
         (local_buffer[0] != '\0') ? local_buffer : "unknown",
         (int64_t)epoch_utc,
         rtc_ok ? "ok" : "fail");
  return 0;
}

/**
 * @brief Execute PrintRtcUsage.
 */
static void
PrintRtcUsage(void)
{
  printf("rtc status\n");
  printf("rtc set period_ms <0..86400000>\n");
}

/**
 * @brief Execute CommandRtc.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRtc(int argc, char** argv)
{
  if (g_runtime == NULL || g_runtime->settings == NULL) {
    return 1;
  }
  if (argc < 2) {
    PrintRtcUsage();
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "status") == 0) {
    const runtime_state_t* state = RuntimeGetState();
    printf("rtc_resync_period_ms: %u\n",
           (unsigned)g_runtime->settings->rtc_resync_period_ms);
    printf("time_jump_back_arm_next: %s\n",
           (state != NULL && state->time_jump_back_arm_next) ? "yes" : "no");
    printf("time_jump_back_pending_confirm: %s\n",
           (state != NULL && state->time_jump_back_pending_confirm) ? "yes"
                                                                    : "no");
    if (state != NULL && state->time_jump_back_pending_confirm) {
      printf("time_jump_back_attempt_record_id: %" PRIu64 "\n",
             state->time_jump_back_attempt_record_id);
    } else {
      printf("time_jump_back_attempt_record_id: n/a\n");
    }
    if (state != NULL && state->last_time_jump_back_delta_sec != 0) {
      printf("last_time_jump_back_delta_sec: %" PRId64 "\n",
             state->last_time_jump_back_delta_sec);
    } else {
      printf("last_time_jump_back_delta_sec: n/a\n");
    }

    const int64_t system_epoch = (int64_t)time(NULL);
    printf("system_epoch_utc: %" PRId64 "\n", system_epoch);
    int64_t rtc_epoch = 0;
    if (g_runtime->time_sync != NULL &&
        TimeSyncReadRtcEpoch(g_runtime->time_sync, &rtc_epoch) == ESP_OK) {
      printf("rtc_epoch_utc: %" PRId64 "\n", rtc_epoch);
      printf("rtc_delta_seconds: %" PRId64 "\n", rtc_epoch - system_epoch);
    } else {
      printf("rtc_epoch_utc: n/a\n");
      printf("rtc_delta_seconds: n/a\n");
    }
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (argc != 4 || strcmp(argv[2], "period_ms") != 0) {
      PrintRtcUsage();
      return 1;
    }
    char* end = NULL;
    const unsigned long value = strtoul(argv[3], &end, 10);
    if (end == argv[3] || *end != '\0' || value > 86400000ul) {
      PrintRtcUsage();
      return 1;
    }
    g_runtime->settings->rtc_resync_period_ms = (uint32_t)value;
    const esp_err_t result = AppSettingsSaveRtcResyncPeriodMs((uint32_t)value);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("rtc_resync_period_ms set to %lu\n", value);
    return 0;
  }

  PrintRtcUsage();
  return 1;
}

// --- Water saturation point calculator
// ----------------------------------------
/**
 * @brief Execute StationPressureFromSlpInHg.
 * @param slp_inHg Parameter slp_inHg.
 * @param elev_ft Parameter elev_ft.
 * @return Return the function result.
 */
static float
StationPressureFromSlpInHg(float slp_inHg, float elev_ft)
{
  if (slp_inHg <= 0.0f) {
    return NAN;
  }
  const float height_m = (elev_ft <= 0.0f) ? 0.0f : (elev_ft * 0.3048f);
  const float base = 1.0f - 2.25577e-5f * height_m;
  if (base <= 0.0f) {
    return NAN; // Out of model range.
  }
  return slp_inHg * powf(base, 5.25588f);
}

// Antoine equation (water), pressure in mmHg -> boiling point in °C.
// Validity is approximate; this is intended for quick calibration reference.
/**
 * @brief Execute BoilingPointCFromStationInHg.
 * @param station_inHg Parameter station_inHg.
 * @return Return the function result.
 */
static float
SaturationTempCFromStationInHg(float station_inHg)
{
  if (station_inHg <= 0.0f) {
    return NAN;
  }
  const float pressure_mmHg = station_inHg * 25.4f; // 1 inHg = 25.4 mmHg
  if (pressure_mmHg <= 0.0f) {
    return NAN;
  }

  const float A = 8.07131f;
  const float B = 1730.63f;
  const float C = 233.426f;

  const float log_p = log10f(pressure_mmHg);
  const float denom = A - log_p;
  if (denom == 0.0f) {
    return NAN;
  }
  return B / denom - C;
}

/**
 * @brief Parse METAR altimeter token as inches of mercury.
 * @param token Input token, either A#### or decimal inHg.
 * @param altimeter_inHg_out Parsed value output.
 * @return true on success.
 */
static bool
ParseMetarAltimeterInHg(const char* token, float* altimeter_inHg_out)
{
  if (token == NULL || altimeter_inHg_out == NULL || token[0] == '\0') {
    return false;
  }

  if ((token[0] == 'A' || token[0] == 'a') && strlen(token) == 5U) {
    for (size_t i = 1; i < 5; ++i) {
      if (!isdigit((unsigned char)token[i])) {
        return false;
      }
    }
    *altimeter_inHg_out = (float)strtol(token + 1, NULL, 10) / 100.0f;
    return *altimeter_inHg_out > 0.0f;
  }

  char* endptr = NULL;
  const float parsed_inHg = strtof(token, &endptr);
  if (endptr == token || *endptr != '\0' || isnan(parsed_inHg) ||
      parsed_inHg <= 0.0f) {
    return false;
  }
  *altimeter_inHg_out = parsed_inHg;
  return true;
}

static bool
MetarTokenLooksLikeStation_(const char* token)
{
  if (token == NULL) {
    return false;
  }
  const size_t len = strlen(token);
  if (len < 4 || len > 7) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    if (!isalnum((unsigned char)token[i])) {
      return false;
    }
    if (isalpha((unsigned char)token[i]) && !isupper((unsigned char)token[i])) {
      return false;
    }
  }
  return true;
}

static bool
MetarTokenLooksLikeObservation_(const char* token)
{
  if (token == NULL || strlen(token) != 7U || token[6] != 'Z') {
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    if (!isdigit((unsigned char)token[i])) {
      return false;
    }
  }
  const int day = (token[0] - '0') * 10 + (token[1] - '0');
  const int hour = (token[2] - '0') * 10 + (token[3] - '0');
  const int minute = (token[4] - '0') * 10 + (token[5] - '0');
  return day >= 1 && day <= 31 && hour >= 0 && hour <= 23 && minute >= 0 &&
         minute <= 59;
}

static bool
MetarTokenLooksLikeTempDew_(const char* token)
{
  if (token == NULL) {
    return false;
  }
  const char* slash = strchr(token, '/');
  if (slash == NULL || slash == token || slash[1] == '\0') {
    return false;
  }
  return true;
}

static bool
ResolveMetarObservationIsoUtc_(const char* token,
                               bool system_time_valid,
                               int64_t now_epoch_utc,
                               char* iso_out,
                               size_t iso_out_size,
                               bool* resolved_out)
{
  if (iso_out == NULL || iso_out_size == 0 || resolved_out == NULL) {
    return false;
  }
  *resolved_out = false;
  iso_out[0] = '\0';
  if (!system_time_valid || now_epoch_utc <= 0 ||
      !MetarTokenLooksLikeObservation_(token)) {
    return true;
  }

  struct tm now_tm = { 0 };
  time_t now_time = (time_t)now_epoch_utc;
  if (gmtime_r(&now_time, &now_tm) == NULL) {
    return true;
  }

  const int obs_day = (token[0] - '0') * 10 + (token[1] - '0');
  const int obs_hour = (token[2] - '0') * 10 + (token[3] - '0');
  const int obs_minute = (token[4] - '0') * 10 + (token[5] - '0');
  int64_t best_delta_seconds = INT64_MAX;
  int64_t best_epoch = 0;

  for (int month_offset = -1; month_offset <= 1; ++month_offset) {
    int year = now_tm.tm_year + 1900;
    int month = (now_tm.tm_mon + 1) + month_offset;
    while (month < 1) {
      month += 12;
      --year;
    }
    while (month > 12) {
      month -= 12;
      ++year;
    }
    int64_t day_epoch = 0;
    if (!TimeCivilUtcEpochFromDate(year, month, obs_day, &day_epoch)) {
      continue;
    }
    const int64_t candidate =
      day_epoch + (int64_t)obs_hour * 3600LL + (int64_t)obs_minute * 60LL;
    const int64_t delta = llabs(candidate - now_epoch_utc);
    if (delta < best_delta_seconds) {
      best_delta_seconds = delta;
      best_epoch = candidate;
    }
  }

  if (best_epoch <= 0) {
    return true;
  }
  FormatUtcEpochIso8601(best_epoch, iso_out, iso_out_size);
  *resolved_out = true;
  return true;
}

static bool
ParseMetarLine_(const char* metar_line,
                calibration_metar_reference_t* metar_out,
                char* error_out,
                size_t error_out_size)
{
  if (error_out != NULL && error_out_size > 0) {
    error_out[0] = '\0';
  }
  if (metar_line == NULL || metar_out == NULL) {
    return false;
  }
  if (strlen(metar_line) >= APP_SETTINGS_CAL_METAR_RAW_MAX_LEN) {
    if (error_out != NULL && error_out_size > 0) {
      snprintf(error_out,
               error_out_size,
               "METAR line too long (max %u chars)",
               (unsigned)(APP_SETTINGS_CAL_METAR_RAW_MAX_LEN - 1));
    }
    return false;
  }

  memset(metar_out, 0, sizeof(*metar_out));
  snprintf(metar_out->source_type, sizeof(metar_out->source_type), "METAR");
  snprintf(
    metar_out->raw_metar, sizeof(metar_out->raw_metar), "%s", metar_line);

  char working[APP_SETTINGS_CAL_METAR_RAW_MAX_LEN] = { 0 };
  snprintf(working, sizeof(working), "%s", metar_line);

  char* saveptr = NULL;
  char* token = strtok_r(working, " \t", &saveptr);
  bool have_station = false;
  bool have_obs = false;
  bool have_altimeter = false;
  bool in_remarks = false;
  while (token != NULL) {
    if (strcmp(token, "METAR") == 0 || strcmp(token, "SPECI") == 0) {
      token = strtok_r(NULL, " \t", &saveptr);
      continue;
    }
    if (!have_station && MetarTokenLooksLikeStation_(token)) {
      snprintf(
        metar_out->station_id, sizeof(metar_out->station_id), "%s", token);
      have_station = true;
      token = strtok_r(NULL, " \t", &saveptr);
      continue;
    }
    if (!have_obs && MetarTokenLooksLikeObservation_(token)) {
      snprintf(metar_out->observation_token,
               sizeof(metar_out->observation_token),
               "%s",
               token);
      have_obs = true;
      token = strtok_r(NULL, " \t", &saveptr);
      continue;
    }
    if (!have_altimeter) {
      float parsed_altimeter = NAN;
      if (ParseMetarAltimeterInHg(token, &parsed_altimeter)) {
        metar_out->altimeter_inhg = parsed_altimeter;
        have_altimeter = true;
      }
    }
    if ((strcmp(token, "AUTO") == 0 || strcmp(token, "COR") == 0) &&
        metar_out->auto_or_cor[0] == '\0') {
      snprintf(
        metar_out->auto_or_cor, sizeof(metar_out->auto_or_cor), "%s", token);
    }
    if (metar_out->temp_dew_token[0] == '\0' &&
        MetarTokenLooksLikeTempDew_(token)) {
      snprintf(metar_out->temp_dew_token,
               sizeof(metar_out->temp_dew_token),
               "%s",
               token);
    }
    if (strcmp(token, "RMK") == 0) {
      in_remarks = true;
      token = strtok_r(NULL, " \t", &saveptr);
      continue;
    }
    if (in_remarks) {
      if (metar_out->remarks[0] != '\0') {
        strncat(metar_out->remarks,
                " ",
                sizeof(metar_out->remarks) - strlen(metar_out->remarks) - 1);
      }
      strncat(metar_out->remarks,
              token,
              sizeof(metar_out->remarks) - strlen(metar_out->remarks) - 1);
    }
    token = strtok_r(NULL, " \t", &saveptr);
  }

  if (!have_station) {
    if (error_out != NULL && error_out_size > 0) {
      snprintf(error_out, error_out_size, "METAR station ID missing/malformed");
    }
    return false;
  }
  if (!have_obs) {
    if (error_out != NULL && error_out_size > 0) {
      snprintf(error_out, error_out_size, "METAR observation token missing");
    }
    return false;
  }
  if (!have_altimeter || !(metar_out->altimeter_inhg > 0.0f)) {
    if (error_out != NULL && error_out_size > 0) {
      snprintf(error_out, error_out_size, "METAR altimeter token missing");
    }
    return false;
  }
  return true;
}

static void
PrintCalibrationMetarBlock_(const calibration_metar_reference_t* metar,
                            const char* heading,
                            bool include_report_block)
{
  if (metar == NULL || metar->valid == 0u) {
    printf("%s:\n  none stored\n", (heading != NULL) ? heading : "METAR");
    return;
  }
  char stored_at[32] = { 0 };
  FormatUtcEpochIso8601(
    metar->stored_at_utc_epoch, stored_at, sizeof(stored_at));
  printf("%s:\n", (heading != NULL) ? heading : "Calibration Steam Reference");
  printf("  Source type:         %s\n", metar->source_type);
  printf("  Raw METAR:           %s\n", metar->raw_metar);
  printf("  Station:             %s\n", metar->station_id);
  if (metar->observation_resolved != 0u &&
      metar->observation_iso_utc[0] != '\0') {
    printf("  Observation UTC:     %s\n", metar->observation_iso_utc);
  } else {
    printf("  Observation token:   %s (unresolved)\n",
           metar->observation_token);
  }
  printf("  Elevation used_ft:   %u\n", (unsigned)metar->elevation_ft);
  printf("  Altimeter inHg:      %.2f\n", (double)metar->altimeter_inhg);
  printf("  Station pressure:    %.2f inHg\n",
         (double)metar->station_pressure_inhg);
  printf("  Calculated satpt_C:  %.3f\n", (double)metar->satpt_c);
  printf("  Stored at UTC:       %s\n", stored_at);
  printf("  Method note:         %s\n", metar->method_note);
  if (metar->auto_or_cor[0] != '\0') {
    printf("  Flag:                %s\n", metar->auto_or_cor);
  }
  if (metar->temp_dew_token[0] != '\0') {
    printf("  Temp/Dew token:      %s\n", metar->temp_dew_token);
  }
  if (metar->remarks[0] != '\0') {
    printf("  Remarks:             %s\n", metar->remarks);
  }

  if (!include_report_block) {
    return;
  }

  printf("Calibration Report Block:\n");
  printf("  Pressure source type: %s\n", metar->source_type);
  printf("  METAR station: %s\n", metar->station_id);
  printf("  Raw METAR: %s\n", metar->raw_metar);
  if (metar->observation_resolved != 0u &&
      metar->observation_iso_utc[0] != '\0') {
    printf("  Observation time UTC: %s\n", metar->observation_iso_utc);
  } else {
    printf("  Observation time UTC: unresolved (%s)\n",
           metar->observation_token);
  }
  printf("  Elevation used: %u ft\n", (unsigned)metar->elevation_ft);
  printf("  Altimeter setting: %.2f inHg\n", (double)metar->altimeter_inhg);
  printf("  Estimated station pressure: %.2f inHg\n",
         (double)metar->station_pressure_inhg);
  printf("  Calculated steam temperature: %.3f C\n", (double)metar->satpt_c);
  printf("  Method note: %s\n", metar->method_note);
}

static bool
ParseElevationFt_(const char* token, uint16_t* elevation_ft_out)
{
  if (token == NULL || elevation_ft_out == NULL || token[0] == '\0') {
    return false;
  }
  char* endptr = NULL;
  long parsed = strtol(token, &endptr, 10);
  if (endptr == token || *endptr != '\0' || parsed < 0 || parsed > 20000) {
    return false;
  }
  *elevation_ft_out = (uint16_t)parsed;
  return true;
}

/**
 * @brief Print satpt command usage lines.
 */
static void
PrintSatPtUsage(void)
{
  printf("usage: satpt <station_inHg>\n");
  printf("   or: satpt <A_inHg|A####> <elev_ft>\n");
  printf("  Example: satpt 28.90\n");
  printf("  Example: satpt 29.22 1315\n");
  printf("  Example: satpt A2922 1315\n");
}

/**
 * @brief Compute and print water saturation temperature at local pressure.
 *
 * @param argc Number of command-line arguments.
 * @param argv Argument vector.
 * @return 0 on success; non-zero on argument/parse/validation errors.
 */
static int
CommandSatPt(int argc, char** argv)
{
  if (argc != 2 && argc != 3) {
    PrintSatPtUsage();
    printf("ERR invalid args\n");
    return 1;
  }

  char* endptr = NULL;
  if (argc == 2) {
    if ((argv[1][0] == 'A' || argv[1][0] == 'a') && strlen(argv[1]) == 5U) {
      printf("ERR A#### requires elev_ft\n");
      PrintSatPtUsage();
      return 1;
    }

    const float station_in_hg = strtof(argv[1], &endptr);
    if (endptr == argv[1] || *endptr != '\0' || isnan(station_in_hg) ||
        station_in_hg <= 0.0f) {
      printf("ERR invalid station_inHg\n");
      PrintSatPtUsage();
      return 1;
    }

    const float tsat_c = SaturationTempCFromStationInHg(station_in_hg);
    if (isnan(tsat_c)) {
      printf("ERR invalid saturation-point result\n");
      PrintSatPtUsage();
      return 1;
    }
    const float tsat_f = tsat_c * 9.0f / 5.0f + 32.0f;
    printf("Saturation point at %.3f inHg (station): %.3f C (%.3f F)\n",
           (double)station_in_hg,
           (double)tsat_c,
           (double)tsat_f);
    return 0;
  }

  float altimeter_in_hg = NAN;
  if (!ParseMetarAltimeterInHg(argv[1], &altimeter_in_hg) ||
      isnan(altimeter_in_hg) || altimeter_in_hg <= 0.0f) {
    printf("ERR invalid altimeter_inHg\n");
    PrintSatPtUsage();
    return 1;
  }

  endptr = NULL;
  const float elev_ft = strtof(argv[2], &endptr);
  if (endptr == argv[2] || *endptr != '\0' || isnan(elev_ft)) {
    printf("ERR invalid elev_ft\n");
    PrintSatPtUsage();
    return 1;
  }

  const float station_in_hg =
    StationPressureFromSlpInHg(altimeter_in_hg, elev_ft);
  if (isnan(station_in_hg)) {
    printf("ERR invalid station pressure\n");
    PrintSatPtUsage();
    return 1;
  }

  const float tsat_c = SaturationTempCFromStationInHg(station_in_hg);
  if (isnan(tsat_c)) {
    printf("ERR invalid saturation-point result\n");
    PrintSatPtUsage();
    return 1;
  }
  const float tsat_f = tsat_c * 9.0f / 5.0f + 32.0f;

  printf("Saturation point at station %.3f inHg (METAR AS=%.3f inHg, elev=%.0f "
         "ft): "
         "%.3f C (%.3f F)\n",
         (double)station_in_hg,
         (double)altimeter_in_hg,
         (double)elev_ft,
         (double)tsat_c,
         (double)tsat_f);
  return 0;
}

/**
 * @brief Execute CommandDst.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandDst(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_dst_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_dst_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_dst_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("dst_enabled: %s\n",
           g_runtime->settings->dst_enabled ? "yes" : "no");
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_dst_args.enabled->count != 1) {
      printf("usage: dst set 0|1\n");
      return 1;
    }
    const int enabled = g_dst_args.enabled->ival[0];
    if (enabled != 0 && enabled != 1) {
      printf("usage: dst set 0|1\n");
      return 1;
    }
    g_runtime->settings->dst_enabled = (enabled == 1);
    if (g_runtime->settings->dst_enabled) {
      if (strcmp(g_runtime->settings->tz_posix, APP_SETTINGS_TZ_DEFAULT_STD) ==
          0) {
        snprintf(g_runtime->settings->tz_posix,
                 sizeof(g_runtime->settings->tz_posix),
                 "%s",
                 APP_SETTINGS_TZ_DEFAULT_POSIX);
      }
    } else {
      if (strcmp(g_runtime->settings->tz_posix,
                 APP_SETTINGS_TZ_DEFAULT_POSIX) == 0) {
        snprintf(g_runtime->settings->tz_posix,
                 sizeof(g_runtime->settings->tz_posix),
                 "%s",
                 APP_SETTINGS_TZ_DEFAULT_STD);
      }
    }
    esp_err_t result = AppSettingsSaveTimeZone(
      g_runtime->settings->tz_posix, g_runtime->settings->dst_enabled);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    AppSettingsApplyTimeZone(g_runtime->settings);
    printf("dst_enabled set to %d\n", enabled);
    printf("tz_posix: %s\n", g_runtime->settings->tz_posix);
    return 0;
  }

  printf("unknown action. usage: dst show | dst set 0|1\n");
  return 1;
}

/**
 * @brief Execute CommandRole.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRole(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_role_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_role_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_role_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("role: %s\n",
           AppSettingsRoleToString(g_runtime->settings->node_role));
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_role_args.role->count != 1) {
      printf("usage: role set root|sensor|relay\n");
      return 1;
    }
    const char* role_value = g_role_args.role->sval[0];
    app_node_role_t role = APP_NODE_ROLE_SENSOR;
    if (!AppSettingsParseRole(role_value, &role)) {
      printf("usage: role set root|sensor|relay\n");
      return 1;
    }

    g_runtime->settings->node_role = role;
    esp_err_t result = AppSettingsSaveNodeRole(role);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }

    if (!g_runtime->settings->allow_children_set) {
      const bool allow_children = AppSettingsRoleDefaultAllowsChildren(role);
      g_runtime->settings->allow_children = allow_children;
      result = AppSettingsSaveAllowChildren(allow_children, false);
      if (result != ESP_OK) {
        printf("save failed: %s\n", esp_err_to_name(result));
        return 1;
      }
    }

    printf("role set to %s\n", AppSettingsRoleToString(role));
    return 0;
  }

  printf("unknown action. usage: role show | role set root|sensor|relay\n");
  return 1;
}

/**
 * @brief Execute CommandNet.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandNet(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_net_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_net_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_net_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    const app_net_mode_t configured_mode = g_runtime->settings->net_mode;
    const app_net_mode_t effective_mode = AppSettingsGetEffectiveNetMode(
      g_runtime->settings->node_role, configured_mode);
    printf("configured_net_mode: %s\n",
           AppSettingsNetModeToString(configured_mode));
    printf("effective_net_mode: %s\n",
           AppSettingsNetModeToString(effective_mode));
    const char* net_override_reason = AppSettingsGetNetModeOverrideReason(
      g_runtime->settings->node_role, configured_mode);
    if (net_override_reason != NULL) {
      printf("effective_override_reason: %s\n", net_override_reason);
    }
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_net_args.mode->count != 1) {
      printf("usage: net set mesh|wifi|none\n");
      return 1;
    }
    const char* mode_value = g_net_args.mode->sval[0];
    app_net_mode_t mode = APP_NET_MODE_MESH;
    if (!AppSettingsParseNetMode(mode_value, &mode)) {
      printf("usage: net set mesh|wifi|none\n");
      return 1;
    }
    g_runtime->settings->net_mode = mode;
    esp_err_t result = AppSettingsSaveNetMode(mode);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    const app_net_mode_t effective_mode =
      AppSettingsGetEffectiveNetMode(g_runtime->settings->node_role, mode);
    const char* net_override_reason =
      AppSettingsGetNetModeOverrideReason(g_runtime->settings->node_role, mode);
    printf("OK\n");
    printf("net_mode stored as %s\n", AppSettingsNetModeToString(mode));
    if (effective_mode == mode) {
      printf("effective_net_mode: %s\n", AppSettingsNetModeToString(mode));
    } else {
      printf("effective_net_mode remains %s (%s)\n",
             AppSettingsNetModeToString(effective_mode),
             (net_override_reason != NULL) ? net_override_reason
                                           : "role-based override");
    }
    NotifyNetSupervisor();
    return 0;
  }

  printf("unknown action. usage: net show | net set mesh|wifi|none\n");
  return 1;
}

/**
 * @brief Execute CommandWifi.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandWifi(int argc, char** argv)
{
  if (argc < 2) {
    PrintWifiUsage();
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "help") == 0) {
    PrintWifiUsage();
    return 0;
  }

  if (strcmp(action, "show") == 0) {
    wifi_credentials_t creds;
    WifiCredentialsLoad(&creds);
    printf("configured: %s\n", creds.has_ssid ? "yes" : "no");
    printf("ssid: %s\n", creds.has_ssid ? creds.ssid : "<unset>");
    if (creds.has_ssid) {
      printf("ssid_source: %s\n", creds.from_nvs ? "nvs" : "kconfig");
    } else {
      printf("ssid_source: none\n");
    }
    if (!creds.has_ssid) {
      printf("password: n/a\n");
    } else if (creds.password[0] == '\0') {
      printf("password: <empty>\n");
    } else {
      char masked[65] = { 0 };
      WifiCredentialsMaskPassword(creds.password, masked, sizeof(masked));
      printf("password: %s\n", masked);
    }
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (argc < 3) {
      printf("usage: wifi set <ssid> [password]\n");
      return 1;
    }
    const char* ssid = argv[2];
    const char* password = (argc >= 4) ? argv[3] : "";
    esp_err_t result = WifiCredentialsSave(ssid, password);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("OK\n");
    NotifyNetSupervisor();
    return 0;
  }

  if (strcmp(action, "clear") == 0) {
    esp_err_t result = WifiCredentialsClear();
    if (result != ESP_OK) {
      printf("clear failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("OK\n");
    return 0;
  }

  if (strcmp(action, "scan") == 0) {
    int max_records = 10;
    for (int i = 2; i < argc; ++i) {
      if (strcmp(argv[i], "--max") == 0) {
        if (i + 1 >= argc) {
          printf("usage: wifi scan [--max N]\n");
          return 1;
        }
        max_records = atoi(argv[++i]);
      } else {
        printf("usage: wifi scan [--max N]\n");
        return 1;
      }
    }

    if (max_records <= 0) {
      printf("max must be > 0\n");
      return 1;
    }

    const size_t kMaxScanRecords = 50;
    size_t record_cap = (size_t)max_records;
    if (record_cap > kMaxScanRecords) {
      record_cap = kMaxScanRecords;
      printf("note: max capped at %u\n", (unsigned)kMaxScanRecords);
    }

    bool did_acquire = false;
    esp_err_t wifi_result = AcquireWifiForConsole(&did_acquire);
    if (wifi_result != ESP_OK) {
      printf("scan failed: %s\n", esp_err_to_name(wifi_result));
      if (wifi_result == ESP_ERR_INVALID_STATE) {
        printf("note: Wi-Fi is active in another mode (%s).\n",
               WifiServiceModeToString(WifiServiceActiveMode()));
      }
      return 1;
    }

    wifi_ap_record_t* records =
      (wifi_ap_record_t*)AppCalloc(record_cap, sizeof(*records));
    if (records == NULL) {
      printf("out of memory\n");
      ReleaseWifiForConsoleIfNeeded(did_acquire);
      return 1;
    }

    size_t total_count = 0;
    esp_err_t result = WifiManagerScan(records, record_cap, &total_count);
    if (result != ESP_OK) {
      printf("scan failed: %s\n", esp_err_to_name(result));
      AppFree(records);
      ReleaseWifiForConsoleIfNeeded(did_acquire);
      return 1;
    }

    const size_t listed_count =
      (total_count < record_cap) ? total_count : record_cap;
    printf("aps_found: %u (showing %u)\n",
           (unsigned)total_count,
           (unsigned)listed_count);
    for (size_t index = 0; index < listed_count; ++index) {
      const wifi_ap_record_t* ap = &records[index];
      const char* ssid =
        (ap->ssid[0] != '\0') ? (const char*)ap->ssid : "<hidden>";
      printf("  %2u. %-32s rssi=%d ch=%u auth=%s\n",
             (unsigned)(index + 1),
             ssid,
             ap->rssi,
             (unsigned)ap->primary,
             WifiAuthModeToString(ap->authmode));
    }

    AppFree(records);
    ReleaseWifiForConsoleIfNeeded(did_acquire);
    return 0;
  }

  if (strcmp(action, "status") == 0) {
    wifi_manager_status_t status;
    memset(&status, 0, sizeof(status));
    WifiManagerGetStatus(&status);
    printf("wifi_service_mode: %s\n",
           WifiServiceModeToString(WifiServiceActiveMode()));
    printf("wifi_sta_netif_present: %s\n",
           status.sta_netif_present ? "yes" : "no");
    printf("wifi_owns_sta_netif: %s\n", status.owns_sta_netif ? "yes" : "no");
    printf("wifi_initialized: %s\n", status.wifi_initialized ? "yes" : "no");
    printf("wifi_handler_registered: %s\n",
           status.wifi_handler_registered ? "yes" : "no");
    printf("wifi_ip_handler_registered: %s\n",
           status.ip_handler_registered ? "yes" : "no");
    printf("wifi_started: %s\n", status.wifi_started ? "yes" : "no");
    printf("wifi_started_by_manager: %s\n",
           status.started_by_manager ? "yes" : "no");

    const bool connected = WifiManagerIsConnected();
    printf("wifi_connected: %s\n", connected ? "yes" : "no");

    wifi_ap_record_t ap_info;
    memset(&ap_info, 0, sizeof(ap_info));
    if (connected && esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
      printf("wifi_rssi: %d\n", ap_info.rssi);
      printf("wifi_channel: %u\n", (unsigned)ap_info.primary);
    } else {
      printf("wifi_rssi: n/a\n");
      printf("wifi_channel: n/a\n");
    }

    const wifi_err_reason_t reason = WifiManagerLastDisconnectReason();
    printf("wifi_last_disconnect_reason: %s (%d)\n",
           WifiDisconnectReasonToString(reason),
           (int)reason);
    printf("wifi_last_connect_attempts: %d\n",
           WifiManagerLastConnectAttempts());

    if (connected) {
      esp_netif_ip_info_t ip_info;
      memset(&ip_info, 0, sizeof(ip_info));
      if (WifiManagerGetIpInfo(&ip_info) == ESP_OK) {
        char ip[16] = { 0 }, mask[16] = { 0 }, gw[16] = { 0 };
        esp_ip4addr_ntoa(&ip_info.ip, ip, sizeof(ip));
        esp_ip4addr_ntoa(&ip_info.netmask, mask, sizeof(mask));
        esp_ip4addr_ntoa(&ip_info.gw, gw, sizeof(gw));
        printf("wifi_ip: %s\n", ip);
        printf("wifi_netmask: %s\n", mask);
        printf("wifi_gw: %s\n", gw);
      } else {
        printf("wifi_ip: n/a\n");
        printf("wifi_netmask: n/a\n");
        printf("wifi_gw: n/a\n");
      }
    } else {
      printf("wifi_ip: n/a\n");
      printf("wifi_netmask: n/a\n");
      printf("wifi_gw: n/a\n");
    }
    return 0;
  }

  if (strcmp(action, "connect") == 0) {
    int timeout_ms = 10000;
    for (int i = 2; i < argc; ++i) {
      if (strcmp(argv[i], "--timeout_ms") == 0) {
        if (i + 1 >= argc) {
          printf("usage: wifi connect [--timeout_ms T]\n");
          return 1;
        }
        timeout_ms = atoi(argv[++i]);
      } else {
        printf("usage: wifi connect [--timeout_ms T]\n");
        return 1;
      }
    }

    wifi_credentials_t creds;
    WifiCredentialsLoad(&creds);
    if (!creds.has_ssid || !creds.from_nvs) {
      printf("no saved Wi-Fi credentials in NVS. Use: wifi set <ssid> "
             "[password]\n");
      return 1;
    }

    bool did_acquire = false;
    esp_err_t wifi_result = AcquireWifiForConsole(&did_acquire);
    if (wifi_result != ESP_OK) {
      printf("connect failed: %s\n", esp_err_to_name(wifi_result));
      if (wifi_result == ESP_ERR_INVALID_STATE) {
        printf("note: Wi-Fi is active in another mode (%s).\n",
               WifiServiceModeToString(WifiServiceActiveMode()));
      }
      return 1;
    }

    esp_err_t result =
      WifiManagerConnectSta(creds.ssid, creds.password, timeout_ms);
    if (result != ESP_OK) {
      const wifi_err_reason_t reason = WifiManagerLastDisconnectReason();
      printf("connect failed: %s\n", esp_err_to_name(result));
      printf("  attempts=%d reason=%s (%d)\n",
             WifiManagerLastConnectAttempts(),
             WifiDisconnectReasonToString(reason),
             (int)reason);
      ReleaseWifiForConsoleIfNeeded(did_acquire);
      return 1;
    }
    printf("connected\n");
    ReleaseWifiForConsoleIfNeeded(did_acquire);
    return 0;
  }

  if (strcmp(action, "disconnect") == 0) {
    bool did_acquire = false;
    esp_err_t wifi_result = AcquireWifiForConsole(&did_acquire);
    if (wifi_result != ESP_OK) {
      printf("disconnect failed: %s\n", esp_err_to_name(wifi_result));
      if (wifi_result == ESP_ERR_INVALID_STATE) {
        printf("note: Wi-Fi is active in another mode (%s).\n",
               WifiServiceModeToString(WifiServiceActiveMode()));
      }
      return 1;
    }

    esp_err_t result = WifiManagerDisconnectSta();
    if (result != ESP_OK) {
      printf("disconnect failed: %s\n", esp_err_to_name(result));
      ReleaseWifiForConsoleIfNeeded(did_acquire);
      return 1;
    }
    printf("OK\n");
    ReleaseWifiForConsoleIfNeeded(did_acquire);
    return 0;
  }

  if (strcmp(action, "cfg") == 0) {
    if (argc < 3) {
      PrintWifiUsage();
      return 1;
    }
    const char* subaction = argv[2];
    if (strcmp(subaction, "show") == 0) {
      PrintWifiConfig();
      return 0;
    }
    if (strcmp(subaction, "defaults") == 0) {
      esp_err_t result = AppNetConfigClearAllOverrides();
      if (result != ESP_OK) {
        printf("defaults failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      printf("OK\n");
      if (RuntimeIsRunning()) {
        printf("note: run stop; run start to apply\n");
      }
      return 0;
    }
    if (strcmp(subaction, "set") == 0) {
      if (argc < 5) {
        PrintWifiUsage();
        return 1;
      }
      const char* key = argv[3];
      const char* value = argv[4];
      esp_err_t result = ESP_ERR_INVALID_ARG;

      if (strcmp(key, "sntp") == 0) {
        result = AppNetConfigSetSntpServer(value);
      } else if (strcmp(key, "mesh_chan") == 0) {
        char* end = NULL;
        unsigned long channel = strtoul(value, &end, 10);
        if (end != value && *end == '\0' && channel <= 13) {
          result = AppNetConfigSetMeshChannel((uint8_t)channel);
        }
      } else if (strcmp(key, "mesh_id") == 0) {
        result = AppNetConfigSetMeshIdString(value);
      } else if (strcmp(key, "mesh_ap_pass") == 0) {
        result = AppNetConfigSetMeshApPassword(value);
      } else if (strcmp(key, "no_router") == 0) {
        if (strcmp(value, "0") == 0) {
          result = AppNetConfigSetMeshDisableRouter(false);
        } else if (strcmp(value, "1") == 0) {
          result = AppNetConfigSetMeshDisableRouter(true);
        }
      } else if (strcmp(key, "sntp_fail_n") == 0) {
        char* end = NULL;
        unsigned long threshold = strtoul(value, &end, 10);
        if (end != value && *end == '\0' && threshold <= UINT32_MAX) {
          result = AppNetConfigSetSntpFailThresholdN((uint32_t)threshold);
        }
      } else if (strcmp(key, "time_sync_s") == 0) {
        char* end = NULL;
        unsigned long seconds = strtoul(value, &end, 10);
        if (end != value && *end == '\0' && seconds <= UINT32_MAX) {
          result = AppNetConfigSetTimeSyncPeriodSeconds((uint32_t)seconds);
        }
      } else {
        PrintWifiUsage();
        return 1;
      }

      if (result != ESP_OK) {
        printf("set failed: %s\n", esp_err_to_name(result));
        return 1;
      }

      printf("OK\n");
      NotifyNetSupervisor();
      return 0;
    }

    PrintWifiUsage();
    return 1;
  }

  if (strcmp(action, "ntp") == 0) {
    if (argc < 3) {
      printf("usage: wifi ntp status | wifi ntp sync [--server host] "
             "[--timeout_ms T] [--update-rtc 0|1]\n");
      return 1;
    }
    const char* subaction = argv[2];
    if (strcmp(subaction, "status") == 0) {
      printf("system_time_valid: %s\n",
             TimeSyncIsSystemTimeValid() ? "yes" : "no");
      time_sntp_status_t sntp_status;
      TimeSyncGetSntpStatus(&sntp_status);
      printf("last_sntp_server: %s\n",
             (sntp_status.last_server[0] != '\0') ? sntp_status.last_server
                                                  : "n/a");
      if (sntp_status.last_attempt_epoch > 0) {
        printf("last_sntp_attempt_epoch: %" PRId64 "\n",
               sntp_status.last_attempt_epoch);
      } else {
        printf("last_sntp_attempt_epoch: never\n");
      }
      printf("last_sntp_result: %s (%d)\n",
             esp_err_to_name(sntp_status.last_result),
             (int)sntp_status.last_result);
      if (sntp_status.last_success_epoch > 0) {
        printf("last_sntp_success_epoch: %" PRId64 "\n",
               sntp_status.last_success_epoch);
      } else {
        printf("last_sntp_success_epoch: never\n");
      }

      const bool rtc_present =
        (g_runtime != NULL && g_runtime->time_sync != NULL &&
         g_runtime->time_sync->is_ds3231_ready);
      printf("rtc_present: %s\n", rtc_present ? "yes" : "no");

      int64_t rtc_epoch = 0;
      if (rtc_present && g_runtime != NULL && g_runtime->time_sync != NULL &&
          TimeSyncReadRtcEpoch(g_runtime->time_sync, &rtc_epoch) == ESP_OK) {
        printf("rtc_epoch_utc: %" PRId64 "\n", rtc_epoch);
      } else {
        printf("rtc_epoch_utc: n/a\n");
      }

      const int64_t rtc_last_set_epoch = TimeSyncGetLastRtcSetEpoch();
      if (rtc_last_set_epoch > 0) {
        printf("rtc_last_set_epoch: %" PRId64 "\n", rtc_last_set_epoch);
      } else {
        printf("rtc_last_set_epoch: n/a\n");
      }
      printf("sntp_consecutive_failures: %u\n",
             (unsigned)NetSupervisorGetSntpConsecutiveFailures());
      printf("sntp_fail_threshold_n: %u\n",
             (unsigned)AppNetConfigGetSntpFailThresholdN());
      printf("sntp_failure_alert_active: %s\n",
             NetSupervisorIsSntpFailureAlertActive() ? "yes" : "no");
      return 0;
    }

    if (strcmp(subaction, "sync") == 0) {
      const char* server = NULL;
      int timeout_ms = 30000;
      bool update_rtc = true;

      for (int i = 3; i < argc; ++i) {
        if (strcmp(argv[i], "--server") == 0) {
          if (i + 1 >= argc) {
            printf("usage: wifi ntp sync [--server host] [--timeout_ms T] "
                   "[--update-rtc 0|1]\n");
            return 1;
          }
          server = argv[++i];
        } else if (strcmp(argv[i], "--timeout_ms") == 0) {
          if (i + 1 >= argc) {
            printf("usage: wifi ntp sync [--server host] [--timeout_ms T] "
                   "[--update-rtc 0|1]\n");
            return 1;
          }
          timeout_ms = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--update-rtc") == 0) {
          if (i + 1 >= argc) {
            printf("usage: wifi ntp sync [--server host] [--timeout_ms T] "
                   "[--update-rtc 0|1]\n");
            return 1;
          }
          update_rtc = (atoi(argv[++i]) != 0);
        } else {
          printf("usage: wifi ntp sync [--server host] [--timeout_ms T] "
                 "[--update-rtc 0|1]\n");
          return 1;
        }
      }

      if (server == NULL) {
        server = PickDefaultSntpServer();
      }

      esp_err_t result = TimeSyncStartSntpAndWait(server, timeout_ms);
      if (result != ESP_OK) {
        printf("sntp sync failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      printf("sntp sync OK\n");

      if (update_rtc) {
        if (g_runtime != NULL && g_runtime->time_sync != NULL &&
            g_runtime->time_sync->is_ds3231_ready) {
          esp_err_t rtc_result = TimeSyncSetRtcFromSystem(g_runtime->time_sync);
          if (rtc_result != ESP_OK) {
            printf("rtc update failed: %s\n", esp_err_to_name(rtc_result));
            return 1;
          }
          printf("rtc updated\n");
        } else {
          printf("rtc update skipped: rtc not available\n");
        }
      }
      return 0;
    }

    printf("usage: wifi ntp status | wifi ntp sync [--server host] "
           "[--timeout_ms T] [--update-rtc 0|1]\n");
    return 1;
  }

  PrintWifiUsage();
  return 1;
}

/**
 * @brief Execute PrintMqttRestartNote.
 */
static void
PrintMqttRestartNote(void)
{
  if (RuntimeIsRunning()) {
    printf("Change applied; takes effect after run stop/start\n");
  }
}

/**
 * @brief Execute CommandMqtt.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandMqtt(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: mqtt show | mqtt enable on|off | mqtt broker set <uri> | "
           "mqtt prefix set <prefix> | mqtt qos set 0|1 | mqtt retain set "
           "on|off | mqtt bridge set off|serial|broker|both\n");
    return 1;
  }

  app_settings_t* settings = g_runtime->settings;
  const char* action = argv[1];

  if (strcmp(action, "show") == 0) {
    printf("mqtt_enabled: %s\n", settings->mqtt_enabled ? "yes" : "no");
    printf("mqtt_broker_uri: %s\n", settings->mqtt_broker_uri);
    printf("mqtt_topic_prefix: %s\n", settings->mqtt_topic_prefix);
    printf("mqtt_qos: %u\n", (unsigned)settings->mqtt_qos);
    printf("mqtt_retain: %s\n", settings->mqtt_retain ? "yes" : "no");
    printf("mqtt_bridge_mode: %s\n",
           AppSettingsMqttBridgeModeToString(settings->mqtt_bridge_mode));
    return 0;
  }

  if (strcmp(action, "enable") == 0) {
    if (argc != 3) {
      printf("usage: mqtt enable on|off\n");
      return 1;
    }
    const bool enabled = (strcmp(argv[2], "on") == 0);
    if (!enabled && strcmp(argv[2], "off") != 0) {
      printf("usage: mqtt enable on|off\n");
      return 1;
    }
    settings->mqtt_enabled = enabled;
    esp_err_t result = AppSettingsSaveMqttEnabled(enabled);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("mqtt_enabled set to %s\n", enabled ? "on" : "off");
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "broker") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt broker set <uri>\n");
      return 1;
    }
    const char* uri = argv[3];
    if (uri[0] == '\0') {
      printf("invalid broker uri\n");
      return 1;
    }
    esp_err_t result = AppSettingsSaveMqttBrokerUri(uri);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    snprintf(
      settings->mqtt_broker_uri, sizeof(settings->mqtt_broker_uri), "%s", uri);
    printf("mqtt_broker_uri set to %s\n", settings->mqtt_broker_uri);
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "prefix") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt prefix set <prefix>\n");
      return 1;
    }
    const char* prefix = argv[3];
    if (prefix[0] == '\0') {
      printf("invalid prefix\n");
      return 1;
    }
    esp_err_t result = AppSettingsSaveMqttTopicPrefix(prefix);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    snprintf(settings->mqtt_topic_prefix,
             sizeof(settings->mqtt_topic_prefix),
             "%s",
             prefix);
    printf("mqtt_topic_prefix set to %s\n", settings->mqtt_topic_prefix);
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "qos") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt qos set 0|1\n");
      return 1;
    }
    char* end = NULL;
    long qos_long = strtol(argv[3], &end, 10);
    if (end == argv[3] || *end != '\0' || (qos_long != 0 && qos_long != 1)) {
      printf("usage: mqtt qos set 0|1\n");
      return 1;
    }
    settings->mqtt_qos = (uint8_t)qos_long;
    esp_err_t result = AppSettingsSaveMqttQos((uint8_t)qos_long);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("mqtt_qos set to %ld\n", qos_long);
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "retain") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt retain set on|off\n");
      return 1;
    }
    const bool retain = (strcmp(argv[3], "on") == 0);
    if (!retain && strcmp(argv[3], "off") != 0) {
      printf("usage: mqtt retain set on|off\n");
      return 1;
    }
    settings->mqtt_retain = retain;
    esp_err_t result = AppSettingsSaveMqttRetain(retain);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("mqtt_retain set to %s\n", retain ? "on" : "off");
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "bridge") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt bridge set off|serial|broker|both\n");
      return 1;
    }
    mqtt_bridge_mode_t mode = MQTT_BRIDGE_OFF;
    if (!AppSettingsParseMqttBridgeMode(argv[3], &mode)) {
      printf("usage: mqtt bridge set off|serial|broker|both\n");
      return 1;
    }
    settings->mqtt_bridge_mode = mode;
    esp_err_t result = AppSettingsSaveMqttBridgeMode(mode);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    if (settings->node_role != APP_NODE_ROLE_ROOT) {
      printf("note: bridge mode applies to root nodes\n");
    }
    printf("mqtt_bridge_mode set to %s\n",
           AppSettingsMqttBridgeModeToString(mode));
    PrintMqttRestartNote();
    return 0;
  }

  printf("unknown action. usage: mqtt show | mqtt enable on|off | mqtt broker "
         "set <uri> | mqtt prefix set <prefix> | mqtt qos set 0|1 | mqtt "
         "retain set on|off | mqtt bridge set off|serial|broker|both\n");
  return 1;
}

/**
 * @brief Execute CommandChildren.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandChildren(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_children_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_children_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_children_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("allow_children: %u\n", g_runtime->settings->allow_children ? 1 : 0);
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_children_args.enabled->count != 1) {
      printf("usage: children set 0|1\n");
      return 1;
    }
    const int enabled = g_children_args.enabled->ival[0];
    if (enabled != 0 && enabled != 1) {
      printf("usage: children set 0|1\n");
      return 1;
    }
    g_runtime->settings->allow_children = (enabled == 1);
    g_runtime->settings->allow_children_set = true;
    esp_err_t result =
      AppSettingsSaveAllowChildren(g_runtime->settings->allow_children, true);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("allow_children set to %d\n", enabled);
    return 0;
  }

  printf("unknown action. usage: children show | children set 0|1\n");
  return 1;
}

/**
 * @brief Execute PrintDiagUsage.
 */
static void
PrintDiagUsage(void)
{
  printf("diag help\n");
  printf("diag all quick|full [--verbose N]\n");
  printf("diag sd quick|full [--format-if-needed] [--mount] [--verbose N]\n");
  printf("diag storage quick|full [--verbose N]\n");
  printf("diag fram quick|full [--bytes N] [--verbose N]\n");
  printf("diag rtd quick|full [--samples N] [--delay_ms M] [--verbose N]\n");
  printf("diag rtc quick|full [--set-known] [--verbose N]\n");
  printf("diag heapcheck on|off|now\n");
  printf("diag cycle [--count N] [--run_ms M] [--stop_ms M]\n");
  printf("diag wifi quick|full [--scan 0|1] [--connect 0|1] [--dns 0|1] "
         "[--keep-connected 0|1] [--verbose N]\n");
  printf("diag mesh quick|full [--start] [--stop] [--root] [--timeout_ms T] "
         "[--verbose N]\n"
         "  note: if you use --start without --stop, the mesh stays running\n");
}

/**
 * @brief Execute ParseVerbose.
 * @param value Parameter value.
 * @param verbosity_out Parameter verbosity_out.
 * @return Return the function result.
 */
static bool
ParseVerbose(const char* value, int* verbosity_out)
{
  if (value == NULL || verbosity_out == NULL) {
    return false;
  }
  char* end = NULL;
  const long parsed = strtol(value, &end, 10);
  if (end == NULL || *end != '\0') {
    return false;
  }
  *verbosity_out = (int)parsed;
  return true;
}

static const char*
CalDueUnitToString(uint8_t unit, bool plural)
{
  switch (unit) {
    case CAL_DUE_UNIT_DAYS:
      return plural ? "days" : "day";
    case CAL_DUE_UNIT_MONTHS:
      return plural ? "months" : "month";
    case CAL_DUE_UNIT_YEARS:
      return plural ? "years" : "year";
    default:
      return "unknown";
  }
}

static bool
ParseCalDueUnit(const char* value, uint8_t* unit_out)
{
  if (value == NULL || unit_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "day") == 0 || strcasecmp(value, "days") == 0) {
    *unit_out = (uint8_t)CAL_DUE_UNIT_DAYS;
    return true;
  }
  if (strcasecmp(value, "month") == 0 || strcasecmp(value, "months") == 0) {
    *unit_out = (uint8_t)CAL_DUE_UNIT_MONTHS;
    return true;
  }
  if (strcasecmp(value, "year") == 0 || strcasecmp(value, "years") == 0) {
    *unit_out = (uint8_t)CAL_DUE_UNIT_YEARS;
    return true;
  }
  return false;
}

static void
FormatUtcEpochIso8601(int64_t epoch_utc, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  if (epoch_utc <= 0) {
    snprintf(buffer, buffer_size, "n/a");
    return;
  }
  time_t raw = (time_t)epoch_utc;
  struct tm utc_time;
  if (gmtime_r(&raw, &utc_time) == NULL) {
    snprintf(buffer, buffer_size, "n/a");
    return;
  }
  if (strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%SZ", &utc_time) == 0) {
    snprintf(buffer, buffer_size, "n/a");
  }
}

static void
FormatLocalEpochIso8601(int64_t epoch_utc, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  if (epoch_utc <= 0) {
    snprintf(buffer, buffer_size, "n/a");
    return;
  }
  time_t raw = (time_t)epoch_utc;
  struct tm local_time;
  if (localtime_r(&raw, &local_time) == NULL) {
    snprintf(buffer, buffer_size, "n/a");
    return;
  }
  if (strftime(buffer, buffer_size, "%Y-%m-%d %H:%M:%S", &local_time) == 0) {
    snprintf(buffer, buffer_size, "n/a");
  }
}

static void
FormatErrlogFlags(uint16_t flags, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  buffer[0] = '\0';
  size_t pos = 0;
  if ((flags & kFramErrorLogEntryFlagActive) != 0) {
    pos += (size_t)snprintf(
      buffer + pos, buffer_size - pos, "%sactive", (pos == 0) ? "" : ",");
  }
  if ((flags & kFramErrorLogEntryFlagResolved) != 0) {
    pos += (size_t)snprintf(
      buffer + pos, buffer_size - pos, "%sresolved", (pos == 0) ? "" : ",");
  }
  if (pos == 0) {
    snprintf(buffer, buffer_size, "none");
  }
}

static void
FormatCalDueEvery(uint16_t count,
                  uint8_t unit,
                  char* buffer,
                  size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  if (count == 0 || unit == 0) {
    snprintf(buffer, buffer_size, "disabled");
    return;
  }
  const bool plural = (count != 1);
  snprintf(buffer,
           buffer_size,
           "%u %s",
           (unsigned)count,
           CalDueUnitToString(unit, plural));
}

static void
ResolveCalibrationSchedule(const app_settings_t* settings,
                           int64_t* last_cal_out,
                           uint16_t* due_count_out,
                           uint8_t* due_unit_out)
{
  if (settings == NULL) {
    return;
  }
  if (last_cal_out != NULL) {
    *last_cal_out = (settings->cal_last_override_utc != 0)
                      ? settings->cal_last_override_utc
                      : settings->cal_last_utc;
  }
  if (due_count_out != NULL) {
    *due_count_out = (settings->cal_due_override_count != 0)
                       ? settings->cal_due_override_count
                       : settings->cal_due_count;
  }
  if (due_unit_out != NULL) {
    *due_unit_out = (settings->cal_due_override_unit != 0)
                      ? settings->cal_due_override_unit
                      : settings->cal_due_unit;
  }
}

/**
 * @brief Evaluate whether calibration is currently applied/valid for
 * reporting.
 * @param settings Current settings.
 * @param state Current runtime state (may be NULL).
 * @param reason_out Optional output reason string.
 * @return true when calibration is applied.
 */
static bool
ConsoleCalibrationIsApplied(const app_settings_t* settings,
                            const runtime_state_t* state,
                            const char** reason_out)
{
  if (reason_out != NULL) {
    *reason_out = "ok";
  }
  if (settings == NULL) {
    if (reason_out != NULL) {
      *reason_out = "settings unavailable";
    }
    return false;
  }
  if (state == NULL) {
    if (reason_out != NULL) {
      *reason_out = "runtime state unavailable";
    }
    return false;
  }
  if (settings->calibration_points_count == 0) {
    if (reason_out != NULL) {
      *reason_out = "no calibration points";
    }
    return false;
  }
  if (!settings->calibration.is_valid) {
    if (reason_out != NULL) {
      *reason_out = "calibration model not valid";
    }
    return false;
  }
  if (state->cal_due_check_suspended) {
    if (reason_out != NULL) {
      *reason_out = "due check suspended";
    }
    return false;
  }
  if (state->cal_overdue) {
    if (reason_out != NULL) {
      *reason_out = "calibration overdue";
    }
    return false;
  }
  return true;
}

static void
PrintCalibrationStatusBlock(const app_settings_t* settings,
                            const runtime_state_t* state)
{
  int64_t last_cal = 0;
  uint16_t due_count = 0;
  uint8_t due_unit = 0;
  ResolveCalibrationSchedule(settings, &last_cal, &due_count, &due_unit);

  char last_buffer[32];
  char due_every[32];
  char due_date_buffer[32];
  FormatUtcEpochIso8601(last_cal, last_buffer, sizeof(last_buffer));
  FormatCalDueEvery(due_count, due_unit, due_every, sizeof(due_every));

  int64_t due_date = 0;
  if (last_cal != 0 && due_count != 0) {
    due_date =
      CalComputeDueDateUtc(last_cal, due_count, (cal_due_unit_t)due_unit);
  }
  FormatUtcEpochIso8601(due_date, due_date_buffer, sizeof(due_date_buffer));

  const bool time_valid = TimeSyncIsSystemTimeValid();
  const bool time_stable = (state != NULL) ? state->cal_time_stable : false;
  const bool check_suspended =
    (state != NULL) ? state->cal_due_check_suspended : !time_valid;
  const bool overdue = (state != NULL) ? state->cal_overdue : false;
  const char* applied_reason = NULL;
  const bool applied =
    ConsoleCalibrationIsApplied(settings, state, &applied_reason);

  printf("Calibration:\n");
  printf("  Last UTC:  %s\n", last_buffer);
  printf("  Points: %u\n", (unsigned)settings->calibration_points_count);
  printf("  Model valid: %s\n", settings->calibration.is_valid ? "yes" : "no");
  printf("  Applied: %s\n", applied ? "yes" : "no");
  if (!applied) {
    printf("  Applied reason: %s\n", applied_reason);
  }
  if (applied && settings->cal_method[0] == '\0') {
    printf("  Note: Method is unset. Set it with: cal set method \"ice + satpt "
           "steam\"\n");
  }
  if (!time_valid) {
    printf("  Due check suspended (time invalid)\n");
    return;
  }
  printf("  Due every: %s\n", due_every);
  printf("  Due date:  %s\n", due_date_buffer);
  printf("  Overdue:   %s\n", overdue ? "yes" : "no");
  printf("  Time valid: %s\n", time_valid ? "yes" : "no");
  printf("  Time stable: %s\n", time_stable ? "yes" : "no");
  printf("  Check: %s\n", check_suspended ? "suspended" : "active");
}

static bool
ComputeIdealReferenceResistanceOhm_(double reference_temp_c,
                                    double* out_ideal_ref_res_ohm)
{
  if (out_ideal_ref_res_ohm == NULL || g_runtime == NULL ||
      g_runtime->sensor == NULL) {
    return false;
  }
  const double ideal_ohm = Max31865TemperatureToResistanceCvd(
    reference_temp_c, g_runtime->sensor->rtd_nominal_ohm);
  if (!isfinite(ideal_ohm)) {
    return false;
  }
  *out_ideal_ref_res_ohm = ideal_ohm;
  return true;
}

static void
BuildCalibrationPointPrintSummary_(const calibration_point_t* point,
                                   cal_point_print_summary_t* summary_out)
{
  if (point == NULL || summary_out == NULL) {
    return;
  }
  memset(summary_out, 0, sizeof(*summary_out));
  const double reference_c = point->actual_mC / 1000.0;
  summary_out->captured_raw_res_available = (point->raw_avg_mOhm > 0);
  if (summary_out->captured_raw_res_available) {
    summary_out->captured_raw_res_avg_ohm = point->raw_avg_mOhm / 1000.0;
  }
  summary_out->captured_raw_temp_avg_available =
    (point->raw_avg_mC != INT32_MIN);
  if (summary_out->captured_raw_temp_avg_available) {
    summary_out->captured_raw_temp_avg_c = point->raw_avg_mC / 1000.0;
  }
  summary_out->captured_raw_temp_stddev_available = (point->raw_stddev_mC >= 0);
  if (summary_out->captured_raw_temp_stddev_available) {
    summary_out->captured_raw_temp_stddev_c = point->raw_stddev_mC / 1000.0;
  }
  summary_out->captured_raw_res_stddev_available =
    (point->raw_stddev_mOhm >= 0);
  if (summary_out->captured_raw_res_stddev_available) {
    summary_out->captured_raw_res_stddev_ohm = point->raw_stddev_mOhm / 1000.0;
  }
  summary_out->captured_drift_available =
    (point->captured_drift_mC_per_min !=
     CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN);
  if (summary_out->captured_drift_available) {
    summary_out->captured_drift_c_per_min =
      point->captured_drift_mC_per_min / 1000.0;
  }
  summary_out->captured_delta_available =
    (point->captured_delta_mC != CAL_CAPTURE_DELTA_UNAVAILABLE_MC);
  if (summary_out->captured_delta_available) {
    summary_out->captured_delta_c = point->captured_delta_mC / 1000.0;
  }
  summary_out->drift_limit_available =
    (point->capture_drift_limit_mC_per_min !=
     CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN);
  if (summary_out->drift_limit_available) {
    summary_out->drift_limit_c_per_min =
      point->capture_drift_limit_mC_per_min / 1000.0;
  }
  summary_out->drift_limit_source =
    (calibration_drift_limit_source_t)point->drift_limit_source;
  summary_out->captured_window_available =
    (point->captured_window_s != CAL_CAPTURE_WINDOW_S_UNAVAILABLE);
  if (summary_out->captured_window_available) {
    summary_out->captured_window_s = point->captured_window_s;
  }
  summary_out->captured_ema_alpha_available =
    (point->captured_ema_alpha_permille !=
     CAL_CAPTURE_EMA_ALPHA_UNAVAILABLE_PERMILLE);
  if (summary_out->captured_ema_alpha_available) {
    summary_out->captured_ema_alpha =
      point->captured_ema_alpha_permille / 1000.0;
  }
  summary_out->ideal_ref_res_available = ComputeIdealReferenceResistanceOhm_(
    reference_c, &summary_out->ideal_ref_res_ohm);
  if (summary_out->ideal_ref_res_available &&
      summary_out->captured_raw_res_available) {
    summary_out->residual_res_ohm =
      summary_out->ideal_ref_res_ohm - summary_out->captured_raw_res_avg_ohm;
    summary_out->residual_res_available = true;
  }
  if (summary_out->captured_raw_temp_avg_available) {
    summary_out->residual_temp_c =
      reference_c - summary_out->captured_raw_temp_avg_c;
    summary_out->residual_temp_available = true;
  }
}

static void
PrintCalibrationPointStatusLine_(const calibration_point_t* point,
                                 const cal_point_print_summary_t* summary,
                                 uint32_t display_index,
                                 bool point_is_resistance_domain)
{
  if (point == NULL || summary == NULL) {
    return;
  }
  const double reference_c = point->actual_mC / 1000.0;
  char ideal_res_buf[24] = { 0 };
  char raw_temp_avg_buf[24] = { 0 };
  char raw_temp_stddev_buf[24] = { 0 };
  char raw_res_avg_buf[24] = { 0 };
  char raw_res_stddev_buf[24] = { 0 };
  char residual_res_buf[24] = { 0 };
  char residual_temp_buf[24] = { 0 };
  char drift_buf[24] = { 0 };
  char delta_buf[24] = { 0 };
  char drift_limit_buf[24] = { 0 };
  char window_buf[24] = { 0 };
  char ema_alpha_buf[24] = { 0 };
  char drift_source_buf[28] = { 0 };

  snprintf(ideal_res_buf,
           sizeof(ideal_res_buf),
           "%s",
           summary->ideal_ref_res_available ? "" : "n/a");
  if (summary->ideal_ref_res_available) {
    snprintf(
      ideal_res_buf, sizeof(ideal_res_buf), "%.3f", summary->ideal_ref_res_ohm);
  }
  snprintf(raw_temp_avg_buf,
           sizeof(raw_temp_avg_buf),
           "%s",
           summary->captured_raw_temp_avg_available ? "" : "n/a");
  if (summary->captured_raw_temp_avg_available) {
    snprintf(raw_temp_avg_buf,
             sizeof(raw_temp_avg_buf),
             "%.3f",
             summary->captured_raw_temp_avg_c);
  }
  snprintf(raw_temp_stddev_buf,
           sizeof(raw_temp_stddev_buf),
           "%s",
           summary->captured_raw_temp_stddev_available ? "" : "n/a");
  if (summary->captured_raw_temp_stddev_available) {
    snprintf(raw_temp_stddev_buf,
             sizeof(raw_temp_stddev_buf),
             "%.3f",
             summary->captured_raw_temp_stddev_c);
  }
  snprintf(raw_res_avg_buf,
           sizeof(raw_res_avg_buf),
           "%s",
           summary->captured_raw_res_available ? "" : "n/a");
  if (summary->captured_raw_res_available) {
    snprintf(raw_res_avg_buf,
             sizeof(raw_res_avg_buf),
             "%.3f",
             summary->captured_raw_res_avg_ohm);
  }
  snprintf(raw_res_stddev_buf,
           sizeof(raw_res_stddev_buf),
           "%s",
           summary->captured_raw_res_stddev_available ? "" : "n/a");
  if (summary->captured_raw_res_stddev_available) {
    snprintf(raw_res_stddev_buf,
             sizeof(raw_res_stddev_buf),
             "%.3f",
             summary->captured_raw_res_stddev_ohm);
  }
  snprintf(residual_res_buf,
           sizeof(residual_res_buf),
           "%s",
           summary->residual_res_available ? "" : "n/a");
  if (summary->residual_res_available) {
    snprintf(residual_res_buf,
             sizeof(residual_res_buf),
             "%.3f",
             summary->residual_res_ohm);
  }
  snprintf(residual_temp_buf,
           sizeof(residual_temp_buf),
           "%s",
           summary->residual_temp_available ? "" : "n/a");
  if (summary->residual_temp_available) {
    snprintf(residual_temp_buf,
             sizeof(residual_temp_buf),
             "%.3f",
             summary->residual_temp_c);
  }
  snprintf(drift_buf, sizeof(drift_buf), "%s", "n/a");
  if (summary->captured_drift_available) {
    snprintf(
      drift_buf, sizeof(drift_buf), "%.3f", summary->captured_drift_c_per_min);
  }
  snprintf(delta_buf, sizeof(delta_buf), "%s", "n/a");
  if (summary->captured_delta_available) {
    snprintf(delta_buf, sizeof(delta_buf), "%.3f", summary->captured_delta_c);
  }
  snprintf(drift_limit_buf, sizeof(drift_limit_buf), "%s", "n/a");
  if (summary->drift_limit_available) {
    snprintf(drift_limit_buf,
             sizeof(drift_limit_buf),
             "%.3f",
             summary->drift_limit_c_per_min);
  }
  snprintf(drift_source_buf,
           sizeof(drift_source_buf),
           "%s",
           DriftLimitSourceToString_(summary->drift_limit_source));
  snprintf(window_buf, sizeof(window_buf), "%s", "n/a");
  if (summary->captured_window_available) {
    snprintf(window_buf, sizeof(window_buf), "%d", summary->captured_window_s);
  }
  snprintf(ema_alpha_buf, sizeof(ema_alpha_buf), "%s", "n/a");
  if (summary->captured_ema_alpha_available) {
    snprintf(ema_alpha_buf,
             sizeof(ema_alpha_buf),
             "%.3f",
             summary->captured_ema_alpha);
  }

  printf("  %u: reference_temp_C=%.3f ideal_ref_res_Ohm=%s "
         "captured_raw_temp_avg_C=%s captured_raw_temp_stddev_C=%s "
         "captured_raw_res_avg_Ohm=%s captured_raw_res_stddev_Ohm=%s "
         "residual_res_Ohm=%s residual_C=%s captured_drift_C_per_min=%s "
         "captured_delta_C=%s drift_limit_C_per_min=%s drift_limit_source=%s "
         "captured_window_s=%s captured_ema_alpha=%s%s\n",
         (unsigned)display_index,
         reference_c,
         ideal_res_buf,
         raw_temp_avg_buf,
         raw_temp_stddev_buf,
         raw_res_avg_buf,
         raw_res_stddev_buf,
         residual_res_buf,
         residual_temp_buf,
         drift_buf,
         delta_buf,
         drift_limit_buf,
         drift_source_buf,
         window_buf,
         ema_alpha_buf,
         point_is_resistance_domain ? "" : " point_domain=temp/manual");
}

static void
PrintCalibrationPointReportBlock_(const calibration_point_t* point,
                                  const cal_point_print_summary_t* summary,
                                  uint32_t display_index,
                                  bool point_is_resistance_domain)
{
  if (point == NULL || summary == NULL) {
    return;
  }
  const double reference_c = point->actual_mC / 1000.0;
  printf("  Point %u:\n", (unsigned)display_index);
  printf("    Point domain: %s\n",
         point_is_resistance_domain ? "resistance" : "temp/manual");
  printf("    Reference temperature: %.3f C\n", reference_c);
  if (summary->ideal_ref_res_available) {
    printf("    Ideal reference resistance: %.3f ohm\n",
           summary->ideal_ref_res_ohm);
  } else {
    printf("    Ideal reference resistance: n/a\n");
  }
  if (summary->captured_raw_temp_avg_available) {
    printf("    Captured raw average temperature: %.3f C\n",
           summary->captured_raw_temp_avg_c);
  } else {
    printf("    Captured raw average temperature: n/a\n");
  }
  if (summary->captured_raw_res_available) {
    printf("    Captured raw average resistance: %.3f ohm\n",
           summary->captured_raw_res_avg_ohm);
  } else {
    printf("    Captured raw average resistance: n/a\n");
  }
  if (summary->residual_temp_available) {
    printf("    Residual temperature: %.3f C\n", summary->residual_temp_c);
  } else {
    printf("    Residual temperature: n/a\n");
  }
  if (summary->residual_res_available) {
    printf("    Residual resistance: %.3f ohm\n", summary->residual_res_ohm);
  } else {
    printf("    Residual resistance: n/a\n");
  }
  if (summary->captured_raw_temp_stddev_available) {
    printf("    Captured raw temp stddev: %.3f C\n",
           summary->captured_raw_temp_stddev_c);
  } else {
    printf("    Captured raw temp stddev: n/a\n");
  }
  if (summary->captured_raw_res_stddev_available) {
    printf("    Captured raw resistance stddev: %.3f ohm\n",
           summary->captured_raw_res_stddev_ohm);
  } else {
    printf("    Captured raw resistance stddev: n/a\n");
  }
  if (summary->captured_drift_available) {
    printf("    Captured drift: %.3f C/min\n",
           summary->captured_drift_c_per_min);
  } else {
    printf("    Captured drift: n/a\n");
  }
  if (summary->captured_delta_available) {
    printf("    Captured delta: %.3f C\n", summary->captured_delta_c);
  } else {
    printf("    Captured delta: n/a\n");
  }
  if (summary->drift_limit_available) {
    printf("    Drift limit used: %.3f C/min\n",
           summary->drift_limit_c_per_min);
  } else {
    printf("    Drift limit used: n/a\n");
  }
  printf("    Drift limit source: %s\n",
         DriftLimitSourceToString_(summary->drift_limit_source));
  if (summary->captured_window_available) {
    printf("    Captured window duration: %d s\n", summary->captured_window_s);
  } else {
    printf("    Captured window duration: n/a\n");
  }
  if (summary->captured_ema_alpha_available) {
    printf("    Captured EMA alpha: %.3f\n", summary->captured_ema_alpha);
  } else {
    printf("    Captured EMA alpha: n/a\n");
  }
  printf("\n");
}

/**
 * @brief Print polynomial coefficients using symbolic names that map from
 * highest-order term down to constant term.
 * @param coefficients Stored polynomial coefficients in constant-first order.
 * @param degree Polynomial degree to print.
 */
static void
PrintNamedPolynomialCoefficients_(const double* coefficients, uint8_t degree)
{
  for (uint8_t symbol_index = 0; symbol_index <= degree; ++symbol_index) {
    const char symbol = (char)('a' + symbol_index);
    const uint8_t coefficient_index = (uint8_t)(degree - symbol_index);
    printf("%c: %.9g\n", symbol, coefficients[coefficient_index]);
  }
}

/**
 * @brief Print human-readable calibration equation and named coefficients.
 * @param model Active stored calibration model.
 */
static void
PrintCalibrationEquationBlock_(const calibration_model_t* model)
{
  if (model->mode == CAL_FIT_MODE_PIECEWISE) {
    printf("calibration_equation: piecewise interpolation\n");
    printf("calibration_coefficients: n/a (piecewise uses stored calibration "
           "points, not polynomial coefficients)\n");
    return;
  }

  switch (model->degree) {
    case 0:
      printf("calibration_equation: a\n");
      break;
    case 1:
      printf("calibration_equation: ax+b\n");
      break;
    case 2:
      printf("calibration_equation: ax^2+bx+c\n");
      break;
    case 3:
      printf("calibration_equation: ax^3+bx^2+cx+d\n");
      break;
    default:
      printf("calibration_equation: polynomial_degree_%u\n",
             (unsigned)model->degree);
      break;
  }
  PrintNamedPolynomialCoefficients_(model->coefficients, model->degree);
}

static double
ConsoleResistanceToTemperatureFitCallback_(double resistance_ohm, void* context)
{
  const max31865_reader_t* reader = (const max31865_reader_t*)context;
  if (reader == NULL) {
    return NAN;
  }
  return Max31865ResistanceToTemperature(reader, resistance_ohm);
}

static void
PrintCalibrationFitLabelAndValueOrNa_(const char* label,
                                      bool available,
                                      double value)
{
  PrintCalibrationFitValueOrNa_(label, available, value, "");
}

static const char*
CalibrationFitDomainDisplayString_(calibration_domain_t fit_domain)
{
  return (fit_domain == CAL_DOMAIN_RESISTANCE_OHM) ? "Resistance (ohms)"
                                                   : "Legacy temperature (C)";
}

static const char*
CalibrationFitCorrectionLabel_(calibration_domain_t fit_domain)
{
  if (fit_domain == CAL_DOMAIN_RESISTANCE_OHM) {
    return "Largest applied correction (ohms)";
  }
  if (fit_domain == CAL_DOMAIN_TEMP_C) {
    return "Largest applied correction (C)";
  }
  return "Largest applied correction (fit domain)";
}

static const char*
CalibrationFitCorrectionColumnHeader_(calibration_domain_t fit_domain)
{
  if (fit_domain == CAL_DOMAIN_RESISTANCE_OHM) {
    return "correction_ohm";
  }
  if (fit_domain == CAL_DOMAIN_TEMP_C) {
    return "correction_C";
  }
  return "applied_correction";
}

static const char*
CalibrationFitDegreeLabel_(calibration_fit_mode_t fit_mode)
{
  return (fit_mode == CAL_FIT_MODE_POLY) ? "Polynomial degree" : "Degree";
}

static void
PrintCalibrationFitValueOrNa_(const char* label,
                              bool available,
                              double value,
                              const char* unit_suffix)
{
  if (available) {
    printf("    %s: %.9g%s\n",
           label,
           value,
           (unit_suffix != NULL) ? unit_suffix : "");
  } else {
    printf("    %s: n/a\n", label);
  }
}

static void
PrintCalibrationFitDiagnosticsBlock_(
  const calibration_fit_diagnostics_t* diagnostics)
{
  if (diagnostics == NULL) {
    return;
  }
  printf("    Fit domain: %s\n",
         CalibrationFitDomainDisplayString_(diagnostics->fit_domain));
  printf("    Model type: %s\n",
         CalibrationModeToString(diagnostics->fit_mode));
  printf("    %s: %u\n",
         CalibrationFitDegreeLabel_(diagnostics->fit_mode),
         (unsigned)diagnostics->degree);
  printf("    Calibration points used: %u\n",
         (unsigned)diagnostics->point_count);
  printf("    Model parameters: %u\n", (unsigned)diagnostics->parameter_count);
  printf("    Degrees of freedom: %d\n", (int)diagnostics->degrees_of_freedom);
  PrintCalibrationFitLabelAndValueOrNa_(
    "Mean signed residual (ohms)",
    diagnostics->mean_signed_residual_ohm_available,
    diagnostics->mean_signed_residual_ohm);
  PrintCalibrationFitLabelAndValueOrNa_(
    "Mean absolute residual (ohms)",
    diagnostics->mean_abs_residual_ohm_available,
    diagnostics->mean_abs_residual_ohm);
  PrintCalibrationFitLabelAndValueOrNa_("Root mean square error (ohms)",
                                        diagnostics->rmse_ohm_available,
                                        diagnostics->rmse_ohm);
  PrintCalibrationFitLabelAndValueOrNa_(
    "Residual standard deviation (ohms)",
    diagnostics->residual_stddev_ohm_available,
    diagnostics->residual_stddev_ohm);
  PrintCalibrationFitLabelAndValueOrNa_(
    "Maximum absolute residual (ohms)",
    diagnostics->max_abs_residual_ohm_available,
    diagnostics->max_abs_residual_ohm);
  PrintCalibrationFitLabelAndValueOrNa_("Sum of squared error (ohm^2)",
                                        diagnostics->sse_ohm_available,
                                        diagnostics->sse_ohm);
  PrintCalibrationFitLabelAndValueOrNa_(
    "Mean signed residual (C)",
    diagnostics->mean_signed_residual_c_available,
    diagnostics->mean_signed_residual_c);
  PrintCalibrationFitLabelAndValueOrNa_(
    "Mean absolute residual (C)",
    diagnostics->mean_abs_residual_c_available,
    diagnostics->mean_abs_residual_c);
  PrintCalibrationFitLabelAndValueOrNa_("Root mean square error (C)",
                                        diagnostics->rmse_c_available,
                                        diagnostics->rmse_c);
  PrintCalibrationFitLabelAndValueOrNa_(
    "Residual standard deviation (C)",
    diagnostics->residual_stddev_c_available,
    diagnostics->residual_stddev_c);
  PrintCalibrationFitLabelAndValueOrNa_(
    "Maximum absolute residual (C)",
    diagnostics->max_abs_residual_c_available,
    diagnostics->max_abs_residual_c);
  PrintCalibrationFitLabelAndValueOrNa_("Sum of squared error (C^2)",
                                        diagnostics->sse_c_available,
                                        diagnostics->sse_c);
  if (diagnostics->r_squared_available &&
      diagnostics->r_squared_is_meaningful) {
    PrintCalibrationFitLabelAndValueOrNa_("R^2", true, diagnostics->r_squared);
  } else {
    printf("    R^2: n/a\n");
  }
  if (diagnostics->adjusted_r_squared_available &&
      diagnostics->r_squared_is_meaningful) {
    PrintCalibrationFitLabelAndValueOrNa_(
      "Adjusted R^2", true, diagnostics->adjusted_r_squared);
  } else {
    printf("    Adjusted R^2: n/a\n");
  }
  PrintCalibrationFitLabelAndValueOrNa_(
    CalibrationFitCorrectionLabel_(diagnostics->fit_domain),
    diagnostics->max_abs_correction_fit_domain_available,
    diagnostics->max_abs_correction_fit_domain);
}

static void
PrintCalibrationModelFitTable_(const calibration_fit_report_t* fit_report)
{
  if (fit_report == NULL) {
    return;
  }

  printf("    Fit Table:\n");
  printf("      %-4s %-13s %-15s %-14s %-16s %-12s %-14s %-16s %-14s %-16s\n",
         "pt",
         "Target (C)",
         "Target (ohm)",
         "Captured (C)",
         "Captured (ohm)",
         "Model (C)",
         "Model (ohm)",
         "Temp.Res (C)",
         "Ohm.Res",
         CalibrationFitCorrectionColumnHeader_(fit_report->summary.fit_domain));

  for (size_t i = 0; i < fit_report->point_results_count; ++i) {
    const calibration_fit_point_result_t* row = &fit_report->point_results[i];

    printf("      %-4u ", (unsigned)row->point_index);

    if (row->target_temp_c_available) {
      printf("%-13.6f ", row->target_temp_c);
    } else {
      printf("%-13s ", "n/a");
    }

    if (row->target_res_ohm_available) {
      printf("%-15.6f ", row->target_res_ohm);
    } else {
      printf("%-15s ", "n/a");
    }

    if (row->captured_raw_temp_c_available) {
      printf("%-14.6f ", row->captured_raw_temp_c);
    } else {
      printf("%-14s ", "n/a");
    }

    if (row->captured_raw_res_ohm_available) {
      printf("%-16.6f ", row->captured_raw_res_ohm);
    } else {
      printf("%-16s ", "n/a");
    }

    if (row->fitted_temp_c_available) {
      printf("%-12.6f ", row->fitted_temp_c);
    } else {
      printf("%-12s ", "n/a");
    }

    if (row->fitted_res_ohm_available) {
      printf("%-14.6f ", row->fitted_res_ohm);
    } else {
      printf("%-14s ", "n/a");
    }

    if (row->temp_residual_c_available) {
      printf("%-16.6f ", row->temp_residual_c);
    } else {
      printf("%-16s ", "n/a");
    }

    if (row->res_residual_ohm_available) {
      printf("%-14.6f ", row->res_residual_ohm);
    } else {
      printf("%-14s ", "n/a");
    }

    if (row->correction_fit_domain_available) {
      printf("%-16.6f\n", row->correction_fit_domain);
    } else {
      printf("%-16s\n", "n/a");
    }
  }
}

static void
PrintCalibrationModelFitReport_(const app_settings_t* settings)
{
  if (settings == NULL || g_runtime == NULL || g_runtime->sensor == NULL) {
    printf("Calibration Model Fit Report:\n");
    printf("  unavailable: runtime context missing\n");
    return;
  }
  if (settings->calibration_points_count == 0u) {
    printf("Calibration Model Fit Report:\n");
    printf("  no points\n");
    return;
  }
  calibration_fit_report_t report;
  esp_err_t report_result =
    CalibrationBuildFitReport(settings->calibration_points,
                              settings->calibration_points_count,
                              settings->calibration_domain,
                              &settings->calibration,
                              g_runtime->sensor->rtd_nominal_ohm,
                              ConsoleResistanceToTemperatureFitCallback_,
                              g_runtime->sensor,
                              &report);
  printf("Calibration Model Fit Report:\n");
  if (report_result != ESP_OK) {
    printf("  unavailable: fit report failed (%s)\n",
           esp_err_to_name(report_result));
    return;
  }
  PrintCalibrationEquationBlock_(&settings->calibration);
  PrintCalibrationFitDiagnosticsBlock_(&report.summary);
  PrintCalibrationModelFitTable_(&report);
}

static void
PrintCalibrationStatusUnified(const app_settings_t* settings,
                              const runtime_state_t* state)
{
  int32_t last_raw_mC = 0;
  int32_t mean_raw_mC = 0;
  int32_t stddev_mC = 0;
  int32_t last_raw_mOhm = 0;
  int32_t mean_raw_mOhm = 0;
  int32_t stddev_mOhm = 0;
  int32_t begin_mean_mC = 0;
  int32_t end_mean_mC = 0;
  int32_t delta_mC = 0;
  double drift_c_per_min = 0.0;
  double delta_c_ema = 0.0;
  double drift_c_per_min_ema = 0.0;
  bool trend_ema_initialized = false;
  (void)MaybePushCalRawSampleFromSensorWithStatus_();

  CalWindowGetStats(&last_raw_mC, &mean_raw_mC, &stddev_mC);
  CalWindowGetResistanceStats(&last_raw_mOhm, &mean_raw_mOhm, &stddev_mOhm);
  CalWindowGetTrendStats(
    &begin_mean_mC, &end_mean_mC, &delta_mC, NULL, &drift_c_per_min, NULL);
  CalWindowGetTrendEmaStats(
    &delta_c_ema, &drift_c_per_min_ema, &trend_ema_initialized);
  const size_t sample_count = CalWindowGetSampleCount();
  printf("cal_window_raw_last_c: %.3f\n", last_raw_mC / 1000.0);
  printf("cal_window_raw_avg_c: %.3f\n", mean_raw_mC / 1000.0);
  printf("cal_window_raw_stddev_c: %.3f\n", stddev_mC / 1000.0);
  printf("cal_window_raw_last_ohm: %.3f\n", last_raw_mOhm / 1000.0);
  printf("cal_window_raw_avg_ohm: %.3f\n", mean_raw_mOhm / 1000.0);
  printf("cal_window_raw_stddev_ohm: %.3f\n", stddev_mOhm / 1000.0);
  printf("cal_window_begin_mean_c: %.3f\n", begin_mean_mC / 1000.0);
  printf("cal_window_end_mean_c: %.3f\n", end_mean_mC / 1000.0);
  printf("cal_window_delta_c_raw: %.3f\n", delta_mC / 1000.0);
  printf("cal_window_drift_c_per_min_raw: %.3f\n", drift_c_per_min);
  printf("cal_window_delta_c_ema: %s%.3f\n",
         trend_ema_initialized ? "" : "seed_pending ",
         delta_c_ema);
  printf("cal_window_drift_c_per_min_ema: %s%.3f\n",
         trend_ema_initialized ? "" : "seed_pending ",
         drift_c_per_min_ema);
  printf("cal_cfg_window_s: %u\n", (unsigned)settings->cal_window_duration_s);
  printf("cal_cfg_ema_alpha: %.3f\n",
         settings->cal_trend_ema_alpha_permille / 1000.0);
  printf("cal_cfg_drift_limit_default_c_per_min: %.3f\n",
         kDefaultCaptureDriftLimitCPerMin);
  printf("cal_window_samples: %u\n", (unsigned)sample_count);
  printf("cal_window_ready: %s\n", CalWindowIsReady() ? "yes" : "no");
  printf("calibration_mode: %s\n",
         CalibrationModeToString(settings->calibration.mode));
  printf("calibration_domain: %s\n",
         (settings->calibration_domain == CAL_DOMAIN_RESISTANCE_OHM)
           ? "resistance_ohm (fit uses raw/corrected ohms)"
           : "legacy_temp_c (fit uses raw/corrected Celsius)");
  PrintCalibrationEquationBlock_(&settings->calibration);
  printf("cal_points: %u\n", (unsigned)settings->calibration_points_count);
  const char* applied_reason = NULL;
  const bool applied =
    ConsoleCalibrationIsApplied(settings, state, &applied_reason);
  printf("cal_model_valid: %s\n",
         settings->calibration.is_valid ? "yes" : "no");
  printf("cal_applied: %s\n", applied ? "yes" : "no");
  if (!applied) {
    printf("cal_applied_reason: %s\n", applied_reason);
  }
  if (settings->calibration_domain == CAL_DOMAIN_RESISTANCE_OHM) {
    printf("cal_runtime_path: raw measured ohms -> corrected ohms -> corrected "
           "temperature\n");
  }
  for (size_t index = 0; index < settings->calibration_points_count; ++index) {
    const calibration_point_t* point = &settings->calibration_points[index];
    cal_point_print_summary_t summary;
    BuildCalibrationPointPrintSummary_(point, &summary);
    const bool point_is_resistance_domain = (point->raw_avg_mOhm > 0);
    PrintCalibrationPointStatusLine_(
      point, &summary, (uint32_t)(index + 1), point_is_resistance_domain);
  }
  printf("Calibration Points Report Block:\n");
  if (settings->calibration_points_count == 0u) {
    printf("  none stored\n");
  } else {
    for (size_t index = 0; index < settings->calibration_points_count;
         ++index) {
      const calibration_point_t* point = &settings->calibration_points[index];
      cal_point_print_summary_t summary;
      BuildCalibrationPointPrintSummary_(point, &summary);
      const bool point_is_resistance_domain = (point->raw_avg_mOhm > 0);
      PrintCalibrationPointReportBlock_(
        point, &summary, (uint32_t)(index + 1), point_is_resistance_domain);
    }
  }
  PrintCalibrationModelFitReport_(settings);

  char base_last[32];
  char override_last[32];
  char base_due[32];
  char override_due[32];
  char effective_last[32];
  char effective_due[32];
  char due_date_buffer[32];

  FormatUtcEpochIso8601(settings->cal_last_utc, base_last, sizeof(base_last));
  FormatUtcEpochIso8601(
    settings->cal_last_override_utc, override_last, sizeof(override_last));
  FormatCalDueEvery(settings->cal_due_count,
                    settings->cal_due_unit,
                    base_due,
                    sizeof(base_due));
  FormatCalDueEvery(settings->cal_due_override_count,
                    settings->cal_due_override_unit,
                    override_due,
                    sizeof(override_due));

  int64_t last_cal = 0;
  uint16_t due_count = 0;
  uint8_t due_unit = 0;
  ResolveCalibrationSchedule(settings, &last_cal, &due_count, &due_unit);
  FormatUtcEpochIso8601(last_cal, effective_last, sizeof(effective_last));
  FormatCalDueEvery(due_count, due_unit, effective_due, sizeof(effective_due));

  int64_t due_date = 0;
  if (last_cal != 0 && due_count != 0) {
    due_date =
      CalComputeDueDateUtc(last_cal, due_count, (cal_due_unit_t)due_unit);
  }
  FormatUtcEpochIso8601(due_date, due_date_buffer, sizeof(due_date_buffer));

  const bool time_valid = TimeSyncIsSystemTimeValid();
  const bool time_stable = (state != NULL) ? state->cal_time_stable : false;
  const bool check_suspended =
    (state != NULL) ? state->cal_due_check_suspended : !time_valid;
  const bool overdue = (state != NULL) ? state->cal_overdue : false;
  printf("Calibration:\n");
  printf("  Last UTC (base):     %s\n", base_last);
  printf("  Last UTC (override): %s\n", override_last);
  printf("  Due every (base):    %s\n", base_due);
  printf("  Due every (override): %s\n", override_due);
  printf("  Method:             %s\n",
         (settings->cal_method[0] != '\0') ? settings->cal_method : "<unset>");
  printf("  Effective last UTC:  %s\n", effective_last);
  printf("  Effective due:       %s\n", effective_due);
  printf("  Due date:            %s\n", due_date_buffer);
  if (settings->cal_method[0] == '\0') {
    printf("  Note: Method is unset. Set it with: cal set method \"<describe "
           "your method>\"\n");
  }
  printf("  Overdue:             %s\n", overdue ? "yes" : "no");
  printf("  Time valid:          %s\n", time_valid ? "yes" : "no");
  printf("  Time stable:         %s\n", time_stable ? "yes" : "no");
  printf("  Due check:           %s\n",
         check_suspended ? "suspended" : "active");
  if (settings->cal_metar.valid != 0u) {
    PrintCalibrationMetarBlock_(
      &settings->cal_metar, "Calibration Steam Reference", true);
  } else {
    printf("Calibration Steam Reference:\n");
    printf(
      "  none stored (use: cal metar set <elevation_ft> \"<raw METAR>\")\n");
    printf("Calibration Report Block:\n");
    printf("  Pressure source type: <none>\n");
  }
}

static void
ForcePtlogRevisionForCalibrationMetadata(const char* reason)
{
  if (g_runtime == NULL || g_runtime->sd_logger == NULL) {
    return;
  }
  if (!g_runtime->sd_logger->is_mounted) {
    printf("PTLOG rollover deferred (%s): SD not mounted\n", reason);
    return;
  }
  if (!TimeSyncIsSystemTimeValid()) {
    printf("PTLOG rollover deferred (%s): system time invalid\n", reason);
    return;
  }

  runtime_state_t* state = RuntimeGetState();
  if (state == NULL) {
    printf("PTLOG rollover deferred (%s): runtime state unavailable\n", reason);
    return;
  }
  if (!RuntimeSdFsLock(state, pdMS_TO_TICKS(2000))) {
    printf("PTLOG rollover deferred (%s): SD lock timeout\n", reason);
    return;
  }

  const int64_t now_epoch = (int64_t)time(NULL);
  const uint32_t previous_signature =
    g_runtime->sd_logger->current_header_signature;
  ptlog_header_t header;
  uint32_t signature = 0;
  bool header_ok = RuntimeBuildPtlogHeader(now_epoch, &header, &signature);
  esp_err_t ensure_result = ESP_FAIL;
  if (header_ok) {
    ensure_result = SdLoggerEnsureDailyFileWithHeader(
      g_runtime->sd_logger, now_epoch, &header, signature);
  }

  RuntimeSdFsUnlock(state);

  if (!header_ok) {
    printf("PTLOG rollover failed (%s): unable to build PTLOG header\n",
           reason);
    return;
  }
  if (ensure_result != ESP_OK) {
    printf("PTLOG rollover failed (%s): %s\n",
           reason,
           esp_err_to_name(ensure_result));
    return;
  }

  if (signature != previous_signature) {
    printf("PTLOG revision rolled (%s): header changed (0x%08" PRIx32
           " -> 0x%08" PRIx32 ")\n",
           reason,
           previous_signature,
           signature);
  } else {
    printf("PTLOG header unchanged (%s); current file revision retained\n",
           reason);
  }
}

static bool
ParseUtcDateString(const char* value, int64_t* epoch_out)
{
  if (value == NULL || epoch_out == NULL) {
    return false;
  }
  int year = 0;
  int month = 0;
  int day = 0;
  char trailing = '\0';
  const int matched =
    sscanf(value, "%4d-%2d-%2d%c", &year, &month, &day, &trailing);
  if (matched != 3) {
    return false;
  }
  return TimeCivilUtcEpochFromDate(year, month, day, epoch_out);
}

/**
 * @brief Execute ParseOptionalBool.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @param index Parameter index.
 * @param target Parameter target.
 * @return Return the function result.
 */
static int
ParseOptionalBool(int argc, char** argv, int* index, bool* target)
{
  if (argv == NULL || index == NULL || target == NULL) {
    return 0;
  }
  const int i = *index;
  if ((i + 1) < argc &&
      (strcmp(argv[i + 1], "0") == 0 || strcmp(argv[i + 1], "1") == 0)) {
    *target = (argv[i + 1][0] == '1');
    *index = i + 1;
  } else {
    *target = true;
  }
  return 1;
}

/**
 * @brief Execute CommandDiagnostics.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandDiagnostics(int argc, char** argv)
{
  if (argc < 2) {
    PrintDiagUsage();
    return 2;
  }

  const char* target = argv[1];
  int verbosity = 0;
  bool format_if_needed = false;
  bool mount = false;
  bool scan = false;
  bool connect = false;
  bool dns_lookup = false;
  bool keep_connected = false;
  bool set_known = false;
  int bytes = 0;
  int samples = 0;
  int delay_ms = -1;
  bool start_mesh = false;
  bool stop_mesh = false;
  bool mesh_force_root = false;
  int mesh_timeout_ms = 10000;

  const app_runtime_t* runtime = RuntimeGetRuntime();

  if (strcmp(target, "help") == 0) {
    PrintDiagUsage();
    return 0;
  }

  if (strcmp(target, "heapcheck") == 0) {
    runtime_state_t* state = RuntimeGetState();
    if (state == NULL) {
      printf("diag heapcheck failed: runtime unavailable\n");
      return 1;
    }
    if (argc < 3) {
      printf("diag heapcheck: %s\n",
             state->diag_heap_check_enabled ? "on" : "off");
      printf("usage: diag heapcheck on|off|now\n");
      return 0;
    }
    const char* action = argv[2];
    if (strcmp(action, "on") == 0) {
      state->diag_heap_check_enabled = true;
      printf("diag heapcheck: on\n");
      return 0;
    }
    if (strcmp(action, "off") == 0) {
      state->diag_heap_check_enabled = false;
      printf("diag heapcheck: off\n");
      return 0;
    }
    if (strcmp(action, "now") == 0) {
      const bool ok = RuntimeDiagHeapCheck(state, "diag heapcheck now", true);
      printf("diag heapcheck: %s\n", ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    printf("usage: diag heapcheck on|off|now\n");
    return 1;
  }

  if (strcmp(target, "cycle") == 0) {
    int count = 1;
    int run_ms = 1000;
    int stop_ms = 0;
    for (int i = 2; i < argc; ++i) {
      if (strcmp(argv[i], "--count") == 0 && (i + 1) < argc) {
        count = atoi(argv[++i]);
      } else if (strcmp(argv[i], "--run_ms") == 0 && (i + 1) < argc) {
        run_ms = atoi(argv[++i]);
      } else if (strcmp(argv[i], "--stop_ms") == 0 && (i + 1) < argc) {
        stop_ms = atoi(argv[++i]);
      } else {
        printf("unknown option: %s\n", argv[i]);
        PrintDiagUsage();
        return 2;
      }
    }

    if (count <= 0 || run_ms < 0 || stop_ms < 0) {
      printf("invalid values: count > 0, run_ms >= 0, stop_ms >= 0\n");
      PrintDiagUsage();
      return 2;
    }

    runtime_state_t* state = RuntimeGetState();
    if (state == NULL) {
      printf("diag cycle failed: runtime unavailable\n");
      return 1;
    }

    if (RuntimeIsRunning()) {
      printf("diag cycle: runtime running; stopping first\n");
      esp_err_t stop_result = EnterDiagMode();
      if (stop_result != ESP_OK) {
        printf("diag cycle: stop failed: %s\n", esp_err_to_name(stop_result));
        return 1;
      }
    }

    size_t min_free_heap = SIZE_MAX;
    size_t min_free_internal = SIZE_MAX;
    uint32_t spi_min = UINT32_MAX;
    uint32_t spi_max = 0;
    int start_fail = 0;
    int stop_fail = 0;
    int mount_attempts = 0;
    int mount_ok = 0;
    int unmount_attempts = 0;
    int unmount_ok = 0;

    for (int cycle = 1; cycle <= count; ++cycle) {
      printf("diag cycle %d/%d: start\n", cycle, count);
      esp_err_t start_result = EnterRunMode();
      if (start_result != ESP_OK) {
        printf("diag cycle: start failed on cycle %d: %s\n",
               cycle,
               esp_err_to_name(start_result));
        start_fail++;
        break;
      }

      if (run_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(run_ms));
      }

      size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
      if (free_heap < min_free_heap) {
        min_free_heap = free_heap;
      }
      size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      if (free_internal < min_free_internal) {
        min_free_internal = free_internal;
      }

      uint32_t spi_count = RuntimeGetSpiDeviceCount();
      if (spi_count < spi_min) {
        spi_min = spi_count;
      }
      if (spi_count > spi_max) {
        spi_max = spi_count;
      }

      if (SdCardDetectIsPresent(&state->sd_card_detect)) {
        mount_attempts++;
        if (state->sd_logger.is_mounted) {
          mount_ok++;
        }
      }

      printf("diag cycle %d/%d: stop\n", cycle, count);
      esp_err_t stop_result = EnterDiagMode();
      if (stop_result != ESP_OK) {
        printf("diag cycle: stop failed on cycle %d: %s\n",
               cycle,
               esp_err_to_name(stop_result));
        stop_fail++;
      }

      free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
      if (free_heap < min_free_heap) {
        min_free_heap = free_heap;
      }
      free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      if (free_internal < min_free_internal) {
        min_free_internal = free_internal;
      }

      spi_count = RuntimeGetSpiDeviceCount();
      if (spi_count < spi_min) {
        spi_min = spi_count;
      }
      if (spi_count > spi_max) {
        spi_max = spi_count;
      }

      if (SdCardDetectIsPresent(&state->sd_card_detect)) {
        unmount_attempts++;
        if (!state->sd_logger.is_mounted) {
          unmount_ok++;
        }
      }

      if (stop_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(stop_ms));
      }
    }

    if (min_free_heap == SIZE_MAX) {
      min_free_heap = 0;
    }
    if (min_free_internal == SIZE_MAX) {
      min_free_internal = 0;
    }
    if (spi_min == UINT32_MAX) {
      spi_min = 0;
    }

    printf("diag cycle done: cycles=%d start_fail=%d stop_fail=%d\n",
           count,
           start_fail,
           stop_fail);
    printf("diag cycle heap min: default=%u internal=%u\n",
           (unsigned)min_free_heap,
           (unsigned)min_free_internal);
    printf("diag cycle spi devices: min=%" PRIu32 " max=%" PRIu32 "\n",
           spi_min,
           spi_max);
    printf("diag cycle SD mount ok=%d/%d unmount ok=%d/%d\n",
           mount_ok,
           mount_attempts,
           unmount_ok,
           unmount_attempts);

    if (start_fail != 0 || stop_fail != 0) {
      return 1;
    }
    if ((mount_attempts > 0 && mount_ok != mount_attempts) ||
        (unmount_attempts > 0 && unmount_ok != unmount_attempts)) {
      return 1;
    }
    return 0;
  }

  const bool target_requires_mode =
    strcmp(target, "all") == 0 || strcmp(target, "sd") == 0 ||
    strcmp(target, "storage") == 0 || strcmp(target, "fram") == 0 ||
    strcmp(target, "rtc") == 0 || strcmp(target, "rtd") == 0 ||
    strcmp(target, "wifi") == 0 || strcmp(target, "mesh") == 0;
  const char* mode = (argc > 2) ? argv[2] : NULL;
  if (strcmp(target, "check") == 0) {
    target = "all";
    mode = "quick";
  } else if (!target_requires_mode) {
    printf("unknown diag target. try 'diag help'\n");
    return 2;
  }

  if (mode == NULL ||
      (strcmp(mode, "quick") != 0 && strcmp(mode, "full") != 0)) {
    printf("missing or invalid mode (quick|full)\n");
    PrintDiagUsage();
    return 2;
  }

  const bool full = strcmp(mode, "full") == 0;

  if (strcmp(target, "wifi") == 0 || strcmp(target, "all") == 0) {
    scan = true;
    if (full) {
      connect = true;
      dns_lookup = true;
    }
  }

  for (int i = 3; i < argc; ++i) {
    if (strcmp(argv[i], "--verbose") == 0 && (i + 1) < argc) {
      if (!ParseVerbose(argv[i + 1], &verbosity)) {
        printf("--verbose requires an integer value\n");
        PrintDiagUsage();
        return 2;
      }
      ++i;
    } else if (strcmp(argv[i], "--format-if-needed") == 0) {
      format_if_needed = true;
    } else if (strcmp(argv[i], "--mount") == 0) {
      mount = true;
    } else if (strcmp(argv[i], "--scan") == 0) {
      ParseOptionalBool(argc, argv, &i, &scan);
    } else if (strcmp(argv[i], "--connect") == 0) {
      ParseOptionalBool(argc, argv, &i, &connect);
    } else if (strcmp(argv[i], "--dns") == 0) {
      ParseOptionalBool(argc, argv, &i, &dns_lookup);
    } else if (strcmp(argv[i], "--keep-connected") == 0) {
      ParseOptionalBool(argc, argv, &i, &keep_connected);
    } else if (strcmp(argv[i], "--set-known") == 0) {
      set_known = true;
    } else if (strcmp(argv[i], "--bytes") == 0 && (i + 1) < argc) {
      bytes = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--samples") == 0 && (i + 1) < argc) {
      samples = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--delay_ms") == 0 && (i + 1) < argc) {
      delay_ms = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--start") == 0) {
      start_mesh = true;
    } else if (strcmp(argv[i], "--stop") == 0) {
      stop_mesh = true;
    } else if (strcmp(argv[i], "--root") == 0) {
      mesh_force_root = true;
    } else if (strcmp(argv[i], "--timeout_ms") == 0 && (i + 1) < argc) {
      mesh_timeout_ms = atoi(argv[++i]);
    } else {
      printf("unknown option: %s\n", argv[i]);
      PrintDiagUsage();
      return 2;
    }
  }

  int overall = 0;
  const diag_verbosity_t diag_verbosity =
    (verbosity >= 2) ? kDiagVerbosity2
                     : ((verbosity > 0) ? kDiagVerbosity1 : kDiagVerbosity0);

  if (strcmp(target, "sd") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |=
        RunDiagSd(runtime, full, format_if_needed, mount, diag_verbosity);
    }
    if (strcmp(target, "sd") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "storage") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |= RunDiagStorage(runtime, full, diag_verbosity);
    }
    if (strcmp(target, "storage") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "fram") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |= RunDiagFram(runtime, full, bytes, diag_verbosity);
    }
    if (strcmp(target, "fram") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "rtd") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |= RunDiagRtd(runtime, full, samples, delay_ms, diag_verbosity);
    }
    if (strcmp(target, "rtd") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "rtc") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |= RunDiagRtc(runtime, full, set_known, diag_verbosity);
    }
    if (strcmp(target, "rtc") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "wifi") == 0 || strcmp(target, "all") == 0) {
    overall |= RunDiagWifi(
      runtime, full, scan, connect, dns_lookup, keep_connected, diag_verbosity);
    if (strcmp(target, "wifi") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "mesh") == 0 || strcmp(target, "all") == 0) {
    if (full && !start_mesh && !stop_mesh) {
      start_mesh = true;
      stop_mesh = true;
    }
    overall |= RunDiagMesh(runtime,
                           full,
                           start_mesh,
                           stop_mesh,
                           mesh_force_root,
                           mesh_timeout_ms,
                           diag_verbosity);
    if (strcmp(target, "mesh") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "all") == 0) {
    return overall;
  }

  printf("unknown diag target. try 'diag help'\n");
  return 2;
}

/**
 * @brief Execute CommandReboot.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandReboot(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  printf("rebooting...\n");
  esp_restart();
  return 0;
}

/**
 * @brief Execute CommandMarker.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandMarker(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: marker show|clear\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "show") == 0) {
    RuntimeMarkersDump(RuntimeGetState());
    return 0;
  }
  if (strcmp(action, "clear") == 0) {
    RuntimeMarkersClear();
    printf("Markers cleared.\n");
    return 0;
  }

  printf("Unknown marker action: %s\n", action);
  return 1;
}

/**
 * @brief Execute CommandDeferLog.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandDeferLog(int argc, char** argv)
{
  if (argc < 2 || strcmp(argv[1], "status") != 0) {
    printf("usage: deferlog status\n");
    return (argc < 2) ? 2 : 1;
  }

  const runtime_cached_status_t* status = RuntimeGetCachedStatus();
  if (status == NULL) {
    printf("runtime unavailable\n");
    return 1;
  }

  printf("deferred_count: %u\n", (unsigned)status->deferred_count);
  printf("deferred_drops: %u\n", (unsigned)status->deferred_drops);
  printf("deferred_active: %s\n", status->deferred_active ? "yes" : "no");
  printf("i2c_quiesce_active: %s\n", status->i2c_quiesce_active ? "yes" : "no");
  return 0;
}

/**
 * @brief Execute CommandLocks.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandLocks(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  RuntimeDumpLocksManual("manual");
  RuntimeDumpI2cOpStateManual("manual");
  runtime_state_t* state = RuntimeGetState();
  if (state != NULL) {
    const i2c_bus_state_t bus_state = I2cBusGetState(&state->i2c_bus);
    printf("i2c bus: port=%d sda_pin=%d scl_pin=%d sda_level=%d scl_level=%d "
           "controller_busy=%u\n",
           (int)state->i2c_bus.port,
           state->i2c_bus.sda_gpio,
           state->i2c_bus.scl_gpio,
           bus_state.sda_level,
           bus_state.scl_level,
           bus_state.controller_busy ? 1u : 0u);
  }
  return 0;
}

static void
PrintMqttHelpBody(void)
{
  printf("SUBCOMMANDS\n");
  printf("  show                              Show MQTT configuration\n");
  printf(
    "  enable on|off                     Enable/disable MQTT publishing\n");
  printf("  broker set <uri>                  Set broker URI\n");
  printf("  prefix set <prefix>               Set topic prefix\n");
  printf("  qos set 0|1                       Set QoS level\n");
  printf("  retain set on|off                 Configure retain flag\n");
  printf("  bridge set off|serial|broker|both Configure bridge mode\n\n");
  printf("EXAMPLES\n");
  printf("  mqtt show\n");
  printf("  mqtt enable on\n");
  printf("  mqtt broker set mqtt://192.168.1.10\n");
}

static void
PrintWifiHelpBody(void)
{
  printf("SUBCOMMANDS\n");
  printf(
    "  show                              Show saved station credentials\n");
  printf("  set <ssid> [password]             Store station credentials\n");
  printf("  clear                             Remove saved credentials\n");
  printf("  scan [--max N]                    Scan for APs\n");
  printf("  status                            Show Wi-Fi link status\n");
  printf(
    "  connect [--timeout_ms T]          Connect using saved credentials\n");
  printf("  disconnect                        Disconnect from AP\n");
  printf(
    "  ntp status|sync ...               Inspect or trigger time sync\n\n");
  printf("EXAMPLES\n");
  printf("  wifi scan --max 10\n");
  printf("  wifi set MySsid supersecret\n");
  printf("  wifi connect --timeout_ms 20000\n");
  printf("  wifi ntp sync pool.ntp.org\n");
}

static void
PrintSdHelpBody(void)
{
  printf("SUBCOMMANDS\n");
  printf("  status | mount | unmount | format | view | verify\n\n");
  printf("NOTES\n");
  printf(
    "  sd verify controls readback verification mode for future appends.\n");
  printf("  It does not run an immediate verification pass.\n\n");
  printf("EXAMPLES\n");
  printf("  sd status\n");
  printf("  sd verify\n");
  printf("  sd verify on\n");
  printf("  sd verify off\n");
}

static const console_help_topic_t kSdTopics[] = {
  {
    .name = "verify",
    .summary = "Show or set SD append readback-verification mode",
    .synopsis = "sd verify\nsd verify on\nsd verify off",
    .details = "Shows or toggles the readback verification mode used for "
               "future SD append operations. This command does not run an "
               "immediate verification sweep.",
    .options = "  on    Enable readback verification for future appends.\n"
               "  off   Disable readback verification for future appends.",
    .examples = "  sd verify\n"
                "  sd verify on\n"
                "  sd verify off",
  },
  { 0 }
};

static int
SdTopicHelp(const char* topic)
{
  if (topic == NULL || topic[0] == '\0') {
    return 1;
  }

  for (size_t i = 0; kSdTopics[i].name != NULL; ++i) {
    if (strcmp(kSdTopics[i].name, topic) == 0) {
      ConsoleHelpPrintTopicManpage("sd", &kSdTopics[i]);
      return 0;
    }
  }

  return 1;
}

static void
PrintFramHelpBody(void)
{
  printf("SUBCOMMANDS\n");
  printf("  status | show | clear\n\n");
  printf("EXAMPLES\n");
  printf("  fram status\n");
  printf("  fram clear\n");
}

static void
PrintErrlogHelpBody(void)
{
  printf("EXAMPLES\n");
  printf("  errlog show --last 20\n");
  printf("  errlog stats\n");
  printf("  errlog clear\n");
}

static void
PrintDiagHelpBody(void)
{
  printf("EXAMPLES\n");
  printf("  diag help\n");
  printf("  diag sd check\n");
  printf("  diag wifi connect\n");
}

static const console_help_topic_t kCalTopics[] = {
  {
    .name = "cfg",
    .summary = "Show/set calibration window duration and EMA smoothing",
    .synopsis = "cal cfg show\n"
                "cal cfg set ema_alpha <0.0..1.0>\n"
                "cal cfg set window_s <seconds>",
    .details =
      "Displayed drift and delta are EMA-smoothed for operator readability; "
      "capture gating uses |smoothed drift|. Window duration controls the "
      "analysis span for mean/stddev/delta/regression drift. Values are "
      "persisted in NVS and applied to future cal live/cal livecal/cal "
      "capture sessions.",
    .examples = "  cal cfg show\n"
                "  cal cfg set ema_alpha 0.200\n"
                "  cal cfg set window_s 60",
  },
  {
    .name = "add",
    .summary = "Add a manual legacy temp-domain point (raw vs actual Celsius)",
    .synopsis = "cal add <raw_c> <C>",
    .details =
      "Adds one calibration point to the in-memory list and saves the points "
      "to NVS. This command is legacy/manual temp-domain only. Preferred "
      "modern workflow is 'cal capture <actual_temp_c>' so the point includes "
      "captured raw temperature and raw resistance statistics. If a point "
      "already exists with the same reference temperature (exact mC match), "
      "the existing point is overwritten in place instead of appending a "
      "duplicate.",
    .options =
      "  <raw_c>    Raw sensor temperature in Celsius (supports negatives).\n"
      "  <C>        Reference (actual) temperature in Celsius.",
    .examples = "  cal add 24.81 25.00\n"
                "  cal add -10.12 -10.00",
  },
  {
    .name = "del",
    .summary = "Delete one calibration point by 1-based index (remove alias)",
    .synopsis = "cal del <index>\ncal remove <index>",
    .details =
      "Deletes one saved calibration point by its displayed 1-based index, "
      "compacts remaining points, and saves changes to NVS.",
    .options = "  <index>    1-based index shown by cal list/cal status.",
    .examples = "  cal list\n"
                "  cal del 1\n"
                "  cal remove 2",
  },
  {
    .name = "apply",
    .summary = "Fit and persist a calibration model from saved points",
    .synopsis =
      "cal apply [--mode linear|piecewise|polyN] [--allow_wide_slope]",
    .details =
      "Builds a model from saved points, writes it to NVS, and updates "
      "calibration metadata (including last calibration time when system time "
      "is valid). This changes runtime calibration immediately and persists "
      "across reboot.\n"
      "Point-set domain behavior:\n"
      "  - Temp-domain set (legacy): fit raw temperature -> reference "
      "temperature.\n"
      "  - Resistance-domain set (preferred): if captured points include raw "
      "resistance, each entered reference temperature is converted to ideal "
      "PT100 resistance using the existing CVD helper, then fit is performed "
      "in ohms.\n"
      "  - Mixed temp-only and resistance points are rejected.\n"
      "Runtime path when resistance-domain calibration is active:\n"
      "  raw measured ohms -> corrected ohms -> corrected temperature.\n"
      "After fitting, cal apply prints an audit-oriented model-fit diagnostics "
      "suite (domain-aware metrics with n/a where not meaningful).\n"
      "Operator note: cal apply warns when calibration method text is unset; "
      "set it with 'cal set method \"...\"' for traceable records.",
    .options =
      "  --mode <linear|piecewise|polyN>  Fit mode (polyN supports N up to "
      "firmware limit).\n"
      "  --allow_wide_slope              Relax slope constraints for fitting.",
    .examples = "  cal apply\n"
                "  cal apply --mode piecewise\n"
                "  cal apply --mode poly2 --allow_wide_slope",
  },
  {
    .name = "status",
    .summary = "Show unified calibration status, points, and schedule metadata",
    .synopsis = "cal status",
    .details = "Prints window statistics, model/point state, residuals, and "
               "calibration schedule metadata (last/due/method/time status) in "
               "one deduplicated output. Also prints the stored calibration "
               "equation and named coefficients for polynomial models, "
               "explicitly identifies piecewise interpolation mode, emits the "
               "stored calibration steam-reference METAR block, reports "
               "per-point ideal reference resistance + residuals (C and ohms), "
               "includes copy/paste-friendly report blocks for both "
               "steam-reference and calibration points, and adds a separate "
               "Calibration Model Fit Report block with full model-fit "
               "diagnostics + per-point fit table.",
    .options = NULL,
    .examples = "  cal status",
  },
  {
    .name = "metar",
    .summary = "Parse/store calibration steam-reference data from a raw METAR",
    .synopsis = "cal metar set <elevation_ft> \"<raw METAR line>\"\n"
                "cal metar show\n"
                "cal metar clear",
    .details =
      "Parses METAR station/observation/altimeter fields, computes station "
      "pressure from altimeter + elevation (same path used by satpt), computes "
      "steam-point saturation temperature, and persists the block in the "
      "authoritative calibration metadata stored in NVS. cal status then emits "
      "both diagnostics and report-ready steam-reference fields from this "
      "stored metadata.",
    .options =
      "  set <elevation_ft> \"<raw METAR>\"  Parse, compute, and persist.\n"
      "  show                              Show currently stored METAR block.\n"
      "  clear                             Remove stored METAR block.",
    .examples = "  cal metar set 1391 \"METAR KBJI 081955Z AUTO 26010KT 10SM "
                "CLR 05/M02 A2969 RMK AO2 T00501022\"\n"
                "  cal metar show\n"
                "  cal metar clear",
  },
  {
    .name = "set",
    .summary = "Set calibration schedule metadata and method text",
    .synopsis = "cal set date <YYYY-MM-DD>\n"
                "cal set due <count> <days|months|years>\n"
                "cal set due_override <count> <days|months|years>\n"
                "cal set method <string...>",
    .details =
      "Updates calibration metadata stored in NVS. Method text should "
      "describe how calibration was performed and is embedded in PTLOG "
      "headers; changes trigger immediate PTLOG revision rollover when "
      "SD is mounted and system time is valid.",
    .options = "  date <YYYY-MM-DD>                Override last calibration "
               "date (UTC).\n"
               "  due <count> <unit>               Baseline due interval.\n"
               "  due_override <count> <unit>      Override due interval.\n"
               "  method <string...>               Method description; quote "
               "multi-word text.",
    .examples = "  cal set date 2026-01-15\n"
                "  cal set due 12 months\n"
                "  cal set due_override 30 days\n"
                "  cal set method \"ice + satpt steam\"",
  },
  {
    .name = "capture",
    .summary = "Capture a stable point from live sensor data and save it",
    .synopsis = "cal capture <actual_temp_c> [--stable_stddev_c 0.05] "
                "[--min_seconds 5] [--timeout_seconds 120] "
                "[--drift_c_per_min 0.020] [--no-drift-limit]",
    .details =
      "Starts a background capture operation (preferred modern calibration "
      "command). When the raw window stays within the stability threshold for "
      "the minimum duration, firmware saves one resistance-domain point. If a "
      "point with the same entered reference temperature already exists "
      "(exact mC match), it is updated in place instead of appending.\n"
      "When cal live is already running, cal capture attaches to that shared "
      "session, reuses the current buffered live data, and can capture "
      "immediately if existing buffered data already satisfies the requested "
      "criteria.\n"
      "Domain guardrail: if any temp-domain points exist, capture is refused "
      "until 'cal clear' is used.\n"
      "Each captured point stores:\n"
      "  - entered reference temperature (C)\n"
      "  - captured raw temperature average/stddev\n"
      "  - captured raw resistance average/stddev\n"
      "  - captured window drift (C/min) and window delta (C)\n"
      "Drift is computed as a least-squares regression slope of temperature "
      "vs time across the full active window. Delta is end-window mean minus "
      "begin-window mean; drift and delta are different metrics. Displayed "
      "drift/delta are EMA-smoothed; capture drift gating uses |smoothed "
      "drift|.\n"
      "During capture, status lines display the instantaneous last sample "
      "(temperature and resistance) along with running statistics.\n"
      "Recommended fixed-point workflow (ice + steam):\n"
      "  1) cal clear\n"
      "  2) cal set method \"slushy ice bath + satpt steam space\"\n"
      "  3) cal live (confirm stability before capture)\n"
      "  4) Build slushy ice bath (well-packed crushed ice + small amount of "
      "water), equilibrate, then capture near 0 C.\n"
      "  5) For steam-point work, run satpt first and use the computed "
      "Celsius result in cal capture.\n"
      "  6) Prefer steam-space above boiling water (not probe immersion in "
      "boiling liquid).\n"
      "  7) Do not assume 100.0 C unless local pressure conditions justify "
      "that value.\n"
      "  8) cal apply.\n"
      "Use 'cal stop' to abort an active live/livecal/capture operation.",
    .options =
      "  <actual_temp_c>             Reference temperature in Celsius (C "
      "suffix allowed).\n"
      "  --stable_stddev_c <C>       Stability threshold for raw window "
      "stddev.\n"
      "  --min_seconds <seconds>     Required stable duration before saving.\n"
      "  --timeout_seconds <seconds> Max capture time before abort.\n"
      "  --drift_c_per_min <C/min>   Require |smoothed drift| <= value "
      "(default 0.020 C/min).\n"
      "  --no-drift-limit            Disable drift gating.",
    .examples = "  cal clear\n"
                "  cal set method \"slushy ice bath + satpt steam space\"\n"
                "  cal live\n"
                "  cal capture 0.0 --stable_stddev_c 0.02 --min_seconds 8\n"
                "  satpt A2922 1315\n"
                "  cal capture 98.7 --stable_stddev_c 0.05 --min_seconds 8\n"
                "  cal apply\n"
                "  cal stop",
  },
  {
    .name = "import",
    .summary = "Manually import/restore a resistance-domain calibration point",
    .synopsis = "cal import <raw_ohm> <actual_c> [--raw_c <value>] "
                "[--stddev_c <value>]\n"
                "cal import <cal status point line>\n"
                "cal import \"<cal status point line>\"\n"
                "cal restore <raw_ohm> <actual_c> [--raw_c <value>] "
                "[--stddev_c <value>]",
    .details =
      "Adds or updates one captured-style resistance-domain calibration "
      "point from known values, without live capture. Required fields are raw "
      "resistance in ohms and reference temperature in Celsius. Optional "
      "captured raw temperature average/stddev may also be provided.\n"
      "The command also accepts a pasted point line copied directly from "
      "'cal status' and restores all available per-point metadata fields "
      "(including drift/window fields) when present. Pasted status-line "
      "imports work both with quotes and without quotes.\n"
      "Domain guardrail: if temp-domain legacy points exist, import is "
      "refused until 'cal clear' is used.\n"
      "Overwrite behavior: if a point already exists with the same reference "
      "temperature (exact mC match), that point is updated in place.",
    .options =
      "  <raw_ohm>                  Measured raw resistance in ohms (>0).\n"
      "  <actual_c>                 Reference temperature in Celsius.\n"
      "  --raw_c <value>            Optional captured raw average Celsius.\n"
      "  --stddev_c <value>         Optional captured raw stddev Celsius "
      "(>=0).\n"
      "  <status line>              Pasted line from cal status/report "
      "(quoted or unquoted).",
    .examples = "  cal import 99.970 0.0 --raw_c -0.078 --stddev_c 0.019\n"
                "  cal import 135.650 98.388 --raw_c 92.475 --stddev_c 0.036\n"
                "  cal restore 99.970 0.0\n"
                "  cal import \"1: reference_temp_C=98.795 "
                "captured_raw_res_avg_Ohm=138.132 "
                "captured_drift_C_per_min=0.010 "
                "drift_limit_C_per_min=0.020 drift_limit_source=DEFAULT "
                "captured_window_s=8 captured_ema_alpha=0.250\"\n"
                "  cal import 1: reference_temp_C=98.795 "
                "captured_raw_res_avg_Ohm=138.132 "
                "captured_drift_C_per_min=0.010 "
                "drift_limit_C_per_min=0.020 drift_limit_source=DEFAULT "
                "captured_window_s=8 captured_ema_alpha=0.250",
  },
  {
    .name = "clear",
    .summary = "Clear calibration model/points or schedule metadata",
    .synopsis = "cal clear | cal clear <date|due|due_override|method>",
    .details =
      "Without a target, resets the active calibration model to identity "
      "(y=x), removes all stored calibration points, and saves both changes. "
      "With a target, clears only that schedule/method field.",
    .options =
      "  (none)          Reset model and erase all calibration points.\n"
      "  date            Clear calibration override date.\n"
      "  due             Clear baseline due interval.\n"
      "  due_override    Clear due override interval.\n"
      "  method          Clear saved calibration method text.",
    .examples = "  cal clear\n"
                "  cal clear due_override",
  },
  {
    .name = "list",
    .summary = "List all currently saved calibration points",
    .synopsis = "cal list",
    .details = "Prints each saved point index with raw average, actual "
               "temperature, standard deviation, resistance fields, and sample "
               "count. For imported resistance points where optional raw "
               "temperature values were omitted, those fields are shown as "
               "n/a.",
    .options = NULL,
    .examples = "  cal list",
  },
  {
    .name = "live",
    .summary = "Stream live calibration-window statistics",
    .synopsis = "cal live [seconds] [--every_ms 1000] [--seconds N] "
                "[--drift_c_per_min X]",
    .details =
      "Starts a background live print loop of window stats. Drift is "
      "least-squares regression slope over full window samples (C/min), while "
      "delta is end-window mean minus begin-window mean (C). Displayed drift "
      "and delta values are EMA-smoothed. Optional drift notification can be "
      "armed with --drift_c_per_min; threshold is interpreted as absolute "
      "value and fires once when |smoothed drift| stays under threshold for "
      "the configured calibration window duration. Notification state resets "
      "when 'cal stop' is run or a new cal live session starts. The run can "
      "be bounded by seconds (positional or --seconds) and can be canceled "
      "with 'cal stop'. cal capture may be invoked while cal live is running; "
      "capture reuses this same live session buffer/state.",
    .options =
      "  [seconds]          Optional positional duration in seconds.\n"
      "  --every_ms <ms>    Print period in milliseconds (default 1000).\n"
      "  --seconds <N>      Optional named duration (do not combine with "
      "positional).\n"
      "  --drift_c_per_min X Send one ntfy notification when |drift| <= |X| "
      "continuously for the calibration window duration.",
    .examples = "  cal live\n"
                "  cal live 30 --every_ms 500\n"
                "  cal live --seconds 20\n"
                "  cal live --drift_c_per_min -0.020",
  },
  {
    .name = "livecal",
    .summary = "Stream live calibrated-window statistics",
    .synopsis = "cal livecal [seconds] [--every_ms 1000] [--seconds N] "
                "[--drift_c_per_min X]",
    .details =
      "Starts the same shared background live loop used by cal live, but "
      "prints calibrated temperature fields alongside raw resistance stats. "
      "Start is blocked unless calibration is currently applied/valid "
      "(points present, model valid, due-check active, and not overdue). "
      "No raw fallback occurs; command exits with a clear reason when "
      "blocked. Drift/delta and optional drift notification behavior match "
      "cal live exactly.",
    .options =
      "  [seconds]          Optional positional duration in seconds.\n"
      "  --every_ms <ms>    Print period in milliseconds (default 1000).\n"
      "  --seconds <N>      Optional named duration (do not combine with "
      "positional).\n"
      "  --drift_c_per_min X Send one ntfy notification when |drift| <= |X| "
      "continuously for the calibration window duration.",
    .examples = "  cal livecal\n"
                "  cal livecal 30 --every_ms 500\n"
                "  cal livecal --seconds 20\n"
                "  cal livecal --drift_c_per_min 0.020",
  },
  {
    .name = "stop",
    .summary = "Request cancellation of an active live/capture operation",
    .synopsis = "cal stop",
    .details =
      "If a calibration background operation is running, requests cancellation "
      "of the shared calibration session and returns immediately. This clears "
      "live streaming, any armed capture request, and associated stability "
      "tracking state. If none is active, reports that state.",
    .options = NULL,
    .examples = "  cal stop",
  },
  { 0 }
};

static const console_help_topic_t kModeTopics[] = {
  {
    .name = "show",
    .summary = "Show stored boot mode and current runtime streaming state",
    .synopsis = "mode show",
    .details = "Prints nvs_boot_mode, current_mode, and data_streaming so "
               "operators can compare persisted boot defaults with live "
               "runtime behavior.",
    .options = NULL,
    .examples = "  mode show",
  },
  {
    .name = "run",
    .summary = "Switch to run policy immediately (runtime only)",
    .synopsis = "mode run",
    .details = "Applies run logging policy immediately and enables data "
               "streaming without writing NVS.",
    .options = NULL,
    .examples = "  mode run",
  },
  {
    .name = "diag",
    .summary = "Switch to diagnostics policy immediately (runtime only)",
    .synopsis = "mode diag",
    .details = "Applies diagnostics logging policy immediately and disables "
               "data streaming without writing NVS.",
    .options = NULL,
    .examples = "  mode diag",
  },
  {
    .name = "set",
    .summary = "Persist boot default mode in NVS",
    .synopsis = "mode set diag|run",
    .details = "Writes the boot default mode to NVS for next startup. Reboot "
               "is required and runtime mode may not change immediately.",
    .options = "  <diag|run>    Boot mode value to persist in NVS.",
    .examples = "  mode set run\n"
                "  mode set diag",
  },
  { 0 }
};

static void
PrintCalHelpBody(void)
{
  ConsoleHelpPrintTopicList(kCalTopics);
  printf("Use: help cal <subcommand>\n\n");
  printf("RECOMMENDED WORKFLOW\n");
  printf("  1) cal clear\n");
  printf("  2) cal set method \"slushy ice bath + satpt steam space\"\n");
  printf("  3) cal live   (confirm stable readings first)\n");
  printf("  4) cal capture 0.0 --stable_stddev_c 0.02 --min_seconds 8 "
         "--drift_c_per_min 0.020\n");
  printf("  5) cal metar set 1391 \"METAR KBJI 081955Z AUTO ... A2969 ...\"\n");
  printf("  6) cal status   (copy point + steam-reference report blocks)\n");
  printf("  7) cal capture <satpt from cal metar> --stable_stddev_c 0.05 "
         "--min_seconds 8 --drift_c_per_min 0.020\n");
  printf("  8) cal apply\n");
  printf(
    "  Manual restore option: use 'cal import' (or alias 'cal restore')\n");
  printf("  when raw resistance/reference values are already known.\n");
  printf("  'cal add' is legacy temp-domain only.\n");
  printf(
    "  Note: satpt is an ad hoc calculator; cal metar parses+stores report\n");
  printf(
    "        metadata for audit use and uses the same pressure->satpt path.\n");
  printf("  Note: drift is full-window regression slope (C/min); delta is\n");
  printf("        end-window mean minus begin-window mean (C).\n");
  printf("  Note: cal live supports --drift_c_per_min X (absolute value) to\n");
  printf("        send one ntfy when |drift| stays under threshold for the\n");
  printf("        configured calibration window duration.\n");
  printf(
    "  Note: cal livecal mirrors cal live arguments but prints calibrated\n");
  printf(
    "        values and refuses to start unless calibration is applied.\n");
  printf(
    "  Note: during cal live, cal capture attaches to the same session,\n");
  printf("        reuses buffered live data, and may capture immediately.\n");
  printf("  Note: 'cal stop' aborts an active cal live/cal livecal/cal capture "
         "operation.\n\n");
  printf("EXAMPLES\n");
  printf("  cal status\n");
  printf("  cal set method \"ice + satpt steam\"\n");
  printf("  cal metar set 1391 \"METAR KBJI 081955Z AUTO 26010KT 10SM CLR "
         "05/M02 A2969 RMK AO2 T00501022\"\n");
  printf("  cal metar show\n");
  printf("  cal capture 0.0 --stable_stddev_c 0.02 --min_seconds 8\n");
  printf("  cal import 99.970 0.0 --raw_c -0.078 --stddev_c 0.019\n");
  printf("  cal livecal --seconds 20 --every_ms 500\n");
  printf("  satpt A2922 1315\n");
  printf("  cal import 135.650 98.388 --raw_c 92.475 --stddev_c 0.036\n");
  printf("  cal import \"1: reference_temp_C=98.795 captured_raw_res_avg_Ohm="
         "138.132 ... drift_limit_source=DEFAULT ...\"\n");
  printf("  cal import 1: reference_temp_C=98.795 captured_raw_res_avg_Ohm="
         "138.132 ... drift_limit_source=DEFAULT ...\n");
  printf("  cal del 1\n");
  printf("  cal apply\n");
}

static int
CalTopicHelp(const char* topic)
{
  if (topic == NULL || topic[0] == '\0') {
    return 1;
  }

  for (size_t i = 0; kCalTopics[i].name != NULL; ++i) {
    if (strcmp(kCalTopics[i].name, topic) == 0 ||
        ((strcmp(topic, "restore") == 0) &&
         (strcmp(kCalTopics[i].name, "import") == 0)) ||
        ((strcmp(topic, "remove") == 0) &&
         (strcmp(kCalTopics[i].name, "del") == 0))) {
      ConsoleHelpPrintTopicManpage("cal", &kCalTopics[i]);
      return 0;
    }
  }

  return 1;
}

/**
 * @brief Print the mode command help body with available subtopics and
 * examples.
 */
static void
PrintModeHelpBody(void)
{
  ConsoleHelpPrintTopicList(kModeTopics);
  printf("Use: help mode <subcommand>\n\n");
  printf("EXAMPLES\n");
  printf("  mode show\n");
  printf("  mode run\n");
  printf("  mode set diag\n");
}

static void
PrintNetHelpBody(void)
{
  printf("SUBCOMMANDS\n");
  printf("  show                              Show configured and effective "
         "net mode\n");
  printf("  set mesh|wifi|none                Save configured net mode\n\n");
  printf("NOTES\n");
  printf("  Effective mode may be overridden by current role.\n");
  printf("  root role forces effective mode to mesh.\n\n");
  printf("EXAMPLES\n");
  printf("  net show\n");
  printf("  net set wifi\n");
  printf("  net set mesh\n");
}

/**
 * @brief Print detailed mode subtopic help for a specific subcommand.
 * @param topic Mode subtopic name.
 * @return 0 if the topic was found and printed, otherwise 1.
 */
static int
ModeTopicHelp(const char* topic)
{
  if (topic == NULL || topic[0] == '\0') {
    return 1;
  }

  for (size_t i = 0; kModeTopics[i].name != NULL; ++i) {
    if (strcmp(kModeTopics[i].name, topic) == 0) {
      ConsoleHelpPrintTopicManpage("mode", &kModeTopics[i]);
      return 0;
    }
  }

  return 1;
}

static void
PrintHelpCommandBody(void)
{
  printf("EXAMPLES\n");
  printf("  help\n");
  printf("  help wifi\n");
  printf("  help cal add\n");
}

/**
 * @brief Execute RegisterCommands.
 */
static void
RegisterCommands(void)
{
  static const console_registry_entry_t help_cmd = {
    .command = "help",
    .summary = "Show command list or detailed help",
    .synopsis = "help [command] [topic]",
    .description =
      "Show top-level command index, command help, or command subtopic help.",
    .print_body = PrintHelpCommandBody,
    .func = &ConsoleHelpCommand,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&help_cmd));

  static const console_registry_entry_t status_cmd = {
    .command = "status",
    .summary = "Show current settings and runtime state",
    .synopsis = "status",
    .func = &CommandStatus,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&status_cmd));

  static const console_registry_entry_t stack_cmd = {
    .command = "stack",
    .summary = "Show stack usage for monitored tasks",
    .synopsis = "stack [show] [--headroom BYTES]",
    .func = &CommandStack,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&stack_cmd));

  static const console_registry_entry_t disp_cmd = {
    .command = "disp",
    .summary = "Configure display units and attention mode",
    .synopsis =
      "disp show | disp units C|F | disp interval <ms> | disp attn ...",
    .func = &CommandDisplay,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&disp_cmd));

  static const console_registry_entry_t units_cmd = {
    .command = "units",
    .summary = "Set temporary display units override",
    .synopsis = "units set C|F",
    .func = &CommandUnits,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&units_cmd));

  static const console_registry_entry_t raw_cmd = {
    .command = "raw",
    .summary = "Print one raw reading and calibrated value",
    .synopsis = "raw",
    .func = &CommandRaw,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&raw_cmd));

  static const console_registry_entry_t flush_cmd = {
    .command = "flush",
    .summary = "Force FRAM to SD flush",
    .synopsis = "flush [--async] [--timeout <ms>]",
    .func = &CommandFlush,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&flush_cmd));

  static const console_registry_entry_t marker_cmd = {
    .command = "marker",
    .summary = "Inspect or clear startup marker",
    .synopsis = "marker show | marker clear",
    .func = &CommandMarker,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&marker_cmd));

  static const console_registry_entry_t locks_cmd = {
    .command = "locks",
    .summary = "Dump mutex and I2C lock diagnostics",
    .synopsis = "locks",
    .func = &CommandLocks,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&locks_cmd));

  static const console_registry_entry_t deferlog_cmd = {
    .command = "deferlog",
    .summary = "Show deferred logging status",
    .synopsis = "deferlog status",
    .func = &CommandDeferLog,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&deferlog_cmd));

  static const console_registry_entry_t sd_cmd = {
    .command = "sd",
    .summary = "Manage SD card and log files",
    .synopsis = "sd <status|mount|unmount|format|view|verify> [args]",
    .print_body = PrintSdHelpBody,
    .topic_help = &SdTopicHelp,
    .func = &CommandSd,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&sd_cmd));

  static const console_registry_entry_t fram_cmd = {
    .command = "fram",
    .summary = "Inspect and manage FRAM log buffer",
    .synopsis = "fram <status|show|clear>",
    .print_body = PrintFramHelpBody,
    .func = &CommandFram,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&fram_cmd));

  static const console_registry_entry_t errlog_cmd = {
    .command = "errlog",
    .summary = "Inspect and clear FRAM error log",
    .synopsis =
      "errlog show [--last N] | errlog stats | errlog status | errlog clear",
    .print_body = PrintErrlogHelpBody,
    .func = &CommandErrlog,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&errlog_cmd));

  static const console_registry_entry_t log_cmd = {
    .command = "log",
    .summary = "Tune runtime logging parameters",
    .synopsis = "log interval <ms> | log watermark <records> | log "
                "flush_period <ms> | log batch <bytes> | log show",
    .func = &CommandLog,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&log_cmd));

  static const console_registry_entry_t mqtt_cmd = {
    .command = "mqtt",
    .summary = "Configure MQTT publish and bridge behavior",
    .synopsis = "mqtt <show|enable|broker|prefix|qos|retain|bridge> ...",
    .print_body = PrintMqttHelpBody,
    .func = &CommandMqtt,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&mqtt_cmd));

  static const console_registry_entry_t rtd_cmd = {
    .command = "rtd",
    .summary = "Configure RTD filtering and display",
    .synopsis =
      "rtd show | rtd ema show | rtd ema on|off | rtd ema alpha <0.0..1.0>",
    .func = &CommandRtd,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&rtd_cmd));

  g_cal_args.action =
    arg_str1(NULL,
             NULL,
             "<action>",
             "status|cfg|set|clear|add|import|restore|del|remove|"
             "list|apply|live|capture|stop");
  g_cal_args.raw_c =
    arg_dbl0(NULL, NULL, "<raw_c>", "Raw Celsius sample (use with 'add')");
  g_cal_args.actual_c =
    arg_dbl0(NULL, NULL, "<C>", "Known actual temperature in Celsius");
  g_cal_args.every_ms =
    arg_int0(NULL, "every_ms", "<ms>", "Print interval for live mode");
  g_cal_args.seconds =
    arg_int0(NULL, "seconds", "<seconds>", "Duration for live mode");
  g_cal_args.stable_stddev_c =
    arg_dbl0(NULL,
             "stable_stddev_c",
             "<C>",
             "Stable stddev threshold in Celsius (capture)");
  g_cal_args.min_seconds =
    arg_int0(NULL, "min_seconds", "<seconds>", "Min stable duration (capture)");
  g_cal_args.timeout_seconds =
    arg_int0(NULL, "timeout_seconds", "<seconds>", "Capture timeout");
  g_cal_args.drift_c_per_min = arg_dbl0(
    NULL, "drift_c_per_min", "<C/min>", "Drift limit (capture/live notify)");
  g_cal_args.no_drift_limit =
    arg_lit0(NULL, "no-drift-limit", "Disable capture drift gating");
  g_cal_args.mode =
    arg_str0(NULL, "mode", "<linear|piecewise|polyN>", "Fit mode (apply)");
  g_cal_args.allow_wide_slope =
    arg_lit0(NULL, "allow_wide_slope", "Allow wider slope constraints (apply)");
  g_cal_args.end = arg_end(20);
  static const console_registry_entry_t cal_cmd = {
    .command = "cal",
    .summary = "Manage calibration metadata, live/import point capture, and "
               "model fitting",
    .synopsis =
      "cal "
      "<status|cfg|metar|set|list|add|import|restore|del|remove|clear|"
      "apply|live|capture|stop> ...",
    .print_body = PrintCalHelpBody,
    .topic_help = &CalTopicHelp,
    .func = &CommandCal,
    .argtable = (void**)&g_cal_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&cal_cmd));

  g_mode_args.action = arg_str1(NULL, NULL, "<action>", "show|run|diag|set");
  g_mode_args.mode_value =
    arg_str0(NULL, NULL, "<diag|run>", "Boot mode to store in NVS (set only)");
  g_mode_args.end = arg_end(2);
  static const console_registry_entry_t mode_cmd = {
    .command = "mode",
    .summary = "Show or set boot/runtime mode",
    .synopsis = "mode show | mode run | mode diag | mode set diag|run",
    .description =
      "mode run/diag applies runtime behavior immediately without NVS writes. "
      "mode set diag|run persists boot default in NVS (reboot required). "
      "mode show prints stored boot mode plus current runtime streaming state.",
    .print_body = PrintModeHelpBody,
    .topic_help = &ModeTopicHelp,
    .func = &CommandMode,
    .argtable = (void**)&g_mode_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&mode_cmd));

  g_data_args.action = arg_str1(NULL, NULL, "<action>", "show|on|off");
  g_data_args.end = arg_end(1);
  static const console_registry_entry_t data_cmd = {
    .command = "data",
    .summary = "Control data streaming mode",
    .synopsis = "data show | data on | data off",
    .func = &CommandData,
    .argtable = (void**)&g_data_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&data_cmd));

  g_run_args.action = arg_str1(NULL, NULL, "<action>", "status|start|stop");
  g_run_args.end = arg_end(1);
  static const console_registry_entry_t run_cmd = {
    .command = "run",
    .summary = "Start or stop periodic runtime loop",
    .synopsis = "run status | run start | run stop",
    .func = &CommandRun,
    .argtable = (void**)&g_run_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&run_cmd));

  g_tz_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_tz_args.posix = arg_str0(NULL, NULL, "<tz>", "POSIX TZ string");
  g_tz_args.end = arg_end(2);
  static const console_registry_entry_t tz_cmd = {
    .command = "tz",
    .summary = "Show or set timezone",
    .synopsis = "tz show | tz set <posix_tz>",
    .func = &CommandTz,
    .argtable = (void**)&g_tz_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&tz_cmd));

  g_time_args.action = arg_str1(NULL, NULL, "<action>", "show|setlocal");
  g_time_args.local_time =
    arg_str0(NULL, NULL, "<YYYY-MM-DD HH:MM:SS>", "Local time value");
  g_time_args.is_dst =
    arg_int0(NULL, "is_dst", "<0|1>", "Resolve ambiguous local time");
  g_time_args.end = arg_end(3);
  static const console_registry_entry_t time_cmd = {
    .command = "time",
    .summary = "Show, set, or sync system time",
    .synopsis =
      "time show | time setlocal <YYYY-MM-DD HH:MM:SS> [--is_dst 0|1]",
    .func = &CommandTime,
    .argtable = (void**)&g_time_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&time_cmd));

  static const console_registry_entry_t rtc_cmd = {
    .command = "rtc",
    .summary = "Inspect and control external RTC",
    .synopsis = "rtc <show|sync|set|temp|status>",
    .func = &CommandRtc,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&rtc_cmd));

  static const console_registry_entry_t satpt_cmd = {
    .command = "satpt",
    .summary = "Compute steam-point reference temperature from local pressure "
               "(for calibration)",
    .synopsis = "satpt <station_inHg> | satpt <A_inHg|A####> <elev_ft>",
    .description =
      "Calculate water saturation temperature at local pressure "
      "(boiling/steam point).\n"
      "Pressure input forms:\n"
      "  1) satpt <station_inHg>\n"
      "     Station pressure directly from a local barometer.\n"
      "  2) satpt <A_inHg|A####> <elev_ft>\n"
      "     METAR altimeter setting (A#### or inHg) + elevation in feet; "
      "converted to station pressure via ISA approximation.\n"
      "Steam-space method (preferred):\n"
      "  - Place probe in steam space above boiling water, not in liquid.\n"
      "  - Use a loose/vented cover (foil/lid with a hole) so steam surrounds "
      "the probe.\n"
      "  - Do not seal the beaker; overpressure raises saturation temperature "
      "and invalidates the calculation.\n"
      "  - Maintain a steady rolling boil long enough to equilibrate.\n"
      "Well-mixed boiling-liquid method (acceptable):\n"
      "  - Keep liquid well mixed (magnetic stirrer or circulation) to avoid "
      "gradients.\n"
      "  - Keep the sensing element near the surface for agreement with "
      "station-pressure saturation temperature.\n"
      "  - Warning: deeper probe submergence increases local pressure "
      "(hydrostatic head) and can raise measured boiling temperature above "
      "station-pressure value.\n"
      "Traceability disclaimer: estimate only (Antoine + ISA approximation). "
      "For traceable calibration, use a calibrated barometer at the bath and "
      "a proper fixed-point/steam-point apparatus.\n"
      "Use 'cal metar set <elevation_ft> \"<raw METAR>\"' when you need the "
      "same "
      "math path plus persisted calibration-report metadata.",
    .func = &CommandSatPt,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&satpt_cmd));

  g_dst_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_dst_args.enabled = arg_int0(NULL, NULL, "<0|1>", "DST enabled");
  g_dst_args.end = arg_end(2);
  static const console_registry_entry_t dst_cmd = {
    .command = "dst",
    .summary = "Show or set daylight saving behavior",
    .synopsis = "dst show | dst set 0|1",
    .func = &CommandDst,
    .argtable = (void**)&g_dst_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&dst_cmd));

  g_role_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_role_args.role = arg_str0(NULL, NULL, "<root|sensor|relay>", "Node role");
  g_role_args.end = arg_end(2);
  static const console_registry_entry_t role_cmd = {
    .command = "role",
    .summary = "Show or set node role",
    .synopsis = "role show | role set root|sensor|relay",
    .func = &CommandRole,
    .argtable = (void**)&g_role_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&role_cmd));

  g_net_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_net_args.mode =
    arg_str0(NULL, NULL, "<mesh|wifi|none>", "Network mode (mesh|wifi|none)");
  g_net_args.end = arg_end(2);
  static const console_registry_entry_t net_cmd = {
    .command = "net",
    .summary = "Show or set network mode",
    .synopsis = "net show | net set mesh|wifi|none",
    .description = "Show configured/effective network mode and save network "
                   "preference. Root role forces effective mesh mode.",
    .print_body = PrintNetHelpBody,
    .func = &CommandNet,
    .argtable = (void**)&g_net_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&net_cmd));

  static const console_registry_entry_t wifi_cmd = {
    .command = "wifi",
    .summary = "Manage Wi-Fi credentials and connectivity",
    .synopsis = "wifi <show|set|clear|scan|status|connect|disconnect|ntp> ...",
    .print_body = PrintWifiHelpBody,
    .func = &CommandWifi,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&wifi_cmd));

  g_children_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_children_args.enabled =
    arg_int0(NULL, NULL, "<0|1>", "Allow downstream children");
  g_children_args.end = arg_end(2);
  static const console_registry_entry_t children_cmd = {
    .command = "children",
    .summary = "Control whether downstream children are allowed",
    .synopsis = "children show | children set 0|1",
    .func = &CommandChildren,
    .argtable = (void**)&g_children_args,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&children_cmd));

  static const console_registry_entry_t diag_cmd = {
    .command = "diag",
    .summary = "Run hardware and service diagnostics",
    .synopsis = "diag <help|sd|wifi|mesh|rtc|rtd|fram|storage> ...",
    .print_body = PrintDiagHelpBody,
    .func = &CommandDiagnostics,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&diag_cmd));

  static const console_registry_entry_t reboot_cmd = {
    .command = "reboot",
    .summary = "Soft reboot the device",
    .synopsis = "reboot",
    .func = &CommandReboot,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&reboot_cmd));

  ConsoleAlertsRegister(g_runtime);
}

/**
 * @brief Execute ConsoleTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the ConsoleTask task.
 */
static void
ConsoleTask(void* context)
{
  (void)context;

  printf("\nType 'help' to list commands.\n");
  while (true) {
#if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    if (RuntimeIsDataStreamingEnabled()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
#endif
    char* line = ConsoleReadLine_("pt100> ");
    if (line == NULL) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (strlen(line) > 0) {
      if (ConsoleInputTooLongOrUnterminatedQuote_(line,
                                                  kConsoleMaxCmdlineLength)) {
        printf("input too long or unterminated quoted string\n");
        continue;
      }
      if (!ConsoleCmdlineSnapshotCapture_(line, kConsoleMaxCmdlineLength)) {
        ESP_LOGW(kTag, "console snapshot capture failed; using stale snapshot");
      }
      int run_result = 0;
      esp_err_t result = esp_console_run(line, &run_result);
      if (result == ESP_ERR_NOT_FOUND) {
        printf("Unrecognized command\n");
      } else if (result == ESP_ERR_INVALID_ARG) {
        // Command already printed errors.
      } else if (result != ESP_OK) {
        printf("Command failed: %s\n", esp_err_to_name(result));
      } else if (run_result != 0) {
        printf("Command returned non-zero: %d\n", run_result);
      }
    }
  }
}

static bool
ConsoleCmdlineBufferInit_(size_t max_cmdline_length)
{
  if (s_console_cmdline_buffer != NULL && s_console_cmdline_snapshot != NULL) {
    return true;
  }

  if (s_console_cmdline_buffer == NULL) {
    s_console_cmdline_buffer = (char*)heap_caps_malloc(
      max_cmdline_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_console_cmdline_buffer_in_psram =
      (s_console_cmdline_buffer != NULL &&
       esp_ptr_external_ram(s_console_cmdline_buffer));
  }
  if (s_console_cmdline_buffer == NULL) {
    s_console_cmdline_buffer =
      (char*)heap_caps_malloc(max_cmdline_length, MALLOC_CAP_8BIT);
    s_console_cmdline_buffer_in_psram = false;
  }
  if (s_console_cmdline_buffer == NULL) {
    ESP_LOGE(kTag,
             "console buffer allocation failed (%u bytes)",
             (unsigned)max_cmdline_length);
    return false;
  }

  if (s_console_cmdline_snapshot == NULL) {
    if (s_console_cmdline_buffer_in_psram) {
      s_console_cmdline_snapshot = (char*)heap_caps_malloc(
        max_cmdline_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      s_console_cmdline_snapshot_in_psram =
        (s_console_cmdline_snapshot != NULL &&
         esp_ptr_external_ram(s_console_cmdline_snapshot));
      if (s_console_cmdline_snapshot == NULL) {
        s_console_cmdline_snapshot =
          (char*)heap_caps_malloc(max_cmdline_length, MALLOC_CAP_8BIT);
        s_console_cmdline_snapshot_in_psram = false;
      }
    } else {
      s_console_cmdline_snapshot =
        (char*)heap_caps_malloc(max_cmdline_length, MALLOC_CAP_8BIT);
      s_console_cmdline_snapshot_in_psram = false;
      if (s_console_cmdline_snapshot == NULL) {
        s_console_cmdline_snapshot = (char*)heap_caps_malloc(
          max_cmdline_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_console_cmdline_snapshot_in_psram =
          (s_console_cmdline_snapshot != NULL &&
           esp_ptr_external_ram(s_console_cmdline_snapshot));
      }
    }
  }
  if (s_console_cmdline_snapshot == NULL) {
    ESP_LOGE(kTag,
             "console snapshot allocation failed (%u bytes)",
             (unsigned)max_cmdline_length);
    return false;
  }

  memset(s_console_cmdline_buffer, 0, max_cmdline_length);
  memset(s_console_cmdline_snapshot, 0, max_cmdline_length);
  ESP_LOGI(kTag,
           "console buffer: size=%u location=%s",
           (unsigned)max_cmdline_length,
           s_console_cmdline_buffer_in_psram ? "PSRAM" : "INTERNAL");
  ESP_LOGI(kTag,
           "console snapshot buffer: size=%u location=%s",
           (unsigned)max_cmdline_length,
           s_console_cmdline_snapshot_in_psram ? "PSRAM" : "INTERNAL");
  return true;
}

static bool
ConsoleCmdlineSnapshotCapture_(const char* line, size_t max_cmdline_length)
{
  if (line == NULL || s_console_cmdline_snapshot == NULL ||
      max_cmdline_length == 0u) {
    return false;
  }

  const size_t line_len = strlen(line);
  const size_t copy_len = (line_len < (max_cmdline_length - 1u))
                            ? line_len
                            : (max_cmdline_length - 1u);
  memcpy(s_console_cmdline_snapshot, line, copy_len);
  s_console_cmdline_snapshot[copy_len] = '\0';
  return (copy_len == line_len);
}

static char*
ConsoleReadLine_(const char* prompt)
{
  if (s_console_cmdline_buffer == NULL) {
    return NULL;
  }
  if (prompt != NULL) {
    printf("%s", prompt);
    fflush(stdout);
  }

  size_t write_index = 0u;
  while (true) {
    int c = getchar();
    if (c == EOF) {
      return NULL;
    }
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      break;
    }
    if ((c == '\b' || c == 0x7F) && write_index > 0u) {
      --write_index;
      continue;
    }
    if (write_index < (kConsoleMaxCmdlineLength - 1u)) {
      s_console_cmdline_buffer[write_index++] = (char)c;
    }
  }
  s_console_cmdline_buffer[write_index] = '\0';
  return s_console_cmdline_buffer;
}

static bool
ConsoleInputTooLongOrUnterminatedQuote_(const char* line,
                                        size_t max_cmdline_length)
{
  if (line == NULL || max_cmdline_length < 2u) {
    return false;
  }

  const size_t line_length = strlen(line);
  const bool maybe_truncated = (line_length >= (max_cmdline_length - 1u));
  bool in_quotes = false;
  bool escaped = false;
  for (size_t index = 0; index < line_length; ++index) {
    const char current = line[index];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (in_quotes && current == '\\') {
      escaped = true;
      continue;
    }
    if (current == '"') {
      in_quotes = !in_quotes;
    }
  }
  return maybe_truncated || in_quotes;
}

/**
 * @brief Execute ConsoleCommandsStart.
 * @param runtime Parameter runtime.
 * @param boot_mode Parameter boot_mode.
 * @return Return the function result.
 */
esp_err_t
ConsoleCommandsStart(app_runtime_t* runtime, app_boot_mode_t boot_mode)
{
  if (runtime == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  g_runtime = runtime;
  g_boot_mode = boot_mode;
  if (runtime->settings != NULL) {
    CalWindowSetDurationSeconds(runtime->settings->cal_window_duration_s);
    CalWindowSetTrendEmaAlphaPermille(
      runtime->settings->cal_trend_ema_alpha_permille);
  }
  if (g_cal_console_op_mutex == NULL) {
    g_cal_console_op_mutex = xSemaphoreCreateMutex();
    if (g_cal_console_op_mutex == NULL) {
      return ESP_ERR_NO_MEM;
    }
  }

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG

  // USB Serial/JTAG console (native USB port)
  // Data CSV is streamed on UART0 (USB-to-UART bridge) to keep CSV parse-clean.
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

  // Make stdin/stdout blocking (line input is read from stdin).
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  // The esp_console max_cmdline_length bound alone is not enough for long
  // pasted commands. If the transport RX FIFO/ring is smaller than the line,
  // input can be truncated before ConsoleReadLine_() or snapshot capture.
  usb_serial_jtag_driver_config_t usb_cfg = {
    .tx_buffer_size = kConsoleTransportTxBufferSize,
    .rx_buffer_size = kConsoleTransportRxBufferSize,
  };
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
  usb_serial_jtag_vfs_use_driver();
  setvbuf(stdin, NULL, _IONBF, 0);

#else

  // UART console (USB-to-UART bridge port)
  const int uart_num = CONFIG_ESP_CONSOLE_UART_NUM;
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  // Keep UART transport buffers comfortably larger than the command line
  // buffer so long "cal import <status-line>" pastes are delivered intact.
  ESP_ERROR_CHECK(uart_driver_install(uart_num,
                                      kConsoleTransportRxBufferSize,
                                      kConsoleTransportTxBufferSize,
                                      0,
                                      NULL,
                                      0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  uart_vfs_dev_use_driver(uart_num);

#endif

  esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
  console_config.max_cmdline_length = (uint32_t)kConsoleMaxCmdlineLength;
  // Keep headroom for long unquoted "cal import <status-line>" payloads.
  // Even though import parsing now prefers raw command-tail scanning, a
  // generous argv limit avoids accidental truncation in other command paths.
  console_config.max_cmdline_args = 48;
  if (!ConsoleCmdlineBufferInit_(kConsoleMaxCmdlineLength)) {
    return ESP_ERR_NO_MEM;
  }
  ESP_ERROR_CHECK(esp_console_init(&console_config));

  ConsoleHelpInit();

  RegisterCommands();

  xTaskCreate(ConsoleTask, "console", 12288, NULL, 2, NULL);
  return ESP_OK;
}
