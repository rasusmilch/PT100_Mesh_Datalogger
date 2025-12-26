#include "console_commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "argtable3/argtable3.h"
#include "boot_mode.h"
#include "calibration.h"
#include "diagnostics/diag_fram.h"
#include "diagnostics/diag_mesh.h"
#include "diagnostics/diag_rtc.h"
#include "diagnostics/diag_rtd.h"
#include "diagnostics/diag_sd.h"
#include "diagnostics/diag_wifi.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_console.h"
#include "esp_timer.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include <fcntl.h>
#include <unistd.h>
#endif

#include "esp_log.h"
#include "esp_system.h"
#include "linenoise/linenoise.h"
#include "runtime_manager.h"
#include "time_sync.h"

static const char* kTag = "console";

static app_runtime_t* g_runtime = NULL;
static app_boot_mode_t g_boot_mode = APP_BOOT_MODE_DIAGNOSTICS;

static const char*
BootModeToString(app_boot_mode_t mode)
{
  return (mode == APP_BOOT_MODE_RUN) ? "run" : "diagnostics";
}

static int
CommandStatus(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  if (g_runtime == NULL) {
    return 1;
  }

  const app_settings_t* settings = g_runtime->settings;

  printf("node_id: %s\n", g_runtime->node_id_string);
  printf("runtime_running: %s\n", RuntimeIsRunning() ? "yes" : "no");
  printf("time_valid: %s\n", TimeSyncIsSystemTimeValid() ? "yes" : "no");
  printf("log_period_ms: %u\n", (unsigned)settings->log_period_ms);
  printf("sd_flush_period_ms: %u\n", (unsigned)settings->sd_flush_period_ms);
  printf("sd_batch_target_bytes: %u\n",
         (unsigned)settings->sd_batch_bytes_target);
  printf("node_role: %s\n", AppSettingsRoleToString(settings->node_role));
  printf("allow_children: %s\n", settings->allow_children ? "yes" : "no");
  printf("tz_posix: %s\n", settings->tz_posix);
  printf("dst_enabled: %s\n", settings->dst_enabled ? "yes" : "no");

// Ensure the TZ rules are loaded before formatting local time.
// (TZ is applied via AppSettingsApplyTimeZone() at boot and by the tz/dst commands.)
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
  strftime(local_buffer, sizeof(local_buffer), "%Y-%m-%d %H:%M:%S", &local_time);
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
  printf("fram: buffered=%u / cap=%u (flush_watermark=%u)\n",
         (unsigned)FramLogGetBufferedRecords(g_runtime->fram_log),
         (unsigned)FramLogGetCapacityRecords(g_runtime->fram_log),
         (unsigned)settings->fram_flush_watermark_records);
  const bool fram_full =
    (g_runtime->fram_full != NULL) ? *g_runtime->fram_full : false;
  printf("fram_full: %s\n", fram_full ? "yes" : "no");
  printf("fram_count/seq: %u/%u\n",
         (unsigned)FramLogGetBufferedRecords(g_runtime->fram_log),
         (unsigned)FramLogNextSequence(g_runtime->fram_log));
  const uint32_t export_dropped = (g_runtime->export_dropped_count != NULL)
                                    ? *g_runtime->export_dropped_count
                                    : 0u;
  const uint32_t export_write_fail =
    (g_runtime->export_write_fail_count != NULL)
      ? *g_runtime->export_write_fail_count
      : 0u;
  printf("export_dropped_count: %u\n", (unsigned)export_dropped);
  printf("export_write_fail_count: %u\n", (unsigned)export_write_fail);

  printf("calibration: degree=%u coeffs=[%.9g, %.9g, %.9g, %.9g]\n",
         (unsigned)settings->calibration.degree,
         settings->calibration.coefficients[0],
         settings->calibration.coefficients[1],
         settings->calibration.coefficients[2],
         settings->calibration.coefficients[3]);

  printf("sd_mounted: %s\n", (g_runtime->sd_logger->is_mounted ? "yes" : "no"));
  printf("sd_last_seq: %u\n",
         (unsigned)SdLoggerLastSequenceOnSd(g_runtime->sd_logger));
  printf("mesh_connected: %s\n",
         MeshTransportIsConnected(g_runtime->mesh) ? "yes" : "no");
  printf("cal_points: %u\n",
         (unsigned)g_runtime->settings->calibration_points_count);
  return 0;
}

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

  const double calibrated = CalibrationModelEvaluate(
    &g_runtime->settings->calibration, sample.temperature_c);
  char fault[64] = { 0 };
  Max31865FormatFault(sample.fault_status, fault, sizeof(fault));

  printf("adc_code_15: %u\n", (unsigned)sample.adc_code);
  printf("resistance_ohm: %.3f\n", sample.resistance_ohm);
  printf("temp_raw_c: %.3f\n", sample.temperature_c);
  printf("temp_cal_c: %.3f\n", calibrated);
  printf("fault: %s (0x%02x)\n", fault, (unsigned)sample.fault_status);
  return 0;
}

static esp_err_t
FlushAllRecordsToSd(app_runtime_t* runtime)
{
  if (runtime->flush_callback == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return runtime->flush_callback(runtime->flush_context);
}

static int
CommandFlush(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  if (g_runtime == NULL) {
    return 1;
  }

  esp_err_t result = FlushAllRecordsToSd(g_runtime);
  if (result != ESP_OK) {
    printf("flush failed: %s\n", esp_err_to_name(result));
    return 1;
  }
  return 0;
}

static int
CommandFram(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: fram status\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "status") != 0 && strcmp(action, "show") != 0) {
    printf("unknown fram command. try 'fram status'\n");
    return 1;
  }

  fram_log_status_t status;
  esp_err_t result = FramLogGetStatus(g_runtime->fram_log, &status);
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
  printf("fram: write=%u read=%u count=%u seq=%u\n",
         (unsigned)status.write_index_abs,
         (unsigned)status.read_index_abs,
         (unsigned)status.buffered_count,
         (unsigned)status.next_sequence);
  printf("FRAM log: cap=%u rec write=%u read=%u count=%u seq=%u\n",
         (unsigned)status.capacity_records,
         (unsigned)status.write_index_abs,
         (unsigned)status.read_index_abs,
         (unsigned)status.buffered_count,
         (unsigned)status.next_sequence);
  return 0;
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
  struct arg_int* enabled;
  struct arg_end* end;
} g_children_args;

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
  struct arg_end* end;
} g_cal_args;

static int
CommandCal(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_cal_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_cal_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  app_settings_t* settings = g_runtime->settings;
  const char* action = g_cal_args.action->sval[0];

  if (strcmp(action, "clear") == 0) {
    CalibrationModelInitIdentity(&settings->calibration);
    settings->calibration_points_count = 0;
    memset(settings->calibration_points,
           0,
           sizeof(settings->calibration_points));
    esp_err_t result = AppSettingsSaveCalibration(&settings->calibration);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    result = AppSettingsSaveCalibrationPoints(
      settings->calibration_points, settings->calibration_points_count);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("calibration reset to identity (y=x)\n");
    return 0;
  }

  if (strcmp(action, "add") == 0) {
    if (g_cal_args.raw_c->count != 1 || g_cal_args.actual_c->count != 1) {
      printf("usage: cal add <raw_c> <actual_c>\n");
      return 1;
    }
    if (settings->calibration_points_count >= CALIBRATION_MAX_POINTS) {
      printf("already have %u points; run 'cal apply' or 'cal clear'\n",
             (unsigned)settings->calibration_points_count);
      return 1;
    }

    calibration_point_t* point =
      &settings->calibration_points[settings->calibration_points_count];
    point->raw_avg_mC = (int32_t)llround(g_cal_args.raw_c->dval[0] * 1000.0);
    point->actual_mC =
      (int32_t)llround(g_cal_args.actual_c->dval[0] * 1000.0);
    point->raw_stddev_mC = 0;
    point->sample_count = 0;
    point->time_valid = TimeSyncIsSystemTimeValid() ? 1u : 0u;
    point->timestamp_epoch_sec = point->time_valid ? (int64_t)time(NULL) : 0;
    settings->calibration_points_count++;
    printf("added point %u: raw=%.6f actual=%.6f\n",
           (unsigned)settings->calibration_points_count,
           g_cal_args.raw_c->dval[0],
           g_cal_args.actual_c->dval[0]);
    esp_err_t result = AppSettingsSaveCalibrationPoints(
      settings->calibration_points, settings->calibration_points_count);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    return 0;
  }

  if (strcmp(action, "list") == 0) {
    printf("calibration points (%u):\n",
           (unsigned)settings->calibration_points_count);
    for (size_t index = 0; index < settings->calibration_points_count;
         ++index) {
      const calibration_point_t* point = &settings->calibration_points[index];
      printf("  %u: raw_avg=%.6f actual=%.6f stddev=%.6f samples=%u\n",
             (unsigned)(index + 1),
             point->raw_avg_mC / 1000.0,
             point->actual_mC / 1000.0,
             point->raw_stddev_mC / 1000.0,
             (unsigned)point->sample_count);
    }
    return 0;
  }

  if (strcmp(action, "show") == 0) {
    int32_t last_raw_mC = 0;
    int32_t mean_raw_mC = 0;
    int32_t stddev_mC = 0;
    CalWindowGetStats(&last_raw_mC, &mean_raw_mC, &stddev_mC);
    const size_t sample_count = CalWindowGetSampleCount();
    printf("cal_window_raw_last_c: %.3f\n", last_raw_mC / 1000.0);
    printf("cal_window_raw_avg_c: %.3f\n", mean_raw_mC / 1000.0);
    printf("cal_window_raw_stddev_c: %.3f\n", stddev_mC / 1000.0);
    printf("cal_window_samples: %u\n", (unsigned)sample_count);
    printf("cal_window_ready: %s\n",
           CalWindowIsReady() ? "yes" : "no");
    printf("cal_points: %u (raw_avg_C uses window average)\n",
           (unsigned)settings->calibration_points_count);
    for (size_t index = 0; index < settings->calibration_points_count;
         ++index) {
      const calibration_point_t* point = &settings->calibration_points[index];
      const double raw_avg_c = point->raw_avg_mC / 1000.0;
      const double actual_c = point->actual_mC / 1000.0;
      const double residual_c = actual_c - raw_avg_c;
      printf("  %u: raw_avg_C=%.3f actual_C=%.3f residual_C=%.3f stddev_C=%.3f\n",
             (unsigned)(index + 1),
             raw_avg_c,
             actual_c,
             residual_c,
             point->raw_stddev_mC / 1000.0);
    }
    return 0;
  }

  if (strcmp(action, "apply") == 0) {
    if (settings->calibration_points_count < 1) {
      printf("no points; use 'cal add <raw_c> <actual_c>' first\n");
      return 1;
    }

    calibration_model_t model;
    esp_err_t result = CalibrationModelFitFromPoints(
      settings->calibration_points, settings->calibration_points_count, &model);
    if (result != ESP_OK) {
      printf("fit failed: %s\n", esp_err_to_name(result));
      return 1;
    }

    settings->calibration = model;
    result = AppSettingsSaveCalibration(&model);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }

    printf("calibration applied: degree=%u coeffs=[%.9g, %.9g, %.9g, %.9g]\n",
           (unsigned)model.degree,
           model.coefficients[0],
           model.coefficients[1],
           model.coefficients[2],
           model.coefficients[3]);

    return 0;
  }

  if (strcmp(action, "live") == 0) {
    const int every_ms =
      (g_cal_args.every_ms->count > 0) ? g_cal_args.every_ms->ival[0] : 500;
    const int seconds =
      (g_cal_args.seconds->count > 0) ? g_cal_args.seconds->ival[0] : 10;
    if (every_ms <= 0 || seconds <= 0) {
      printf("usage: cal live [--every_ms 500] [--seconds 10]\n");
      return 1;
    }

    const int64_t duration_us = (int64_t)seconds * 1000000LL;
    const int64_t start_us = esp_timer_get_time();
    while (esp_timer_get_time() - start_us < duration_us) {
      int32_t last_raw_mC = 0;
      int32_t mean_raw_mC = 0;
      int32_t stddev_mC = 0;
      CalWindowGetStats(&last_raw_mC, &mean_raw_mC, &stddev_mC);
      printf("raw_last_C=%.3f raw_avg_C=%.3f raw_stddev_C=%.3f\n",
             last_raw_mC / 1000.0,
             mean_raw_mC / 1000.0,
             stddev_mC / 1000.0);
      vTaskDelay(pdMS_TO_TICKS((uint32_t)every_ms));
    }
    return 0;
  }

  if (strcmp(action, "capture") == 0) {
    if (g_cal_args.actual_c->count != 1) {
      printf("usage: cal capture <actual_temp_c> "
             "[--stable_stddev_c 0.05] [--min_seconds 5] "
             "[--timeout_seconds 120]\n");
      return 1;
    }
    if (settings->calibration_points_count >= CALIBRATION_MAX_POINTS) {
      printf("already have %u points; run 'cal apply' or 'cal clear'\n",
             (unsigned)settings->calibration_points_count);
      return 1;
    }

    const double actual_temp_c = g_cal_args.actual_c->dval[0];
    const double stable_stddev_c = (g_cal_args.stable_stddev_c->count > 0)
                                     ? g_cal_args.stable_stddev_c->dval[0]
                                     : 0.05;
    const int min_seconds = (g_cal_args.min_seconds->count > 0)
                              ? g_cal_args.min_seconds->ival[0]
                              : 5;
    const int timeout_seconds = (g_cal_args.timeout_seconds->count > 0)
                                  ? g_cal_args.timeout_seconds->ival[0]
                                  : 120;
    if (stable_stddev_c <= 0.0 || min_seconds <= 0 || timeout_seconds <= 0) {
      printf("usage: cal capture <actual_temp_c> "
             "[--stable_stddev_c 0.05] [--min_seconds 5] "
             "[--timeout_seconds 120]\n");
      return 1;
    }

    const int64_t start_us = esp_timer_get_time();
    int64_t stable_start_us = -1;
    while (esp_timer_get_time() - start_us <
           (int64_t)timeout_seconds * 1000000LL) {
      int32_t last_raw_mC = 0;
      int32_t mean_raw_mC = 0;
      int32_t stddev_mC = 0;
      CalWindowGetStats(&last_raw_mC, &mean_raw_mC, &stddev_mC);
      const double stddev_c = stddev_mC / 1000.0;

      if (CalWindowIsReady() && stddev_c <= stable_stddev_c) {
        if (stable_start_us < 0) {
          stable_start_us = esp_timer_get_time();
        }
        if (esp_timer_get_time() - stable_start_us >=
            (int64_t)min_seconds * 1000000LL) {
          calibration_point_t* point =
            &settings->calibration_points[settings->calibration_points_count];
          point->raw_avg_mC = mean_raw_mC;
          point->actual_mC = (int32_t)llround(actual_temp_c * 1000.0);
          point->raw_stddev_mC = stddev_mC;
          point->sample_count = (uint16_t)CalWindowGetSampleCount();
          point->time_valid = TimeSyncIsSystemTimeValid() ? 1u : 0u;
          point->timestamp_epoch_sec =
            point->time_valid ? (int64_t)time(NULL) : 0;
          settings->calibration_points_count++;
          printf("cal capture ok: raw_avg=%.3fC raw_std=%.3fC actual=%.3fC\n",
                 mean_raw_mC / 1000.0,
                 stddev_c,
                 actual_temp_c);
          esp_err_t save_result = AppSettingsSaveCalibrationPoints(
            settings->calibration_points, settings->calibration_points_count);
          if (save_result != ESP_OK) {
            printf("save failed: %s\n", esp_err_to_name(save_result));
            return 1;
          }
          return 0;
        }
      } else {
        stable_start_us = -1;
      }

      vTaskDelay(pdMS_TO_TICKS(200));
    }

    printf("cal capture failed: timed out after %d seconds\n",
           timeout_seconds);
    return 1;
  }

  printf("unknown action. usage: cal clear | cal add <raw_c> <actual_c> | cal "
         "list | cal show | cal apply | cal live [--every_ms 500] [--seconds "
         "10] | cal capture <actual_temp_c> [--stable_stddev_c 0.05] "
         "[--min_seconds 5] [--timeout_seconds 120]\n");
  return 1;
}

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
    printf("runtime stopped\n");
    return 0;
  }

  printf("unknown action. usage: run status | run start | run stop\n");
  return 1;
}

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
PrintTimeUsage(void)
{
  printf("time setlocal \"YYYY-MM-DD HH:MM:SS\" [--is_dst 0|1]\n");
  printf("  input is LOCAL wall time; converted to UTC epoch + RTC stored as UTC\n");
  printf("  use --is_dst to disambiguate fall-back hour\n");
}

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
      ESP_LOGW(kTag,
               "mesh time broadcast failed: %s",
               esp_err_to_name(mesh_result));
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

static void
PrintDiagUsage(void)
{
  printf("diag help\n");
  printf("diag all quick|full [--verbose N]\n");
  printf("diag sd quick|full [--format-if-needed] [--mount] [--verbose N]\n");
  printf("diag fram quick|full [--bytes N] [--verbose N]\n");
  printf("diag rtd quick|full [--samples N] [--delay_ms M] [--verbose N]\n");
  printf("diag rtc quick|full [--set-known] [--verbose N]\n");
  printf("diag wifi quick|full [--scan 0|1] [--connect 0|1] [--dns 0|1] "
         "[--keep-connected 0|1] [--verbose N]\n");
  printf("diag mesh quick|full [--start] [--stop] [--root] [--timeout_ms T] "
         "[--verbose N]\n"
         "  note: if you use --start without --stop, the mesh stays running\n");
}

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

  const bool target_requires_mode =
    strcmp(target, "all") == 0 || strcmp(target, "sd") == 0 ||
    strcmp(target, "fram") == 0 || strcmp(target, "rtc") == 0 ||
    strcmp(target, "rtd") == 0 || strcmp(target, "wifi") == 0 ||
    strcmp(target, "mesh") == 0;
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

static int
CommandReboot(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  printf("rebooting...\n");
  esp_restart();
  return 0;
}

static void
RegisterCommands(void)
{
  const esp_console_cmd_t status_cmd = {
    .command = "status",
    .help = "Show current settings and runtime state",
    .hint = NULL,
    .func = &CommandStatus,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&status_cmd));

  const esp_console_cmd_t raw_cmd = {
    .command = "raw",
    .help = "Print one raw reading and calibrated value",
    .hint = NULL,
    .func = &CommandRaw,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&raw_cmd));

  const esp_console_cmd_t flush_cmd = {
    .command = "flush",
    .help = "Force flush FRAM -> SD (best-effort)",
    .hint = NULL,
    .func = &CommandFlush,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&flush_cmd));

  const esp_console_cmd_t fram_cmd = {
    .command = "fram",
    .help = "FRAM log commands: fram status | fram show",
    .hint = NULL,
    .func = &CommandFram,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&fram_cmd));

  const esp_console_cmd_t log_cmd = {
    .command = "log",
    .help = "Logging config: log interval <ms> | log watermark <records> | log "
            "flush_period <ms> | log batch <bytes> | log show",
    .hint = NULL,
    .func = &CommandLog,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&log_cmd));

  g_cal_args.action =
    arg_str1(NULL, NULL, "<action>", "clear|add|list|show|apply|live|capture");
  g_cal_args.raw_c =
    arg_dbl0(NULL, NULL, "<raw_c>", "Raw temperature (C) from 'raw'");
  g_cal_args.actual_c =
    arg_dbl0(NULL, NULL, "<actual_c>", "Actual temperature (C)");
  g_cal_args.every_ms =
    arg_int0(NULL, "every_ms", "<every_ms>", "Live interval (ms)");
  g_cal_args.seconds =
    arg_int0(NULL, "seconds", "<seconds>", "Live duration (s)");
  g_cal_args.stable_stddev_c =
    arg_dbl0(NULL,
             "stable_stddev_c",
             "<stable_stddev_c>",
             "Capture stable stddev threshold (C)");
  g_cal_args.min_seconds =
    arg_int0(NULL, "min_seconds", "<min_seconds>", "Capture min stable time");
  g_cal_args.timeout_seconds = arg_int0(NULL,
                                        "timeout_seconds",
                                        "<timeout_seconds>",
                                        "Capture timeout");
  g_cal_args.end = arg_end(8);

  const esp_console_cmd_t cal_cmd = {
    .command = "cal",
    .help =
      "Calibration: cal clear | cal add <raw_c> <actual_c> | cal list | cal "
      "show | cal apply | cal live [--every_ms 500] [--seconds 10] | cal "
      "capture <actual_temp_c> [--stable_stddev_c 0.05] [--min_seconds 5] "
      "[--timeout_seconds 120]",
    .hint = NULL,
    .func = &CommandCal,
    .argtable = &g_cal_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&cal_cmd));

  g_mode_args.action = arg_str1(NULL, NULL, "<action>", "show|run|diag|set");
  g_mode_args.mode_value = arg_str0(NULL, NULL, "diag|run", "Target mode");
  g_mode_args.end = arg_end(2);
  const esp_console_cmd_t mode_cmd = {
    .command = "mode",
    .help = "mode show | mode run | mode diag | mode set diag|run",
    .hint = NULL,
    .func = &CommandMode,
    .argtable = &g_mode_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&mode_cmd));

  g_data_args.action = arg_str1(NULL, NULL, "<action>", "show|on|off");
  g_data_args.end = arg_end(1);
  const esp_console_cmd_t data_cmd = {
    .command = "data",
    .help = "data show | data on | data off",
    .hint = NULL,
    .func = &CommandData,
    .argtable = &g_data_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&data_cmd));

  g_run_args.action = arg_str1(NULL, NULL, "<action>", "status|start|stop");
  g_run_args.end = arg_end(1);
  const esp_console_cmd_t run_cmd = {
    .command = "run",
    .help = "run status | run start | run stop",
    .hint = NULL,
    .func = &CommandRun,
    .argtable = &g_run_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&run_cmd));

  g_tz_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_tz_args.posix = arg_str0(NULL, NULL, "<posix>", "POSIX TZ string");
  g_tz_args.end = arg_end(2);
  const esp_console_cmd_t tz_cmd = {
    .command = "tz",
    .help = "tz show | tz set \"<posix>\"",
    .hint = NULL,
    .func = &CommandTz,
    .argtable = &g_tz_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&tz_cmd));

  g_time_args.action = arg_str1(NULL, NULL, "<action>", "setlocal");
  g_time_args.local_time = arg_str0(NULL, NULL, "<local_time>", "LOCAL time");
  g_time_args.is_dst = arg_int0(NULL, "is_dst", "<0|1>", "DST disambiguation");
  g_time_args.end = arg_end(3);
  const esp_console_cmd_t time_cmd = {
    .command = "time",
    .help =
      "time setlocal \"YYYY-MM-DD HH:MM:SS\" [--is_dst 0|1]\n"
      "  input is LOCAL wall time; converted to UTC epoch + RTC stored as UTC\n"
      "  use --is_dst to disambiguate fall-back hour",
    .hint = NULL,
    .func = &CommandTime,
    .argtable = &g_time_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&time_cmd));

  g_dst_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_dst_args.enabled = arg_int0(NULL, NULL, "<0|1>", "DST enabled");
  g_dst_args.end = arg_end(2);
  const esp_console_cmd_t dst_cmd = {
    .command = "dst",
    .help = "dst show | dst set 0|1",
    .hint = NULL,
    .func = &CommandDst,
    .argtable = &g_dst_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&dst_cmd));

  g_role_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_role_args.role = arg_str0(NULL, NULL, "<root|sensor|relay>", "Node role");
  g_role_args.end = arg_end(2);
  const esp_console_cmd_t role_cmd = {
    .command = "role",
    .help = "role show | role set root|sensor|relay",
    .hint = NULL,
    .func = &CommandRole,
    .argtable = &g_role_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&role_cmd));

  g_children_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_children_args.enabled =
    arg_int0(NULL, NULL, "<0|1>", "Allow downstream children");
  g_children_args.end = arg_end(2);
  const esp_console_cmd_t children_cmd = {
    .command = "children",
    .help = "children show | children set 0|1",
    .hint = NULL,
    .func = &CommandChildren,
    .argtable = &g_children_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&children_cmd));

  const esp_console_cmd_t diag_cmd = {
    .command = "diag",
    .help = "Diagnostics entry point",
    .hint = NULL,
    .func = &CommandDiagnostics,
    .argtable = NULL,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&diag_cmd));

  const esp_console_cmd_t reboot_cmd = {
    .command = "reboot",
    .help = "Soft reboot the device",
    .hint = NULL,
    .func = &CommandReboot,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&reboot_cmd));
}

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
    char* line = linenoise("pt100> ");
    if (line == NULL) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (strlen(line) > 0) {
      linenoiseHistoryAdd(line);
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
    linenoiseFree(line);
  }
}

esp_err_t
ConsoleCommandsStart(app_runtime_t* runtime, app_boot_mode_t boot_mode)
{
  if (runtime == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  g_runtime = runtime;
  g_boot_mode = boot_mode;

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG

  // USB Serial/JTAG console (native USB port)
  // Data CSV is streamed on UART0 (USB-to-UART bridge) to keep CSV parse-clean.
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

  // Make stdin/stdout blocking (helps linenoise)
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  usb_serial_jtag_driver_config_t usb_cfg = {
    .tx_buffer_size = 256,
    .rx_buffer_size = 256,
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

  ESP_ERROR_CHECK(uart_driver_install(uart_num, 256, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  uart_vfs_dev_use_driver(uart_num);

#endif

  esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
  console_config.max_cmdline_length = 256;
  console_config.max_cmdline_args = 8;
  ESP_ERROR_CHECK(esp_console_init(&console_config));

  // Register built-in help so users can discover available commands.
  ESP_ERROR_CHECK(esp_console_register_help_command());

  linenoiseSetDumbMode(1);
  linenoiseHistorySetMaxLen(50);

  RegisterCommands();

  xTaskCreate(ConsoleTask, "console", 12288, NULL, 2, NULL);
  return ESP_OK;
}
