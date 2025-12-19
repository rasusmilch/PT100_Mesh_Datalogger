#include "console_commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "boot_mode.h"
#include "diagnostics/diag_fram.h"
#include "diagnostics/diag_mesh.h"
#include "diagnostics/diag_rtc.h"
#include "diagnostics/diag_rtd.h"
#include "diagnostics/diag_sd.h"
#include "diagnostics/diag_wifi.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "runtime_manager.h"

static const char* kTag = "console";

static app_runtime_t* g_runtime = NULL;
static app_boot_mode_t g_boot_mode = APP_BOOT_MODE_DIAGNOSTICS;

static calibration_point_t g_pending_points[CALIBRATION_MAX_POINTS];
static size_t g_pending_points_count = 0;

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
  printf("fram: buffered=%u / cap=%u (flush_watermark=%u)\n",
         (unsigned)FramLogGetBufferedRecords(g_runtime->fram_log),
         (unsigned)FramLogGetCapacityRecords(g_runtime->fram_log),
         (unsigned)settings->fram_flush_watermark_records);
  const bool fram_full =
    (g_runtime->fram_full != NULL) ? *g_runtime->fram_full : false;
  printf("fram_full: %s\n", fram_full ? "yes" : "no");

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
  printf("pending_cal_points: %u\n", (unsigned)g_pending_points_count);
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

  float raw_c = 0;
  float resistance_ohm = 0;
  esp_err_t result =
    Max31865ReaderRead(g_runtime->sensor, &raw_c, &resistance_ohm);
  if (result != ESP_OK) {
    printf("read failed: %s\n", esp_err_to_name(result));
    return 1;
  }

  const double calibrated =
    CalibrationModelEvaluate(&g_runtime->settings->calibration, raw_c);

  printf("raw_c: %.3f\n", raw_c);
  printf("resistance_ohm: %.3f\n", resistance_ohm);
  printf("cal_c: %.3f\n", calibrated);
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

static struct
{
  struct arg_str* action;
  struct arg_int* interval_ms;
  struct arg_int* watermark_records;
  struct arg_int* flush_period_ms;
  struct arg_int* batch_bytes;
  struct arg_end* end;
} g_log_args;

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
} g_run_args;

static int
CommandLog(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_log_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_log_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_log_args.action->sval[0];
  if (strcmp(action, "interval") == 0) {
    if (g_log_args.interval_ms->count != 1) {
      printf("usage: log interval <ms>\n");
      return 1;
    }
    const int interval_ms = g_log_args.interval_ms->ival[0];
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
    if (g_log_args.watermark_records->count != 1) {
      printf("usage: log watermark <records>\n");
      return 1;
    }
    const int watermark = g_log_args.watermark_records->ival[0];
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
    if (g_log_args.flush_period_ms->count != 1) {
      printf("usage: log flush_period <ms>\n");
      return 1;
    }
    const int period_ms = g_log_args.flush_period_ms->ival[0];
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
    if (g_log_args.batch_bytes->count != 1) {
      printf("usage: log batch <bytes>\n");
      return 1;
    }
    const int batch_bytes = g_log_args.batch_bytes->ival[0];
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

  const char* action = g_cal_args.action->sval[0];

  if (strcmp(action, "clear") == 0) {
    CalibrationModelInitIdentity(&g_runtime->settings->calibration);
    esp_err_t result =
      AppSettingsSaveCalibration(&g_runtime->settings->calibration);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    g_pending_points_count = 0;
    printf("calibration reset to identity (y=x)\n");
    return 0;
  }

  if (strcmp(action, "add") == 0) {
    if (g_cal_args.raw_c->count != 1 || g_cal_args.actual_c->count != 1) {
      printf("usage: cal add <raw_c> <actual_c>\n");
      return 1;
    }
    if (g_pending_points_count >= CALIBRATION_MAX_POINTS) {
      printf("already have %u points; run 'cal apply' or 'cal clear'\n",
             (unsigned)g_pending_points_count);
      return 1;
    }

    g_pending_points[g_pending_points_count].raw_c = g_cal_args.raw_c->dval[0];
    g_pending_points[g_pending_points_count].actual_c =
      g_cal_args.actual_c->dval[0];
    g_pending_points_count++;
    printf("added point %u: raw=%.6f actual=%.6f\n",
           (unsigned)g_pending_points_count,
           g_cal_args.raw_c->dval[0],
           g_cal_args.actual_c->dval[0]);
    return 0;
  }

  if (strcmp(action, "list") == 0) {
    printf("pending calibration points (%u):\n",
           (unsigned)g_pending_points_count);
    for (size_t index = 0; index < g_pending_points_count; ++index) {
      printf("  %u: raw=%.6f actual=%.6f\n",
             (unsigned)(index + 1),
             g_pending_points[index].raw_c,
             g_pending_points[index].actual_c);
    }
    return 0;
  }

  if (strcmp(action, "apply") == 0) {
    if (g_pending_points_count < 1) {
      printf("no points; use 'cal add <raw_c> <actual_c>' first\n");
      return 1;
    }

    calibration_model_t model;
    esp_err_t result = CalibrationModelFitFromPoints(
      g_pending_points, g_pending_points_count, &model);
    if (result != ESP_OK) {
      printf("fit failed: %s\n", esp_err_to_name(result));
      return 1;
    }

    g_runtime->settings->calibration = model;
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

    g_pending_points_count = 0;
    return 0;
  }

  printf("unknown action. usage: cal clear | cal add <raw_c> <actual_c> | cal "
         "list | cal apply\n");
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
    const app_boot_mode_t running =
      RuntimeIsRunning() ? APP_BOOT_MODE_RUN : APP_BOOT_MODE_DIAGNOSTICS;
    printf("nvs_boot_mode: %s\n", BootModeToString(stored));
    printf("current_mode: %s\n", BootModeToString(running));
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

  printf("unknown action. usage: mode show | mode set diag|run\n");
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
    esp_err_t result = RuntimeStart();
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
    esp_err_t result = RuntimeStop();
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

static void
PrintDiagUsage(void)
{
  printf("diag help\n");
  printf("diag all quick|full [--verbose N]\n");
  printf("diag sd quick|full [--format-if-needed] [--mount] [--verbose N]\n");
  printf("diag fram quick|full [--bytes N] [--verbose N]\n");
  printf("diag rtd quick|full [--samples N] [--verbose N]\n");
  printf("diag rtc quick|full [--set-known] [--verbose N]\n");
  printf("diag wifi quick|full [--scan] [--connect] [--verbose N]\n");
  printf("diag mesh quick|full [--start] [--stop] [--verbose N]\n");
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
  bool set_known = false;
  int bytes = 0;
  int samples = 0;
  bool start_mesh = false;
  bool stop_mesh = false;

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
      scan = true;
    } else if (strcmp(argv[i], "--connect") == 0) {
      connect = true;
    } else if (strcmp(argv[i], "--set-known") == 0) {
      set_known = true;
    } else if (strcmp(argv[i], "--bytes") == 0 && (i + 1) < argc) {
      bytes = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--samples") == 0 && (i + 1) < argc) {
      samples = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--start") == 0) {
      start_mesh = true;
    } else if (strcmp(argv[i], "--stop") == 0) {
      stop_mesh = true;
    } else {
      printf("unknown option: %s\n", argv[i]);
      PrintDiagUsage();
      return 2;
    }
  }

  const bool full = strcmp(mode, "full") == 0;
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
      overall |= RunDiagRtd(runtime, full, samples, diag_verbosity);
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
    overall |= RunDiagWifi(runtime, full, scan, connect, diag_verbosity);
    if (strcmp(target, "wifi") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "mesh") == 0 || strcmp(target, "all") == 0) {
    overall |=
      RunDiagMesh(runtime, full, start_mesh, stop_mesh, diag_verbosity);
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

  g_log_args.action = arg_str1(
    NULL, NULL, "<action>", "interval|watermark|flush_period|batch|show");
  g_log_args.interval_ms =
    arg_int0(NULL, NULL, "<ms>", "Logging interval in ms (for 'interval')");
  g_log_args.watermark_records =
    arg_int0(NULL, NULL, "<records>", "FRAM flush watermark (for 'watermark')");
  g_log_args.flush_period_ms =
    arg_int0(NULL, NULL, "<ms>", "Periodic SD flush ms (for 'flush_period')");
  g_log_args.batch_bytes =
    arg_int0(NULL, NULL, "<bytes>", "Batch target bytes (for 'batch')");
  g_log_args.end = arg_end(5);
  const esp_console_cmd_t log_cmd = {
    .command = "log",
    .help = "Logging config: log interval <ms> | log watermark <records> | log "
            "flush_period <ms> | log batch <bytes> | log show",
    .hint = NULL,
    .func = &CommandLog,
    .argtable = &g_log_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&log_cmd));

  g_cal_args.action = arg_str1(NULL, NULL, "<action>", "clear|add|list|apply");
  g_cal_args.raw_c =
    arg_dbl0(NULL, NULL, "<raw_c>", "Raw temperature (C) from 'raw'");
  g_cal_args.actual_c =
    arg_dbl0(NULL, NULL, "<actual_c>", "Actual temperature (C)");
  g_cal_args.end = arg_end(3);

  const esp_console_cmd_t cal_cmd = {
    .command = "cal",
    .help = "Calibration: cal clear | cal add <raw_c> <actual_c> | cal list | "
            "cal apply",
    .hint = NULL,
    .func = &CommandCal,
    .argtable = &g_cal_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&cal_cmd));

  g_mode_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_mode_args.mode_value = arg_str0(NULL, NULL, "diag|run", "Target mode");
  g_mode_args.end = arg_end(2);
  const esp_console_cmd_t mode_cmd = {
    .command = "mode",
    .help = "mode show | mode set diag|run",
    .hint = NULL,
    .func = &CommandMode,
    .argtable = &g_mode_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&mode_cmd));

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
  // Use default pins for UART0; for other UARTs, set pins via uart_set_pin().

  esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

  esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
  console_config.max_cmdline_length = 256;
  console_config.max_cmdline_args = 8;
  ESP_ERROR_CHECK(esp_console_init(&console_config));

  // Register built-in help so users can discover available commands.
  ESP_ERROR_CHECK(esp_console_register_help_command());

  linenoiseSetDumbMode(1);
  linenoiseHistorySetMaxLen(50);

  RegisterCommands();

  xTaskCreate(ConsoleTask, "console", 4096, NULL, 2, NULL);
  return ESP_OK;
}
