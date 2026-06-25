#include "diagnostics/diag_sd.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "sdmmc_cmd.h"

#include "sd_logger.h"

/**
 * @brief Report the last daily PTLOG preparation failure as informational data.
 *
 * Historical daily-open failures help the next hardware run diagnose nested
 * path/directory/open issues, but they are not a current SD health verdict;
 * this step always reports ESP_OK when the logger is readable.
 */
static void
DiagReportLastDailyPtlog_(diag_ctx_t* ctx,
                          const sd_logger_t* logger,
                          int step,
                          int total_steps)
{
  SdLoggerDailyDiagnostics daily_diag;
  if (!SdLoggerGetLastDailyDiagnostics(logger, &daily_diag)) {
    DiagReportStep(ctx,
                   step,
                   total_steps,
                   "daily ptlog",
                   ESP_OK,
                   "last_daily_stage=none");
    return;
  }

  DiagReportStep(ctx,
                 step,
                 total_steps,
                 "daily ptlog",
                 ESP_OK,
                 "last_daily_stage=%s result=%s errno=%d path=%s date=%s "
                 "month=%s rev=%" PRIu32 " existed=%s empty=%s "
                 "cleanup_attempted=%s cleanup_succeeded=%s",
                 SdLoggerDailyStageName(daily_diag.stage),
                 esp_err_to_name(daily_diag.result),
                 daily_diag.errno_value,
                 daily_diag.path[0] != '\0' ? daily_diag.path : "-",
                 daily_diag.date[0] != '\0' ? daily_diag.date : "-",
                 daily_diag.month[0] != '\0' ? daily_diag.month : "-",
                 daily_diag.revision,
                 daily_diag.file_existed_before_open ? "yes" : "no",
                 daily_diag.file_was_empty ? "yes" : "no",
                 daily_diag.empty_file_unlink_attempted ? "yes" : "no",
                 daily_diag.empty_file_unlink_succeeded ? "yes" : "no");
}

/**
 * @brief Execute RunDiagSdImpl.
 * @param runtime Parameter runtime.
 * @param full Parameter full.
 * @param format_if_needed Parameter format_if_needed.
 * @param mount Parameter mount.
 * @param verbosity Parameter verbosity.
 * @return Return the function result.
 */
static int
RunDiagSdImpl(const app_runtime_t* runtime,
              bool full,
              bool format_if_needed,
              bool mount,
              diag_verbosity_t verbosity)
{

  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "SD", verbosity);
  const int total_steps = full ? 5 : 3;
  int result = 1;

  DiagHeapCheck(&ctx, "pre_sd_diag");

  if (runtime == NULL || runtime->sd_logger == NULL) {
    DiagReportStep(&ctx, 1, total_steps, "runtime available", ESP_ERR_INVALID_STATE, "runtime not initialized");
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  runtime_state_t* state = RuntimeGetState();
  bool locked = false;
  if (state != NULL) {
    locked = RuntimeSdIoLock(state, pdMS_TO_TICKS(2000));
    if (!locked) {
      DiagReportStep(&ctx,
                     1,
                     total_steps,
                     "sd lock",
                     ESP_ERR_TIMEOUT,
                     "sd_io_lock_timeout");
      DiagPrintSummary(&ctx, total_steps);
      return 1;
    }
  }

  const bool should_mount = mount || format_if_needed;
  esp_err_t mount_result = ESP_OK;
  if (should_mount && !runtime->sd_logger->is_mounted) {
    mount_result = SdLoggerTryRemount(runtime->sd_logger, format_if_needed);
  }
  const char* mount_err_string =
    should_mount ? esp_err_to_name(mount_result) : "n/a";

  DiagReportStep(&ctx,
                 1,
                 total_steps,
                 "logger mounted",
                 runtime->sd_logger->is_mounted ? ESP_OK : ESP_FAIL,
                 "mounted=%s attempted=%s format=%s err=%s",
                 runtime->sd_logger->is_mounted ? "yes" : "no",
                 should_mount ? "yes" : "no",
                 format_if_needed ? "yes" : "no",
                 mount_err_string);

  DiagReportLastDailyPtlog_(&ctx, runtime->sd_logger, 2, total_steps);

  if (!full) {
    DiagReportStep(&ctx,
                   3,
                   total_steps,
                   "last id",
                   ESP_OK,
                   "last_record_id=%" PRIu64,
                   SdLoggerLastRecordIdOnSd(runtime->sd_logger));
  }

  if (full) {
    const sdmmc_card_t* card = runtime->sd_logger->card;
    if (card != NULL) {
      DiagReportStep(&ctx, 3, total_steps, "card info", ESP_OK, "name=%s oem=%u size=%lluMB", card->cid.name, (unsigned)card->cid.oem_id, (unsigned long long)((uint64_t)card->csd.capacity * card->csd.sector_size / (1024 * 1024)));
    } else {
      DiagReportStep(&ctx, 3, total_steps, "card info", ESP_FAIL, "card structure missing");
    }

    if (runtime->sd_logger->is_mounted) {
      const char* mount_point = runtime->sd_logger->mount_point;
      char test_path[128];
      snprintf(test_path, sizeof(test_path), "%s/diag_sd_test.bin", mount_point);
      FILE* f = fopen(test_path, "wb");
      if (f != NULL) {
        const char payload[] = "diag";
        fwrite(payload, 1, sizeof(payload), f);
        fflush(f);
        fclose(f);
        FILE* r = fopen(test_path, "rb");
        bool match = false;
        if (r != NULL) {
          char buffer[8] = { 0 };
          size_t n = fread(buffer, 1, sizeof(buffer), r);
          fclose(r);
          match =
            (n >= sizeof(payload) && memcmp(payload, buffer, sizeof(payload)) == 0);
        }
        remove(test_path);
        DiagReportStep(&ctx,
                       4,
                       total_steps,
                       "file r/w",
                       match ? ESP_OK : ESP_FAIL,
                       "mount=%s path=%s",
                       mount_point,
                       test_path);
      } else {
        const esp_err_t err = ESP_FAIL;
        DiagReportStep(&ctx,
                       4,
                       total_steps,
                       "file r/w",
                       err,
                       "mount=%s path=%s errno=%d (%s)",
                       mount_point,
                       test_path,
                       errno,
                       strerror(errno));
      }
    } else {
      DiagReportStep(&ctx, 4, total_steps, "file r/w", ESP_FAIL, "not mounted");
    }

    DiagReportStep(&ctx,
                   5,
                   total_steps,
                   "last id",
                   ESP_OK,
                   "last_record_id=%" PRIu64,
                   SdLoggerLastRecordIdOnSd(runtime->sd_logger));
  }

  DiagHeapCheck(&ctx, "post_sd_diag");
  DiagPrintSummary(&ctx, total_steps);
  result = (ctx.steps_failed == 0) ? 0 : 1;

  if (locked) {
    RuntimeSdIoUnlock(state);
  }
  return result;
}

typedef struct
{
  bool full;
  bool format_if_needed;
  bool mount;
  diag_verbosity_t verbosity;
  int result;
} diag_sd_ctx_t;

/**
 * @brief Execute RunDiagSdWithMount.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
RunDiagSdWithMount(app_runtime_t* runtime, void* ctx)
{
  diag_sd_ctx_t* args = (diag_sd_ctx_t*)ctx;
  args->result = RunDiagSdImpl(runtime,
                               args->full,
                               args->format_if_needed,
                               args->mount,
                               args->verbosity);
  return (args->result == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Execute RunDiagSd.
 * @param runtime Parameter runtime.
 * @param full Parameter full.
 * @param format_if_needed Parameter format_if_needed.
 * @param mount Parameter mount.
 * @param verbosity Parameter verbosity.
 * @return Return the function result.
 */
int
RunDiagSd(const app_runtime_t* runtime,
          bool full,
          bool format_if_needed,
          bool mount,
          diag_verbosity_t verbosity)
{
  if (!RuntimeIsRunning()) {
    diag_sd_ctx_t ctx = {
      .full = full,
      .format_if_needed = format_if_needed,
      .mount = mount,
      .verbosity = verbosity,
      .result = 1,
    };
    esp_err_t result = RuntimeWithTemporarySdMount(&RunDiagSdWithMount, &ctx);
    if (result != ESP_OK) {
      return 1;
    }
    return ctx.result;
  }

  return RunDiagSdImpl(runtime, full, format_if_needed, mount, verbosity);
}
