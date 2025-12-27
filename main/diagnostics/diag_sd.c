#include "diagnostics/diag_sd.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "sdmmc_cmd.h"

#include "sd_logger.h"

int
RunDiagSd(const app_runtime_t* runtime,
          bool full,
          bool format_if_needed,
          bool mount,
          diag_verbosity_t verbosity)
{

  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "SD", verbosity);
  const int total_steps = full ? 4 : 2;

  DiagHeapCheck(&ctx, "pre_sd_diag");

  if (runtime == NULL || runtime->sd_logger == NULL) {
    DiagReportStep(&ctx, 1, total_steps, "runtime available", ESP_ERR_INVALID_STATE, "runtime not initialized");
    DiagPrintSummary(&ctx, total_steps);
    return 1;
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

  if (!full) {
    DiagReportStep(&ctx,
                   2,
                   total_steps,
                   "last seq",
                   ESP_OK,
                   "last_sequence=%u",
                   (unsigned)SdLoggerLastSequenceOnSd(runtime->sd_logger));
  }

  if (full) {
    const sdmmc_card_t* card = runtime->sd_logger->card;
    if (card != NULL) {
      DiagReportStep(&ctx, 2, total_steps, "card info", ESP_OK, "name=%s oem=%u size=%lluMB", card->cid.name, (unsigned)card->cid.oem_id, (unsigned long long)((uint64_t)card->csd.capacity * card->csd.sector_size / (1024 * 1024)));
    } else {
      DiagReportStep(&ctx, 2, total_steps, "card info", ESP_FAIL, "card structure missing");
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
                       3,
                       total_steps,
                       "file r/w",
                       match ? ESP_OK : ESP_FAIL,
                       "mount=%s path=%s",
                       mount_point,
                       test_path);
      } else {
        const esp_err_t err = ESP_FAIL;
        DiagReportStep(&ctx,
                       3,
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
      DiagReportStep(&ctx, 3, total_steps, "file r/w", ESP_FAIL, "not mounted");
    }

    DiagReportStep(&ctx, 4, total_steps, "last seq", ESP_OK, "last_sequence=%u", (unsigned)SdLoggerLastSequenceOnSd(runtime->sd_logger));
  }

  DiagHeapCheck(&ctx, "post_sd_diag");
  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
