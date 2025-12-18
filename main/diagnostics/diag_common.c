#include "diagnostics/diag_common.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void
DiagInitCtx(diag_ctx_t* ctx, const char* name, bool verbose)
{
  if (ctx == NULL) {
    return;
  }
  ctx->name = name;
  ctx->steps_failed = 0;
  ctx->steps_run = 0;
  ctx->verbose = verbose;
}

void
DiagReportStep(diag_ctx_t* ctx,
               int step_index,
               int total_steps,
               const char* step,
               esp_err_t result,
               const char* details_fmt,
               ...)
{
  if (ctx == NULL) {
    return;
  }
  ctx->steps_run++;
  if (result != ESP_OK) {
    ctx->steps_failed++;
  }

  const char* status = (result == ESP_OK) ? "PASS" : "FAIL";
  printf("[%s] STEP %d/%d: %s .... %s", ctx->name, step_index, total_steps, step, status);
  if (result != ESP_OK) {
    printf(" (%s)", esp_err_to_name(result));
  }
  printf("\n");

  if (details_fmt != NULL && ctx->verbose) {
    va_list args;
    va_start(args, details_fmt);
    printf("      ");
    vprintf(details_fmt, args);
    printf("\n");
    va_end(args);
  }
}

void
DiagPrintSummary(const diag_ctx_t* ctx, int total_steps)
{
  if (ctx == NULL) {
    return;
  }
  const bool pass = ctx->steps_failed == 0 && ctx->steps_run == total_steps;
  printf("[%s] SUMMARY: %d/%d steps completed, %d failed => %s\n",
         ctx->name,
         ctx->steps_run,
         total_steps,
         ctx->steps_failed,
         pass ? "PASS" : "FAIL");
}

void
DiagHexdump(const char* label, const uint8_t* bytes, size_t len)
{
  if (bytes == NULL || len == 0) {
    return;
  }
  printf("%s (%u bytes):", label, (unsigned)len);
  for (size_t i = 0; i < len; ++i) {
    if (i % 16 == 0) {
      printf("\n      ");
    }
    printf("%02x ", bytes[i]);
  }
  printf("\n");
}

void
DiagPrintErr(esp_err_t err)
{
  printf("%s", esp_err_to_name(err));
}
