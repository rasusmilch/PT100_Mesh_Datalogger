#include "diagnostics/diag_rtd.h"

#include "esp_err.h"

int
RunDiagRtd(const app_runtime_t* runtime,
           bool full,
           int samples,
           diag_verbosity_t verbosity)
{
  (void)runtime;
  (void)samples;
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "RTD", verbosity);
  const int total_steps = full ? 2 : 1;
  DiagReportStep(&ctx,
                 1,
                 total_steps,
                 "spi probe",
                 ESP_FAIL,
                 "RTD diagnostics not implemented");
  if (full) {
    DiagReportStep(&ctx,
                   2,
                   total_steps,
                   "sample",
                   ESP_FAIL,
                   "No RTD access available");
  }
  DiagPrintSummary(&ctx, total_steps);
  return 1;
}
