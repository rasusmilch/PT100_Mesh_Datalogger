#include "diagnostics/diag_fram.h"

#include "esp_err.h"

int
RunDiagFram(const app_runtime_t* runtime,
            bool full,
            int bytes,
            diag_verbosity_t verbosity)
{
  (void)runtime;
  (void)bytes;
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "FRAM", verbosity);
  const int total_steps = full ? 2 : 1;
  DiagReportStep(&ctx,
                 1,
                 total_steps,
                 "i2c probe",
                 ESP_FAIL,
                 "FRAM diagnostics not implemented");
  if (full) {
    DiagReportStep(&ctx,
                   2,
                   total_steps,
                   "rw test",
                   ESP_FAIL,
                   "No FRAM access available");
  }
  DiagPrintSummary(&ctx, total_steps);
  return 1;
}
