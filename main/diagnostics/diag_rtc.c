#include "diagnostics/diag_rtc.h"

#include "esp_err.h"

int
RunDiagRtc(const app_runtime_t* runtime, bool full, bool set_known, bool verbose)
{
  (void)runtime;
  (void)set_known;
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "RTC", verbose);
  const int total_steps = full ? 2 : 1;
  DiagReportStep(&ctx, 1, total_steps, "bus probe", ESP_ERR_NOT_SUPPORTED, "RTC probe not implemented");
  if (full) {
    DiagReportStep(&ctx, 2, total_steps, "time read", ESP_ERR_NOT_SUPPORTED, "RTC access not implemented");
  }
  DiagPrintSummary(&ctx, total_steps);
  return 1;
}
