#include "diagnostics/diag_rtc.h"

#include "esp_err.h"

int
RunDiagRtc(const app_runtime_t* runtime,
           bool full,
           bool set_known,
           diag_verbosity_t verbosity)
{
  (void)runtime;
  (void)set_known;
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "RTC", verbosity);
  const int total_steps = full ? 2 : 1;
  DiagReportStep(&ctx,
                 1,
                 total_steps,
                 "bus probe",
                 ESP_FAIL,
                 "RTC probe not implemented");
  if (full) {
    DiagReportStep(&ctx,
                   2,
                   total_steps,
                   "time read",
                   ESP_FAIL,
                   "RTC access not implemented");
  }
  DiagPrintSummary(&ctx, total_steps);
  return 1;
}
