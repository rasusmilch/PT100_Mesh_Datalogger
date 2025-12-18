#include "diagnostics/diag_wifi.h"

#include "esp_err.h"

int
RunDiagWifi(const app_runtime_t* runtime,
            bool full,
            bool scan,
            bool connect,
            diag_verbosity_t verbosity)
{
  (void)runtime;
  (void)full;
  (void)scan;
  (void)connect;
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "WiFi", verbosity);
  const int total_steps = 1;
  DiagReportStep(&ctx,
                 1,
                 total_steps,
                 "wifi diag",
                 ESP_FAIL,
                 "Wi-Fi diagnostics not implemented");
  DiagPrintSummary(&ctx, total_steps);
  return 1;
}
