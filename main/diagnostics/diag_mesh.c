#include "diagnostics/diag_mesh.h"

#include "esp_err.h"

int
RunDiagMesh(const app_runtime_t* runtime, bool full, bool start, bool stop, bool verbose)
{
  (void)runtime;
  (void)full;
  (void)start;
  (void)stop;
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "Mesh", verbose);
  const int total_steps = 1;
  DiagReportStep(&ctx, 1, total_steps, "mesh diag", ESP_ERR_NOT_SUPPORTED, "Mesh diagnostics not implemented");
  DiagPrintSummary(&ctx, total_steps);
  return 1;
}
