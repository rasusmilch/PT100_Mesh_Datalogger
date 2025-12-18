#ifndef PT100_LOGGER_DIAGNOSTICS_COMMON_H_
#define PT100_LOGGER_DIAGNOSTICS_COMMON_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  const char* name;
  int steps_run;
  int steps_failed;
  bool verbose;
} diag_ctx_t;

typedef struct {
  const char* step;
  esp_err_t result;
} diag_step_result_t;

void DiagInitCtx(diag_ctx_t* ctx, const char* name, bool verbose);

void DiagReportStep(diag_ctx_t* ctx,
                    int step_index,
                    int total_steps,
                    const char* step,
                    esp_err_t result,
                    const char* details_fmt,
                    ...);

void DiagPrintSummary(const diag_ctx_t* ctx, int total_steps);

void DiagHexdump(const char* label, const uint8_t* bytes, size_t len);

void DiagPrintErr(esp_err_t err);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_COMMON_H_
