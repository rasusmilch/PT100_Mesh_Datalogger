#ifndef PT100_LOGGER_DIAGNOSTICS_COMMON_H_
#define PT100_LOGGER_DIAGNOSTICS_COMMON_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kDiagVerbosity0 = 0,
  kDiagVerbosity1 = 1,
  kDiagVerbosity2 = 2,
} diag_verbosity_t;

typedef struct {
  const char* name;
  int steps_run;
  int steps_failed;
  diag_verbosity_t verbosity;
} diag_ctx_t;

typedef struct {
  const char* step;
  esp_err_t result;
} diag_step_result_t;

void DiagInitCtx(diag_ctx_t* ctx, const char* name, diag_verbosity_t verbosity);

void DiagReportStep(diag_ctx_t* ctx,
                    int step_index,
                    int total_steps,
                    const char* step,
                    esp_err_t result,
                    const char* details_fmt,
                    ...);

void DiagPrintSummary(const diag_ctx_t* ctx, int total_steps);

void DiagHexdump(const diag_ctx_t* ctx,
                 const char* label,
                 const uint8_t* bytes,
                 size_t len);

void DiagPrintErr(esp_err_t err);

void DiagPrintErrno(const char* prefix);

// Optional heap integrity check used during diagnostics when verbosity is
// high. The check is intentionally gated to avoid expensive scans during
// normal runs.
void DiagHeapCheck(const diag_ctx_t* ctx, const char* label);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_COMMON_H_
