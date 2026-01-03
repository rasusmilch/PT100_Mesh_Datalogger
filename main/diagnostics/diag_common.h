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

/**
 * @brief Execute DiagInitCtx.
 * @param ctx Parameter ctx.
 * @param name Parameter name.
 * @param verbosity Parameter verbosity.
 */
void DiagInitCtx(diag_ctx_t* ctx, const char* name, diag_verbosity_t verbosity);

/**
 * @brief Execute DiagReportStep.
 * @param ctx Parameter ctx.
 * @param step_index Parameter step_index.
 * @param total_steps Parameter total_steps.
 * @param step Parameter step.
 * @param result Parameter result.
 * @param details_fmt Parameter details_fmt.
 * @param ... Parameter ....
 */
void DiagReportStep(diag_ctx_t* ctx,
                    int step_index,
                    int total_steps,
                    const char* step,
                    esp_err_t result,
                    const char* details_fmt,
                    ...);

/**
 * @brief Execute DiagPrintSummary.
 * @param ctx Parameter ctx.
 * @param total_steps Parameter total_steps.
 */
void DiagPrintSummary(const diag_ctx_t* ctx, int total_steps);

/**
 * @brief Execute DiagHexdump.
 * @param ctx Parameter ctx.
 * @param label Parameter label.
 * @param bytes Parameter bytes.
 * @param len Parameter len.
 */
void DiagHexdump(const diag_ctx_t* ctx,
                 const char* label,
                 const uint8_t* bytes,
                 size_t len);

/**
 * @brief Execute DiagPrintErr.
 * @param err Parameter err.
 */
void DiagPrintErr(esp_err_t err);

/**
 * @brief Execute DiagPrintErrno.
 * @param prefix Parameter prefix.
 */
void DiagPrintErrno(const char* prefix);

// Optional heap integrity check used during diagnostics when verbosity is
// high. The check is intentionally gated to avoid expensive scans during
// normal runs.
/**
 * @brief Execute DiagHeapCheck.
 * @param ctx Parameter ctx.
 * @param label Parameter label.
 */
void DiagHeapCheck(const diag_ctx_t* ctx, const char* label);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_COMMON_H_
