#ifndef PT100_LOGGER_DIAGNOSTICS_RTD_H_
#define PT100_LOGGER_DIAGNOSTICS_RTD_H_

#include <stdbool.h>

#include "diagnostics/diag_common.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

int RunDiagRtd(const app_runtime_t* runtime, bool full, int samples, bool verbose);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_RTD_H_
