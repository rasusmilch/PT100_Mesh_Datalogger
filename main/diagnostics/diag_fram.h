#ifndef PT100_LOGGER_DIAGNOSTICS_FRAM_H_
#define PT100_LOGGER_DIAGNOSTICS_FRAM_H_

#include <stdbool.h>

#include "diagnostics/diag_common.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

int RunDiagFram(const app_runtime_t* runtime,
                bool full,
                int bytes,
                diag_verbosity_t verbosity);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_FRAM_H_
