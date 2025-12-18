#ifndef PT100_LOGGER_DIAGNOSTICS_SD_H_
#define PT100_LOGGER_DIAGNOSTICS_SD_H_

#include <stdbool.h>

#include "diagnostics/diag_common.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

int RunDiagSd(const app_runtime_t* runtime,
              bool full,
              bool format_if_needed,
              bool mount,
              bool verbose);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_SD_H_
