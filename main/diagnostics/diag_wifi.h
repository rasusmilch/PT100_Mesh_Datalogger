#ifndef PT100_LOGGER_DIAGNOSTICS_WIFI_H_
#define PT100_LOGGER_DIAGNOSTICS_WIFI_H_

#include <stdbool.h>

#include "diagnostics/diag_common.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

int RunDiagWifi(const app_runtime_t* runtime, bool full, bool scan, bool connect, bool verbose);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_WIFI_H_
