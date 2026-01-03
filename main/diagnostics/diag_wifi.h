#ifndef PT100_LOGGER_DIAGNOSTICS_WIFI_H_
#define PT100_LOGGER_DIAGNOSTICS_WIFI_H_

#include <stdbool.h>

#include "diagnostics/diag_common.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute RunDiagWifi.
 * @param runtime Parameter runtime.
 * @param full Parameter full.
 * @param scan Parameter scan.
 * @param connect Parameter connect.
 * @param dns_lookup Parameter dns_lookup.
 * @param keep_connected Parameter keep_connected.
 * @param verbosity Parameter verbosity.
 * @return Return the function result.
 */
int RunDiagWifi(const app_runtime_t* runtime,
                bool full,
                bool scan,
                bool connect,
                bool dns_lookup,
                bool keep_connected,
                diag_verbosity_t verbosity);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_WIFI_H_
