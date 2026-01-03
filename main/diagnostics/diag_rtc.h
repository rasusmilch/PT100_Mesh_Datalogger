#ifndef PT100_LOGGER_DIAGNOSTICS_RTC_H_
#define PT100_LOGGER_DIAGNOSTICS_RTC_H_

#include <stdbool.h>

#include "diagnostics/diag_common.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute RunDiagRtc.
 * @param runtime Parameter runtime.
 * @param full Parameter full.
 * @param set_known Parameter set_known.
 * @param verbosity Parameter verbosity.
 * @return Return the function result.
 */
int RunDiagRtc(const app_runtime_t* runtime,
               bool full,
               bool set_known,
               diag_verbosity_t verbosity);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_RTC_H_
