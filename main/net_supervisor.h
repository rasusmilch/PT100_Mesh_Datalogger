#ifndef PT100_LOGGER_NET_SUPERVISOR_H_
#define PT100_LOGGER_NET_SUPERVISOR_H_

#include <stdbool.h>

#include "esp_err.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute NetSupervisorStart.
 * @param runtime Parameter runtime.
 * @return Return the function result.
 */
  esp_err_t NetSupervisorStart(const app_runtime_t* runtime);

/**
 * @brief Execute NetSupervisorNotifyUpdate.
 */
  void NetSupervisorNotifyUpdate(void);

/**
 * @brief Get SNTP consecutive failure count.
 * @return Consecutive failure count.
 */
  uint32_t NetSupervisorGetSntpConsecutiveFailures(void);

/**
 * @brief Get SNTP failure alert state.
 * @return true if alert is active.
 */
  bool NetSupervisorIsSntpFailureAlertActive(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_NET_SUPERVISOR_H_
