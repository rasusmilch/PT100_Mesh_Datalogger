#ifndef PT100_LOGGER_CONSOLE_ALERTS_H_
#define PT100_LOGGER_CONSOLE_ALERTS_H_

#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute ConsoleAlertsRegister.
 * @param runtime Parameter runtime.
 */
void ConsoleAlertsRegister(app_runtime_t* runtime);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_CONSOLE_ALERTS_H_
