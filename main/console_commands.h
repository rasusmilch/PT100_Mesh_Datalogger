#ifndef PT100_LOGGER_CONSOLE_COMMANDS_H_
#define PT100_LOGGER_CONSOLE_COMMANDS_H_

#include "esp_err.h"
#include "runtime_manager.h"
#include "boot_mode.h"

#ifdef __cplusplus
extern "C"
{
#endif

  // Initializes and starts the serial console.
  esp_err_t ConsoleCommandsStart(app_runtime_t* runtime,
                                 app_boot_mode_t boot_mode);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_CONSOLE_COMMANDS_H_
