#ifndef PT100_LOGGER_BOOT_MODE_H_
#define PT100_LOGGER_BOOT_MODE_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum
  {
    APP_BOOT_MODE_DIAGNOSTICS = 0,
    APP_BOOT_MODE_RUN = 1,
  } app_boot_mode_t;

  app_boot_mode_t BootModeReadFromNvsOrDefault(void);

  esp_err_t BootModeWriteToNvs(app_boot_mode_t mode);

  app_boot_mode_t BootModeDetermineAtStartup(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_BOOT_MODE_H_
