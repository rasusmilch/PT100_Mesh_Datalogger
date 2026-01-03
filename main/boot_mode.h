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

/**
 * @brief Execute BootModeReadFromNvsOrDefault.
 * @return Return the function result.
 */
  app_boot_mode_t BootModeReadFromNvsOrDefault(void);

/**
 * @brief Execute BootModeWriteToNvs.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
  esp_err_t BootModeWriteToNvs(app_boot_mode_t mode);

/**
 * @brief Execute BootModeDetermineAtStartup.
 * @return Return the function result.
 */
  app_boot_mode_t BootModeDetermineAtStartup(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_BOOT_MODE_H_
