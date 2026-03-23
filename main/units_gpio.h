#ifndef PT100_LOGGER_UNITS_GPIO_H_
#define PT100_LOGGER_UNITS_GPIO_H_

#include <stdbool.h>

#include "app_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute UnitsGpioInit.
 * @param settings Parameter settings.
 */
  void UnitsGpioInit(app_settings_t* settings);

/**
 * @brief Execute UnitsGpioApplySettings.
 * @param settings Parameter settings.
 */
  void UnitsGpioApplySettings(const app_settings_t* settings);


/**
 * @brief Execute UnitsGpioHandleButtonPress.
 */
  void UnitsGpioHandleButtonPress(void);

/**
 * @brief Set temporary units override (RTC-backed) without touching NVS.
 * @param units Temporary units selection.
 * @return true when override was accepted.
 */
  bool UnitsGpioSetTemporaryUnits(app_display_units_t units);

/**
 * @brief Execute UnitsGpioClearRtcOverride.
 */
  void UnitsGpioClearRtcOverride(void);

/**
 * @brief Execute AppDisplayUnitsGetEffective.
 * @return Return the function result.
 */
  app_display_units_t AppDisplayUnitsGetEffective(void);

#ifdef __cplusplus
}
#endif

#endif  // PT100_LOGGER_UNITS_GPIO_H_
