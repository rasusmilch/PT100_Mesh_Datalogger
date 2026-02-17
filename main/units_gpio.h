#ifndef PT100_LOGGER_UNITS_GPIO_H_
#define PT100_LOGGER_UNITS_GPIO_H_

#include <stdbool.h>

#include "app_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    bool enabled;
    bool toggle_on_press;
    int32_t pin;
    app_units_gpio_pull_t pull;
    bool c_level_high;
    bool pin_valid;
    bool last_level_high;
    bool pressed_level_high;
    bool pressed;
    bool rtc_override_valid;
    app_display_units_t rtc_override_units;
    app_display_units_t effective_units;
    app_display_units_t saved_units;
  } units_gpio_status_t;

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
 * @brief Execute UnitsGpioGetStatus.
 * @param status_out Parameter status_out.
 */
  void UnitsGpioGetStatus(units_gpio_status_t* status_out);

/**
 * @brief Execute UnitsGpioClearRtcOverride.
 */
  void UnitsGpioClearRtcOverride(void);

/**
 * @brief Execute AppDisplayUnitsGetEffective.
 * @return Return the function result.
 */
  app_display_units_t AppDisplayUnitsGetEffective(void);

/**
 * @brief Execute UnitsGpioPullToString.
 * @param pull Parameter pull.
 * @return Return the function result.
 */
  const char* UnitsGpioPullToString(app_units_gpio_pull_t pull);

/**
 * @brief Execute UnitsGpioParsePull.
 * @param value Parameter value.
 * @param pull_out Parameter pull_out.
 * @return Return the function result.
 */
  bool UnitsGpioParsePull(const char* value, app_units_gpio_pull_t* pull_out);

/**
 * @brief Execute UnitsGpioLevelToString.
 * @param level_high Parameter level_high.
 * @return Return the function result.
 */
  const char* UnitsGpioLevelToString(bool level_high);

/**
 * @brief Execute UnitsGpioParseLevel.
 * @param value Parameter value.
 * @param level_high_out Parameter level_high_out.
 * @return Return the function result.
 */
  bool UnitsGpioParseLevel(const char* value, bool* level_high_out);

#ifdef __cplusplus
}
#endif

#endif  // PT100_LOGGER_UNITS_GPIO_H_
