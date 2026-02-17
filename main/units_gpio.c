#include "units_gpio.h"

#include <string.h>
#include <strings.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* kTag = "units_gpio";
static const uint32_t kUnitsGpioPollMs = 10;
static const uint32_t kUnitsGpioDebounceMs = 30;
static const uint32_t kUnitsGpioRtcMagic = 0x55475031;

typedef struct
{
  uint32_t magic;
  uint32_t units;
} units_gpio_rtc_store_t;

static bool UnitsGpioIsValidPin(int32_t pin);
static app_display_units_t UnitsGpioComputeUnits(bool level_high, bool c_level_high);
static bool UnitsGpioConfigureInput(int32_t pin, app_units_gpio_pull_t pull);
static bool UnitsGpioToggleModeEnabled(void);
static bool UnitsGpioPressedLevelHigh(app_units_gpio_pull_t pull, bool c_level_high);
static bool UnitsGpioIsRtcOverrideValid(void);
static app_display_units_t UnitsGpioGetRtcOverrideUnits(void);
static void UnitsGpioSetRtcOverride(app_display_units_t units);
static void UnitsGpioClearRtcOverrideLocked(void);
static app_display_units_t UnitsGpioGetSavedUnitsLocked(void);
static app_display_units_t UnitsGpioGetEffectiveUnitsLocked(void);
static void UnitsGpioUpdateState(int32_t pin,
                                 app_units_gpio_pull_t pull,
                                 bool c_level_high,
                                 bool pin_valid);
static void UnitsGpioTask(void* context);

RTC_DATA_ATTR static units_gpio_rtc_store_t g_units_gpio_rtc = {
  .magic = 0,
  .units = 0,
};

typedef struct
{
  app_settings_t* settings;
  TaskHandle_t task;
  portMUX_TYPE lock;
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
  bool debounce_last_sample_high;
  bool debounce_stable_level_high;
  TickType_t debounce_last_change_tick;
  bool debounce_last_pressed;
  app_display_units_t effective_units;
} units_gpio_state_t;

static units_gpio_state_t g_units_gpio = {
  .settings = NULL,
  .task = NULL,
  .lock = portMUX_INITIALIZER_UNLOCKED,
  .enabled = false,
  .toggle_on_press = false,
  .pin = -1,
  .pull = APP_UNITS_GPIO_PULL_NONE,
  .c_level_high = false,
  .pin_valid = false,
  .last_level_high = false,
  .pressed_level_high = false,
  .pressed = false,
  .rtc_override_valid = false,
  .rtc_override_units = APP_DISPLAY_UNITS_F,
  .debounce_last_sample_high = false,
  .debounce_stable_level_high = false,
  .debounce_last_change_tick = 0,
  .debounce_last_pressed = false,
  .effective_units = APP_DISPLAY_UNITS_F,
};

static bool
UnitsGpioToggleModeEnabled(void)
{
#if CONFIG_APP_UNITS_GPIO_TOGGLE_ON_PRESS
  return true;
#else
  return false;
#endif
}

static bool
UnitsGpioPressedLevelHigh(app_units_gpio_pull_t pull, bool c_level_high)
{
  if (pull == APP_UNITS_GPIO_PULL_UP) {
    return false;
  }
  if (pull == APP_UNITS_GPIO_PULL_DOWN) {
    return true;
  }
  return c_level_high;
}

static bool
UnitsGpioIsRtcOverrideValid(void)
{
  if (g_units_gpio_rtc.magic != kUnitsGpioRtcMagic) {
    return false;
  }
  return (g_units_gpio_rtc.units == (uint32_t)APP_DISPLAY_UNITS_C)
         || (g_units_gpio_rtc.units == (uint32_t)APP_DISPLAY_UNITS_F);
}

static app_display_units_t
UnitsGpioGetRtcOverrideUnits(void)
{
  if (UnitsGpioIsRtcOverrideValid()) {
    return (app_display_units_t)g_units_gpio_rtc.units;
  }
  return APP_DISPLAY_UNITS_F;
}

static void
UnitsGpioSetRtcOverride(app_display_units_t units)
{
  g_units_gpio_rtc.magic = kUnitsGpioRtcMagic;
  g_units_gpio_rtc.units = (uint32_t)units;
}

static void
UnitsGpioClearRtcOverrideLocked(void)
{
  g_units_gpio_rtc.magic = 0;
  g_units_gpio_rtc.units = 0;
  g_units_gpio.rtc_override_valid = false;
  g_units_gpio.rtc_override_units = APP_DISPLAY_UNITS_F;
}

static app_display_units_t
UnitsGpioGetSavedUnitsLocked(void)
{
  return (g_units_gpio.settings != NULL) ? g_units_gpio.settings->display_units
                                         : APP_DISPLAY_UNITS_F;
}

static app_display_units_t
UnitsGpioGetEffectiveUnitsLocked(void)
{
  const app_display_units_t saved_units = UnitsGpioGetSavedUnitsLocked();

  if (!g_units_gpio.enabled) {
    return saved_units;
  }

  if (g_units_gpio.toggle_on_press) {
    if (g_units_gpio.rtc_override_valid) {
      return g_units_gpio.rtc_override_units;
    }
    return saved_units;
  }

  if (!g_units_gpio.pin_valid) {
    return saved_units;
  }

  return UnitsGpioComputeUnits(g_units_gpio.last_level_high,
                               g_units_gpio.c_level_high);
}

/**
 * @brief Check whether a configured pin number refers to a valid GPIO.
 * @param pin GPIO number as a signed integer.
 * @return true if the pin is non-negative and refers to a valid GPIO; otherwise false.
 */
static bool
UnitsGpioIsValidPin(int32_t pin)
{
  if (pin < 0) {
    return false;
  }
  return GPIO_IS_VALID_GPIO((gpio_num_t)pin);
}

/**
 * @brief Execute UnitsGpioComputeUnits.
 * @param level_high Parameter level_high.
 * @param c_level_high Parameter c_level_high.
 * @return Return the function result.
 */
static app_display_units_t
UnitsGpioComputeUnits(bool level_high, bool c_level_high)
{
  return (level_high == c_level_high) ? APP_DISPLAY_UNITS_C
                                      : APP_DISPLAY_UNITS_F;
}

/**
 * @brief Execute UnitsGpioConfigureInput.
 * @param pin Parameter pin.
 * @param pull Parameter pull.
 * @return Return the function result.
 */
static bool
UnitsGpioConfigureInput(int32_t pin, app_units_gpio_pull_t pull)
{
  gpio_config_t config = {
    .pin_bit_mask = 1ULL << pin,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en =
      (pull == APP_UNITS_GPIO_PULL_UP) ? GPIO_PULLUP_ENABLE
                                       : GPIO_PULLUP_DISABLE,
    .pull_down_en =
      (pull == APP_UNITS_GPIO_PULL_DOWN) ? GPIO_PULLDOWN_ENABLE
                                         : GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t result = gpio_config(&config);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "GPIO %ld config failed: %s", (long)pin, esp_err_to_name(result));
    return false;
  }
  return true;
}

/**
 * @brief Execute UnitsGpioUpdateState.
 * @param pin Parameter pin.
 * @param pull Parameter pull.
 * @param c_level_high Parameter c_level_high.
 * @param pin_valid Parameter pin_valid.
 */
static void
UnitsGpioUpdateState(int32_t pin,
                     app_units_gpio_pull_t pull,
                     bool c_level_high,
                     bool pin_valid)
{
  bool level_high = false;
  if (pin_valid) {
    level_high = (gpio_get_level((gpio_num_t)pin) != 0);
  }

  portENTER_CRITICAL(&g_units_gpio.lock);
  const bool toggle_on_press = UnitsGpioToggleModeEnabled();
  const bool rtc_override_valid = UnitsGpioIsRtcOverrideValid();
  const app_display_units_t rtc_override_units = UnitsGpioGetRtcOverrideUnits();
  const bool pressed_level_high = UnitsGpioPressedLevelHigh(pull, c_level_high);
  g_units_gpio.pin = pin;
  g_units_gpio.pull = pull;
  g_units_gpio.c_level_high = c_level_high;
  g_units_gpio.pin_valid = pin_valid;
  g_units_gpio.toggle_on_press = toggle_on_press;
  g_units_gpio.rtc_override_valid = rtc_override_valid;
  g_units_gpio.rtc_override_units = rtc_override_units;
  g_units_gpio.last_level_high = level_high;
  g_units_gpio.pressed_level_high = pressed_level_high;
  g_units_gpio.pressed = pin_valid && (level_high == pressed_level_high);
  g_units_gpio.debounce_last_sample_high = level_high;
  g_units_gpio.debounce_stable_level_high = level_high;
  g_units_gpio.debounce_last_change_tick = xTaskGetTickCount();
  g_units_gpio.debounce_last_pressed = g_units_gpio.pressed;
  g_units_gpio.effective_units = UnitsGpioGetEffectiveUnitsLocked();
  portEXIT_CRITICAL(&g_units_gpio.lock);
}

/**
 * @brief Execute UnitsGpioTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the UnitsGpio task.
 */
static void
UnitsGpioTask(void* context)
{
  (void)context;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(kUnitsGpioPollMs));
    if (!g_units_gpio.enabled || !g_units_gpio.pin_valid) {
      continue;
    }
    const TickType_t now = xTaskGetTickCount();
    const bool level_high = (gpio_get_level((gpio_num_t)g_units_gpio.pin) != 0);
    bool log_toggle = false;
    app_display_units_t old_units = APP_DISPLAY_UNITS_F;
    app_display_units_t new_units = APP_DISPLAY_UNITS_F;

    portENTER_CRITICAL(&g_units_gpio.lock);
    g_units_gpio.last_level_high = level_high;

    if (!g_units_gpio.toggle_on_press) {
      g_units_gpio.effective_units =
        UnitsGpioComputeUnits(level_high, g_units_gpio.c_level_high);
      portEXIT_CRITICAL(&g_units_gpio.lock);
      continue;
    }

    if (level_high != g_units_gpio.debounce_last_sample_high) {
      g_units_gpio.debounce_last_sample_high = level_high;
      g_units_gpio.debounce_last_change_tick = now;
    }

    const TickType_t debounce_ticks = pdMS_TO_TICKS(kUnitsGpioDebounceMs);
    if (((now - g_units_gpio.debounce_last_change_tick) >= debounce_ticks)
        && (level_high != g_units_gpio.debounce_stable_level_high)) {
      g_units_gpio.debounce_stable_level_high = level_high;
      g_units_gpio.pressed =
        (g_units_gpio.debounce_stable_level_high == g_units_gpio.pressed_level_high);

      if (g_units_gpio.pressed && !g_units_gpio.debounce_last_pressed) {
        old_units = g_units_gpio.effective_units;
        new_units = (old_units == APP_DISPLAY_UNITS_F) ? APP_DISPLAY_UNITS_C
                                                        : APP_DISPLAY_UNITS_F;
        UnitsGpioSetRtcOverride(new_units);
        g_units_gpio.rtc_override_valid = true;
        g_units_gpio.rtc_override_units = new_units;
        g_units_gpio.effective_units = new_units;
        log_toggle = true;
      }

      g_units_gpio.debounce_last_pressed = g_units_gpio.pressed;
    }

    g_units_gpio.effective_units = UnitsGpioGetEffectiveUnitsLocked();
    portEXIT_CRITICAL(&g_units_gpio.lock);

    if (log_toggle) {
      ESP_LOGI(kTag,
               "Units toggled: %s -> %s (RTC persisted)",
               AppSettingsDisplayUnitsToString(old_units),
               AppSettingsDisplayUnitsToString(new_units));
    }
  }
}

/**
 * @brief Execute UnitsGpioInit.
 * @param settings Parameter settings.
 */
void
UnitsGpioInit(app_settings_t* settings)
{
  g_units_gpio.settings = settings;
  g_units_gpio.enabled = (CONFIG_APP_UNITS_GPIO_ENABLE != 0);
  UnitsGpioApplySettings(settings);

  if (g_units_gpio.enabled && g_units_gpio.task == NULL) {
    const uint32_t kUnitsGpioTaskStackBytes = 2048;
    BaseType_t created = xTaskCreate(&UnitsGpioTask,
                                     "units_gpio",
                                     kUnitsGpioTaskStackBytes,
                                     NULL,
                                     1,
                                     &g_units_gpio.task);
    if (created != pdPASS) {
      g_units_gpio.task = NULL;
      ESP_LOGW(kTag, "Units GPIO task create failed");
    }
  }
}

/**
 * @brief Execute UnitsGpioApplySettings.
 * @param settings Parameter settings.
 */
void
UnitsGpioApplySettings(const app_settings_t* settings)
{
  if (settings == NULL) {
    return;
  }

  const bool enabled = (CONFIG_APP_UNITS_GPIO_ENABLE != 0);
  const int32_t pin = settings->units_gpio_pin;
  const app_units_gpio_pull_t pull = settings->units_gpio_pull;
  const bool c_level_high = settings->units_gpio_c_level_high;
  bool pin_valid = enabled && UnitsGpioIsValidPin(pin);

  g_units_gpio.enabled = enabled;
  if (enabled && pin_valid) {
    if (!UnitsGpioConfigureInput(pin, pull)) {
      pin_valid = false;
    }
  }

  UnitsGpioUpdateState(pin, pull, c_level_high, pin_valid);
}

/**
 * @brief Execute UnitsGpioGetStatus.
 * @param status_out Parameter status_out.
 */
void
UnitsGpioGetStatus(units_gpio_status_t* status_out)
{
  if (status_out == NULL) {
    return;
  }
  portENTER_CRITICAL(&g_units_gpio.lock);
  *status_out = (units_gpio_status_t){
    .enabled = g_units_gpio.enabled,
    .toggle_on_press = g_units_gpio.toggle_on_press,
    .pin = g_units_gpio.pin,
    .pull = g_units_gpio.pull,
    .c_level_high = g_units_gpio.c_level_high,
    .pin_valid = g_units_gpio.pin_valid,
    .last_level_high = g_units_gpio.last_level_high,
    .pressed_level_high = g_units_gpio.pressed_level_high,
    .pressed = g_units_gpio.pressed,
    .rtc_override_valid = g_units_gpio.rtc_override_valid,
    .rtc_override_units = g_units_gpio.rtc_override_units,
    .effective_units = g_units_gpio.effective_units,
    .saved_units = UnitsGpioGetSavedUnitsLocked(),
  };
  portEXIT_CRITICAL(&g_units_gpio.lock);
}

void
UnitsGpioClearRtcOverride(void)
{
  portENTER_CRITICAL(&g_units_gpio.lock);
  UnitsGpioClearRtcOverrideLocked();
  g_units_gpio.effective_units = UnitsGpioGetEffectiveUnitsLocked();
  portEXIT_CRITICAL(&g_units_gpio.lock);
}

/**
 * @brief Execute AppDisplayUnitsGetEffective.
 * @return Return the function result.
 */
app_display_units_t
AppDisplayUnitsGetEffective(void)
{
  app_display_units_t units = APP_DISPLAY_UNITS_F;
  portENTER_CRITICAL(&g_units_gpio.lock);
  units = UnitsGpioGetEffectiveUnitsLocked();
  portEXIT_CRITICAL(&g_units_gpio.lock);
  return units;
}

/**
 * @brief Execute UnitsGpioPullToString.
 * @param pull Parameter pull.
 * @return Return the function result.
 */
const char*
UnitsGpioPullToString(app_units_gpio_pull_t pull)
{
  switch (pull) {
    case APP_UNITS_GPIO_PULL_NONE:
      return "none";
    case APP_UNITS_GPIO_PULL_UP:
      return "up";
    case APP_UNITS_GPIO_PULL_DOWN:
      return "down";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute UnitsGpioParsePull.
 * @param value Parameter value.
 * @param pull_out Parameter pull_out.
 * @return Return the function result.
 */
bool
UnitsGpioParsePull(const char* value, app_units_gpio_pull_t* pull_out)
{
  if (value == NULL || pull_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "none") == 0) {
    *pull_out = APP_UNITS_GPIO_PULL_NONE;
    return true;
  }
  if (strcasecmp(value, "up") == 0) {
    *pull_out = APP_UNITS_GPIO_PULL_UP;
    return true;
  }
  if (strcasecmp(value, "down") == 0) {
    *pull_out = APP_UNITS_GPIO_PULL_DOWN;
    return true;
  }
  return false;
}

/**
 * @brief Execute UnitsGpioLevelToString.
 * @param level_high Parameter level_high.
 * @return Return the function result.
 */
const char*
UnitsGpioLevelToString(bool level_high)
{
  return level_high ? "high" : "low";
}

/**
 * @brief Execute UnitsGpioParseLevel.
 * @param value Parameter value.
 * @param level_high_out Parameter level_high_out.
 * @return Return the function result.
 */
bool
UnitsGpioParseLevel(const char* value, bool* level_high_out)
{
  if (value == NULL || level_high_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "high") == 0) {
    *level_high_out = true;
    return true;
  }
  if (strcasecmp(value, "low") == 0) {
    *level_high_out = false;
    return true;
  }
  return false;
}
