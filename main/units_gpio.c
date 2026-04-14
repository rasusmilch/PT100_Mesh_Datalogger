#include "units_gpio.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "gpio_buttons.h"
#include "freertos/FreeRTOS.h"

#if defined(CONFIG_APP_UNITS_GPIO_ENABLE)
#define APP_UNITS_GPIO_ENABLED_BY_KCONFIG 1
#else
#define APP_UNITS_GPIO_ENABLED_BY_KCONFIG 0
#endif

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
static app_display_units_t UnitsGpioGetBootUnitsLocked(void);
static app_display_units_t UnitsGpioGetEffectiveUnitsLocked(void);
static void UnitsGpioUpdateState(int32_t pin,
                                 app_units_gpio_pull_t pull,
                                 bool c_level_high,
                                 bool pin_valid);
static void UnitsGpioRegisterButtonLocked(void);

RTC_DATA_ATTR static units_gpio_rtc_store_t g_units_gpio_rtc = {
  .magic = 0,
  .units = 0,
};

typedef struct
{
  app_settings_t* settings;
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
  app_display_units_t effective_units;
} units_gpio_state_t;

static units_gpio_state_t g_units_gpio = {
  .settings = NULL,
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
  return APP_DISPLAY_UNITS_C;
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
  g_units_gpio.rtc_override_units = APP_DISPLAY_UNITS_C;
}

static app_display_units_t
UnitsGpioGetBootUnitsLocked(void)
{
  return (g_units_gpio.settings != NULL) ? g_units_gpio.settings->display_units
                                         : APP_DISPLAY_UNITS_C;
}

static app_display_units_t
UnitsGpioGetEffectiveUnitsLocked(void)
{
  const app_display_units_t boot_units = UnitsGpioGetBootUnitsLocked();

  if (!g_units_gpio.enabled) {
    return boot_units;
  }

  if (g_units_gpio.toggle_on_press) {
    if (g_units_gpio.rtc_override_valid) {
      return g_units_gpio.rtc_override_units;
    }
    return boot_units;
  }

  if (!g_units_gpio.pin_valid) {
    return boot_units;
  }

  return UnitsGpioComputeUnits(g_units_gpio.last_level_high,
                               g_units_gpio.c_level_high);
}

static bool
UnitsGpioIsValidPin(int32_t pin)
{
  if (pin < 0) {
    return false;
  }
  return GPIO_IS_VALID_GPIO((gpio_num_t)pin);
}

static app_display_units_t
UnitsGpioComputeUnits(bool level_high, bool c_level_high)
{
  return (level_high == c_level_high) ? APP_DISPLAY_UNITS_C
                                      : APP_DISPLAY_UNITS_F;
}

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

  return gpio_config(&config) == ESP_OK;
}

static void
UnitsGpioRegisterButtonLocked(void)
{
  const gpio_button_config_t units_button = {
    .id = GPIO_BUTTON_UNITS_TOGGLE,
    .gpio = (gpio_num_t)g_units_gpio.pin,
    .pull_up = (g_units_gpio.pull == APP_UNITS_GPIO_PULL_UP),
    .pull_down = (g_units_gpio.pull == APP_UNITS_GPIO_PULL_DOWN),
    .active_high = g_units_gpio.pressed_level_high,
    .debounce_ms = CONFIG_APP_BUTTONS_DEBOUNCE_MS,
    .hold_ms = CONFIG_APP_UNITS_TOGGLE_HOLD_MS,
    .holdoff_ms = CONFIG_APP_BUTTONS_HOLDOFF_MS,
    .enabled = g_units_gpio.enabled && g_units_gpio.pin_valid
               && g_units_gpio.toggle_on_press,
  };
  (void)GpioButtonsRegister(&units_button);
}

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
  g_units_gpio.pin = pin;
  g_units_gpio.pull = pull;
  g_units_gpio.c_level_high = c_level_high;
  g_units_gpio.pin_valid = pin_valid;
  g_units_gpio.toggle_on_press = UnitsGpioToggleModeEnabled();
  g_units_gpio.rtc_override_valid = UnitsGpioIsRtcOverrideValid();
  g_units_gpio.rtc_override_units = UnitsGpioGetRtcOverrideUnits();
  g_units_gpio.last_level_high = level_high;
  g_units_gpio.pressed_level_high = UnitsGpioPressedLevelHigh(pull, c_level_high);
  g_units_gpio.pressed = pin_valid && (level_high == g_units_gpio.pressed_level_high);
  UnitsGpioRegisterButtonLocked();
  g_units_gpio.effective_units = UnitsGpioGetEffectiveUnitsLocked();
  portEXIT_CRITICAL(&g_units_gpio.lock);
}

void
UnitsGpioInit(app_settings_t* settings)
{
  GpioButtonsInit();
  g_units_gpio.settings = settings;
  g_units_gpio.enabled = (APP_UNITS_GPIO_ENABLED_BY_KCONFIG != 0);
  UnitsGpioApplySettings(settings);
}

void
UnitsGpioApplySettings(const app_settings_t* settings)
{
  if (settings == NULL) {
    return;
  }

  const bool enabled = (APP_UNITS_GPIO_ENABLED_BY_KCONFIG != 0);
  const int32_t pin = (int32_t)CONFIG_APP_UNITS_GPIO_DEFAULT_PIN;
  const app_units_gpio_pull_t pull =
    (app_units_gpio_pull_t)CONFIG_APP_UNITS_GPIO_DEFAULT_PULL;
#if CONFIG_APP_UNITS_GPIO_DEFAULT_LEVEL_FOR_C
  const bool c_level_high = true;
#else
  const bool c_level_high = false;
#endif
  bool pin_valid = enabled && UnitsGpioIsValidPin(pin);

  g_units_gpio.enabled = enabled;
  if (enabled && pin_valid) {
    if (!UnitsGpioConfigureInput(pin, pull)) {
      pin_valid = false;
    }
  }

  UnitsGpioUpdateState(pin, pull, c_level_high, pin_valid);
}

void
UnitsGpioHandleButtonPress(void)
{
  portENTER_CRITICAL(&g_units_gpio.lock);
  if (!g_units_gpio.enabled || !g_units_gpio.pin_valid || !g_units_gpio.toggle_on_press) {
    portEXIT_CRITICAL(&g_units_gpio.lock);
    return;
  }

  const app_display_units_t old_units = UnitsGpioGetEffectiveUnitsLocked();
  const app_display_units_t new_units =
    (old_units == APP_DISPLAY_UNITS_F) ? APP_DISPLAY_UNITS_C : APP_DISPLAY_UNITS_F;
  UnitsGpioSetRtcOverride(new_units);
  g_units_gpio.rtc_override_valid = true;
  g_units_gpio.rtc_override_units = new_units;
  g_units_gpio.effective_units = new_units;
  portEXIT_CRITICAL(&g_units_gpio.lock);
}

bool
UnitsGpioSetTemporaryUnits(app_display_units_t units)
{
  if (units != APP_DISPLAY_UNITS_C && units != APP_DISPLAY_UNITS_F) {
    return false;
  }

  portENTER_CRITICAL(&g_units_gpio.lock);
  UnitsGpioSetRtcOverride(units);
  g_units_gpio.rtc_override_valid = true;
  g_units_gpio.rtc_override_units = units;
  g_units_gpio.effective_units = units;
  portEXIT_CRITICAL(&g_units_gpio.lock);
  return true;
}

void
UnitsGpioClearRtcOverride(void)
{
  portENTER_CRITICAL(&g_units_gpio.lock);
  UnitsGpioClearRtcOverrideLocked();
  g_units_gpio.effective_units = UnitsGpioGetEffectiveUnitsLocked();
  portEXIT_CRITICAL(&g_units_gpio.lock);
}

app_display_units_t
AppDisplayUnitsGetEffective(void)
{
  app_display_units_t units = APP_DISPLAY_UNITS_C;
  portENTER_CRITICAL(&g_units_gpio.lock);
  if (!g_units_gpio.toggle_on_press && g_units_gpio.pin_valid) {
    g_units_gpio.last_level_high = (gpio_get_level((gpio_num_t)g_units_gpio.pin) != 0);
  }
  units = UnitsGpioGetEffectiveUnitsLocked();
  g_units_gpio.effective_units = units;
  portEXIT_CRITICAL(&g_units_gpio.lock);
  return units;
}
