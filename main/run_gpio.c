#include "run_gpio.h"

#include "gpio_buttons.h"

void
RunGpioInit(void)
{
  GpioButtonsInit();

#if CONFIG_APP_RUN_STOP_GPIO_ENABLE
  const gpio_button_config_t stop_config = {
    .id = GPIO_BUTTON_RUN_STOP,
    .gpio = (gpio_num_t)CONFIG_APP_RUN_STOP_GPIO_NUM,
    .pull_up = true,
    .pull_down = false,
    .active_high = false,
    .debounce_ms = CONFIG_APP_BUTTONS_DEBOUNCE_MS,
    .hold_ms = CONFIG_APP_RUN_STOP_HOLD_MS,
    .holdoff_ms = CONFIG_APP_BUTTONS_HOLDOFF_MS,
    .enabled = true,
  };
  (void)GpioButtonsRegister(&stop_config);
#endif

#if CONFIG_APP_RUN_START_GPIO_ENABLE
  const gpio_button_config_t start_config = {
    .id = GPIO_BUTTON_RUN_START,
    .gpio = (gpio_num_t)CONFIG_APP_RUN_START_GPIO_NUM,
    .pull_up = true,
    .pull_down = false,
    .active_high = false,
    .debounce_ms = CONFIG_APP_BUTTONS_DEBOUNCE_MS,
    .hold_ms = CONFIG_APP_RUN_START_HOLD_MS,
    .holdoff_ms = CONFIG_APP_BUTTONS_HOLDOFF_MS,
    .enabled = true,
  };
  (void)GpioButtonsRegister(&start_config);
#endif
}

bool
RunGpioStopActive(void)
{
  return GpioButtonsIsActive(GPIO_BUTTON_RUN_STOP);
}
