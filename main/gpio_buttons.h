#ifndef PT100_LOGGER_GPIO_BUTTONS_H_
#define PT100_LOGGER_GPIO_BUTTONS_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  GPIO_BUTTON_RUN_START = 0,
  GPIO_BUTTON_RUN_STOP,
  GPIO_BUTTON_UNITS_TOGGLE,
  GPIO_BUTTON_COUNT,
} gpio_button_id_t;

typedef enum
{
  GPIO_BUTTON_EVENT_PRESS = 0,
} gpio_button_event_type_t;

typedef struct
{
  gpio_button_id_t id;
  gpio_num_t gpio;
  bool pull_up;
  bool pull_down;
  bool active_high;
  uint32_t debounce_ms;
  uint32_t hold_ms;
  uint32_t holdoff_ms;
  bool enabled;
} gpio_button_config_t;

typedef struct
{
  gpio_button_id_t id;
  gpio_button_event_type_t event_type;
  uint32_t uptime_ms;
  int gpio_level;
} gpio_button_event_t;

void GpioButtonsInit(void);
bool GpioButtonsRegister(const gpio_button_config_t* config);
bool GpioButtonsReceive(gpio_button_event_t* event_out, TickType_t timeout_ticks);
bool GpioButtonsIsActive(gpio_button_id_t id);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_GPIO_BUTTONS_H_
