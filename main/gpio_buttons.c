#include "gpio_buttons.h"

#include <string.h>

#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const uint32_t kButtonsQueueDepth = 8;

typedef struct
{
  bool registered;
  bool configured;
  gpio_button_config_t config;
  bool last_level;
  bool last_active;
  TickType_t last_change_ticks;
  TickType_t active_start_ticks;
  TickType_t holdoff_until_ticks;
  bool waiting_release;
} gpio_button_state_t;

static QueueHandle_t g_buttons_queue = NULL;
static TaskHandle_t g_buttons_task = NULL;
static portMUX_TYPE g_buttons_lock = portMUX_INITIALIZER_UNLOCKED;
static gpio_button_state_t g_buttons[GPIO_BUTTON_COUNT];

static bool GpioButtonsConfigureInput(const gpio_button_config_t* config);
static void GpioButtonsPrimeState(gpio_button_state_t* button);
static bool GpioButtonsUpdate(gpio_button_state_t* button,
                              TickType_t now_ticks,
                              gpio_button_event_t* event_out);
static bool GpioButtonsLevelToActive(const gpio_button_state_t* button,
                                     bool level_high);
static void GpioButtonsTask(void* context);

static bool
GpioButtonsConfigureInput(const gpio_button_config_t* config)
{
  if (config == NULL || !config->enabled || config->gpio < 0) {
    return false;
  }

  gpio_config_t gpio_config_input = {
    .pin_bit_mask = 1ULL << config->gpio,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = config->pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
    .pull_down_en =
      config->pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };

  return gpio_config(&gpio_config_input) == ESP_OK;
}

static bool
GpioButtonsLevelToActive(const gpio_button_state_t* button, bool level_high)
{
  if (button == NULL) {
    return false;
  }
  return button->config.active_high ? level_high : !level_high;
}

static void
GpioButtonsPrimeState(gpio_button_state_t* button)
{
  if (button == NULL || !button->registered || !button->configured) {
    return;
  }

  bool last_level = (gpio_get_level(button->config.gpio) != 0);
  TickType_t last_change_ticks = xTaskGetTickCount();

  while (true) {
    const TickType_t now_ticks = xTaskGetTickCount();
    const bool level_high = (gpio_get_level(button->config.gpio) != 0);
    if (level_high != last_level) {
      last_level = level_high;
      last_change_ticks = now_ticks;
    }

    const uint32_t stable_ms =
      (uint32_t)pdTICKS_TO_MS(now_ticks - last_change_ticks);
    if (stable_ms >= button->config.debounce_ms) {
      button->last_level = last_level;
      button->last_change_ticks = now_ticks;
      button->last_active = GpioButtonsLevelToActive(button, last_level);
      button->active_start_ticks = button->last_active ? now_ticks : 0;
      button->waiting_release = button->last_active;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_APP_BUTTONS_POLL_MS));
  }
}

static bool
GpioButtonsUpdate(gpio_button_state_t* button,
                  TickType_t now_ticks,
                  gpio_button_event_t* event_out)
{
  if (button == NULL || !button->registered || !button->configured
      || event_out == NULL) {
    return false;
  }

  const bool level_high = (gpio_get_level(button->config.gpio) != 0);
  if (level_high != button->last_level) {
    button->last_level = level_high;
    button->last_change_ticks = now_ticks;
    if (GpioButtonsLevelToActive(button, level_high)) {
      button->active_start_ticks = now_ticks;
    } else {
      button->active_start_ticks = 0;
    }
  }

  const uint32_t stable_ms =
    (uint32_t)pdTICKS_TO_MS(now_ticks - button->last_change_ticks);
  if (stable_ms < button->config.debounce_ms) {
    return false;
  }

  const bool active = GpioButtonsLevelToActive(button, button->last_level);
  if (active != button->last_active) {
    button->last_active = active;
    if (!active) {
      button->waiting_release = false;
    }
  }

  if (active && !button->waiting_release) {
    const uint32_t active_ms =
      (uint32_t)pdTICKS_TO_MS(now_ticks - button->active_start_ticks);
    if (active_ms >= button->config.hold_ms) {
      if (button->holdoff_until_ticks != 0
          && now_ticks < button->holdoff_until_ticks) {
        button->waiting_release = true;
        return false;
      }
      button->waiting_release = true;
      button->holdoff_until_ticks =
        now_ticks + pdMS_TO_TICKS(button->config.holdoff_ms);
      event_out->id = button->config.id;
      event_out->event_type = GPIO_BUTTON_EVENT_PRESS;
      event_out->uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
      event_out->gpio_level = button->last_level ? 1 : 0;
      return true;
    }
  }

  return false;
}

static void
GpioButtonsTask(void* context)
{
  (void)context;

  for (size_t i = 0; i < GPIO_BUTTON_COUNT; ++i) {
    GpioButtonsPrimeState(&g_buttons[i]);
  }

  while (true) {
    const TickType_t now_ticks = xTaskGetTickCount();
    for (size_t i = 0; i < GPIO_BUTTON_COUNT; ++i) {
      gpio_button_event_t event = { 0 };
      if (GpioButtonsUpdate(&g_buttons[i], now_ticks, &event)) {
        (void)xQueueSend(g_buttons_queue, &event, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(CONFIG_APP_BUTTONS_POLL_MS));
  }
}

void
GpioButtonsInit(void)
{
  if (g_buttons_queue == NULL) {
    g_buttons_queue = xQueueCreate(kButtonsQueueDepth, sizeof(gpio_button_event_t));
  }
  if (g_buttons_task == NULL && g_buttons_queue != NULL) {
    (void)xTaskCreate(&GpioButtonsTask,
                      "gpio_buttons",
                      CONFIG_APP_BUTTONS_TASK_STACK_BYTES,
                      NULL,
                      3,
                      &g_buttons_task);
  }
}

bool
GpioButtonsRegister(const gpio_button_config_t* config)
{
  if (config == NULL || config->id >= GPIO_BUTTON_COUNT) {
    return false;
  }

  bool configured = false;
  if (config->enabled) {
    configured = GpioButtonsConfigureInput(config);
    if (!configured) {
      return false;
    }
  }

  taskENTER_CRITICAL(&g_buttons_lock);
  g_buttons[config->id].registered = config->enabled;
  g_buttons[config->id].configured = configured;
  g_buttons[config->id].config = *config;
  g_buttons[config->id].last_level = true;
  g_buttons[config->id].last_active = false;
  g_buttons[config->id].last_change_ticks = xTaskGetTickCount();
  g_buttons[config->id].active_start_ticks = 0;
  g_buttons[config->id].holdoff_until_ticks = 0;
  g_buttons[config->id].waiting_release = false;
  taskEXIT_CRITICAL(&g_buttons_lock);
  return true;
}

bool
GpioButtonsReceive(gpio_button_event_t* event_out, TickType_t timeout_ticks)
{
  if (event_out == NULL || g_buttons_queue == NULL) {
    return false;
  }
  return xQueueReceive(g_buttons_queue, event_out, timeout_ticks) == pdTRUE;
}

bool
GpioButtonsIsActive(gpio_button_id_t id)
{
  if (id >= GPIO_BUTTON_COUNT) {
    return false;
  }
  return g_buttons[id].last_active;
}
