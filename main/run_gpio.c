#include "run_gpio.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime_manager.h"

static const char* kTag = "run_gpio";
static const uint32_t kRunGpioPollMs = 10;

typedef struct
{
  bool enabled;
  gpio_num_t gpio;
  bool last_level;
  TickType_t last_change_ticks;
  TickType_t low_start_ticks;
  bool waiting_release;
} run_gpio_input_t;

typedef struct
{
  run_gpio_input_t start;
  run_gpio_input_t stop;
  uint32_t debounce_ms;
  uint32_t hold_ms;
} run_gpio_state_t;

static run_gpio_state_t g_run_gpio;

/**
 * @brief Execute ConfigureInput.
 * @param input Parameter input.
 * @param gpio_num Parameter gpio_num.
 * @return Return the function result.
 */
static bool
ConfigureInput(run_gpio_input_t* input, int gpio_num)
{
  if (input == NULL || gpio_num < 0) {
    return false;
  }

  gpio_config_t config = {
    .pin_bit_mask = 1ULL << gpio_num,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t result = gpio_config(&config);
  if (result != ESP_OK) {
    ESP_LOGW(
      kTag, "GPIO %d config failed: %s", gpio_num, esp_err_to_name(result));
    return false;
  }

  const TickType_t now_ticks = xTaskGetTickCount();
  input->enabled = true;
  input->gpio = (gpio_num_t)gpio_num;
  input->last_level = gpio_get_level(input->gpio);
  ESP_LOGI(
    kTag, "GPIO %d initial=%s", gpio_num, input->last_level ? "HIGH" : "LOW");
  input->last_change_ticks = now_ticks;
  input->low_start_ticks = input->last_level ? 0 : now_ticks;
  input->waiting_release = false;
  return true;
}

/**
 * @brief Execute UpdateInput.
 * @param input Parameter input.
 * @param now_ticks Parameter now_ticks.
 * @param debounce_ms Parameter debounce_ms.
 * @param hold_ms Parameter hold_ms.
 * @return Return the function result.
 */
static bool
UpdateInput(run_gpio_input_t* input,
            TickType_t now_ticks,
            uint32_t debounce_ms,
            uint32_t hold_ms)
{
  if (input == NULL || !input->enabled) {
    return false;
  }

  const bool level_high = (gpio_get_level(input->gpio) != 0);
  if (level_high != input->last_level) {
    input->last_level = level_high;
    input->last_change_ticks = now_ticks;
    if (level_high) {
      input->low_start_ticks = 0;
      input->waiting_release = false;
    } else {
      input->low_start_ticks = now_ticks;
    }
  }

  if (!level_high && !input->waiting_release) {
    const uint32_t stable_ms =
      (uint32_t)pdTICKS_TO_MS(now_ticks - input->last_change_ticks);
    const uint32_t low_ms =
      (uint32_t)pdTICKS_TO_MS(now_ticks - input->low_start_ticks);
    if (stable_ms >= debounce_ms && low_ms >= hold_ms) {
      input->waiting_release = true;
      return true;
    }
  }

  return false;
}

/**
 * @brief Execute PrimeInput.
 * @param input Parameter input.
 * @param debounce_ms Parameter debounce_ms.
 */
static void
PrimeInput(run_gpio_input_t* input, uint32_t debounce_ms)
{
  if (input == NULL || !input->enabled) {
    return;
  }

  bool last_level = (gpio_get_level(input->gpio) != 0);
  TickType_t last_change_ticks = xTaskGetTickCount();

  while (true) {
    const TickType_t now_ticks = xTaskGetTickCount();
    const bool level_high = (gpio_get_level(input->gpio) != 0);
    if (level_high != last_level) {
      last_level = level_high;
      last_change_ticks = now_ticks;
    }

    const uint32_t stable_ms =
      (uint32_t)pdTICKS_TO_MS(now_ticks - last_change_ticks);
    if (stable_ms >= debounce_ms) {
      input->last_level = last_level;
      input->last_change_ticks = now_ticks;
      input->low_start_ticks = last_level ? 0 : now_ticks;
      input->waiting_release = !last_level;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(kRunGpioPollMs));
  }
}

/**
 * @brief Execute RunGpioTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the RunGpioTask task.
 */
static void
RunGpioTask(void* context)
{
  run_gpio_state_t* state = (run_gpio_state_t*)context;

  PrimeInput(&state->stop, state->debounce_ms);
  PrimeInput(&state->start, state->debounce_ms);

  while (true) {
    const TickType_t now_ticks = xTaskGetTickCount();
    const bool stop_triggered =
      UpdateInput(&state->stop, now_ticks, state->debounce_ms, state->hold_ms);
    const bool start_triggered =
      UpdateInput(&state->start, now_ticks, state->debounce_ms, state->hold_ms);

    if (stop_triggered) {
      RuntimeRequestRunStop();
    } else if (start_triggered) {
      RuntimeRequestRunStart();
    }

    vTaskDelay(pdMS_TO_TICKS(kRunGpioPollMs));
  }
}

/**
 * @brief Execute RunGpioInit.
 */
void
RunGpioInit(void)
{
  memset(&g_run_gpio, 0, sizeof(g_run_gpio));
  g_run_gpio.debounce_ms = CONFIG_APP_RUN_GPIO_DEBOUNCE_MS;
  g_run_gpio.hold_ms = CONFIG_APP_RUN_GPIO_HOLD_MS;

  bool any_enabled = false;
#if CONFIG_APP_RUN_STOP_GPIO_ENABLE
  if (ConfigureInput(&g_run_gpio.stop, CONFIG_APP_RUN_STOP_GPIO_NUM)) {
    ESP_LOGI(kTag, "Run stop GPIO enabled on %d", CONFIG_APP_RUN_STOP_GPIO_NUM);
    any_enabled = true;
  } else {
    ESP_LOGW(kTag, "Run stop GPIO enabled but invalid config");
  }
#endif

#if CONFIG_APP_RUN_START_GPIO_ENABLE
  if (ConfigureInput(&g_run_gpio.start, CONFIG_APP_RUN_START_GPIO_NUM)) {
    ESP_LOGI(
      kTag, "Run start GPIO enabled on %d", CONFIG_APP_RUN_START_GPIO_NUM);
    any_enabled = true;
  } else {
    ESP_LOGW(kTag, "Run start GPIO enabled but invalid config");
  }
#endif

  if (!any_enabled) {
    return;
  }

  BaseType_t created =
    xTaskCreate(&RunGpioTask, "run_gpio", 2048, &g_run_gpio, 3, NULL);
  if (created != pdPASS) {
    ESP_LOGE(kTag, "Failed to create run GPIO task");
  }
}
