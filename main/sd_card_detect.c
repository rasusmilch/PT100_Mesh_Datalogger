#include "sd_card_detect.h"

#include <string.h>

#include "driver/gpio.h"
#include "freertos/task.h"

static bool
SdCardDetectRawPresent(const sd_card_detect_t* detect)
{
  if (detect == NULL || !detect->enabled) {
    return true;
  }
  const int level = gpio_get_level((gpio_num_t)detect->gpio);
  const bool level_high = (level != 0);
  return detect->present_level_high ? level_high : !level_high;
}

void
SdCardDetectInit(sd_card_detect_t* detect)
{
  if (detect == NULL) {
    return;
  }

  memset(detect, 0, sizeof(*detect));
  detect->gpio = CONFIG_APP_SD_CARD_DETECT_GPIO;
  detect->enabled = (detect->gpio >= 0);
#if CONFIG_APP_SD_CARD_DETECT_PRESENT_HIGH
  detect->present_level_high = true;
#else
  detect->present_level_high = false;
#endif
#if defined(CONFIG_APP_SD_CARD_DETECT_DEBOUNCE_MS)
  detect->debounce_ms = CONFIG_APP_SD_CARD_DETECT_DEBOUNCE_MS;
#else
  detect->debounce_ms = 0;
#endif
  detect->present_debounced = true;
  detect->last_raw_present = true;
  detect->last_raw_change_ticks = xTaskGetTickCount();
  detect->initialized = true;

  if (!detect->enabled) {
    return;
  }

  gpio_config_t config = {
    .pin_bit_mask = (1ULL << detect->gpio),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };

#if CONFIG_APP_SD_CARD_DETECT_USE_INTERNAL_PULL
  if (detect->present_level_high) {
    config.pull_down_en = GPIO_PULLDOWN_ENABLE;
  } else {
    config.pull_up_en = GPIO_PULLUP_ENABLE;
  }
#endif

  (void)gpio_config(&config);

  const bool raw_present = SdCardDetectRawPresent(detect);
  detect->present_debounced = raw_present;
  detect->last_raw_present = raw_present;
  detect->last_raw_change_ticks = xTaskGetTickCount();
}

bool
SdCardDetectPoll(sd_card_detect_t* detect, bool* out_changed)
{
  if (out_changed != NULL) {
    *out_changed = false;
  }
  if (detect == NULL || !detect->initialized || !detect->enabled) {
    return true;
  }

  const TickType_t now_ticks = xTaskGetTickCount();
  const bool raw_present = SdCardDetectRawPresent(detect);
  if (raw_present != detect->last_raw_present) {
    detect->last_raw_present = raw_present;
    detect->last_raw_change_ticks = now_ticks;
  }

  const TickType_t debounce_ticks =
    pdMS_TO_TICKS((TickType_t)detect->debounce_ms);
  if (raw_present != detect->present_debounced) {
    if (detect->debounce_ms == 0 ||
        (now_ticks - detect->last_raw_change_ticks) >= debounce_ticks) {
      detect->present_debounced = raw_present;
      if (out_changed != NULL) {
        *out_changed = true;
      }
    }
  }

  return detect->present_debounced;
}

bool
SdCardDetectIsEnabled(const sd_card_detect_t* detect)
{
  return (detect != NULL) ? detect->enabled : false;
}

bool
SdCardDetectIsPresent(const sd_card_detect_t* detect)
{
  if (detect == NULL || !detect->initialized || !detect->enabled) {
    return true;
  }
  return detect->present_debounced;
}
