#include "boot_mode.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>

static const char* kTag = "boot_mode";
static const char* kNvsNamespace = "app";
static const char* kBootModeKey = "boot_mode";

static app_boot_mode_t
ValidateOrDefault(uint8_t raw)
{
  return (raw == (uint8_t)APP_BOOT_MODE_RUN) ? APP_BOOT_MODE_RUN
                                            : APP_BOOT_MODE_DIAGNOSTICS;
}

app_boot_mode_t
BootModeReadFromNvsOrDefault(void)
{
  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
  if (result != ESP_OK) {
    if (result != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGW(kTag, "nvs_open failed: %s", esp_err_to_name(result));
    }
    return APP_BOOT_MODE_DIAGNOSTICS;
  }

  uint8_t raw_mode = 0;
  result = nvs_get_u8(handle, kBootModeKey, &raw_mode);
  nvs_close(handle);
  if (result == ESP_OK) {
    return ValidateOrDefault(raw_mode);
  }
  if (result != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(kTag, "nvs_get_u8 failed: %s", esp_err_to_name(result));
  }
  return APP_BOOT_MODE_DIAGNOSTICS;
}

esp_err_t
BootModeWriteToNvs(app_boot_mode_t mode)
{
  const uint8_t raw_mode = (uint8_t)ValidateOrDefault((uint8_t)mode);
  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kBootModeKey, raw_mode);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

static bool
DiagnosticsOverrideRequested(void)
{
#if CONFIG_APP_DIAGNOSTICS_OVERRIDE_GPIO >= 0
  const gpio_num_t gpio = (gpio_num_t)CONFIG_APP_DIAGNOSTICS_OVERRIDE_GPIO;

  gpio_config_t config = {
    .pin_bit_mask = 1ULL << gpio,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  (void)gpio_config(&config);

  return gpio_get_level(gpio) == 1;
#else
  return false;
#endif
}

app_boot_mode_t
BootModeDetermineAtStartup(void)
{
  app_boot_mode_t mode = BootModeReadFromNvsOrDefault();

  if (DiagnosticsOverrideRequested()) {
    ESP_LOGW(kTag, "BOOT override: diagnostics forced");
    printf("BOOT override: diagnostics forced\n");
    return APP_BOOT_MODE_DIAGNOSTICS;
  }

  return mode;
}
