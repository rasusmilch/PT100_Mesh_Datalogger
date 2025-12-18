#include "boot_mode.h"
#include "console_commands.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "runtime_manager.h"

static const char* kTag = "app";

static void
InitNvs(void)
{
  esp_err_t result = nvs_flash_init();
  if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
      result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(kTag, "NVS partition full or version mismatch; erasing");
    esp_err_t erase_result = nvs_flash_erase();
    if (erase_result != ESP_OK) {
      ESP_LOGE(kTag, "nvs_flash_erase failed: %s", esp_err_to_name(erase_result));
      return;
    }
    result = nvs_flash_init();
  }
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "nvs_flash_init failed: %s", esp_err_to_name(result));
  }
}

void
app_main(void)
{
  InitNvs();

  const app_boot_mode_t boot_mode = BootModeDetermineAtStartup();

  esp_err_t runtime_result = RuntimeManagerInit();
  if (runtime_result != ESP_OK) {
    ESP_LOGE(kTag, "Runtime init reported error: %s", esp_err_to_name(runtime_result));
  }

  const app_runtime_t* runtime = RuntimeGetRuntime();
  if (runtime != NULL) {
    ESP_ERROR_CHECK(ConsoleCommandsStart((app_runtime_t*)runtime));
  } else {
    ESP_LOGE(kTag, "Runtime unavailable; console not started");
    return;
  }

  if (boot_mode == APP_BOOT_MODE_RUN) {
    esp_err_t start_result = RuntimeStart();
    if (start_result != ESP_OK) {
      ESP_LOGE(kTag, "Failed to start runtime: %s", esp_err_to_name(start_result));
    }
  } else {
    ESP_LOGI(kTag, "Diagnostics mode active (boot default)");
  }

  ESP_LOGI(kTag, "Boot complete (boot_mode=%s)",
           (boot_mode == APP_BOOT_MODE_RUN) ? "run" : "diagnostics");
}
