#include "boot_mode.h"
#include "console_commands.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "run_gpio.h"
#include "runtime_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* kTag = "app";
static const app_runtime_t* g_runtime = NULL;

static void AppInitTask(void* context);

/**
 * @brief Execute InitNvs.
 */
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

/**
 * @brief Execute app_main.
 */
void
app_main(void)
{
  // Keep the ESP-IDF "main" task lightweight. Initialization can be stack-heavy
  // (console, SD/FAT, Wi-Fi), and the default main task stack can be small.
  // Run init on a dedicated task with an explicit stack size.
  InitNvs();

  static const uint32_t kAppInitStackWords = 4096;  // 16 KB
  static const UBaseType_t kAppInitPriority = 5;

  BaseType_t created = xTaskCreate(&AppInitTask,
                                  "app_init",
                                  kAppInitStackWords,
                                  NULL,
                                  kAppInitPriority,
                                  NULL);
  if (created != pdPASS) {
    ESP_LOGE(kTag, "Failed to create app_init task");
  }
  // Return so the ESP-IDF main task can delete itself.
}

/**
 * @brief Execute AppInitTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the AppInitTask task.
 */
static void
AppInitTask(void* context)
{
  (void)context;

  const app_boot_mode_t boot_mode = BootModeDetermineAtStartup();

  esp_err_t runtime_result = RuntimeManagerInit();
  if (runtime_result != ESP_OK) {
    ESP_LOGE(kTag,
             "Runtime init reported error: %s",
             esp_err_to_name(runtime_result));
  }

  g_runtime = RuntimeGetRuntime();
  if (g_runtime != NULL) {
    ESP_ERROR_CHECK(ConsoleCommandsStart((app_runtime_t*)g_runtime, boot_mode));
    RunGpioInit();
  } else {
    ESP_LOGE(kTag, "Runtime unavailable; console not started");
    vTaskDelete(NULL);
    return;
  }

  if (boot_mode == APP_BOOT_MODE_RUN) {
    esp_err_t start_result = EnterRunMode();
    if (start_result != ESP_OK) {
      ESP_LOGE(kTag, "Failed to start runtime: %s", esp_err_to_name(start_result));
    }
  } else {
    (void)EnterDiagMode();
    ESP_LOGI(kTag, "Diagnostics mode active (boot default)");
  }

  ESP_LOGI(kTag,
           "Boot complete (boot_mode=%s)",
           (boot_mode == APP_BOOT_MODE_RUN) ? "run" : "diagnostics");

  // Init is complete; nothing else to do in this task.
  vTaskDelete(NULL);
}
