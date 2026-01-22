#include "boot_mode.h"
#include "console_commands.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_phase_log.h"
#include "net_supervisor.h"
#include "nvs_flash.h"
#include "run_gpio.h"
#include "runtime_manager.h"
#include "runtime_markers.h"

static const char* kTag = "app";
static const app_runtime_t* g_runtime = NULL;
static const UBaseType_t kAppInitStackWarnThresholdBytes = 1024;

static void
AppInitTask(void* context);
static void
LogAppInitStackWatermark(const char* label);

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
      ESP_LOGE(
        kTag, "nvs_flash_erase failed: %s", esp_err_to_name(erase_result));
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
  // Run init on a dedicated task with an explicit stack size (bytes).

  InitNvs();
  HeapLogPhase("nvs_init");

  static const uint32_t kAppInitStackBytes = 16384; // 16 KB
  static const UBaseType_t kAppInitPriority = 5;

  BaseType_t created = xTaskCreate(
    &AppInitTask, "app_init", kAppInitStackBytes, NULL, kAppInitPriority, NULL);
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
  RuntimeMarkersDumpOnBoot();
  const esp_reset_reason_t reset_reason = esp_reset_reason();
  ESP_LOGI(kTag,
           "Reset reason: %s (%d)",
           RuntimeMarkersResetReasonToString(reset_reason),
           (int)reset_reason);
  HeapLogPhase("startup_begin");

  esp_err_t runtime_result = RuntimeManagerInit();
  if (runtime_result != ESP_OK) {
    ESP_LOGE(
      kTag, "Runtime init reported error: %s", esp_err_to_name(runtime_result));
  }
  HeapLogPhase("runtime_init");

  g_runtime = RuntimeGetRuntime();
  if (g_runtime != NULL) {
    esp_err_t net_result = NetSupervisorStart(g_runtime);
    if (net_result != ESP_OK) {
      ESP_LOGE(
        kTag, "Net supervisor start failed: %s", esp_err_to_name(net_result));
    }
    HeapLogPhase("net_supervisor_start");
    ESP_ERROR_CHECK(ConsoleCommandsStart((app_runtime_t*)g_runtime, boot_mode));
    HeapLogPhase("console_start");
    RunGpioInit();
    HeapLogPhase("run_gpio_init");
  } else {
    ESP_LOGE(kTag, "Runtime unavailable; console not started");
    vTaskDelete(NULL);
    return;
  }

  if (boot_mode == APP_BOOT_MODE_RUN) {
    HeapLogPhase("pre_run_mode");
    LogAppInitStackWatermark("before EnterRunMode");
    esp_err_t start_result = EnterRunMode();
    LogAppInitStackWatermark("after EnterRunMode");
    HeapLogPhase("post_run_mode");
    if (start_result != ESP_OK) {
      ESP_LOGE(
        kTag, "Failed to start runtime: %s", esp_err_to_name(start_result));
    }
  } else {
    (void)EnterDiagMode();
    HeapLogPhase("diag_mode");
    ESP_LOGI(kTag, "Diagnostics mode active (boot default)");
  }

  ESP_LOGI(kTag,
           "Boot complete (boot_mode=%s)",
           (boot_mode == APP_BOOT_MODE_RUN) ? "run" : "diagnostics");

  // Init is complete; nothing else to do in this task.
  vTaskDelete(NULL);
}

static void
LogAppInitStackWatermark(const char* label)
{
  const UBaseType_t watermark_bytes = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGI(kTag,
           "app_init stack watermark %s: %u bytes",
           (label != NULL) ? label : "unknown",
           (unsigned)watermark_bytes);
  if (watermark_bytes < kAppInitStackWarnThresholdBytes) {
    ESP_LOGW(kTag,
             "app_init stack low: %u bytes remaining (< %u bytes)",
             (unsigned)watermark_bytes,
             (unsigned)kAppInitStackWarnThresholdBytes);
  }
}
