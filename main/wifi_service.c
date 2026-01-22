#include "wifi_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "log_rate_limit.h"
#include "net_stack.h"
#include "wifi_manager.h"

static const char* kTag = "wifi_svc";
static const uint32_t kWifiHeapLogRateLimitMs = 5000;
static const size_t kWifiInitBudgetInternalBytes = 130u * 1024u;
static const size_t kMinInternalLargestForWifiInitBytes = 4096u;

// Wi-Fi/COEX uses esp_timer and other internal-only allocations. When internal
// heap is extremely low/fragmented, esp_wifi_start() may abort inside IDF
// (ESP_ERROR_CHECK) before returning an error to the application.
//
// To keep "run start" fail-safe (logging continues even when Wi-Fi cannot be
// started), do a conservative preflight check and fail gracefully.
// Wi-Fi start allocates additional internal heap (driver/task plumbing). If we
// start too close to the edge, we can fail later in hard-to-debug ways.
static const size_t kMinInternalFreeForWifiStartBytes = 16u * 1024u;
static const size_t kMinInternalLargestForWifiStartBytes = 2048u;

static bool s_initialized = false;
static wifi_service_mode_t s_active_mode = WIFI_SERVICE_MODE_NONE;
static int s_refcount = 0;
static bool s_wifi_initialized = false;
static bool s_wifi_started = false;
static SemaphoreHandle_t s_mutex = NULL;
static uint32_t s_wifi_heap_log_ms = 0;

#ifndef CONFIG_APP_WIFI_HEAP_DEBUG
#define CONFIG_APP_WIFI_HEAP_DEBUG 0
#endif

static esp_err_t EnsureMutex(void);
static esp_err_t Lock(TickType_t timeout);
static void Unlock(void);
static void WifiServiceLogHeap(const char* phase);

/**
 * @brief Execute WifiServiceLogHeap.
 * @param phase Parameter phase.
 */
static void
WifiServiceLogHeap(const char* phase)
{
  if (!CONFIG_APP_WIFI_HEAP_DEBUG || phase == NULL) {
    return;
  }
  if (!LogRateLimitAllow(&s_wifi_heap_log_ms, kWifiHeapLogRateLimitMs)) {
    return;
  }
  const size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  const size_t min_internal =
    heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  const size_t largest_internal =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  ESP_LOGI(kTag,
           "%s heap (internal): free=%u min=%u largest=%u",
           phase,
           (unsigned)free_internal,
           (unsigned)min_internal,
           (unsigned)largest_internal);
}

/**
 * @brief Execute EnsureMutex.
 * @return Return the function result.
 */
static esp_err_t
EnsureMutex(void)
{
  if (s_mutex == NULL) {
    s_mutex = xSemaphoreCreateMutex();
  }
  return (s_mutex != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

/**
 * @brief Execute Lock.
 * @param timeout Parameter timeout.
 * @return Return the function result.
 */
static esp_err_t
Lock(TickType_t timeout)
{
  esp_err_t result = EnsureMutex();
  if (result != ESP_OK) {
    return result;
  }
  if (xSemaphoreTake(s_mutex, timeout) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

/**
 * @brief Execute Unlock.
 */
static void
Unlock(void)
{
  if (s_mutex != NULL) {
    xSemaphoreGive(s_mutex);
  }
}

/**
 * @brief Execute WifiServiceInitOnce.
 * @return Return the function result.
 */
esp_err_t
WifiServiceInitOnce(void)
{
  esp_err_t net_result = NetStackInitOnce();
  if (net_result != ESP_OK) {
    return net_result;
  }

  if (!s_initialized) {
    esp_err_t mutex_result = EnsureMutex();
    if (mutex_result != ESP_OK) {
      return mutex_result;
    }
    s_initialized = true;
  }
  return ESP_OK;
}

/**
 * @brief Execute WifiServiceAcquire.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
esp_err_t
WifiServiceAcquire(wifi_service_mode_t mode)
{
  if (mode == WIFI_SERVICE_MODE_NONE) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t init_result = WifiServiceInitOnce();
  if (init_result != ESP_OK) {
    return init_result;
  }

  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  if (s_active_mode != WIFI_SERVICE_MODE_NONE && s_active_mode != mode) {
    Unlock();
    ESP_LOGW(kTag, "service already active (mode=%d)", (int)s_active_mode);
    return ESP_ERR_INVALID_STATE;
  }

  if (!s_wifi_initialized) {
    const size_t free_internal_pre_init =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t largest_internal_pre_init =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    WifiServiceLogHeap("pre-init");

    if (free_internal_pre_init < kWifiInitBudgetInternalBytes ||
        largest_internal_pre_init < kMinInternalLargestForWifiInitBytes) {
      ESP_LOGE(kTag,
               "insufficient internal heap for Wi-Fi init (free=%u, largest=%u)",
               (unsigned)free_internal_pre_init,
               (unsigned)largest_internal_pre_init);
      Unlock();
      return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t init_result = esp_wifi_init(&wifi_config);
    if (init_result == ESP_ERR_WIFI_INIT_STATE ||
        init_result == ESP_ERR_INVALID_STATE) {
      init_result = ESP_OK;
    }
    if (init_result != ESP_OK) {
      Unlock();
      ESP_LOGE(kTag, "esp_wifi_init failed: %s", esp_err_to_name(init_result));
      const size_t free_internal_post_fail =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      const size_t min_internal_post_fail =
        heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
      const size_t largest_internal_post_fail =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
      ESP_LOGE(kTag,
               "wifi init heap after failure (internal): free=%u min=%u largest=%u",
               (unsigned)free_internal_post_fail,
               (unsigned)min_internal_post_fail,
               (unsigned)largest_internal_post_fail);
      return init_result;
    }
    WifiServiceLogHeap("post-init");
    s_wifi_initialized = true;
  }

  if (s_refcount == 0 && !s_wifi_started) {
    esp_err_t mode_result = ESP_OK;
    switch (mode) {
      case WIFI_SERVICE_MODE_DIAGNOSTIC_STA:
        mode_result = WifiManagerInit();
        break;
      case WIFI_SERVICE_MODE_MESH:
        mode_result = esp_wifi_set_storage(WIFI_STORAGE_RAM);
        if (mode_result == ESP_ERR_WIFI_NOT_INIT) {
          mode_result = ESP_ERR_INVALID_STATE;
        }
        if (mode_result == ESP_OK) {
          mode_result = esp_wifi_set_mode(WIFI_MODE_APSTA);
        }
        break;
      default:
        mode_result = ESP_ERR_INVALID_ARG;
        break;
    }

    if (mode_result != ESP_OK) {
      Unlock();
      return mode_result;
    }

    const size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t largest_internal =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    WifiServiceLogHeap("pre-start");

    if (free_internal < kMinInternalFreeForWifiStartBytes ||
        largest_internal < kMinInternalLargestForWifiStartBytes) {
      ESP_LOGE(kTag,
               "insufficient internal heap for Wi-Fi start (free=%u, largest=%u)",
               (unsigned)free_internal,
               (unsigned)largest_internal);

      // Unwind driver allocations so the rest of the system can continue.
      // Without this, a failed Wi-Fi start can strand the system in a low-heap
      // state, causing unrelated task/queue creation to fail.
      if (mode == WIFI_SERVICE_MODE_DIAGNOSTIC_STA) {
        (void)WifiManagerStop();
      }
      if (s_wifi_initialized) {
        (void)esp_wifi_deinit();
        s_wifi_initialized = false;
      }
      s_wifi_started = false;
      s_active_mode = WIFI_SERVICE_MODE_NONE;
      Unlock();
      return ESP_ERR_NO_MEM;
    }

    esp_err_t start_result = esp_wifi_start();
    if (start_result == ESP_ERR_WIFI_CONN || start_result == ESP_ERR_WIFI_STATE ||
        start_result == ESP_ERR_INVALID_STATE) {
      start_result = ESP_OK;
    }
    if (start_result != ESP_OK) {
      Unlock();
      ESP_LOGE(kTag, "esp_wifi_start failed: %s", esp_err_to_name(start_result));
      return start_result;
    }

    s_wifi_started = true;
    s_active_mode = mode;
    WifiServiceLogHeap("post-start");
    if (mode == WIFI_SERVICE_MODE_DIAGNOSTIC_STA) {
      WifiManagerNotifyWifiStarted();
    }
  }

  ++s_refcount;
  Unlock();
  return ESP_OK;
}

/**
 * @brief Execute WifiServiceRelease.
 * @return Return the function result.
 */
esp_err_t
WifiServiceRelease(void)
{
  if (!s_initialized) {
    return ESP_OK;
  }

  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  if (s_refcount > 0) {
    --s_refcount;
  }

  if (s_refcount > 0 || !s_wifi_started) {
    Unlock();
    return ESP_OK;
  }

  esp_err_t result = ESP_OK;
  if (s_active_mode == WIFI_SERVICE_MODE_DIAGNOSTIC_STA) {
    result = WifiManagerStop();
  }

  esp_err_t stop_result = esp_wifi_stop();
  if (stop_result == ESP_ERR_WIFI_NOT_INIT ||
      stop_result == ESP_ERR_WIFI_NOT_STARTED) {
    stop_result = ESP_OK;
  }

  s_wifi_started = false;
  s_active_mode = WIFI_SERVICE_MODE_NONE;

  Unlock();
  return (result == ESP_OK) ? stop_result : result;
}

/**
 * @brief Execute WifiServiceActiveMode.
 * @return Return the function result.
 */
wifi_service_mode_t
WifiServiceActiveMode(void)
{
  return s_active_mode;
}
