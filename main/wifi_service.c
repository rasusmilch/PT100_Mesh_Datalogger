#include "wifi_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_heap_caps.h"
#include "esp_heap_trace.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "heap_phase_log.h"
#include "log_rate_limit.h"
#include "net_stack.h"
#include "wifi_manager.h"

static const char* kTag = "wifi_svc";
static const uint32_t kWifiHeapLogRateLimitMs = 5000;

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

// If the largest DMA-capable internal heap block is only slightly below the
// configured threshold, allow a best-effort attempt at esp_wifi_init() anyway.
// This helps with edge cases where the pre-check is more conservative than the
// actual Wi-Fi init path.
static const size_t kWifiInitLargestDmaSlackBytes = 2048u;

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

#ifndef CONFIG_APP_WIFI_HEAP_TRACE_ENABLE
#define CONFIG_APP_WIFI_HEAP_TRACE_ENABLE 0
#endif

#ifndef CONFIG_APP_WIFI_HEAP_TRACE_ENABLE
#define CONFIG_APP_WIFI_HEAP_TRACE_ENABLE 0
#endif

// When APP_WIFI_HEAP_TRACE_ENABLE is disabled, dependent Kconfig symbols may
// not be emitted into sdkconfig.h. Provide safe defaults so this file compiles
// in all configurations.
#ifndef CONFIG_APP_WIFI_HEAP_TRACE_LARGEST_INTERNAL_THRESHOLD_BYTES
#define CONFIG_APP_WIFI_HEAP_TRACE_LARGEST_INTERNAL_THRESHOLD_BYTES 0
#endif

static esp_err_t
EnsureMutex(void);
static esp_err_t
Lock(TickType_t timeout);
static void
Unlock(void);
static void
WifiServiceLogHeap(const char* phase);
static void
WifiServiceHeapTraceStart(const char* reason);
static void
WifiServiceHeapTraceStop(bool dump, const char* reason);
static bool
WifiHasSufficientHeapForWifiInit_(bool log_as_warning,
                                  size_t free_internal,
                                  size_t largest_internal,
                                  size_t free_dma_internal,
                                  size_t largest_dma_internal,
                                  bool* dma_largest_borderline);
static esp_err_t
WifiServiceEnsureDriverInitializedLocked_(bool log_failure_as_warning);

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
  const size_t free_dma_internal =
    heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  const size_t largest_dma_internal =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  ESP_LOGI(
    kTag,
    "%s heap (internal): free=%u min=%u largest=%u dma_free=%u dma_largest=%u",
    phase,
    (unsigned)free_internal,
    (unsigned)min_internal,
    (unsigned)largest_internal,
    (unsigned)free_dma_internal,
    (unsigned)largest_dma_internal);
}

#if CONFIG_APP_WIFI_HEAP_TRACE_ENABLE && CONFIG_HEAP_TRACING
static bool s_wifi_heap_trace_initialized = false;
static bool s_wifi_heap_trace_active = false;
static heap_trace_record_t
  s_wifi_heap_trace_records[CONFIG_APP_WIFI_HEAP_TRACE_RECORDS];

static void
WifiServiceHeapTraceStart(const char* reason)
{
  if (!s_wifi_heap_trace_initialized) {
    esp_err_t init_result = heap_trace_init_standalone(
      s_wifi_heap_trace_records, CONFIG_APP_WIFI_HEAP_TRACE_RECORDS);
    if (init_result != ESP_OK) {
      ESP_LOGW(
        kTag, "heap trace init failed: %s", esp_err_to_name(init_result));
      return;
    }
    s_wifi_heap_trace_initialized = true;
  }
  if (s_wifi_heap_trace_active) {
    heap_trace_stop();
    s_wifi_heap_trace_active = false;
  }
  esp_err_t start_result = heap_trace_start(HEAP_TRACE_LEAKS);
  if (start_result != ESP_OK) {
    ESP_LOGW(
      kTag, "heap trace start failed: %s", esp_err_to_name(start_result));
    return;
  }
  s_wifi_heap_trace_active = true;
  if (reason != NULL) {
    ESP_LOGI(kTag, "heap trace start: %s", reason);
  }
}

static void
WifiServiceHeapTraceStop(bool dump, const char* reason)
{
  if (!s_wifi_heap_trace_active) {
    return;
  }
  heap_trace_stop();
  s_wifi_heap_trace_active = false;
  if (dump) {
    if (reason != NULL) {
      ESP_LOGW(kTag, "heap trace dump: %s", reason);
    } else {
      ESP_LOGW(kTag, "heap trace dump");
    }
    heap_trace_dump();
  }
}
#else
static void
WifiServiceHeapTraceStart(const char* reason)
{
  (void)reason;
}

static void
WifiServiceHeapTraceStop(bool dump, const char* reason)
{
  (void)dump;
  (void)reason;
}
#endif

/**
 * @brief Determine whether heap is sufficient for Wi-Fi init.
 * @param log_as_warning Parameter log_as_warning.
 * @param free_internal Parameter free_internal.
 * @param largest_internal Parameter largest_internal.
 * @param free_dma_internal Parameter free_dma_internal.
 * @param largest_dma_internal Parameter largest_dma_internal.
 * @param dma_largest_borderline Parameter dma_largest_borderline.
 * @return Return the function result.
 */
static bool
WifiHasSufficientHeapForWifiInit_(bool log_as_warning,
                                  size_t free_internal,
                                  size_t largest_internal,
                                  size_t free_dma_internal,
                                  size_t largest_dma_internal,
                                  bool* dma_largest_borderline)
{
  const size_t min_free_internal =
    (size_t)CONFIG_APP_WIFI_INIT_MIN_FREE_INTERNAL_BYTES;
  const size_t min_dma_largest =
    (size_t)CONFIG_APP_WIFI_INIT_MIN_LARGEST_DMA_INTERNAL_BYTES;

  const bool is_borderline =
    (largest_dma_internal < min_dma_largest) &&
    (largest_dma_internal + kWifiInitLargestDmaSlackBytes >= min_dma_largest);

  if (dma_largest_borderline != NULL) {
    *dma_largest_borderline = is_borderline;
  }

  if (free_internal < min_free_internal ||
      (!is_borderline && largest_dma_internal < min_dma_largest)) {
    if (log_as_warning) {
      ESP_LOGW(kTag,
               "insufficient heap for Wi-Fi init (free=%u, largest=%u, "
               "dma_free=%u, dma_largest=%u) thresholds (min_free=%u, "
               "min_dma_largest=%u)",
               (unsigned)free_internal,
               (unsigned)largest_internal,
               (unsigned)free_dma_internal,
               (unsigned)largest_dma_internal,
               (unsigned)min_free_internal,
               (unsigned)min_dma_largest);
    } else {
      ESP_LOGE(kTag,
               "insufficient heap for Wi-Fi init (free=%u, largest=%u, "
               "dma_free=%u, dma_largest=%u) thresholds (min_free=%u, "
               "min_dma_largest=%u)",
               (unsigned)free_internal,
               (unsigned)largest_internal,
               (unsigned)free_dma_internal,
               (unsigned)largest_dma_internal,
               (unsigned)min_free_internal,
               (unsigned)min_dma_largest);
    }
    return false;
  }

  if (is_borderline) {
    ESP_LOGW(kTag,
             "Wi-Fi init DMA largest block below threshold but within slack "
             "(dma_largest=%u min=%u slack=%u); attempting init anyway",
             (unsigned)largest_dma_internal,
             (unsigned)min_dma_largest,
             (unsigned)kWifiInitLargestDmaSlackBytes);
  }

  return true;
}

/**
 * @brief Ensure the Wi-Fi driver is initialized (mutex held).
 * @param log_failure_as_warning Parameter log_failure_as_warning.
 * @return Return the function result.
 */
static esp_err_t
WifiServiceEnsureDriverInitializedLocked_(bool log_failure_as_warning)
{
  if (s_wifi_initialized) {
    return ESP_OK;
  }

  const size_t free_internal_pre_init =
    heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  const size_t largest_internal_pre_init =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  const size_t free_dma_internal_pre_init =
    heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  const size_t largest_dma_internal_pre_init =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

  WifiServiceLogHeap("pre-init");

  if (!WifiHasSufficientHeapForWifiInit_(log_failure_as_warning,
                                         free_internal_pre_init,
                                         largest_internal_pre_init,
                                         free_dma_internal_pre_init,
                                         largest_dma_internal_pre_init,
                                         NULL)) {
    WifiServiceHeapTraceStop(true /* log */, "wifi init heap precheck failed");
    return ESP_ERR_NO_MEM;
  }

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  HeapLogPhase("wifi_init_before");
  esp_err_t init_result = esp_wifi_init(&wifi_config);
  HeapLogPhase("wifi_init_after");
  if (init_result == ESP_ERR_WIFI_INIT_STATE ||
      init_result == ESP_ERR_INVALID_STATE) {
    init_result = ESP_OK;
  }
  if (init_result != ESP_OK) {
    ESP_LOGE(kTag, "esp_wifi_init failed: %s", esp_err_to_name(init_result));
    WifiServiceHeapTraceStop(true, "wifi_init_failed");
    const size_t free_internal_post_fail =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t min_internal_post_fail =
      heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    const size_t largest_internal_post_fail =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    const size_t free_dma_internal_post_fail =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t largest_dma_internal_post_fail =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_LOGE(kTag,
             "wifi init heap after failure (internal): free=%u min=%u "
             "largest=%u dma_free=%u dma_largest=%u",
             (unsigned)free_internal_post_fail,
             (unsigned)min_internal_post_fail,
             (unsigned)largest_internal_post_fail,
             (unsigned)free_dma_internal_post_fail,
             (unsigned)largest_dma_internal_post_fail);
    return init_result;
  }

  WifiServiceLogHeap("post-init");
  s_wifi_initialized = true;
  return ESP_OK;
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
  const bool should_trace_window = (s_refcount == 0 && !s_wifi_started);

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

  if (should_trace_window) {
    WifiServiceHeapTraceStart("wifi_service_start");
  }

  if (!s_wifi_initialized) {
    esp_err_t ensure_result =
      WifiServiceEnsureDriverInitializedLocked_(false);
    if (ensure_result != ESP_OK) {
      Unlock();
      return ensure_result;
    }
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
          HeapLogPhase("wifi_set_mode_before");
          mode_result = esp_wifi_set_mode(WIFI_MODE_APSTA);
          HeapLogPhase("wifi_set_mode_after");
        }
        break;
      default:
        mode_result = ESP_ERR_INVALID_ARG;
        break;
    }

    if (mode_result != ESP_OK) {
      WifiServiceHeapTraceStop(false, "wifi_start_skipped");
      Unlock();
      return mode_result;
    }

    const size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t largest_internal =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    WifiServiceLogHeap("pre-start");

    if (free_internal < kMinInternalFreeForWifiStartBytes ||
        largest_internal < kMinInternalLargestForWifiStartBytes) {
      ESP_LOGE(
        kTag,
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
      WifiServiceHeapTraceStop(true, "wifi_start_precheck_failed");
      Unlock();
      return ESP_ERR_NO_MEM;
    }

    HeapLogPhase("wifi_start_before");
    esp_err_t start_result = esp_wifi_start();
    HeapLogPhase("wifi_start_after");
    if (start_result == ESP_ERR_WIFI_CONN ||
        start_result == ESP_ERR_WIFI_STATE ||
        start_result == ESP_ERR_INVALID_STATE) {
      start_result = ESP_OK;
    }
    if (start_result != ESP_OK) {
      Unlock();
      ESP_LOGE(
        kTag, "esp_wifi_start failed: %s", esp_err_to_name(start_result));
      WifiServiceHeapTraceStop(true, "wifi_start_failed");
      return start_result;
    }

    s_wifi_started = true;
    s_active_mode = mode;
    const size_t largest_internal_after_start =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    WifiServiceLogHeap("post-start");
    const int largest_internal_threshold_bytes =
      CONFIG_APP_WIFI_HEAP_TRACE_LARGEST_INTERNAL_THRESHOLD_BYTES;
    if ((largest_internal_threshold_bytes > 0) &&
        (largest_internal_after_start <
         (size_t)largest_internal_threshold_bytes)) {
      WifiServiceHeapTraceStop(true, "wifi_start_largest_internal_drop");
    } else {
      WifiServiceHeapTraceStop(false, "wifi_start_complete");
    }
    if (mode == WIFI_SERVICE_MODE_DIAGNOSTIC_STA) {
      WifiManagerNotifyWifiStarted();
    }
  }

  ++s_refcount;
  Unlock();
  return ESP_OK;
}

/**
 * @brief Execute WifiServiceReserveEarly.
 * @return Return the function result.
 */
esp_err_t
WifiServiceReserveEarly(void)
{
  esp_err_t init_result = WifiServiceInitOnce();
  if (init_result != ESP_OK) {
    return init_result;
  }

  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  esp_err_t result = ESP_OK;
  if (!s_wifi_initialized) {
    result = WifiServiceEnsureDriverInitializedLocked_(true);
    if (result == ESP_OK) {
      result = WifiManagerInit();
      if (result != ESP_OK) {
        ESP_LOGE(
          kTag, "WifiManagerInit failed: %s", esp_err_to_name(result));
      }
    }
  }

  Unlock();
  return result;
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
