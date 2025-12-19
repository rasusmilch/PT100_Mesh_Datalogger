#include "wifi_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "net_stack.h"
#include "wifi_manager.h"

static const char* kTag = "wifi_svc";

static bool s_initialized = false;
static wifi_service_mode_t s_active_mode = WIFI_SERVICE_MODE_NONE;
static int s_refcount = 0;
static bool s_wifi_initialized = false;
static bool s_wifi_started = false;
static SemaphoreHandle_t s_mutex = NULL;

static esp_err_t
EnsureMutex(void)
{
  if (s_mutex == NULL) {
    s_mutex = xSemaphoreCreateMutex();
  }
  return (s_mutex != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

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

static void
Unlock(void)
{
  if (s_mutex != NULL) {
    xSemaphoreGive(s_mutex);
  }
}

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
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t init_result = esp_wifi_init(&wifi_config);
    if (init_result == ESP_ERR_WIFI_INIT_STATE ||
        init_result == ESP_ERR_INVALID_STATE) {
      init_result = ESP_OK;
    }
    if (init_result != ESP_OK) {
      Unlock();
      ESP_LOGE(kTag, "esp_wifi_init failed: %s", esp_err_to_name(init_result));
      return init_result;
    }
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
    if (mode == WIFI_SERVICE_MODE_DIAGNOSTIC_STA) {
      WifiManagerNotifyWifiStarted();
    }
  }

  ++s_refcount;
  Unlock();
  return ESP_OK;
}

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

wifi_service_mode_t
WifiServiceActiveMode(void)
{
  return s_active_mode;
}
