#include "wifi_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "net_stack.h"
#include "wifi_manager.h"

static const char* kTag = "wifi_svc";

static bool s_initialized = false;
static wifi_service_mode_t s_active_mode = WIFI_SERVICE_MODE_NONE;
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
WifiServiceStart(wifi_service_mode_t mode)
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

  if (s_active_mode == mode) {
    Unlock();
    return ESP_OK;
  }
  if (s_active_mode != WIFI_SERVICE_MODE_NONE) {
    Unlock();
    ESP_LOGW(kTag, "service already active (mode=%d)", (int)s_active_mode);
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = ESP_OK;
  switch (mode) {
    case WIFI_SERVICE_MODE_DIAGNOSTIC_STA:
      result = WifiManagerInit();
      break;
    case WIFI_SERVICE_MODE_MESH:
      result = ESP_OK;
      break;
    default:
      result = ESP_ERR_INVALID_ARG;
      break;
  }

  if (result == ESP_OK) {
    s_active_mode = mode;
  }
  Unlock();
  return result;
}

esp_err_t
WifiServiceStop(void)
{
  if (!s_initialized) {
    return ESP_OK;
  }

  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  esp_err_t result = ESP_OK;
  switch (s_active_mode) {
    case WIFI_SERVICE_MODE_DIAGNOSTIC_STA:
      result = WifiManagerStop();
      break;
    case WIFI_SERVICE_MODE_MESH:
    case WIFI_SERVICE_MODE_NONE:
      result = ESP_OK;
      break;
    default:
      result = ESP_ERR_INVALID_STATE;
      break;
  }

  if (result == ESP_OK) {
    s_active_mode = WIFI_SERVICE_MODE_NONE;
  }

  Unlock();
  return result;
}

wifi_service_mode_t
WifiServiceActiveMode(void)
{
  return s_active_mode;
}
