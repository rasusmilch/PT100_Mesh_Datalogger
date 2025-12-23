#include "wifi_manager.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char* kTag = "wifi_mgr";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_SCAN_DONE_BIT BIT2

static EventGroupHandle_t s_event_group = NULL;
static esp_netif_t* s_sta_netif = NULL;
static esp_event_handler_instance_t s_wifi_handler = NULL;
static esp_event_handler_instance_t s_ip_handler = NULL;
static bool s_wifi_initialized = false;
static bool s_wifi_started = false;
static bool s_wifi_connected = false;
static bool s_owns_sta_netif = false;
static bool s_wifi_handler_registered = false;
static bool s_ip_handler_registered = false;
static bool s_started_by_manager = false;
static wifi_err_reason_t s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
static int s_last_connect_attempts = 0;
static SemaphoreHandle_t s_mutex = NULL;

static esp_err_t
CleanupLocked(bool release_resources);

static void
WifiEventHandler(void* arg,
                 esp_event_base_t event_base,
                 int32_t event_id,
                 void* event_data)
{
  (void)arg;
  (void)event_base;

  if (s_event_group == NULL) {
    return;
  }

  switch (event_id) {
    case WIFI_EVENT_STA_START:
      xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
      s_wifi_started = true;
      break;

    case WIFI_EVENT_STA_DISCONNECTED: {
      wifi_event_sta_disconnected_t* info =
        (wifi_event_sta_disconnected_t*)event_data;
      s_wifi_connected = false;
      s_last_disconnect_reason =
        (info != NULL) ? info->reason : WIFI_REASON_UNSPECIFIED;
      xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT);
      xEventGroupSetBits(s_event_group, WIFI_FAIL_BIT);
      break;
    }

    case WIFI_EVENT_SCAN_DONE:
      xEventGroupSetBits(s_event_group, WIFI_SCAN_DONE_BIT);
      break;

    default:
      break;
  }
}

static void
IpEventHandler(void* arg,
               esp_event_base_t event_base,
               int32_t event_id,
               void* event_data)
{
  (void)arg;
  (void)event_data;

  if (s_event_group == NULL) {
    return;
  }

  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    s_wifi_connected = true;
    s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
    xEventGroupClearBits(s_event_group, WIFI_FAIL_BIT);
    xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
  }
}

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

static esp_err_t
EnsureEventGroup(void)
{
  if (s_event_group == NULL) {
    s_event_group = xEventGroupCreate();
  }
  return (s_event_group != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

// Optional thin wrappers to make call sites self-documenting.
static esp_err_t
CleanupLocked(bool release_resources);

static esp_err_t
CleanupKeepResourcesLocked(void)
{
  return CleanupLocked(/*release_resources=*/false);
}

static esp_err_t
CleanupReleaseResourcesLocked(void)
{
  return CleanupLocked(/*release_resources=*/true);
}

static esp_err_t
CleanupLocked(bool release_resources)
{
  esp_err_t result = ESP_OK;

  s_wifi_connected = false;
  s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;

  // Stop/disconnect only if we started Wi-Fi in this module.
  if (s_wifi_started && s_started_by_manager) {
    // Stop any in-flight scans to avoid scan done callbacks arriving during
    // teardown/reconfiguration.
    (void)esp_wifi_scan_stop();

    esp_err_t disconnect_result = esp_wifi_disconnect();
    if (disconnect_result != ESP_OK &&
        disconnect_result != ESP_ERR_WIFI_NOT_INIT &&
        disconnect_result != ESP_ERR_WIFI_NOT_STARTED &&
        disconnect_result != ESP_ERR_WIFI_NOT_CONNECT && result == ESP_OK) {
      result = disconnect_result;
    }

    s_wifi_started = false;
    s_started_by_manager = false;
  }

  // Unregister handlers only when releasing resources.
  if (release_resources && s_wifi_handler_registered &&
      s_wifi_handler != NULL) {
    esp_err_t unregister_result = esp_event_handler_instance_unregister(
      WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_handler);
    if (result == ESP_OK && unregister_result != ESP_OK) {
      result = unregister_result;
    }
  }
  if (release_resources) {
    s_wifi_handler_registered = false;
    s_wifi_handler = NULL;
  }

  if (release_resources && s_ip_handler_registered && s_ip_handler != NULL) {
    esp_err_t unregister_result = esp_event_handler_instance_unregister(
      IP_EVENT, IP_EVENT_STA_GOT_IP, s_ip_handler);
    if (result == ESP_OK && unregister_result != ESP_OK) {
      result = unregister_result;
    }
  }
  if (release_resources) {
    s_ip_handler_registered = false;
    s_ip_handler = NULL;
  }

  // Destroy netif only if we created it AND caller asked to release resources.
  if (release_resources && s_owns_sta_netif && s_sta_netif != NULL) {
    esp_netif_destroy(s_sta_netif);
    s_sta_netif = NULL;
    s_owns_sta_netif = false;
  }

  return result;
}

esp_err_t
WifiManagerInit(void)
{
  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  if (s_wifi_started) {
    Unlock();
    return ESP_OK;
  }

  esp_err_t result = EnsureEventGroup();
  if (result != ESP_OK) {
    Unlock();
    return result;
  }

  if (s_sta_netif == NULL) {
    esp_netif_t* existing = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (existing != NULL) {
      s_sta_netif = existing;
      s_owns_sta_netif = false;
    } else {
      s_sta_netif = esp_netif_create_default_wifi_sta();
      if (s_sta_netif == NULL) {
        ESP_LOGE(kTag, "failed to create default Wi-Fi STA netif");
        Unlock();
        return ESP_ERR_NO_MEM;
      }
      s_owns_sta_netif = true;
    }
  }

  if (!s_wifi_initialized) {
    s_wifi_initialized = true;
  }
  s_started_by_manager = true;

  if (!s_wifi_handler_registered) {
    result = esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiEventHandler, NULL, &s_wifi_handler);
    if (result == ESP_OK) {
      s_wifi_handler_registered = true;

    } else {
      ESP_LOGE(
        kTag, "wifi handler register failed: %s", esp_err_to_name(result));
      CleanupReleaseResourcesLocked();
      Unlock();
      return result;
    }
  }

  if (!s_ip_handler_registered) {
    result = esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &IpEventHandler, NULL, &s_ip_handler);
    if (result == ESP_OK) {
      s_ip_handler_registered = true;
    } else {
      ESP_LOGE(kTag, "ip handler register failed: %s", esp_err_to_name(result));
      CleanupReleaseResourcesLocked();
      Unlock();
      return result;
    }
  }

  result = esp_wifi_set_mode(WIFI_MODE_STA);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "esp_wifi_set_mode failed: %s", esp_err_to_name(result));
    CleanupReleaseResourcesLocked();
    Unlock();
    return result;
  }

  Unlock();
  return result;
}

esp_err_t
WifiManagerDeinit(void)
{
  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  // IMPORTANT:
  // In this project, mesh may also own and use Wi-Fi. A "hard deinit"
  // here can
  // break mesh or other networking users. Default behavior should be
  // "stop only".
  esp_err_t result = CleanupKeepResourcesLocked();

  Unlock();
  return result;
}

// If you truly need a hard teardown (generally: avoid in firmware),
// implement and call this explicitly from controlled contexts only.
// esp_err_t WifiManagerHardDeinit(void) { lock;
// CleanupReleaseResourcesLocked(); unlock; }
esp_err_t
WifiManagerStop(void)
{
  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  esp_err_t result = CleanupKeepResourcesLocked();

  Unlock();
  return result;
}

esp_err_t
WifiManagerScan(wifi_ap_record_t* out_records,
                size_t max_records,
                size_t* out_count)
{
  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  if (!s_wifi_started || s_event_group == NULL) {
    Unlock();
    return ESP_ERR_INVALID_STATE;
  }

  xEventGroupClearBits(s_event_group, WIFI_SCAN_DONE_BIT);

  wifi_scan_config_t config;
  memset(&config, 0, sizeof(config));

  esp_err_t result = ESP_OK;
  result = esp_wifi_scan_start(&config, false);
  if (result != ESP_OK) {
    goto exit;
  }

  const EventBits_t bits = xEventGroupWaitBits(
    s_event_group, WIFI_SCAN_DONE_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(15000));
  if ((bits & WIFI_SCAN_DONE_BIT) == 0) {
    // A scan can outlive the caller's timeout. If the caller then stops Wi-Fi
    // soon after, the late SCAN_DONE event can arrive during teardown and
    // trigger hard-to-debug crashes. Best-effort stop the scan before returning.
    (void)esp_wifi_scan_stop();
    // Give the driver a moment to post SCAN_DONE (if it will) to drain event
    // processing.
    (void)xEventGroupWaitBits(s_event_group,
                             WIFI_SCAN_DONE_BIT,
                             pdTRUE,
                             pdFALSE,
                             pdMS_TO_TICKS(1000));
    result = ESP_ERR_TIMEOUT;
    goto exit;
  }

  uint16_t num_aps = 0;
  result = esp_wifi_scan_get_ap_num(&num_aps);
  if (result != ESP_OK) {
    goto exit;
  }

  uint16_t record_cap =
    (uint16_t)((max_records > UINT16_MAX) ? UINT16_MAX : max_records);
  if (out_records != NULL && record_cap > 0) {
    uint16_t record_count = record_cap;
    result = esp_wifi_scan_get_ap_records(&record_count, out_records);
    if (result != ESP_OK) {
      goto exit;
    }
  }

  if (out_count != NULL) {
    *out_count = num_aps;
  }

exit:
  Unlock();
  return result;
}

esp_err_t
WifiManagerConnectSta(const char* ssid, const char* password, int timeout_ms)
{
  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  esp_err_t result = ESP_OK;

  if (!s_wifi_started || s_event_group == NULL) {
    result = ESP_ERR_INVALID_STATE;
    goto exit;
  }
  if (ssid == NULL || ssid[0] == '\0') {
    result = ESP_ERR_INVALID_ARG;
    goto exit;
  }

  wifi_config_t config;
  memset(&config, 0, sizeof(config));

  // Ensure NUL termination (strncpy does not guarantee it).
  // ESP-IDF provides strlcpy in many configs; if unavailable, do manual.
  // Use a safe copy pattern.
  size_t ssid_cap = sizeof(config.sta.ssid);
  strncpy((char*)config.sta.ssid, ssid, ssid_cap - 1);
  config.sta.ssid[ssid_cap - 1] = '\0';

  if (password != NULL) {
    size_t pass_cap = sizeof(config.sta.password);
    strncpy((char*)config.sta.password, password, pass_cap - 1);
    config.sta.password[pass_cap - 1] = '\0';
  }

  result = esp_wifi_set_config(WIFI_IF_STA, &config);
  if (result != ESP_OK) {
    goto exit;
  }

  const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
  s_last_connect_attempts = 0;
  s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
  bool saw_fail_bit = false;

  while (esp_timer_get_time() < deadline_us && s_last_connect_attempts < 3) {
    ++s_last_connect_attempts;
    xEventGroupClearBits(
      s_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_SCAN_DONE_BIT);

    result = esp_wifi_disconnect();
    if (result != ESP_OK && result != ESP_ERR_WIFI_NOT_INIT &&
        result != ESP_ERR_WIFI_NOT_STARTED && result != ESP_ERR_WIFI_CONN) {
      goto exit;
    }

    result = esp_wifi_connect();
    if (result != ESP_OK) {
      goto exit;
    }

    int wait_ms = (int)((deadline_us - esp_timer_get_time()) / 1000);
    if (wait_ms < 1000) {
      wait_ms = 1000;
    }

    const EventBits_t bits =
      xEventGroupWaitBits(s_event_group,
                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                          pdFALSE,
                          pdFALSE,
                          pdMS_TO_TICKS(wait_ms));
    if ((bits & WIFI_CONNECTED_BIT) != 0) {
      s_wifi_connected = true;
      result = ESP_OK;
      goto exit;
    }

    if ((bits & WIFI_FAIL_BIT) == 0) {
      break;
    }
    saw_fail_bit = true;
  }

  s_wifi_connected = false;
  result = saw_fail_bit ? ESP_FAIL : ESP_ERR_TIMEOUT;

exit:
  Unlock();
  return result;
}

esp_err_t
WifiManagerDisconnectSta(void)
{
  esp_err_t lock_result = Lock(pdMS_TO_TICKS(5000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  esp_err_t result = ESP_OK;

  if (!s_wifi_started) {
    goto exit;
  }
  if (!s_started_by_manager) {
    goto exit;
  }

  s_wifi_connected = false;
  s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
  result = esp_wifi_disconnect();
  if (result == ESP_ERR_WIFI_NOT_INIT || result == ESP_ERR_WIFI_NOT_STARTED) {
    result = ESP_OK;
  }

exit:
  Unlock();
  return result;
}

void
WifiManagerGetStatus(wifi_manager_status_t* out_status)
{
  if (out_status == NULL) {
    return;
  }
  memset(out_status, 0, sizeof(*out_status));

  if (Lock(pdMS_TO_TICKS(1000)) != ESP_OK) {
    return;
  }

  out_status->sta_netif_present = (s_sta_netif != NULL);
  out_status->owns_sta_netif = s_owns_sta_netif;
  out_status->wifi_initialized = s_wifi_initialized;
  out_status->wifi_handler_registered = s_wifi_handler_registered;
  out_status->ip_handler_registered = s_ip_handler_registered;
  out_status->wifi_started = s_wifi_started;
  out_status->started_by_manager = s_started_by_manager;
  out_status->wifi_connected = s_wifi_connected;

  Unlock();
}

bool
WifiManagerIsStarted(void)
{
  return s_wifi_started;
}

bool
WifiManagerIsConnected(void)
{
  return s_wifi_connected;
}

void
WifiManagerNotifyWifiStarted(void)
{
  if (Lock(pdMS_TO_TICKS(1000)) != ESP_OK) {
    return;
  }

  s_wifi_started = true;
  s_started_by_manager = true;

  Unlock();
}

esp_err_t
WifiManagerGetIpInfo(esp_netif_ip_info_t* out_ip)
{
  if (out_ip == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t lock_result = Lock(pdMS_TO_TICKS(1000));
  if (lock_result != ESP_OK) {
    return lock_result;
  }

  esp_err_t result = ESP_OK;
  if (!s_wifi_connected || s_sta_netif == NULL) {
    result = ESP_ERR_INVALID_STATE;
  } else {
    result = esp_netif_get_ip_info(s_sta_netif, out_ip);
  }

  Unlock();
  return result;
}

wifi_err_reason_t
WifiManagerLastDisconnectReason(void)
{
  return s_last_disconnect_reason;
}

int
WifiManagerLastConnectAttempts(void)
{
  return s_last_connect_attempts;
}
