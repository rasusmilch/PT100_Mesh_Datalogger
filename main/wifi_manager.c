#include "wifi_manager.h"

#include <stdint.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
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
static wifi_err_reason_t s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
static int s_last_connect_attempts = 0;

static void
WifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  (void)arg;
  (void)event_base;

  switch (event_id) {
    case WIFI_EVENT_STA_START:
      xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
      s_wifi_started = true;
      break;

    case WIFI_EVENT_STA_DISCONNECTED: {
      wifi_event_sta_disconnected_t* info = (wifi_event_sta_disconnected_t*)event_data;
      s_wifi_connected = false;
      s_last_disconnect_reason = (info != NULL) ? info->reason : WIFI_REASON_UNSPECIFIED;
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
IpEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  (void)arg;
  (void)event_data;

  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    s_wifi_connected = true;
    s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
    xEventGroupClearBits(s_event_group, WIFI_FAIL_BIT);
    xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
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

esp_err_t
WifiManagerInit(void)
{
  if (s_wifi_started) {
    return ESP_OK;
  }

  esp_err_t result = EnsureEventGroup();
  if (result != ESP_OK) {
    return result;
  }

  result = esp_netif_init();
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "esp_netif_init failed: %s", esp_err_to_name(result));
    return result;
  }

  result = esp_event_loop_create_default();
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "event loop creation failed: %s", esp_err_to_name(result));
    return result;
  }

  if (s_sta_netif == NULL) {
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (s_sta_netif == NULL) {
      ESP_LOGE(kTag, "failed to create default Wi-Fi STA netif");
      return ESP_ERR_NO_MEM;
    }
  }

  if (!s_wifi_initialized) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&cfg);
    if (result != ESP_OK && result != ESP_ERR_WIFI_INIT_STATE) {
      ESP_LOGE(kTag, "esp_wifi_init failed: %s", esp_err_to_name(result));
      return result;
    }
    if (result == ESP_OK) {
      s_wifi_initialized = true;
    } else {
      // Already initialized elsewhere; treat as ready.
      s_wifi_initialized = true;
    }
  }

  result = esp_event_handler_instance_register(
    WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiEventHandler, NULL, &s_wifi_handler);
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "wifi handler register failed: %s", esp_err_to_name(result));
    return result;
  }
  if (result == ESP_ERR_INVALID_STATE) {
    s_wifi_handler = NULL;
  }

  result = esp_event_handler_instance_register(
    IP_EVENT, IP_EVENT_STA_GOT_IP, &IpEventHandler, NULL, &s_ip_handler);
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "ip handler register failed: %s", esp_err_to_name(result));
    return result;
  }
  if (result == ESP_ERR_INVALID_STATE) {
    s_ip_handler = NULL;
  }

  result = esp_wifi_set_mode(WIFI_MODE_STA);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "esp_wifi_set_mode failed: %s", esp_err_to_name(result));
    return result;
  }

  result = esp_wifi_start();
  if (result != ESP_OK && result != ESP_ERR_WIFI_CONN) {
    ESP_LOGE(kTag, "esp_wifi_start failed: %s", esp_err_to_name(result));
    return result;
  }

  s_wifi_started = true;
  return ESP_OK;
}

esp_err_t
WifiManagerDeinit(void)
{
  if (!s_wifi_initialized) {
    return ESP_OK;
  }

  s_wifi_connected = false;
  s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;

  if (s_wifi_started) {
    (void)esp_wifi_stop();
    s_wifi_started = false;
  }

  if (s_wifi_handler != NULL) {
    (void)esp_event_handler_instance_unregister(
      WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_handler);
    s_wifi_handler = NULL;
  }

  if (s_ip_handler != NULL) {
    (void)esp_event_handler_instance_unregister(
      IP_EVENT, IP_EVENT_STA_GOT_IP, s_ip_handler);
    s_ip_handler = NULL;
  }

  esp_err_t result = esp_wifi_deinit();
  if (result == ESP_ERR_WIFI_NOT_INIT) {
    result = ESP_OK;
  }
  s_wifi_initialized = false;

  if (s_sta_netif != NULL) {
    esp_netif_destroy(s_sta_netif);
    s_sta_netif = NULL;
  }

  return result;
}

esp_err_t
WifiManagerScan(wifi_ap_record_t* out_records, size_t max_records, size_t* out_count)
{
  if (!s_wifi_started) {
    return ESP_ERR_INVALID_STATE;
  }

  xEventGroupClearBits(s_event_group, WIFI_SCAN_DONE_BIT);

  wifi_scan_config_t config;
  memset(&config, 0, sizeof(config));

  esp_err_t result = esp_wifi_scan_start(&config, false);
  if (result != ESP_OK) {
    return result;
  }

  const EventBits_t bits = xEventGroupWaitBits(s_event_group,
                                               WIFI_SCAN_DONE_BIT,
                                               pdTRUE,
                                               pdFALSE,
                                               pdMS_TO_TICKS(15000));
  if ((bits & WIFI_SCAN_DONE_BIT) == 0) {
    return ESP_ERR_TIMEOUT;
  }

  uint16_t num_aps = 0;
  result = esp_wifi_scan_get_ap_num(&num_aps);
  if (result != ESP_OK) {
    return result;
  }

  uint16_t record_cap = (uint16_t)((max_records > UINT16_MAX) ? UINT16_MAX
                                                             : max_records);
  if (out_records != NULL && record_cap > 0) {
    uint16_t record_count = record_cap;
    result = esp_wifi_scan_get_ap_records(&record_count, out_records);
    if (result != ESP_OK) {
      return result;
    }
  }

  if (out_count != NULL) {
    *out_count = num_aps;
  }

  return ESP_OK;
}

esp_err_t
WifiManagerConnectSta(const char* ssid, const char* password, int timeout_ms)
{
  if (!s_wifi_started) {
    return ESP_ERR_INVALID_STATE;
  }
  if (ssid == NULL || ssid[0] == '\0') {
    return ESP_ERR_INVALID_ARG;
  }

  wifi_config_t config;
  memset(&config, 0, sizeof(config));
  strncpy((char*)config.sta.ssid, ssid, sizeof(config.sta.ssid));
  if (password != NULL) {
    strncpy((char*)config.sta.password, password, sizeof(config.sta.password));
  }

  esp_err_t result = esp_wifi_set_config(WIFI_IF_STA, &config);
  if (result != ESP_OK) {
    return result;
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
      return result;
    }

    result = esp_wifi_connect();
    if (result != ESP_OK) {
      return result;
    }

    int wait_ms = (int)((deadline_us - esp_timer_get_time()) / 1000);
    if (wait_ms < 1000) {
      wait_ms = 1000;
    }

    const EventBits_t bits = xEventGroupWaitBits(s_event_group,
                                                 WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                 pdFALSE,
                                                 pdFALSE,
                                                 pdMS_TO_TICKS(wait_ms));
    if ((bits & WIFI_CONNECTED_BIT) != 0) {
      s_wifi_connected = true;
      return ESP_OK;
    }

    if ((bits & WIFI_FAIL_BIT) == 0) {
      break;
    }
    saw_fail_bit = true;
  }

  s_wifi_connected = false;
  return saw_fail_bit ? ESP_FAIL : ESP_ERR_TIMEOUT;
}

esp_err_t
WifiManagerDisconnectSta(void)
{
  if (!s_wifi_started) {
    return ESP_OK;
  }
  s_wifi_connected = false;
  s_last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
  return esp_wifi_disconnect();
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

esp_err_t
WifiManagerGetIpInfo(esp_netif_ip_info_t* out_ip)
{
  if (out_ip == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!s_wifi_connected || s_sta_netif == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return esp_netif_get_ip_info(s_sta_netif, out_ip);
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
