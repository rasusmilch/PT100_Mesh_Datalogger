#include "net_supervisor.h"

#include <string.h>

#include "app_net_config.h"
#include "app_settings.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "time_sync.h"
#include "wifi_credentials.h"
#include "wifi_manager.h"
#include "wifi_service.h"

static const char* kTag = "net_sup";

typedef enum
{
  NET_SUP_MODE_NONE = 0,
  NET_SUP_MODE_WIFI = 1,
  NET_SUP_MODE_MESH = 2,
} net_supervisor_mode_t;

static const app_runtime_t* s_runtime = NULL;
static TaskHandle_t s_task = NULL;

static app_net_mode_t
GetDesiredNetMode(void)
{
  if (s_runtime == NULL || s_runtime->settings == NULL) {
    return APP_NET_MODE_NONE;
  }
  return s_runtime->settings->net_mode;
}

static net_supervisor_mode_t
ToSupervisorMode(app_net_mode_t net_mode)
{
  switch (net_mode) {
    case APP_NET_MODE_DIRECT_WIFI:
      return NET_SUP_MODE_WIFI;
    case APP_NET_MODE_MESH:
      return NET_SUP_MODE_MESH;
    case APP_NET_MODE_NONE:
      return NET_SUP_MODE_NONE;
    default:
      return NET_SUP_MODE_NONE;
  }
}

static wifi_service_mode_t
ToWifiServiceMode(net_supervisor_mode_t mode)
{
  switch (mode) {
    case NET_SUP_MODE_WIFI:
      return WIFI_SERVICE_MODE_DIAGNOSTIC_STA;
    case NET_SUP_MODE_MESH:
      return WIFI_SERVICE_MODE_MESH;
    default:
      return WIFI_SERVICE_MODE_NONE;
  }
}

static void
ResetWifiState(bool* connected,
               TickType_t* next_connect_ticks,
               TickType_t* next_time_sync_ticks,
               uint32_t* retry_delay_ms)
{
  if (connected != NULL) {
    *connected = false;
  }
  if (next_connect_ticks != NULL) {
    *next_connect_ticks = 0;
  }
  if (next_time_sync_ticks != NULL) {
    *next_time_sync_ticks = 0;
  }
  if (retry_delay_ms != NULL) {
    *retry_delay_ms = 30 * 1000;
  }
}

static void
MaybeLogConnectionChange(bool connected, bool* last_connected)
{
  if (last_connected == NULL || connected == *last_connected) {
    return;
  }
  if (connected) {
    ESP_LOGI(kTag, "Wi-Fi connected");
  } else {
    ESP_LOGW(kTag, "Wi-Fi disconnected");
  }
  *last_connected = connected;
}

static void
NetSupervisorTask(void* context)
{
  (void)context;

  net_supervisor_mode_t active_mode = NET_SUP_MODE_NONE;
  app_net_mode_t last_net_mode = GetDesiredNetMode();
  uint32_t last_net_mode_revision = AppSettingsGetNetModeRevision();
  uint32_t last_credentials_revision = WifiCredentialsGetRevision();
  bool last_connected = WifiManagerIsConnected();
  bool connected = last_connected;
  TickType_t next_connect_ticks = 0;
  TickType_t next_time_sync_ticks = 0;
  uint32_t retry_delay_ms = 30 * 1000;
  const uint32_t max_retry_delay_ms = 5 * 60 * 1000;

  for (;;) {
    const TickType_t now_ticks = xTaskGetTickCount();

    const uint32_t net_mode_revision = AppSettingsGetNetModeRevision();
    const app_net_mode_t desired_net_mode = GetDesiredNetMode();
    const net_supervisor_mode_t desired_mode =
      ToSupervisorMode(desired_net_mode);

    const bool net_mode_changed =
      (desired_net_mode != last_net_mode) ||
      (net_mode_revision != last_net_mode_revision);

    if (net_mode_changed) {
      last_net_mode = desired_net_mode;
      last_net_mode_revision = net_mode_revision;

      if (active_mode == NET_SUP_MODE_WIFI) {
        (void)WifiManagerDisconnectSta();
      }
      if (active_mode != NET_SUP_MODE_NONE) {
        (void)WifiServiceRelease();
      }
      active_mode = NET_SUP_MODE_NONE;
      ResetWifiState(
        &connected, &next_connect_ticks, &next_time_sync_ticks, &retry_delay_ms);
      ESP_LOGI(kTag,
               "Net mode change -> %s",
               AppSettingsNetModeToString(desired_net_mode));
    }

    if (active_mode != desired_mode) {
      const wifi_service_mode_t svc_mode = ToWifiServiceMode(desired_mode);
      if (svc_mode != WIFI_SERVICE_MODE_NONE) {
        const esp_err_t acquire_result = WifiServiceAcquire(svc_mode);
        if (acquire_result == ESP_OK) {
          active_mode = desired_mode;
          ResetWifiState(&connected,
                         &next_connect_ticks,
                         &next_time_sync_ticks,
                         &retry_delay_ms);
          ESP_LOGI(kTag, "Wi-Fi service acquired (mode=%d)", (int)svc_mode);
        } else {
          ESP_LOGW(kTag,
                   "Wi-Fi service acquire failed: %s",
                   esp_err_to_name(acquire_result));
        }
      }
    }

    const uint32_t credential_revision = WifiCredentialsGetRevision();
    if (credential_revision != last_credentials_revision) {
      last_credentials_revision = credential_revision;
      if (active_mode == NET_SUP_MODE_WIFI) {
        (void)WifiManagerDisconnectSta();
        ResetWifiState(&connected,
                       &next_connect_ticks,
                       &next_time_sync_ticks,
                       &retry_delay_ms);
      }
    }

    if (active_mode == NET_SUP_MODE_WIFI) {
      wifi_credentials_t creds;
      WifiCredentialsLoad(&creds);
      const bool has_creds = creds.has_ssid;

      connected = WifiManagerIsConnected();
      MaybeLogConnectionChange(connected, &last_connected);

      if (!has_creds) {
        if (connected) {
          (void)WifiManagerDisconnectSta();
          connected = false;
        }
        ResetWifiState(&connected,
                       &next_connect_ticks,
                       &next_time_sync_ticks,
                       &retry_delay_ms);
      } else if (!connected) {
        if (next_connect_ticks == 0 ||
            (int32_t)(now_ticks - next_connect_ticks) >= 0) {
          const esp_err_t connect_result =
            WifiManagerConnectSta(creds.ssid, creds.password, 10000);
          connected = (connect_result == ESP_OK);
          MaybeLogConnectionChange(connected, &last_connected);

          if (connected) {
            retry_delay_ms = 30 * 1000;
            next_connect_ticks = 0;
            next_time_sync_ticks = now_ticks;
          } else {
            retry_delay_ms = (retry_delay_ms < max_retry_delay_ms / 2)
                               ? retry_delay_ms * 2
                               : max_retry_delay_ms;
            next_connect_ticks = now_ticks + pdMS_TO_TICKS(retry_delay_ms);
          }
        }
      }

      if (connected && next_time_sync_ticks != 0 &&
          (int32_t)(now_ticks - next_time_sync_ticks) >= 0) {
        const char* sntp_server = AppNetConfigGetSntpServer();
        esp_err_t sntp_result = ESP_ERR_INVALID_STATE;
        if (sntp_server != NULL && sntp_server[0] != '\0') {
          sntp_result = TimeSyncStartSntpAndWait(sntp_server, 30 * 1000);
        }
        if (sntp_result == ESP_OK && s_runtime != NULL &&
            s_runtime->time_sync != NULL) {
          const esp_err_t rtc_result =
            TimeSyncSetRtcFromSystem(s_runtime->time_sync);
          if (rtc_result == ESP_OK) {
            ESP_LOGI(kTag, "Time synchronized (SNTP -> RTC UTC)");
          } else {
            ESP_LOGW(kTag,
                     "Time synchronized (SNTP), but RTC update failed: %s",
                     esp_err_to_name(rtc_result));
          }
        } else if (sntp_result != ESP_OK) {
          ESP_LOGW(kTag, "SNTP sync failed: %s", esp_err_to_name(sntp_result));
        }

        const uint32_t time_sync_period_s =
          AppNetConfigGetTimeSyncPeriodSeconds();
        if (time_sync_period_s == 0) {
          next_time_sync_ticks = 0;
        } else {
          TickType_t period_ticks =
            pdMS_TO_TICKS((uint64_t)time_sync_period_s * 1000ULL);
          if (period_ticks == 0) {
            period_ticks = pdMS_TO_TICKS(60 * 1000);
          }
          if (sntp_result != ESP_OK) {
            const TickType_t retry_ticks = pdMS_TO_TICKS(30 * 1000);
            if (period_ticks > retry_ticks) {
              period_ticks = retry_ticks;
            }
          }
          next_time_sync_ticks = now_ticks + period_ticks;
        }
      }
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(1000);
    if (active_mode == NET_SUP_MODE_WIFI) {
      TickType_t next_event_ticks = 0;
      if (next_connect_ticks != 0) {
        next_event_ticks = next_connect_ticks;
      }
      if (next_time_sync_ticks != 0) {
        if (next_event_ticks == 0 ||
            (int32_t)(next_time_sync_ticks - next_event_ticks) < 0) {
          next_event_ticks = next_time_sync_ticks;
        }
      }
      if (next_event_ticks != 0 &&
          (int32_t)(next_event_ticks - now_ticks) > 0) {
        const TickType_t delta = next_event_ticks - now_ticks;
        if (delta < wait_ticks) {
          wait_ticks = delta;
        }
      }
    }
    (void)ulTaskNotifyTake(pdTRUE, wait_ticks);
  }
}

esp_err_t
NetSupervisorStart(const app_runtime_t* runtime)
{
  if (s_task != NULL) {
    return ESP_OK;
  }
  if (runtime == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  s_runtime = runtime;

  const uint32_t kNetSupervisorStackBytes = 4096;
  const UBaseType_t kNetSupervisorPriority = 2;
  BaseType_t created = xTaskCreate(&NetSupervisorTask,
                                   "net_supervisor",
                                   kNetSupervisorStackBytes,
                                   NULL,
                                   kNetSupervisorPriority,
                                   &s_task);
  if (created != pdPASS) {
    s_task = NULL;
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

void
NetSupervisorNotifyUpdate(void)
{
  if (s_task != NULL) {
    xTaskNotifyGive(s_task);
  }
}
