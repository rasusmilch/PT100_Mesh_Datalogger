#include "net_supervisor.h"

#include <inttypes.h>
#include <string.h>

#include "app_net_config.h"
#include "app_settings.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime_manager.h"
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
static uint32_t s_sntp_consecutive_failures_time_invalid = 0;
static uint32_t s_sntp_consecutive_failures_time_valid = 0;
static bool s_sntp_failure_alert_active = false;

static esp_err_t TrySntpSyncWithFailover(int timeout_ms, uint8_t max_servers);

static app_net_mode_t
GetDesiredNetMode(void)
{
  if (s_runtime == NULL || s_runtime->settings == NULL) {
    return APP_NET_MODE_NONE;
  }
  return AppSettingsGetEffectiveNetMode(s_runtime->settings->node_role,
                                        s_runtime->settings->net_mode);
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
               uint32_t* retry_delay_ms,
               uint32_t* time_sync_retry_ms)
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
  if (time_sync_retry_ms != NULL) {
    *time_sync_retry_ms = 30 * 1000;
  }
}

/**
 * @brief Execute ResetWifiAcquireBackoff.
 * @param backoff_ms Parameter backoff_ms.
 * @param next_attempt_ms Parameter next_attempt_ms.
 */
static void
ResetWifiAcquireBackoff(uint32_t* backoff_ms, int64_t* next_attempt_ms)
{
  if (backoff_ms != NULL) {
    *backoff_ms = 0;
  }
  if (next_attempt_ms != NULL) {
    *next_attempt_ms = 0;
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

static esp_err_t
TrySntpSyncWithFailover(int timeout_ms, uint8_t max_servers)
{
  const uint8_t count = AppNetConfigGetSntpServerCount();
  if (count == 0) {
    return ESP_FAIL;
  }

  uint8_t attempt_count =
    (count < APP_NET_SNTP_SERVERS_MAX_COUNT) ? count
                                             : APP_NET_SNTP_SERVERS_MAX_COUNT;
  if (max_servers > 0 && max_servers < attempt_count) {
    attempt_count = max_servers;
  }
  esp_err_t last_result = ESP_FAIL;
  for (uint8_t idx = 0; idx < attempt_count; ++idx) {
    const char* server = AppNetConfigGetSntpServerAt(idx);
    if (server == NULL || server[0] == '\0') {
      last_result = ESP_FAIL;
      continue;
    }
    const esp_err_t result = TimeSyncStartSntpAndWait(server, timeout_ms);
    if (result == ESP_OK) {
      return ESP_OK;
    }
    const char* role =
      (idx == 0) ? "primary" : (idx == 1) ? "secondary" : "tertiary";
    ESP_LOGI(kTag,
             "SNTP failed for %s server=%s: %s; trying next",
             role,
             server,
             esp_err_to_name(result));
    last_result = result;
  }
  return last_result;
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
  uint32_t time_sync_retry_ms = 30 * 1000;
  const uint32_t max_retry_delay_ms = 5 * 60 * 1000;
  const uint32_t max_time_sync_retry_ms = 5 * 60 * 1000;
  const uint32_t min_wifi_acquire_backoff_ms = 1000;
  const uint32_t max_wifi_acquire_backoff_ms = 5 * 60 * 1000;
  uint32_t wifi_acquire_backoff_ms = 0;
  int64_t wifi_next_acquire_attempt_ms = 0;
  size_t last_wifi_acquire_free_internal = 0;
  size_t last_wifi_acquire_largest_internal = 0;
  size_t last_wifi_acquire_free_dma_internal = 0;
  size_t last_wifi_acquire_largest_dma_internal = 0;

  for (;;) {
    const TickType_t now_ticks = xTaskGetTickCount();
    const int64_t now_ms = esp_timer_get_time() / 1000;

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
        (void)WifiServiceRelease();
      } else if (active_mode == NET_SUP_MODE_MESH) {
        (void)RuntimeApplyNetMode(APP_NET_MODE_NONE);
      }
      active_mode = NET_SUP_MODE_NONE;
      ResetWifiState(
        &connected,
        &next_connect_ticks,
        &next_time_sync_ticks,
        &retry_delay_ms,
        &time_sync_retry_ms);
      ResetWifiAcquireBackoff(&wifi_acquire_backoff_ms,
                              &wifi_next_acquire_attempt_ms);
      ESP_LOGI(kTag,
               "Net mode change -> %s",
               AppSettingsNetModeToString(desired_net_mode));
    }

    if (active_mode != desired_mode) {
      if (desired_mode == NET_SUP_MODE_MESH) {
        ResetWifiAcquireBackoff(&wifi_acquire_backoff_ms,
                                &wifi_next_acquire_attempt_ms);
        const esp_err_t mesh_result = RuntimeApplyNetMode(desired_net_mode);
        if (mesh_result == ESP_OK) {
          active_mode = NET_SUP_MODE_MESH;
          ResetWifiState(&connected,
                         &next_connect_ticks,
                         &next_time_sync_ticks,
                         &retry_delay_ms,
                         &time_sync_retry_ms);
          ESP_LOGI(kTag, "Mesh transport started");
        } else {
          ESP_LOGW(kTag,
                   "Mesh transport start failed: %s",
                   esp_err_to_name(mesh_result));
        }
      } else if (desired_mode == NET_SUP_MODE_WIFI) {
        if (wifi_next_acquire_attempt_ms == 0 ||
            now_ms >= wifi_next_acquire_attempt_ms) {
          (void)RuntimeApplyNetMode(desired_net_mode);
          const wifi_service_mode_t svc_mode = ToWifiServiceMode(desired_mode);
          const esp_err_t acquire_result = WifiServiceAcquire(svc_mode);
          if (acquire_result == ESP_OK) {
            active_mode = NET_SUP_MODE_WIFI;
            ResetWifiState(&connected,
                           &next_connect_ticks,
                           &next_time_sync_ticks,
                           &retry_delay_ms,
                           &time_sync_retry_ms);
            ResetWifiAcquireBackoff(&wifi_acquire_backoff_ms,
                                    &wifi_next_acquire_attempt_ms);
            ESP_LOGI(kTag, "Wi-Fi service acquired (mode=%d)", (int)svc_mode);
          } else {
            if (acquire_result == ESP_ERR_NO_MEM) {
              last_wifi_acquire_free_internal =
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
              last_wifi_acquire_largest_internal =
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
              last_wifi_acquire_free_dma_internal =
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
              last_wifi_acquire_largest_dma_internal =
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                                 MALLOC_CAP_DMA);
              if (wifi_acquire_backoff_ms == 0) {
                wifi_acquire_backoff_ms = min_wifi_acquire_backoff_ms;
              } else {
                wifi_acquire_backoff_ms =
                  (wifi_acquire_backoff_ms < max_wifi_acquire_backoff_ms / 2)
                    ? wifi_acquire_backoff_ms * 2
                    : max_wifi_acquire_backoff_ms;
                if (wifi_acquire_backoff_ms < min_wifi_acquire_backoff_ms) {
                  wifi_acquire_backoff_ms = min_wifi_acquire_backoff_ms;
                }
                if (wifi_acquire_backoff_ms > max_wifi_acquire_backoff_ms) {
                  wifi_acquire_backoff_ms = max_wifi_acquire_backoff_ms;
                }
              }
              wifi_next_acquire_attempt_ms =
                now_ms + wifi_acquire_backoff_ms;
              ESP_LOGW(
                kTag,
                "Wi-Fi acquire deferred (%" PRIu32
                " ms); next retry in %" PRIu32
                " ms (heap free=%u, largest=%u, dma_free=%u, dma_largest=%u)",
                wifi_acquire_backoff_ms,
                wifi_acquire_backoff_ms,
                (unsigned)last_wifi_acquire_free_internal,
                (unsigned)last_wifi_acquire_largest_internal,
                (unsigned)last_wifi_acquire_free_dma_internal,
                (unsigned)last_wifi_acquire_largest_dma_internal);
            } else {
              ESP_LOGW(kTag,
                       "Wi-Fi service acquire failed: %s",
                       esp_err_to_name(acquire_result));
            }
          }
        }
      } else {
        (void)RuntimeApplyNetMode(desired_net_mode);
        if (active_mode == NET_SUP_MODE_WIFI) {
          (void)WifiManagerDisconnectSta();
          (void)WifiServiceRelease();
        } else if (active_mode == NET_SUP_MODE_MESH) {
          (void)RuntimeApplyNetMode(APP_NET_MODE_NONE);
        }
        active_mode = NET_SUP_MODE_NONE;
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
                       &retry_delay_ms,
                       &time_sync_retry_ms);
      }
    }

    if (active_mode == NET_SUP_MODE_WIFI) {
      wifi_credentials_t creds;
      WifiCredentialsLoad(&creds);
      const bool has_creds = creds.has_ssid;

      connected = WifiManagerIsConnected();
      const bool connection_changed = (connected != last_connected);
      MaybeLogConnectionChange(connected, &last_connected);

      if (!has_creds) {
        if (connected) {
          (void)WifiManagerDisconnectSta();
          connected = false;
        }
        ResetWifiState(&connected,
                       &next_connect_ticks,
                       &next_time_sync_ticks,
                       &retry_delay_ms,
                       &time_sync_retry_ms);
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
            time_sync_retry_ms = 30 * 1000;
          } else {
            retry_delay_ms = (retry_delay_ms < max_retry_delay_ms / 2)
                               ? retry_delay_ms * 2
                               : max_retry_delay_ms;
            next_connect_ticks = now_ticks + pdMS_TO_TICKS(retry_delay_ms);
          }
        }
      } else if (connection_changed) {
        next_time_sync_ticks = now_ticks;
        time_sync_retry_ms = 30 * 1000;
      }

      if (connected && next_time_sync_ticks != 0 &&
          (int32_t)(now_ticks - next_time_sync_ticks) >= 0) {
        if (RuntimeIsI2cQuiesceActive()) {
          next_time_sync_ticks = now_ticks + pdMS_TO_TICKS(1000);
          continue;
        }
        const bool system_time_valid = TimeSyncIsSystemTimeValid();
        const int timeout_ms = system_time_valid ? (5 * 1000) : (30 * 1000);
        const uint8_t max_servers = system_time_valid ? 1 : APP_NET_SNTP_SERVERS_MAX_COUNT;
        const esp_err_t sntp_result =
          TrySntpSyncWithFailover(timeout_ms, max_servers);
        if (sntp_result == ESP_OK) {
          s_sntp_consecutive_failures_time_invalid = 0;
          s_sntp_consecutive_failures_time_valid = 0;
          s_sntp_failure_alert_active = false;
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
          if (!system_time_valid) {
            s_sntp_consecutive_failures_time_invalid++;
            s_sntp_consecutive_failures_time_valid = 0;
            ESP_LOGW(kTag, "SNTP unsuccessful (time invalid)");
            const uint32_t threshold = AppNetConfigGetSntpFailThresholdN();
            if (s_sntp_consecutive_failures_time_invalid >= threshold) {
              if (!s_sntp_failure_alert_active) {
                ESP_LOGE(kTag,
                         "SNTP failed %u consecutive times; "
                         "raising display/console alert",
                         (unsigned)s_sntp_consecutive_failures_time_invalid);
              }
              s_sntp_failure_alert_active = true;
            }
          } else {
            s_sntp_consecutive_failures_time_valid++;
            s_sntp_consecutive_failures_time_invalid = 0;
            if ((s_sntp_consecutive_failures_time_valid % 6U) == 1U) {
              ESP_LOGI(kTag,
                       "SNTP unsuccessful (time already valid) count=%u",
                       (unsigned)s_sntp_consecutive_failures_time_valid);
            }
          }
        }

        const uint32_t time_sync_period_s =
          AppNetConfigGetTimeSyncPeriodSeconds();
        if (sntp_result == ESP_OK) {
          time_sync_retry_ms = 30 * 1000;
          if (time_sync_period_s == 0) {
            next_time_sync_ticks = 0;
          } else {
            TickType_t period_ticks =
              pdMS_TO_TICKS((uint64_t)time_sync_period_s * 1000ULL);
            if (period_ticks == 0) {
              period_ticks = pdMS_TO_TICKS(60 * 1000);
            }
            next_time_sync_ticks = now_ticks + period_ticks;
          }
        } else if (system_time_valid) {
          TickType_t period_ticks =
            pdMS_TO_TICKS((uint64_t)time_sync_period_s * 1000ULL);
          if (period_ticks == 0) {
            period_ticks = pdMS_TO_TICKS(60 * 1000);
          }
          next_time_sync_ticks = now_ticks + period_ticks;
        } else {
          const uint32_t retry_ms = time_sync_retry_ms;
          time_sync_retry_ms = (time_sync_retry_ms < max_time_sync_retry_ms / 2)
                                 ? time_sync_retry_ms * 2
                                 : max_time_sync_retry_ms;
          next_time_sync_ticks = now_ticks + pdMS_TO_TICKS(retry_ms);
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
  s_sntp_consecutive_failures_time_invalid = 0;
  s_sntp_consecutive_failures_time_valid = 0;
  s_sntp_failure_alert_active = false;

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

uint32_t
NetSupervisorGetSntpConsecutiveFailures(void)
{
  return s_sntp_consecutive_failures_time_invalid;
}

bool
NetSupervisorIsSntpFailureAlertActive(void)
{
  return s_sntp_failure_alert_active;
}
