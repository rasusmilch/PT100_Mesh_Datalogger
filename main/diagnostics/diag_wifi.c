#include "diagnostics/diag_wifi.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "wifi_service.h"
#include "wifi_manager.h"

typedef enum
{
  kWifiCredsNone = 0,
  kWifiCredsNvs,
  kWifiCredsKconfig,
} wifi_creds_source_t;

typedef struct
{
  char ssid[33];
  char password[65];
  wifi_creds_source_t source;
} wifi_credentials_t;

typedef struct
{
  size_t free_8bit;
  size_t free_total;
  size_t min_free;
} heap_snapshot_t;

static const char*
AuthModeToString(wifi_auth_mode_t mode)
{
  switch (mode) {
    case WIFI_AUTH_OPEN:
      return "open";
    case WIFI_AUTH_WEP:
      return "wep";
    case WIFI_AUTH_WPA_PSK:
      return "wpa_psk";
    case WIFI_AUTH_WPA2_PSK:
      return "wpa2_psk";
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "wpa_wpa2";
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "wpa2_ent";
    case WIFI_AUTH_WPA3_PSK:
      return "wpa3_psk";
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "wpa2_wpa3";
    case WIFI_AUTH_WAPI_PSK:
      return "wapi_psk";
    default:
      return "unknown";
  }
}

static const char*
ReasonToString(wifi_err_reason_t reason)
{
  switch (reason) {
    case WIFI_REASON_AUTH_EXPIRE:
      return "auth_expire";
    case WIFI_REASON_AUTH_LEAVE:
      return "auth_leave";
    case WIFI_REASON_ASSOC_EXPIRE:
      return "assoc_expire";
    case WIFI_REASON_ASSOC_TOOMANY:
      return "assoc_toomany";
    case WIFI_REASON_NOT_AUTHED:
      return "not_authed";
    case WIFI_REASON_NOT_ASSOCED:
      return "not_assoc";
    case WIFI_REASON_ASSOC_LEAVE:
      return "assoc_leave";
    case WIFI_REASON_ASSOC_NOT_AUTHED:
      return "assoc_not_authed";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:
      return "disassoc_pwrcap";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
      return "disassoc_supchan";
    case WIFI_REASON_IE_INVALID:
      return "ie_invalid";
    case WIFI_REASON_MIC_FAILURE:
      return "mic_failure";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
      return "4way_timeout";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
      return "gk_timeout";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:
      return "ie_4way_diff";
    case WIFI_REASON_GROUP_CIPHER_INVALID:
      return "group_cipher";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
      return "pairwise_cipher";
    case WIFI_REASON_AKMP_INVALID:
      return "akmp_invalid";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
      return "rsn_ver";
    case WIFI_REASON_INVALID_RSN_IE_CAP:
      return "rsn_cap";
    case WIFI_REASON_802_1X_AUTH_FAILED:
      return "8021x_failed";
    case WIFI_REASON_BEACON_TIMEOUT:
      return "beacon_timeout";
    case WIFI_REASON_AUTH_FAIL:
      return "auth_fail";
    case WIFI_REASON_NO_AP_FOUND:
      return "no_ap";
    case WIFI_REASON_CONNECTION_FAIL:
      return "conn_fail";
    case WIFI_REASON_ASSOC_FAIL:
      return "assoc_fail";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
      return "handshake_timeout";
    default:
      return "unknown";
  }
}

static const char*
YesNo(bool value)
{
  return value ? "yes" : "no";
}

static heap_snapshot_t
CaptureHeapSnapshot(void)
{
  heap_snapshot_t snapshot;
  heap_caps_check_integrity_all(true);
  snapshot.free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  snapshot.free_total = esp_get_free_heap_size();
  snapshot.min_free = esp_get_minimum_free_heap_size();
  return snapshot;
}

static void
PrintHeapSnapshot(const diag_ctx_t* ctx,
                  const char* label,
                  const heap_snapshot_t* snapshot)
{
  if (ctx == NULL || ctx->verbosity < kDiagVerbosity1 || snapshot == NULL) {
    return;
  }
  printf("      heap[%s]: free8=%u total=%u min=%u\n",
         label,
         (unsigned)snapshot->free_8bit,
         (unsigned)snapshot->free_total,
         (unsigned)snapshot->min_free);
}

static void
LoadCredentials(wifi_credentials_t* creds)
{
  if (creds == NULL) {
    return;
  }
  memset(creds, 0, sizeof(*creds));

  nvs_handle_t handle;
  esp_err_t result = nvs_open("app", NVS_READONLY, &handle);
  if (result == ESP_OK) {
    size_t ssid_len = sizeof(creds->ssid);
    result = nvs_get_str(handle, "wifi_ssid", creds->ssid, &ssid_len);
    if (result != ESP_OK || creds->ssid[0] == '\0') {
      memset(creds->ssid, 0, sizeof(creds->ssid));
    }

    size_t pass_len = sizeof(creds->password);
    result = nvs_get_str(handle, "wifi_pass", creds->password, &pass_len);
    if (result != ESP_OK || creds->password[0] == '\0') {
      memset(creds->password, 0, sizeof(creds->password));
    }
    nvs_close(handle);
  }

  if (creds->ssid[0] == '\0' && CONFIG_APP_WIFI_ROUTER_SSID[0] != '\0') {
    strncpy(creds->ssid, CONFIG_APP_WIFI_ROUTER_SSID, sizeof(creds->ssid) - 1);
    creds->ssid[sizeof(creds->ssid) - 1] = '\0';
  }

  if (creds->password[0] == '\0' && CONFIG_APP_WIFI_ROUTER_PASSWORD[0] != '\0') {
    strncpy(creds->password,
            CONFIG_APP_WIFI_ROUTER_PASSWORD,
            sizeof(creds->password) - 1);
    creds->password[sizeof(creds->password) - 1] = '\0';
  }

  if (creds->ssid[0] != '\0') {
    creds->source =
      (result == ESP_OK) ? kWifiCredsNvs : kWifiCredsKconfig;
  }
}

static void
PrintScanResults(const diag_ctx_t* ctx,
                 const wifi_ap_record_t* records,
                 size_t listed_count,
                 size_t total_count)
{
  if (ctx == NULL || ctx->verbosity < kDiagVerbosity1) {
    return;
  }
  const size_t to_show = (listed_count < 10) ? listed_count : 10;
  printf("      APs found: %u (showing %u of %u)\n",
         (unsigned)total_count,
         (unsigned)to_show,
         (unsigned)listed_count);
  for (size_t index = 0; index < to_show; ++index) {
    const wifi_ap_record_t* ap = &records[index];
    const char* ssid = (ap->ssid[0] != '\0') ? (const char*)ap->ssid : "<hidden>";
    printf("        %2u. %-32s rssi=%d ch=%u auth=%s\n",
           (unsigned)(index + 1),
           ssid,
           ap->rssi,
           (unsigned)ap->primary,
           AuthModeToString(ap->authmode));
  }
}

static const char*
CredsSourceToString(wifi_creds_source_t source)
{
  switch (source) {
    case kWifiCredsNvs:
      return "nvs";
    case kWifiCredsKconfig:
      return "kconfig";
    default:
      return "none";
  }
}

static const char*
PickDnsHost(void)
{
  return (CONFIG_APP_SNTP_SERVER[0] != '\0') ? CONFIG_APP_SNTP_SERVER
                                             : "pool.ntp.org";
}

int
RunDiagWifi(const app_runtime_t* runtime,
            bool full,
            bool scan,
            bool connect,
            bool dns_lookup,
            bool keep_connected,
            diag_verbosity_t verbosity)
{
  (void)full;

  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "WiFi", verbosity);

  wifi_credentials_t creds;
  LoadCredentials(&creds);
  const bool has_ssid = creds.ssid[0] != '\0';
  const wifi_service_mode_t active_mode = WifiServiceActiveMode();

  const int total_steps = 4 + (scan ? 1 : 0) + (connect ? 1 : 0) +
                          (dns_lookup ? 1 : 0);
  int step = 1;

  const bool runtime_running = RuntimeIsRunning();
  const bool mesh_active = (runtime != NULL && runtime->mesh != NULL &&
                            runtime->mesh->is_started);
  DiagReportStep(&ctx,
                 step++,
                 total_steps,
                 "runtime idle",
                 (runtime_running || mesh_active ||
                  active_mode == WIFI_SERVICE_MODE_MESH)
                   ? ESP_ERR_INVALID_STATE
                   : ESP_OK,
                 runtime_running
                   ? "stop runtime first: `run stop`"
                   : (mesh_active || active_mode == WIFI_SERVICE_MODE_MESH)
                       ? "mesh active; stop runtime to use Wi-Fi diag"
                       : "idle");
  if (runtime_running || mesh_active ||
      active_mode == WIFI_SERVICE_MODE_MESH) {
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  heap_snapshot_t net_before = CaptureHeapSnapshot();
  DiagHeapCheck(&ctx, "pre_net");
  esp_err_t net_result = WifiServiceInitOnce();
  heap_snapshot_t net_after = CaptureHeapSnapshot();
  DiagHeapCheck(&ctx, "post_net");
  DiagReportStep(&ctx,
                 step++,
                 total_steps,
                 "net stack",
                 net_result,
                 "heap8_before=%u heap8_after=%u min_free=%u",
                 (unsigned)net_before.free_8bit,
                 (unsigned)net_after.free_8bit,
                 (unsigned)net_after.min_free);
  PrintHeapSnapshot(&ctx, "net_before", &net_before);
  PrintHeapSnapshot(&ctx, "net_after", &net_after);
  if (net_result != ESP_OK) {
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  wifi_manager_status_t before_status;
  memset(&before_status, 0, sizeof(before_status));
  WifiManagerGetStatus(&before_status);
  heap_snapshot_t wifi_before = CaptureHeapSnapshot();
  DiagHeapCheck(&ctx, "pre_wifi_start");
  esp_err_t init_result = WifiServiceStart(WIFI_SERVICE_MODE_DIAGNOSTIC_STA);
  wifi_manager_status_t after_status;
  memset(&after_status, 0, sizeof(after_status));
  WifiManagerGetStatus(&after_status);
  heap_snapshot_t wifi_after = CaptureHeapSnapshot();
  DiagHeapCheck(&ctx, "post_wifi_start");
  const bool sta_created =
    (!before_status.sta_netif_present && after_status.sta_netif_present);

  DiagReportStep(&ctx,
                 step++,
                 total_steps,
                 "wifi init",
                 init_result,
                 "sta_netif=%s (owned=%s created=%s) wifi_init_owned=%s handlers=%s/%s started=%s",
                 YesNo(after_status.sta_netif_present),
                 YesNo(after_status.owns_sta_netif),
                 YesNo(sta_created),
                 YesNo(after_status.owns_wifi_init),
                 YesNo(after_status.wifi_handler_registered),
                 YesNo(after_status.ip_handler_registered),
                 YesNo(after_status.wifi_started));
  PrintHeapSnapshot(&ctx, "wifi_init_before", &wifi_before);
  PrintHeapSnapshot(&ctx, "wifi_init_after", &wifi_after);

  wifi_ap_record_t ap_records[20];
  memset(ap_records, 0, sizeof(ap_records));
  size_t ap_count = 0;
  esp_err_t scan_result = ESP_ERR_INVALID_STATE;
  if (scan) {
    heap_snapshot_t scan_before = CaptureHeapSnapshot();
    DiagHeapCheck(&ctx, "pre_scan");
    if (init_result == ESP_OK) {
      scan_result = WifiManagerScan(ap_records, 20, &ap_count);
    }
    bool ssid_present = false;
    if (scan_result == ESP_OK && has_ssid) {
      for (size_t i = 0; i < ap_count; ++i) {
        if (strncmp((const char*)ap_records[i].ssid,
                    creds.ssid,
                    sizeof(ap_records[i].ssid)) == 0) {
          ssid_present = true;
          break;
        }
      }
    }

    heap_snapshot_t scan_after = CaptureHeapSnapshot();
    DiagHeapCheck(&ctx, "post_scan");
    DiagReportStep(&ctx,
                   step++,
                   total_steps,
                   "scan",
                   scan_result,
                   "aps=%u ssid_present=%s heap8_before=%u heap8_after=%u min_free=%u",
                   (unsigned)ap_count,
                   has_ssid ? (ssid_present ? "yes" : "no") : "n/a",
                   (unsigned)scan_before.free_8bit,
                   (unsigned)scan_after.free_8bit,
                   (unsigned)scan_after.min_free);
    PrintHeapSnapshot(&ctx, "scan_before", &scan_before);
    PrintHeapSnapshot(&ctx, "scan_after", &scan_after);

    if (scan_result == ESP_OK) {
      const size_t listed_count = (ap_count < (sizeof(ap_records) /
                                              sizeof(ap_records[0])))
                                    ? ap_count
                                    : (sizeof(ap_records) /
                                       sizeof(ap_records[0]));
      PrintScanResults(&ctx, ap_records, listed_count, ap_count);
    }
  }

  bool connected = false;
  esp_err_t connect_result = ESP_ERR_INVALID_STATE;
  if (connect) {
    heap_snapshot_t connect_before = CaptureHeapSnapshot();
    heap_snapshot_t connect_after = connect_before;
    DiagHeapCheck(&ctx, "pre_connect");
    if (!has_ssid) {
      connect_result = ESP_OK;
      DiagReportStep(&ctx,
                     step++,
                     total_steps,
                     "connect",
                     ESP_OK,
                     "skipped: no SSID configured");
    } else if (init_result != ESP_OK) {
      DiagReportStep(&ctx,
                     step++,
                     total_steps,
                     "connect",
                     ESP_ERR_INVALID_STATE,
                     "skipped: init failed");
    } else {
      connect_result =
        WifiManagerConnectSta(creds.ssid, creds.password, 30000);
      connected = (connect_result == ESP_OK);
      connect_after = CaptureHeapSnapshot();
      DiagHeapCheck(&ctx, "post_connect");

      if (connected) {
        esp_netif_ip_info_t ip_info;
        memset(&ip_info, 0, sizeof(ip_info));
        esp_err_t ip_result = WifiManagerGetIpInfo(&ip_info);

        wifi_ap_record_t ap_info;
        memset(&ap_info, 0, sizeof(ap_info));
        const esp_err_t ap_info_result = esp_wifi_sta_get_ap_info(&ap_info);

        char ip[16] = { 0 }, mask[16] = { 0 }, gw[16] = { 0 };
        esp_ip4addr_ntoa(&ip_info.ip, ip, sizeof(ip));
        esp_ip4addr_ntoa(&ip_info.netmask, mask, sizeof(mask));
        esp_ip4addr_ntoa(&ip_info.gw, gw, sizeof(gw));

        DiagReportStep(&ctx,
                       step++,
                       total_steps,
                       "connect",
                       (ip_result == ESP_OK) ? ESP_OK : ip_result,
                       "ip=%s netmask=%s gw=%s rssi=%d ch=%u heap8_before=%u heap8_after=%u min_free=%u",
                       ip,
                       mask,
                       gw,
                       (ap_info_result == ESP_OK) ? ap_info.rssi : 0,
                       (ap_info_result == ESP_OK) ? (unsigned)ap_info.primary : 0U,
                       (unsigned)connect_before.free_8bit,
                       (unsigned)connect_after.free_8bit,
                       (unsigned)connect_after.min_free);
      } else {
        const wifi_err_reason_t reason = WifiManagerLastDisconnectReason();
        const int attempts = WifiManagerLastConnectAttempts();
        DiagReportStep(&ctx,
                       step++,
                       total_steps,
                       "connect",
                       connect_result,
                       "attempts=%d reason=%d (%s) heap8_before=%u heap8_after=%u min_free=%u",
                       attempts,
                       (int)reason,
                       ReasonToString(reason),
                       (unsigned)connect_before.free_8bit,
                       (unsigned)connect_after.free_8bit,
                       (unsigned)connect_after.min_free);
      }
    }
    PrintHeapSnapshot(&ctx, "connect_before", &connect_before);
    PrintHeapSnapshot(&ctx, "connect_after", &connect_after);
  }

  if (dns_lookup) {
    heap_snapshot_t dns_before = CaptureHeapSnapshot();
    heap_snapshot_t dns_after = dns_before;
    DiagHeapCheck(&ctx, "pre_dns");
    if (!connect) {
      DiagReportStep(&ctx,
                     step++,
                     total_steps,
                     "dns",
                     ESP_OK,
                     "skipped: connect not requested");
    } else if (!connected) {
      DiagReportStep(&ctx,
                     step++,
                     total_steps,
                     "dns",
                     ESP_ERR_INVALID_STATE,
                     "skipped: not connected");
    } else {
      const char* host = PickDnsHost();
      struct addrinfo hints;
      memset(&hints, 0, sizeof(hints));
      hints.ai_family = AF_UNSPEC;
      struct addrinfo* results = NULL;
      const int gai_err = getaddrinfo(host, NULL, &hints, &results);
      dns_after = CaptureHeapSnapshot();
      DiagHeapCheck(&ctx, "post_dns");
      if (gai_err != 0 || results == NULL) {
        DiagReportStep(&ctx,
                       step++,
                       total_steps,
                       "dns",
                       ESP_FAIL,
                       "host=%s err=%d heap8_before=%u heap8_after=%u min_free=%u",
                       host,
                       gai_err,
                       (unsigned)dns_before.free_8bit,
                       (unsigned)dns_after.free_8bit,
                       (unsigned)dns_after.min_free);
      } else {
        char addr_text[INET6_ADDRSTRLEN] = { 0 };
        int addr_count = 0;
        for (struct addrinfo* it = results; it != NULL && addr_count < 3;
             it = it->ai_next, ++addr_count) {
          void* addr_ptr = NULL;
          if (it->ai_family == AF_INET) {
            addr_ptr = &((struct sockaddr_in*)it->ai_addr)->sin_addr;
          } else if (it->ai_family == AF_INET6) {
            addr_ptr = &((struct sockaddr_in6*)it->ai_addr)->sin6_addr;
          }
          if (addr_ptr != NULL) {
            inet_ntop(it->ai_family, addr_ptr, addr_text, sizeof(addr_text));
          }
        }
        freeaddrinfo(results);
        const char* resolved = (addr_text[0] != '\0') ? addr_text : "<none>";
        DiagReportStep(&ctx,
                       step++,
                       total_steps,
                       "dns",
                       ESP_OK,
                       "host=%s resolved=%s heap8_before=%u heap8_after=%u min_free=%u",
                       host,
                       resolved,
                       (unsigned)dns_before.free_8bit,
                       (unsigned)dns_after.free_8bit,
                       (unsigned)dns_after.min_free);
      }
    }
    PrintHeapSnapshot(&ctx, "dns_before", &dns_before);
    PrintHeapSnapshot(&ctx, "dns_after", &dns_after);
  }

  heap_snapshot_t teardown_before = CaptureHeapSnapshot();
  DiagHeapCheck(&ctx, "pre_teardown");
  esp_err_t teardown_result = ESP_OK;
  if (!keep_connected) {
    teardown_result = WifiServiceStop();
  }
  heap_snapshot_t teardown_after = CaptureHeapSnapshot();
  DiagHeapCheck(&ctx, "post_teardown");
  DiagReportStep(&ctx,
                 step++,
                 total_steps,
                 "teardown",
                 teardown_result,
                 "keep_connected=%s heap8_before=%u heap8_after=%u min_free=%u",
                 YesNo(keep_connected),
                 (unsigned)teardown_before.free_8bit,
                 (unsigned)teardown_after.free_8bit,
                 (unsigned)teardown_after.min_free);
  PrintHeapSnapshot(&ctx, "teardown_before", &teardown_before);
  PrintHeapSnapshot(&ctx, "teardown_after", &teardown_after);

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
