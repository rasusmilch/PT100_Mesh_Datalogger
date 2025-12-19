#include "diagnostics/diag_wifi.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>

#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "sdkconfig.h"
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

  const int total_steps = 2 + (scan ? 1 : 0) + (connect ? 1 : 0) +
                          (dns_lookup ? 1 : 0);
  int step = 1;

  const bool runtime_running = RuntimeIsRunning();
  DiagReportStep(&ctx,
                 step++,
                 total_steps,
                 "runtime idle",
                 runtime_running ? ESP_ERR_INVALID_STATE : ESP_OK,
                 runtime_running ? "Stop run mode first: run stop" : "idle");
  if (runtime_running) {
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  esp_err_t init_result = WifiManagerInit();
  DiagReportStep(&ctx,
                 step++,
                 total_steps,
                 "init",
                 init_result,
                 "started=%s creds=%s",
                 WifiManagerIsStarted() ? "yes" : "no",
                 CredsSourceToString(creds.source));
  if (init_result != ESP_OK) {
    DiagPrintSummary(&ctx, total_steps);
    (void)WifiManagerDeinit();
    return 1;
  }

  wifi_ap_record_t ap_records[20];
  memset(ap_records, 0, sizeof(ap_records));
  size_t ap_count = 0;
  esp_err_t scan_result = ESP_OK;
  if (scan) {
    scan_result = WifiManagerScan(ap_records, 20, &ap_count);
    bool ssid_present = false;
    if (scan_result == ESP_OK && has_ssid) {
      for (size_t i = 0; i < ap_count; ++i) {
        if (strncmp((const char*)ap_records[i].ssid, creds.ssid, sizeof(ap_records[i].ssid)) == 0) {
          ssid_present = true;
          break;
        }
      }
    }

    DiagReportStep(&ctx,
                   step++,
                   total_steps,
                   "scan",
                   scan_result,
                   "aps=%u ssid_present=%s",
                   (unsigned)ap_count,
                   has_ssid ? (ssid_present ? "yes" : "no") : "n/a");
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
  esp_err_t connect_result = ESP_OK;
  if (connect) {
    if (!has_ssid) {
      DiagReportStep(
        &ctx, step++, total_steps, "connect", ESP_OK, "skipped: no SSID configured");
    } else {
      connect_result = WifiManagerConnectSta(creds.ssid, creds.password, 30000);
      connected = (connect_result == ESP_OK);

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
                       "ip=%s netmask=%s gw=%s rssi=%d ch=%u",
                       ip,
                       mask,
                       gw,
                       (ap_info_result == ESP_OK) ? ap_info.rssi : 0,
                       (ap_info_result == ESP_OK) ? (unsigned)ap_info.primary
                                                  : 0U);
      } else {
        const wifi_err_reason_t reason = WifiManagerLastDisconnectReason();
        const int attempts = WifiManagerLastConnectAttempts();
        DiagReportStep(&ctx,
                       step++,
                       total_steps,
                       "connect",
                       connect_result,
                       "attempts=%d reason=%d (%s)",
                       attempts,
                       (int)reason,
                       ReasonToString(reason));
      }
    }
  }

  if (dns_lookup) {
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
      if (gai_err != 0 || results == NULL) {
        DiagReportStep(&ctx,
                       step++,
                       total_steps,
                       "dns",
                       ESP_FAIL,
                       "host=%s err=%d", host, gai_err);
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
                       "host=%s resolved=%s",
                       host,
                       resolved);
      }
    }
  }

  if (!keep_connected) {
    (void)WifiManagerDisconnectSta();
  }
  (void)WifiManagerDeinit();

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
