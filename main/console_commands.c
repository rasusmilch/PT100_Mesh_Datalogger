#include "console_commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "app_net_config.h"
#include "argtable3/argtable3.h"
#include "boot_mode.h"
#include "calibration.h"
#include "diagnostics/diag_fram.h"
#include "diagnostics/diag_mesh.h"
#include "diagnostics/diag_rtc.h"
#include "diagnostics/diag_rtd.h"
#include "diagnostics/diag_sd.h"
#include "diagnostics/diag_storage.h"
#include "diagnostics/diag_wifi.h"
#include "display_attention.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_console.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "net_supervisor.h"
#include "runtime_health.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include <fcntl.h>
#include <unistd.h>
#endif

#include "console_alerts.h"
#include "esp_log.h"
#include "esp_system.h"
#include "linenoise/linenoise.h"
#include "mem_guard.h"
#include "runtime_manager.h"
#include "sd_card_detect.h"
#include "sdkconfig.h"
#include "time_sync.h"
#include "units_gpio.h"
#include "wifi_credentials.h"
#include "wifi_manager.h"
#include "wifi_service.h"

static const char* kTag = "console";
static void
FormatFileTime(const time_t* timestamp, char* buffer, size_t buffer_size);
/**
 * @brief Execute MaybePushCalRawSampleFromSensor.
 */
static void
MaybePushCalRawSampleFromSensor(void)
{
  if (RuntimeIsRunning()) {
    return;
  }

  const app_runtime_t* runtime = RuntimeGetRuntime();
  if (runtime == NULL || runtime->sensor == NULL) {
    return;
  }

  max31865_sample_t sample = { 0 };
  esp_err_t read_result = Max31865ReadOnce(runtime->sensor, &sample);
  if (read_result != ESP_OK) {
    return;
  }

  if (sample.fault_present) {
    return;
  }

  int32_t raw_milli_c = (int32_t)llround(sample.temperature_c * 1000.0);
  CalWindowPushRawSample(raw_milli_c);
}

static app_runtime_t* g_runtime = NULL;
static app_boot_mode_t g_boot_mode = APP_BOOT_MODE_DIAGNOSTICS;

/**
 * @brief Execute BootModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
BootModeToString(app_boot_mode_t mode)
{
  return (mode == APP_BOOT_MODE_RUN) ? "run" : "diagnostics";
}

/**
 * @brief Execute DisplayAttentionItemToName.
 * @param item Parameter item.
 * @return Return the function result.
 */
static const char*
DisplayAttentionItemToName(display_attention_item_t item)
{
  switch (item) {
    case kDispAttnItemSdOut:
      return "sdout";
    case kDispAttnItemSdIo:
      return "sdio";
    case kDispAttnItemFramOvr:
      return "framovr";
    case kDispAttnItemRtdFault:
      return "rtd";
    case kDispAttnItemTimeBad:
      return "time";
    case kDispAttnItemMeshDown:
      return "mesh";
    case kDispAttnItemHeap:
      return "heap";
    default:
      return "unknown";
  }
}

static void
NotifyNetSupervisor(void)
{
  NetSupervisorNotifyUpdate();
}

static void
FormatPermille(uint16_t permille, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  const unsigned int whole = permille / 1000u;
  const unsigned int frac = permille % 1000u;
  snprintf(buffer, buffer_size, "%u.%03u", whole, frac);
}

static void
PrintRtdEmaSettings(const app_settings_t* settings,
                    const max31865_reader_t* reader)
{
  if (settings == NULL) {
    return;
  }
  char alpha_buffer[8] = { 0 };
  FormatPermille(
    settings->rtd_ema_alpha_permille, alpha_buffer, sizeof(alpha_buffer));
  printf("rtd_ema_enabled: %s\n", settings->rtd_ema_enabled ? "yes" : "no");
  printf("rtd_ema_alpha: %s\n", alpha_buffer);
  if (reader != NULL) {
    printf("rtd_ema_valid: %s\n", reader->ema_valid ? "yes" : "no");
  }
}

/**
 * @brief Execute ParseDisplayAttentionName.
 * @param value Parameter value.
 * @param item_out Parameter item_out.
 * @return Return the function result.
 */
static bool
ParseDisplayAttentionName(const char* value, display_attention_item_t* item_out)
{
  if (value == NULL || item_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "sdout") == 0) {
    *item_out = kDispAttnItemSdOut;
    return true;
  }
  if (strcasecmp(value, "sdio") == 0) {
    *item_out = kDispAttnItemSdIo;
    return true;
  }
  if (strcasecmp(value, "framovr") == 0) {
    *item_out = kDispAttnItemFramOvr;
    return true;
  }
  if (strcasecmp(value, "rtd") == 0) {
    *item_out = kDispAttnItemRtdFault;
    return true;
  }
  if (strcasecmp(value, "time") == 0) {
    *item_out = kDispAttnItemTimeBad;
    return true;
  }
  if (strcasecmp(value, "mesh") == 0) {
    *item_out = kDispAttnItemMeshDown;
    return true;
  }
  if (strcasecmp(value, "heap") == 0) {
    *item_out = kDispAttnItemHeap;
    return true;
  }
  return false;
}

/**
 * @brief Execute WifiServiceModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
WifiServiceModeToString(wifi_service_mode_t mode)
{
  switch (mode) {
    case WIFI_SERVICE_MODE_NONE:
      return "none";
    case WIFI_SERVICE_MODE_DIAGNOSTIC_STA:
      return "diagnostic_sta";
    case WIFI_SERVICE_MODE_MESH:
      return "mesh";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute WifiAuthModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
WifiAuthModeToString(wifi_auth_mode_t mode)
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

/**
 * @brief Execute WifiDisconnectReasonToString.
 * @param reason Parameter reason.
 * @return Return the function result.
 */
static const char*
WifiDisconnectReasonToString(wifi_err_reason_t reason)
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

/**
 * @brief Execute PrintWifiUsage.
 */
static void
PrintWifiUsage(void)
{
  printf(
    "usage:\n"
    "  wifi help\n"
    "  wifi show\n"
    "  wifi set <ssid> [password]\n"
    "  wifi clear\n"
    "  wifi scan [--max N]\n"
    "  wifi status\n"
    "  wifi connect [--timeout_ms T]\n"
    "  wifi disconnect\n"
    "  wifi cfg show\n"
    "  wifi cfg defaults\n"
    "  wifi cfg set sntp <server>\n"
    "  wifi cfg set mesh_chan <1..13>\n"
    "  wifi cfg set mesh_id <XX:XX:XX:XX:XX:XX>\n"
    "  wifi cfg set mesh_ap_pass <password>\n"
    "  wifi cfg set no_router 0|1\n"
    "  wifi cfg set time_sync_s <seconds>\n"
    "  wifi ntp status\n"
    "  wifi ntp sync [--server host] [--timeout_ms T] [--update-rtc 0|1]\n"
    "\n"
    "notes:\n"
    "  - wifi cfg set ... writes to NVS and persists across reboot.\n"
    "  - wifi cfg defaults clears NVS overrides (revert to Kconfig defaults).\n"
    "  - Changes are applied by the network supervisor when possible.\n"
    "  - time_sync_s is clamped to 5..3600 seconds.\n"
    "  - wifi ntp sync performs a one-off sync; it does not change NVS\n"
    "    unless you also use wifi cfg set sntp/time_sync_s.\n");
}

// In diagnostics/console mode, Wi-Fi may not be initialized yet. Most Wi-Fi
// manager operations (scan/connect/disconnect) require the Wi-Fi service to be
// acquired so the netif/event loop/Wi-Fi driver are ready.
//
// This helper acquires the diagnostic STA mode if Wi-Fi is currently idle.
// If the system is running Wi-Fi for another mode (e.g., mesh), we refuse to
// mutate it from the console.
static esp_err_t
AcquireWifiForConsole(bool* did_acquire)
{
  if (did_acquire != NULL) {
    *did_acquire = false;
  }

  (void)WifiServiceInitOnce();

  const wifi_service_mode_t active_mode = WifiServiceActiveMode();
  if (active_mode != WIFI_SERVICE_MODE_NONE &&
      active_mode != WIFI_SERVICE_MODE_DIAGNOSTIC_STA) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t result = WifiServiceAcquire(WIFI_SERVICE_MODE_DIAGNOSTIC_STA);
  if (result == ESP_OK && did_acquire != NULL) {
    *did_acquire = true;
  }
  return result;
}

static void
ReleaseWifiForConsoleIfNeeded(bool did_acquire)
{
  if (did_acquire) {
    (void)WifiServiceRelease();
  }
}

/**
 * @brief Execute PrintWifiConfig.
 */
static void
PrintWifiConfig(void)
{
  const uint8_t mesh_channel = AppNetConfigGetMeshChannel();
  const char* mesh_id = AppNetConfigGetMeshIdString();
  const bool mesh_id_valid = (mesh_id != NULL && mesh_id[0] != '\0');
  const char* ap_password = AppNetConfigGetMeshApPassword();
  const size_t ap_password_len = strlen(ap_password);
  const char* sntp_server = AppNetConfigGetSntpServer();

  printf("mesh_channel: %u\n", (unsigned)mesh_channel);
  printf("mesh_channel_source: %s\n",
         AppNetConfigMeshChannelIsOverridden() ? "nvs" : "kconfig");
  printf("mesh_id: %s\n", mesh_id_valid ? mesh_id : "<invalid>");
  printf("mesh_id_source: %s\n",
         AppNetConfigMeshIdIsOverridden() ? "nvs" : "kconfig");
  if (ap_password_len == 0) {
    printf("mesh_ap_password: <empty>\n");
  } else {
    printf("mesh_ap_password: <redacted>\n");
  }
  printf("mesh_ap_password_len: %u\n", (unsigned)ap_password_len);
  printf("mesh_ap_password_source: %s\n",
         AppNetConfigMeshApPasswordIsOverridden() ? "nvs" : "kconfig");
  printf("mesh_no_router: %u\n", AppNetConfigGetMeshDisableRouter() ? 1u : 0u);
  printf("mesh_no_router_source: %s\n",
         AppNetConfigMeshDisableRouterIsOverridden() ? "nvs" : "kconfig");

  printf("sntp_server: %s\n",
         (sntp_server[0] != '\0') ? sntp_server : "<empty>");
  printf("sntp_server_source: %s\n",
         AppNetConfigSntpServerIsOverridden() ? "nvs" : "kconfig");
  printf("time_sync_s: %u\n", (unsigned)AppNetConfigGetTimeSyncPeriodSeconds());
  printf("time_sync_s_source: %s\n",
         AppNetConfigTimeSyncPeriodIsOverridden() ? "nvs" : "kconfig");
}

/**
 * @brief Execute PickDefaultSntpServer.
 * @return Return the function result.
 */
static const char*
PickDefaultSntpServer(void)
{
  const char* server = AppNetConfigGetSntpServer();
  return (server[0] != '\0') ? server : "pool.ntp.org";
}

/**
 * @brief Execute PrintDisplayAttentionPolicy.
 * @param policy Parameter policy.
 */
static void
PrintDisplayAttentionPolicy(uint32_t policy)
{
  const display_attention_item_t items[] = {
    kDispAttnItemSdOut,    kDispAttnItemSdIo,    kDispAttnItemFramOvr,
    kDispAttnItemRtdFault, kDispAttnItemTimeBad, kDispAttnItemMeshDown,
    kDispAttnItemHeap,
  };
  printf("display_attention_policy: 0x%08" PRIX32 "\n", policy);
  for (size_t idx = 0; idx < sizeof(items) / sizeof(items[0]); ++idx) {
    const display_attention_item_t item = items[idx];
    const display_attention_severity_t severity =
      DisplayAttentionPolicyGet(policy, item);
    const char* label = "off";
    if (severity == DISP_SEV_WARN) {
      label = "warn";
    } else if (severity == DISP_SEV_ERROR) {
      label = "error";
    }
    printf("  %s: %s\n", DisplayAttentionItemToName(item), label);
  }
}

/**
 * @brief Execute SaveDisplayAttentionPolicy.
 * @param policy Parameter policy.
 * @return Return the function result.
 */
static int
SaveDisplayAttentionPolicy(uint32_t policy)
{
  if (g_runtime == NULL) {
    return 1;
  }
  const esp_err_t result = AppSettingsSaveDisplayAttentionPolicy(policy);
  if (result != ESP_OK) {
    printf("save failed: %s\n", esp_err_to_name(result));
    return 1;
  }
  g_runtime->settings->display_attention_policy = policy;
  g_runtime->settings->display_attention_mask =
    AppSettingsGetDisplayAttentionMask();

  // Ensure the display task reflects policy changes immediately.
  RuntimeSetDisplayAttentionPolicy(policy);
  printf("OK\n");
  PrintDisplayAttentionPolicy(policy);
  return 0;
}

/**
 * @brief Execute ParseDisplayAttentionSeverity.
 * @param value Parameter value.
 * @param severity_out Parameter severity_out.
 * @return Return the function result.
 */
static bool
ParseDisplayAttentionSeverity(const char* value,
                              display_attention_severity_t* severity_out)
{
  if (value == NULL || severity_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "off") == 0) {
    *severity_out = DISP_SEV_OFF;
    return true;
  }
  if (strcasecmp(value, "warn") == 0) {
    *severity_out = DISP_SEV_WARN;
    return true;
  }
  if (strcasecmp(value, "error") == 0) {
    *severity_out = DISP_SEV_ERROR;
    return true;
  }
  return false;
}

/**
 * @brief Execute CalibrationModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
static const char*
CalibrationModeToString(calibration_fit_mode_t mode)
{
  switch (mode) {
    case CAL_FIT_MODE_LINEAR:
      return "linear";
    case CAL_FIT_MODE_PIECEWISE:
      return "piecewise";
    case CAL_FIT_MODE_POLY:
      return "poly";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute SaveCalibrationWithContext.
 * @param model Parameter model.
 * @return Return the function result.
 */
static esp_err_t
SaveCalibrationWithContext(const calibration_model_t* model)
{
  if (g_runtime == NULL || g_runtime->sensor == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  calibration_context_t context;
  AppSettingsBuildCalibrationContextFromReader(&context, g_runtime->sensor);
  return AppSettingsSaveCalibrationWithContext(model, &context);
}

/**
 * @brief Execute CommandStatus.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandStatus(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  if (g_runtime == NULL) {
    return 1;
  }

  const app_settings_t* settings = g_runtime->settings;

  printf("node_id: %s\n", g_runtime->node_id_string);
  printf("runtime_running: %s\n", RuntimeIsRunning() ? "yes" : "no");
  printf("time_valid: %s\n", TimeSyncIsSystemTimeValid() ? "yes" : "no");
  printf("log_period_ms: %u\n", (unsigned)settings->log_period_ms);
  printf("sd_flush_period_ms: %u\n", (unsigned)settings->sd_flush_period_ms);
  printf("sd_batch_target_bytes: %u\n",
         (unsigned)settings->sd_batch_bytes_target);
  printf("node_role: %s\n", AppSettingsRoleToString(settings->node_role));
  printf("net_mode: %s\n", AppSettingsNetModeToString(settings->net_mode));
  const app_net_mode_t effective_net_mode =
    (settings->node_role == APP_NODE_ROLE_ROOT) ? APP_NET_MODE_MESH
                                                : settings->net_mode;
  if (effective_net_mode != settings->net_mode) {
    printf("net_mode_effective: %s\n",
           AppSettingsNetModeToString(effective_net_mode));
  }
  printf("allow_children: %s\n", settings->allow_children ? "yes" : "no");
  printf("tz_posix: %s\n", settings->tz_posix);
  printf("dst_enabled: %s\n", settings->dst_enabled ? "yes" : "no");
  printf("mqtt_enabled: %s\n", settings->mqtt_enabled ? "yes" : "no");
  printf("mqtt_broker_uri: %s\n", settings->mqtt_broker_uri);
  printf("mqtt_topic_prefix: %s\n", settings->mqtt_topic_prefix);
  printf("mqtt_qos: %u\n", (unsigned)settings->mqtt_qos);
  printf("mqtt_retain: %s\n", settings->mqtt_retain ? "yes" : "no");
  if (settings->node_role == APP_NODE_ROLE_ROOT) {
    printf("bridge_mode: %s\n",
           AppSettingsMqttBridgeModeToString(settings->mqtt_bridge_mode));
    const bool broker_bridge_active =
      settings->mqtt_enabled &&
      (settings->mqtt_bridge_mode == MQTT_BRIDGE_BROKER ||
       settings->mqtt_bridge_mode == MQTT_BRIDGE_BOTH);
    printf("broker_bridge_active: %s\n", broker_bridge_active ? "yes" : "no");
  }
  PrintRtdEmaSettings(settings, g_runtime->sensor);

  // Ensure the TZ rules are loaded before formatting local time.
  // (TZ is applied via AppSettingsApplyTimeZone() at boot and by the tz/dst
  // commands.)
  tzset();

  const time_t now = time(NULL);

  struct tm utc_time;
  char utc_buffer[48] = { 0 };
  if (gmtime_r(&now, &utc_time) != NULL) {
    strftime(utc_buffer, sizeof(utc_buffer), "%Y-%m-%d %H:%M:%SZ", &utc_time);
  }
  printf("utc_time: %s (epoch=%ld)\n",
         (utc_buffer[0] != '\0') ? utc_buffer : "unknown",
         (long)now);

  struct tm local_time;
  char local_buffer[48] = { 0 };
  if (localtime_r(&now, &local_time) != NULL) {
    strftime(
      local_buffer, sizeof(local_buffer), "%Y-%m-%d %H:%M:%S", &local_time);
  }

  // Compute UTC offset in seconds (local = UTC + offset).
  // Avoid relying on non-portable tm_gmtoff.
  struct tm utc_as_local = utc_time;
  utc_as_local.tm_isdst = -1;
  const time_t utc_epoch_as_local = mktime(&utc_as_local);
  long utc_offset_sec = 0;
  if (utc_epoch_as_local != (time_t)-1) {
    utc_offset_sec = (long)difftime(now, utc_epoch_as_local);
  }

  printf("local_time: %s (utc_offset_sec=%ld dst_in_effect=%d)\n",
         (local_buffer[0] != '\0') ? local_buffer : "unknown",
         utc_offset_sec,
         local_time.tm_isdst);
  const size_t fram_count = FramLogGetCountRecords(g_runtime->fram_log);
  const size_t fram_capacity = FramLogGetCapacityRecords(g_runtime->fram_log);
  const uint32_t fram_fill_pct =
    (fram_capacity > 0) ? (uint32_t)((fram_count * 100u) / fram_capacity) : 0u;
  const uint64_t fram_overrun_total =
    FramLogGetOverrunRecordsTotal(g_runtime->fram_log);
  printf("fram_count: %zu\n", fram_count);
  printf("fram_capacity: %zu\n", fram_capacity);
  printf("fram_fill_pct: %u\n", (unsigned)fram_fill_pct);
  printf("fram_overrun_records_total: %" PRIu64 "\n", fram_overrun_total);
  printf("fram_flush_watermark_records: %u\n",
         (unsigned)settings->fram_flush_watermark_records);
  const bool fram_full =
    (g_runtime->fram_full != NULL) ? *g_runtime->fram_full : false;
  printf("fram_full: %s\n", fram_full ? "yes" : "no");
  printf("fram_count/seq: %u/%u\n",
         (unsigned)FramLogGetBufferedRecords(g_runtime->fram_log),
         (unsigned)FramLogNextSequence(g_runtime->fram_log));
  runtime_state_t* state = RuntimeGetState();
  if (state != NULL) {
    printf("fram_corrupt_detect_count: %u\n",
           (unsigned)state->fram_corrupt_detect_count);
    printf("fram_corrupt_skip_count: %u\n",
           (unsigned)state->fram_corrupt_skip_count);
    printf("fram_corrupt_last_reason: %s\n",
           FramLogValidateResultToString(state->fram_corrupt_last_reason));
    printf("fram_corrupt_last_slot: %u\n",
           (unsigned)state->fram_corrupt_last_slot);
    printf("fram_corrupt_last_addr: 0x%04x\n",
           (unsigned)state->fram_corrupt_last_addr);
    printf("fram_corrupt_last_magic: 0x%08" PRIx32 "\n",
           state->fram_corrupt_last_magic);
    printf("fram_corrupt_last_schema: %u\n",
           (unsigned)state->fram_corrupt_last_schema);
    printf("fram_corrupt_last_exp_crc: 0x%04x\n",
           (unsigned)state->fram_corrupt_last_exp_crc);
    printf("fram_corrupt_last_act_crc: 0x%04x\n",
           (unsigned)state->fram_corrupt_last_act_crc);
  }
  const uint32_t export_dropped = (g_runtime->export_dropped_count != NULL)
                                    ? *g_runtime->export_dropped_count
                                    : 0u;
  const uint32_t export_write_fail =
    (g_runtime->export_write_fail_count != NULL)
      ? *g_runtime->export_write_fail_count
      : 0u;
  printf("data_csv_drop_count: %u\n", (unsigned)export_dropped);
  printf("data_csv_write_fail_count: %u\n", (unsigned)export_write_fail);
  const uint32_t export_drop_count =
    (g_runtime->export_drop_count != NULL) ? *g_runtime->export_drop_count : 0u;
  const uint32_t export_send_fail_count =
    (g_runtime->export_send_fail_count != NULL)
      ? *g_runtime->export_send_fail_count
      : 0u;
  const uint32_t broker_drop_count =
    (g_runtime->broker_drop_count != NULL) ? *g_runtime->broker_drop_count : 0u;
  const uint32_t broker_send_fail_count =
    (g_runtime->broker_send_fail_count != NULL)
      ? *g_runtime->broker_send_fail_count
      : 0u;
  const runtime_cached_status_t* cached_status = RuntimeGetCachedStatus();
  runtime_health_snapshot_t health = { 0 };
  if (state != NULL) {
    RuntimeHealthRead(&state->health_cache, &health);
  }
  const bool mqtt_connected = (g_runtime->mqtt_client_connected != NULL)
                                ? *g_runtime->mqtt_client_connected
                                : false;
  UBaseType_t export_outbox_used = 0;
  if (g_runtime->export_outbox != NULL && *g_runtime->export_outbox != NULL) {
    export_outbox_used = uxQueueMessagesWaiting(*g_runtime->export_outbox);
  }
  printf("mqtt_connected: %s\n", mqtt_connected ? "yes" : "no");
  printf("export_outbox_depth/used: %u/%u\n",
         (unsigned)CONFIG_APP_EXPORT_OUTBOX_DEPTH,
         (unsigned)export_outbox_used);
  printf("export_drop_count: %u\n", (unsigned)export_drop_count);
  printf("export_send_fail_count: %u\n", (unsigned)export_send_fail_count);
  if (settings->node_role == APP_NODE_ROLE_ROOT) {
    UBaseType_t broker_outbox_used = 0;
    if (g_runtime->broker_outbox != NULL && *g_runtime->broker_outbox != NULL) {
      broker_outbox_used = uxQueueMessagesWaiting(*g_runtime->broker_outbox);
    }
    printf("broker_outbox_depth/used: %u/%u\n",
           (unsigned)CONFIG_APP_BROKER_OUTBOX_DEPTH,
           (unsigned)broker_outbox_used);
    printf("broker_drop_count: %u\n", (unsigned)broker_drop_count);
    printf("broker_send_fail_count: %u\n", (unsigned)broker_send_fail_count);
    if (cached_status != NULL) {
      printf("root_publish_consumer_active: %s\n",
             cached_status->root_publish_consumer_active ? "yes" : "no");
      printf("root_publish_drop_no_consumer: %u\n",
             (unsigned)cached_status->root_publish_drop_no_consumer);
    }
  }

  printf("calibration: mode=%s degree=%u coeffs=[%.9g, %.9g, %.9g, %.9g]\n",
         CalibrationModeToString(settings->calibration.mode),
         (unsigned)settings->calibration.degree,
         settings->calibration.coefficients[0],
         settings->calibration.coefficients[1],
         settings->calibration.coefficients[2],
         settings->calibration.coefficients[3]);

  const bool sd_mounted = g_runtime->sd_logger->is_mounted;
  const bool sd_degraded = RuntimeSdIsDegraded();
  const uint32_t sd_fail_count = RuntimeSdFailCount();
  const TickType_t now_ticks = xTaskGetTickCount();
  const uint32_t sd_backoff_until = RuntimeSdBackoffUntilTicks();
  uint32_t sd_backoff_remaining_ms = 0;
  if (sd_backoff_until != 0 && now_ticks < (TickType_t)sd_backoff_until) {
    sd_backoff_remaining_ms =
      (uint32_t)pdTICKS_TO_MS((TickType_t)sd_backoff_until - now_ticks);
  }
  const char* sd_card_present = "unknown";
  const char* sd_safe_to_remove = "unknown";
  if (cached_status != NULL) {
    sd_card_present = cached_status->sd_card_present ? "yes" : "no";
    sd_safe_to_remove = cached_status->sd_safe_to_remove ? "yes" : "no";
  }
  printf("sd_mounted: %s\n", sd_mounted ? "yes" : "no");
  printf("sd_card_present: %s\n", sd_card_present);
  printf("sd_safe_to_remove: %s\n", sd_safe_to_remove);
  printf("sd_card_detect_gpio: %d\n", CONFIG_APP_SD_CARD_DETECT_GPIO);
  printf("sd_card_detect_present_level: %s\n",
#if CONFIG_APP_SD_CARD_DETECT_PRESENT_HIGH
         "high"
#else
         "low"
#endif
  );
  printf("sd_degraded: %s\n", sd_degraded ? "yes" : "no");
  printf("sd_fail_count: %u\n", (unsigned)sd_fail_count);
  printf("sd_backoff_remaining_ms: %u\n", (unsigned)sd_backoff_remaining_ms);
  printf("sd_last_record_id: %" PRIu64 "\n",
         SdLoggerLastRecordIdOnSd(g_runtime->sd_logger));
#if CONFIG_APP_HEAP_MONITOR_ENABLE
  printf("heap_internal_free_bytes: %u\n",
         (unsigned)health.heap_internal_free_bytes);
  printf("heap_internal_largest_free_block_bytes: %u\n",
         (unsigned)health.heap_internal_largest_free_block_bytes);
  printf("heap_internal_min_free_bytes: %u\n",
         (unsigned)health.heap_internal_min_free_bytes);
  printf("heap_internal_min_largest_free_block_bytes: %u\n",
         (unsigned)health.heap_internal_min_largest_free_block_bytes);
  printf("heap_internal_frag_percent: %u\n",
         (unsigned)health.heap_internal_frag_percent);
  printf("heap_internal_warn: %s\n", health.heap_internal_warn ? "yes" : "no");
  printf("heap_internal_crit: %s\n", health.heap_internal_crit ? "yes" : "no");
#if CONFIG_APP_HEAP_PSRAM_MONITOR_ENABLE
  printf("heap_psram_free_bytes: %u\n", (unsigned)health.heap_psram_free_bytes);
  printf("heap_psram_largest_free_block_bytes: %u\n",
         (unsigned)health.heap_psram_largest_free_block_bytes);
  printf("heap_psram_min_free_bytes: %u\n",
         (unsigned)health.heap_psram_min_free_bytes);
  printf("heap_psram_min_largest_free_block_bytes: %u\n",
         (unsigned)health.heap_psram_min_largest_free_block_bytes);
  printf("heap_psram_frag_percent: %u\n",
         (unsigned)health.heap_psram_frag_percent);
  printf("heap_psram_warn: %s\n", health.heap_psram_warn ? "yes" : "no");
  printf("heap_psram_crit: %s\n", health.heap_psram_crit ? "yes" : "no");
#endif
#endif
  printf("allocs_since_run: %" PRIu64 "\n", MemGuardGetAllocCountSinceRun());
  printf("mesh_connected: %s\n",
         MeshTransportIsConnected(g_runtime->mesh) ? "yes" : "no");
  const bool wifi_connected = WifiManagerIsConnected();
  printf("wifi_connected: %s\n", wifi_connected ? "yes" : "no");
  if (wifi_connected) {
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    if (WifiManagerGetIpInfo(&ip_info) == ESP_OK) {
      char ip[16] = { 0 };
      esp_ip4addr_ntoa(&ip_info.ip, ip, sizeof(ip));
      printf("wifi_ip: %s\n", ip);
    } else {
      printf("wifi_ip: n/a\n");
    }
  } else {
    printf("wifi_ip: n/a\n");
  }
  printf("cal_points: %u\n",
         (unsigned)g_runtime->settings->calibration_points_count);
  return 0;
}

/**
 * @brief Execute CommandRtd.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRtd(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: rtd show | rtd ema show | rtd ema on|off | rtd ema alpha "
           "<0.0..1.0>\n");
    return 1;
  }

  app_settings_t* settings = g_runtime->settings;
  const char* action = argv[1];

  if (strcmp(action, "show") == 0) {
    PrintRtdEmaSettings(settings, g_runtime->sensor);
    return 0;
  }

  if (strcmp(action, "ema") != 0) {
    printf("usage: rtd show | rtd ema show | rtd ema on|off | rtd ema alpha "
           "<0.0..1.0>\n");
    return 1;
  }

  if (argc < 3) {
    printf("usage: rtd ema show | rtd ema on|off | rtd ema alpha <0.0..1.0>\n");
    return 1;
  }

  const char* subaction = argv[2];
  if (strcmp(subaction, "show") == 0) {
    PrintRtdEmaSettings(settings, g_runtime->sensor);
    return 0;
  }

  if (strcmp(subaction, "on") == 0 || strcmp(subaction, "off") == 0) {
    if (argc != 3) {
      printf("usage: rtd ema on|off\n");
      return 1;
    }
    const bool enabled = (strcmp(subaction, "on") == 0);
    settings->rtd_ema_enabled = enabled;
    esp_err_t result = AppSettingsSaveRtdEmaEnabled(enabled);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("rtd_ema_enabled set to %s\n", enabled ? "on" : "off");
    return 0;
  }

  if (strcmp(subaction, "alpha") == 0) {
    if (argc != 4) {
      printf("usage: rtd ema alpha <0.0..1.0>\n");
      return 1;
    }
    char* end = NULL;
    const double value = strtod(argv[3], &end);
    if (end == argv[3] || *end != '\0' || value <= 0.0 || value > 1.0) {
      printf("usage: rtd ema alpha <0.0..1.0>\n");
      return 1;
    }
    long long permille = llround(value * 1000.0);
    if (permille < 1) {
      permille = 1;
    } else if (permille > 1000) {
      permille = 1000;
    }
    const uint16_t permille_u16 = (uint16_t)permille;
    settings->rtd_ema_alpha_permille = permille_u16;
    esp_err_t result = AppSettingsSaveRtdEmaAlphaPermille(permille_u16);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    char alpha_buffer[8] = { 0 };
    FormatPermille(permille_u16, alpha_buffer, sizeof(alpha_buffer));
    printf("rtd_ema_alpha set to %s\n", alpha_buffer);
    return 0;
  }

  printf("usage: rtd show | rtd ema show | rtd ema on|off | rtd ema alpha "
         "<0.0..1.0>\n");
  return 1;
}

static void
PrintStackUsage(void)
{
  printf("usage: stack [show] [--headroom BYTES]\n");
}

/**
 * @brief Execute CommandStack.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandStack(int argc, char** argv)
{
  uint32_t headroom_bytes = 1024;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "show") == 0) {
      continue;
    }
    if (strcmp(argv[i], "--headroom") == 0) {
      if ((i + 1) >= argc) {
        printf("--headroom requires a byte count\n");
        PrintStackUsage();
        return 2;
      }
      headroom_bytes = (uint32_t)strtoul(argv[++i], NULL, 10);
      continue;
    }

    printf("unknown option: %s\n", argv[i]);
    PrintStackUsage();
    return 2;
  }

  RuntimePrintStackMonitor(headroom_bytes);
  return 0;
}

/**
 * @brief Execute CommandDisplay.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandDisplay(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf(
      "usage: disp show | disp units C|F | disp attn ... | disp test [ms]\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "show") == 0) {
    const spi_host_device_t display_host = RuntimeGetDisplaySpiHost();
    const int display_host_id = (display_host == SPI3_HOST) ? 3 : 2;
#if CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
    const bool display_shared = true;
#else
    const bool display_shared = false;
#endif
#if CONFIG_APP_MAX7219_ENABLE
    const bool display_enabled = true;
#else
    const bool display_enabled = false;
#endif
    const app_display_units_t effective_units =
      RuntimeGetEffectiveDisplayUnits();
    printf("display_units: %s (effective: %s)\n",
           AppSettingsDisplayUnitsToString(g_runtime->settings->display_units),
           AppSettingsDisplayUnitsToString(effective_units));
    printf("max7219_enabled: %s\n", display_enabled ? "yes" : "no");
    printf("max7219_spi_host: %d%s\n",
           display_host_id,
           display_shared ? " (shared)" : "");
    printf("max7219_chain_len: %d\n", CONFIG_APP_MAX7219_CHAIN_LEN);
    printf("max7219_intensity: %d\n", CONFIG_APP_MAX7219_INTENSITY);
    printf("max7219_pins: mosi=%d sclk=%d cs=%d%s\n",
           RuntimeGetDisplayMosiGpio(),
           RuntimeGetDisplaySclkGpio(),
           RuntimeGetDisplayCsGpio(),
           display_shared ? " (shared)" : "");
    PrintDisplayAttentionPolicy(AppSettingsGetDisplayAttentionPolicy());
    return 0;
  }

  if (strcmp(action, "units") == 0) {
    if (argc < 3) {
      printf("usage: disp units C|F\n");
      return 1;
    }
    app_display_units_t units = APP_DISPLAY_UNITS_F;
    if (!AppSettingsParseDisplayUnits(argv[2], &units)) {
      printf("usage: disp units C|F\n");
      return 1;
    }
    g_runtime->settings->display_units = units;
    esp_err_t result = AppSettingsSaveDisplayUnits(units);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("display_units set to %s\n", AppSettingsDisplayUnitsToString(units));
    return 0;
  }

  if (strcmp(action, "attn") == 0) {
    if (argc < 3) {
      printf("usage: disp attn show|set|defaults|ack\n");
      return 1;
    }
    const char* attn_action = argv[2];
    if (strcmp(attn_action, "show") == 0) {
      PrintDisplayAttentionPolicy(AppSettingsGetDisplayAttentionPolicy());
      return 0;
    }
    if (strcmp(attn_action, "set") == 0) {
      if (argc < 5) {
        printf("usage: disp attn set <name> <off|warn|error>\n");
        return 1;
      }
      display_attention_item_t item = kDispAttnItemSdOut;
      if (!ParseDisplayAttentionName(argv[3], &item)) {
        printf(
          "unknown name. valid: sdout, sdio, framovr, rtd, time, mesh, heap\n");
        return 1;
      }
      display_attention_severity_t severity = DISP_SEV_OFF;
      if (!ParseDisplayAttentionSeverity(argv[4], &severity)) {
        printf("unknown severity. valid: off, warn, error\n");
        return 1;
      }
      uint32_t policy = AppSettingsGetDisplayAttentionPolicy();
      policy = DisplayAttentionPolicySet(policy, item, severity);
      return SaveDisplayAttentionPolicy(policy);
    }
    if (strcmp(attn_action, "defaults") == 0) {
      const uint32_t policy = AppSettingsDefaultDisplayAttentionPolicy();
      return SaveDisplayAttentionPolicy(policy);
    }
    if (strcmp(attn_action, "ack") == 0) {
      if (RuntimeAcknowledgeDisplayAttention(kDispAttnItemFramOvr)) {
        printf("OK\n");
        return 0;
      }
      printf("ack failed\n");
      return 1;
    }
    printf("unknown action. usage: disp attn show|set|defaults|ack\n");
    return 1;
  }

  if (strcmp(action, "test") == 0) {
    uint32_t duration_ms = 2000u;
    if (argc >= 3) {
      char* end = NULL;
      unsigned long parsed = strtoul(argv[2], &end, 10);
      if (end == argv[2] || *end != '\0') {
        printf("usage: disp test [ms]\n");
        return 1;
      }
      duration_ms = (uint32_t)parsed;
    }
    esp_err_t result = RuntimeShowDisplayTestPattern(duration_ms);
    if (result != ESP_OK) {
      printf("display test failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("display test pattern for %u ms\n", (unsigned)duration_ms);
    return 0;
  }

  printf(
    "unknown action. usage: disp show | disp units C|F | disp attn ... | disp "
    "test [ms]\n");
  return 1;
}

static void
PrintUnitsUsage(void)
{
  printf(
    "usage: units set C|F | units gpio show | units gpio set pin <n> | "
    "units gpio set pull <up|down|none> | units gpio set c_level <high|low>\n");
}

/**
 * @brief Execute CommandUnits.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandUnits(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    PrintUnitsUsage();
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "set") == 0) {
    if (argc < 3) {
      PrintUnitsUsage();
      return 1;
    }
    app_display_units_t units = APP_DISPLAY_UNITS_F;
    if (!AppSettingsParseDisplayUnits(argv[2], &units)) {
      PrintUnitsUsage();
      return 1;
    }
    g_runtime->settings->display_units = units;
    esp_err_t result = AppSettingsSaveDisplayUnits(units);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("display_units set to %s\n", AppSettingsDisplayUnitsToString(units));
    return 0;
  }

  if (strcmp(action, "gpio") == 0) {
    if (argc < 3) {
      PrintUnitsUsage();
      return 1;
    }
    const char* gpio_action = argv[2];
    if (strcmp(gpio_action, "show") == 0) {
      units_gpio_status_t status = { 0 };
      UnitsGpioGetStatus(&status);
      printf("units_gpio_enabled: %s\n", status.enabled ? "yes" : "no");
      printf("units_gpio_pin: %ld%s\n",
             (long)status.pin,
             status.pin_valid ? "" : " (invalid)");
      printf("units_gpio_pull: %s\n", UnitsGpioPullToString(status.pull));
      printf("units_gpio_c_level: %s\n",
             UnitsGpioLevelToString(status.c_level_high));
      printf("units_gpio_level: %s\n",
             status.pin_valid ? UnitsGpioLevelToString(status.last_level_high)
                              : "n/a");
      printf("units_gpio_effective: %s\n",
             AppSettingsDisplayUnitsToString(status.effective_units));
      printf(
        "units_saved: %s\n",
        AppSettingsDisplayUnitsToString(g_runtime->settings->display_units));
      return 0;
    }
    if (strcmp(gpio_action, "set") == 0) {
      if (argc < 5) {
        PrintUnitsUsage();
        return 1;
      }
      const char* field = argv[3];
      if (strcmp(field, "pin") == 0) {
        char* end = NULL;
        long pin = strtol(argv[4], &end, 10);
        if (end == argv[4] || *end != '\0') {
          PrintUnitsUsage();
          return 1;
        }
        esp_err_t result = AppSettingsSaveUnitsGpioPin((int32_t)pin);
        if (result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(result));
          return 1;
        }
        g_runtime->settings->units_gpio_pin = (int32_t)pin;
        UnitsGpioApplySettings(g_runtime->settings);
        printf("units_gpio_pin set to %ld\n", pin);
        return 0;
      }
      if (strcmp(field, "pull") == 0) {
        app_units_gpio_pull_t pull = APP_UNITS_GPIO_PULL_NONE;
        if (!UnitsGpioParsePull(argv[4], &pull)) {
          PrintUnitsUsage();
          return 1;
        }
        esp_err_t result = AppSettingsSaveUnitsGpioPull(pull);
        if (result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(result));
          return 1;
        }
        g_runtime->settings->units_gpio_pull = pull;
        UnitsGpioApplySettings(g_runtime->settings);
        printf("units_gpio_pull set to %s\n", UnitsGpioPullToString(pull));
        return 0;
      }
      if (strcmp(field, "c_level") == 0) {
        bool level_high = false;
        if (!UnitsGpioParseLevel(argv[4], &level_high)) {
          PrintUnitsUsage();
          return 1;
        }
        esp_err_t result = AppSettingsSaveUnitsGpioCLevel(level_high);
        if (result != ESP_OK) {
          printf("save failed: %s\n", esp_err_to_name(result));
          return 1;
        }
        g_runtime->settings->units_gpio_c_level_high = level_high;
        UnitsGpioApplySettings(g_runtime->settings);
        printf("units_gpio_c_level set to %s\n",
               UnitsGpioLevelToString(level_high));
        return 0;
      }
      PrintUnitsUsage();
      return 1;
    }
    PrintUnitsUsage();
    return 1;
  }

  PrintUnitsUsage();
  return 1;
}

/**
 * @brief Execute CommandRaw.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRaw(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  if (g_runtime == NULL) {
    return 1;
  }

  max31865_sample_t sample;
  esp_err_t result = Max31865ReadOnce(g_runtime->sensor, &sample);
  if (result != ESP_OK) {
    printf("read failed: %s\n", esp_err_to_name(result));
    return 1;
  }

  const double calibrated = CalibrationModelEvaluateWithPoints(
    &g_runtime->settings->calibration,
    sample.temperature_c,
    g_runtime->settings->calibration_points,
    g_runtime->settings->calibration_points_count);
  char fault[64] = { 0 };
  Max31865FormatFault(sample.fault_status, fault, sizeof(fault));

  printf("adc_code_15: %u\n", (unsigned)sample.adc_code);
  printf("resistance_ohm: %.3f\n", sample.resistance_ohm);
  printf("temp_raw_c: %.3f\n", sample.temperature_c);
  printf("temp_cal_c: %.3f\n", calibrated);
  printf("fault: %s (0x%02x)\n", fault, (unsigned)sample.fault_status);
  return 0;
}

/**
 * @brief Execute FlushAllRecordsToSd.
 * @param runtime Parameter runtime.
 * @return Return the function result.
 */
static esp_err_t
FlushAllRecordsToSd(app_runtime_t* runtime)
{
  if (runtime->flush_callback == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return runtime->flush_callback(runtime->flush_context);
}

/**
 * @brief Execute FlushOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
FlushOp(app_runtime_t* runtime, void* ctx)
{
  (void)ctx;
  return FlushAllRecordsToSd(runtime);
}

/**
 * @brief Execute FormatRecordFlags.
 * @param flags Parameter flags.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
static void
FormatRecordFlags(uint16_t flags, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  out[0] = '\0';
  const struct
  {
    uint16_t flag;
    const char* name;
  } entries[] = {
    { LOG_RECORD_FLAG_TIME_VALID, "time_valid" },
    { LOG_RECORD_FLAG_CAL_VALID, "cal_valid" },
    { LOG_RECORD_FLAG_SD_ERROR, "sd_error" },
    { LOG_RECORD_FLAG_MESH_CONNECTED, "mesh_connected" },
    { LOG_RECORD_FLAG_SENSOR_FAULT, "sensor_fault" },
    { LOG_RECORD_FLAG_FRAM_FULL, "fram_full" },
    { LOG_RECORD_FLAG_RTD_EMA, "rtd_ema" },
  };

  size_t used = 0;
  bool first = true;
  for (size_t i = 0; i < sizeof(entries) / sizeof(entries[0]); ++i) {
    if ((flags & entries[i].flag) == 0) {
      continue;
    }
    const char* separator = first ? "" : ",";
    const int written =
      snprintf(out + used, out_size - used, "%s%s", separator, entries[i].name);
    if (written < 0 || (size_t)written >= out_size - used) {
      out[out_size - 1] = '\0';
      return;
    }
    used += (size_t)written;
    first = false;
  }

  if (first) {
    snprintf(out, out_size, "none");
  }
}

/**
 * @brief Execute CommandFlush.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandFlush(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  if (g_runtime == NULL) {
    return 1;
  }

  esp_err_t result = ESP_OK;
  if (RuntimeIsRunning()) {
    result = FlushAllRecordsToSd(g_runtime);
  } else {
    result = RuntimeWithTemporarySdMount(&FlushOp, NULL);
  }
  if (result != ESP_OK) {
    printf("flush failed: %s\n", esp_err_to_name(result));
    return 1;
  }
  return 0;
}

/**
 * @brief Execute CommandFram.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandFram(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: fram status | fram show | fram clear\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "status") != 0 && strcmp(action, "show") != 0 &&
      strcmp(action, "clear") != 0) {
    printf(
      "unknown fram command. try 'fram status | fram show | fram clear'\n");
    return 1;
  }

  fram_log_status_t status;
  esp_err_t result = FramLogGetStatus(g_runtime->fram_log, &status);
  if (result != ESP_OK) {
    printf("fram: not initialized\n");
    return 1;
  }
  status.flush_watermark_records =
    g_runtime->settings->fram_flush_watermark_records;

  printf("fram: mounted=%s full=%s\n",
         status.mounted ? "yes" : "no",
         status.full ? "yes" : "no");
  printf("fram: cap=%u rec_size=%u watermark=%u\n",
         (unsigned)status.capacity_records,
         (unsigned)status.record_size_bytes,
         (unsigned)status.flush_watermark_records);
  printf("fram: write=%u read=%u count=%u seq=%u id=%" PRIu64 "\n",
         (unsigned)status.write_index_abs,
         (unsigned)status.read_index_abs,
         (unsigned)status.buffered_count,
         (unsigned)status.next_sequence,
         status.next_record_id);
  if (strcmp(action, "clear") == 0) {
    result = FramLogReset(g_runtime->fram_log);
    if (result != ESP_OK) {
      printf("fram clear failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    if (g_runtime->fram_full != NULL) {
      *g_runtime->fram_full = false;
    }
    printf("fram: cleared\n");
    return 0;
  }
  if (strcmp(action, "show") == 0) {
    const uint32_t buffered = FramLogGetBufferedRecords(g_runtime->fram_log);
    const uint64_t last_sd_id = SdLoggerLastRecordIdOnSd(g_runtime->sd_logger);
    printf("fram: buffered=%u last_sd_record_id=%" PRIu64 "\n",
           (unsigned)buffered,
           last_sd_id);
    for (uint32_t offset = 0; offset < buffered; ++offset) {
      log_record_t record;
      const esp_err_t peek_result =
        FramLogPeekOffset(g_runtime->fram_log, offset, &record);
      if (peek_result == ESP_ERR_NOT_FOUND) {
        break;
      }
      const bool corrupted = (peek_result == ESP_ERR_INVALID_RESPONSE);
      if (peek_result != ESP_OK && !corrupted) {
        printf("fram show failed at offset %u: %s\n",
               (unsigned)offset,
               esp_err_to_name(peek_result));
        return 1;
      }

      if (corrupted) {
        uint16_t actual_crc = 0;
        const fram_log_validate_result_t validate_result =
          FramLogValidateRecord(&record, &actual_crc);
        printf(
          "record[%u]: corrupt=yes reason=%s exp_crc=0x%04x act_crc=0x%04x\n",
          (unsigned)offset,
          FramLogValidateResultToString(validate_result),
          (unsigned)record.crc16_ccitt,
          (unsigned)actual_crc);
        continue;
      }

      char time_string[32] = "unknown";
      if (record.timestamp_epoch_sec > 0) {
        const time_t epoch = (time_t)record.timestamp_epoch_sec;
        FormatFileTime(&epoch, time_string, sizeof(time_string));
      }

      char flags_string[128];
      FormatRecordFlags(record.flags, flags_string, sizeof(flags_string));
      const bool pending = record.record_id > last_sd_id;

      printf("record[%u]: id=%" PRIu64 " seq=%u pending=%s corrupt=no\n",
             (unsigned)offset,
             record.record_id,
             (unsigned)record.sequence,
             pending ? "yes" : "no");
      printf("  time: %s.%03d epoch=%" PRIi64 "\n",
             time_string,
             (int)record.timestamp_millis,
             record.timestamp_epoch_sec);
      printf("  temp: raw=%.3fC cal=%.3fC resistance=%.3f ohm\n",
             record.raw_temp_milli_c / 1000.0,
             record.temp_milli_c / 1000.0,
             record.resistance_milli_ohm / 1000.0);
      printf("  flags: 0x%04x [%s]\n", (unsigned)record.flags, flags_string);
    }
  }
  return 0;
}

/**
 * @brief Execute PrintSdStatus.
 * @param runtime Parameter runtime.
 */
static void
PrintSdStatus(const app_runtime_t* runtime)
{
  const sd_logger_t* logger = runtime->sd_logger;
  const bool sd_mounted = logger->is_mounted;
  const bool sd_degraded = RuntimeSdIsDegraded();
  const uint32_t sd_fail_count = RuntimeSdFailCount();
  const TickType_t now_ticks = xTaskGetTickCount();
  const uint32_t sd_backoff_until = RuntimeSdBackoffUntilTicks();
  uint32_t sd_backoff_remaining_ms = 0;
  if (sd_backoff_until != 0 && now_ticks < (TickType_t)sd_backoff_until) {
    sd_backoff_remaining_ms =
      (uint32_t)pdTICKS_TO_MS((TickType_t)sd_backoff_until - now_ticks);
  }

  printf("sd_mounted: %s\n", sd_mounted ? "yes" : "no");
  printf("sd_degraded: %s\n", sd_degraded ? "yes" : "no");
  printf("sd_fail_count: %u\n", (unsigned)sd_fail_count);
  printf("sd_backoff_remaining_ms: %u\n", (unsigned)sd_backoff_remaining_ms);
  runtime_state_t* state = RuntimeGetState();
  if (state != NULL) {
    printf("sd_verify_readback: %s\n",
           state->sd_verify_readback ? "on" : "off");
  }
  printf("sd_last_record_id: %" PRIu64 "\n", SdLoggerLastRecordIdOnSd(logger));
  if (sd_mounted) {
    printf("sd_mount_point: %s\n", logger->mount_point);
    if (logger->card != NULL) {
      const uint64_t size_bytes = (uint64_t)logger->card->csd.capacity *
                                  (uint64_t)logger->card->csd.sector_size;
      printf("sd_card_name: %s\n", logger->card->cid.name);
      printf("sd_card_size_mb: %llu\n",
             (unsigned long long)(size_bytes / (1024ULL * 1024ULL)));
    }
  }
}

/**
 * @brief Execute FormatFileTime.
 * @param timestamp Parameter timestamp.
 * @param buffer Parameter buffer.
 * @param buffer_size Parameter buffer_size.
 */
static void
FormatFileTime(const time_t* timestamp, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  struct tm time_info;
  if (timestamp == NULL || gmtime_r(timestamp, &time_info) == NULL) {
    buffer[0] = '\0';
    return;
  }
  strftime(buffer, buffer_size, "%Y-%m-%d %H:%M:%SZ", &time_info);
}

/**
 * @brief Execute CommandSdView.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
static int
CommandSdView(const sd_logger_t* logger)
{
  if (!logger->is_mounted) {
    printf("sd not mounted\n");
    return 1;
  }

  runtime_state_t* state = RuntimeGetState();
  if (state == NULL || !RuntimeSdIoLock(state, pdMS_TO_TICKS(2000))) {
    printf("sd view failed: sd io lock timeout\n");
    return 1;
  }

  DIR* dir = opendir(logger->mount_point);
  if (dir == NULL) {
    RuntimeSdIoUnlock(state);
    printf("sd view failed: %s\n", strerror(errno));
    return 1;
  }

  printf("sd files in %s:\n", logger->mount_point);
  struct dirent* entry = NULL;
  int file_count = 0;
  while ((entry = readdir(dir)) != NULL) {
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
      continue;
    }
    // FATFS long file names may be up to 255 chars. Include mount point, '/',
    // and NUL terminator. Keep this comfortably above worst-case.
    char path[512];
    int path_length =
      snprintf(path, sizeof(path), "%s/%s", logger->mount_point, entry->d_name);
    if (path_length < 0 || path_length >= (int)sizeof(path)) {
      printf("  %s (path too long)\n", entry->d_name);
      continue;
    }
    struct stat info;
    if (stat(path, &info) != 0) {
      printf("  %s (stat failed: %s)\n", entry->d_name, strerror(errno));
      continue;
    }
    char time_buffer[32];
    FormatFileTime(&info.st_mtime, time_buffer, sizeof(time_buffer));
    const char* kind = S_ISDIR(info.st_mode) ? "dir" : "file";
    printf("  %s %-4s size=%llu mtime=%s\n",
           entry->d_name,
           kind,
           (unsigned long long)info.st_size,
           time_buffer);
    ++file_count;
  }
  closedir(dir);
  RuntimeSdIoUnlock(state);
  printf("sd file count: %d\n", file_count);
  return 0;
}

/**
 * @brief Execute SdStatusOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdStatusOp(app_runtime_t* runtime, void* ctx)
{
  (void)ctx;
  PrintSdStatus(runtime);
  return ESP_OK;
}

/**
 * @brief Execute SdViewOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdViewOp(app_runtime_t* runtime, void* ctx)
{
  (void)ctx;
  return (CommandSdView(runtime->sd_logger) == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Execute SdFormatOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdFormatOp(app_runtime_t* runtime, void* ctx)
{
  (void)ctx;
  runtime_state_t* state = RuntimeGetState();
  if (state == NULL || !RuntimeSdIoLock(state, pdMS_TO_TICKS(2000))) {
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t result = SdLoggerFormatDestructive(runtime->sd_logger);
  RuntimeSdIoUnlock(state);
  return result;
}

/**
 * @brief Execute SdMountOp.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
SdMountOp(app_runtime_t* runtime, void* ctx)
{
  (void)runtime;
  (void)ctx;
  return ESP_OK;
}

/**
 * @brief Execute CommandSd.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandSd(int argc, char** argv)
{
  if (g_runtime == NULL || g_runtime->sd_logger == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: sd status|mount|unmount|format|view|verify\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "status") == 0) {
    if (!heap_caps_check_integrity_all(true)) {
      printf("heap integrity check failed (heap corrupted)\r\n");
      return 1;
    }

    if (!RuntimeIsRunning()) {
      esp_err_t result = RuntimeWithTemporarySdMount(&SdStatusOp, NULL);
      if (result != ESP_OK) {
        printf("sd status failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      return 0;
    }
    PrintSdStatus(g_runtime);
    return 0;
  }

  if (strcmp(action, "view") == 0) {
    if (!RuntimeIsRunning()) {
      esp_err_t result = RuntimeWithTemporarySdMount(&SdViewOp, NULL);
      if (result != ESP_OK) {
        printf("sd view failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      return 0;
    }
    return CommandSdView(g_runtime->sd_logger);
  }

  if (strcmp(action, "verify") == 0) {
    runtime_state_t* state = RuntimeGetState();
    if (state == NULL) {
      printf("sd verify failed: runtime unavailable\n");
      return 1;
    }
    if (argc < 3) {
      printf("sd verify: %s\n", state->sd_verify_readback ? "on" : "off");
      return 0;
    }
    const char* value = argv[2];
    if (strcmp(value, "on") == 0) {
      state->sd_verify_readback = true;
      printf("sd verify: on\n");
      return 0;
    }
    if (strcmp(value, "off") == 0) {
      state->sd_verify_readback = false;
      printf("sd verify: off\n");
      return 0;
    }
    printf("usage: sd verify on|off\n");
    return 1;
  }

  if (RuntimeIsRunning()) {
    printf("Stop run mode first: run stop\n");
    return 1;
  }

  if (strcmp(action, "mount") == 0) {
    esp_err_t result = RuntimeWithTemporarySdMount(&SdMountOp, NULL);
    if (result != ESP_OK) {
      printf("sd mount failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd mounted (temporary)\n");
    return 0;
  }

  if (strcmp(action, "unmount") == 0) {
    esp_err_t result = RuntimeSdUnmountNow();
    if (result != ESP_OK) {
      printf("sd unmount failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd unmounted\n");
    return 0;
  }

  if (strcmp(action, "format") == 0) {
    esp_err_t result = RuntimeWithTemporarySdMount(&SdFormatOp, NULL);
    if (result != ESP_OK) {
      printf("sd format failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd format complete (FAT filesystem recreated)\n");
    return 0;
  }

  printf(
    "unknown sd command. try 'sd status|mount|unmount|format|view|verify'\n");
  return 1;
}

// NOTE: The "log" command uses manual argv parsing.
//
// We intentionally do NOT use argtable for this command because argtable's
// positional parsing cannot express "subcommand + single positional value"
// without accidentally binding the value to the wrong field.
//
// Example of the problem with argtable positional arguments:
//   "log flush_period 300000"
// would bind "300000" to the first positional integer field, not the one
// associated with "flush_period".
//
// Manual parsing keeps the CLI simple and predictable.

static struct
{
  struct arg_str* action;
  struct arg_str* mode_value;
  struct arg_end* end;
} g_mode_args;

static struct
{
  struct arg_str* action;
  struct arg_end* end;
} g_data_args;

static struct
{
  struct arg_str* action;
  struct arg_end* end;
} g_run_args;

static struct
{
  struct arg_str* action;
  struct arg_str* posix;
  struct arg_end* end;
} g_tz_args;

static struct
{
  struct arg_str* action;
  struct arg_str* local_time;
  struct arg_int* is_dst;
  struct arg_end* end;
} g_time_args;

static struct
{
  struct arg_str* action;
  struct arg_int* enabled;
  struct arg_end* end;
} g_dst_args;

static struct
{
  struct arg_str* action;
  struct arg_str* role;
  struct arg_end* end;
} g_role_args;

static struct
{
  struct arg_str* action;
  struct arg_str* mode;
  struct arg_end* end;
} g_net_args;

static struct
{
  struct arg_str* action;
  struct arg_int* enabled;
  struct arg_end* end;
} g_children_args;

/**
 * @brief Execute CommandLog.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandLog(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }

  if (argc < 2) {
    printf("usage: log interval <ms> | log watermark <records> | log "
           "flush_period <ms> | log batch <bytes> | log show\n");
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "flush_ms") == 0) {
    // Backwards/typo-friendly alias.
    action = "flush_period";
  }
  if (strcmp(action, "interval") == 0) {
    if (argc != 3) {
      printf("usage: log interval <ms>\n");
      return 1;
    }
    char* end = NULL;
    long interval_ms_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid interval\n");
      return 1;
    }
    const int interval_ms = (int)interval_ms_long;
    if (interval_ms < 100 || interval_ms > 3600000) {
      printf("invalid interval\n");
      return 1;
    }
    g_runtime->settings->log_period_ms = (uint32_t)interval_ms;
    esp_err_t result = AppSettingsSaveLogPeriodMs((uint32_t)interval_ms);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("log_period_ms set to %d\n", interval_ms);
    return 0;
  }

  if (strcmp(action, "watermark") == 0) {
    if (argc != 3) {
      printf("usage: log watermark <records>\n");
      return 1;
    }
    char* end = NULL;
    long watermark_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid watermark\n");
      return 1;
    }
    const int watermark = (int)watermark_long;
    if (watermark < 1) {
      printf("invalid watermark\n");
      return 1;
    }
    g_runtime->settings->fram_flush_watermark_records = (uint32_t)watermark;
    esp_err_t result =
      AppSettingsSaveFramFlushWatermarkRecords((uint32_t)watermark);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("fram flush watermark set to %d\n", watermark);
    return 0;
  }

  if (strcmp(action, "flush_period") == 0) {
    if (argc != 3) {
      printf("usage: log flush_period <ms>\n");
      return 1;
    }
    char* end = NULL;
    long period_ms_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid period\n");
      return 1;
    }
    const int period_ms = (int)period_ms_long;
    if (period_ms < 1000) {
      printf("invalid period\n");
      return 1;
    }
    g_runtime->settings->sd_flush_period_ms = (uint32_t)period_ms;
    esp_err_t result = AppSettingsSaveSdFlushPeriodMs((uint32_t)period_ms);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd_flush_period_ms set to %d\n", period_ms);
    return 0;
  }

  if (strcmp(action, "batch") == 0) {
    if (argc != 3) {
      printf("usage: log batch <bytes>\n");
      return 1;
    }
    char* end = NULL;
    long batch_bytes_long = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
      printf("invalid batch size\n");
      return 1;
    }
    const int batch_bytes = (int)batch_bytes_long;
    if (batch_bytes < 4096) {
      printf("invalid batch size\n");
      return 1;
    }
    g_runtime->settings->sd_batch_bytes_target = (uint32_t)batch_bytes;
    esp_err_t result = AppSettingsSaveSdBatchBytes((uint32_t)batch_bytes);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("sd batch target set to %d bytes\n", batch_bytes);
    return 0;
  }

  if (strcmp(action, "show") == 0) {
    printf("log_period_ms: %u\n", (unsigned)g_runtime->settings->log_period_ms);
    printf("fram_flush_watermark_records: %u\n",
           (unsigned)g_runtime->settings->fram_flush_watermark_records);
    printf("sd_flush_period_ms: %u\n",
           (unsigned)g_runtime->settings->sd_flush_period_ms);
    printf("sd_batch_target_bytes: %u\n",
           (unsigned)g_runtime->settings->sd_batch_bytes_target);
    return 0;
  }

  printf("unknown action. usage: log interval <ms> | log watermark <records> | "
         "log flush_period <ms> | log batch <bytes> | log show\n");
  return 1;
}

static struct
{
  struct arg_str* action;
  struct arg_dbl* raw_c;
  struct arg_dbl* actual_c;
  struct arg_int* every_ms;
  struct arg_int* seconds;
  struct arg_dbl* stable_stddev_c;
  struct arg_int* min_seconds;
  struct arg_int* timeout_seconds;
  struct arg_str* mode;
  struct arg_lit* allow_wide_slope;
  struct arg_end* end;
} g_cal_args;

/**
 * @brief Execute CommandCal.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandCal(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_cal_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_cal_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  app_settings_t* settings = g_runtime->settings;
  const char* action = g_cal_args.action->sval[0];

  if (strcmp(action, "clear") == 0) {
    CalibrationModelInitIdentity(&settings->calibration);
    settings->calibration_points_count = 0;
    memset(
      settings->calibration_points, 0, sizeof(settings->calibration_points));
    esp_err_t result = SaveCalibrationWithContext(&settings->calibration);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    result = AppSettingsSaveCalibrationPoints(
      settings->calibration_points, settings->calibration_points_count);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("calibration reset to identity (y=x)\n");
    return 0;
  }

  if (strcmp(action, "add") == 0) {
    if (g_cal_args.raw_c->count != 1 || g_cal_args.actual_c->count != 1) {
      printf("usage: cal add <raw_c> <actual_c>\n");
      return 1;
    }
    if (settings->calibration_points_count >= CALIBRATION_MAX_POINTS) {
      printf("already have %u points; run 'cal apply' or 'cal clear'\n",
             (unsigned)settings->calibration_points_count);
      return 1;
    }

    calibration_point_t* point =
      &settings->calibration_points[settings->calibration_points_count];
    point->raw_avg_mC = (int32_t)llround(g_cal_args.raw_c->dval[0] * 1000.0);
    point->actual_mC = (int32_t)llround(g_cal_args.actual_c->dval[0] * 1000.0);
    point->raw_stddev_mC = 0;
    point->sample_count = 0;
    point->time_valid = TimeSyncIsSystemTimeValid() ? 1u : 0u;
    point->timestamp_epoch_sec = point->time_valid ? (int64_t)time(NULL) : 0;
    settings->calibration_points_count++;
    printf("added point %u: raw=%.6f actual=%.6f\n",
           (unsigned)settings->calibration_points_count,
           g_cal_args.raw_c->dval[0],
           g_cal_args.actual_c->dval[0]);
    esp_err_t result = AppSettingsSaveCalibrationPoints(
      settings->calibration_points, settings->calibration_points_count);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    return 0;
  }

  if (strcmp(action, "list") == 0) {
    printf("calibration points (%u):\n",
           (unsigned)settings->calibration_points_count);
    for (size_t index = 0; index < settings->calibration_points_count;
         ++index) {
      const calibration_point_t* point = &settings->calibration_points[index];
      printf("  %u: raw_avg=%.6f actual=%.6f stddev=%.6f samples=%u\n",
             (unsigned)(index + 1),
             point->raw_avg_mC / 1000.0,
             point->actual_mC / 1000.0,
             point->raw_stddev_mC / 1000.0,
             (unsigned)point->sample_count);
    }
    return 0;
  }

  if (strcmp(action, "show") == 0) {
    int32_t last_raw_mC = 0;
    int32_t mean_raw_mC = 0;
    int32_t stddev_mC = 0;
    MaybePushCalRawSampleFromSensor();

    CalWindowGetStats(&last_raw_mC, &mean_raw_mC, &stddev_mC);
    const size_t sample_count = CalWindowGetSampleCount();
    printf("cal_window_raw_last_c: %.3f\n", last_raw_mC / 1000.0);
    printf("cal_window_raw_avg_c: %.3f\n", mean_raw_mC / 1000.0);
    printf("cal_window_raw_stddev_c: %.3f\n", stddev_mC / 1000.0);
    printf("cal_window_samples: %u\n", (unsigned)sample_count);
    printf("cal_window_ready: %s\n", CalWindowIsReady() ? "yes" : "no");
    printf("calibration_mode: %s\n",
           CalibrationModeToString(settings->calibration.mode));
    printf("cal_points: %u (raw_avg_C uses window average)\n",
           (unsigned)settings->calibration_points_count);
    for (size_t index = 0; index < settings->calibration_points_count;
         ++index) {
      const calibration_point_t* point = &settings->calibration_points[index];
      const double raw_avg_c = point->raw_avg_mC / 1000.0;
      const double actual_c = point->actual_mC / 1000.0;
      const double residual_c = actual_c - raw_avg_c;
      printf(
        "  %u: raw_avg_C=%.3f actual_C=%.3f residual_C=%.3f stddev_C=%.3f\n",
        (unsigned)(index + 1),
        raw_avg_c,
        actual_c,
        residual_c,
        point->raw_stddev_mC / 1000.0);
    }
    return 0;
  }

  if (strcmp(action, "apply") == 0) {
    if (settings->calibration_points_count < 1) {
      printf("no points; use 'cal add <raw_c> <actual_c>' first\n");
      return 1;
    }

    calibration_model_t model;
    calibration_fit_options_t options;
    CalibrationFitOptionsInitDefault(&options);
    if (g_cal_args.mode->count > 0) {
      const char* mode = g_cal_args.mode->sval[0];
      if (strcmp(mode, "linear") == 0) {
        options.mode = CAL_FIT_MODE_LINEAR;
      } else if (strcmp(mode, "piecewise") == 0) {
        options.mode = CAL_FIT_MODE_PIECEWISE;
      } else if (strncmp(mode, "poly", 4) == 0) {
        const int degree = atoi(mode + 4);
        if (degree < 1 || degree > CALIBRATION_MAX_DEGREE) {
          printf("invalid poly degree; use poly1..poly%u\n",
                 CALIBRATION_MAX_DEGREE);
          return 1;
        }
        options.mode = CAL_FIT_MODE_POLY;
        options.poly_degree = (uint8_t)degree;
      } else {
        printf("invalid mode; use linear|piecewise|polyN\n");
        return 1;
      }
    }
    options.allow_wide_slope = (g_cal_args.allow_wide_slope->count > 0);

    calibration_fit_diagnostics_t diagnostics = { 0 };
    esp_err_t result = CalibrationModelFitFromPointsWithOptions(
      settings->calibration_points,
      settings->calibration_points_count,
      &options,
      &model,
      &diagnostics);
    if (result != ESP_OK) {
      printf("fit failed: %s\n", esp_err_to_name(result));
      return 1;
    }

    settings->calibration = model;
    result = SaveCalibrationWithContext(&model);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }

    printf("calibration applied: mode=%s degree=%u coeffs=[%.9g, %.9g, %.9g, "
           "%.9g]\n",
           CalibrationModeToString(model.mode),
           (unsigned)model.degree,
           model.coefficients[0],
           model.coefficients[1],
           model.coefficients[2],
           model.coefficients[3]);
    printf("fit diagnostics: rms_error=%.6f max_abs_residual=%.6f\n",
           diagnostics.rms_error_c,
           diagnostics.max_abs_residual_c);

    return 0;
  }

  if (strcmp(action, "live") == 0) {
    const int every_ms =
      (g_cal_args.every_ms->count > 0) ? g_cal_args.every_ms->ival[0] : 500;
    const int seconds =
      (g_cal_args.seconds->count > 0) ? g_cal_args.seconds->ival[0] : 10;
    if (every_ms <= 0 || seconds <= 0) {
      printf("usage: cal live [--every_ms 500] [--seconds 10]\n");
      return 1;
    }

    const int64_t duration_us = (int64_t)seconds * 1000000LL;
    const int64_t start_us = esp_timer_get_time();
    while (esp_timer_get_time() - start_us < duration_us) {
      int32_t last_raw_mC = 0;
      int32_t mean_raw_mC = 0;
      int32_t stddev_mC = 0;
      MaybePushCalRawSampleFromSensor();

      CalWindowGetStats(&last_raw_mC, &mean_raw_mC, &stddev_mC);
      printf("raw_last_C=%.3f raw_avg_C=%.3f raw_stddev_C=%.3f\n",
             last_raw_mC / 1000.0,
             mean_raw_mC / 1000.0,
             stddev_mC / 1000.0);
      vTaskDelay(pdMS_TO_TICKS((uint32_t)every_ms));
    }
    return 0;
  }

  if (strcmp(action, "capture") == 0) {
    if (g_cal_args.actual_c->count != 1) {
      printf("usage: cal capture <actual_temp_c> "
             "[--stable_stddev_c 0.05] [--min_seconds 5] "
             "[--timeout_seconds 120]\n");
      return 1;
    }
    if (settings->calibration_points_count >= CALIBRATION_MAX_POINTS) {
      printf("already have %u points; run 'cal apply' or 'cal clear'\n",
             (unsigned)settings->calibration_points_count);
      return 1;
    }

    const double actual_temp_c = g_cal_args.actual_c->dval[0];
    const double stable_stddev_c = (g_cal_args.stable_stddev_c->count > 0)
                                     ? g_cal_args.stable_stddev_c->dval[0]
                                     : 0.05;
    const int min_seconds =
      (g_cal_args.min_seconds->count > 0) ? g_cal_args.min_seconds->ival[0] : 5;
    const int timeout_seconds = (g_cal_args.timeout_seconds->count > 0)
                                  ? g_cal_args.timeout_seconds->ival[0]
                                  : 120;
    if (stable_stddev_c <= 0.0 || min_seconds <= 0 || timeout_seconds <= 0) {
      printf("usage: cal capture <actual_temp_c> "
             "[--stable_stddev_c 0.05] [--min_seconds 5] "
             "[--timeout_seconds 120]\n");
      return 1;
    }

    const int64_t start_us = esp_timer_get_time();
    int64_t stable_start_us = -1;
    while (esp_timer_get_time() - start_us <
           (int64_t)timeout_seconds * 1000000LL) {
      int32_t last_raw_mC = 0;
      int32_t mean_raw_mC = 0;
      int32_t stddev_mC = 0;
      MaybePushCalRawSampleFromSensor();

      CalWindowGetStats(&last_raw_mC, &mean_raw_mC, &stddev_mC);
      const double stddev_c = stddev_mC / 1000.0;

      if (CalWindowIsReady() && stddev_c <= stable_stddev_c) {
        if (stable_start_us < 0) {
          stable_start_us = esp_timer_get_time();
        }
        if (esp_timer_get_time() - stable_start_us >=
            (int64_t)min_seconds * 1000000LL) {
          calibration_point_t* point =
            &settings->calibration_points[settings->calibration_points_count];
          point->raw_avg_mC = mean_raw_mC;
          point->actual_mC = (int32_t)llround(actual_temp_c * 1000.0);
          point->raw_stddev_mC = stddev_mC;
          point->sample_count = (uint16_t)CalWindowGetSampleCount();
          point->time_valid = TimeSyncIsSystemTimeValid() ? 1u : 0u;
          point->timestamp_epoch_sec =
            point->time_valid ? (int64_t)time(NULL) : 0;
          settings->calibration_points_count++;
          printf("cal capture ok: raw_last=%.3fC raw_avg=%.3fC "
                 "raw_std=%.3fC actual=%.3fC\n",
                 last_raw_mC / 1000.0,
                 mean_raw_mC / 1000.0,
                 stddev_c,
                 actual_temp_c);
          esp_err_t save_result = AppSettingsSaveCalibrationPoints(
            settings->calibration_points, settings->calibration_points_count);
          if (save_result != ESP_OK) {
            printf("save failed: %s\n", esp_err_to_name(save_result));
            return 1;
          }
          return 0;
        }
      } else {
        stable_start_us = -1;
      }

      vTaskDelay(pdMS_TO_TICKS(200));
    }

    printf("cal capture failed: timed out after %d seconds\n", timeout_seconds);
    return 1;
  }

  printf("unknown action. usage: cal clear | cal add <raw_c> <actual_c> | cal "
         "list | cal show | cal apply | cal live [--every_ms 500] [--seconds "
         "10] | cal capture <actual_temp_c> [--stable_stddev_c 0.05] "
         "[--min_seconds 5] [--timeout_seconds 120]\n");
  return 1;
}

/**
 * @brief Execute CommandMode.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandMode(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_mode_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_mode_args.end, argv[0]);
    return 1;
  }

  const char* action = g_mode_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    const app_boot_mode_t stored = BootModeReadFromNvsOrDefault();
    const app_boot_mode_t running = RuntimeIsDataStreamingEnabled()
                                      ? APP_BOOT_MODE_RUN
                                      : APP_BOOT_MODE_DIAGNOSTICS;
    printf("nvs_boot_mode: %s\n", BootModeToString(stored));
    printf("current_mode: %s\n", BootModeToString(running));
    printf("data_streaming: %s\n",
           RuntimeIsDataStreamingEnabled() ? "on" : "off");
    return 0;
  }

  if (strcmp(action, "run") == 0 || strcmp(action, "diag") == 0) {
    const bool run_mode = strcmp(action, "run") == 0;
    if (run_mode) {
      RuntimeSetLogPolicyRun();
      RuntimeEnableDataStreaming(true);
    } else {
      RuntimeSetLogPolicyDiag();
      RuntimeEnableDataStreaming(false);
    }
    printf("mode set to %s\n", run_mode ? "run" : "diag");
    return 0;
  }

  app_boot_mode_t target = APP_BOOT_MODE_DIAGNOSTICS;
  if (strcmp(action, "set") == 0 && g_mode_args.mode_value->count == 1) {
    const char* mode = g_mode_args.mode_value->sval[0];
    if (strcmp(mode, "diag") == 0) {
      target = APP_BOOT_MODE_DIAGNOSTICS;
    } else if (strcmp(mode, "run") == 0) {
      target = APP_BOOT_MODE_RUN;
    } else {
      printf("usage: mode set diag|run\n");
      return 1;
    }
    esp_err_t result = BootModeWriteToNvs(target);
    if (result != ESP_OK) {
      printf("write failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("set; reboot required\n");
    return 0;
  }

  printf("unknown action. usage: mode show | mode run | mode diag | mode set "
         "diag|run\n");
  return 1;
}

/**
 * @brief Execute CommandData.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandData(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_data_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_data_args.end, argv[0]);
    return 1;
  }

  const char* action = g_data_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("data_streaming: %s\n",
           RuntimeIsDataStreamingEnabled() ? "on" : "off");
    return 0;
  }

  if (strcmp(action, "on") == 0) {
    RuntimeEnableDataStreaming(true);
    printf("data streaming enabled\n");
    return 0;
  }

  if (strcmp(action, "off") == 0) {
    RuntimeEnableDataStreaming(false);
    printf("data streaming disabled\n");
    return 0;
  }

  printf("unknown action. usage: data show | data on | data off\n");
  return 1;
}

/**
 * @brief Execute CommandRun.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRun(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_run_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_run_args.end, argv[0]);
    return 1;
  }

  const char* action = g_run_args.action->sval[0];

  if (strcmp(action, "status") == 0) {
    printf("running: %s\n", RuntimeIsRunning() ? "yes" : "no");
    return 0;
  }

  if (strcmp(action, "start") == 0) {
    if (RuntimeIsRunning()) {
      printf("already running\n");
      return 0;
    }
    esp_err_t result = EnterRunMode();
    if (result != ESP_OK) {
      printf("start failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    const runtime_cached_status_t* status = RuntimeGetCachedStatus();
    if (status != NULL) {
      printf("drain: flushed=%ld remaining=%u duration=%u ms result=%s\n",
             status->last_drain_flushed_records,
             (unsigned)status->last_drain_remaining,
             (unsigned)status->last_drain_duration_ms,
             esp_err_to_name((esp_err_t)status->last_drain_result));
      if (status->last_drain_result == ESP_ERR_TIMEOUT) {
        printf("drain timed out; remaining=%u\n",
               (unsigned)status->last_drain_remaining);
      }
    }
    printf("runtime started\n");
    return 0;
  }

  if (strcmp(action, "stop") == 0) {
    if (!RuntimeIsRunning()) {
      printf("already stopped\n");
      return 0;
    }
    esp_err_t result = EnterDiagMode();
    if (result != ESP_OK) {
      printf("stop failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    const runtime_cached_status_t* status = RuntimeGetCachedStatus();
    if (status != NULL) {
      printf("drain: flushed=%ld remaining=%u duration=%u ms result=%s\n",
             status->last_drain_flushed_records,
             (unsigned)status->last_drain_remaining,
             (unsigned)status->last_drain_duration_ms,
             esp_err_to_name((esp_err_t)status->last_drain_result));
      if (status->last_drain_result == ESP_ERR_TIMEOUT) {
        printf("drain timed out; remaining=%u\n",
               (unsigned)status->last_drain_remaining);
      }
    }
    printf("runtime stopped\n");
    return 0;
  }

  printf("unknown action. usage: run status | run start | run stop\n");
  return 1;
}

/**
 * @brief Execute CommandTz.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandTz(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_tz_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_tz_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_tz_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("tz_posix: %s\n", g_runtime->settings->tz_posix);
    printf("dst_enabled: %s\n",
           g_runtime->settings->dst_enabled ? "yes" : "no");
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_tz_args.posix->count != 1) {
      printf("usage: tz set \"<posix>\"\n");
      return 1;
    }
    const char* tz_posix = g_tz_args.posix->sval[0];
    if (tz_posix[0] == '\0' ||
        strlen(tz_posix) >= sizeof(g_runtime->settings->tz_posix)) {
      printf("invalid tz string\n");
      return 1;
    }
    snprintf(g_runtime->settings->tz_posix,
             sizeof(g_runtime->settings->tz_posix),
             "%s",
             tz_posix);
    g_runtime->settings->dst_enabled = (strchr(tz_posix, ',') != NULL);
    esp_err_t result = AppSettingsSaveTimeZone(
      g_runtime->settings->tz_posix, g_runtime->settings->dst_enabled);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    AppSettingsApplyTimeZone(g_runtime->settings);
    printf("tz_posix set to %s\n", g_runtime->settings->tz_posix);
    return 0;
  }

  printf("unknown action. usage: tz show | tz set \"<posix>\"\n");
  return 1;
}

/**
 * @brief Execute PrintTimeUsage.
 */
static void
PrintTimeUsage(void)
{
  printf("time setlocal \"YYYY-MM-DD HH:MM:SS\" [--is_dst 0|1]\n");
  printf(
    "  input is LOCAL wall time; converted to UTC epoch + RTC stored as UTC\n");
  printf("  use --is_dst to disambiguate fall-back hour\n");
}

/**
 * @brief Execute CommandTime.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandTime(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_time_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_time_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_time_args.action->sval[0];
  if (strcmp(action, "setlocal") != 0) {
    PrintTimeUsage();
    return 1;
  }

  if (g_time_args.local_time->count != 1) {
    PrintTimeUsage();
    return 1;
  }

  struct tm tm_local;
  esp_err_t result =
    TimeParseLocalIso(g_time_args.local_time->sval[0], &tm_local);
  if (result != ESP_OK) {
    printf("invalid time format (use YYYY-MM-DD HH:MM:SS)\n");
    return 1;
  }

  if (g_time_args.is_dst->count == 1) {
    const int is_dst = g_time_args.is_dst->ival[0];
    if (is_dst != 0 && is_dst != 1) {
      PrintTimeUsage();
      return 1;
    }
    tm_local.tm_isdst = is_dst;
  }

  time_t epoch_utc = 0;
  bool ambiguous = false;
  result = TimeLocalTmToEpochUtc(&tm_local, &epoch_utc, &ambiguous);
  if (result == ESP_ERR_NOT_SUPPORTED && ambiguous) {
    printf("ambiguous local time; use --is_dst 0|1\n");
    return 1;
  }
  if (result == ESP_ERR_INVALID_STATE) {
    printf("invalid local time (DST gap)\n");
    return 1;
  }
  if (result != ESP_OK) {
    printf("time conversion failed: %s\n", esp_err_to_name(result));
    return 1;
  }

  (void)TimeSyncSetSystemEpoch((int64_t)epoch_utc, false, g_runtime->time_sync);

  bool rtc_ok = false;
  if (g_runtime->time_sync != NULL) {
    rtc_ok = (TimeSyncSetRtcFromSystem(g_runtime->time_sync) == ESP_OK);
  }

  if (g_runtime->mesh != NULL &&
      g_runtime->settings->node_role == APP_NODE_ROLE_ROOT) {
    const esp_err_t mesh_result =
      MeshTransportBroadcastTime(g_runtime->mesh, (int64_t)epoch_utc);
    if (mesh_result != ESP_OK) {
      ESP_LOGW(
        kTag, "mesh time broadcast failed: %s", esp_err_to_name(mesh_result));
    }
  }

  struct tm local_time;
  char local_buffer[32] = { 0 };
  if (localtime_r(&epoch_utc, &local_time) != NULL) {
    strftime(
      local_buffer, sizeof(local_buffer), "%Y-%m-%d %H:%M:%S", &local_time);
  }
  printf("time setlocal ok: local=%s utc_epoch=%" PRId64 " rtc=%s\n",
         (local_buffer[0] != '\0') ? local_buffer : "unknown",
         (int64_t)epoch_utc,
         rtc_ok ? "ok" : "fail");
  return 0;
}

// --- Boiling point calculator (from MeshTemps-LeafNode.ino) ------------------
//
// Usage:
//   boilpt <inHg> [elev_ft]
//
// If elev_ft is provided, <inHg> is treated as sea-level pressure / altimeter
// setting and is converted to station pressure via a simple ISA troposphere
// approximation. If elev_ft is omitted, <inHg> is treated as station pressure.
//
/**
 * @brief Execute StationPressureFromSlpInHg.
 * @param slp_inHg Parameter slp_inHg.
 * @param elev_ft Parameter elev_ft.
 * @return Return the function result.
 */
static float
StationPressureFromSlpInHg(float slp_inHg, float elev_ft)
{
  if (slp_inHg <= 0.0f) {
    return NAN;
  }
  const float height_m = (elev_ft <= 0.0f) ? 0.0f : (elev_ft * 0.3048f);
  const float base = 1.0f - 2.25577e-5f * height_m;
  if (base <= 0.0f) {
    return NAN; // Out of model range.
  }
  return slp_inHg * powf(base, 5.25588f);
}

// Antoine equation (water), pressure in mmHg -> boiling point in °C.
// Validity is approximate; this is intended for quick calibration reference.
/**
 * @brief Execute BoilingPointCFromStationInHg.
 * @param station_inHg Parameter station_inHg.
 * @return Return the function result.
 */
static float
BoilingPointCFromStationInHg(float station_inHg)
{
  if (station_inHg <= 0.0f) {
    return NAN;
  }
  const float pressure_mmHg = station_inHg * 25.4f; // 1 inHg = 25.4 mmHg
  if (pressure_mmHg <= 0.0f) {
    return NAN;
  }

  const float A = 8.07131f;
  const float B = 1730.63f;
  const float C = 233.426f;

  const float log_p = log10f(pressure_mmHg);
  const float denom = A - log_p;
  if (denom == 0.0f) {
    return NAN;
  }
  return B / denom - C;
}

/**
 * @brief Execute CommandBoilPt.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandBoilPt(int argc, char** argv)
{
  if (argc < 2) {
    printf("usage: boilpt <inHg> [elev_ft]\n");
    printf("  Example (altimeter + elevation): boilpt 29.81 1300\n");
    printf("  Example (station pressure):       boilpt 28.90\n");
    return 1;
  }

  char* endptr = NULL;
  const float in_hg = strtof(argv[1], &endptr);
  if (endptr == argv[1] || isnan(in_hg) || in_hg <= 0.0f) {
    printf("ERR invalid inHg\n");
    return 1;
  }

  if (argc >= 3) {
    endptr = NULL;
    const float elev_ft = strtof(argv[2], &endptr);
    if (endptr == argv[2] || isnan(elev_ft)) {
      printf("ERR invalid elev_ft\n");
      return 1;
    }

    const float station_in_hg = StationPressureFromSlpInHg(in_hg, elev_ft);
    const float boil_c = BoilingPointCFromStationInHg(station_in_hg);
    if (isnan(boil_c)) {
      printf("ERR invalid inputs\n");
      return 1;
    }
    const float boil_f = boil_c * 9.0f / 5.0f + 32.0f;

    printf("Boiling point at station %.3f inHg (AS=%.3f, elev=%.0f ft): "
           "%.3f C (%.3f F)\n",
           (double)station_in_hg,
           (double)in_hg,
           (double)elev_ft,
           (double)boil_c,
           (double)boil_f);
    return 0;
  }

  // Back-compat: single arg -> treat as station pressure directly.
  const float boil_c = BoilingPointCFromStationInHg(in_hg);
  if (isnan(boil_c)) {
    printf("ERR invalid pressure\n");
    return 1;
  }
  const float boil_f = boil_c * 9.0f / 5.0f + 32.0f;
  printf("Boiling point at %.3f inHg (station): %.3f C (%.3f F)\n",
         (double)in_hg,
         (double)boil_c,
         (double)boil_f);
  return 0;
}

/**
 * @brief Execute CommandDst.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandDst(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_dst_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_dst_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_dst_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("dst_enabled: %s\n",
           g_runtime->settings->dst_enabled ? "yes" : "no");
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_dst_args.enabled->count != 1) {
      printf("usage: dst set 0|1\n");
      return 1;
    }
    const int enabled = g_dst_args.enabled->ival[0];
    if (enabled != 0 && enabled != 1) {
      printf("usage: dst set 0|1\n");
      return 1;
    }
    g_runtime->settings->dst_enabled = (enabled == 1);
    if (g_runtime->settings->dst_enabled) {
      if (strcmp(g_runtime->settings->tz_posix, APP_SETTINGS_TZ_DEFAULT_STD) ==
          0) {
        snprintf(g_runtime->settings->tz_posix,
                 sizeof(g_runtime->settings->tz_posix),
                 "%s",
                 APP_SETTINGS_TZ_DEFAULT_POSIX);
      }
    } else {
      if (strcmp(g_runtime->settings->tz_posix,
                 APP_SETTINGS_TZ_DEFAULT_POSIX) == 0) {
        snprintf(g_runtime->settings->tz_posix,
                 sizeof(g_runtime->settings->tz_posix),
                 "%s",
                 APP_SETTINGS_TZ_DEFAULT_STD);
      }
    }
    esp_err_t result = AppSettingsSaveTimeZone(
      g_runtime->settings->tz_posix, g_runtime->settings->dst_enabled);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    AppSettingsApplyTimeZone(g_runtime->settings);
    printf("dst_enabled set to %d\n", enabled);
    printf("tz_posix: %s\n", g_runtime->settings->tz_posix);
    return 0;
  }

  printf("unknown action. usage: dst show | dst set 0|1\n");
  return 1;
}

/**
 * @brief Execute CommandRole.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandRole(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_role_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_role_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_role_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("role: %s\n",
           AppSettingsRoleToString(g_runtime->settings->node_role));
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_role_args.role->count != 1) {
      printf("usage: role set root|sensor|relay\n");
      return 1;
    }
    const char* role_value = g_role_args.role->sval[0];
    app_node_role_t role = APP_NODE_ROLE_SENSOR;
    if (!AppSettingsParseRole(role_value, &role)) {
      printf("usage: role set root|sensor|relay\n");
      return 1;
    }

    g_runtime->settings->node_role = role;
    esp_err_t result = AppSettingsSaveNodeRole(role);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }

    if (!g_runtime->settings->allow_children_set) {
      const bool allow_children = AppSettingsRoleDefaultAllowsChildren(role);
      g_runtime->settings->allow_children = allow_children;
      result = AppSettingsSaveAllowChildren(allow_children, false);
      if (result != ESP_OK) {
        printf("save failed: %s\n", esp_err_to_name(result));
        return 1;
      }
    }

    printf("role set to %s\n", AppSettingsRoleToString(role));
    return 0;
  }

  printf("unknown action. usage: role show | role set root|sensor|relay\n");
  return 1;
}

/**
 * @brief Execute CommandNet.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandNet(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_net_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_net_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_net_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("net_mode: %s\n",
           AppSettingsNetModeToString(g_runtime->settings->net_mode));
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_net_args.mode->count != 1) {
      printf("usage: net set mesh|wifi|none\n");
      return 1;
    }
    const char* mode_value = g_net_args.mode->sval[0];
    app_net_mode_t mode = APP_NET_MODE_MESH;
    if (!AppSettingsParseNetMode(mode_value, &mode)) {
      printf("usage: net set mesh|wifi|none\n");
      return 1;
    }
    g_runtime->settings->net_mode = mode;
    esp_err_t result = AppSettingsSaveNetMode(mode);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("OK\n");
    printf("net_mode set to %s\n", AppSettingsNetModeToString(mode));
    NotifyNetSupervisor();
    return 0;
  }

  printf("unknown action. usage: net show | net set mesh|wifi|none\n");
  return 1;
}

/**
 * @brief Execute CommandWifi.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandWifi(int argc, char** argv)
{
  if (argc < 2) {
    PrintWifiUsage();
    return 1;
  }

  const char* action = argv[1];
  if (strcmp(action, "help") == 0) {
    PrintWifiUsage();
    return 0;
  }

  if (strcmp(action, "show") == 0) {
    wifi_credentials_t creds;
    WifiCredentialsLoad(&creds);
    printf("configured: %s\n", creds.has_ssid ? "yes" : "no");
    printf("ssid: %s\n", creds.has_ssid ? creds.ssid : "<unset>");
    if (creds.has_ssid) {
      printf("ssid_source: %s\n", creds.from_nvs ? "nvs" : "kconfig");
    } else {
      printf("ssid_source: none\n");
    }
    if (!creds.has_ssid) {
      printf("password: n/a\n");
    } else if (creds.password[0] == '\0') {
      printf("password: <empty>\n");
    } else {
      char masked[65] = { 0 };
      WifiCredentialsMaskPassword(creds.password, masked, sizeof(masked));
      printf("password: %s\n", masked);
    }
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (argc < 3) {
      printf("usage: wifi set <ssid> [password]\n");
      return 1;
    }
    const char* ssid = argv[2];
    const char* password = (argc >= 4) ? argv[3] : "";
    esp_err_t result = WifiCredentialsSave(ssid, password);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("OK\n");
    NotifyNetSupervisor();
    return 0;
  }

  if (strcmp(action, "clear") == 0) {
    esp_err_t result = WifiCredentialsClear();
    if (result != ESP_OK) {
      printf("clear failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("OK\n");
    return 0;
  }

  if (strcmp(action, "scan") == 0) {
    int max_records = 10;
    for (int i = 2; i < argc; ++i) {
      if (strcmp(argv[i], "--max") == 0) {
        if (i + 1 >= argc) {
          printf("usage: wifi scan [--max N]\n");
          return 1;
        }
        max_records = atoi(argv[++i]);
      } else {
        printf("usage: wifi scan [--max N]\n");
        return 1;
      }
    }

    if (max_records <= 0) {
      printf("max must be > 0\n");
      return 1;
    }

    const size_t kMaxScanRecords = 50;
    size_t record_cap = (size_t)max_records;
    if (record_cap > kMaxScanRecords) {
      record_cap = kMaxScanRecords;
      printf("note: max capped at %u\n", (unsigned)kMaxScanRecords);
    }

    bool did_acquire = false;
    esp_err_t wifi_result = AcquireWifiForConsole(&did_acquire);
    if (wifi_result != ESP_OK) {
      printf("scan failed: %s\n", esp_err_to_name(wifi_result));
      if (wifi_result == ESP_ERR_INVALID_STATE) {
        printf("note: Wi-Fi is active in another mode (%s).\n",
               WifiServiceModeToString(WifiServiceActiveMode()));
      }
      return 1;
    }

    wifi_ap_record_t* records =
      (wifi_ap_record_t*)AppCalloc(record_cap, sizeof(*records));
    if (records == NULL) {
      printf("out of memory\n");
      ReleaseWifiForConsoleIfNeeded(did_acquire);
      return 1;
    }

    size_t total_count = 0;
    esp_err_t result = WifiManagerScan(records, record_cap, &total_count);
    if (result != ESP_OK) {
      printf("scan failed: %s\n", esp_err_to_name(result));
      AppFree(records);
      ReleaseWifiForConsoleIfNeeded(did_acquire);
      return 1;
    }

    const size_t listed_count =
      (total_count < record_cap) ? total_count : record_cap;
    printf("aps_found: %u (showing %u)\n",
           (unsigned)total_count,
           (unsigned)listed_count);
    for (size_t index = 0; index < listed_count; ++index) {
      const wifi_ap_record_t* ap = &records[index];
      const char* ssid =
        (ap->ssid[0] != '\0') ? (const char*)ap->ssid : "<hidden>";
      printf("  %2u. %-32s rssi=%d ch=%u auth=%s\n",
             (unsigned)(index + 1),
             ssid,
             ap->rssi,
             (unsigned)ap->primary,
             WifiAuthModeToString(ap->authmode));
    }

    AppFree(records);
    ReleaseWifiForConsoleIfNeeded(did_acquire);
    return 0;
  }

  if (strcmp(action, "status") == 0) {
    wifi_manager_status_t status;
    memset(&status, 0, sizeof(status));
    WifiManagerGetStatus(&status);
    printf("wifi_service_mode: %s\n",
           WifiServiceModeToString(WifiServiceActiveMode()));
    printf("wifi_sta_netif_present: %s\n",
           status.sta_netif_present ? "yes" : "no");
    printf("wifi_owns_sta_netif: %s\n", status.owns_sta_netif ? "yes" : "no");
    printf("wifi_initialized: %s\n", status.wifi_initialized ? "yes" : "no");
    printf("wifi_handler_registered: %s\n",
           status.wifi_handler_registered ? "yes" : "no");
    printf("wifi_ip_handler_registered: %s\n",
           status.ip_handler_registered ? "yes" : "no");
    printf("wifi_started: %s\n", status.wifi_started ? "yes" : "no");
    printf("wifi_started_by_manager: %s\n",
           status.started_by_manager ? "yes" : "no");

    const bool connected = WifiManagerIsConnected();
    printf("wifi_connected: %s\n", connected ? "yes" : "no");

    wifi_ap_record_t ap_info;
    memset(&ap_info, 0, sizeof(ap_info));
    if (connected && esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
      printf("wifi_rssi: %d\n", ap_info.rssi);
      printf("wifi_channel: %u\n", (unsigned)ap_info.primary);
    } else {
      printf("wifi_rssi: n/a\n");
      printf("wifi_channel: n/a\n");
    }

    const wifi_err_reason_t reason = WifiManagerLastDisconnectReason();
    printf("wifi_last_disconnect_reason: %s (%d)\n",
           WifiDisconnectReasonToString(reason),
           (int)reason);
    printf("wifi_last_connect_attempts: %d\n",
           WifiManagerLastConnectAttempts());

    if (connected) {
      esp_netif_ip_info_t ip_info;
      memset(&ip_info, 0, sizeof(ip_info));
      if (WifiManagerGetIpInfo(&ip_info) == ESP_OK) {
        char ip[16] = { 0 }, mask[16] = { 0 }, gw[16] = { 0 };
        esp_ip4addr_ntoa(&ip_info.ip, ip, sizeof(ip));
        esp_ip4addr_ntoa(&ip_info.netmask, mask, sizeof(mask));
        esp_ip4addr_ntoa(&ip_info.gw, gw, sizeof(gw));
        printf("wifi_ip: %s\n", ip);
        printf("wifi_netmask: %s\n", mask);
        printf("wifi_gw: %s\n", gw);
      } else {
        printf("wifi_ip: n/a\n");
        printf("wifi_netmask: n/a\n");
        printf("wifi_gw: n/a\n");
      }
    } else {
      printf("wifi_ip: n/a\n");
      printf("wifi_netmask: n/a\n");
      printf("wifi_gw: n/a\n");
    }
    return 0;
  }

  if (strcmp(action, "connect") == 0) {
    int timeout_ms = 10000;
    for (int i = 2; i < argc; ++i) {
      if (strcmp(argv[i], "--timeout_ms") == 0) {
        if (i + 1 >= argc) {
          printf("usage: wifi connect [--timeout_ms T]\n");
          return 1;
        }
        timeout_ms = atoi(argv[++i]);
      } else {
        printf("usage: wifi connect [--timeout_ms T]\n");
        return 1;
      }
    }

    wifi_credentials_t creds;
    WifiCredentialsLoad(&creds);
    if (!creds.has_ssid || !creds.from_nvs) {
      printf("no saved Wi-Fi credentials in NVS. Use: wifi set <ssid> "
             "[password]\n");
      return 1;
    }

    bool did_acquire = false;
    esp_err_t wifi_result = AcquireWifiForConsole(&did_acquire);
    if (wifi_result != ESP_OK) {
      printf("connect failed: %s\n", esp_err_to_name(wifi_result));
      if (wifi_result == ESP_ERR_INVALID_STATE) {
        printf("note: Wi-Fi is active in another mode (%s).\n",
               WifiServiceModeToString(WifiServiceActiveMode()));
      }
      return 1;
    }

    esp_err_t result =
      WifiManagerConnectSta(creds.ssid, creds.password, timeout_ms);
    if (result != ESP_OK) {
      const wifi_err_reason_t reason = WifiManagerLastDisconnectReason();
      printf("connect failed: %s\n", esp_err_to_name(result));
      printf("  attempts=%d reason=%s (%d)\n",
             WifiManagerLastConnectAttempts(),
             WifiDisconnectReasonToString(reason),
             (int)reason);
      ReleaseWifiForConsoleIfNeeded(did_acquire);
      return 1;
    }
    printf("connected\n");
    ReleaseWifiForConsoleIfNeeded(did_acquire);
    return 0;
  }

  if (strcmp(action, "disconnect") == 0) {
    bool did_acquire = false;
    esp_err_t wifi_result = AcquireWifiForConsole(&did_acquire);
    if (wifi_result != ESP_OK) {
      printf("disconnect failed: %s\n", esp_err_to_name(wifi_result));
      if (wifi_result == ESP_ERR_INVALID_STATE) {
        printf("note: Wi-Fi is active in another mode (%s).\n",
               WifiServiceModeToString(WifiServiceActiveMode()));
      }
      return 1;
    }

    esp_err_t result = WifiManagerDisconnectSta();
    if (result != ESP_OK) {
      printf("disconnect failed: %s\n", esp_err_to_name(result));
      ReleaseWifiForConsoleIfNeeded(did_acquire);
      return 1;
    }
    printf("OK\n");
    ReleaseWifiForConsoleIfNeeded(did_acquire);
    return 0;
  }

  if (strcmp(action, "cfg") == 0) {
    if (argc < 3) {
      PrintWifiUsage();
      return 1;
    }
    const char* subaction = argv[2];
    if (strcmp(subaction, "show") == 0) {
      PrintWifiConfig();
      return 0;
    }
    if (strcmp(subaction, "defaults") == 0) {
      esp_err_t result = AppNetConfigClearAllOverrides();
      if (result != ESP_OK) {
        printf("defaults failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      printf("OK\n");
      if (RuntimeIsRunning()) {
        printf("note: run stop; run start to apply\n");
      }
      return 0;
    }
    if (strcmp(subaction, "set") == 0) {
      if (argc < 5) {
        PrintWifiUsage();
        return 1;
      }
      const char* key = argv[3];
      const char* value = argv[4];
      esp_err_t result = ESP_ERR_INVALID_ARG;

      if (strcmp(key, "sntp") == 0) {
        result = AppNetConfigSetSntpServer(value);
      } else if (strcmp(key, "mesh_chan") == 0) {
        char* end = NULL;
        unsigned long channel = strtoul(value, &end, 10);
        if (end != value && *end == '\0' && channel <= 13) {
          result = AppNetConfigSetMeshChannel((uint8_t)channel);
        }
      } else if (strcmp(key, "mesh_id") == 0) {
        result = AppNetConfigSetMeshIdString(value);
      } else if (strcmp(key, "mesh_ap_pass") == 0) {
        result = AppNetConfigSetMeshApPassword(value);
      } else if (strcmp(key, "no_router") == 0) {
        if (strcmp(value, "0") == 0) {
          result = AppNetConfigSetMeshDisableRouter(false);
        } else if (strcmp(value, "1") == 0) {
          result = AppNetConfigSetMeshDisableRouter(true);
        }
      } else if (strcmp(key, "time_sync_s") == 0) {
        char* end = NULL;
        unsigned long seconds = strtoul(value, &end, 10);
        if (end != value && *end == '\0' && seconds <= UINT32_MAX) {
          result = AppNetConfigSetTimeSyncPeriodSeconds((uint32_t)seconds);
        }
      } else {
        PrintWifiUsage();
        return 1;
      }

      if (result != ESP_OK) {
        printf("set failed: %s\n", esp_err_to_name(result));
        return 1;
      }

      printf("OK\n");
      NotifyNetSupervisor();
      return 0;
    }

    PrintWifiUsage();
    return 1;
  }

  if (strcmp(action, "ntp") == 0) {
    if (argc < 3) {
      printf("usage: wifi ntp status | wifi ntp sync [--server host] "
             "[--timeout_ms T] [--update-rtc 0|1]\n");
      return 1;
    }
    const char* subaction = argv[2];
    if (strcmp(subaction, "status") == 0) {
      printf("system_time_valid: %s\n",
             TimeSyncIsSystemTimeValid() ? "yes" : "no");
      time_sntp_status_t sntp_status;
      TimeSyncGetSntpStatus(&sntp_status);
      printf("last_sntp_server: %s\n",
             (sntp_status.last_server[0] != '\0') ? sntp_status.last_server
                                                  : "n/a");
      if (sntp_status.last_attempt_epoch > 0) {
        printf("last_sntp_attempt_epoch: %" PRId64 "\n",
               sntp_status.last_attempt_epoch);
      } else {
        printf("last_sntp_attempt_epoch: never\n");
      }
      printf("last_sntp_result: %s (%d)\n",
             esp_err_to_name(sntp_status.last_result),
             (int)sntp_status.last_result);
      if (sntp_status.last_success_epoch > 0) {
        printf("last_sntp_success_epoch: %" PRId64 "\n",
               sntp_status.last_success_epoch);
      } else {
        printf("last_sntp_success_epoch: never\n");
      }

      const bool rtc_present =
        (g_runtime != NULL && g_runtime->time_sync != NULL &&
         g_runtime->time_sync->is_ds3231_ready);
      printf("rtc_present: %s\n", rtc_present ? "yes" : "no");

      int64_t rtc_epoch = 0;
      if (rtc_present && g_runtime != NULL && g_runtime->time_sync != NULL &&
          TimeSyncReadRtcEpoch(g_runtime->time_sync, &rtc_epoch) == ESP_OK) {
        printf("rtc_epoch_utc: %" PRId64 "\n", rtc_epoch);
      } else {
        printf("rtc_epoch_utc: n/a\n");
      }

      const int64_t rtc_last_set_epoch = TimeSyncGetLastRtcSetEpoch();
      if (rtc_last_set_epoch > 0) {
        printf("rtc_last_set_epoch: %" PRId64 "\n", rtc_last_set_epoch);
      } else {
        printf("rtc_last_set_epoch: n/a\n");
      }
      return 0;
    }

    if (strcmp(subaction, "sync") == 0) {
      const char* server = NULL;
      int timeout_ms = 30000;
      bool update_rtc = true;

      for (int i = 3; i < argc; ++i) {
        if (strcmp(argv[i], "--server") == 0) {
          if (i + 1 >= argc) {
            printf("usage: wifi ntp sync [--server host] [--timeout_ms T] "
                   "[--update-rtc 0|1]\n");
            return 1;
          }
          server = argv[++i];
        } else if (strcmp(argv[i], "--timeout_ms") == 0) {
          if (i + 1 >= argc) {
            printf("usage: wifi ntp sync [--server host] [--timeout_ms T] "
                   "[--update-rtc 0|1]\n");
            return 1;
          }
          timeout_ms = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--update-rtc") == 0) {
          if (i + 1 >= argc) {
            printf("usage: wifi ntp sync [--server host] [--timeout_ms T] "
                   "[--update-rtc 0|1]\n");
            return 1;
          }
          update_rtc = (atoi(argv[++i]) != 0);
        } else {
          printf("usage: wifi ntp sync [--server host] [--timeout_ms T] "
                 "[--update-rtc 0|1]\n");
          return 1;
        }
      }

      if (server == NULL) {
        server = PickDefaultSntpServer();
      }

      esp_err_t result = TimeSyncStartSntpAndWait(server, timeout_ms);
      if (result != ESP_OK) {
        printf("sntp sync failed: %s\n", esp_err_to_name(result));
        return 1;
      }
      printf("sntp sync OK\n");

      if (update_rtc) {
        if (g_runtime != NULL && g_runtime->time_sync != NULL &&
            g_runtime->time_sync->is_ds3231_ready) {
          esp_err_t rtc_result = TimeSyncSetRtcFromSystem(g_runtime->time_sync);
          if (rtc_result != ESP_OK) {
            printf("rtc update failed: %s\n", esp_err_to_name(rtc_result));
            return 1;
          }
          printf("rtc updated\n");
        } else {
          printf("rtc update skipped: rtc not available\n");
        }
      }
      return 0;
    }

    printf("usage: wifi ntp status | wifi ntp sync [--server host] "
           "[--timeout_ms T] [--update-rtc 0|1]\n");
    return 1;
  }

  PrintWifiUsage();
  return 1;
}

/**
 * @brief Execute PrintMqttRestartNote.
 */
static void
PrintMqttRestartNote(void)
{
  if (RuntimeIsRunning()) {
    printf("Change applied; takes effect after run stop/start\n");
  }
}

/**
 * @brief Execute CommandMqtt.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandMqtt(int argc, char** argv)
{
  if (g_runtime == NULL) {
    return 1;
  }
  if (argc < 2) {
    printf("usage: mqtt show | mqtt enable on|off | mqtt broker set <uri> | "
           "mqtt prefix set <prefix> | mqtt qos set 0|1 | mqtt retain set "
           "on|off | mqtt bridge set off|serial|broker|both\n");
    return 1;
  }

  app_settings_t* settings = g_runtime->settings;
  const char* action = argv[1];

  if (strcmp(action, "show") == 0) {
    printf("mqtt_enabled: %s\n", settings->mqtt_enabled ? "yes" : "no");
    printf("mqtt_broker_uri: %s\n", settings->mqtt_broker_uri);
    printf("mqtt_topic_prefix: %s\n", settings->mqtt_topic_prefix);
    printf("mqtt_qos: %u\n", (unsigned)settings->mqtt_qos);
    printf("mqtt_retain: %s\n", settings->mqtt_retain ? "yes" : "no");
    printf("mqtt_bridge_mode: %s\n",
           AppSettingsMqttBridgeModeToString(settings->mqtt_bridge_mode));
    return 0;
  }

  if (strcmp(action, "enable") == 0) {
    if (argc != 3) {
      printf("usage: mqtt enable on|off\n");
      return 1;
    }
    const bool enabled = (strcmp(argv[2], "on") == 0);
    if (!enabled && strcmp(argv[2], "off") != 0) {
      printf("usage: mqtt enable on|off\n");
      return 1;
    }
    settings->mqtt_enabled = enabled;
    esp_err_t result = AppSettingsSaveMqttEnabled(enabled);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("mqtt_enabled set to %s\n", enabled ? "on" : "off");
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "broker") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt broker set <uri>\n");
      return 1;
    }
    const char* uri = argv[3];
    if (uri[0] == '\0') {
      printf("invalid broker uri\n");
      return 1;
    }
    esp_err_t result = AppSettingsSaveMqttBrokerUri(uri);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    snprintf(
      settings->mqtt_broker_uri, sizeof(settings->mqtt_broker_uri), "%s", uri);
    printf("mqtt_broker_uri set to %s\n", settings->mqtt_broker_uri);
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "prefix") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt prefix set <prefix>\n");
      return 1;
    }
    const char* prefix = argv[3];
    if (prefix[0] == '\0') {
      printf("invalid prefix\n");
      return 1;
    }
    esp_err_t result = AppSettingsSaveMqttTopicPrefix(prefix);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    snprintf(settings->mqtt_topic_prefix,
             sizeof(settings->mqtt_topic_prefix),
             "%s",
             prefix);
    printf("mqtt_topic_prefix set to %s\n", settings->mqtt_topic_prefix);
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "qos") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt qos set 0|1\n");
      return 1;
    }
    char* end = NULL;
    long qos_long = strtol(argv[3], &end, 10);
    if (end == argv[3] || *end != '\0' || (qos_long != 0 && qos_long != 1)) {
      printf("usage: mqtt qos set 0|1\n");
      return 1;
    }
    settings->mqtt_qos = (uint8_t)qos_long;
    esp_err_t result = AppSettingsSaveMqttQos((uint8_t)qos_long);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("mqtt_qos set to %ld\n", qos_long);
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "retain") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt retain set on|off\n");
      return 1;
    }
    const bool retain = (strcmp(argv[3], "on") == 0);
    if (!retain && strcmp(argv[3], "off") != 0) {
      printf("usage: mqtt retain set on|off\n");
      return 1;
    }
    settings->mqtt_retain = retain;
    esp_err_t result = AppSettingsSaveMqttRetain(retain);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("mqtt_retain set to %s\n", retain ? "on" : "off");
    PrintMqttRestartNote();
    return 0;
  }

  if (strcmp(action, "bridge") == 0) {
    if (argc != 4 || strcmp(argv[2], "set") != 0) {
      printf("usage: mqtt bridge set off|serial|broker|both\n");
      return 1;
    }
    mqtt_bridge_mode_t mode = MQTT_BRIDGE_OFF;
    if (!AppSettingsParseMqttBridgeMode(argv[3], &mode)) {
      printf("usage: mqtt bridge set off|serial|broker|both\n");
      return 1;
    }
    settings->mqtt_bridge_mode = mode;
    esp_err_t result = AppSettingsSaveMqttBridgeMode(mode);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    if (settings->node_role != APP_NODE_ROLE_ROOT) {
      printf("note: bridge mode applies to root nodes\n");
    }
    printf("mqtt_bridge_mode set to %s\n",
           AppSettingsMqttBridgeModeToString(mode));
    PrintMqttRestartNote();
    return 0;
  }

  printf("unknown action. usage: mqtt show | mqtt enable on|off | mqtt broker "
         "set <uri> | mqtt prefix set <prefix> | mqtt qos set 0|1 | mqtt "
         "retain set on|off | mqtt bridge set off|serial|broker|both\n");
  return 1;
}

/**
 * @brief Execute CommandChildren.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandChildren(int argc, char** argv)
{
  int errors = arg_parse(argc, argv, (void**)&g_children_args);
  if (errors != 0) {
    arg_print_errors(stderr, g_children_args.end, argv[0]);
    return 1;
  }
  if (g_runtime == NULL) {
    return 1;
  }

  const char* action = g_children_args.action->sval[0];
  if (strcmp(action, "show") == 0) {
    printf("allow_children: %u\n", g_runtime->settings->allow_children ? 1 : 0);
    return 0;
  }

  if (strcmp(action, "set") == 0) {
    if (g_children_args.enabled->count != 1) {
      printf("usage: children set 0|1\n");
      return 1;
    }
    const int enabled = g_children_args.enabled->ival[0];
    if (enabled != 0 && enabled != 1) {
      printf("usage: children set 0|1\n");
      return 1;
    }
    g_runtime->settings->allow_children = (enabled == 1);
    g_runtime->settings->allow_children_set = true;
    esp_err_t result =
      AppSettingsSaveAllowChildren(g_runtime->settings->allow_children, true);
    if (result != ESP_OK) {
      printf("save failed: %s\n", esp_err_to_name(result));
      return 1;
    }
    printf("allow_children set to %d\n", enabled);
    return 0;
  }

  printf("unknown action. usage: children show | children set 0|1\n");
  return 1;
}

/**
 * @brief Execute PrintDiagUsage.
 */
static void
PrintDiagUsage(void)
{
  printf("diag help\n");
  printf("diag all quick|full [--verbose N]\n");
  printf("diag sd quick|full [--format-if-needed] [--mount] [--verbose N]\n");
  printf("diag storage quick|full [--verbose N]\n");
  printf("diag fram quick|full [--bytes N] [--verbose N]\n");
  printf("diag rtd quick|full [--samples N] [--delay_ms M] [--verbose N]\n");
  printf("diag rtc quick|full [--set-known] [--verbose N]\n");
  printf("diag heapcheck on|off|now\n");
  printf("diag cycle [--count N] [--run_ms M] [--stop_ms M]\n");
  printf("diag wifi quick|full [--scan 0|1] [--connect 0|1] [--dns 0|1] "
         "[--keep-connected 0|1] [--verbose N]\n");
  printf("diag mesh quick|full [--start] [--stop] [--root] [--timeout_ms T] "
         "[--verbose N]\n"
         "  note: if you use --start without --stop, the mesh stays running\n");
}

/**
 * @brief Execute ParseVerbose.
 * @param value Parameter value.
 * @param verbosity_out Parameter verbosity_out.
 * @return Return the function result.
 */
static bool
ParseVerbose(const char* value, int* verbosity_out)
{
  if (value == NULL || verbosity_out == NULL) {
    return false;
  }
  char* end = NULL;
  const long parsed = strtol(value, &end, 10);
  if (end == NULL || *end != '\0') {
    return false;
  }
  *verbosity_out = (int)parsed;
  return true;
}

/**
 * @brief Execute ParseOptionalBool.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @param index Parameter index.
 * @param target Parameter target.
 * @return Return the function result.
 */
static int
ParseOptionalBool(int argc, char** argv, int* index, bool* target)
{
  if (argv == NULL || index == NULL || target == NULL) {
    return 0;
  }
  const int i = *index;
  if ((i + 1) < argc &&
      (strcmp(argv[i + 1], "0") == 0 || strcmp(argv[i + 1], "1") == 0)) {
    *target = (argv[i + 1][0] == '1');
    *index = i + 1;
  } else {
    *target = true;
  }
  return 1;
}

/**
 * @brief Execute CommandDiagnostics.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandDiagnostics(int argc, char** argv)
{
  if (argc < 2) {
    PrintDiagUsage();
    return 2;
  }

  const char* target = argv[1];
  int verbosity = 0;
  bool format_if_needed = false;
  bool mount = false;
  bool scan = false;
  bool connect = false;
  bool dns_lookup = false;
  bool keep_connected = false;
  bool set_known = false;
  int bytes = 0;
  int samples = 0;
  int delay_ms = -1;
  bool start_mesh = false;
  bool stop_mesh = false;
  bool mesh_force_root = false;
  int mesh_timeout_ms = 10000;

  const app_runtime_t* runtime = RuntimeGetRuntime();

  if (strcmp(target, "help") == 0) {
    PrintDiagUsage();
    return 0;
  }

  if (strcmp(target, "heapcheck") == 0) {
    runtime_state_t* state = RuntimeGetState();
    if (state == NULL) {
      printf("diag heapcheck failed: runtime unavailable\n");
      return 1;
    }
    if (argc < 3) {
      printf("diag heapcheck: %s\n",
             state->diag_heap_check_enabled ? "on" : "off");
      printf("usage: diag heapcheck on|off|now\n");
      return 0;
    }
    const char* action = argv[2];
    if (strcmp(action, "on") == 0) {
      state->diag_heap_check_enabled = true;
      printf("diag heapcheck: on\n");
      return 0;
    }
    if (strcmp(action, "off") == 0) {
      state->diag_heap_check_enabled = false;
      printf("diag heapcheck: off\n");
      return 0;
    }
    if (strcmp(action, "now") == 0) {
      const bool ok = RuntimeDiagHeapCheck(state, "diag heapcheck now", true);
      printf("diag heapcheck: %s\n", ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    printf("usage: diag heapcheck on|off|now\n");
    return 1;
  }

  if (strcmp(target, "cycle") == 0) {
    int count = 1;
    int run_ms = 1000;
    int stop_ms = 0;
    for (int i = 2; i < argc; ++i) {
      if (strcmp(argv[i], "--count") == 0 && (i + 1) < argc) {
        count = atoi(argv[++i]);
      } else if (strcmp(argv[i], "--run_ms") == 0 && (i + 1) < argc) {
        run_ms = atoi(argv[++i]);
      } else if (strcmp(argv[i], "--stop_ms") == 0 && (i + 1) < argc) {
        stop_ms = atoi(argv[++i]);
      } else {
        printf("unknown option: %s\n", argv[i]);
        PrintDiagUsage();
        return 2;
      }
    }

    if (count <= 0 || run_ms < 0 || stop_ms < 0) {
      printf("invalid values: count > 0, run_ms >= 0, stop_ms >= 0\n");
      PrintDiagUsage();
      return 2;
    }

    runtime_state_t* state = RuntimeGetState();
    if (state == NULL) {
      printf("diag cycle failed: runtime unavailable\n");
      return 1;
    }

    if (RuntimeIsRunning()) {
      printf("diag cycle: runtime running; stopping first\n");
      esp_err_t stop_result = EnterDiagMode();
      if (stop_result != ESP_OK) {
        printf("diag cycle: stop failed: %s\n", esp_err_to_name(stop_result));
        return 1;
      }
    }

    size_t min_free_heap = SIZE_MAX;
    size_t min_free_internal = SIZE_MAX;
    uint32_t spi_min = UINT32_MAX;
    uint32_t spi_max = 0;
    int start_fail = 0;
    int stop_fail = 0;
    int mount_attempts = 0;
    int mount_ok = 0;
    int unmount_attempts = 0;
    int unmount_ok = 0;

    for (int cycle = 1; cycle <= count; ++cycle) {
      printf("diag cycle %d/%d: start\n", cycle, count);
      esp_err_t start_result = EnterRunMode();
      if (start_result != ESP_OK) {
        printf("diag cycle: start failed on cycle %d: %s\n",
               cycle,
               esp_err_to_name(start_result));
        start_fail++;
        break;
      }

      if (run_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(run_ms));
      }

      size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
      if (free_heap < min_free_heap) {
        min_free_heap = free_heap;
      }
      size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      if (free_internal < min_free_internal) {
        min_free_internal = free_internal;
      }

      uint32_t spi_count = RuntimeGetSpiDeviceCount();
      if (spi_count < spi_min) {
        spi_min = spi_count;
      }
      if (spi_count > spi_max) {
        spi_max = spi_count;
      }

      if (SdCardDetectIsPresent(&state->sd_card_detect)) {
        mount_attempts++;
        if (state->sd_logger.is_mounted) {
          mount_ok++;
        }
      }

      printf("diag cycle %d/%d: stop\n", cycle, count);
      esp_err_t stop_result = EnterDiagMode();
      if (stop_result != ESP_OK) {
        printf("diag cycle: stop failed on cycle %d: %s\n",
               cycle,
               esp_err_to_name(stop_result));
        stop_fail++;
      }

      free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
      if (free_heap < min_free_heap) {
        min_free_heap = free_heap;
      }
      free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      if (free_internal < min_free_internal) {
        min_free_internal = free_internal;
      }

      spi_count = RuntimeGetSpiDeviceCount();
      if (spi_count < spi_min) {
        spi_min = spi_count;
      }
      if (spi_count > spi_max) {
        spi_max = spi_count;
      }

      if (SdCardDetectIsPresent(&state->sd_card_detect)) {
        unmount_attempts++;
        if (!state->sd_logger.is_mounted) {
          unmount_ok++;
        }
      }

      if (stop_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(stop_ms));
      }
    }

    if (min_free_heap == SIZE_MAX) {
      min_free_heap = 0;
    }
    if (min_free_internal == SIZE_MAX) {
      min_free_internal = 0;
    }
    if (spi_min == UINT32_MAX) {
      spi_min = 0;
    }

    printf("diag cycle done: cycles=%d start_fail=%d stop_fail=%d\n",
           count,
           start_fail,
           stop_fail);
    printf("diag cycle heap min: default=%u internal=%u\n",
           (unsigned)min_free_heap,
           (unsigned)min_free_internal);
    printf("diag cycle spi devices: min=%" PRIu32 " max=%" PRIu32 "\n",
           spi_min,
           spi_max);
    printf("diag cycle SD mount ok=%d/%d unmount ok=%d/%d\n",
           mount_ok,
           mount_attempts,
           unmount_ok,
           unmount_attempts);

    if (start_fail != 0 || stop_fail != 0) {
      return 1;
    }
    if ((mount_attempts > 0 && mount_ok != mount_attempts) ||
        (unmount_attempts > 0 && unmount_ok != unmount_attempts)) {
      return 1;
    }
    return 0;
  }

  const bool target_requires_mode =
    strcmp(target, "all") == 0 || strcmp(target, "sd") == 0 ||
    strcmp(target, "storage") == 0 || strcmp(target, "fram") == 0 ||
    strcmp(target, "rtc") == 0 || strcmp(target, "rtd") == 0 ||
    strcmp(target, "wifi") == 0 || strcmp(target, "mesh") == 0;
  const char* mode = (argc > 2) ? argv[2] : NULL;
  if (strcmp(target, "check") == 0) {
    target = "all";
    mode = "quick";
  } else if (!target_requires_mode) {
    printf("unknown diag target. try 'diag help'\n");
    return 2;
  }

  if (mode == NULL ||
      (strcmp(mode, "quick") != 0 && strcmp(mode, "full") != 0)) {
    printf("missing or invalid mode (quick|full)\n");
    PrintDiagUsage();
    return 2;
  }

  const bool full = strcmp(mode, "full") == 0;

  if (strcmp(target, "wifi") == 0 || strcmp(target, "all") == 0) {
    scan = true;
    if (full) {
      connect = true;
      dns_lookup = true;
    }
  }

  for (int i = 3; i < argc; ++i) {
    if (strcmp(argv[i], "--verbose") == 0 && (i + 1) < argc) {
      if (!ParseVerbose(argv[i + 1], &verbosity)) {
        printf("--verbose requires an integer value\n");
        PrintDiagUsage();
        return 2;
      }
      ++i;
    } else if (strcmp(argv[i], "--format-if-needed") == 0) {
      format_if_needed = true;
    } else if (strcmp(argv[i], "--mount") == 0) {
      mount = true;
    } else if (strcmp(argv[i], "--scan") == 0) {
      ParseOptionalBool(argc, argv, &i, &scan);
    } else if (strcmp(argv[i], "--connect") == 0) {
      ParseOptionalBool(argc, argv, &i, &connect);
    } else if (strcmp(argv[i], "--dns") == 0) {
      ParseOptionalBool(argc, argv, &i, &dns_lookup);
    } else if (strcmp(argv[i], "--keep-connected") == 0) {
      ParseOptionalBool(argc, argv, &i, &keep_connected);
    } else if (strcmp(argv[i], "--set-known") == 0) {
      set_known = true;
    } else if (strcmp(argv[i], "--bytes") == 0 && (i + 1) < argc) {
      bytes = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--samples") == 0 && (i + 1) < argc) {
      samples = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--delay_ms") == 0 && (i + 1) < argc) {
      delay_ms = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--start") == 0) {
      start_mesh = true;
    } else if (strcmp(argv[i], "--stop") == 0) {
      stop_mesh = true;
    } else if (strcmp(argv[i], "--root") == 0) {
      mesh_force_root = true;
    } else if (strcmp(argv[i], "--timeout_ms") == 0 && (i + 1) < argc) {
      mesh_timeout_ms = atoi(argv[++i]);
    } else {
      printf("unknown option: %s\n", argv[i]);
      PrintDiagUsage();
      return 2;
    }
  }

  int overall = 0;
  const diag_verbosity_t diag_verbosity =
    (verbosity >= 2) ? kDiagVerbosity2
                     : ((verbosity > 0) ? kDiagVerbosity1 : kDiagVerbosity0);

  if (strcmp(target, "sd") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |=
        RunDiagSd(runtime, full, format_if_needed, mount, diag_verbosity);
    }
    if (strcmp(target, "sd") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "storage") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |= RunDiagStorage(runtime, full, diag_verbosity);
    }
    if (strcmp(target, "storage") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "fram") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |= RunDiagFram(runtime, full, bytes, diag_verbosity);
    }
    if (strcmp(target, "fram") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "rtd") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |= RunDiagRtd(runtime, full, samples, delay_ms, diag_verbosity);
    }
    if (strcmp(target, "rtd") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "rtc") == 0 || strcmp(target, "all") == 0) {
    if (RuntimeIsRunning()) {
      printf("Stop run mode first: run stop\n");
      overall = 1;
    } else {
      overall |= RunDiagRtc(runtime, full, set_known, diag_verbosity);
    }
    if (strcmp(target, "rtc") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "wifi") == 0 || strcmp(target, "all") == 0) {
    overall |= RunDiagWifi(
      runtime, full, scan, connect, dns_lookup, keep_connected, diag_verbosity);
    if (strcmp(target, "wifi") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "mesh") == 0 || strcmp(target, "all") == 0) {
    if (full && !start_mesh && !stop_mesh) {
      start_mesh = true;
      stop_mesh = true;
    }
    overall |= RunDiagMesh(runtime,
                           full,
                           start_mesh,
                           stop_mesh,
                           mesh_force_root,
                           mesh_timeout_ms,
                           diag_verbosity);
    if (strcmp(target, "mesh") == 0) {
      return overall;
    }
  }

  if (strcmp(target, "all") == 0) {
    return overall;
  }

  printf("unknown diag target. try 'diag help'\n");
  return 2;
}

/**
 * @brief Execute CommandReboot.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandReboot(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  printf("rebooting...\n");
  esp_restart();
  return 0;
}

/**
 * @brief Execute RegisterCommands.
 */
static void
RegisterCommands(void)
{
  const esp_console_cmd_t status_cmd = {
    .command = "status",
    .help = "Show current settings and runtime state",
    .hint = NULL,
    .func = &CommandStatus,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&status_cmd));

  const esp_console_cmd_t stack_cmd = {
    .command = "stack",
    .help = "Stack usage: stack [show] [--headroom BYTES]",
    .hint = NULL,
    .func = &CommandStack,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&stack_cmd));

  const esp_console_cmd_t disp_cmd = {
    .command = "disp",
    .help = "Display settings: disp show | disp units C|F | disp attn "
            "show|set|defaults|ack",
    .hint = NULL,
    .func = &CommandDisplay,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&disp_cmd));

  const esp_console_cmd_t units_cmd = {
    .command = "units",
    .help = "Units settings: units set C|F | units gpio show | units gpio set "
            "pin <n> | units gpio set pull <up|down|none> | units gpio set "
            "c_level <high|low>",
    .hint = NULL,
    .func = &CommandUnits,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&units_cmd));

  const esp_console_cmd_t raw_cmd = {
    .command = "raw",
    .help = "Print one raw reading and calibrated value",
    .hint = NULL,
    .func = &CommandRaw,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&raw_cmd));

  const esp_console_cmd_t flush_cmd = {
    .command = "flush",
    .help = "Force flush FRAM -> SD (best-effort)",
    .hint = NULL,
    .func = &CommandFlush,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&flush_cmd));

  const esp_console_cmd_t sd_cmd = {
    .command = "sd",
    .help =
      "SD commands: sd status | sd mount | sd unmount | sd format | sd view | "
      "sd verify",
    .hint = NULL,
    .func = &CommandSd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&sd_cmd));

  const esp_console_cmd_t fram_cmd = {
    .command = "fram",
    .help = "FRAM log commands: fram status | fram show | fram clear",
    .hint = NULL,
    .func = &CommandFram,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&fram_cmd));

  const esp_console_cmd_t log_cmd = {
    .command = "log",
    .help = "Logging config: log interval <ms> | log watermark <records> | log "
            "flush_period <ms> | log batch <bytes> | log show",
    .hint = NULL,
    .func = &CommandLog,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&log_cmd));

  const esp_console_cmd_t mqtt_cmd = {
    .command = "mqtt",
    .help =
      "MQTT settings: mqtt show | mqtt enable on|off | mqtt broker set "
      "<uri> | mqtt prefix set <prefix> | mqtt qos set 0|1 | mqtt "
      "retain set on|off | mqtt bridge set off|serial|broker|both\n"
      "Note: broker bridge requires mqtt enabled; serial bridge does not.",
    .hint = NULL,
    .func = &CommandMqtt,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&mqtt_cmd));

  const esp_console_cmd_t rtd_cmd = {
    .command = "rtd",
    .help = "RTD settings: rtd show | rtd ema show | rtd ema on|off | rtd ema "
            "alpha <0.0..1.0>",
    .hint = NULL,
    .func = &CommandRtd,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&rtd_cmd));

  g_cal_args.action =
    arg_str1(NULL, NULL, "<action>", "clear|add|list|show|apply|live|capture");
  g_cal_args.raw_c =
    arg_dbl0(NULL, NULL, "<raw_c>", "Raw temperature (C) from 'raw'");
  g_cal_args.actual_c =
    arg_dbl0(NULL, NULL, "<actual_c>", "Actual temperature (C)");
  g_cal_args.every_ms =
    arg_int0(NULL, "every_ms", "<every_ms>", "Live interval (ms)");
  g_cal_args.seconds =
    arg_int0(NULL, "seconds", "<seconds>", "Live duration (s)");
  g_cal_args.stable_stddev_c = arg_dbl0(NULL,
                                        "stable_stddev_c",
                                        "<stable_stddev_c>",
                                        "Capture stable stddev threshold (C)");
  g_cal_args.min_seconds =
    arg_int0(NULL, "min_seconds", "<min_seconds>", "Capture min stable time");
  g_cal_args.timeout_seconds =
    arg_int0(NULL, "timeout_seconds", "<timeout_seconds>", "Capture timeout");
  g_cal_args.mode =
    arg_str0(NULL, "mode", "<mode>", "Fit mode (linear|piecewise|polyN)");
  g_cal_args.allow_wide_slope =
    arg_lit0(NULL, "allow_wide_slope", "Allow slope outside expected bounds");
  g_cal_args.end = arg_end(10);

  const esp_console_cmd_t cal_cmd = {
    .command = "cal",
    .help =
      "Calibration: cal clear | cal add <raw_c> <actual_c> | cal list | cal "
      "show |\n"
      "             cal apply [--mode linear|piecewise|polyN] "
      "[--allow_wide_slope] |\n"
      "             cal live [--every_ms 500] [--seconds 10] | cal capture "
      "<actual_temp_c>\n"
      "             [--stable_stddev_c 0.05] [--min_seconds 5] "
      "[--timeout_seconds 120]\n"
      "Workflow:\n"
      "        - cal clear\n"
      "        - let node settle at reference temp\n"
      "        - cal live --seconds 10\n"
      "        - cal capture 0.00 --stable_stddev_c 0.05 --min_seconds 10\n"
      "        - move to another reference\n"
      "        - cal capture 100.00 ...\n"
      "        - cal apply\n"
      "        - cal show.\n\n"
      "Notes: *may want to use boilpt command*; capture uses windowed\n"
      "average after stability; live/capture show raw vs average; apply uses\n"
      "least-squares and reports residuals; calibration invalidates when\n"
      "conversion mode, wire count, filter Hz, Rref, R0, or PT100 table\n"
      "changes.\n\n",
    .hint = NULL,
    .func = &CommandCal,
    .argtable = &g_cal_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&cal_cmd));

  g_mode_args.action = arg_str1(NULL, NULL, "<action>", "show|run|diag|set");
  g_mode_args.mode_value = arg_str0(NULL, NULL, "diag|run", "Target mode");
  g_mode_args.end = arg_end(2);
  const esp_console_cmd_t mode_cmd = {
    .command = "mode",
    .help = "mode show | mode run | mode diag | mode set diag|run",
    .hint = NULL,
    .func = &CommandMode,
    .argtable = &g_mode_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&mode_cmd));

  g_data_args.action = arg_str1(NULL, NULL, "<action>", "show|on|off");
  g_data_args.end = arg_end(1);
  const esp_console_cmd_t data_cmd = {
    .command = "data",
    .help = "data show | data on | data off",
    .hint = NULL,
    .func = &CommandData,
    .argtable = &g_data_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&data_cmd));

  g_run_args.action = arg_str1(NULL, NULL, "<action>", "status|start|stop");
  g_run_args.end = arg_end(1);
  const esp_console_cmd_t run_cmd = {
    .command = "run",
    .help = "run status | run start | run stop",
    .hint = NULL,
    .func = &CommandRun,
    .argtable = &g_run_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&run_cmd));

  g_tz_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_tz_args.posix = arg_str0(NULL, NULL, "<posix>", "POSIX TZ string");
  g_tz_args.end = arg_end(2);
  const esp_console_cmd_t tz_cmd = {
    .command = "tz",
    .help = "tz show | tz set \"<posix>\"",
    .hint = NULL,
    .func = &CommandTz,
    .argtable = &g_tz_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&tz_cmd));

  g_time_args.action = arg_str1(NULL, NULL, "<action>", "setlocal");
  g_time_args.local_time = arg_str0(NULL, NULL, "<local_time>", "LOCAL time");
  g_time_args.is_dst = arg_int0(NULL, "is_dst", "<0|1>", "DST disambiguation");
  g_time_args.end = arg_end(3);
  const esp_console_cmd_t time_cmd = {
    .command = "time",
    .help =
      "time setlocal \"YYYY-MM-DD HH:MM:SS\" [--is_dst 0|1]\n"
      "  input is LOCAL wall time; converted to UTC epoch + RTC stored as UTC\n"
      "  use --is_dst to disambiguate fall-back hour",
    .hint = NULL,
    .func = &CommandTime,
    .argtable = &g_time_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&time_cmd));

  const esp_console_cmd_t boilpt_cmd = {
    .command = "boilpt",
    .help =
      "Compute boiling point of water from pressure: boilpt <inHg> [elev_ft]",
    .hint = NULL,
    .func = &CommandBoilPt,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&boilpt_cmd));

  g_dst_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_dst_args.enabled = arg_int0(NULL, NULL, "<0|1>", "DST enabled");
  g_dst_args.end = arg_end(2);
  const esp_console_cmd_t dst_cmd = {
    .command = "dst",
    .help = "dst show | dst set 0|1",
    .hint = NULL,
    .func = &CommandDst,
    .argtable = &g_dst_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&dst_cmd));

  g_role_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_role_args.role = arg_str0(NULL, NULL, "<root|sensor|relay>", "Node role");
  g_role_args.end = arg_end(2);
  const esp_console_cmd_t role_cmd = {
    .command = "role",
    .help = "role show | role set root|sensor|relay",
    .hint = NULL,
    .func = &CommandRole,
    .argtable = &g_role_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&role_cmd));

  g_net_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_net_args.mode =
    arg_str0(NULL, NULL, "<mesh|wifi|none>", "Network mode (mesh|wifi|none)");
  g_net_args.end = arg_end(2);
  const esp_console_cmd_t net_cmd = {
    .command = "net",
    .help = "net show | net set mesh|wifi|none",
    .hint = NULL,
    .func = &CommandNet,
    .argtable = &g_net_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&net_cmd));

  const esp_console_cmd_t wifi_cmd = {
    .command = "wifi",
    .help = "wifi help | wifi show | wifi set <ssid> [password] | wifi clear | "
            "wifi scan [--max N] | wifi status | wifi connect [--timeout_ms T] "
            "| wifi disconnect | wifi ntp status | wifi ntp sync ...",
    .hint = NULL,
    .func = &CommandWifi,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&wifi_cmd));

  g_children_args.action = arg_str1(NULL, NULL, "<action>", "show|set");
  g_children_args.enabled =
    arg_int0(NULL, NULL, "<0|1>", "Allow downstream children");
  g_children_args.end = arg_end(2);
  const esp_console_cmd_t children_cmd = {
    .command = "children",
    .help = "children show | children set 0|1",
    .hint = NULL,
    .func = &CommandChildren,
    .argtable = &g_children_args,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&children_cmd));

  const esp_console_cmd_t diag_cmd = {
    .command = "diag",
    .help = "Diagnostics entry point",
    .hint = NULL,
    .func = &CommandDiagnostics,
    .argtable = NULL,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&diag_cmd));

  const esp_console_cmd_t reboot_cmd = {
    .command = "reboot",
    .help = "Soft reboot the device",
    .hint = NULL,
    .func = &CommandReboot,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&reboot_cmd));

  ConsoleAlertsRegister(g_runtime);
}

/**
 * @brief Execute ConsoleTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the ConsoleTask task.
 */
static void
ConsoleTask(void* context)
{
  (void)context;

  printf("\nType 'help' to list commands.\n");
  while (true) {
#if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    if (RuntimeIsDataStreamingEnabled()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
#endif
    char* line = linenoise("pt100> ");
    if (line == NULL) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (strlen(line) > 0) {
      linenoiseHistoryAdd(line);
      int run_result = 0;
      esp_err_t result = esp_console_run(line, &run_result);
      if (result == ESP_ERR_NOT_FOUND) {
        printf("Unrecognized command\n");
      } else if (result == ESP_ERR_INVALID_ARG) {
        // Command already printed errors.
      } else if (result != ESP_OK) {
        printf("Command failed: %s\n", esp_err_to_name(result));
      } else if (run_result != 0) {
        printf("Command returned non-zero: %d\n", run_result);
      }
    }
    linenoiseFree(line);
  }
}

/**
 * @brief Execute ConsoleCommandsStart.
 * @param runtime Parameter runtime.
 * @param boot_mode Parameter boot_mode.
 * @return Return the function result.
 */
esp_err_t
ConsoleCommandsStart(app_runtime_t* runtime, app_boot_mode_t boot_mode)
{
  if (runtime == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  g_runtime = runtime;
  g_boot_mode = boot_mode;

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG

  // USB Serial/JTAG console (native USB port)
  // Data CSV is streamed on UART0 (USB-to-UART bridge) to keep CSV parse-clean.
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

  // Make stdin/stdout blocking (helps linenoise)
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  usb_serial_jtag_driver_config_t usb_cfg = {
    .tx_buffer_size = 256,
    .rx_buffer_size = 256,
  };
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
  usb_serial_jtag_vfs_use_driver();
  setvbuf(stdin, NULL, _IONBF, 0);

#else

  // UART console (USB-to-UART bridge port)
  const int uart_num = CONFIG_ESP_CONSOLE_UART_NUM;
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_driver_install(uart_num, 256, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  uart_vfs_dev_use_driver(uart_num);

#endif

  esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
  console_config.max_cmdline_length = 256;
  console_config.max_cmdline_args = 8;
  ESP_ERROR_CHECK(esp_console_init(&console_config));

  // Register built-in help so users can discover available commands.
  ESP_ERROR_CHECK(esp_console_register_help_command());

  linenoiseSetDumbMode(1);
  linenoiseHistorySetMaxLen(50);

  RegisterCommands();

  xTaskCreate(ConsoleTask, "console", 12288, NULL, 2, NULL);
  return ESP_OK;
}
