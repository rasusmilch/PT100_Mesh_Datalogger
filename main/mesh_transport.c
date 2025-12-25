#include "mesh_transport.h"

#include <stddef.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_core.h"
#include "esp_mesh_lite_port.h"
#include "esp_wifi.h"
#include "wifi_service.h"

static const char* kTag = "mesh";
static const char* kMeshSoftApSsid = "PT100_MESH";

static mesh_transport_t* g_mesh = NULL;

#ifndef ESP_MESH_LITE_RAW_MSG_ACTION_END
#define ESP_MESH_LITE_RAW_MSG_ACTION_END { 0, 0, NULL }
#endif

typedef enum
{
  MESH_MESSAGE_RECORD = 1,
  MESH_MESSAGE_TIME_REQUEST = 2,
  MESH_MESSAGE_TIME_SYNC = 3,
} mesh_message_type_t;

#pragma pack(push, 1)
typedef struct
{
  uint8_t type;
  uint8_t src_mac[6];
  union
  {
    log_record_t record;
    int64_t epoch_seconds;
  } payload;
} mesh_message_t;
#pragma pack(pop)

static const uint32_t kRawMsgIdRecord = 0x00000001u;
static const uint32_t kRawMsgIdTimeRequest = 0x00000002u;
static const uint32_t kRawMsgIdTimeSync = 0x00000003u;

static const uint32_t kRawMsgMaxRetry = 3u;
static const uint16_t kRawMsgRetryIntervalMs = 300u;

static size_t
MeshMessageHeaderSize(void)
{
  return offsetof(mesh_message_t, payload);
}

static esp_err_t
PopulateMeshMessageSrc(mesh_message_t* msg)
{
  if (msg == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t local_mac[6] = { 0 };
  esp_err_t result = esp_wifi_get_mac(WIFI_IF_STA, local_mac);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "esp_wifi_get_mac failed: %s", esp_err_to_name(result));
    return result;
  }
  memcpy(msg->src_mac, local_mac, sizeof(msg->src_mac));
  return ESP_OK;
}

static esp_err_t
SendRawMessage(uint32_t msg_id,
               const uint8_t* data,
               size_t size,
               esp_err_t (*raw_resend)(const uint8_t* data, size_t size))
{
  esp_mesh_lite_msg_config_t config = {
    .raw_msg = {
      .msg_id = msg_id,
      .expect_resp_msg_id = 0,
      .max_retry = kRawMsgMaxRetry,
      .retry_interval = kRawMsgRetryIntervalMs,
      .data = data,
      .size = size,
      .raw_resend = raw_resend,
      .raw_send_fail = NULL,
    },
  };
  return esp_mesh_lite_send_msg(ESP_MESH_LITE_RAW_MSG, &config);
}

static void
ResetRawMessageOutput(uint8_t** out_data, uint32_t* out_len)
{
  if (out_data != NULL) {
    *out_data = NULL;
  }
  if (out_len != NULL) {
    *out_len = 0;
  }
}

static esp_err_t
OnRawRecord(uint8_t* data,
            uint32_t len,
            uint8_t** out_data,
            uint32_t* out_len,
            uint32_t seq)
{
  (void)seq;
  ResetRawMessageOutput(out_data, out_len);

  if (g_mesh == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  const size_t header_size = MeshMessageHeaderSize();
  if (len < header_size + sizeof(log_record_t)) {
    return ESP_ERR_INVALID_SIZE;
  }

  mesh_message_t msg;
  memset(&msg, 0, sizeof(msg));
  memcpy(&msg, data, header_size);
  memcpy(&msg.payload.record, data + header_size, sizeof(log_record_t));
  if (msg.type != MESH_MESSAGE_RECORD) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  if (g_mesh->record_rx_callback != NULL) {
    pt100_mesh_addr_t from = Pt100MeshAddrFromMac(msg.src_mac);
    g_mesh->record_rx_callback(
      &from, &msg.payload.record, g_mesh->record_rx_context);
  }
  return ESP_OK;
}

static esp_err_t
OnRawTimeRequest(uint8_t* data,
                 uint32_t len,
                 uint8_t** out_data,
                 uint32_t* out_len,
                 uint32_t seq)
{
  (void)seq;
  ResetRawMessageOutput(out_data, out_len);

  if (g_mesh == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  const size_t header_size = MeshMessageHeaderSize();
  if (len < header_size) {
    return ESP_ERR_INVALID_SIZE;
  }

  mesh_message_t msg;
  memset(&msg, 0, sizeof(msg));
  memcpy(&msg, data, header_size);
  if (msg.type != MESH_MESSAGE_TIME_REQUEST) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  if (g_mesh->is_root && TimeSyncIsSystemTimeValid()) {
    const int64_t now_seconds = (int64_t)time(NULL);
    mesh_message_t response = {
      .type = MESH_MESSAGE_TIME_SYNC,
      .payload.epoch_seconds = now_seconds,
    };
    if (PopulateMeshMessageSrc(&response) == ESP_OK) {
      const size_t response_size =
        MeshMessageHeaderSize() + sizeof(response.payload.epoch_seconds);
      (void)SendRawMessage(kRawMsgIdTimeSync,
                           (const uint8_t*)&response,
                           response_size,
                           esp_mesh_lite_send_broadcast_raw_msg_to_child);
    }
  }

  return ESP_OK;
}

static esp_err_t
OnRawTimeSync(uint8_t* data,
              uint32_t len,
              uint8_t** out_data,
              uint32_t* out_len,
              uint32_t seq)
{
  (void)seq;
  ResetRawMessageOutput(out_data, out_len);

  if (g_mesh == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  const size_t header_size = MeshMessageHeaderSize();
  if (len < header_size + sizeof(int64_t)) {
    return ESP_ERR_INVALID_SIZE;
  }

  mesh_message_t msg;
  memset(&msg, 0, sizeof(msg));
  memcpy(&msg, data, header_size);
  memcpy(&msg.payload.epoch_seconds,
         data + header_size,
         sizeof(msg.payload.epoch_seconds));
  if (msg.type != MESH_MESSAGE_TIME_SYNC) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  if (!g_mesh->is_root && g_mesh->time_sync != NULL) {
    (void)TimeSyncSetSystemEpoch(
      msg.payload.epoch_seconds, true, g_mesh->time_sync);
  }

  return ESP_OK;
}

static const esp_mesh_lite_raw_msg_action_t kMeshRawActions[] = {
  { kRawMsgIdRecord, 0, OnRawRecord },
  { kRawMsgIdTimeRequest, 0, OnRawTimeRequest },
  { kRawMsgIdTimeSync, 0, OnRawTimeSync },
  ESP_MESH_LITE_RAW_MSG_ACTION_END,
};

esp_err_t __attribute__((weak))
esp_mesh_lite_stop(void)
{
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t __attribute__((weak))
esp_mesh_lite_deinit(void)
{
  return ESP_ERR_NOT_SUPPORTED;
}

// Mesh-Lite component currently does not publish a public "stop" API surface
// for all internal background work (reconnect scans, resend timers, etc.), but
// the prebuilt library exports private helpers that are used by examples.
//
// We provide weak fallbacks so the app still builds if the symbol set changes.
void __attribute__((weak))
esp_mesh_lite_comm_stop_reconnect(void)
{
}

void __attribute__((weak))
esp_mesh_lite_stop_resend_raw_msg(void)
{
}

void __attribute__((weak))
esp_mesh_lite_stop_resend_json_msg(void)
{
}

void __attribute__((weak))
esp_mesh_lite_clear_scan_status(void)
{
}

void __attribute__((weak))
esp_mesh_lite_comm_clear_scan_status(void)
{
}

static bool
IsValidWifiChannel(int channel)
{
  return (channel >= 1) && (channel <= 14);
}

static void
ApplyMeshSoftApChannelBestEffort(int channel)
{
  if (!IsValidWifiChannel(channel)) {
    ESP_LOGW(kTag, "mesh channel %d out of range; ignoring", channel);
    return;
  }

  wifi_config_t ap_config;
  memset(&ap_config, 0, sizeof(ap_config));
  esp_err_t get_result = esp_wifi_get_config(WIFI_IF_AP, &ap_config);
  if (get_result != ESP_OK) {
    // Mesh-Lite may not have configured the SoftAP yet. Build a full AP config
    // using our menuconfig defaults so the channel takes effect immediately.
    ESP_LOGW(kTag,
             "esp_wifi_get_config(AP) failed: %s; applying full AP config",
             esp_err_to_name(get_result));

    memset(&ap_config, 0, sizeof(ap_config));
    strlcpy((char*)ap_config.ap.ssid, kMeshSoftApSsid, sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = (uint8_t)strlen((const char*)ap_config.ap.ssid);

#ifdef CONFIG_APP_MESH_AP_PASSWORD
    strlcpy((char*)ap_config.ap.password,
            CONFIG_APP_MESH_AP_PASSWORD,
            sizeof(ap_config.ap.password));
#endif

    if (ap_config.ap.password[0] == '\0') {
      ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
      ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ap_config.ap.ssid_hidden = 0;
    ap_config.ap.beacon_interval = 100;

#ifdef CONFIG_APP_MESH_AP_CONNECTIONS
    ap_config.ap.max_connection = (uint8_t)CONFIG_APP_MESH_AP_CONNECTIONS;
#else
    ap_config.ap.max_connection = 6;
#endif
  }

  ap_config.ap.channel = (uint8_t)channel;

#ifdef CONFIG_APP_MESH_AP_CONNECTIONS
  // If max_connection is unset, apply the menuconfig default.
  if (ap_config.ap.max_connection == 0) {
    ap_config.ap.max_connection = (uint8_t)CONFIG_APP_MESH_AP_CONNECTIONS;
  }
#endif

  esp_err_t set_result = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
  if (set_result != ESP_OK) {
    ESP_LOGW(kTag,
             "esp_wifi_set_config(AP) failed: %s",
             esp_err_to_name(set_result));
    return;
  }

  // Best-effort: force the primary channel immediately. In APSTA mode the
  // radio has a single primary channel.
  (void)esp_wifi_set_channel((uint8_t)channel, WIFI_SECOND_CHAN_NONE);
  ESP_LOGI(kTag, "mesh SoftAP channel set to %d", channel);
}

static void
StopMeshLiteBackgroundWorkBestEffort(void)
{
  // If Wi-Fi teardown posts late DISCONNECTED/STOP events, Mesh-Lite may
  // re-arm its reconnect timer. Make the timer effectively inert before we
  // stop it, so even if it is restarted it will not spam scans.
  esp_mesh_lite_set_wifi_reconnect_interval(/*retry_connect_parent_interval=*/3600,
                                           /*retry_connect_parent_count=*/1,
                                           /*reconnect_interval=*/3600);

  // Stop Mesh-Lite reconnect scans (this is the source of the ESP_FAIL:0x3002
  // spam when Wi-Fi has already been stopped).
  esp_mesh_lite_comm_stop_reconnect();

  // Clear any internal scan-in-progress flags so Mesh-Lite doesn't immediately
  // re-arm scanning after teardown.
  esp_mesh_lite_clear_scan_status();
  esp_mesh_lite_comm_clear_scan_status();

  // Stop resend timers for raw/json messages.
  esp_mesh_lite_stop_resend_raw_msg();
  esp_mesh_lite_stop_resend_json_msg();

  // Best-effort: cancel any in-flight scan.
  (void)esp_wifi_scan_stop();
}

static void
CacheMeshLevel(mesh_transport_t* mesh)
{
  if (mesh == NULL) {
    return;
  }
  mesh->last_level = esp_mesh_lite_get_level();
  mesh->is_connected = (mesh->last_level > 0);
}

bool
MeshTransportIsStarted(const mesh_transport_t* mesh)
{
  return mesh != NULL && mesh->mesh_lite_started;
}

bool
MeshTransportMeshLiteIsActive(void)
{
  return g_mesh != NULL && g_mesh->mesh_lite_started;
}

esp_err_t
MeshTransportStart(mesh_transport_t* mesh,
                   bool is_root,
                   bool allow_children,
                   const char* router_ssid,
                   const char* router_password,
                   mesh_record_rx_callback_t record_rx_callback,
                   void* record_rx_context,
                   const time_sync_t* time_sync)
{
  if (mesh == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(mesh, 0, sizeof(*mesh));
  mesh->is_root = is_root;
  mesh->record_rx_callback = record_rx_callback;
  mesh->record_rx_context = record_rx_context;
  mesh->time_sync = time_sync;

  g_mesh = mesh;

  esp_err_t svc_result = WifiServiceAcquire(WIFI_SERVICE_MODE_MESH);
  if (svc_result != ESP_OK) {
    g_mesh = NULL;
    return svc_result;
  }

  bool router_disabled = false;
#ifdef CONFIG_APP_MESH_DISABLE_ROUTER
  router_disabled = CONFIG_APP_MESH_DISABLE_ROUTER;
#endif

  esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
  mesh_lite_config.join_mesh_ignore_router_status = true;
  // Required for "no-router" deployments: allow forming/joining a mesh even
  // when there is no configured upstream Wi-Fi router.
  mesh_lite_config.join_mesh_without_configured_wifi =
    router_disabled || !is_root;
  mesh_lite_config.softap_ssid = kMeshSoftApSsid;
  mesh_lite_config.softap_password = CONFIG_APP_MESH_AP_PASSWORD;

  ESP_LOGI(kTag, "starting Mesh-Lite (root=%d)", (int)is_root);
  esp_mesh_lite_init(&mesh_lite_config);

  // Routerless root nodes do not configure an upstream STA SSID, but Mesh-Lite
  // can still run its reconnect/scan loop and repeatedly call esp_wifi_connect()
  // which then fails with ESP_ERR_WIFI_SSID (0x300a) and spams the console.
  //
  // Mitigation:
  //  - Rate-limit Mesh-Lite reconnect/scan intervals to a large value.
  //  - Reduce Mesh-Lite vendor_ie logging to ERROR (keeps errors visible).
  if (is_root && router_disabled) {
    esp_mesh_lite_set_wifi_reconnect_interval(
      /*retry_connect_parent_interval=*/3600,
      /*retry_connect_parent_count=*/1,
      /*reconnect_interval=*/3600);
    esp_log_level_set("vendor_ie", ESP_LOG_ERROR);
  }

  // Match the Mesh-Lite "no_router" example behavior: configure the SoftAP
  // channel explicitly. If the root later connects to an upstream router, the
  // Wi-Fi driver will move APSTA to the router's channel.
#ifdef CONFIG_APP_MESH_CHANNEL
  ApplyMeshSoftApChannelBestEffort(CONFIG_APP_MESH_CHANNEL);
#endif

  esp_err_t raw_action_result =
    esp_mesh_lite_raw_msg_action_list_register(kMeshRawActions);
  if (raw_action_result != ESP_OK) {
    ESP_LOGW(
      kTag, "raw msg register failed: %s", esp_err_to_name(raw_action_result));
  }

  const bool allow_join = is_root ? true : allow_children;
  if (allow_join) {
    (void)esp_mesh_lite_set_allowed_level(1);
    (void)esp_mesh_lite_allow_others_to_join(true);
  } else {
    (void)esp_mesh_lite_set_disallowed_level(1);
    (void)esp_mesh_lite_allow_others_to_join(false);
  }

  // Only the root should be configured with an upstream router.
  if (is_root && !router_disabled && router_ssid != NULL &&
      router_ssid[0] != '\0') {
    mesh_lite_sta_config_t router_config = { 0 };
    strlcpy((char*)router_config.ssid, router_ssid, sizeof(router_config.ssid));
    strlcpy((char*)router_config.password,
            (router_password != NULL) ? router_password : "",
            sizeof(router_config.password));

    esp_err_t router_result = esp_mesh_lite_set_router_config(&router_config);
    if (router_result != ESP_OK) {
      return router_result;
    }
  }

  esp_mesh_lite_start();

  // In routerless root mode there is no upstream to reconnect to. Stop the
  // Mesh-Lite reconnect machinery so it doesn't keep attempting STA connects.
  if (is_root && router_disabled) {
    esp_mesh_lite_comm_stop_reconnect();
    esp_mesh_lite_clear_scan_status();
    esp_mesh_lite_comm_clear_scan_status();
  }

  mesh->mesh_lite_started = true;
  mesh->is_started = true;
  CacheMeshLevel(mesh);

  return ESP_OK;
}

bool
MeshTransportIsConnected(const mesh_transport_t* mesh)
{
  if (mesh == NULL || !mesh->mesh_lite_started) {
    return false;
  }

  CacheMeshLevel((mesh_transport_t*)mesh);
  return mesh->is_connected;
}

esp_err_t
MeshTransportGetRootAddress(const mesh_transport_t* mesh,
                            pt100_mesh_addr_t* root_out)
{
  if (mesh == NULL || root_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!mesh->mesh_lite_started) {
    memset(root_out, 0, sizeof(*root_out));
    return ESP_ERR_INVALID_STATE;
  }
  pt100_mesh_addr_t root_addr = mesh->root_address;
  *root_out = root_addr;
  return ESP_OK;
}

esp_err_t
MeshTransportSendRecord(const mesh_transport_t* mesh,
                        const log_record_t* record)
{
  if (mesh == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!mesh->mesh_lite_started || !mesh->is_connected) {
    return ESP_ERR_INVALID_STATE;
  }
  mesh_message_t msg = {
    .type = MESH_MESSAGE_RECORD,
    .payload.record = *record,
  };
  esp_err_t mac_result = PopulateMeshMessageSrc(&msg);
  if (mac_result != ESP_OK) {
    return mac_result;
  }
  const size_t msg_size = MeshMessageHeaderSize() + sizeof(log_record_t);
  return SendRawMessage(kRawMsgIdRecord,
                        (const uint8_t*)&msg,
                        msg_size,
                        esp_mesh_lite_send_raw_msg_to_root);
}

esp_err_t
MeshTransportBroadcastTime(const mesh_transport_t* mesh, int64_t epoch_seconds)
{
  if (mesh == NULL || !mesh->is_root) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!mesh->mesh_lite_started || !mesh->is_connected) {
    return ESP_ERR_INVALID_STATE;
  }
  mesh_message_t msg = {
    .type = MESH_MESSAGE_TIME_SYNC,
    .payload.epoch_seconds = epoch_seconds,
  };
  esp_err_t mac_result = PopulateMeshMessageSrc(&msg);
  if (mac_result != ESP_OK) {
    return mac_result;
  }
  const size_t msg_size =
    MeshMessageHeaderSize() + sizeof(msg.payload.epoch_seconds);
  return SendRawMessage(kRawMsgIdTimeSync,
                        (const uint8_t*)&msg,
                        msg_size,
                        esp_mesh_lite_send_broadcast_raw_msg_to_child);
}

esp_err_t
MeshTransportRequestTime(const mesh_transport_t* mesh)
{
  if (mesh == NULL || mesh->is_root) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!mesh->mesh_lite_started || !mesh->is_connected) {
    return ESP_ERR_INVALID_STATE;
  }
  mesh_message_t msg = {
    .type = MESH_MESSAGE_TIME_REQUEST,
  };
  esp_err_t mac_result = PopulateMeshMessageSrc(&msg);
  if (mac_result != ESP_OK) {
    return mac_result;
  }
  const size_t msg_size = MeshMessageHeaderSize();
  return SendRawMessage(kRawMsgIdTimeRequest,
                        (const uint8_t*)&msg,
                        msg_size,
                        esp_mesh_lite_send_raw_msg_to_root);
}

esp_err_t
MeshTransportStop(mesh_transport_t* mesh)
{
  if (mesh == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  mesh->is_connected = false;
  if (!mesh->mesh_lite_started) {
    g_mesh = NULL;
    return ESP_OK;
  }

  esp_err_t result = ESP_OK;

  // Stop reconnect scan loops before stopping Wi-Fi or tearing down Mesh-Lite.
  // This prevents the periodic ESP_FAIL:0x3002 spam after diagnostics teardown.
  StopMeshLiteBackgroundWorkBestEffort();

  esp_err_t stop_result = esp_mesh_lite_stop();
  if (stop_result != ESP_ERR_NOT_SUPPORTED && stop_result != ESP_OK) {
    result = stop_result;
    ESP_LOGW(
      kTag, "esp_mesh_lite_stop returned: %s", esp_err_to_name(stop_result));
  }

  esp_err_t deinit_result = esp_mesh_lite_deinit();
  if (deinit_result != ESP_ERR_NOT_SUPPORTED && deinit_result != ESP_OK &&
      result == ESP_OK) {
    result = deinit_result;
    ESP_LOGW(kTag,
             "esp_mesh_lite_deinit returned: %s",
             esp_err_to_name(deinit_result));
  }

  mesh->mesh_lite_started = false;
  mesh->is_started = false;
  mesh->last_level = -1;
  g_mesh = NULL;

  if (WifiServiceActiveMode() == WIFI_SERVICE_MODE_MESH) {
    esp_err_t svc_result = WifiServiceRelease();
    if (result == ESP_OK && svc_result != ESP_OK) {
      result = svc_result;
    }
  }

  // Wi-Fi teardown triggers WIFI_EVENT_STA_STOP / DISCONNECTED events.
  // Mesh-Lite may re-arm its reconnect timer in those handlers, so yield briefly
  // and then stop background work again.
  vTaskDelay(pdMS_TO_TICKS(250));

  // One more best-effort stop after Wi-Fi teardown to ensure no background
  // tasks keep trying to scan.
  StopMeshLiteBackgroundWorkBestEffort();

  return result;
}
