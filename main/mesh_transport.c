#include "mesh_transport.h"

#include <stddef.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_core.h"
#include "esp_mesh_lite_port.h"
#include "esp_wifi.h"
#include "wifi_service.h"

static const char* kTag = "mesh";
static const char* kMeshSoftApSsid = "PT100_MESH";

static mesh_transport_t* g_mesh = NULL;

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

static const uint32_t kMeshMsgIdRecord = 0x01u;
static const uint32_t kMeshMsgIdTimeRequest = 0x02u;
static const uint32_t kMeshMsgIdTimeSync = 0x03u;

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
    ESP_LOGW(kTag,
             "esp_wifi_get_mac failed: %s",
             esp_err_to_name(result));
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
      .max_retry = 0,
      .retry_interval = 0,
      .data = data,
      .size = size,
      .raw_resend = raw_resend,
      .raw_send_fail = NULL,
    },
  };
  return esp_mesh_lite_send_msg(ESP_MESH_LITE_RAW_MSG, &config);
}

static esp_err_t
HandleMeshMessage(uint8_t* data,
                  uint32_t len,
                  uint8_t** out_data,
                  uint32_t* out_len,
                  uint32_t seq)
{
  (void)seq;
  if (out_data != NULL) {
    *out_data = NULL;
  }
  if (out_len != NULL) {
    *out_len = 0;
  }
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
  if (len > header_size) {
    const size_t payload_len = len - header_size;
    const size_t copy_len =
      (payload_len > sizeof(msg.payload)) ? sizeof(msg.payload) : payload_len;
    memcpy(&msg.payload, data + header_size, copy_len);
  }

  pt100_mesh_addr_t from = Pt100MeshAddrFromMac(msg.src_mac);

  switch (msg.type) {
    case MESH_MESSAGE_RECORD:
      if (len < header_size + sizeof(log_record_t)) {
        return ESP_ERR_INVALID_SIZE;
      }
      if (g_mesh->record_rx_callback != NULL) {
        g_mesh->record_rx_callback(
          &from, &msg.payload.record, g_mesh->record_rx_context);
      }
      break;
    case MESH_MESSAGE_TIME_REQUEST:
      if (g_mesh->is_root && TimeSyncIsSystemTimeValid()) {
        const int64_t now_seconds = (int64_t)time(NULL);
        mesh_message_t response = {
          .type = MESH_MESSAGE_TIME_SYNC,
          .payload.epoch_seconds = now_seconds,
        };
        if (PopulateMeshMessageSrc(&response) == ESP_OK) {
          const size_t response_size =
            MeshMessageHeaderSize() + sizeof(response.payload.epoch_seconds);
          (void)SendRawMessage(kMeshMsgIdTimeSync,
                               (const uint8_t*)&response,
                               response_size,
                               esp_mesh_lite_send_broadcast_raw_msg_to_child);
        }
      }
      break;
    case MESH_MESSAGE_TIME_SYNC:
      if (!g_mesh->is_root && g_mesh->time_sync != NULL) {
        if (len < header_size + sizeof(int64_t)) {
          return ESP_ERR_INVALID_SIZE;
        }
        (void)TimeSyncSetSystemEpoch(
          msg.payload.epoch_seconds, true, g_mesh->time_sync);
      }
      break;
    default:
      return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

static const esp_mesh_lite_raw_msg_action_t kMeshRawActions[] = {
  { kMeshMsgIdRecord, 0, HandleMeshMessage },
  { kMeshMsgIdTimeRequest, 0, HandleMeshMessage },
  { kMeshMsgIdTimeSync, 0, HandleMeshMessage },
  { 0, 0, NULL },
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

esp_err_t
MeshTransportStart(mesh_transport_t* mesh,
                   bool is_root,
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

  esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
  mesh_lite_config.join_mesh_ignore_router_status = true;
  mesh_lite_config.join_mesh_without_configured_wifi = !is_root;
  mesh_lite_config.softap_ssid = kMeshSoftApSsid;
  mesh_lite_config.softap_password = CONFIG_APP_MESH_AP_PASSWORD;

  ESP_LOGI(kTag, "starting Mesh-Lite (root=%d)", (int)is_root);
  esp_mesh_lite_init(&mesh_lite_config);
  esp_err_t raw_action_result =
    esp_mesh_lite_raw_msg_action_list_register(kMeshRawActions);
  if (raw_action_result != ESP_OK) {
    ESP_LOGW(kTag,
             "raw msg register failed: %s",
             esp_err_to_name(raw_action_result));
  }

  if (is_root) {
    (void)esp_mesh_lite_set_allowed_level(1);
    (void)esp_mesh_lite_allow_others_to_join(true);
  } else {
    (void)esp_mesh_lite_set_disallowed_level(1);
    (void)esp_mesh_lite_allow_others_to_join(false);
  }

  bool router_disabled = false;
#ifdef CONFIG_APP_MESH_DISABLE_ROUTER
  router_disabled = CONFIG_APP_MESH_DISABLE_ROUTER;
#endif

  if (!router_disabled && router_ssid != NULL && router_ssid[0] != '\0') {
    mesh_lite_sta_config_t router_config = { 0 };
    strlcpy((char*)router_config.ssid,
            router_ssid,
            sizeof(router_config.ssid));
    strlcpy((char*)router_config.password,
            (router_password != NULL) ? router_password : "",
            sizeof(router_config.password));

    esp_err_t router_result = esp_mesh_lite_set_router_config(&router_config);
    if (router_result != ESP_OK) {
      return router_result;
    }
  }

  esp_mesh_lite_start();

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
  memcpy(root_out, &mesh->root_address, sizeof(*root_out));
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
  return SendRawMessage(
    kMeshMsgIdRecord,
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
  return SendRawMessage(
    kMeshMsgIdTimeSync,
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
  return SendRawMessage(
    kMeshMsgIdTimeRequest,
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

  return result;
}
