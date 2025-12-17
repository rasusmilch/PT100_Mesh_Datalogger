#include "mesh_transport.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <string.h>
#include <time.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char* kTag = "mesh";

typedef enum
{
  MESH_MSG_TYPE_RECORD = 1,
  MESH_MSG_TYPE_TIME_SYNC = 2,
  MESH_MSG_TYPE_TIME_REQUEST = 3,
} mesh_msg_type_t;

#pragma pack(push, 1)
typedef struct
{
  uint8_t type;
  uint8_t reserved[3];
  int64_t epoch_seconds; // used for time sync / request
  log_record_t record;   // valid when type==MESH_MSG_TYPE_RECORD
} mesh_message_t;
#pragma pack(pop)

static mesh_transport_t* g_mesh = NULL;

static bool
ParseMeshIdFromConfig(const char* mesh_id_string, uint8_t* mesh_id_out)
{
  // Expected: "77:77:77:77:77:77"
  int values[6] = { 0 };
  if (sscanf(mesh_id_string,
             "%x:%x:%x:%x:%x:%x",
             &values[0],
             &values[1],
             &values[2],
             &values[3],
             &values[4],
             &values[5]) != 6) {
    return false;
  }
  for (int index = 0; index < 6; ++index) {
    mesh_id_out[index] = (uint8_t)values[index];
  }
  return true;
}

static void
MeshEventHandler(void* arg,
                 esp_event_base_t event_base,
                 int32_t event_id,
                 void* event_data)
{
  if (g_mesh == NULL) {
    return;
  }

  mesh_event_info_t* info = (mesh_event_info_t*)event_data;

  switch (event_id) {
    case MESH_EVENT_STARTED:
      ESP_LOGI(kTag, "MESH_EVENT_STARTED");
      g_mesh->is_started = true;
      break;

    case MESH_EVENT_STOPPED:
      ESP_LOGI(kTag, "MESH_EVENT_STOPPED");
      g_mesh->is_started = false;
      g_mesh->is_connected = false;
      break;

    case MESH_EVENT_PARENT_CONNECTED:
      ESP_LOGI(kTag,
               "MESH_EVENT_PARENT_CONNECTED, layer=%d",
               (int)info->connected.self_layer);
      g_mesh->is_connected = true;
      break;

    case MESH_EVENT_PARENT_DISCONNECTED:
      ESP_LOGI(kTag, "MESH_EVENT_PARENT_DISCONNECTED");
      g_mesh->is_connected = false;
      break;

    case MESH_EVENT_ROOT_ADDRESS:
      memcpy(&g_mesh->root_address, &info->root_addr.addr, sizeof(mesh_addr_t));
      ESP_LOGI(kTag,
               "MESH_EVENT_ROOT_ADDRESS: " MACSTR,
               MAC2STR(g_mesh->root_address.addr));
      break;

    default:
      break;
  }
}

static esp_err_t
SendMessageTo(const mesh_transport_t* mesh,
              const mesh_addr_t* destination,
              const mesh_message_t* message)
{
  mesh_data_t data;
  data.data = (uint8_t*)message;
  data.size = sizeof(*message);
  data.proto = MESH_PROTO_BIN;
  data.tos = MESH_TOS_P2P;

  // Set flag 0 for normal P2P.
  return esp_mesh_send(destination, &data, 0, NULL, 0);
}

static void
MeshRxTask(void* context)
{
  (void)context;

  uint8_t rx_buffer[sizeof(mesh_message_t)];
  mesh_data_t data;
  data.data = rx_buffer;
  data.size = sizeof(rx_buffer);
  data.proto = MESH_PROTO_BIN;
  data.tos = MESH_TOS_P2P;

  while (true) {
    if (g_mesh == NULL || !g_mesh->is_started) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    mesh_addr_t from;
    int flag = 0;
    data.size = sizeof(rx_buffer);

    esp_err_t result =
      esp_mesh_recv(&from, &data, pdMS_TO_TICKS(500), &flag, NULL, 0);
    if (result == ESP_ERR_MESH_TIMEOUT) {
      continue;
    }
    if (result != ESP_OK) {
      ESP_LOGW(kTag, "esp_mesh_recv: %s", esp_err_to_name(result));
      continue;
    }
    if (data.size != sizeof(mesh_message_t)) {
      continue;
    }

    const mesh_message_t* message = (const mesh_message_t*)rx_buffer;

    if (message->type == MESH_MSG_TYPE_RECORD) {
      if (g_mesh->record_rx_callback != NULL) {
        g_mesh->record_rx_callback(
          &from, &message->record, g_mesh->record_rx_context);
      }
    } else if (message->type == MESH_MSG_TYPE_TIME_SYNC) {
      // Apply time update on all nodes.
      (void)TimeSyncSetSystemEpoch(
        message->epoch_seconds, true, g_mesh->time_sync);
    } else if (message->type == MESH_MSG_TYPE_TIME_REQUEST) {
      // Only root should respond.
      if (g_mesh->is_root) {
        mesh_message_t reply;
        memset(&reply, 0, sizeof(reply));
        reply.type = MESH_MSG_TYPE_TIME_SYNC;
        reply.epoch_seconds = (int64_t)time(NULL);
        (void)SendMessageTo(g_mesh, &from, &reply);
      }
    }
  }
}

static esp_err_t
InitWifiAndMesh(bool is_root,
                const char* router_ssid,
                const char* router_password)
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_t* netif_sta = NULL;
  esp_netif_t* netif_ap = NULL;
  ESP_ERROR_CHECK(
    esp_netif_create_default_wifi_mesh_netifs(&netif_sta, &netif_ap));

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));

  // MESH uses both STA and SoftAP.
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_ERROR_CHECK(esp_event_handler_register(
    MESH_EVENT, ESP_EVENT_ANY_ID, &MeshEventHandler, NULL));

  ESP_ERROR_CHECK(esp_mesh_init());
  ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));
  ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1.0f));
  ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(30));
  ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, true));

  mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
  cfg.channel = CONFIG_APP_MESH_CHANNEL;

  uint8_t mesh_id[6] = { 0 };
  if (!ParseMeshIdFromConfig(CONFIG_APP_MESH_ID_HEX, mesh_id)) {
    ESP_LOGE(kTag, "Invalid mesh ID config: %s", CONFIG_APP_MESH_ID_HEX);
    return ESP_ERR_INVALID_ARG;
  }
  memcpy((uint8_t*)&cfg.mesh_id, mesh_id, 6);

  // Router config is required for root connection to external network.
  if (router_ssid != NULL && router_ssid[0] != '\0') {
    strncpy((char*)cfg.router.ssid, router_ssid, sizeof(cfg.router.ssid));
    cfg.router.ssid_len = strlen(router_ssid);
  }
  if (router_password != NULL && router_password[0] != '\0') {
    strncpy(
      (char*)cfg.router.password, router_password, sizeof(cfg.router.password));
  }

  // SoftAP for children.
  cfg.mesh_ap.max_connection = CONFIG_APP_MESH_AP_CONNECTIONS;
  strncpy((char*)cfg.mesh_ap.password,
          CONFIG_APP_MESH_AP_PASSWORD,
          sizeof(cfg.mesh_ap.password));

  ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

  if (is_root) {
    ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));
  } else {
    ESP_ERROR_CHECK(esp_mesh_set_type(MESH_NODE));
  }

  ESP_ERROR_CHECK(esp_mesh_start());
  ESP_LOGI(kTag, "mesh started (is_root=%d)", (int)is_root);
  return ESP_OK;
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

  esp_err_t result = InitWifiAndMesh(is_root, router_ssid, router_password);
  if (result != ESP_OK) {
    return result;
  }

  xTaskCreate(MeshRxTask, "mesh_rx", 4096, NULL, 5, NULL);
  return ESP_OK;
}

bool
MeshTransportIsConnected(const mesh_transport_t* mesh)
{
  return mesh != NULL && mesh->is_connected;
}

esp_err_t
MeshTransportSendRecord(const mesh_transport_t* mesh,
                        const log_record_t* record)
{
  if (mesh == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!mesh->is_started || !mesh->is_connected) {
    return ESP_ERR_INVALID_STATE;
  }
  if (mesh->is_root) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  mesh_message_t message;
  memset(&message, 0, sizeof(message));
  message.type = MESH_MSG_TYPE_RECORD;
  memcpy(&message.record, record, sizeof(*record));

  return SendMessageTo(mesh, &mesh->root_address, &message);
}

esp_err_t
MeshTransportBroadcastTime(const mesh_transport_t* mesh, int64_t epoch_seconds)
{
  if (mesh == NULL || !mesh->is_root) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!mesh->is_started || !mesh->is_connected) {
    return ESP_ERR_INVALID_STATE;
  }

  mesh_message_t message;
  memset(&message, 0, sizeof(message));
  message.type = MESH_MSG_TYPE_TIME_SYNC;
  message.epoch_seconds = epoch_seconds;

  // Send to all nodes in the routing table.
  const int routing_table_size = esp_mesh_get_routing_table_size();
  if (routing_table_size <= 0) {
    return ESP_OK;
  }

  mesh_addr_t routing_table[64];
  int routing_table_entries = 0;
  esp_err_t result = esp_mesh_get_routing_table(
    routing_table, sizeof(routing_table), &routing_table_entries);
  if (result != ESP_OK) {
    return result;
  }

  for (int index = 0; index < routing_table_entries; ++index) {
    // Skip self (root).
    if (memcmp(routing_table[index].addr, mesh->root_address.addr, 6) == 0) {
      continue;
    }
    (void)SendMessageTo(mesh, &routing_table[index], &message);
  }
  return ESP_OK;
}

esp_err_t
MeshTransportRequestTime(const mesh_transport_t* mesh)
{
  if (mesh == NULL || mesh->is_root) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!mesh->is_started || !mesh->is_connected) {
    return ESP_ERR_INVALID_STATE;
  }

  mesh_message_t message;
  memset(&message, 0, sizeof(message));
  message.type = MESH_MSG_TYPE_TIME_REQUEST;
  message.epoch_seconds = 0;
  return SendMessageTo(mesh, &mesh->root_address, &message);
}
