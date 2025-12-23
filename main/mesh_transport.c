#include "mesh_transport.h"

#include <string.h>

#include "esp_log.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_core.h"
#include "esp_mesh_lite_port.h"
#include "wifi_service.h"

static const char* kTag = "mesh";
static const char* kMeshSoftApSsid = "PT100_MESH";

static mesh_transport_t* g_mesh = NULL;

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
  return ESP_ERR_NOT_SUPPORTED;
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
  (void)epoch_seconds;
  return ESP_ERR_NOT_SUPPORTED;
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
  return ESP_ERR_NOT_SUPPORTED;
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
