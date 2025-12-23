#include "diagnostics/diag_mesh.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_port.h"
#include "esp_netif_ip_addr.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"
#include "mesh_addr.h"
#include "mesh_transport.h"
#include "sdkconfig.h"
#include "wifi_service.h"

static const char* kTag = "diag_mesh";

// Heap integrity checks can trigger asserts when corruption already exists.
// Keep them opt-in for diagnostics.
#ifndef DIAG_MESH_ENABLE_HEAP_INTEGRITY_CHECKS
#define DIAG_MESH_ENABLE_HEAP_INTEGRITY_CHECKS 0
#endif

typedef struct
{
  size_t free_8bit;
  size_t free_total;
  size_t min_free;
} heap_snapshot_t;

typedef struct
{
  bool started;
  bool connected;
  bool is_root;
  int layer;
  pt100_mesh_addr_t parent;
  bool parent_known;
  uint8_t mesh_id;
  bool mesh_id_known;
  pt100_mesh_addr_t root_addr;
  bool root_addr_known;
  esp_ip4_addr_t root_ip;
  bool root_ip_known;
  int routing_table_size;
  int channel;
  wifi_service_mode_t owner_mode;
  int last_disconnect_reason;
} mesh_status_t;

typedef struct
{
  pt100_mesh_addr_t mesh_id;
  bool mesh_id_valid;
  size_t router_ssid_len;
  size_t router_password_len;
  bool router_password_valid;
  size_t mesh_ap_password_len;
  bool mesh_ap_password_valid;
} mesh_diag_config_t;

static const char*
YesNo(bool value)
{
  return value ? "yes" : "no";
}

static void
MeshDiagHeapCheck(const diag_ctx_t* ctx, const char* label)
{
#if DIAG_MESH_ENABLE_HEAP_INTEGRITY_CHECKS
  DiagHeapCheck(ctx, label);
#else
  (void)ctx;
  (void)label;
#endif
}

static heap_snapshot_t
CaptureHeapSnapshot(void)
{
  heap_snapshot_t snapshot;
#if DIAG_MESH_ENABLE_HEAP_INTEGRITY_CHECKS
  heap_caps_check_integrity_all(true);
#endif
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
FormatMac(char* out, size_t out_size, const uint8_t mac[6])
{
  if (out == NULL || out_size == 0 || mac == NULL) {
    return;
  }
  snprintf(out, out_size, MACSTR, MAC2STR(mac));
}

static const char*
WifiModeToString(wifi_service_mode_t mode)
{
  switch (mode) {
    case WIFI_SERVICE_MODE_NONE:
      return "NONE";
    case WIFI_SERVICE_MODE_DIAGNOSTIC_STA:
      return "DIAGNOSTIC_STA";
    case WIFI_SERVICE_MODE_MESH:
      return "MESH";
    default:
      return "UNKNOWN";
  }
}

static void
PrintStackSizeWarning(const diag_ctx_t* ctx)
{
#if CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE < 4096
  if (ctx != NULL && ctx->verbosity >= kDiagVerbosity0) {
    printf("      note: CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE=%d; "
           "diagnostic mesh/wifi logging may need >=4096 to avoid "
           "sys_evt stack overflow\n",
           CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE);
  }
#endif
}

static bool
ParseMeshId(const char* mesh_id_string, pt100_mesh_addr_t* mesh_id_out)
{
  if (mesh_id_string == NULL || mesh_id_out == NULL) {
    return false;
  }

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
    mesh_id_out->addr[index] = (uint8_t)values[index];
  }
  return true;
}

static esp_err_t
ValidateMeshConfig(bool start_as_root, mesh_diag_config_t* config)
{
  if (config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(config, 0, sizeof(*config));

  config->mesh_id_valid = ParseMeshId(CONFIG_APP_MESH_ID_HEX, &config->mesh_id);
  if (!config->mesh_id_valid) {
    return ESP_ERR_INVALID_ARG;
  }

  config->router_ssid_len = strlen(CONFIG_APP_WIFI_ROUTER_SSID);
  config->router_password_len = strlen(CONFIG_APP_WIFI_ROUTER_PASSWORD);
  config->router_password_valid =
    (config->router_password_len == 0 ||
     (config->router_password_len >= 8 && config->router_password_len <= 63));

  config->mesh_ap_password_len = strlen(CONFIG_APP_MESH_AP_PASSWORD);
  config->mesh_ap_password_valid =
    (config->mesh_ap_password_len == 0 ||
     (config->mesh_ap_password_len >= 8 && config->mesh_ap_password_len <= 63));

  if (!config->mesh_ap_password_valid) {
    return ESP_ERR_INVALID_ARG;
  }

  if (start_as_root) {
    if (config->router_ssid_len == 0) {
      return ESP_ERR_INVALID_ARG;
    }
    if (!config->router_password_valid) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  return ESP_OK;
}

static bool
MeshReady(bool expect_root, const mesh_transport_t* mesh, int* layer_out)
{
  if (mesh == NULL || !MeshTransportIsStarted(mesh)) {
    if (layer_out != NULL) {
      *layer_out = -1;
    }
    return false;
  }

  const int layer = (int)esp_mesh_lite_get_level();
  if (layer_out != NULL) {
    *layer_out = layer;
  }

  if (expect_root) {
    return layer == 1;
  }
  return layer >= 2;
}

static esp_err_t
WaitForMeshReady(const mesh_transport_t* mesh,
                 bool expect_root,
                 int timeout_ms,
                 bool* ready_out,
                 int* waited_ms_out,
                 int* layer_out)
{
  if (ready_out != NULL) {
    *ready_out = false;
  }
  if (waited_ms_out != NULL) {
    *waited_ms_out = 0;
  }
  if (layer_out != NULL) {
    *layer_out = -1;
  }
  if (mesh == NULL || !MeshTransportIsStarted(mesh)) {
    return ESP_ERR_INVALID_STATE;
  }

  const TickType_t start_ticks = xTaskGetTickCount();
  const TickType_t timeout_ticks =
    pdMS_TO_TICKS((timeout_ms > 0) ? timeout_ms : 10000);
  bool ready = MeshReady(expect_root, mesh, layer_out);

  while (!ready && (xTaskGetTickCount() - start_ticks) < timeout_ticks) {
    vTaskDelay(pdMS_TO_TICKS(200));
    ready = MeshReady(expect_root, mesh, layer_out);
  }

  const TickType_t elapsed = xTaskGetTickCount() - start_ticks;
  if (waited_ms_out != NULL) {
    *waited_ms_out = (int)pdTICKS_TO_MS(elapsed);
  }
  if (ready_out != NULL) {
    *ready_out = ready;
  }
  return ready ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void
CaptureMeshStatus(mesh_status_t* status, const mesh_transport_t* mesh)
{
  if (status == NULL) {
    return;
  }
  memset(status, 0, sizeof(*status));
  status->layer = -1;
  status->channel = -1;
  status->last_disconnect_reason = -1;
  status->owner_mode = WifiServiceActiveMode();

  if (mesh == NULL) {
    return;
  }
  status->started = MeshTransportIsStarted(mesh);
  status->connected = MeshTransportIsConnected(mesh);
  if (!status->started) {
    return;
  }

  status->is_root = mesh->is_root;
  status->layer = (int)esp_mesh_lite_get_level();
  status->mesh_id = esp_mesh_lite_get_mesh_id();
  status->mesh_id_known = true;

  mesh_lite_ap_record_t ap_record = { 0 };
  if (esp_mesh_lite_get_ap_record(&ap_record) == ESP_OK) {
    memcpy(status->parent.addr, ap_record.bssid, sizeof(status->parent.addr));
    status->parent_known = true;
    status->channel = ap_record.primary;
  }

  pt100_mesh_addr_t root_addr = { 0 };
  if (MeshTransportGetRootAddress(mesh, &root_addr) == ESP_OK &&
      !Pt100MeshAddrIsZero(&root_addr)) {
    memcpy(&status->root_addr, &root_addr, sizeof(status->root_addr));
    status->root_addr_known = true;
  }

  esp_ip_addr_t root_ip = { 0 };
  if (esp_mesh_lite_get_root_ip(ESP_IPADDR_TYPE_V4, &root_ip) == ESP_OK) {
    status->root_ip.addr = root_ip.u_addr.ip4.addr;
    status->root_ip_known = true;
  }

  status->routing_table_size = (int)esp_mesh_lite_get_mesh_node_number();
}

static void
PrintRoutingTable(const diag_ctx_t* ctx)
{
  if (ctx == NULL || ctx->verbosity < kDiagVerbosity2) {
    return;
  }

#if CONFIG_MESH_LITE_NODE_INFO_REPORT
  uint32_t total_nodes = 0;
  const node_info_list_t* node = esp_mesh_lite_get_nodes_list(&total_nodes);
  if (node == NULL || total_nodes == 0) {
    return;
  }

  const int to_show = (total_nodes < 10u) ? (int)total_nodes : 10;
  printf("      mesh nodes: %u entries (showing %d)\n",
         (unsigned)total_nodes,
         to_show);
  int index = 0;
  for (const node_info_list_t* entry = node;
       entry != NULL && index < to_show;
       entry = entry->next, ++index) {
    char mac[20] = { 0 };
    const esp_mesh_lite_node_info_t* info = entry->node;
    if (info != NULL) {
      FormatMac(mac, sizeof(mac), info->mac_addr);
    }
    ip4_addr_t node_ip = { 0 };
    const char* node_ip_str = "<unknown>";
    if (info != NULL && info->ip_addr != 0) {
      node_ip.addr = info->ip_addr;
      node_ip_str = ip4addr_ntoa(&node_ip);
    }
    printf("        %2d. %s level=%u ip=%s\n",
           index + 1,
           (info != NULL) ? mac : "<unknown>",
           (info != NULL) ? (unsigned)info->level : 0u,
           node_ip_str);
  }
#else
  (void)ctx;
#endif
}

int
RunDiagMesh(const app_runtime_t* runtime,
            bool full,
            bool start,
            bool stop,
            bool force_root,
            int timeout_ms,
            diag_verbosity_t verbosity)
{
  const bool mesh_available = (runtime != NULL && runtime->mesh != NULL);
  const bool mesh_started_before =
    MeshTransportIsStarted(mesh_available ? runtime->mesh : NULL);
  const bool mesh_connected_before =
    MeshTransportIsConnected(mesh_available ? runtime->mesh : NULL);
  bool mesh_started_by_diag = false;
  const bool wait_for_ready = (start || mesh_started_before) && full;
  const bool perform_stop = stop || (start && !mesh_started_before);
  const int total_steps = 7;

  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "Mesh", verbosity);

  int step_index = 1;
  const bool runtime_running = RuntimeIsRunning();
  const esp_err_t runtime_result =
    (!mesh_available || runtime_running) ? ESP_ERR_INVALID_STATE : ESP_OK;
  DiagReportStep(
    &ctx,
    step_index++,
    total_steps,
    "runtime idle",
    runtime_result,
    (!mesh_available)
      ? "runtime not initialized"
      : (runtime_running ? "stop runtime first: run stop" : "idle"));
  if (runtime_result != ESP_OK) {
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  const wifi_service_mode_t active_mode = WifiServiceActiveMode();
  const bool wifi_owner_ok = (active_mode == WIFI_SERVICE_MODE_NONE ||
                              active_mode == WIFI_SERVICE_MODE_MESH);
  DiagReportStep(
    &ctx,
    step_index++,
    total_steps,
    "wifi owner check",
    wifi_owner_ok ? ESP_OK : ESP_ERR_INVALID_STATE,
    "wifi_service_mode=%s mesh_started_before=%s mesh_connected=%s",
    WifiModeToString(active_mode),
    YesNo(mesh_started_before),
    YesNo(mesh_connected_before));
  if (!wifi_owner_ok) {
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  PrintStackSizeWarning(&ctx);

  heap_snapshot_t wifi_before = CaptureHeapSnapshot();
  heap_snapshot_t wifi_after = wifi_before;
  MeshDiagHeapCheck(&ctx, "pre_wifi");
  bool wifi_acquired = false;
  const bool need_wifi = start && !mesh_started_before;
  esp_err_t wifi_result = ESP_OK;
  if (!mesh_available) {
    wifi_result = ESP_ERR_INVALID_STATE;
  } else if (!wifi_owner_ok) {
    wifi_result = ESP_ERR_INVALID_STATE;
  } else if (need_wifi) {
    wifi_result = WifiServiceAcquire(WIFI_SERVICE_MODE_MESH);
    wifi_acquired = (wifi_result == ESP_OK);
  }
  wifi_after = CaptureHeapSnapshot();
  MeshDiagHeapCheck(&ctx, "post_wifi");
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "wifi/net stack",
                 wifi_result,
                 "need_wifi=%s acquired=%s mode_before=%s mode_after=%s"
                 " heap8_before=%u heap8_after=%u min_free=%u",
                 YesNo(need_wifi),
                 YesNo(wifi_acquired),
                 WifiModeToString(active_mode),
                 WifiModeToString(WifiServiceActiveMode()),
                 (unsigned)wifi_before.free_8bit,
                 (unsigned)wifi_after.free_8bit,
                 (unsigned)wifi_after.min_free);
  PrintHeapSnapshot(&ctx, "wifi_before", &wifi_before);
  PrintHeapSnapshot(&ctx, "wifi_after", &wifi_after);

#if CONFIG_APP_NODE_IS_ROOT
  const bool default_root = true;
#else
  const bool default_root = false;
#endif
  const bool start_as_root = force_root || default_root;

  mesh_diag_config_t diag_config;
  memset(&diag_config, 0, sizeof(diag_config));
  const esp_err_t config_result =
    (mesh_available && wifi_owner_ok)
      ? ValidateMeshConfig(start_as_root, &diag_config)
      : ESP_ERR_INVALID_STATE;
  char mesh_id_str[20] = { 0 };
  if (diag_config.mesh_id_valid) {
    FormatMac(mesh_id_str, sizeof(mesh_id_str), diag_config.mesh_id.addr);
  }
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "mesh config",
                 config_result,
                 "mesh_id=%s router_ssid_len=%u router_pwd_len=%u ap_pwd_len=%u"
                 " root_required_ssid=%s pwd_valid=%s",
                 diag_config.mesh_id_valid ? mesh_id_str : "<invalid>",
                 (unsigned)diag_config.router_ssid_len,
                 (unsigned)diag_config.router_password_len,
                 (unsigned)diag_config.mesh_ap_password_len,
                 YesNo(start_as_root),
                 YesNo(diag_config.router_password_valid));

  heap_snapshot_t start_before = CaptureHeapSnapshot();
  heap_snapshot_t start_after = start_before;
  MeshDiagHeapCheck(&ctx, "pre_mesh_start");
  esp_err_t start_result = ESP_OK;
  bool mesh_started = mesh_started_before;
  bool mesh_connected = mesh_connected_before;

  if (start) {
    if (!mesh_available) {
      start_result = ESP_ERR_INVALID_STATE;
    } else if (!wifi_owner_ok || config_result != ESP_OK) {
      start_result = ESP_ERR_INVALID_STATE;
    } else if (mesh_started_before) {
      start_result = ESP_OK;
    } else {
      start_result = MeshTransportStart(runtime->mesh,
                                        start_as_root,
                                        CONFIG_APP_WIFI_ROUTER_SSID,
                                        CONFIG_APP_WIFI_ROUTER_PASSWORD,
                                        NULL,
                                        NULL,
                                        runtime->time_sync);
      mesh_started_by_diag = (start_result == ESP_OK);
    }

    mesh_started =
      MeshTransportIsStarted(mesh_available ? runtime->mesh : NULL);
    mesh_connected =
      MeshTransportIsConnected(mesh_available ? runtime->mesh : NULL);
    start_after = CaptureHeapSnapshot();
  }
  MeshDiagHeapCheck(&ctx, "post_mesh_start");
  const wifi_service_mode_t mode_after_start = WifiServiceActiveMode();

  bool ready = mesh_connected;
  int waited_ms = 0;
  int wait_layer =
    mesh_started ? (int)esp_mesh_lite_get_level() : -1;
  esp_err_t wait_result = ESP_OK;
  if (wait_for_ready) {
    if (!mesh_started) {
      wait_result = ESP_ERR_INVALID_STATE;
    } else {
      wait_result = WaitForMeshReady(mesh_available ? runtime->mesh : NULL,
                                     start_as_root,
                                     timeout_ms,
                                     &ready,
                                     &waited_ms,
                                     &wait_layer);
    }
  }

  const esp_err_t mesh_start_step_result =
    (start_result != ESP_OK) ? start_result : wait_result;
  DiagReportStep(
    &ctx,
    step_index++,
    total_steps,
    "mesh start/wait",
    mesh_start_step_result,
    "requested=%s root=%s started_by_diag=%s started=%s connected=%s ready=%s"
    " waited_ms=%d layer=%d wifi_mode=%s"
    " heap8_before=%u heap8_after=%u min_free=%u",
    YesNo(start),
    YesNo(start_as_root),
    YesNo(mesh_started_by_diag),
    YesNo(mesh_started),
    YesNo(mesh_connected),
    YesNo(ready),
    waited_ms,
    wait_layer,
    WifiModeToString(mode_after_start),
    (unsigned)start_before.free_8bit,
    (unsigned)start_after.free_8bit,
    (unsigned)start_after.min_free);
  PrintHeapSnapshot(&ctx, "mesh_start_before", &start_before);
  PrintHeapSnapshot(&ctx, "mesh_start_after", &start_after);

  mesh_status_t status;
  CaptureMeshStatus(&status, mesh_available ? runtime->mesh : NULL);

  char parent_str[20] = { 0 };
  if (status.parent_known) {
    FormatMac(parent_str, sizeof(parent_str), status.parent.addr);
  }
  char status_mesh_id[20] = { 0 };
  if (status.mesh_id_known) {
    snprintf(status_mesh_id,
             sizeof(status_mesh_id),
             "0x%02x",
             status.mesh_id);
  }
  char root_addr_str[20] = { 0 };
  if (status.root_addr_known) {
    FormatMac(root_addr_str, sizeof(root_addr_str), status.root_addr.addr);
  }

  ip4_addr_t root_ip_lwip = { 0 };
  char root_ip_buf[IP4ADDR_STRLEN_MAX] = { 0 };
  const char* root_ip_str = "<unknown>";
  if (status.root_ip_known) {
    root_ip_lwip.addr = status.root_ip.addr;
    root_ip_str = ip4addr_ntoa_r(&root_ip_lwip, root_ip_buf, sizeof(root_ip_buf));
  }

  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "mesh status",
                 ESP_OK,
                 "started=%s connected=%s root=%s layer=%d parent=%s mesh_id=%s"
                 " root_addr=%s rt_size=%d owner=%s last_disc_reason=%d"
                 " root_ip=%s",
                 YesNo(status.started),
                 YesNo(status.connected),
                 YesNo(status.is_root),
                 status.layer,
                 status.parent_known ? parent_str : "<none>",
                 status.mesh_id_known ? status_mesh_id : "<unknown>",
                 status.root_addr_known ? root_addr_str : "<none>",
                 status.routing_table_size,
                 WifiModeToString(status.owner_mode),
                 status.last_disconnect_reason,
                 root_ip_str);

  PrintRoutingTable(&ctx);

  heap_snapshot_t stop_before = CaptureHeapSnapshot();
  heap_snapshot_t stop_after = stop_before;
  MeshDiagHeapCheck(&ctx, "pre_mesh_stop");
  esp_err_t stop_result = ESP_OK;
  if (!mesh_available) {
    stop_result = ESP_ERR_INVALID_STATE;
  } else if (perform_stop && MeshTransportIsStarted(runtime->mesh)) {
    stop_result = MeshTransportStop(runtime->mesh);
  }

  if (wifi_acquired) {
    const esp_err_t release_result = WifiServiceRelease();
    if (stop_result == ESP_OK && release_result != ESP_OK) {
      stop_result = release_result;
    }
  }
  stop_after = CaptureHeapSnapshot();
  MeshDiagHeapCheck(&ctx, "post_mesh_stop");
  const wifi_service_mode_t mode_after_stop = WifiServiceActiveMode();
  DiagReportStep(
    &ctx,
    step_index++,
    total_steps,
    "teardown",
    stop_result,
    "stop_requested=%s started_before=%s started_after=%s"
    " wifi_mode_after=%s heap8_before=%u heap8_after=%u"
    " min_free=%u",
    YesNo(perform_stop),
    YesNo(mesh_started_before),
    YesNo(MeshTransportIsStarted(mesh_available ? runtime->mesh : NULL)),
    WifiModeToString(mode_after_stop),
    (unsigned)stop_before.free_8bit,
    (unsigned)stop_after.free_8bit,
    (unsigned)stop_after.min_free);
  PrintHeapSnapshot(&ctx, "stop_before", &stop_before);
  PrintHeapSnapshot(&ctx, "stop_after", &stop_after);

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
