#include "diagnostics/diag_mesh.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_mesh.h"
#include "esp_netif_ip_addr.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mesh_transport.h"
#include "sdkconfig.h"
#include "wifi_service.h"

static const char* kTag = "diag_mesh";

// Heap integrity checks can trigger asserts when corruption already exists.
// Keep them opt-in for diagnostics.
#ifndef DIAG_MESH_ENABLE_HEAP_INTEGRITY_CHECKS
#define DIAG_MESH_ENABLE_HEAP_INTEGRITY_CHECKS 0
#endif

#define DIAG_MESH_EVENT_PARENT_CONNECTED BIT0
#define DIAG_MESH_EVENT_LAYER_CHANGED BIT1
#define DIAG_MESH_EVENT_ROOT_IP BIT2
#define DIAG_MESH_EVENT_PARENT_DISCONNECTED BIT3
#define DIAG_MESH_EVENT_TODS BIT4
#define DIAG_MESH_EVENT_ROOT_ADDR BIT5

typedef struct
{
  size_t free_8bit;
  size_t free_total;
  size_t min_free;
} heap_snapshot_t;

typedef struct
{
  EventGroupHandle_t group;
  esp_event_handler_instance_t mesh_handler;
  esp_event_handler_instance_t ip_handler;
  int last_layer;
  int last_disconnect_reason;
  mesh_addr_t parent;
  bool parent_known;
  mesh_addr_t root_addr;
  bool root_known;
  esp_ip4_addr_t root_ip;
  bool root_has_ip;
  bool to_ds;
  bool to_ds_known;
} mesh_diag_events_t;

typedef struct
{
  bool started;
  bool connected;
  bool is_root;
  int layer;
  mesh_addr_t parent;
  bool parent_known;
  mesh_addr_t mesh_id;
  bool mesh_id_known;
  mesh_addr_t root_addr;
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
  mesh_addr_t mesh_id;
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
ParseMeshId(const char* mesh_id_string, mesh_addr_t* mesh_id_out)
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

static void
MeshDiagEventHandler(void* arg,
                     esp_event_base_t event_base,
                     int32_t event_id,
                     void* event_data)
{
  mesh_diag_events_t* events = (mesh_diag_events_t*)arg;
  if (events == NULL || events->group == NULL || event_base != MESH_EVENT) {
    return;
  }

  switch (event_id) {
    case MESH_EVENT_PARENT_CONNECTED: {
      const mesh_event_connected_t* info =
        (const mesh_event_connected_t*)event_data;
      events->last_layer =
        (info != NULL) ? (int)info->self_layer : esp_mesh_get_layer();

      if (info != NULL) {
        memcpy(events->parent.addr, info->connected.bssid, 6);
        events->parent_known = true;
      } else {
        mesh_addr_t parent;
        memset(&parent, 0, sizeof(parent));
        if (esp_mesh_get_parent_bssid(&parent) == ESP_OK) {
          memcpy(events->parent.addr, parent.addr, 6);
          events->parent_known = true;
        }
      }

      xEventGroupSetBits(events->group,
                         DIAG_MESH_EVENT_PARENT_CONNECTED |
                           DIAG_MESH_EVENT_LAYER_CHANGED);
      break;
    }

    case MESH_EVENT_LAYER_CHANGE: {
      const mesh_event_layer_change_t* info =
        (const mesh_event_layer_change_t*)event_data;
      events->last_layer =
        (info != NULL) ? (int)info->new_layer : esp_mesh_get_layer();
      xEventGroupSetBits(events->group, DIAG_MESH_EVENT_LAYER_CHANGED);
      break;
    }

    case MESH_EVENT_PARENT_DISCONNECTED: {
      const mesh_event_disconnected_t* info =
        (const mesh_event_disconnected_t*)event_data;
      events->last_disconnect_reason = (info != NULL) ? info->reason : -1;
      events->parent_known = false;
      xEventGroupSetBits(events->group, DIAG_MESH_EVENT_PARENT_DISCONNECTED);
      break;
    }

    case MESH_EVENT_ROOT_ADDRESS: {
      const mesh_event_root_address_t* info =
        (const mesh_event_root_address_t*)event_data;
      if (info != NULL) {
        memcpy(events->root_addr.addr, info->addr, 6);
        events->root_known = true;
      }
      xEventGroupSetBits(events->group, DIAG_MESH_EVENT_ROOT_ADDR);
      break;
    }

    case MESH_EVENT_TODS_STATE: {
      const mesh_event_toDS_state_t* info =
        (const mesh_event_toDS_state_t*)event_data;
      events->to_ds_known = true;
      events->to_ds = (info != NULL) ? (*info) : false;
      xEventGroupSetBits(events->group, DIAG_MESH_EVENT_TODS);
      break;
    }

    default:
      break;
  }
}

static void
MeshDiagIpEventHandler(void* arg,
                       esp_event_base_t event_base,
                       int32_t event_id,
                       void* event_data)
{
  mesh_diag_events_t* events = (mesh_diag_events_t*)arg;
  if (events == NULL || events->group == NULL || event_base != IP_EVENT ||
      event_id != IP_EVENT_STA_GOT_IP) {
    return;
  }

  const ip_event_got_ip_t* got_ip = (const ip_event_got_ip_t*)event_data;
  events->root_has_ip = esp_mesh_is_root();
  if (events->root_has_ip && got_ip != NULL) {
    events->root_ip = got_ip->ip_info.ip;
  }
  if (events->root_has_ip) {
    xEventGroupSetBits(events->group, DIAG_MESH_EVENT_ROOT_IP);
  }
}

static void
CleanupEventTracking(mesh_diag_events_t* events);

static esp_err_t
InitEventTracking(mesh_diag_events_t* events)
{
  if (events == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(events, 0, sizeof(*events));
  events->last_layer = -1;
  events->last_disconnect_reason = -1;
  events->group = xEventGroupCreate();
  if (events->group == NULL) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t result = esp_event_handler_instance_register(MESH_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &MeshDiagEventHandler,
                                                         events,
                                                         &events->mesh_handler);
  if (result != ESP_OK) {
    CleanupEventTracking(events);
    return result;
  }

  result = esp_event_handler_instance_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &MeshDiagIpEventHandler,
                                               events,
                                               &events->ip_handler);
  if (result != ESP_OK) {
    CleanupEventTracking(events);
    return result;
  }

  return ESP_OK;
}

static void
CleanupEventTracking(mesh_diag_events_t* events)
{
  if (events == NULL) {
    return;
  }

  if (events->mesh_handler != NULL) {
    (void)esp_event_handler_instance_unregister(
      MESH_EVENT, ESP_EVENT_ANY_ID, events->mesh_handler);
    events->mesh_handler = NULL;
  }

  if (events->ip_handler != NULL) {
    (void)esp_event_handler_instance_unregister(
      IP_EVENT, IP_EVENT_STA_GOT_IP, events->ip_handler);
    events->ip_handler = NULL;
  }

  if (events->group != NULL) {
    vEventGroupDelete(events->group);
    events->group = NULL;
  }
}

static bool
MeshReady(bool expect_root,
          const mesh_transport_t* mesh,
          const mesh_diag_events_t* events)
{
  if (mesh == NULL || !MeshTransportIsStarted(mesh)) {
    return false;
  }

  const EventBits_t bits = (events != NULL && events->group != NULL)
                             ? xEventGroupGetBits(events->group)
                             : 0;
  const bool connected = MeshTransportIsConnected(mesh);
  const int layer = esp_mesh_get_layer();
  const bool is_root = esp_mesh_is_root();

  if (expect_root || is_root) {
    if ((bits & DIAG_MESH_EVENT_ROOT_IP) != 0) {
      return true;
    }
    if (events != NULL && events->to_ds_known && events->to_ds) {
      return true;
    }
    return connected;
  }

  if ((bits & DIAG_MESH_EVENT_PARENT_CONNECTED) != 0) {
    return true;
  }
  return (connected && layer > 0);
}

static esp_err_t
WaitForMeshReady(const mesh_transport_t* mesh,
                 mesh_diag_events_t* events,
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
  bool ready = MeshReady(expect_root, mesh, events);

  while (!ready && (xTaskGetTickCount() - start_ticks) < timeout_ticks) {
    if (events != NULL && events->group != NULL) {
      (void)xEventGroupWaitBits(
        events->group,
        DIAG_MESH_EVENT_PARENT_CONNECTED | DIAG_MESH_EVENT_LAYER_CHANGED |
          DIAG_MESH_EVENT_ROOT_IP | DIAG_MESH_EVENT_PARENT_DISCONNECTED |
          DIAG_MESH_EVENT_TODS,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(200));
    } else {
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    ready = MeshReady(expect_root, mesh, events);
  }

  const TickType_t elapsed = xTaskGetTickCount() - start_ticks;
  if (waited_ms_out != NULL) {
    *waited_ms_out = (int)pdTICKS_TO_MS(elapsed);
  }
  if (layer_out != NULL) {
    *layer_out = esp_mesh_get_layer();
  }
  if (ready_out != NULL) {
    *ready_out = ready;
  }
  return ready ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void
CaptureMeshStatus(mesh_status_t* status,
                  const mesh_transport_t* mesh,
                  const mesh_diag_events_t* events)
{
  if (status == NULL) {
    return;
  }
  memset(status, 0, sizeof(*status));
  status->layer = -1;
  status->channel = -1;
  status->last_disconnect_reason =
    (events != NULL) ? events->last_disconnect_reason : -1;
  status->owner_mode = WifiServiceActiveMode();

  if (mesh == NULL) {
    return;
  }
  status->started = MeshTransportIsStarted(mesh);
  status->connected = MeshTransportIsConnected(mesh);
  if (!status->started) {
    return;
  }

  status->is_root = esp_mesh_is_root();
  status->layer = esp_mesh_get_layer();

  mesh_cfg_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  if (esp_mesh_get_config(&cfg) == ESP_OK) {
    memcpy(&status->mesh_id, &cfg.mesh_id, sizeof(status->mesh_id));
    status->mesh_id_known = true;
    status->channel = cfg.channel;
  }

  mesh_addr_t parent;
  memset(&parent, 0, sizeof(parent));
  if (esp_mesh_get_parent_bssid(&parent) == ESP_OK) {
    memcpy(&status->parent, &parent, sizeof(status->parent));
    status->parent_known = true;
  } else if (events != NULL && events->parent_known) {
    memcpy(&status->parent, &events->parent, sizeof(status->parent));
    status->parent_known = true;
  }

  mesh_addr_t root_addr;
  memset(&root_addr, 0, sizeof(root_addr));
  // esp_mesh.h does not expose a direct API to query the current root MAC.
  // - If we are the root, the root address is our own mesh ID.
  // - Otherwise, rely on the cached MESH_EVENT_ROOT_ADDRESS tracking (if
  //   enabled for this diagnostic run).
  bool root_addr_known = false;
  if (status->is_root) {
    if (esp_mesh_get_id(&root_addr) == ESP_OK) {
      memcpy(&status->root_addr, &root_addr, sizeof(status->root_addr));
      status->root_addr_known = true;
      root_addr_known = true;
    }
  }
  if (!root_addr_known && events != NULL && events->root_known) {
    memcpy(&status->root_addr, &events->root_addr, sizeof(status->root_addr));
    status->root_addr_known = true;
  }

  if (events != NULL && events->root_has_ip) {
    status->root_ip = events->root_ip;
    status->root_ip_known = true;
  }

  status->routing_table_size = esp_mesh_get_routing_table_size();
}

static void
PrintRoutingTable(const diag_ctx_t* ctx)
{
  if (ctx == NULL || ctx->verbosity < kDiagVerbosity2) {
    return;
  }

  const int routing_table_size = esp_mesh_get_routing_table_size();
  if (routing_table_size <= 0) {
    return;
  }

  mesh_addr_t routing_table[10];
  int table_entries = 0;
  const esp_err_t result = esp_mesh_get_routing_table(
    routing_table, sizeof(routing_table), &table_entries);
  if (result != ESP_OK || table_entries <= 0) {
    return;
  }

  const int to_show =
    (table_entries < (int)(sizeof(routing_table) / sizeof(routing_table[0])))
      ? table_entries
      : (int)(sizeof(routing_table) / sizeof(routing_table[0]));
  printf("      routing table: %d entries (showing %d)\n",
         routing_table_size,
         to_show);
  for (int index = 0; index < to_show; ++index) {
    char mac[20] = { 0 };
    FormatMac(mac, sizeof(mac), routing_table[index].addr);
    printf("        %2d. %s\n", index + 1, mac);
  }
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

  mesh_diag_events_t events;
  memset(&events, 0, sizeof(events));
  const bool need_events = wait_for_ready || start;
  esp_err_t event_result = ESP_OK;
  if (need_events && runtime_result == ESP_OK && wifi_owner_ok) {
    event_result = InitEventTracking(&events);
    if (event_result != ESP_OK) {
      ESP_LOGW(
        kTag, "event tracking unavailable: %s", esp_err_to_name(event_result));
    }
  }

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
  int wait_layer = mesh_started ? esp_mesh_get_layer() : -1;
  esp_err_t wait_result = ESP_OK;
  if (wait_for_ready) {
    if (!mesh_started) {
      wait_result = ESP_ERR_INVALID_STATE;
    } else if (event_result != ESP_OK) {
      wait_result = ESP_ERR_INVALID_STATE;
    } else {
      wait_result = WaitForMeshReady(mesh_available ? runtime->mesh : NULL,
                                     (event_result == ESP_OK) ? &events : NULL,
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
    " waited_ms=%d layer=%d wifi_mode=%s event_result=%s"
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
    esp_err_to_name(event_result),
    (unsigned)start_before.free_8bit,
    (unsigned)start_after.free_8bit,
    (unsigned)start_after.min_free);
  PrintHeapSnapshot(&ctx, "mesh_start_before", &start_before);
  PrintHeapSnapshot(&ctx, "mesh_start_after", &start_after);

  mesh_status_t status;
  CaptureMeshStatus(&status,
                    mesh_available ? runtime->mesh : NULL,
                    (event_result == ESP_OK) ? &events : NULL);

  char parent_str[20] = { 0 };
  if (status.parent_known) {
    FormatMac(parent_str, sizeof(parent_str), status.parent.addr);
  }
  char status_mesh_id[20] = { 0 };
  if (status.mesh_id_known) {
    FormatMac(status_mesh_id, sizeof(status_mesh_id), status.mesh_id.addr);
  }
  char root_addr_str[20] = { 0 };
  if (status.root_addr_known) {
    FormatMac(root_addr_str, sizeof(root_addr_str), status.root_addr.addr);
  }

  ip4_addr_t root_ip_lwip = { 0 };
  const char* root_ip_str = "<unknown>";
  if (status.root_ip_known) {
    root_ip_lwip.addr = status.root_ip.addr;
    root_ip_str = ip4addr_ntoa(&root_ip_lwip);
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

  CleanupEventTracking((event_result == ESP_OK) ? &events : NULL);

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
