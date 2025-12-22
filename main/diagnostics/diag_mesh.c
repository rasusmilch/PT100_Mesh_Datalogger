#include "diagnostics/diag_mesh.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_mesh.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mesh_transport.h"
#include "net_stack.h"
#include "sdkconfig.h"
#include "wifi_service.h"

static const char* kTag = "diag_mesh";

#define DIAG_MESH_EVENT_PARENT_CONNECTED BIT0
#define DIAG_MESH_EVENT_LAYER_CHANGED BIT1
#define DIAG_MESH_EVENT_ROOT_GOT_IP BIT2
#define DIAG_MESH_EVENT_PARENT_DISCONNECTED BIT3

typedef struct
{
  size_t free_8bit;
  size_t free_total;
  size_t min_free;
} heap_snapshot_t;

typedef struct
{
  EventGroupHandle_t group;
  int last_layer;
  int last_disconnect_reason;
  mesh_addr_t parent;
  bool parent_known;
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
  int channel;
  wifi_service_mode_t owner_mode;
  int last_disconnect_reason;
} mesh_status_t;

static const char*
YesNo(bool value)
{
  return value ? "yes" : "no";
}

static heap_snapshot_t
CaptureHeapSnapshot(const diag_ctx_t* ctx)
{
  heap_snapshot_t snapshot;
  if (ctx != NULL && ctx->verbosity >= kDiagVerbosity2) {
    heap_caps_check_integrity_all(true);
  }
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

  mesh_event_info_t* info = (mesh_event_info_t*)event_data;

  switch (event_id) {
    case MESH_EVENT_PARENT_CONNECTED:
      events->last_layer = (info != NULL) ? (int)info->connected.self_layer
                                          : esp_mesh_get_layer();
      if (info != NULL) {
        memcpy(events->parent.addr, info->connected.parent_bssid, 6);
        events->parent_known = true;
      } else {
        mesh_addr_t parent;
        if (esp_mesh_get_parent_bssid(&parent) == ESP_OK) {
          memcpy(events->parent.addr, parent.addr, 6);
          events->parent_known = true;
        }
      }
      xEventGroupSetBits(events->group, DIAG_MESH_EVENT_PARENT_CONNECTED);
      break;

    case MESH_EVENT_LAYER_CHANGE:
      events->last_layer = (info != NULL) ? (int)info->layer_change.new_layer
                                          : esp_mesh_get_layer();
      xEventGroupSetBits(events->group, DIAG_MESH_EVENT_LAYER_CHANGED);
      break;

    case MESH_EVENT_ROOT_GOT_IP:
      xEventGroupSetBits(events->group, DIAG_MESH_EVENT_ROOT_GOT_IP);
      break;

    case MESH_EVENT_PARENT_DISCONNECTED:
      if (info != NULL) {
        events->last_disconnect_reason = info->disconnected.reason;
      }
      xEventGroupSetBits(events->group, DIAG_MESH_EVENT_PARENT_DISCONNECTED);
      break;

    default:
      break;
  }
}

static esp_err_t
InitEventTracking(mesh_diag_events_t* events,
                  esp_event_handler_instance_t* handler_out)
{
  if (events == NULL || handler_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(events, 0, sizeof(*events));
  events->last_layer = -1;
  events->last_disconnect_reason = -1;
  events->group = xEventGroupCreate();
  if (events->group == NULL) {
    return ESP_ERR_NO_MEM;
  }

  return esp_event_handler_instance_register(MESH_EVENT,
                                             ESP_EVENT_ANY_ID,
                                             &MeshDiagEventHandler,
                                             events,
                                             handler_out);
}

static void
CleanupEventTracking(mesh_diag_events_t* events,
                     esp_event_handler_instance_t handler)
{
  if (handler != NULL) {
    (void)esp_event_handler_instance_unregister(
      MESH_EVENT, ESP_EVENT_ANY_ID, handler);
  }
  if (events != NULL && events->group != NULL) {
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
    if ((bits & DIAG_MESH_EVENT_ROOT_GOT_IP) != 0) {
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
      (void)xEventGroupWaitBits(events->group,
                                DIAG_MESH_EVENT_PARENT_CONNECTED |
                                  DIAG_MESH_EVENT_LAYER_CHANGED |
                                  DIAG_MESH_EVENT_ROOT_GOT_IP |
                                  DIAG_MESH_EVENT_PARENT_DISCONNECTED,
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
  status->last_disconnect_reason = (events != NULL)
                                     ? events->last_disconnect_reason
                                     : -1;
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
  const bool wait_for_ready = full && (start || mesh_started_before);
  const bool perform_stop = stop;
  const int total_steps = 5 + (wait_for_ready ? 1 : 0) + (perform_stop ? 1 : 0);
  int step_index = 1;

  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "Mesh", verbosity);

  esp_err_t runtime_result = ESP_OK;
  const bool runtime_running = RuntimeIsRunning();
  if (!mesh_available) {
    runtime_result = ESP_ERR_INVALID_STATE;
  } else if (runtime_running) {
    runtime_result = ESP_ERR_INVALID_STATE;
  }

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
  DiagReportStep(&ctx,
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

  heap_snapshot_t net_before = CaptureHeapSnapshot(&ctx);
  DiagHeapCheck(&ctx, "pre_net");
  const esp_err_t net_result = NetStackInitOnce();
  heap_snapshot_t net_after = CaptureHeapSnapshot(&ctx);
  DiagHeapCheck(&ctx, "post_net");
  DiagReportStep(&ctx,
                 step_index++,
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

#if CONFIG_APP_NODE_IS_ROOT
  const bool default_root = true;
#else
  const bool default_root = false;
#endif
  const bool start_as_root = force_root || default_root;

  mesh_diag_events_t events;
  memset(&events, 0, sizeof(events));
  esp_event_handler_instance_t event_handler = NULL;
  if (wait_for_ready) {
    const esp_err_t handler_result = InitEventTracking(&events, &event_handler);
    if (handler_result != ESP_OK) {
      ESP_LOGW(kTag,
               "mesh event tracking not available: %s",
               esp_err_to_name(handler_result));
      CleanupEventTracking(&events, NULL);
      memset(&events, 0, sizeof(events));
    }
  }

  heap_snapshot_t start_before = CaptureHeapSnapshot(&ctx);
  heap_snapshot_t start_after = start_before;
  DiagHeapCheck(&ctx, "pre_mesh_start");
  esp_err_t start_result = ESP_OK;
  bool mesh_started = mesh_started_before;
  bool mesh_connected = mesh_connected_before;
  if (start) {
    if (!mesh_available) {
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
      if (start_result == ESP_ERR_INVALID_STATE &&
          MeshTransportIsStarted(runtime->mesh)) {
        start_result = ESP_OK;
      }
    }
    mesh_started = MeshTransportIsStarted(runtime->mesh);
    mesh_connected = MeshTransportIsConnected(runtime->mesh);
    start_after = CaptureHeapSnapshot(&ctx);
  }
  DiagHeapCheck(&ctx, "post_mesh_start");
  const wifi_service_mode_t mode_after_start = WifiServiceActiveMode();
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "mesh start",
                 start ? start_result : ESP_OK,
                 "requested=%s root=%s started=%s connected=%s wifi_mode=%s "
                 "heap8_before=%u heap8_after=%u min_free=%u",
                 YesNo(start),
                 YesNo(start_as_root),
                 YesNo(mesh_started),
                 YesNo(mesh_connected),
                 WifiModeToString(mode_after_start),
                 (unsigned)start_before.free_8bit,
                 (unsigned)start_after.free_8bit,
                 (unsigned)start_after.min_free);
  PrintHeapSnapshot(&ctx, "start_before", &start_before);
  PrintHeapSnapshot(&ctx, "start_after", &start_after);

  if (wait_for_ready) {
    bool ready = mesh_connected;
    int waited_ms = 0;
    int wait_layer = mesh_started ? esp_mesh_get_layer() : -1;
    esp_err_t wait_result = ESP_OK;
    if (!mesh_started) {
      wait_result = ESP_ERR_INVALID_STATE;
    } else {
      wait_result = WaitForMeshReady(runtime->mesh,
                                     (event_handler != NULL) ? &events : NULL,
                                     start_as_root,
                                     timeout_ms,
                                     &ready,
                                     &waited_ms,
                                     &wait_layer);
    }
    DiagReportStep(&ctx,
                   step_index++,
                   total_steps,
                   "mesh ready wait",
                   wait_result,
                   "ready=%s waited_ms=%d layer=%d bits=0x%02x",
                   YesNo(ready),
                   waited_ms,
                   wait_layer,
                   (unsigned)((event_handler != NULL && events.group != NULL)
                                ? xEventGroupGetBits(events.group)
                                : 0U));
  }

  mesh_status_t status;
  CaptureMeshStatus(&status,
                    mesh_available ? runtime->mesh : NULL,
                    (event_handler != NULL) ? &events : NULL);
  char parent_str[20] = { 0 };
  if (status.parent_known) {
    FormatMac(parent_str, sizeof(parent_str), status.parent.addr);
  }
  char mesh_id_str[20] = { 0 };
  if (status.mesh_id_known) {
    FormatMac(mesh_id_str, sizeof(mesh_id_str), status.mesh_id.addr);
  }
  DiagReportStep(
    &ctx,
    step_index++,
    total_steps,
    "mesh status",
    ESP_OK,
    "started=%s connected=%s root=%s layer=%d parent=%s mesh_id=%s ch=%d "
    "owner=%s last_disc_reason=%d",
    YesNo(status.started),
    YesNo(status.connected),
    YesNo(status.is_root),
    status.layer,
    status.parent_known ? parent_str : "<none>",
    status.mesh_id_known ? mesh_id_str : "<unknown>",
    status.channel,
    WifiModeToString(status.owner_mode),
    status.last_disconnect_reason);

  if (perform_stop) {
    heap_snapshot_t stop_before = CaptureHeapSnapshot(&ctx);
    heap_snapshot_t stop_after = stop_before;
    DiagHeapCheck(&ctx, "pre_mesh_stop");
    esp_err_t stop_result = ESP_OK;
    if (mesh_available && MeshTransportIsStarted(runtime->mesh)) {
      stop_result = MeshTransportStop(runtime->mesh);
    }
    const wifi_service_mode_t mode_after_stop = WifiServiceActiveMode();
    if (mode_after_stop == WIFI_SERVICE_MODE_MESH) {
      const esp_err_t release_result = WifiServiceRelease();
      if (stop_result == ESP_OK && release_result != ESP_OK) {
        stop_result = release_result;
      }
    }
    stop_after = CaptureHeapSnapshot(&ctx);
    DiagHeapCheck(&ctx, "post_mesh_stop");
    DiagReportStep(&ctx,
                   step_index++,
                   total_steps,
                   "mesh stop",
                   stop_result,
                   "requested=%s started_after=%s wifi_mode=%s heap8_before=%u "
                   "heap8_after=%u min_free=%u",
                   YesNo(perform_stop),
                   YesNo(MeshTransportIsStarted(mesh_available ? runtime->mesh
                                                               : NULL)),
                   WifiModeToString(mode_after_stop),
                   (unsigned)stop_before.free_8bit,
                   (unsigned)stop_after.free_8bit,
                   (unsigned)stop_after.min_free);
    PrintHeapSnapshot(&ctx, "stop_before", &stop_before);
    PrintHeapSnapshot(&ctx, "stop_after", &stop_after);
  }

  CleanupEventTracking((event_handler != NULL) ? &events : NULL, event_handler);

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
