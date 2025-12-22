#include "diagnostics/diag_mesh.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_err.h"
#include "crc16.h"
#include "log_record.h"
#include "mesh_transport.h"
#include "sdkconfig.h"
#include "wifi_service.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_mesh.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char*
YesNo(bool value)
{
  return value ? "yes" : "no";
}

static void
FormatMac(char* out, size_t out_size, const uint8_t mac[6])
{
  if (out == NULL || out_size == 0 || mac == NULL) {
    return;
  }
  snprintf(out, out_size, MACSTR, MAC2STR(mac));
}

static void
DiagPrint(const diag_ctx_t* ctx, const char* fmt, ...)
{
  if (ctx == NULL || ctx->verbosity < kDiagVerbosity1 || fmt == NULL) {
    return;
  }
  va_list args;
  va_start(args, fmt);
  printf("      ");
  vprintf(fmt, args);
  printf("\n");
  va_end(args);
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

int
RunDiagMesh(const app_runtime_t* runtime,
            bool full,
            bool start,
            bool stop,
            diag_verbosity_t verbosity)
{
  const bool mesh_available = (runtime != NULL && runtime->mesh != NULL);
  const bool mesh_started_before =
    MeshTransportIsStarted(mesh_available ? runtime->mesh : NULL);
  const bool perform_stop =
    stop || (start && mesh_available && !mesh_started_before);
  const int total_steps = 4 + (full ? 2 : 0) + (perform_stop ? 1 : 0);
  int step_index = 1;

  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "Mesh", verbosity);

  const bool runtime_running = RuntimeIsRunning();
  const esp_err_t runtime_result =
    (!mesh_available) ? ESP_ERR_INVALID_STATE
                      : (runtime_running ? ESP_ERR_INVALID_STATE : ESP_OK);
  DiagReportStep(&ctx,
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
  const char* wifi_owner_details =
    wifi_owner_ok
      ? "mode=%s"
      : "wifi_service already active in mode=%s; run diag wifi ... with "
        "keep_connected=0, or stop wifi/mesh, or reboot.";
  DiagReportStep(
    &ctx,
    step_index++,
    total_steps,
    "wifi owner check",
    wifi_owner_ok ? ESP_OK : ESP_ERR_INVALID_STATE,
    wifi_owner_details,
    WifiModeToString(active_mode));
  if (!wifi_owner_ok) {
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  bool mesh_started = mesh_started_before;
  bool mesh_connected =
    MeshTransportIsConnected(mesh_available ? runtime->mesh : NULL);
  wifi_service_mode_t mode_after_start = active_mode;
  esp_err_t start_result = ESP_OK;

  if (start) {
    if (!mesh_available) {
      start_result = ESP_ERR_INVALID_STATE;
    } else if (mesh_started_before) {
      start_result = ESP_OK;
    } else {
#if CONFIG_APP_NODE_IS_ROOT
      const bool is_root = true;
#else
      const bool is_root = false;
#endif
      start_result = MeshTransportStart(runtime->mesh,
                                        is_root,
                                        CONFIG_APP_WIFI_ROUTER_SSID,
                                        CONFIG_APP_WIFI_ROUTER_PASSWORD,
                                        NULL,
                                        NULL,
                                        runtime->time_sync);
      mesh_started = MeshTransportIsStarted(runtime->mesh);
      mesh_connected = MeshTransportIsConnected(runtime->mesh);
      if (start_result == ESP_ERR_INVALID_STATE && mesh_started) {
        start_result = ESP_OK;
      }
    }
  }
  mesh_started = MeshTransportIsStarted(mesh_available ? runtime->mesh : NULL);
  mesh_connected =
    MeshTransportIsConnected(mesh_available ? runtime->mesh : NULL);
  if (start && start_result == ESP_OK && !mesh_started) {
    start_result = ESP_FAIL;
  }
  mode_after_start = WifiServiceActiveMode();

  const size_t heap_free = esp_get_free_heap_size();
  const size_t heap_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "start mesh",
                 start ? start_result : ESP_OK,
                 "requested=%s started=%s connected=%s wifi_service_mode=%s",
                 YesNo(start),
                 YesNo(mesh_started),
                 YesNo(mesh_connected),
                 WifiModeToString(mode_after_start));
  DiagPrint(&ctx,
            "heap_free=%u min_free_8bit=%u",
            (unsigned)heap_free,
            (unsigned)heap_min_free);

  mesh_addr_t root_addr;
  memset(&root_addr, 0, sizeof(root_addr));
  mesh_addr_t parent_addr;
  memset(&parent_addr, 0, sizeof(parent_addr));
  int routing_table_size = 0;
  int routing_entries = 0;
  esp_err_t status_result = ESP_OK;
  int mesh_layer = -1;

  if (mesh_started) {
    mesh_layer = esp_mesh_get_layer();
    const esp_err_t parent_result = esp_mesh_get_parent_bssid(&parent_addr);
    if (parent_result != ESP_OK && status_result == ESP_OK) {
      status_result = parent_result;
    }
    const esp_err_t root_result =
      MeshTransportGetRootAddress(runtime->mesh, &root_addr);
    if (root_result != ESP_OK && status_result == ESP_OK &&
        root_result != ESP_ERR_INVALID_STATE) {
      status_result = root_result;
    }
    routing_table_size = esp_mesh_get_routing_table_size();
    if (routing_table_size < 0) {
      routing_table_size = 0;
    }
    const int routing_cap = (routing_table_size < 20) ? routing_table_size : 20;
    if (routing_cap > 0 && ctx.verbosity >= kDiagVerbosity1) {
      mesh_addr_t routing_table[20];
      const esp_err_t table_result = esp_mesh_get_routing_table(
        routing_table, routing_cap * sizeof(mesh_addr_t), &routing_entries);
      if (table_result != ESP_OK && status_result == ESP_OK) {
        status_result = table_result;
      }
      if (table_result == ESP_OK && routing_entries > 0 &&
          ctx.verbosity >= kDiagVerbosity1) {
        const int to_show = (routing_entries < 10) ? routing_entries : 10;
        for (int index = 0; index < to_show; ++index) {
          char mac_str[20] = { 0 };
          FormatMac(mac_str,
                    sizeof(mac_str),
                    routing_table[index].addr);
          DiagPrint(&ctx, "route[%d]=%s", index, mac_str);
        }
      }
    }
  }

  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "status",
                 status_result,
                 "started=%s connected=%s layer=%d routes=%d",
                 YesNo(mesh_started),
                 YesNo(mesh_connected),
                 mesh_layer,
                 routing_table_size);

  if (mesh_started && ctx.verbosity >= kDiagVerbosity1) {
    char root_str[20] = { 0 };
    FormatMac(root_str, sizeof(root_str), root_addr.addr);
    DiagPrint(&ctx, "root=%s", root_str);
    char parent_str[20] = { 0 };
    FormatMac(parent_str, sizeof(parent_str), parent_addr.addr);
    DiagPrint(&ctx, "parent=%s", parent_str);
  }

  bool connected_after_wait = mesh_connected;
  int wait_ms = 0;
  int wait_layer = mesh_layer;
  if (full) {
    if (!mesh_started) {
      DiagReportStep(&ctx,
                     step_index++,
                     total_steps,
                     "connect wait",
                     ESP_OK,
                     "skipped (mesh not started)");
    } else {
      const TickType_t wait_start = xTaskGetTickCount();
      const TickType_t wait_timeout = pdMS_TO_TICKS(10000);
      while ((xTaskGetTickCount() - wait_start) < wait_timeout) {
        connected_after_wait = MeshTransportIsConnected(runtime->mesh);
        if (connected_after_wait) {
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
      }
      wait_ms = (int)pdTICKS_TO_MS(xTaskGetTickCount() - wait_start);
      wait_layer = esp_mesh_get_layer();
      DiagReportStep(&ctx,
                     step_index++,
                     total_steps,
                     "connect wait",
                     ESP_OK,
                     "connected=%s after %dms layer=%d",
                     YesNo(connected_after_wait),
                     wait_ms,
                     wait_layer);
    }
  }

  if (full) {
    esp_err_t send_result = ESP_OK;
    if (!mesh_started) {
      send_result = ESP_OK;
      DiagReportStep(&ctx,
                     step_index++,
                     total_steps,
                     "payload send",
                     send_result,
                     "skipped (mesh not started)");
    } else if (!connected_after_wait) {
      send_result = ESP_OK;
      DiagReportStep(&ctx,
                     step_index++,
                     total_steps,
                     "payload send",
                     send_result,
                     "skipped (not connected)");
    } else {
      log_record_t record;
      memset(&record, 0, sizeof(record));
      record.magic = LOG_RECORD_MAGIC;
      record.sequence = 0xD146;
      record.timestamp_epoch_sec = (int64_t)time(NULL);
      record.timestamp_millis = 123;
      record.raw_temp_milli_c = 12345;
      record.temp_milli_c = 12345;
      record.resistance_milli_ohm = 98765;
      record.flags = LOG_RECORD_FLAG_TIME_VALID | LOG_RECORD_FLAG_MESH_CONNECTED;
      record.crc16_ccitt = 0;
      record.crc16_ccitt = Crc16CcittFalse(
        &record, sizeof(record) - sizeof(record.crc16_ccitt));

      send_result = MeshTransportSendRecord(runtime->mesh, &record);
      DiagReportStep(&ctx,
                     step_index++,
                     total_steps,
                     "payload send",
                     send_result,
                     "connected=%s crc=0x%04x temp=12.345C",
                     YesNo(connected_after_wait),
                     (unsigned)record.crc16_ccitt);
      DiagPrint(&ctx, "layer=%d", wait_layer);
      if (send_result == ESP_OK && !runtime->mesh->is_root) {
        const esp_err_t time_result =
          MeshTransportRequestTime(runtime->mesh);
        DiagPrint(&ctx, "time_request=%s", esp_err_to_name(time_result));
      }
    }
  }

  if (perform_stop) {
    esp_err_t stop_result = ESP_OK;
    bool started_before_stop =
      MeshTransportIsStarted(mesh_available ? runtime->mesh : NULL);
    if (mesh_available && started_before_stop) {
      stop_result = MeshTransportStop(runtime->mesh);
    }
    const bool started_after_stop =
      MeshTransportIsStarted(mesh_available ? runtime->mesh : NULL);
    if ((stop_result == ESP_ERR_MESH_NOT_INIT ||
         stop_result == ESP_ERR_MESH_NOT_START ||
         stop_result == ESP_ERR_INVALID_STATE) &&
        !started_after_stop) {
      stop_result = ESP_OK;
    }
    const wifi_service_mode_t mode_after_stop = WifiServiceActiveMode();
    DiagReportStep(&ctx,
                   step_index++,
                   total_steps,
                   "stop mesh",
                   stop_result,
                   "requested=%s started_after=%s wifi_service_mode=%s",
                   YesNo(perform_stop),
                   YesNo(started_after_stop),
                   WifiModeToString(mode_after_stop));
  }

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
