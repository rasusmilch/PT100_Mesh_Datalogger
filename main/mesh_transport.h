#ifndef PT100_LOGGER_MESH_TRANSPORT_H_
#define PT100_LOGGER_MESH_TRANSPORT_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "log_record.h"
#include "mesh_addr.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef void (*mesh_record_rx_callback_t)(const pt100_mesh_addr_t* from,
                                            const log_record_t* record,
                                            void* context);

  typedef struct
  {
    // NOTE: These flags are read/written from multiple tasks (event handler,
    // RX task, console/diagnostics).
    bool is_root;
    bool is_started;
    bool is_connected;
    bool mesh_lite_started;
    int last_level; // cached esp_mesh_lite_get_level()
    pt100_mesh_addr_t root_address;
    mesh_record_rx_callback_t record_rx_callback;
    void* record_rx_context;
    const time_sync_t* time_sync; // used for RTC updates on time sync messages
  } mesh_transport_t;

  bool MeshTransportIsStarted(const mesh_transport_t* mesh);
  bool MeshTransportMeshLiteIsActive(void);

  // Initializes Wi-Fi + Mesh-Lite and starts background activity for mesh
  // transport. Root node connects to the external router and runs SNTP
  // elsewhere (app_main).
  esp_err_t MeshTransportStart(mesh_transport_t* mesh,
                               bool is_root,
                               bool allow_children,
                               const char* router_ssid,
                               const char* router_password,
                               mesh_record_rx_callback_t record_rx_callback,
                               void* record_rx_context,
                               const time_sync_t* time_sync);

  bool MeshTransportIsConnected(const mesh_transport_t* mesh);

  esp_err_t MeshTransportGetRootAddress(const mesh_transport_t* mesh,
                                        pt100_mesh_addr_t* root_out);

  // Leaf nodes: send a log record upstream to the root.
  esp_err_t MeshTransportSendRecord(const mesh_transport_t* mesh,
                                    const log_record_t* record);

  // Root nodes: broadcast time to all known nodes.
  esp_err_t MeshTransportBroadcastTime(const mesh_transport_t* mesh,
                                       int64_t epoch_seconds);

  // Leaf nodes: request current time from the root (root replies with
  // TIME_SYNC).
  esp_err_t MeshTransportRequestTime(const mesh_transport_t* mesh);

  // Stops mesh transport and underlying Wi-Fi activity without deinitializing
  // esp_wifi.
  esp_err_t MeshTransportStop(mesh_transport_t* mesh);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_MESH_TRANSPORT_H_
