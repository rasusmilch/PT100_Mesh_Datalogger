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

  typedef void (*mesh_publish_record_rx_callback_t)(const uint8_t src_mac[6],
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
    mesh_publish_record_rx_callback_t publish_record_rx_callback;
    void* publish_record_rx_context;
    
    // NOTE: TimeSyncSetSystemEpoch() mutates the pointed-to state.
    time_sync_t* time_sync; // used for RTC updates on time sync messages
  } mesh_transport_t;

/**
 * @brief Execute MeshTransportIsStarted.
 * @param mesh Parameter mesh.
 * @return Return the function result.
 */
  bool MeshTransportIsStarted(const mesh_transport_t* mesh);
/**
 * @brief Execute MeshTransportMeshLiteIsActive.
 * @return Return the function result.
 */
  bool MeshTransportMeshLiteIsActive(void);

  // Initializes Wi-Fi + Mesh-Lite and starts background activity for mesh
  // transport. Root node connects to the external router and runs SNTP
  // elsewhere (app_main).
/**
 * @brief Execute MeshTransportStart.
 * @param mesh Parameter mesh.
 * @param is_root Parameter is_root.
 * @param allow_children Parameter allow_children.
 * @param router_ssid Parameter router_ssid.
 * @param router_password Parameter router_password.
 * @param record_rx_callback Parameter record_rx_callback.
 * @param record_rx_context Parameter record_rx_context.
 * @param publish_record_rx_callback Parameter publish_record_rx_callback.
 * @param publish_record_rx_context Parameter publish_record_rx_context.
 * @param time_sync Parameter time_sync.
 * @return Return the function result.
 */
  esp_err_t MeshTransportStart(mesh_transport_t* mesh,
                               bool is_root,
                               bool allow_children,
                               const char* router_ssid,
                               const char* router_password,
                               mesh_record_rx_callback_t record_rx_callback,
                               void* record_rx_context,
                               mesh_publish_record_rx_callback_t
                                 publish_record_rx_callback,
                               void* publish_record_rx_context,
                               time_sync_t* time_sync);

/**
 * @brief Execute MeshTransportIsConnected.
 * @param mesh Parameter mesh.
 * @return Return the function result.
 */
  bool MeshTransportIsConnected(const mesh_transport_t* mesh);

/**
 * @brief Execute MeshTransportGetRootAddress.
 * @param mesh Parameter mesh.
 * @param root_out Parameter root_out.
 * @return Return the function result.
 */
  esp_err_t MeshTransportGetRootAddress(const mesh_transport_t* mesh,
                                        pt100_mesh_addr_t* root_out);

  // Leaf nodes: send a log record upstream to the root.
/**
 * @brief Execute MeshTransportSendRecord.
 * @param mesh Parameter mesh.
 * @param record Parameter record.
 * @return Return the function result.
 */
  esp_err_t MeshTransportSendRecord(const mesh_transport_t* mesh,
                                    const log_record_t* record);

  // Leaf nodes: send a publish candidate upstream to the root.
/**
 * @brief Execute MeshTransportSendPublishRecord.
 * @param mesh Parameter mesh.
 * @param src_mac Parameter src_mac.
 * @param record Parameter record.
 * @return Return the function result.
 */
  esp_err_t MeshTransportSendPublishRecord(const mesh_transport_t* mesh,
                                           const uint8_t src_mac[6],
                                           const log_record_t* record);

  // Root nodes: broadcast time to all known nodes.
/**
 * @brief Execute MeshTransportBroadcastTime.
 * @param mesh Parameter mesh.
 * @param epoch_seconds Parameter epoch_seconds.
 * @return Return the function result.
 */
  esp_err_t MeshTransportBroadcastTime(const mesh_transport_t* mesh,
                                       int64_t epoch_seconds);

  // Leaf nodes: request current time from the root (root replies with
  // TIME_SYNC).
/**
 * @brief Execute MeshTransportRequestTime.
 * @param mesh Parameter mesh.
 * @return Return the function result.
 */
  esp_err_t MeshTransportRequestTime(const mesh_transport_t* mesh);

  // Stops mesh transport and underlying Wi-Fi activity without deinitializing
  // esp_wifi.
/**
 * @brief Execute MeshTransportStop.
 * @param mesh Parameter mesh.
 * @return Return the function result.
 */
  esp_err_t MeshTransportStop(mesh_transport_t* mesh);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_MESH_TRANSPORT_H_
