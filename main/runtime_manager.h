#ifndef PT100_LOGGER_RUNTIME_MANAGER_H_
#define PT100_LOGGER_RUNTIME_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_settings.h"
#include "esp_err.h"
#include "fram_i2c.h"
#include "fram_io.h"
#include "fram_log.h"
#include "i2c_bus.h"
#include "max31865_reader.h"
#include "mesh_transport.h"
#include "alerts/alert_manager.h"
#include "runtime_state.h"
#include "sd_logger.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    app_settings_t* settings;
    fram_i2c_t* fram_i2c;
    fram_io_t* fram_io;
    fram_log_t* fram_log;
    sd_logger_t* sd_logger;
    max31865_reader_t* sensor;
    mesh_transport_t* mesh;
    time_sync_t* time_sync;
    i2c_bus_t* i2c_bus;
    const char* node_id_string;
    alert_manager_t* alert_manager;
    esp_err_t (*flush_callback)(void* context);
    void* flush_context;
    bool* fram_full;
    uint32_t* export_dropped_count;
    uint32_t* export_write_fail_count;
    QueueHandle_t* export_outbox;
    QueueHandle_t* broker_outbox;
    uint32_t* export_drop_count;
    uint32_t* export_send_fail_count;
    uint32_t* broker_drop_count;
    uint32_t* broker_send_fail_count;
    bool* mqtt_client_connected;
  } app_runtime_t;

  typedef struct
  {
    int32_t flushed_records;
    int32_t remaining_records;
    int32_t flushed_bytes;
    int32_t duration_ms;
    esp_err_t result;
  } sd_drain_stats_t;

  esp_err_t RuntimeManagerInit(void);

  const app_runtime_t* RuntimeGetRuntime(void);

  const runtime_cached_status_t* RuntimeGetCachedStatus(void);

  esp_err_t RuntimeStart(void);

  esp_err_t RuntimeStop(void);

  bool RuntimeIsRunning(void);

  esp_err_t EnterRunMode(void);

  esp_err_t EnterDiagMode(void);

  typedef esp_err_t (*runtime_sd_op_fn_t)(app_runtime_t* runtime, void* ctx);

  esp_err_t RuntimeWithTemporarySdMount(runtime_sd_op_fn_t op, void* ctx);

  void RuntimeRequestRunStart(void);

  void RuntimeRequestRunStop(void);

  esp_err_t RuntimeSdUnmountNow(void);

  void RuntimeEnableDataStreaming(bool enabled);

  bool RuntimeIsDataStreamingEnabled(void);

  void RuntimeSetLogPolicyRun(void);

  void RuntimeSetLogPolicyDiag(void);

  void RuntimeSetSdAppendFailureOnce(bool enabled);

  bool RuntimeSdIsDegraded(void);

  uint32_t RuntimeSdFailCount(void);

  uint32_t RuntimeSdBackoffUntilTicks(void);

  bool RuntimeAcknowledgeDisplayAttention(display_attention_item_t item);

// Updates the in-memory/cached display attention policy immediately so the
// display task reflects changes without requiring a reboot.
void RuntimeSetDisplayAttentionPolicy(uint32_t policy);

  esp_err_t RuntimeShowDisplayTestPattern(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUNTIME_MANAGER_H_
