#ifndef PT100_LOGGER_RUNTIME_HEALTH_H_
#define PT100_LOGGER_RUNTIME_HEALTH_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    // time
    bool time_valid;
    int32_t utc_offset_sec;
    bool dst_in_effect;

    // mesh
    bool mesh_connected;
    int32_t mesh_level;
    int32_t mesh_rssi;

    // storage / sd
    bool sd_mounted;
    bool sd_degraded;
    uint32_t sd_fail_count;
    uint32_t sd_backoff_remaining_ms;
    bool sd_io_error_active;

    // fram
    uint32_t fram_count;
    uint32_t fram_capacity;
    bool fram_full;
    uint32_t fram_flush_watermark_records;
    bool fram_overrun_active;

    // sensor
    bool sensor_fault_present;

    // export
    uint32_t export_dropped_count;
    uint32_t export_write_fail_count;
    uint32_t export_drop_count;
    uint32_t export_send_fail_count;
    uint32_t broker_drop_count;
    uint32_t broker_send_fail_count;
    bool mqtt_connected;

    // attention config (what to show)
    uint32_t disp_attn_mask;
    uint32_t disp_attn_pol;

    // drain stats
    int32_t last_drain_result;
    uint32_t last_drain_remaining;
    uint32_t last_drain_duration_ms;
    int32_t last_drain_flushed_records;
    int32_t last_drain_flushed_bytes;
  } runtime_health_snapshot_t;

  typedef struct
  {
    volatile uint32_t version;
    runtime_health_snapshot_t snapshot;
  } runtime_health_cache_t;

  void RuntimeHealthInit(runtime_health_cache_t* cache);

  void RuntimeHealthPublish(runtime_health_cache_t* cache,
                            const runtime_health_snapshot_t* new_snapshot);

  void RuntimeHealthRead(const runtime_health_cache_t* cache,
                         runtime_health_snapshot_t* out_snapshot);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUNTIME_HEALTH_H_
