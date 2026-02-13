#include "runtime_health_publisher.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_monitor.h"
#include "runtime_health.h"
#include "sdkconfig.h"

/**
 * @brief Execute RuntimeHealthPublisherInit.
 * @param state Parameter state.
 */
void
RuntimeHealthPublisherInit(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }
  state->health_publisher.publish_period_ms = 200;
  state->health_publisher.last_publish_ticks = 0;
  state->health_publisher.dirty = true;
}

/**
 * @brief Execute RuntimeHealthPublisherTick.
 * @param state Parameter state.
 */
void
RuntimeHealthPublisherTick(runtime_state_t* state)
{
  if (state == NULL) {
    return;
  }

  const TickType_t now_ticks = xTaskGetTickCount();
#if CONFIG_APP_HEAP_MONITOR_ENABLE
  HeapMonitorMaybeSample(state, now_ticks);
#endif
  const uint32_t elapsed_ms =
    (state->health_publisher.last_publish_ticks == 0)
      ? state->health_publisher.publish_period_ms
      : (uint32_t)pdTICKS_TO_MS(now_ticks -
                                state->health_publisher.last_publish_ticks);
  if (!state->health_publisher.dirty &&
      elapsed_ms < state->health_publisher.publish_period_ms) {
    return;
  }

  runtime_health_snapshot_t snapshot = { 0 };
  snapshot.time_valid = state->cached_status.time_valid;
  snapshot.utc_offset_sec = state->cached_status.utc_offset_sec;
  snapshot.dst_in_effect = state->cached_status.dst_in_effect;
  snapshot.ntp_fail_alert_active = state->cached_status.ntp_fail_alert_active;
  snapshot.mesh_connected = state->cached_status.mesh_connected;
  snapshot.mesh_level = state->cached_status.mesh_level;
  snapshot.mesh_rssi = state->cached_status.mesh_rssi;
  snapshot.sd_mounted = state->cached_status.sd_mounted;
  snapshot.sd_degraded = state->cached_status.sd_degraded;
  snapshot.sd_fail_count = state->cached_status.sd_fail_count;
  snapshot.sd_backoff_remaining_ms =
    state->cached_status.sd_backoff_remaining_ms;
  snapshot.sd_io_error_active = state->cached_status.sd_io_error_active;
  snapshot.sd_card_present = state->cached_status.sd_card_present;
  snapshot.sd_safe_to_remove = state->cached_status.sd_safe_to_remove;
  snapshot.sd_space_reclaim_active =
    state->cached_status.sd_space_reclaim_active;
  snapshot.sd_out_of_space_active = state->cached_status.sd_out_of_space_active;
  snapshot.fram_count = state->cached_status.fram_count;
  snapshot.fram_capacity = state->cached_status.fram_capacity;
  snapshot.fram_full = state->cached_status.fram_full;
  snapshot.fram_flush_watermark_records =
    state->cached_status.fram_flush_watermark_records;
  snapshot.fram_overrun_active = state->cached_status.fram_overrun_active;
  snapshot.sensor_fault_present = state->cached_status.sensor_fault_present;
  snapshot.export_dropped_count = state->cached_status.export_dropped_count;
  snapshot.export_write_fail_count =
    state->cached_status.export_write_fail_count;
  snapshot.export_drop_count = state->cached_status.export_drop_count;
  snapshot.export_send_fail_count = state->cached_status.export_send_fail_count;
  snapshot.broker_drop_count = state->cached_status.broker_drop_count;
  snapshot.broker_send_fail_count = state->cached_status.broker_send_fail_count;
  snapshot.mqtt_connected = state->cached_status.mqtt_connected;
  snapshot.disp_attn_mask = state->cached_status.disp_attn_mask;
  snapshot.disp_attn_pol = state->cached_status.disp_attn_pol;
  snapshot.last_drain_result = state->cached_status.last_drain_result;
  snapshot.last_drain_remaining = state->cached_status.last_drain_remaining;
  snapshot.last_drain_duration_ms = state->cached_status.last_drain_duration_ms;
  snapshot.last_drain_flushed_records =
    state->cached_status.last_drain_flushed_records;
  snapshot.last_drain_flushed_bytes =
    state->cached_status.last_drain_flushed_bytes;
  snapshot.heap_internal_free_bytes =
    state->cached_status.heap_internal_free_bytes;
  snapshot.heap_internal_largest_free_block_bytes =
    state->cached_status.heap_internal_largest_free_block_bytes;
  snapshot.heap_internal_min_free_bytes =
    state->cached_status.heap_internal_min_free_bytes;
  snapshot.heap_internal_min_largest_free_block_bytes =
    state->cached_status.heap_internal_min_largest_free_block_bytes;
  snapshot.heap_internal_frag_percent =
    state->cached_status.heap_internal_frag_percent;
  snapshot.heap_internal_warn = state->cached_status.heap_internal_warn;
  snapshot.heap_internal_crit = state->cached_status.heap_internal_crit;
  snapshot.heap_psram_free_bytes = state->cached_status.heap_psram_free_bytes;
  snapshot.heap_psram_largest_free_block_bytes =
    state->cached_status.heap_psram_largest_free_block_bytes;
  snapshot.heap_psram_min_free_bytes =
    state->cached_status.heap_psram_min_free_bytes;
  snapshot.heap_psram_min_largest_free_block_bytes =
    state->cached_status.heap_psram_min_largest_free_block_bytes;
  snapshot.heap_psram_frag_percent =
    state->cached_status.heap_psram_frag_percent;
  snapshot.heap_psram_warn = state->cached_status.heap_psram_warn;
  snapshot.heap_psram_crit = state->cached_status.heap_psram_crit;

  RuntimeHealthPublish(&state->health_cache, &snapshot);
  state->health_publisher.last_publish_ticks = now_ticks;
  state->health_publisher.dirty = false;
}
