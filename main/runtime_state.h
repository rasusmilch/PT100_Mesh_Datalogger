#ifndef PT100_LOGGER_RUNTIME_STATE_H_
#define PT100_LOGGER_RUNTIME_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_settings.h"
#include "esp_err.h"
#include "fram_i2c.h"
#include "fram_io.h"
#include "fram_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "log_record.h"
#include "alerts/alert_manager.h"
#include "i2c_bus.h"
#include "max31865_reader.h"
#include "max7219_display.h"
#include "mesh_transport.h"
#include "mqtt_client_wrap.h"
#include "runtime_health.h"
#include "sd_logger.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    // time (written by time_sync module)
    bool time_valid;
    int32_t utc_offset_sec;
    bool dst_in_effect;

    // mesh (written by mesh task/module)
    bool mesh_connected;
    int32_t mesh_level;
    int32_t mesh_rssi;

    // sd (written by sd_logger/storage task)
    bool sd_mounted;
    bool sd_degraded;
    uint32_t sd_fail_count;
    uint32_t sd_backoff_remaining_ms;
    bool sd_io_error_active;

    // fram (written by fram_log or storage task when it updates counters)
    uint32_t fram_count;
    uint32_t fram_capacity;
    bool fram_full;
    uint32_t fram_flush_watermark_records;
    bool fram_overrun_active;

    // sensor
    bool sensor_fault_present;

    // export counters (written by export/storage path)
    uint32_t export_dropped_count;
    uint32_t export_write_fail_count;
    uint32_t export_drop_count;
    uint32_t export_send_fail_count;
    uint32_t broker_drop_count;
    uint32_t broker_send_fail_count;
    bool mqtt_connected;

    // runtime
    bool runtime_running;
    bool stop_requested;

    // config (written by settings load/apply)
    uint32_t disp_attn_mask;
    uint32_t disp_attn_pol;

    // drain stats (written by drain helper)
    int32_t last_drain_result;
    uint32_t last_drain_remaining;
    uint32_t last_drain_duration_ms;
    int32_t last_drain_flushed_records;
    int32_t last_drain_flushed_bytes;
  } runtime_cached_status_t;

  typedef struct
  {
    TickType_t last_publish_ticks;
    uint32_t publish_period_ms;
    volatile bool dirty;
  } runtime_health_publisher_state_t;

  typedef struct
  {
    uint8_t src_mac[6];
    log_record_t record;
  } export_record_item_t;

  typedef struct runtime_state_t
  {
    app_settings_t settings;
    app_net_mode_t net_mode_active;
    app_node_role_t node_role_active;
    bool mqtt_enabled_active;
    char mqtt_broker_uri_active[128];
    char mqtt_topic_prefix_active[64];
    uint8_t mqtt_qos_active;
    bool mqtt_retain_active;
    mqtt_bridge_mode_t mqtt_bridge_mode_active;
    fram_i2c_t fram_i2c;
    fram_io_t fram_io;
    fram_log_t fram_log;
    sd_logger_t sd_logger;
    max31865_reader_t sensor;
    mesh_transport_t mesh;
    time_sync_t time_sync;
    i2c_bus_t i2c_bus;

    QueueHandle_t log_queue;
    QueueHandle_t export_queue;
    QueueHandle_t export_outbox;
    QueueHandle_t broker_outbox;
    uint8_t* batch_buffer;
    size_t batch_buffer_size;

    TickType_t last_flush_ticks;
    TickType_t sd_next_flush_allowed_ticks;
    TickType_t sd_backoff_until_ticks;
    TickType_t last_sd_flush_warn_ticks;
    uint32_t sd_fail_count;
    uint32_t sd_flush_records_since;
    bool sd_flush_pending;
    // If the FRAM->SD drain was requested at run start but SD was not mounted,
    // keep trying aggressively once the card becomes available.
    bool sd_start_drain_pending;
    bool sd_degraded;
    bool sd_force_unmount_on_append;
    bool sd_last_io_error_active;
    esp_err_t sd_last_io_err;
    int sd_last_errno;
    bool fram_full;
    bool sd_was_mounted;
    TickType_t last_overrun_log_ticks;
    uint64_t last_overrun_records_total;
    uint64_t last_overrun_logged_total;
    uint64_t fram_overrun_ack_total;

    // Sensor fault logging state (rate-limited).
    bool last_sensor_fault_present;
    uint8_t last_sensor_fault_status;
    TickType_t last_sensor_fault_log_ticks;

    char node_id_string[32];
    uint8_t local_mac[6];

    TaskHandle_t sensor_task;
    TaskHandle_t storage_task;
    TaskHandle_t export_task;
    TaskHandle_t export_network_task;
    TaskHandle_t time_sync_task;
    TaskHandle_t topology_task;
    TaskHandle_t display_task;
    TaskHandle_t health_publisher_task;
    TaskHandle_t wifi_direct_task;
    TaskHandle_t bridge_task;
    TaskHandle_t broker_task;
    TaskHandle_t control_task;
    TaskHandle_t alert_monitor_task;
    TaskHandle_t alert_sender_task;

    alert_task_context_t alert_monitor_context;
    alert_task_context_t alert_sender_context;

    bool initialized;
    bool is_running;
    bool stop_requested;
    bool mesh_started;
    bool wifi_direct_started;
    bool wifi_direct_time_synced;
    bool data_streaming_enabled;
    bool log_quiet;

    bool request_run_start;
    bool request_run_stop;
    portMUX_TYPE request_lock;

    uint32_t export_dropped_count;
    uint32_t export_write_fail_count;
    uint32_t export_drop_count;
    uint32_t export_send_fail_count;
    uint32_t broker_drop_count;
    uint32_t broker_send_fail_count;
    bool csv_header_emitted;
    bool root_bridge_header_emitted;

    max7219_display_t display;
    bool display_initialized;
    bool display_test_active;
    TickType_t display_test_start_ticks;
    TickType_t display_test_until_ticks;
    int32_t last_temp_milli_c;
    bool last_temp_valid;
    uint32_t last_flags;
    TickType_t last_update_ticks;
    portMUX_TYPE last_temp_lock;

    runtime_cached_status_t cached_status;
    runtime_health_cache_t health_cache;
    runtime_health_publisher_state_t health_publisher;

    alert_manager_t alert_manager;
    mqtt_client_wrap_t mqtt_client;
    bool mqtt_client_connected;
    bool broker_bridge_requested_without_mqtt;
    uint32_t last_export_drop_log_ms;
    uint32_t last_export_fail_log_ms;
    uint32_t last_broker_drop_log_ms;
    uint32_t last_broker_fail_log_ms;
    uint32_t last_broker_bridge_disabled_log_ms;
    uint32_t last_mqtt_fail_log_ms;
  } runtime_state_t;

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUNTIME_STATE_H_
