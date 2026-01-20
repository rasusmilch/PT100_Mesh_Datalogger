#ifndef PT100_LOGGER_RUNTIME_STATE_H_
#define PT100_LOGGER_RUNTIME_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_settings.h"
#include "esp_err.h"
#include "fram_i2c.h"
#include "fram_error_log.h"
#include "fram_io.h"
#include "fram_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "heap_monitor.h"
#include "log_record.h"
#include "alerts/alert_manager.h"
#include "i2c_bus.h"
#include "max31865_reader.h"
#include "max7219_display.h"
#include "mesh_transport.h"
#include "mqtt_client_wrap.h"
#include "runtime_health.h"
#include "sd_card_detect.h"
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
    bool cal_due_check_suspended;
    bool cal_overdue;
    bool cal_time_stable;

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
    bool sd_card_present;
    bool sd_safe_to_remove;

    // fram (written by fram_log or storage task when it updates counters)
    uint32_t fram_count;
    uint32_t fram_capacity;
    bool fram_full;
    uint32_t fram_flush_watermark_records;
    bool fram_overrun_active;
    bool fram_io_error_active;
    bool i2c_recovery_active;

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
    bool root_publish_consumer_active;
    uint32_t root_publish_drop_no_consumer;

    // runtime
    bool runtime_running;
    bool stop_requested;
    bool system_running;

    // config (written by settings load/apply)
    uint32_t disp_attn_mask;
    uint32_t disp_attn_pol;

    // drain stats (written by drain helper)
    int32_t last_drain_result;
    uint32_t last_drain_remaining;
    uint32_t last_drain_duration_ms;
    int32_t last_drain_flushed_records;
    int32_t last_drain_flushed_bytes;

    // Heap monitor (bytes)
    uint32_t heap_internal_free_bytes;
    uint32_t heap_internal_largest_free_block_bytes;
    uint32_t heap_internal_min_free_bytes;
    uint32_t heap_internal_min_largest_free_block_bytes;
    uint8_t heap_internal_frag_percent;
    bool heap_internal_warn;
    bool heap_internal_crit;

    uint32_t heap_psram_free_bytes;
    uint32_t heap_psram_largest_free_block_bytes;
    uint32_t heap_psram_min_free_bytes;
    uint32_t heap_psram_min_largest_free_block_bytes;
    uint8_t heap_psram_frag_percent;
    bool heap_psram_warn;
    bool heap_psram_crit;
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

  typedef enum
  {
    RUNTIME_PHASE_DIAGNOSTICS = 0,
    RUNTIME_PHASE_STARTING,
    RUNTIME_PHASE_RUNNING,
    RUNTIME_PHASE_STOPPING,
  } runtime_phase_t;

  typedef struct
  {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t pending_system_code;
    bool pending_is_active;
    int64_t pending_epoch;
    uint32_t pending_uptime_ms;
  } runtime_reboot_alert_latch_t;

  enum
  {
    kLogQueueDepth = 64,
    kStorageTaskStackBytes = 8192, // bytes
    kSdFlushTaskStackBytes = 12288, // bytes
    kNetTxTaskStackBytes = 10240, // bytes
  };

  typedef struct
  {
    log_record_t record;
    int32_t disp_raw_temp_milli_c;
    int32_t disp_cal_temp_milli_c;
  } sensor_sample_msg_t;

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
    fram_error_log_t fram_error_log;
    portMUX_TYPE errlog_latch_lock;
    uint64_t errlog_pending_active_mask;
    uint64_t errlog_pending_resolved_mask;
    sd_logger_t sd_logger;
    sd_card_detect_t sd_card_detect;
    StaticSemaphore_t sd_io_mutex_buf;
    SemaphoreHandle_t sd_io_mutex;
    SemaphoreHandle_t fram_log_mutex;
    max31865_reader_t sensor;
    mesh_transport_t mesh;
    time_sync_t time_sync;
    i2c_bus_t i2c_bus;
    StaticSemaphore_t i2c_mutex_buf;
    SemaphoreHandle_t i2c_mutex;
    TickType_t rtc_resync_last_ticks;
    TickType_t last_rtc_force_before_roll_ticks;
    TickType_t last_rtc_resync_warn_ticks;
    bool time_jump_back_arm_next;
    bool time_jump_back_pending_confirm;
    uint64_t time_jump_back_attempt_record_id;
    int64_t last_time_jump_back_delta_sec;
    TickType_t time_jump_back_last_arm_ticks;

    QueueHandle_t log_queue;
    StaticQueue_t log_queue_struct;
    uint8_t log_queue_storage[kLogQueueDepth * sizeof(sensor_sample_msg_t)];
    QueueHandle_t export_queue;
    QueueHandle_t export_outbox;
    QueueHandle_t broker_outbox;
    uint8_t* batch_buffer;
    size_t batch_buffer_size;

    TickType_t last_flush_ticks;
    TickType_t sd_next_flush_allowed_ticks;
    TickType_t sd_backoff_until_ticks;
    TickType_t last_sd_flush_warn_ticks;
    TickType_t last_sd_flush_wait_warn_ticks;
    uint32_t sd_fail_count;
    uint32_t sd_flush_records_since;
    bool sd_flush_in_progress;
    bool sd_flush_pending;
    volatile bool sd_manual_drain_active;
    volatile TickType_t sd_manual_drain_deadline_ticks;
    volatile uint32_t sd_manual_drain_passes;
    volatile uint32_t storage_marker;
    volatile uint32_t sd_flush_marker;
    // If the FRAM->SD drain was requested at run start but SD was not mounted,
    // keep trying aggressively once the card becomes available.
    bool sd_start_drain_pending;
    bool sd_degraded;
    bool sd_force_unmount_on_append;
    bool sd_verify_readback;
    bool sd_last_io_error_active;
    esp_err_t sd_last_io_err;
    int sd_last_errno;
    bool fram_full;
    bool sd_was_mounted;
    TickType_t last_overrun_log_ticks;
    uint64_t last_overrun_records_total;
    uint64_t last_overrun_logged_total;
    uint64_t fram_overrun_ack_total;
    uint32_t fram_corrupt_detect_count;
    uint32_t fram_corrupt_skip_count;
    uint32_t fram_corrupt_last_offset;
    uint32_t fram_corrupt_last_slot;
    uint32_t fram_corrupt_last_addr;
    uint32_t fram_corrupt_last_magic;
    uint16_t fram_corrupt_last_schema;
    uint16_t fram_corrupt_last_exp_crc;
    uint16_t fram_corrupt_last_act_crc;
    fram_log_validate_result_t fram_corrupt_last_reason;
    uint32_t fram_append_fail_streak;
    uint32_t fram_crc_fail_streak;
    uint32_t fram_log_lock_timeout_count_storage;
    uint32_t fram_log_lock_timeout_count_sdflush;
    uint32_t last_fram_log_lock_timeout_storage_log_ms;
    uint32_t last_fram_log_lock_timeout_sdflush_log_ms;
    bool i2c_recovery_in_progress;

    // Sensor fault logging state (rate-limited).
    bool last_sensor_fault_present;
    uint8_t last_sensor_fault_status;
    TickType_t last_sensor_fault_log_ticks;
    TickType_t last_sensor_spi_log_ticks;
    uint32_t sensor_spi_invalid_streak;
    uint32_t sensor_spi_reinit_count;
    int64_t sensor_spi_reinit_window_start_ms;
    int64_t sensor_spi_last_invalid_ms;
    bool sensor_spi_fault_active;
    bool last_rtd_ema_enabled;
    bool cal_due_check_suspended;
    bool cal_overdue;
    bool cal_time_stable;
    int64_t cal_last_time_valid_utc;

    char node_id_string[32];
    uint8_t local_mac[6];
    uint64_t local_leaf_id;

    TaskHandle_t sensor_task;
    TaskHandle_t storage_task;
    StaticTask_t storage_task_tcb;
    StackType_t storage_task_stack[kStorageTaskStackBytes /
                                   sizeof(StackType_t)];
    TaskHandle_t sd_flush_task;
    StaticTask_t sd_flush_task_tcb;
    StackType_t sd_flush_task_stack[kSdFlushTaskStackBytes /
                                    sizeof(StackType_t)];
    TaskHandle_t export_task;
    TaskHandle_t display_task;
    TaskHandle_t wifi_direct_task;
    TaskHandle_t control_task;
    TaskHandle_t net_tx_task;

    bool initialized;
    bool system_running;
    bool logger_running;
    bool stop_requested;
    bool spi_pause_requested;
    bool spi_pause_acked;
    bool mesh_started;
    bool wifi_direct_started;
    bool wifi_direct_time_synced;
    bool data_streaming_enabled;
    bool log_quiet;
    bool diag_heap_check_enabled;
    runtime_phase_t runtime_phase;
    bool pending_start;
    bool pending_stop;
    bool reboot_alert_pending;
    bool reboot_alert_active_sent;
    int64_t reboot_alert_active_sent_ms;
    int64_t reboot_alert_event_epoch;
    uint32_t reboot_alert_event_uptime_ms;
    alert_system_code_t reboot_alert_code;
    int64_t reboot_alert_next_check_ms;

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
    heap_monitor_t heap_monitor;

    alert_manager_t alert_manager;
    mqtt_client_wrap_t mqtt_client;
    bool mqtt_client_connected;
    bool broker_bridge_requested_without_mqtt;
    bool root_publish_consumer_active;
    uint32_t root_publish_drop_no_consumer;
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
