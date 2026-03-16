#ifndef PT100_LOGGER_RUNTIME_STATE_H_
#define PT100_LOGGER_RUNTIME_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#include "alerts/alert_manager.h"
#include "app_settings.h"
#include "esp_err.h"
#include "fram_error_log.h"
#include "fram_i2c.h"
#include "fram_io.h"
#include "fram_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "heap_monitor.h"
#include "i2c_bus.h"
#include "log_record.h"
#include "max31865_reader.h"
#include "max7219_display.h"
#include "mesh_transport.h"
#include "mqtt_client_wrap.h"
#include "runtime_health.h"
#include "sd_card_detect.h"
#include "sd_logger.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define RUNTIME_ERROR_RING_SIZE 8
#define RUNTIME_RTC_ERRLOG_RING_SIZE 8
#define DEFERRED_LOG_CAPACITY 20
#define RUNTIME_SD_FLUSH_SESSION_LABEL_LEN 96

  typedef struct
  {
    log_record_t records[DEFERRED_LOG_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint32_t drops;
    portMUX_TYPE lock;
  } deferred_log_t;

  typedef struct
  {
    uint32_t uptime_ms;
    const char* module;
    esp_err_t err;
    const char* phase;
  } runtime_error_entry_t;

  typedef struct
  {
    // time (written by time_sync module)
    bool time_valid;
    int32_t utc_offset_sec;
    bool dst_in_effect;
    bool cal_due_check_suspended;
    bool cal_overdue;
    bool cal_time_stable;
    bool ntp_fail_alert_active;

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
    uint64_t sd_total_bytes;
    uint64_t sd_free_bytes;
    bool sd_space_reclaim_active;
    uint32_t sd_space_reclaim_deleted_total;
    bool sd_out_of_space_active;

    // fram (written by fram_log or storage task when it updates counters)
    uint32_t fram_count;
    uint32_t fram_capacity;
    bool fram_full;
    uint32_t fram_flush_watermark_records;
    bool fram_overrun_active;
    bool fram_io_error_active;
    bool i2c_recovery_active;
    uint32_t deferred_count;
    uint32_t deferred_drops;
    bool deferred_active;
    bool i2c_quiesce_active;

    bool data_stream_enabled;
    uint32_t data_stream_backend;
    int32_t data_stream_init_err;

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
    bool operator_hold_latched;
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

  typedef enum
  {
    RUNTIME_REBOOT_ALERT_GATE_UNKNOWN = 0,
    RUNTIME_REBOOT_ALERT_GATE_NOT_CONFIGURED = 1,
    RUNTIME_REBOOT_ALERT_GATE_DISABLED_BY_MASK = 2,
    RUNTIME_REBOOT_ALERT_GATE_NOT_ELIGIBLE_NET_MODE = 3,
    RUNTIME_REBOOT_ALERT_GATE_NOT_ELIGIBLE_ROLE = 4,
    RUNTIME_REBOOT_ALERT_GATE_WIFI_DISCONNECTED = 5,
    RUNTIME_REBOOT_ALERT_GATE_MESH_CONNECTED_BLOCKING_DIRECT = 6,
    RUNTIME_REBOOT_ALERT_GATE_COOLDOWN_ACTIVE = 7,
    RUNTIME_REBOOT_ALERT_GATE_QUEUE_FULL = 8,
  } runtime_reboot_alert_gate_reason_t;

  typedef enum
  {
    RUNTIME_REBOOT_ALERT_SEND_NONE = 0,
    RUNTIME_REBOOT_ALERT_SEND_OK = 1,
    RUNTIME_REBOOT_ALERT_SEND_FAIL = 2,
    RUNTIME_REBOOT_ALERT_SEND_SKIPPED = 3,
  } runtime_reboot_alert_send_result_t;

  typedef struct
  {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t pending_system_code;
    bool pending_is_active;
    int64_t pending_epoch;
    uint32_t pending_uptime_ms;
    uint32_t send_attempt_count;
    int64_t last_attempt_epoch;
    uint32_t last_attempt_uptime_ms;
    uint32_t last_gate_reason;
    uint32_t last_send_result;
    int32_t last_http_status;
    int32_t last_ntfy_err;
    int32_t last_retry_after_seconds;
    bool sent_successfully;
  } runtime_reboot_alert_latch_t;

  typedef struct
  {
    uint16_t code;
    bool resolved;
    int32_t detail0;
    int32_t detail1;
    uint32_t epoch_sec;
    uint16_t millis;
  } runtime_rtc_errlog_entry_t;

  typedef struct
  {
    uint32_t magic;
    uint16_t version;
    uint16_t head;
    uint16_t count;
    uint16_t reserved;
    runtime_rtc_errlog_entry_t entries[RUNTIME_RTC_ERRLOG_RING_SIZE];
  } runtime_rtc_errlog_latch_t;

  enum
  {
    kLogQueueDepth = 64,
    kStorageTaskStackBytes = 6144, // bytes

    // SD flush traverses FatFs/VFS/Newlib and the SDSPI DMA path. 4KB proved
    // marginal during watermark drains and day-rollover file creation (very
    // low observed watermark), which can lead to corruption/trace traps.
    kSdFlushTaskStackBytes = 8192,    // bytes
    kNetTxTaskStackBytes = 8192,      // bytes
    kAlertHttpTaskStackBytes = 11264, // bytes
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
    bool errlog_corrupt_alerted;
    sd_logger_t sd_logger;
    sd_card_detect_t sd_card_detect;
    StaticSemaphore_t sd_io_mutex_buf;
    SemaphoreHandle_t sd_io_mutex;
    StaticSemaphore_t spi_bus_mutex_buf;
    SemaphoreHandle_t spi_bus_mutex;
    SemaphoreHandle_t fram_log_mutex;
    max31865_reader_t sensor;
    mesh_transport_t mesh;
    time_sync_t time_sync;
    i2c_bus_t i2c_bus;
    StaticSemaphore_t i2c_mutex_buf;
    SemaphoreHandle_t i2c_mutex;
    const char* fram_log_mutex_holder;
    TickType_t fram_log_mutex_hold_start_ticks;
    uint32_t fram_log_mutex_hold_max_ms_observed;
    uint32_t fram_log_mutex_timeouts;
    const char* i2c_mutex_holder;
    TaskHandle_t i2c_mutex_owner_task;
    TickType_t i2c_mutex_hold_start_ticks;
    uint32_t i2c_mutex_hold_max_ms_observed;
    uint32_t i2c_mutex_timeouts;
    uint32_t i2c_mutex_same_task_reentry_count;
    uint32_t i2c_mutex_timeout_suppressed_count;
    uint32_t i2c_mutex_reentry_suppressed_count;
    TickType_t last_i2c_lock_dump_ticks;
    uint32_t last_i2c_timeout_warn_ms;
    uint32_t last_i2c_reentry_warn_ms;
    const char* sd_io_mutex_holder;
    TickType_t sd_io_mutex_hold_start_ticks;
    uint32_t sd_io_mutex_hold_max_ms_observed;
    uint32_t sd_io_mutex_timeouts;
    TickType_t last_lock_dump_ticks;
    runtime_error_entry_t error_ring[RUNTIME_ERROR_RING_SIZE];
    uint8_t error_ring_head;
    uint8_t error_ring_count;
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
    TickType_t sd_last_bus_reset_ticks;
    TickType_t sd_reset_pending_since_ticks;
    uint32_t sd_bus_reset_count;
    uint32_t sd_flush_records_since;
    bool sd_flush_in_progress;
    bool sd_flush_pending;
    bool sd_flush_quiesce_session_active;
    uint32_t sd_flush_pending_trigger_flags;
    uint32_t sd_flush_trigger_flags;
    uint32_t sd_flush_session_id;
    uint32_t sd_flush_session_records_flushed;
    TickType_t sd_flush_session_start_ticks;
    char sd_flush_session_label[RUNTIME_SD_FLUSH_SESSION_LABEL_LEN];
    volatile bool sd_manual_drain_active;
    volatile TickType_t sd_manual_drain_deadline_ticks;
    volatile uint32_t sd_manual_drain_passes;
    volatile uint32_t storage_marker;
    volatile uint32_t sd_flush_marker;
    const char* sd_flush_phase;
    esp_err_t sd_flush_last_err;
    uint32_t sd_flush_i2c_errs;
    uint32_t sd_flush_sd_errs;
    // If the FRAM->SD drain was requested at run start but SD was not mounted,
    // keep trying aggressively once the card becomes available.
    bool sd_start_drain_pending;
    bool sd_degraded;
    bool sd_force_unmount_on_append;
    bool sd_verify_readback;
    bool sd_last_io_error_active;
    uint32_t sd_io_error_streak;
    uint32_t sd_io_success_streak;
    // Set after SD EIO escalation; triggers a reboot instead of SPI teardown.
    bool sd_reset_pending;
    const char* sd_reset_pending_context;
    int sd_reset_pending_errno;
    esp_err_t sd_last_io_err;
    int sd_last_errno;
    bool fram_full;
    bool fram_available;
    int64_t fram_next_retry_ms;
    uint32_t last_fram_retry_log_ms;
    uint32_t fram_fallback_sequence;
    uint64_t fram_fallback_record_id;
    bool sd_was_mounted;
    TickType_t last_overrun_log_ticks;
    uint64_t last_overrun_records_total;
    uint64_t last_overrun_logged_total;
    uint64_t fram_overrun_ack_total;
    uint32_t fram_overrun_active_streak;
    uint32_t fram_overrun_clear_streak;
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
    uint32_t fram_append_fail_suppressed_count;
    uint32_t fram_crc_fail_suppressed_count;
    uint32_t last_fram_append_suppressed_log_ms;
    uint32_t last_fram_crc_suppressed_log_ms;
    uint32_t fram_log_lock_timeout_count_storage;
    uint32_t fram_log_lock_timeout_count_sdflush;
    uint32_t last_fram_log_lock_timeout_storage_log_ms;
    uint32_t last_fram_log_lock_timeout_sdflush_log_ms;
    bool i2c_recovery_in_progress;
    deferred_log_t deferred_log;
    bool i2c_quiesce_active;
    uint32_t i2c_quiesce_refcount;
    uint32_t i2c_quiesce_enter_count;
    const char* i2c_quiesce_reason;
    uint32_t deferred_drop_last_log_ms;

    // Sensor fault logging/debounce state (rate-limited).
    bool last_sensor_fault_present;
    int64_t rtd_fault_pending_since_ms;
    int64_t rtd_clear_pending_since_ms;
    bool rtd_fault_pending_was_raw;
    bool sensor_read_err_pending;
    uint32_t rtd_fault_suppressed_count;
    uint8_t rtd_fault_last_status;
    uint32_t rtd_fault_first_read_count;
    uint32_t rtd_fault_retry_fault_count;
    uint32_t rtd_fault_retry_clean_count;
    uint8_t rtd_fault_last_first_status;
    uint8_t rtd_fault_last_retry_status;
    esp_err_t rtd_fault_last_retry_err;
    uint32_t rtd_fault_ntfy_pending_first_read_count;
    uint32_t rtd_fault_ntfy_pending_retry_fault_count;
    uint32_t rtd_fault_ntfy_pending_retry_clean_count;
    uint8_t rtd_fault_ntfy_last_first_status;
    uint8_t rtd_fault_ntfy_last_retry_status;
    int64_t rtd_fault_ntfy_first_pending_ms;
    int64_t rtd_fault_ntfy_last_sent_ms;
    uint32_t rtd_fault_ntfy_suppressed_count;
    esp_err_t last_sensor_read_err;
    uint32_t max31865_read_err_count;
    esp_err_t max31865_last_read_err;
    uint32_t max31865_invalid_state_count;
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
    StackType_t
      storage_task_stack[kStorageTaskStackBytes / sizeof(StackType_t)];
    TaskHandle_t sd_flush_task;
    StaticTask_t sd_flush_task_tcb;
    StackType_t
      sd_flush_task_stack[kSdFlushTaskStackBytes / sizeof(StackType_t)];
    TaskHandle_t export_task;
    TaskHandle_t display_task;
    TaskHandle_t wifi_direct_task;
    TaskHandle_t control_task;
    TaskHandle_t net_tx_task;
    TaskHandle_t alert_http_task;
    StackType_t* alert_http_task_stack;
    StaticTask_t* alert_http_task_tcb;

    bool initialized;
    bool full_initialized;
    bool system_running;
    bool logger_running;
    bool stop_requested;
    volatile bool net_tx_pause_requested;
    volatile bool net_tx_paused;
    volatile bool alert_http_pause_requested;
    volatile bool alert_http_paused;
    bool spi_pause_requested;
    uint32_t spi_pause_ack_mask;
    bool mesh_started;
    bool wifi_direct_started;
    bool wifi_direct_time_synced;
    bool data_streaming_enabled;
    esp_err_t data_stream_init_err;
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

    uint32_t storage_stall_last_progress_ms;
    uint32_t storage_stall_last_fram_count;
    uint64_t storage_stall_last_sd_last_record_id;
    uint32_t storage_stall_last_sd_fail_count;
    bool storage_stall_active;

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
    uint32_t last_ntfy_queue_full_log_ms;
    uint32_t last_broker_bridge_disabled_log_ms;
    uint32_t last_mqtt_fail_log_ms;
  } runtime_state_t;

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUNTIME_STATE_H_
