#ifndef PT100_LOGGER_ALERT_MANAGER_H_
#define PT100_LOGGER_ALERT_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "log_record.h"

#include "alerts/alert_ntfy.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ALERT_MAX_LEAVES 1
#define ALERT_MAX_LEAF_OVERRIDES 1

  typedef struct alert_ntfy_batch_scratch_s alert_ntfy_batch_scratch_t;

  typedef enum
  {
    ALERT_TEMP_HIGH = 0,
    ALERT_TEMP_LOW = 1,
    ALERT_MISSING_RECORDS = 2,
    ALERT_LEAF_OFFLINE = 3,
    ALERT_LEAF_RESTART = 4,
    ALERT_ROOT_RESTART = 5,
    ALERT_SYSTEM_BOOT = 6,
    ALERT_SYSTEM_MODE = 7,
    ALERT_SYSTEM_ERROR = 8,
    ALERT_TYPE_COUNT
  } alert_type_t;

  typedef enum
  {
    ALERT_SEV_INFO = 0,
    ALERT_SEV_WARN = 1,
    ALERT_SEV_CRIT = 2,
  } alert_severity_t;

  typedef struct
  {
    uint64_t leaf_id;
    bool has_limits;
    int32_t high_limit_milli_c;
    int32_t low_limit_milli_c;
    bool has_enable_mask;
    uint32_t enable_mask;
  } alert_leaf_config_t;

  typedef struct
  {
    uint32_t version;
    char ntfy_url[128];
    char ntfy_topic[64];
    char ntfy_token[128];
    uint32_t enable_mask;
    uint32_t ntfy_min_send_interval_ms;
    uint32_t per_key_cooldown_ms;
    uint32_t global_max_per_minute;
    uint32_t missing_gap_ms;
    uint32_t offline_ms;
    uint32_t hold_ms;
    int32_t hysteresis_milli_c;
    int32_t default_high_milli_c;
    int32_t default_low_milli_c;
    uint32_t leaf_override_count;
    alert_leaf_config_t leaf_overrides[ALERT_MAX_LEAF_OVERRIDES];
  } alert_config_t;

  typedef struct
  {
    bool in_use;
    uint64_t leaf_id;
    int32_t last_temp_milli_c;
    uint32_t last_seq;
    uint64_t last_record_id;
    int64_t last_rx_epoch;
    int64_t last_rx_uptime_ms;
    int64_t last_restart_log_ms;
    int64_t last_online_ms;
    bool online;
    int64_t high_hold_start_ms;
    int64_t low_hold_start_ms;
    uint8_t missing_gap_sample_count;
    uint32_t missing_gap_threshold_ms;
  } alert_leaf_state_t;

  typedef struct
  {
    bool active;
    int64_t first_active_ms;
    int64_t last_seen_ms;
    int64_t last_change_ms;
    int64_t last_notify_ms;
    uint32_t notify_suppressed_count;
    uint32_t transitions;
    alert_severity_t last_severity;
  } alert_state_t;

  typedef struct
  {
    alert_config_t config;
    alert_leaf_state_t leaves[ALERT_MAX_LEAVES];
    alert_state_t states[ALERT_MAX_LEAVES][ALERT_TYPE_COUNT];
    alert_state_t system_error_state_by_code[ALERT_SYSTEM_CODE_ERROR_MAX + 1];
    alert_ntfy_batch_scratch_t* ntfy_batch_scratch;
    alert_ntfy_t ntfy;
    const char* root_id_string;
    uint64_t local_leaf_id;
    uint32_t ntfy_boot_nonce;
    uint32_t global_window_start_ms;
    uint32_t global_sent_in_window;
    bool startup_ntfy_pending_active;
    bool startup_ntfy_boot_seen;
    bool startup_ntfy_mode_seen;
    alert_system_code_t startup_ntfy_mode_code;
    int64_t startup_ntfy_first_ms;
    int64_t startup_ntfy_first_epoch;
  } alert_manager_t;

  typedef struct
  {
    alert_manager_t* manager;
    volatile bool* stop_requested;
    TaskHandle_t* task_handle;
  } alert_task_context_t;

  /**
   * @brief Execute AlertManagerInit.
   * @param manager Parameter manager.
   * @param root_id_string Parameter root_id_string.
   * @param local_leaf_id Parameter local_leaf_id.
   */
  void AlertManagerInit(alert_manager_t* manager,
                        const char* root_id_string,
                        uint64_t local_leaf_id);

  /**
   * @brief Execute AlertManagerLoadConfig.
   * @param manager Parameter manager.
   * @return Return the function result.
   */
  esp_err_t AlertManagerLoadConfig(alert_manager_t* manager);

  /**
   * @brief Execute AlertManagerSaveConfig.
   * @param manager Parameter manager.
   * @return Return the function result.
   */
  esp_err_t AlertManagerSaveConfig(alert_manager_t* manager);

  /**
   * @brief Execute AlertManagerOnSample.
   * @param manager Parameter manager.
   * @param leaf_id Parameter leaf_id.
   * @param record Parameter record.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   */
  void AlertManagerOnSample(alert_manager_t* manager,
                            uint64_t leaf_id,
                            const log_record_t* record,
                            int64_t now_ms,
                            int64_t now_epoch);

  /**
   * @brief Execute AlertManagerOnLeafOnline.
   * @param manager Parameter manager.
   * @param leaf_id Parameter leaf_id.
   * @param online Parameter online.
   * @param now_ms Parameter now_ms.
   */
  void AlertManagerOnLeafOnline(alert_manager_t* manager,
                                uint64_t leaf_id,
                                bool online,
                                int64_t now_ms);

  /**
   * @brief Reset local-leaf alert baselines at logging session start.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   * @details Treat runtime start as a new logging session by resetting only
   * session-derived local-leaf baselines/state (last receive times,
   * sequence/record-id restart detectors, temperature hold timers, and per-leaf
   * alert transition state). This prevents false missing-gap and restart alerts
   * from spanning a planned stop/start boundary without altering alert
   * configuration or global rate-limit state.
   */
  void AlertManagerOnLoggingSessionStart(alert_manager_t* manager,
                                         int64_t now_ms,
                                         int64_t now_epoch);

  /**
   * @brief Execute AlertManagerTick.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   */
  void AlertManagerTick(alert_manager_t* manager,
                        int64_t now_ms,
                        int64_t now_epoch);

  /**
   * @brief Execute AlertManagerIsConfigured.
   * @param manager Parameter manager.
   * @return Return the function result.
   */
  bool AlertManagerIsConfigured(const alert_manager_t* manager);

  /**
   * @brief Execute AlertManagerEnableType.
   * @param manager Parameter manager.
   * @param type Parameter type.
   * @param enabled Parameter enabled.
   * @param leaf_id Parameter leaf_id.
   * @param per_leaf Parameter per_leaf.
   * @return Return the function result.
   */
  bool AlertManagerEnableType(alert_manager_t* manager,
                              alert_type_t type,
                              bool enabled,
                              uint64_t leaf_id,
                              bool per_leaf);

  /**
   * @brief Execute AlertManagerSetDefaultLimit.
   * @param manager Parameter manager.
   * @param is_high Parameter is_high.
   * @param limit_milli_c Parameter limit_milli_c.
   * @return Return the function result.
   */
  bool AlertManagerSetDefaultLimit(alert_manager_t* manager,
                                   bool is_high,
                                   int32_t limit_milli_c);

  /**
   * @brief Execute AlertManagerSetLeafLimit.
   * @param manager Parameter manager.
   * @param leaf_id Parameter leaf_id.
   * @param is_high Parameter is_high.
   * @param limit_milli_c Parameter limit_milli_c.
   * @return Return the function result.
   */
  bool AlertManagerSetLeafLimit(alert_manager_t* manager,
                                uint64_t leaf_id,
                                bool is_high,
                                int32_t limit_milli_c);

  /**
   * @brief Execute AlertManagerSetMissingGap.
   * @param manager Parameter manager.
   * @param gap_ms Parameter gap_ms.
   * @return Return the function result.
   */
  bool AlertManagerSetMissingGap(alert_manager_t* manager, uint32_t gap_ms);

  /**
   * @brief Execute AlertManagerSetOfflineMs.
   * @param manager Parameter manager.
   * @param offline_ms Parameter offline_ms.
   * @return Return the function result.
   */
  bool AlertManagerSetOfflineMs(alert_manager_t* manager, uint32_t offline_ms);

  /**
   * @brief Execute AlertManagerSetHoldMs.
   * @param manager Parameter manager.
   * @param hold_ms Parameter hold_ms.
   * @return Return the function result.
   */
  bool AlertManagerSetHoldMs(alert_manager_t* manager, uint32_t hold_ms);

  /**
   * @brief Execute AlertManagerSetHysteresis.
   * @param manager Parameter manager.
   * @param hysteresis_milli_c Parameter hysteresis_milli_c.
   * @return Return the function result.
   */
  bool AlertManagerSetHysteresis(alert_manager_t* manager,
                                 int32_t hysteresis_milli_c);

  /**
   * @brief Execute AlertManagerSetRateLimit.
   * @param manager Parameter manager.
   * @param per_key_ms Parameter per_key_ms.
   * @param per_minute Parameter per_minute.
   * @return Return the function result.
   */
  bool AlertManagerSetRateLimit(alert_manager_t* manager,
                                uint32_t per_key_ms,
                                uint32_t per_minute);

  /**
   * @brief Execute AlertManagerSetNtfyMinIntervalMs.
   * @param manager Parameter manager.
   * @param min_interval_ms Parameter min_interval_ms.
   * @return Return the function result.
   */
  bool AlertManagerSetNtfyMinIntervalMs(alert_manager_t* manager,
                                        uint32_t min_interval_ms);

  /**
   * @brief Execute AlertManagerSetNtfyUrl.
   * @param manager Parameter manager.
   * @param url Parameter url.
   * @return Return the function result.
   */
  bool AlertManagerSetNtfyUrl(alert_manager_t* manager, const char* url);

  /**
   * @brief Execute AlertManagerSetNtfyTopic.
   * @param manager Parameter manager.
   * @param topic Parameter topic.
   * @return Return the function result.
   */
  bool AlertManagerSetNtfyTopic(alert_manager_t* manager, const char* topic);

  /**
   * @brief Execute AlertManagerSetNtfyToken.
   * @param manager Parameter manager.
   * @param token Parameter token.
   * @return Return the function result.
   */
  bool AlertManagerSetNtfyToken(alert_manager_t* manager, const char* token);

  /**
   * @brief Execute AlertManagerClear.
   * @param manager Parameter manager.
   * @param type Parameter type.
   * @param leaf_id Parameter leaf_id.
   * @param all_leaves Parameter all_leaves.
   */
  void AlertManagerClear(alert_manager_t* manager,
                         alert_type_t type,
                         uint64_t leaf_id,
                         bool all_leaves);

  /**
   * @brief Execute AlertManagerSendTest.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   * @return True when the test note was queued to ntfy note queue.
   */
  bool AlertManagerSendTest(alert_manager_t* manager, int64_t now_ms);

  /**
   * @brief Queue a lightweight direct ntfy transport/TLS probe message.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   * @return True when the direct ntfy job was queued to ntfy job queue.
   */
  bool AlertManagerSendDirectNtfyTest(alert_manager_t* manager, int64_t now_ms);

  /**
   * @brief Execute AlertManagerEmitRootRestart.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   */
  void AlertManagerEmitRootRestart(alert_manager_t* manager, int64_t now_ms);

  /**
   * @brief Execute AlertManagerEmitSystemBoot.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   */
  void AlertManagerEmitSystemBoot(alert_manager_t* manager,
                                  int64_t now_ms,
                                  int64_t now_epoch);

  /**
   * @brief Execute AlertManagerEmitSystemMode.
   * @param manager Parameter manager.
   * @param mode_code Parameter mode_code.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   */
  void AlertManagerEmitSystemMode(alert_manager_t* manager,
                                  alert_system_code_t mode_code,
                                  int64_t now_ms,
                                  int64_t now_epoch);

  /**
   * @brief Open startup ntfy coalescing window for initial boot/mode events.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   */
  void AlertManagerStartupNtfyBegin(alert_manager_t* manager,
                                    int64_t now_ms,
                                    int64_t now_epoch);

  /**
   * @brief Update pending startup ntfy with the initial mode event.
   * @param manager Parameter manager.
   * @param mode_code Parameter mode_code.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   */
  void AlertManagerStartupNtfyUpdateMode(alert_manager_t* manager,
                                         alert_system_code_t mode_code,
                                         int64_t now_ms,
                                         int64_t now_epoch);

  /**
   * @brief Flush pending startup ntfy when coalescing conditions are met.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   */
  void AlertManagerStartupNtfyTick(alert_manager_t* manager,
                                   int64_t now_ms,
                                   int64_t now_epoch);

  /**
   * @brief Execute AlertManagerProcessSystemError.
   * @param manager Parameter manager.
   * @param error_code Parameter error_code.
   * @param active Parameter active.
   * @param now_ms Parameter now_ms.
   * @param now_epoch Parameter now_epoch.
   */
  void AlertManagerProcessSystemError(alert_manager_t* manager,
                                      alert_system_code_t error_code,
                                      bool active,
                                      int64_t now_ms,
                                      int64_t now_epoch);

  /**
   * @brief Execute AlertManagerCopyLeaves.
   * @param manager Parameter manager.
   * @param out Parameter out.
   * @param max_items Parameter max_items.
   * @return Return the function result.
   */
  size_t AlertManagerCopyLeaves(const alert_manager_t* manager,
                                alert_leaf_state_t* out,
                                size_t max_items);

  /**
   * @brief Execute AlertManagerCopyActiveAlerts.
   * @param manager Parameter manager.
   * @param out_states Parameter out_states.
   * @param out_types Parameter out_types.
   * @param out_leaf_ids Parameter out_leaf_ids.
   * @param max_items Parameter max_items.
   * @return Return the function result.
   */
  size_t AlertManagerCopyActiveAlerts(const alert_manager_t* manager,
                                      alert_state_t* out_states,
                                      alert_type_t* out_types,
                                      uint64_t* out_leaf_ids,
                                      size_t max_items);

  /**
   * @brief Execute AlertManagerFormatLeafId.
   * @param leaf_id Parameter leaf_id.
   * @param out Parameter out.
   * @param out_size Parameter out_size.
   */
  void AlertManagerFormatLeafId(uint64_t leaf_id, char* out, size_t out_size);

  /**
   * @brief Execute AlertManagerPumpNtfy.
   * @param manager Parameter manager.
   * @param now_ms Parameter now_ms.
   * @param next_attempt_ms Parameter next_attempt_ms.
   * @return Return the function result.
   */
  bool AlertManagerPumpNtfy(alert_manager_t* manager,
                            int64_t now_ms,
                            int64_t* next_attempt_ms);

  /**
   * @brief Update ntfy send state after a send attempt.
   * @param manager Parameter manager.
   * @param result Parameter result.
   * @param status Parameter status.
   * @param retry_after_seconds Parameter retry_after_seconds.
   * @param err Parameter err.
   * @param now_ms Parameter now_ms.
   * @param next_attempt_ms Parameter next_attempt_ms.
   */
  void AlertManagerUpdateNtfySendState(alert_manager_t* manager,
                                       alert_ntfy_result_t result,
                                       int status,
                                       int retry_after_seconds,
                                       esp_err_t err,
                                       int64_t now_ms,
                                       int64_t* next_attempt_ms);

  /**
   * @brief Build a consolidated stop-flush ntfy job and drain pending queues.
   * @param manager Parameter manager.
   * @param stop_reason Parameter stop_reason.
   * @param now_ms Parameter now_ms.
   * @param out_job Parameter out_job.
   * @param out_notes_drained Parameter out_notes_drained.
   * @param out_jobs_drained Parameter out_jobs_drained.
   * @return True when a job was built.
   */
  bool AlertManagerBuildStopFlushNtfyJob(alert_manager_t* manager,
                                         const char* stop_reason,
                                         int64_t now_ms,
                                         alert_ntfy_job_t* out_job,
                                         uint32_t* out_notes_drained,
                                         uint32_t* out_jobs_drained);

  /**
   * @brief Execute AlertManagerMonitorTask.
   * @param context Parameter context.
   * @note FreeRTOS task entry for the AlertManagerMonitorTask task.
   */
  void AlertManagerMonitorTask(void* context);

  /**
   * @brief Execute AlertManagerSenderTask.
   * @param context Parameter context.
   * @note FreeRTOS task entry for the AlertManagerSenderTask task.
   */
  void AlertManagerSenderTask(void* context);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_ALERT_MANAGER_H_
