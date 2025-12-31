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
extern "C" {
#endif

#define ALERT_MAX_LEAVES 32
#define ALERT_MAX_LEAF_OVERRIDES 16

typedef enum
{
  ALERT_TEMP_HIGH = 0,
  ALERT_TEMP_LOW = 1,
  ALERT_MISSING_RECORDS = 2,
  ALERT_LEAF_OFFLINE = 3,
  ALERT_LEAF_RESTART = 4,
  ALERT_ROOT_RESTART = 5,
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
  int64_t last_rx_epoch;
  int64_t last_rx_uptime_ms;
  int64_t last_online_ms;
  bool online;
  int64_t high_hold_start_ms;
  int64_t low_hold_start_ms;
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
  alert_ntfy_t ntfy;
  const char* root_id_string;
  uint32_t global_window_start_ms;
  uint32_t global_sent_in_window;
} alert_manager_t;

typedef struct
{
  alert_manager_t* manager;
  volatile bool* stop_requested;
  TaskHandle_t* task_handle;
} alert_task_context_t;

void AlertManagerInit(alert_manager_t* manager, const char* root_id_string);

esp_err_t AlertManagerLoadConfig(alert_manager_t* manager);

esp_err_t AlertManagerSaveConfig(alert_manager_t* manager);

void AlertManagerOnSample(alert_manager_t* manager,
                          uint64_t leaf_id,
                          const log_record_t* record,
                          int64_t now_ms,
                          int64_t now_epoch);

void AlertManagerOnLeafOnline(alert_manager_t* manager,
                              uint64_t leaf_id,
                              bool online,
                              int64_t now_ms);

void AlertManagerTick(alert_manager_t* manager,
                      int64_t now_ms,
                      int64_t now_epoch);

bool AlertManagerIsConfigured(const alert_manager_t* manager);

bool AlertManagerEnableType(alert_manager_t* manager,
                            alert_type_t type,
                            bool enabled,
                            uint64_t leaf_id,
                            bool per_leaf);

bool AlertManagerSetDefaultLimit(alert_manager_t* manager,
                                 bool is_high,
                                 int32_t limit_milli_c);

bool AlertManagerSetLeafLimit(alert_manager_t* manager,
                              uint64_t leaf_id,
                              bool is_high,
                              int32_t limit_milli_c);

bool AlertManagerSetMissingGap(alert_manager_t* manager, uint32_t gap_ms);

bool AlertManagerSetOfflineMs(alert_manager_t* manager, uint32_t offline_ms);

bool AlertManagerSetHoldMs(alert_manager_t* manager, uint32_t hold_ms);

bool AlertManagerSetHysteresis(alert_manager_t* manager,
                               int32_t hysteresis_milli_c);

bool AlertManagerSetRateLimit(alert_manager_t* manager,
                              uint32_t per_key_ms,
                              uint32_t per_minute);

bool AlertManagerSetNtfyUrl(alert_manager_t* manager, const char* url);

bool AlertManagerSetNtfyTopic(alert_manager_t* manager, const char* topic);

bool AlertManagerSetNtfyToken(alert_manager_t* manager, const char* token);

void AlertManagerClear(alert_manager_t* manager,
                       alert_type_t type,
                       uint64_t leaf_id,
                       bool all_leaves);

void AlertManagerSendTest(alert_manager_t* manager, int64_t now_ms);

void AlertManagerEmitRootRestart(alert_manager_t* manager, int64_t now_ms);

size_t AlertManagerCopyLeaves(const alert_manager_t* manager,
                              alert_leaf_state_t* out,
                              size_t max_items);

size_t AlertManagerCopyActiveAlerts(const alert_manager_t* manager,
                                    alert_state_t* out_states,
                                    alert_type_t* out_types,
                                    uint64_t* out_leaf_ids,
                                    size_t max_items);

void AlertManagerFormatLeafId(uint64_t leaf_id, char* out, size_t out_size);

void AlertManagerMonitorTask(void* context);

void AlertManagerSenderTask(void* context);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_ALERT_MANAGER_H_
