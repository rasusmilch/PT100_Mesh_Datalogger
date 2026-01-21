#include "alerts/alert_manager.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_mesh_lite.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "time_sync.h"

static const char* kTag = "alert_mgr";
static const int64_t kNtfyRateLimitBaseCooldownMs = 120000;
static const int64_t kNtfyRateLimitMaxCooldownMs = 900000;
static const int64_t kNtfyDedupeWindowMs = 300000;
static const int64_t kNtfyFailureMaxBackoffMs = 300000;
static const bool kNtfySendSuppressedSummary = true;
static const uint32_t kAlertConfigVersion = 1;
static const char* kAlertNvsNamespace = "alerts";
static const char* kAlertNvsConfigKey = "config";

#if CONFIG_MESH_LITE_NODE_INFO_REPORT
/**
 * @brief Execute PackMacToId.
 * @param mac Parameter mac.
 * @return Return the function result.
 */
static uint64_t
PackMacToId(const uint8_t mac[6])
{
  uint64_t value = 0;
  for (int i = 0; i < 6; ++i) {
    value = (value << 8) | mac[i];
  }
  return value;
}
#endif

/**
 * @brief Resolve pseudo leaf id (0) to the local device leaf id.
 * @param manager Alert manager instance.
 * @param leaf_id Input leaf id.
 * @return Canonical leaf id used for internal tracking.
 */
static uint64_t
ResolveLeafId(const alert_manager_t* manager, uint64_t leaf_id)
{
  if (leaf_id == 0 && manager != NULL && manager->local_leaf_id != 0) {
    return manager->local_leaf_id;
  }
  return leaf_id;
}

/**
 * @brief Execute FindLeaf.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @return Return the function result.
 */
static alert_leaf_state_t*
FindLeaf(alert_manager_t* manager, uint64_t leaf_id)
{
  if (manager == NULL) {
    return NULL;
  }
  leaf_id = ResolveLeafId(manager, leaf_id);
  for (size_t i = 0; i < ALERT_MAX_LEAVES; ++i) {
    if (manager->leaves[i].in_use && manager->leaves[i].leaf_id == leaf_id) {
      return &manager->leaves[i];
    }
  }
  return NULL;
}

/**
 * @brief Execute FindOrAllocateLeaf.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @return Return the function result.
 */
static alert_leaf_state_t*
FindOrAllocateLeaf(alert_manager_t* manager, uint64_t leaf_id)
{
  if (manager == NULL) {
    return NULL;
  }
  leaf_id = ResolveLeafId(manager, leaf_id);
  alert_leaf_state_t* existing = FindLeaf(manager, leaf_id);
  if (existing != NULL) {
    return existing;
  }
  for (size_t i = 0; i < ALERT_MAX_LEAVES; ++i) {
    if (!manager->leaves[i].in_use) {
      manager->leaves[i].in_use = true;
      manager->leaves[i].leaf_id = leaf_id;
      return &manager->leaves[i];
    }
  }
  size_t oldest = 0;
  int64_t oldest_time = INT64_MAX;
  for (size_t i = 0; i < ALERT_MAX_LEAVES; ++i) {
    if (manager->leaves[i].last_rx_uptime_ms < oldest_time) {
      oldest_time = manager->leaves[i].last_rx_uptime_ms;
      oldest = i;
    }
  }
  manager->leaves[oldest] = (alert_leaf_state_t){
    .in_use = true,
    .leaf_id = leaf_id,
  };
  return &manager->leaves[oldest];
}

/**
 * @brief Execute FindOrAllocateLeafIndex.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @param index_out Parameter index_out.
 * @return Return the function result.
 */
static bool
FindOrAllocateLeafIndex(alert_manager_t* manager,
                        uint64_t leaf_id,
                        size_t* index_out)
{
  if (manager == NULL || index_out == NULL) {
    return false;
  }
  alert_leaf_state_t* leaf = FindOrAllocateLeaf(manager, leaf_id);
  if (leaf == NULL) {
    return false;
  }
  *index_out = (size_t)(leaf - manager->leaves);
  return true;
}

/**
 * @brief Execute GetLeafConfig.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @param out Parameter out.
 * @return Return the function result.
 */
static bool
GetLeafConfig(const alert_manager_t* manager,
              uint64_t leaf_id,
              alert_leaf_config_t* out)
{
  if (manager == NULL || out == NULL) {
    return false;
  }
  leaf_id = ResolveLeafId(manager, leaf_id);
  for (size_t i = 0; i < manager->config.leaf_override_count; ++i) {
    if (manager->config.leaf_overrides[i].leaf_id == leaf_id) {
      *out = manager->config.leaf_overrides[i];
      return true;
    }
  }
  return false;
}

/**
 * @brief Execute GetOrCreateLeafConfig.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @return Return the function result.
 */
static alert_leaf_config_t*
GetOrCreateLeafConfig(alert_manager_t* manager, uint64_t leaf_id)
{
  if (manager == NULL) {
    return NULL;
  }
  leaf_id = ResolveLeafId(manager, leaf_id);
  for (size_t i = 0; i < manager->config.leaf_override_count; ++i) {
    if (manager->config.leaf_overrides[i].leaf_id == leaf_id) {
      return &manager->config.leaf_overrides[i];
    }
  }
  if (manager->config.leaf_override_count >= ALERT_MAX_LEAF_OVERRIDES) {
    return NULL;
  }
  alert_leaf_config_t* entry =
    &manager->config.leaf_overrides[manager->config.leaf_override_count++];
  memset(entry, 0, sizeof(*entry));
  entry->leaf_id = leaf_id;
  return entry;
}

/**
 * @brief Execute EffectiveEnableMask.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @return Return the function result.
 */
static uint32_t
EffectiveEnableMask(const alert_manager_t* manager, uint64_t leaf_id)
{
  leaf_id = ResolveLeafId(manager, leaf_id);
  alert_leaf_config_t leaf_config;
  if (GetLeafConfig(manager, leaf_id, &leaf_config) &&
      leaf_config.has_enable_mask) {
    return leaf_config.enable_mask;
  }
  return manager->config.enable_mask;
}

/**
 * @brief Execute AlertStateTransition.
 * @param state Parameter state.
 * @param active Parameter active.
 * @param now_ms Parameter now_ms.
 */
static void
AlertStateTransition(alert_state_t* state, bool active, int64_t now_ms)
{
  if (state == NULL) {
    return;
  }
  state->active = active;
  state->last_change_ms = now_ms;
  state->transitions++;
  if (active) {
    if (state->first_active_ms == 0) {
      state->first_active_ms = now_ms;
    }
    state->last_seen_ms = now_ms;
  }
}

/**
 * @brief Execute AlertManagerQueueNotification.
 * @param manager Parameter manager.
 * @param state Parameter state.
 * @param type Parameter type.
 * @param severity Parameter severity.
 * @param resolved Parameter resolved.
 * @param leaf_id Parameter leaf_id.
 * @param payload Parameter payload.
 * @param now_ms Parameter now_ms.
 * @return Return the function result.
 */
static bool
AlertManagerQueueNotification(alert_manager_t* manager,
                              alert_state_t* state,
                              alert_type_t type,
                              alert_severity_t severity,
                              bool resolved,
                              uint64_t leaf_id,
                              const alert_notification_payload_t* payload,
                              int64_t now_ms)
{
  if (manager == NULL || payload == NULL) {
    return false;
  }
  if (!AlertManagerIsConfigured(manager)) {
    if (state != NULL) {
      state->notify_suppressed_count++;
    }
    return false;
  }

  if (manager->config.global_max_per_minute > 0) {
    if (now_ms - (int64_t)manager->global_window_start_ms >= 60000) {
      manager->global_window_start_ms = (uint32_t)now_ms;
      manager->global_sent_in_window = 0;
    }
    if (manager->global_sent_in_window >=
        manager->config.global_max_per_minute) {
      if (state != NULL) {
        state->notify_suppressed_count++;
      }
      return false;
    }
  }

  alert_notification_t note = {
    .type = type,
    .severity = severity,
    .resolved = resolved,
    .leaf_id = leaf_id,
    .payload = *payload,
  };

  if (!AlertNtfyEnqueue(&manager->ntfy, &note)) {
    if (state != NULL) {
      state->notify_suppressed_count++;
    }
    return false;
  }
  manager->global_sent_in_window++;
  return true;
}

/**
 * @brief Execute AlertManagerQueueOneShot.
 * @param manager Parameter manager.
 * @param state Parameter state.
 * @param type Parameter type.
 * @param severity Parameter severity.
 * @param leaf_id Parameter leaf_id.
 * @param payload Parameter payload.
 * @param now_ms Parameter now_ms.
 * @return Return the function result.
 */
static bool
AlertManagerQueueOneShot(alert_manager_t* manager,
                         alert_state_t* state,
                         alert_type_t type,
                         alert_severity_t severity,
                         uint64_t leaf_id,
                         const alert_notification_payload_t* payload,
                         int64_t now_ms)
{
  if (manager == NULL) {
    return false;
  }
  if (state != NULL && state->last_notify_ms != 0 &&
      manager->config.per_key_cooldown_ms > 0 &&
      (now_ms - state->last_notify_ms) <
        (int64_t)manager->config.per_key_cooldown_ms) {
    state->notify_suppressed_count++;
    return false;
  }
  if (AlertManagerQueueNotification(manager,
                                    state,
                                    type,
                                    severity,
                                    false,
                                    leaf_id,
                                    payload,
                                    now_ms)) {
    if (state != NULL) {
      state->last_notify_ms = now_ms;
      state->last_severity = severity;
    }
    return true;
  }
  return false;
}

/**
 * @brief Execute FillPayloadBase.
 * @param payload Parameter payload.
 * @param leaf Parameter leaf.
 * @param now_ms Parameter now_ms.
 * @param now_epoch Parameter now_epoch.
 */
static void
FillPayloadBase(alert_notification_payload_t* payload,
                const alert_leaf_state_t* leaf,
                int64_t now_ms,
                int64_t now_epoch)
{
  payload->current_temp_milli_c = leaf ? leaf->last_temp_milli_c : 0;
  payload->event_uptime_ms = now_ms;
  payload->event_epoch = (now_epoch > 0) ? now_epoch : -1;
  if (leaf != NULL) {
    payload->last_seq = leaf->last_seq;
    payload->last_rx_epoch = leaf->last_rx_epoch;
    payload->last_rx_uptime_ms = leaf->last_rx_uptime_ms;
  }
}

/**
 * @brief Execute ProcessAlert.
 * @param manager Parameter manager.
 * @param leaf_index Parameter leaf_index.
 * @param type Parameter type.
 * @param severity Parameter severity.
 * @param condition_active Parameter condition_active.
 * @param payload Parameter payload.
 * @param now_ms Parameter now_ms.
 */
static void
ProcessAlert(alert_manager_t* manager,
             size_t leaf_index,
             alert_type_t type,
             alert_severity_t severity,
             bool condition_active,
             alert_notification_payload_t* payload,
             int64_t now_ms)
{
  alert_state_t* state = &manager->states[leaf_index][type];
  if (condition_active) {
    state->last_seen_ms = now_ms;
    if (!state->active) {
      AlertStateTransition(state, true, now_ms);
      if (payload != NULL) {
        payload->transitions = state->transitions;
      }
      if (AlertManagerQueueNotification(manager,
                                        state,
                                        type,
                                        severity,
                                        false,
                                        manager->leaves[leaf_index].leaf_id,
                                        payload,
                                        now_ms)) {
        state->last_notify_ms = now_ms;
      }
      state->last_severity = severity;
    } else if (manager->config.per_key_cooldown_ms > 0 &&
               (now_ms - state->last_notify_ms) >=
                 (int64_t)manager->config.per_key_cooldown_ms) {
      if (payload != NULL) {
        payload->transitions = state->transitions;
      }
      if (AlertManagerQueueNotification(manager,
                                        state,
                                        type,
                                        severity,
                                        false,
                                        manager->leaves[leaf_index].leaf_id,
                                        payload,
                                        now_ms)) {
        state->last_notify_ms = now_ms;
      }
      state->last_severity = severity;
    }
  } else if (state->active) {
    AlertStateTransition(state, false, now_ms);
    if (payload != NULL) {
      payload->transitions = state->transitions;
    }
    if (AlertManagerQueueNotification(manager,
                                      state,
                                      type,
                                      severity,
                                      true,
                                      manager->leaves[leaf_index].leaf_id,
                                      payload,
                                      now_ms)) {
      state->last_notify_ms = now_ms;
    }
  }
}

/**
 * @brief Execute GetLimits.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @param high_out Parameter high_out.
 * @param low_out Parameter low_out.
 * @return Return the function result.
 */
static bool
GetLimits(const alert_manager_t* manager,
          uint64_t leaf_id,
          int32_t* high_out,
          int32_t* low_out)
{
  if (manager == NULL || high_out == NULL || low_out == NULL) {
    return false;
  }
  alert_leaf_config_t leaf_config;
  if (GetLeafConfig(manager, leaf_id, &leaf_config) && leaf_config.has_limits) {
    *high_out = leaf_config.high_limit_milli_c;
    *low_out = leaf_config.low_limit_milli_c;
    return true;
  }
  *high_out = manager->config.default_high_milli_c;
  *low_out = manager->config.default_low_milli_c;
  return true;
}

/**
 * @brief Execute RefreshMeshOnline.
 * @param manager Parameter manager.
 * @param now_ms Parameter now_ms.
 */
static void
RefreshMeshOnline(alert_manager_t* manager, int64_t now_ms)
{
  if (manager == NULL) {
    return;
  }
  bool seen[ALERT_MAX_LEAVES] = { 0 };
  bool update_offline = false;

#if CONFIG_MESH_LITE_NODE_INFO_REPORT
  uint32_t total_nodes = 0;
  const node_info_list_t* node = esp_mesh_lite_get_nodes_list(&total_nodes);
  for (const node_info_list_t* entry = node; entry != NULL;
       entry = entry->next) {
    const esp_mesh_lite_node_info_t* info = entry->node;
    if (info == NULL) {
      continue;
    }
    const uint64_t leaf_id = PackMacToId(info->mac_addr);
    alert_leaf_state_t* leaf = FindOrAllocateLeaf(manager, leaf_id);
    if (leaf == NULL) {
      continue;
    }
    leaf->online = true;
    leaf->last_online_ms = now_ms;
    for (size_t i = 0; i < ALERT_MAX_LEAVES; ++i) {
      if (manager->leaves[i].in_use && manager->leaves[i].leaf_id == leaf_id) {
        seen[i] = true;
        break;
      }
    }
  }
  update_offline = true;
#else
  (void)now_ms;
#endif

  if (update_offline) {
    for (size_t i = 0; i < ALERT_MAX_LEAVES; ++i) {
      if (!manager->leaves[i].in_use) {
        continue;
      }
      if (!seen[i] && manager->leaves[i].online) {
        manager->leaves[i].online = false;
      }
    }
  }
}

/**
 * @brief Execute ApplyDefaults.
 * @param manager Parameter manager.
 */
static void
ApplyDefaults(alert_manager_t* manager)
{
  manager->config.version = kAlertConfigVersion;
  // Avoid -Wformat-zero-length from snprintf("", ...). These strings are
  // intentionally empty defaults.
  memset(manager->config.ntfy_url, 0, sizeof(manager->config.ntfy_url));
  memset(manager->config.ntfy_topic, 0, sizeof(manager->config.ntfy_topic));
  memset(manager->config.ntfy_token, 0, sizeof(manager->config.ntfy_token));
  manager->config.enable_mask =
    (1u << ALERT_MISSING_RECORDS) | (1u << ALERT_LEAF_OFFLINE) |
    (1u << ALERT_LEAF_RESTART) | (1u << ALERT_ROOT_RESTART) |
    (1u << ALERT_SYSTEM_BOOT) | (1u << ALERT_SYSTEM_MODE) |
    (1u << ALERT_SYSTEM_ERROR);
  manager->config.per_key_cooldown_ms = 300000;
  manager->config.global_max_per_minute = 12;
  manager->config.missing_gap_ms = 15000;
  manager->config.offline_ms = 60000;
  manager->config.hold_ms = 5000;
  manager->config.hysteresis_milli_c = 500;
  manager->config.default_high_milli_c = 80000;
  manager->config.default_low_milli_c = 20000;
  manager->config.leaf_override_count = 0;
  memset(
    manager->config.leaf_overrides, 0, sizeof(manager->config.leaf_overrides));
}

/**
 * @brief Execute AlertManagerInit.
 * @param manager Parameter manager.
 * @param root_id_string Parameter root_id_string.
 * @param local_leaf_id Parameter local_leaf_id.
 */
void
AlertManagerInit(alert_manager_t* manager,
                 const char* root_id_string,
                 uint64_t local_leaf_id)
{
  if (manager == NULL) {
    return;
  }
  memset(manager, 0, sizeof(*manager));
  manager->root_id_string = root_id_string;
  manager->local_leaf_id = local_leaf_id;
  ApplyDefaults(manager);
  AlertNtfyInit(&manager->ntfy);
}

/**
 * @brief Execute AlertManagerLoadConfig.
 * @param manager Parameter manager.
 * @return Return the function result.
 */
esp_err_t
AlertManagerLoadConfig(alert_manager_t* manager)
{
  if (manager == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  ApplyDefaults(manager);
  nvs_handle_t handle;
  esp_err_t result = nvs_open(kAlertNvsNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "nvs_open failed: %s", esp_err_to_name(result));
    return result;
  }
  size_t size = sizeof(alert_config_t);
  alert_config_t loaded = { 0 };
  result = nvs_get_blob(handle, kAlertNvsConfigKey, &loaded, &size);
  if (result == ESP_OK && size == sizeof(alert_config_t) &&
      loaded.version == kAlertConfigVersion) {
    manager->config = loaded;
  } else if (result != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(kTag, "invalid alert config; using defaults");
  }
  nvs_close(handle);
  return ESP_OK;
}

/**
 * @brief Execute AlertManagerSaveConfig.
 * @param manager Parameter manager.
 * @return Return the function result.
 */
esp_err_t
AlertManagerSaveConfig(alert_manager_t* manager)
{
  if (manager == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  manager->config.version = kAlertConfigVersion;
  nvs_handle_t handle;
  esp_err_t result = nvs_open(kAlertNvsNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_blob(
    handle, kAlertNvsConfigKey, &manager->config, sizeof(manager->config));
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AlertManagerIsConfigured.
 * @param manager Parameter manager.
 * @return Return the function result.
 */
bool
AlertManagerIsConfigured(const alert_manager_t* manager)
{
  if (manager == NULL) {
    return false;
  }
  return manager->config.ntfy_url[0] != '\0' &&
         manager->config.ntfy_topic[0] != '\0';
}

/**
 * @brief Execute AlertManagerOnSample.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @param record Parameter record.
 * @param now_ms Parameter now_ms.
 * @param now_epoch Parameter now_epoch.
 */
void
AlertManagerOnSample(alert_manager_t* manager,
                     uint64_t leaf_id,
                     const log_record_t* record,
                     int64_t now_ms,
                     int64_t now_epoch)
{
  if (manager == NULL || record == NULL) {
    return;
  }
  leaf_id = ResolveLeafId(manager, leaf_id);
  alert_leaf_state_t* leaf = FindOrAllocateLeaf(manager, leaf_id);
  if (leaf == NULL) {
    return;
  }
  const uint32_t mask = EffectiveEnableMask(manager, leaf_id);
  if ((mask & (1u << ALERT_LEAF_RESTART)) != 0u && leaf->last_seq != 0 &&
      record->sequence < leaf->last_seq) {
    alert_notification_payload_t payload = { 0 };
    FillPayloadBase(&payload, leaf, now_ms, now_epoch);
    payload.last_seq = leaf->last_seq;
    payload.limit_milli_c = 0;
    payload.duration_ms = 0;
    payload.hysteresis_milli_c = 0;
    AlertManagerQueueNotification(manager,
                                  NULL,
                                  ALERT_LEAF_RESTART,
                                  ALERT_SEV_INFO,
                                  false,
                                  leaf_id,
                                  &payload,
                                  now_ms);
  }
  const bool cal_valid =
    (record->flags & LOG_RECORD_FLAG_CAL_VALID) != 0u;
  leaf->last_seq = record->sequence;
  leaf->last_temp_milli_c =
    cal_valid ? record->temp_milli_c : record->raw_temp_milli_c;
  leaf->last_rx_uptime_ms = now_ms;
  leaf->last_rx_epoch = (record->timestamp_epoch_sec > 0)
                          ? record->timestamp_epoch_sec
                          : ((now_epoch > 0) ? now_epoch : -1);
  leaf->online = true;
  leaf->last_online_ms = now_ms;
}

/**
 * @brief Execute AlertManagerOnLeafOnline.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @param online Parameter online.
 * @param now_ms Parameter now_ms.
 */
void
AlertManagerOnLeafOnline(alert_manager_t* manager,
                         uint64_t leaf_id,
                         bool online,
                         int64_t now_ms)
{
  if (manager == NULL) {
    return;
  }
  leaf_id = ResolveLeafId(manager, leaf_id);
  alert_leaf_state_t* leaf = FindOrAllocateLeaf(manager, leaf_id);
  if (leaf == NULL) {
    return;
  }
  leaf->online = online;
  if (online) {
    leaf->last_online_ms = now_ms;
  }
}

/**
 * @brief Execute AlertManagerTick.
 * @param manager Parameter manager.
 * @param now_ms Parameter now_ms.
 * @param now_epoch Parameter now_epoch.
 */
void
AlertManagerTick(alert_manager_t* manager, int64_t now_ms, int64_t now_epoch)
{
  if (manager == NULL) {
    return;
  }
  RefreshMeshOnline(manager, now_ms);

  for (size_t i = 0; i < ALERT_MAX_LEAVES; ++i) {
    alert_leaf_state_t* leaf = &manager->leaves[i];
    if (!leaf->in_use) {
      continue;
    }
    const uint32_t mask = EffectiveEnableMask(manager, leaf->leaf_id);
    int32_t high_limit = 0;
    int32_t low_limit = 0;
    (void)GetLimits(manager, leaf->leaf_id, &high_limit, &low_limit);

    if ((mask & (1u << ALERT_TEMP_HIGH)) != 0u) {
      bool high_active = false;
      alert_state_t* high_state = &manager->states[i][ALERT_TEMP_HIGH];
      if (high_state->active) {
        if (leaf->last_temp_milli_c >=
            (high_limit - manager->config.hysteresis_milli_c)) {
          high_active = true;
        } else {
          leaf->high_hold_start_ms = 0;
        }
      } else if (leaf->last_temp_milli_c >= high_limit) {
        if (leaf->high_hold_start_ms == 0) {
          leaf->high_hold_start_ms = now_ms;
        }
        if ((now_ms - leaf->high_hold_start_ms) >=
            (int64_t)manager->config.hold_ms) {
          high_active = true;
        }
      } else {
        leaf->high_hold_start_ms = 0;
      }
      alert_notification_payload_t payload = { 0 };
      FillPayloadBase(&payload, leaf, now_ms, now_epoch);
      payload.limit_milli_c = high_limit;
      payload.hysteresis_milli_c = manager->config.hysteresis_milli_c;
      ProcessAlert(manager,
                   i,
                   ALERT_TEMP_HIGH,
                   ALERT_SEV_WARN,
                   high_active,
                   &payload,
                   now_ms);
    }

    if ((mask & (1u << ALERT_TEMP_LOW)) != 0u) {
      bool low_active = false;
      alert_state_t* low_state = &manager->states[i][ALERT_TEMP_LOW];
      if (low_state->active) {
        if (leaf->last_temp_milli_c <=
            (low_limit + manager->config.hysteresis_milli_c)) {
          low_active = true;
        } else {
          leaf->low_hold_start_ms = 0;
        }
      } else if (leaf->last_temp_milli_c <= low_limit) {
        if (leaf->low_hold_start_ms == 0) {
          leaf->low_hold_start_ms = now_ms;
        }
        if ((now_ms - leaf->low_hold_start_ms) >=
            (int64_t)manager->config.hold_ms) {
          low_active = true;
        }
      } else {
        leaf->low_hold_start_ms = 0;
      }
      alert_notification_payload_t payload = { 0 };
      FillPayloadBase(&payload, leaf, now_ms, now_epoch);
      payload.limit_milli_c = low_limit;
      payload.hysteresis_milli_c = manager->config.hysteresis_milli_c;
      ProcessAlert(manager,
                   i,
                   ALERT_TEMP_LOW,
                   ALERT_SEV_WARN,
                   low_active,
                   &payload,
                   now_ms);
    }

    if ((mask & (1u << ALERT_MISSING_RECORDS)) != 0u &&
        leaf->last_rx_uptime_ms > 0) {
      const uint32_t gap = (uint32_t)(now_ms - leaf->last_rx_uptime_ms);
      const bool missing_active = gap >= manager->config.missing_gap_ms;
      alert_notification_payload_t payload = { 0 };
      FillPayloadBase(&payload, leaf, now_ms, now_epoch);
      payload.duration_ms = gap;
      payload.limit_milli_c = (int32_t)manager->config.missing_gap_ms;
      ProcessAlert(manager,
                   i,
                   ALERT_MISSING_RECORDS,
                   ALERT_SEV_WARN,
                   missing_active,
                   &payload,
                   now_ms);
    }

    if ((mask & (1u << ALERT_LEAF_OFFLINE)) != 0u && leaf->last_online_ms > 0) {
      const uint32_t offline_ms = (uint32_t)(now_ms - leaf->last_online_ms);
      const bool offline_active =
        (!leaf->online && offline_ms >= manager->config.offline_ms);
      alert_notification_payload_t payload = { 0 };
      FillPayloadBase(&payload, leaf, now_ms, now_epoch);
      payload.duration_ms = offline_ms;
      payload.limit_milli_c = (int32_t)manager->config.offline_ms;
      ProcessAlert(manager,
                   i,
                   ALERT_LEAF_OFFLINE,
                   ALERT_SEV_CRIT,
                   offline_active,
                   &payload,
                   now_ms);
    }
  }
}

/**
 * @brief Execute AlertManagerEnableType.
 * @param manager Parameter manager.
 * @param type Parameter type.
 * @param enabled Parameter enabled.
 * @param leaf_id Parameter leaf_id.
 * @param per_leaf Parameter per_leaf.
 * @return Return the function result.
 */
bool
AlertManagerEnableType(alert_manager_t* manager,
                       alert_type_t type,
                       bool enabled,
                       uint64_t leaf_id,
                       bool per_leaf)
{
  if (manager == NULL) {
    return false;
  }
  if (per_leaf) {
    alert_leaf_config_t* cfg = GetOrCreateLeafConfig(manager, leaf_id);
    if (cfg == NULL) {
      return false;
    }
    if (!cfg->has_enable_mask) {
      cfg->enable_mask = manager->config.enable_mask;
    }
    cfg->has_enable_mask = true;
    if (enabled) {
      cfg->enable_mask |= (1u << type);
    } else {
      cfg->enable_mask &= ~(1u << type);
    }
  } else {
    if (enabled) {
      manager->config.enable_mask |= (1u << type);
    } else {
      manager->config.enable_mask &= ~(1u << type);
    }
  }
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetDefaultLimit.
 * @param manager Parameter manager.
 * @param is_high Parameter is_high.
 * @param limit_milli_c Parameter limit_milli_c.
 * @return Return the function result.
 */
bool
AlertManagerSetDefaultLimit(alert_manager_t* manager,
                            bool is_high,
                            int32_t limit_milli_c)
{
  if (manager == NULL) {
    return false;
  }
  if (is_high && limit_milli_c <= manager->config.default_low_milli_c) {
    return false;
  }
  if (!is_high && limit_milli_c >= manager->config.default_high_milli_c) {
    return false;
  }
  if (is_high) {
    manager->config.default_high_milli_c = limit_milli_c;
  } else {
    manager->config.default_low_milli_c = limit_milli_c;
  }
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetLeafLimit.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @param is_high Parameter is_high.
 * @param limit_milli_c Parameter limit_milli_c.
 * @return Return the function result.
 */
bool
AlertManagerSetLeafLimit(alert_manager_t* manager,
                         uint64_t leaf_id,
                         bool is_high,
                         int32_t limit_milli_c)
{
  if (manager == NULL) {
    return false;
  }
  alert_leaf_config_t* cfg = GetOrCreateLeafConfig(manager, leaf_id);
  if (cfg == NULL) {
    return false;
  }
  int32_t high = cfg->has_limits ? cfg->high_limit_milli_c
                                 : manager->config.default_high_milli_c;
  int32_t low = cfg->has_limits ? cfg->low_limit_milli_c
                                : manager->config.default_low_milli_c;
  if (is_high) {
    high = limit_milli_c;
  } else {
    low = limit_milli_c;
  }
  if (high <= low) {
    return false;
  }
  cfg->has_limits = true;
  if (is_high) {
    cfg->high_limit_milli_c = limit_milli_c;
  } else {
    cfg->low_limit_milli_c = limit_milli_c;
  }
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetMissingGap.
 * @param manager Parameter manager.
 * @param gap_ms Parameter gap_ms.
 * @return Return the function result.
 */
bool
AlertManagerSetMissingGap(alert_manager_t* manager, uint32_t gap_ms)
{
  if (manager == NULL || gap_ms == 0) {
    return false;
  }
  manager->config.missing_gap_ms = gap_ms;
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetOfflineMs.
 * @param manager Parameter manager.
 * @param offline_ms Parameter offline_ms.
 * @return Return the function result.
 */
bool
AlertManagerSetOfflineMs(alert_manager_t* manager, uint32_t offline_ms)
{
  if (manager == NULL || offline_ms == 0) {
    return false;
  }
  manager->config.offline_ms = offline_ms;
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetHoldMs.
 * @param manager Parameter manager.
 * @param hold_ms Parameter hold_ms.
 * @return Return the function result.
 */
bool
AlertManagerSetHoldMs(alert_manager_t* manager, uint32_t hold_ms)
{
  if (manager == NULL || hold_ms == 0) {
    return false;
  }
  manager->config.hold_ms = hold_ms;
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetHysteresis.
 * @param manager Parameter manager.
 * @param hysteresis_milli_c Parameter hysteresis_milli_c.
 * @return Return the function result.
 */
bool
AlertManagerSetHysteresis(alert_manager_t* manager, int32_t hysteresis_milli_c)
{
  if (manager == NULL || hysteresis_milli_c < 0) {
    return false;
  }
  manager->config.hysteresis_milli_c = hysteresis_milli_c;
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetRateLimit.
 * @param manager Parameter manager.
 * @param per_key_ms Parameter per_key_ms.
 * @param per_minute Parameter per_minute.
 * @return Return the function result.
 */
bool
AlertManagerSetRateLimit(alert_manager_t* manager,
                         uint32_t per_key_ms,
                         uint32_t per_minute)
{
  if (manager == NULL) {
    return false;
  }
  manager->config.per_key_cooldown_ms = per_key_ms;
  manager->config.global_max_per_minute = per_minute;
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetNtfyUrl.
 * @param manager Parameter manager.
 * @param url Parameter url.
 * @return Return the function result.
 */
bool
AlertManagerSetNtfyUrl(alert_manager_t* manager, const char* url)
{
  if (manager == NULL || url == NULL) {
    return false;
  }
  snprintf(
    manager->config.ntfy_url, sizeof(manager->config.ntfy_url), "%s", url);
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetNtfyTopic.
 * @param manager Parameter manager.
 * @param topic Parameter topic.
 * @return Return the function result.
 */
bool
AlertManagerSetNtfyTopic(alert_manager_t* manager, const char* topic)
{
  if (manager == NULL || topic == NULL) {
    return false;
  }
  snprintf(manager->config.ntfy_topic,
           sizeof(manager->config.ntfy_topic),
           "%s",
           topic);
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerSetNtfyToken.
 * @param manager Parameter manager.
 * @param token Parameter token.
 * @return Return the function result.
 */
bool
AlertManagerSetNtfyToken(alert_manager_t* manager, const char* token)
{
  if (manager == NULL || token == NULL) {
    return false;
  }
  snprintf(manager->config.ntfy_token,
           sizeof(manager->config.ntfy_token),
           "%s",
           token);
  return AlertManagerSaveConfig(manager) == ESP_OK;
}

/**
 * @brief Execute AlertManagerClear.
 * @param manager Parameter manager.
 * @param type Parameter type.
 * @param leaf_id Parameter leaf_id.
 * @param all_leaves Parameter all_leaves.
 */
void
AlertManagerClear(alert_manager_t* manager,
                  alert_type_t type,
                  uint64_t leaf_id,
                  bool all_leaves)
{
  if (manager == NULL) {
    return;
  }
  for (size_t i = 0; i < ALERT_MAX_LEAVES; ++i) {
    if (!manager->leaves[i].in_use) {
      continue;
    }
    if (!all_leaves && manager->leaves[i].leaf_id != leaf_id) {
      continue;
    }
    if (type == ALERT_TYPE_COUNT) {
      for (size_t t = 0; t < ALERT_TYPE_COUNT; ++t) {
        manager->states[i][t].active = false;
      }
    } else {
      manager->states[i][type].active = false;
    }
  }
}

/**
 * @brief Execute AlertManagerSendTest.
 * @param manager Parameter manager.
 * @param now_ms Parameter now_ms.
 */
void
AlertManagerSendTest(alert_manager_t* manager, int64_t now_ms)
{
  if (manager == NULL) {
    return;
  }
  const uint64_t leaf_id = ResolveLeafId(manager, 0);
  alert_notification_payload_t payload = { 0 };
  payload.event_uptime_ms = now_ms;
  payload.event_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;

  alert_notification_t note = {
    .type = ALERT_ROOT_RESTART,
    .severity = ALERT_SEV_INFO,
    .resolved = false,
    .leaf_id = leaf_id,
    .payload = payload,
  };
  (void)AlertNtfyEnqueue(&manager->ntfy, &note);
}

/**
 * @brief Execute AlertManagerEmitRootRestart.
 * @param manager Parameter manager.
 * @param now_ms Parameter now_ms.
 */
void
AlertManagerEmitRootRestart(alert_manager_t* manager, int64_t now_ms)
{
  if (manager == NULL) {
    return;
  }
  if ((manager->config.enable_mask & (1u << ALERT_ROOT_RESTART)) == 0u) {
    return;
  }
  const uint64_t leaf_id = ResolveLeafId(manager, 0);
  alert_notification_payload_t payload = { 0 };
  payload.event_uptime_ms = now_ms;
  payload.event_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;

  alert_notification_t note = {
    .type = ALERT_ROOT_RESTART,
    .severity = ALERT_SEV_INFO,
    .resolved = false,
    .leaf_id = leaf_id,
    .payload = payload,
  };
  (void)AlertNtfyEnqueue(&manager->ntfy, &note);
}

/**
 * @brief Execute AlertManagerEmitSystemBoot.
 * @param manager Parameter manager.
 * @param now_ms Parameter now_ms.
 * @param now_epoch Parameter now_epoch.
 */
void
AlertManagerEmitSystemBoot(alert_manager_t* manager,
                           int64_t now_ms,
                           int64_t now_epoch)
{
  if (manager == NULL) {
    return;
  }
  const uint64_t leaf_id = ResolveLeafId(manager, 0);
  const uint32_t mask = EffectiveEnableMask(manager, leaf_id);
  if ((mask & (1u << ALERT_SYSTEM_BOOT)) == 0u) {
    return;
  }
  size_t leaf_index = 0;
  if (!FindOrAllocateLeafIndex(manager, leaf_id, &leaf_index)) {
    return;
  }
  alert_notification_payload_t payload = { 0 };
  FillPayloadBase(&payload, &manager->leaves[leaf_index], now_ms, now_epoch);
  payload.event_code = ALERT_SYSTEM_CODE_BOOT;
  (void)AlertManagerQueueOneShot(manager,
                                 &manager->states[leaf_index][ALERT_SYSTEM_BOOT],
                                 ALERT_SYSTEM_BOOT,
                                 ALERT_SEV_INFO,
                                 leaf_id,
                                 &payload,
                                 now_ms);
}

/**
 * @brief Execute AlertManagerEmitSystemMode.
 * @param manager Parameter manager.
 * @param mode_code Parameter mode_code.
 * @param now_ms Parameter now_ms.
 * @param now_epoch Parameter now_epoch.
 */
void
AlertManagerEmitSystemMode(alert_manager_t* manager,
                           alert_system_code_t mode_code,
                           int64_t now_ms,
                           int64_t now_epoch)
{
  if (manager == NULL) {
    return;
  }
  const uint64_t leaf_id = ResolveLeafId(manager, 0);
  const uint32_t mask = EffectiveEnableMask(manager, leaf_id);
  if ((mask & (1u << ALERT_SYSTEM_MODE)) == 0u) {
    return;
  }
  size_t leaf_index = 0;
  if (!FindOrAllocateLeafIndex(manager, leaf_id, &leaf_index)) {
    return;
  }
  alert_notification_payload_t payload = { 0 };
  FillPayloadBase(&payload, &manager->leaves[leaf_index], now_ms, now_epoch);
  payload.event_code = mode_code;
  (void)AlertManagerQueueOneShot(manager,
                                 &manager->states[leaf_index][ALERT_SYSTEM_MODE],
                                 ALERT_SYSTEM_MODE,
                                 ALERT_SEV_INFO,
                                 leaf_id,
                                 &payload,
                                 now_ms);
}

/**
 * @brief Execute AlertManagerProcessSystemError.
 * @param manager Parameter manager.
 * @param error_code Parameter error_code.
 * @param active Parameter active.
 * @param now_ms Parameter now_ms.
 * @param now_epoch Parameter now_epoch.
 */
void
AlertManagerProcessSystemError(alert_manager_t* manager,
                               alert_system_code_t error_code,
                               bool active,
                               int64_t now_ms,
                               int64_t now_epoch)
{
  if (manager == NULL) {
    return;
  }
  const uint64_t leaf_id = ResolveLeafId(manager, 0);
  const uint32_t mask = EffectiveEnableMask(manager, leaf_id);
  if ((mask & (1u << ALERT_SYSTEM_ERROR)) == 0u) {
    return;
  }
  size_t leaf_index = 0;
  if (!FindOrAllocateLeafIndex(manager, leaf_id, &leaf_index)) {
    return;
  }
  alert_notification_payload_t payload = { 0 };
  FillPayloadBase(&payload, &manager->leaves[leaf_index], now_ms, now_epoch);
  payload.event_code = error_code;
  ProcessAlert(manager,
               leaf_index,
               ALERT_SYSTEM_ERROR,
               ALERT_SEV_CRIT,
               active,
               &payload,
               now_ms);
}

/**
 * @brief Execute AlertManagerCopyLeaves.
 * @param manager Parameter manager.
 * @param out Parameter out.
 * @param max_items Parameter max_items.
 * @return Return the function result.
 */
size_t
AlertManagerCopyLeaves(const alert_manager_t* manager,
                       alert_leaf_state_t* out,
                       size_t max_items)
{
  if (manager == NULL || out == NULL) {
    return 0;
  }
  size_t count = 0;
  for (size_t i = 0; i < ALERT_MAX_LEAVES && count < max_items; ++i) {
    if (!manager->leaves[i].in_use) {
      continue;
    }
    out[count++] = manager->leaves[i];
  }
  return count;
}

/**
 * @brief Execute AlertManagerCopyActiveAlerts.
 * @param manager Parameter manager.
 * @param out_states Parameter out_states.
 * @param out_types Parameter out_types.
 * @param out_leaf_ids Parameter out_leaf_ids.
 * @param max_items Parameter max_items.
 * @return Return the function result.
 */
size_t
AlertManagerCopyActiveAlerts(const alert_manager_t* manager,
                             alert_state_t* out_states,
                             alert_type_t* out_types,
                             uint64_t* out_leaf_ids,
                             size_t max_items)
{
  if (manager == NULL || out_states == NULL || out_types == NULL ||
      out_leaf_ids == NULL) {
    return 0;
  }
  size_t count = 0;
  for (size_t i = 0; i < ALERT_MAX_LEAVES; ++i) {
    if (!manager->leaves[i].in_use) {
      continue;
    }
    for (size_t t = 0; t < ALERT_TYPE_COUNT && count < max_items; ++t) {
      if (manager->states[i][t].active) {
        out_states[count] = manager->states[i][t];
        out_types[count] = (alert_type_t)t;
        out_leaf_ids[count] = manager->leaves[i].leaf_id;
        count++;
      }
    }
  }
  return count;
}

/**
 * @brief Execute AlertManagerFormatLeafId.
 * @param leaf_id Parameter leaf_id.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
void
AlertManagerFormatLeafId(uint64_t leaf_id, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  uint8_t mac[6];
  for (int i = 5; i >= 0; --i) {
    mac[i] = (uint8_t)(leaf_id & 0xFFu);
    leaf_id >>= 8;
  }
  snprintf(out,
           out_size,
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],
           mac[1],
           mac[2],
           mac[3],
           mac[4],
           mac[5]);
}

/**
 * @brief Execute AlertManagerMonitorTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the AlertManagerMonitorTask task.
 */
void
AlertManagerMonitorTask(void* context)
{
  alert_task_context_t* ctx = (alert_task_context_t*)context;
  if (ctx == NULL || ctx->manager == NULL) {
    vTaskDelete(NULL);
    return;
  }
  while (!*ctx->stop_requested) {
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t now_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    AlertManagerTick(ctx->manager, now_ms, now_epoch);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  if (ctx->task_handle != NULL) {
    *ctx->task_handle = NULL;
  }
  vTaskDelete(NULL);
}

static int64_t
ResolveNtfyMinIntervalMs(const alert_manager_t* manager)
{
  int64_t min_interval_ms = 15000;
  if (manager != NULL && manager->config.global_max_per_minute > 0) {
    const int64_t per_minute_interval_ms =
      60000 / (int64_t)manager->config.global_max_per_minute;
    if (per_minute_interval_ms > min_interval_ms) {
      min_interval_ms = per_minute_interval_ms;
    }
  }
  return min_interval_ms;
}

static bool
AlertPayloadMatches(const alert_notification_payload_t* left,
                    const alert_notification_payload_t* right)
{
  if (left == NULL || right == NULL) {
    return false;
  }
  return left->current_temp_milli_c == right->current_temp_milli_c &&
         left->limit_milli_c == right->limit_milli_c &&
         left->hysteresis_milli_c == right->hysteresis_milli_c &&
         left->duration_ms == right->duration_ms &&
         left->last_seq == right->last_seq &&
         left->last_rx_epoch == right->last_rx_epoch &&
         left->last_rx_uptime_ms == right->last_rx_uptime_ms &&
         left->event_epoch == right->event_epoch &&
         left->event_uptime_ms == right->event_uptime_ms &&
         left->event_code == right->event_code &&
         left->transitions == right->transitions;
}

static bool
AlertNotificationMatches(const alert_notification_t* left,
                         const alert_notification_t* right)
{
  if (left == NULL || right == NULL) {
    return false;
  }
  return left->type == right->type && left->severity == right->severity &&
         left->resolved == right->resolved && left->leaf_id == right->leaf_id &&
         AlertPayloadMatches(&left->payload, &right->payload);
}

static int64_t
ResolveNtfyCooldownMs(const alert_ntfy_t* ntfy, int retry_after_seconds)
{
  if (retry_after_seconds > 0) {
    int64_t retry_ms = (int64_t)retry_after_seconds * 1000;
    if (retry_ms > kNtfyRateLimitMaxCooldownMs) {
      retry_ms = kNtfyRateLimitMaxCooldownMs;
    }
    return retry_ms;
  }

  uint32_t attempts = 1;
  if (ntfy != NULL && ntfy->rate_limited_count > 0) {
    attempts = ntfy->rate_limited_count;
  }

  int64_t backoff_ms = kNtfyRateLimitBaseCooldownMs;
  while (attempts > 1 && backoff_ms < kNtfyRateLimitMaxCooldownMs) {
    backoff_ms *= 2;
    attempts--;
  }
  if (backoff_ms > kNtfyRateLimitMaxCooldownMs) {
    backoff_ms = kNtfyRateLimitMaxCooldownMs;
  }
  return backoff_ms;
}

static void
BuildNtfySuppressedSummary(alert_manager_t* manager,
                           alert_notification_t* note,
                           uint32_t suppressed_count,
                           int64_t now_ms,
                           int64_t now_epoch)
{
  if (manager == NULL || note == NULL) {
    return;
  }
  memset(note, 0, sizeof(*note));
  note->type = ALERT_SYSTEM_ERROR;
  note->severity = ALERT_SEV_WARN;
  note->resolved = false;
  note->leaf_id = manager->local_leaf_id;
  note->payload.duration_ms = suppressed_count;
  note->payload.event_code = ALERT_SYSTEM_CODE_ERROR_NTFY_RATE_LIMIT;
  note->payload.event_epoch = now_epoch;
  note->payload.event_uptime_ms = now_ms;
}

/**
 * @brief Execute AlertManagerSenderTask.
 * @param context Parameter context.
 * @note FreeRTOS task entry for the AlertManagerSenderTask task.
 */
void
AlertManagerSenderTask(void* context)
{
  alert_task_context_t* ctx = (alert_task_context_t*)context;
  if (ctx == NULL || ctx->manager == NULL) {
    vTaskDelete(NULL);
    return;
  }

  int64_t last_attempt_ms = 0;
  int64_t last_success_ms = 0;

  while (!*ctx->stop_requested) {
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t now_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    if (ctx->manager->ntfy.cooldown_until_ms > now_ms) {
      alert_notification_t pending;
      while (xQueueReceive(ctx->manager->ntfy.queue, &pending, 0) == pdTRUE) {
        ctx->manager->ntfy.pending_note = pending;
        ctx->manager->ntfy.pending_valid = true;
        ctx->manager->ntfy.suppressed_count++;
      }
      int64_t remaining_ms =
        ctx->manager->ntfy.cooldown_until_ms - now_ms;
      if (remaining_ms > 0) {
        (void)ulTaskNotifyTake(pdTRUE,
                               pdMS_TO_TICKS((uint32_t)remaining_ms));
        if (*ctx->stop_requested) {
          break;
        }
      }
      continue;
    }

    if (kNtfySendSuppressedSummary &&
        ctx->manager->ntfy.pending_valid &&
        ctx->manager->ntfy.suppressed_count > 0) {
      const int64_t min_interval_ms =
        ResolveNtfyMinIntervalMs(ctx->manager);
      if (last_attempt_ms > 0 &&
          (now_ms - last_attempt_ms) < min_interval_ms) {
        int64_t remaining_ms = min_interval_ms - (now_ms - last_attempt_ms);
        if (remaining_ms > 0) {
          (void)ulTaskNotifyTake(pdTRUE,
                                 pdMS_TO_TICKS((uint32_t)remaining_ms));
          if (*ctx->stop_requested) {
            break;
          }
        }
      }

      now_ms = esp_timer_get_time() / 1000;
      now_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
      alert_notification_t summary;
      BuildNtfySuppressedSummary(ctx->manager,
                                 &summary,
                                 ctx->manager->ntfy.suppressed_count,
                                 now_ms,
                                 now_epoch);
      alert_ntfy_config_t cfg = {
        .url = ctx->manager->config.ntfy_url,
        .topic = ctx->manager->config.ntfy_topic,
        .token = ctx->manager->config.ntfy_token,
        .root_id = ctx->manager->root_id_string,
        .http_timeout_ms = 0,
      };

      int status = 0;
      int retry_after_seconds = -1;
      esp_err_t err = ESP_OK;
      alert_ntfy_result_t result = AlertNtfySend(&ctx->manager->ntfy,
                                                 &cfg,
                                                 &summary,
                                                 &retry_after_seconds,
                                                 &status,
                                                 &err);
      last_attempt_ms = now_ms;
      if (result == ALERT_NTFY_OK) {
        ctx->manager->ntfy.send_success++;
        ctx->manager->ntfy.last_http_status = status;
        ctx->manager->ntfy.last_err = err;
        ctx->manager->ntfy.backoff_ms = 0;
        last_success_ms = last_attempt_ms;
        ctx->manager->ntfy.last_sent = summary;
        ctx->manager->ntfy.last_sent_ms = last_success_ms;
        ctx->manager->ntfy.last_sent_valid = true;
        ctx->manager->ntfy.suppressed_count = 0;
        ctx->manager->ntfy.rate_limited_count = 0;
        continue;
      }
      ctx->manager->ntfy.send_fail++;
      ctx->manager->ntfy.last_http_status = status;
      ctx->manager->ntfy.last_err = err;
      if (status == 429) {
        ctx->manager->ntfy.rate_limited_count++;
        const int64_t backoff_ms = ResolveNtfyCooldownMs(
          &ctx->manager->ntfy, retry_after_seconds);
        ctx->manager->ntfy.cooldown_until_ms = last_attempt_ms + backoff_ms;
        continue;
      }
      ctx->manager->ntfy.rate_limited_count = 0;
      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
      if (*ctx->stop_requested) {
        break;
      }
      continue;
    }

    alert_notification_t note;
    bool have_note = false;
    if (ctx->manager->ntfy.pending_valid) {
      note = ctx->manager->ntfy.pending_note;
      ctx->manager->ntfy.pending_valid = false;
      have_note = true;
    } else if (xQueueReceive(ctx->manager->ntfy.queue,
                             &note,
                             pdMS_TO_TICKS(1000)) == pdTRUE) {
      have_note = true;
    }
    if (!have_note) {
      continue;
    }

    int64_t min_interval_ms = ResolveNtfyMinIntervalMs(ctx->manager);

    now_ms = esp_timer_get_time() / 1000;
    if (last_attempt_ms > 0 &&
        (now_ms - last_attempt_ms) < min_interval_ms) {
      alert_notification_t newest = note;
      while (xQueueReceive(ctx->manager->ntfy.queue, &note, 0) == pdTRUE) {
        newest = note;
      }
      note = newest;
      int64_t remaining_ms = min_interval_ms - (now_ms - last_attempt_ms);
      if (remaining_ms > 0) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS((uint32_t)remaining_ms));
        if (*ctx->stop_requested) {
          break;
        }
      }
    }

    now_ms = esp_timer_get_time() / 1000;
    if (ctx->manager->ntfy.cooldown_until_ms > now_ms) {
      ctx->manager->ntfy.pending_note = note;
      ctx->manager->ntfy.pending_valid = true;
      ctx->manager->ntfy.suppressed_count++;
      while (xQueueReceive(ctx->manager->ntfy.queue, &note, 0) == pdTRUE) {
        ctx->manager->ntfy.pending_note = note;
        ctx->manager->ntfy.suppressed_count++;
      }
      continue;
    }

    if (ctx->manager->ntfy.last_sent_valid &&
        (now_ms - ctx->manager->ntfy.last_sent_ms) <=
          kNtfyDedupeWindowMs &&
        AlertNotificationMatches(&note, &ctx->manager->ntfy.last_sent)) {
      continue;
    }

    alert_ntfy_config_t cfg = {
      .url = ctx->manager->config.ntfy_url,
      .topic = ctx->manager->config.ntfy_topic,
      .token = ctx->manager->config.ntfy_token,
      .root_id = ctx->manager->root_id_string,
      .http_timeout_ms = 0,
    };

    int status = 0;
    int retry_after_seconds = -1;
    esp_err_t err = ESP_OK;
    alert_ntfy_result_t result = AlertNtfySend(&ctx->manager->ntfy,
                                               &cfg,
                                               &note,
                                               &retry_after_seconds,
                                               &status,
                                               &err);
    last_attempt_ms = now_ms;

    if (result == ALERT_NTFY_OK) {
      ctx->manager->ntfy.send_success++;
      ctx->manager->ntfy.last_http_status = status;
      ctx->manager->ntfy.last_err = err;
      ctx->manager->ntfy.backoff_ms = 0;
      last_success_ms = last_attempt_ms;
      ctx->manager->ntfy.last_sent = note;
      ctx->manager->ntfy.last_sent_ms = last_success_ms;
      ctx->manager->ntfy.last_sent_valid = true;
      ctx->manager->ntfy.cooldown_until_ms = 0;
      ctx->manager->ntfy.suppressed_count = 0;
      ctx->manager->ntfy.rate_limited_count = 0;
    } else if (result == ALERT_NTFY_SKIPPED) {
      ctx->manager->ntfy.last_err = err;
    } else {
      ctx->manager->ntfy.send_fail++;
      ctx->manager->ntfy.last_http_status = status;
      ctx->manager->ntfy.last_err = err;
      if (status == 429) {
        ctx->manager->ntfy.rate_limited_count++;
        const int64_t backoff_ms = ResolveNtfyCooldownMs(
          &ctx->manager->ntfy, retry_after_seconds);
        ctx->manager->ntfy.cooldown_until_ms = now_ms + backoff_ms;
        ctx->manager->ntfy.pending_note = note;
        ctx->manager->ntfy.pending_valid = true;
        ctx->manager->ntfy.suppressed_count++;
        continue;
      }
      ctx->manager->ntfy.rate_limited_count = 0;
      ctx->manager->ntfy.backoff_ms = (ctx->manager->ntfy.backoff_ms == 0)
                                        ? 1000
                                        : (ctx->manager->ntfy.backoff_ms * 2);
      if (ctx->manager->ntfy.backoff_ms > kNtfyFailureMaxBackoffMs) {
        ctx->manager->ntfy.backoff_ms = kNtfyFailureMaxBackoffMs;
      }
      (void)ulTaskNotifyTake(pdTRUE,
                             pdMS_TO_TICKS(ctx->manager->ntfy.backoff_ms));
      if (*ctx->stop_requested) {
        break;
      }
      (void)AlertNtfyEnqueue(&ctx->manager->ntfy, &note);
    }
  }

  if (ctx->task_handle != NULL) {
    *ctx->task_handle = NULL;
  }
  vTaskDelete(NULL);
}
