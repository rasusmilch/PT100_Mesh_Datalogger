#include "alerts/alert_manager.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mesh_lite.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "time_sync.h"

typedef struct
{
  alert_notification_t note;
  uint32_t count;
} alert_batch_entry_t;

typedef struct
{
  alert_batch_entry_t entries[ALERT_NTFY_QUEUE_LEN];
  size_t entry_count;
  uint32_t total_count;
  int64_t last_event_epoch;
  int64_t last_event_uptime_ms;
} alert_batch_t;

typedef struct alert_ntfy_batch_scratch_s
{
  alert_notification_t notes[ALERT_NTFY_QUEUE_LEN];
  alert_batch_t batch;
  char title[ALERT_NTFY_JOB_TITLE_LEN];
  char body[ALERT_NTFY_JOB_BODY_LEN];
} alert_ntfy_batch_scratch_t;

#if CONFIG_MESH_LITE_NODE_INFO_REPORT
static uint64_t
PackMacToId(const uint8_t mac[6]);
#endif
static uint64_t
ResolveLeafId(const alert_manager_t* manager, uint64_t leaf_id);
static void
ResetMissingGapLearning(alert_leaf_state_t* leaf);
static void
UpdateMissingGapLearning(alert_manager_t* manager,
                         alert_leaf_state_t* leaf,
                         int64_t now_ms);
static alert_leaf_state_t*
FindLeaf(alert_manager_t* manager, uint64_t leaf_id);
static alert_leaf_state_t*
FindOrAllocateLeaf(alert_manager_t* manager, uint64_t leaf_id);
static bool
FindOrAllocateLeafIndex(alert_manager_t* manager,
                        uint64_t leaf_id,
                        size_t* index_out);
static bool
GetLeafConfig(const alert_manager_t* manager,
              uint64_t leaf_id,
              alert_leaf_config_t* out);
static alert_leaf_config_t*
GetOrCreateLeafConfig(alert_manager_t* manager, uint64_t leaf_id);
static uint32_t
EffectiveEnableMask(const alert_manager_t* manager, uint64_t leaf_id);
static void
AlertStateTransition(alert_state_t* state, bool active, int64_t now_ms);
static bool
AlertManagerQueueNotification(alert_manager_t* manager,
                              alert_state_t* state,
                              alert_type_t type,
                              alert_severity_t severity,
                              bool resolved,
                              uint64_t leaf_id,
                              const alert_notification_payload_t* payload,
                              int64_t now_ms);
static bool
AlertManagerQueueOneShot(alert_manager_t* manager,
                         alert_state_t* state,
                         alert_type_t type,
                         alert_severity_t severity,
                         uint64_t leaf_id,
                         const alert_notification_payload_t* payload,
                         int64_t now_ms);
static void
FillPayloadBase(alert_notification_payload_t* payload,
                const alert_leaf_state_t* leaf,
                int64_t now_ms,
                int64_t now_epoch);
static void
ProcessAlert(alert_manager_t* manager,
             size_t leaf_index,
             alert_type_t type,
             alert_severity_t severity,
             bool condition_active,
             alert_notification_payload_t* payload,
             int64_t now_ms);
static bool
GetLimits(const alert_manager_t* manager,
          uint64_t leaf_id,
          int32_t* high_out,
          int32_t* low_out);
static void
RefreshMeshOnline(alert_manager_t* manager, int64_t now_ms);
static void
ApplyDefaults(alert_manager_t* manager);
static int64_t
ResolveNtfyMinIntervalMs(const alert_manager_t* manager);
static uint32_t
ResolveNtfyHttpTimeoutMs(const alert_manager_t* manager);
static void
FormatEpoch(int64_t epoch_seconds, char* out, size_t out_size);
static void
FormatMilliC(int32_t milli_c, char* out, size_t out_size);
static void
FormatBatchWindow(int64_t window_ms, char* out, size_t out_size);
static bool
AlertNotificationKeyMatches(const alert_notification_t* left,
                            const alert_notification_t* right);
static bool
AlertNotificationIsLeafScoped(const alert_notification_t* note);
static void
AlertNotificationDescribe(const alert_manager_t* manager,
                          const alert_notification_t* note,
                          char* out,
                          size_t out_size);
static void
AlertManagerLogNtfyNoteQueued(const alert_manager_t* manager,
                              const alert_notification_t* note);
static void
AlertManagerLogNtfyBodyLines(const char* label, const char* text);
static uint32_t
SaturatingDeltaMs(int64_t now_ms, int64_t then_ms);
static void
LogNonmonotonicDelta(uint64_t leaf_id,
                     int64_t now_ms,
                     int64_t then_ms,
                     uint32_t gap_ms);
static bool
IsLikelySequenceWrap(uint32_t last_seq, uint32_t new_seq);
static bool
ShouldAlertLeafRestart(const alert_leaf_state_t* leaf,
                       const log_record_t* record,
                       const char** reason_out);
static void
LogLeafRestartDetection(const alert_leaf_state_t* leaf,
                        const log_record_t* record,
                        int64_t now_ms,
                        int64_t now_epoch,
                        const char* reason);
static void
AlertManagerBatchInit(alert_batch_t* batch);
static void
AlertManagerBatchAdd(alert_batch_t* batch, const alert_notification_t* note);
static size_t
AlertManagerDrainNtfyQueue(alert_manager_t* manager,
                           alert_notification_t* notes,
                           size_t max_notes,
                           uint32_t wait_ms);
static bool
AlertManagerBuildBatchMessage(const alert_manager_t* manager,
                              const alert_batch_t* batch,
                              uint32_t msg_seq,
                              char* title,
                              size_t title_size,
                              char* body,
                              size_t body_size);
static alert_state_t*
GetSystemErrorState(alert_manager_t* manager, alert_system_code_t error_code);
static int64_t
AlertManagerComputeNextSendMs(const alert_manager_t* manager,
                              int64_t now_ms,
                              int retry_after_seconds);
/**
 * @brief Build a stable ntfy sequence ID for a queued HTTP job.
 * @param boot_nonce Random nonce generated at boot.
 * @param msg_seq Monotonic message sequence number.
 * @param out Output buffer for formatted ID.
 * @param out_size Size of output buffer.
 */
static void
AlertManagerBuildNtfySequenceId(uint32_t boot_nonce,
                                uint32_t msg_seq,
                                char* out,
                                size_t out_size);
static bool
AlertManagerSendBatchedNtfy(alert_manager_t* manager,
                            int64_t now_ms,
                            int64_t now_epoch,
                            uint32_t wait_ms,
                            int64_t* next_attempt_ms);
static void
AlertManagerStartupNtfyReset(alert_manager_t* manager);
static bool
AlertManagerStartupNtfyWindowExpired(const alert_manager_t* manager,
                                     int64_t now_ms);
static const char*
AlertManagerStartupModeLabel(alert_system_code_t mode_code);
static void
AlertManagerStartupNtfyBuildMessage(const alert_manager_t* manager,
                                    char* title,
                                    size_t title_size,
                                    char* body,
                                    size_t body_size);
static bool
AlertManagerStartupNtfyFlush(alert_manager_t* manager,
                             int64_t now_ms,
                             int64_t now_epoch,
                             bool force_flush);
static bool
AppendTextLine(char* body, size_t body_size, size_t* used, const char* text);

static const char* kTag = "alert_mgr";
static const int64_t kNtfyFailureMaxBackoffMs = 300000;
static const uint32_t kAlertConfigVersion = 2;
static const char* kAlertNvsNamespace = "alerts";
static const char* kAlertNvsConfigKey = "config";
static const uint32_t kRestartSeqLowValueThreshold = 1000;
static const uint32_t kRestartSeqHighValueThreshold = 100000;
static const uint64_t kRestartRecordIdDropThreshold = 1000;
static const int64_t kRestartWarnLogMinIntervalMs = 10000;
static const uint32_t kSequenceWrapWindow = 1000;
static const int64_t kStartupNtfyCoalesceWindowMs = 5000;
static int64_t s_last_nonmonotonic_log_ms = 0;
RTC_DATA_ATTR static uint32_t g_ntfy_msg_seq = 0;
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

static uint32_t
SaturatingDeltaMs(int64_t now_ms, int64_t then_ms)
{
  if (then_ms <= 0 || now_ms <= then_ms) {
    return 0;
  }
  const int64_t delta = now_ms - then_ms;
  if (delta > (int64_t)UINT32_MAX) {
    return UINT32_MAX;
  }
  return (uint32_t)delta;
}

/**
 * @brief Reset per-leaf missing-gap learning state.
 * @param leaf Leaf state to reset.
 * @details Clears the sample counter and learned threshold so missing-record
 * detection is suppressed until two fresh samples are observed.
 */
static void
ResetMissingGapLearning(alert_leaf_state_t* leaf)
{
  if (leaf == NULL) {
    return;
  }
  leaf->missing_gap_sample_count = 0;
  leaf->missing_gap_threshold_ms = 0;
}

/**
 * @brief Update per-leaf missing-gap learning from the latest sample.
 * @param manager Alert manager instance.
 * @param leaf Leaf state to update.
 * @param now_ms Current uptime in milliseconds.
 * @details On the first observed sample, only increments the sample counter.
 * On subsequent samples, computes a missing-gap threshold from the most recent
 * interval (1.5x) while enforcing config.missing_gap_ms as a floor.
 */
static void
UpdateMissingGapLearning(alert_manager_t* manager,
                         alert_leaf_state_t* leaf,
                         int64_t now_ms)
{
  if (manager == NULL || leaf == NULL) {
    return;
  }

  if (leaf->missing_gap_sample_count == 0) {
    leaf->missing_gap_sample_count = 1;
    return;
  }

  const uint32_t last_interval_ms =
    SaturatingDeltaMs(now_ms, leaf->last_rx_uptime_ms);
  if (last_interval_ms > 0) {
    const uint32_t computed =
      last_interval_ms + (last_interval_ms / 2);
    leaf->missing_gap_threshold_ms =
      (computed > manager->config.missing_gap_ms)
        ? computed
        : manager->config.missing_gap_ms;
  }

  if (leaf->missing_gap_sample_count < UINT8_MAX) {
    ++leaf->missing_gap_sample_count;
  }
}

static void
LogNonmonotonicDelta(uint64_t leaf_id,
                     int64_t now_ms,
                     int64_t then_ms,
                     uint32_t gap_ms)
{
  if (now_ms >= then_ms) {
    return;
  }
  if ((now_ms - s_last_nonmonotonic_log_ms) < 60000) {
    return;
  }
  s_last_nonmonotonic_log_ms = now_ms;
  char leaf_id_string[32] = "";
  AlertManagerFormatLeafId(leaf_id, leaf_id_string, sizeof(leaf_id_string));
  ESP_LOGW(kTag,
           "non-monotonic leaf time for %s now_ms=%" PRId64 " then_ms=%" PRId64
           " gap=%" PRIu32 "ms (clamped negative delta)",
           leaf_id_string,
           now_ms,
           then_ms,
           gap_ms);
}

static bool
IsLikelySequenceWrap(uint32_t last_seq, uint32_t new_seq)
{
  return last_seq >= (UINT32_MAX - kSequenceWrapWindow) &&
         new_seq <= kSequenceWrapWindow;
}

static bool
ShouldAlertLeafRestart(const alert_leaf_state_t* leaf,
                       const log_record_t* record,
                       const char** reason_out)
{
  if (reason_out != NULL) {
    *reason_out = "unknown";
  }
  if (leaf == NULL || record == NULL) {
    return false;
  }
  if (leaf->last_seq == 0 || record->sequence == 0) {
    return false;
  }
  if (record->sequence >= leaf->last_seq) {
    return false;
  }
  if (IsLikelySequenceWrap(leaf->last_seq, record->sequence)) {
    if (reason_out != NULL) {
      *reason_out = "sequence wrap";
    }
    return false;
  }

  if (record->record_id != 0 && leaf->last_record_id != 0 &&
      record->record_id < leaf->last_record_id) {
    const uint64_t drop = leaf->last_record_id - record->record_id;
    if (drop >= kRestartRecordIdDropThreshold) {
      if (reason_out != NULL) {
        *reason_out = "record_id regressed";
      }
      return true;
    }
  }

  if (record->sequence <= kRestartSeqLowValueThreshold &&
      leaf->last_seq >= kRestartSeqHighValueThreshold) {
    if (reason_out != NULL) {
      *reason_out = "sequence reset to low value";
    }
    return true;
  }
  return false;
}

static void
LogLeafRestartDetection(const alert_leaf_state_t* leaf,
                        const log_record_t* record,
                        int64_t now_ms,
                        int64_t now_epoch,
                        const char* reason)
{
  if (leaf == NULL || record == NULL) {
    return;
  }
  if (now_ms > 0 && leaf->last_restart_log_ms > 0 &&
      (now_ms - leaf->last_restart_log_ms) < kRestartWarnLogMinIntervalMs) {
    return;
  }
  char leaf_id_string[32] = "";
  AlertManagerFormatLeafId(
    leaf->leaf_id, leaf_id_string, sizeof(leaf_id_string));
  ESP_LOGW(kTag,
           "leaf sequence regression (%s) for %s last_seq=%" PRIu32
           " new_seq=%" PRIu32 " last_record_id=%" PRIu64
           " new_record_id=%" PRIu64 " last_rx_epoch=%" PRId64
           " now_epoch=%" PRId64,
           (reason != NULL) ? reason : "unknown",
           leaf_id_string,
           leaf->last_seq,
           record->sequence,
           leaf->last_record_id,
           record->record_id,
           leaf->last_rx_epoch,
           now_epoch);
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
      manager->leaves[i] = (alert_leaf_state_t){
        .in_use = true,
        .leaf_id = leaf_id,
      };
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
    char describe[256] = "";
    AlertNotificationDescribe(manager, &note, describe, sizeof(describe));
    ESP_LOGW(kTag, "ntfy note enqueue failed: %s", describe);
    if (state != NULL) {
      state->notify_suppressed_count++;
    }
    return false;
  }
  AlertManagerLogNtfyNoteQueued(manager, &note);
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
  if (AlertManagerQueueNotification(
        manager, state, type, severity, false, leaf_id, payload, now_ms)) {
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
  manager->config.ntfy_min_send_interval_ms = 300000;
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
  manager->ntfy_boot_nonce = esp_random();
  ApplyDefaults(manager);
  manager->ntfy_batch_scratch =
    heap_caps_calloc(1,
                     sizeof(*manager->ntfy_batch_scratch),
                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (manager->ntfy_batch_scratch == NULL) {
    ESP_LOGE(kTag, "Failed to allocate ntfy scratch storage in PSRAM");
  }
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
  const char* restart_reason = NULL;
  if ((mask & (1u << ALERT_LEAF_RESTART)) != 0u &&
      ShouldAlertLeafRestart(leaf, record, &restart_reason)) {
    LogLeafRestartDetection(leaf, record, now_ms, now_epoch, restart_reason);
    leaf->last_restart_log_ms = now_ms;
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
    ResetMissingGapLearning(leaf);
  }
  const bool cal_valid = (record->flags & LOG_RECORD_FLAG_CAL_VALID) != 0u;
  if (record->sequence != 0u) {
    if (leaf->last_seq == 0 || record->sequence > leaf->last_seq ||
        IsLikelySequenceWrap(leaf->last_seq, record->sequence)) {
      leaf->last_seq = record->sequence;
    }
  }
  if (record->record_id != 0u && record->record_id > leaf->last_record_id) {
    leaf->last_record_id = record->record_id;
  }
  leaf->last_temp_milli_c =
    cal_valid ? record->temp_milli_c : record->raw_temp_milli_c;
  UpdateMissingGapLearning(manager, leaf, now_ms);
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
 * @brief Reset local-leaf alert baselines for a new logging session.
 * @param manager Parameter manager.
 * @param now_ms Parameter now_ms.
 * @param now_epoch Parameter now_epoch.
 */
void
AlertManagerOnLoggingSessionStart(alert_manager_t* manager,
                                  int64_t now_ms,
                                  int64_t now_epoch)
{
  if (manager == NULL) {
    return;
  }

  size_t leaf_index = 0;
  if (!FindOrAllocateLeafIndex(
        manager, ResolveLeafId(manager, manager->local_leaf_id), &leaf_index)) {
    return;
  }

  alert_leaf_state_t* leaf = &manager->leaves[leaf_index];
  leaf->last_rx_uptime_ms = now_ms;
  leaf->last_rx_epoch = (now_epoch > 0) ? now_epoch : -1;
  leaf->last_seq = 0;
  leaf->last_record_id = 0;
  leaf->last_restart_log_ms = 0;
  leaf->high_hold_start_ms = 0;
  leaf->low_hold_start_ms = 0;
  ResetMissingGapLearning(leaf);

  memset(manager->states[leaf_index], 0, sizeof(manager->states[leaf_index]));

  char leaf_id_string[32] = "";
  AlertManagerFormatLeafId(
    leaf->leaf_id, leaf_id_string, sizeof(leaf_id_string));
  ESP_LOGI(kTag,
           "logging session start: reset alert baselines (leaf=%s)",
           leaf_id_string);
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
  AlertManagerStartupNtfyTick(manager, now_ms, now_epoch);
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
        leaf->last_rx_uptime_ms > 0 && leaf->missing_gap_sample_count >= 2 &&
        leaf->missing_gap_threshold_ms > 0) {
      const uint32_t gap = SaturatingDeltaMs(now_ms, leaf->last_rx_uptime_ms);
      const uint32_t threshold_ms =
        (leaf->missing_gap_threshold_ms > manager->config.missing_gap_ms)
          ? leaf->missing_gap_threshold_ms
          : manager->config.missing_gap_ms;
      LogNonmonotonicDelta(leaf->leaf_id, now_ms, leaf->last_rx_uptime_ms, gap);
      const bool missing_active = gap >= threshold_ms;
      alert_notification_payload_t payload = { 0 };
      FillPayloadBase(&payload, leaf, now_ms, now_epoch);
      payload.duration_ms = gap;
      payload.limit_milli_c =
        (threshold_ms > (uint32_t)INT32_MAX) ? INT32_MAX : (int32_t)threshold_ms;
      ProcessAlert(manager,
                   i,
                   ALERT_MISSING_RECORDS,
                   ALERT_SEV_WARN,
                   missing_active,
                   &payload,
                   now_ms);
    }

    if ((mask & (1u << ALERT_LEAF_OFFLINE)) != 0u && leaf->last_online_ms > 0) {
      const uint32_t offline_ms =
        SaturatingDeltaMs(now_ms, leaf->last_online_ms);
      LogNonmonotonicDelta(
        leaf->leaf_id, now_ms, leaf->last_online_ms, offline_ms);
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
 * @brief Execute AlertManagerSetNtfyMinIntervalMs.
 * @param manager Parameter manager.
 * @param min_interval_ms Parameter min_interval_ms.
 * @return Return the function result.
 */
bool
AlertManagerSetNtfyMinIntervalMs(alert_manager_t* manager,
                                 uint32_t min_interval_ms)
{
  if (manager == NULL || min_interval_ms == 0) {
    return false;
  }
  manager->config.ntfy_min_send_interval_ms = min_interval_ms;
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

  char normalized_url[sizeof(manager->config.ntfy_url)] = { 0 };
  if (!AlertNtfySanitizeBaseUrl(
        url, normalized_url, sizeof(normalized_url), NULL)) {
    return false;
  }

  strlcpy(
    manager->config.ntfy_url, normalized_url, sizeof(manager->config.ntfy_url));
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
bool
AlertManagerSendTest(alert_manager_t* manager, int64_t now_ms)
{
  if (manager == NULL) {
    return false;
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
  const bool queued = AlertNtfyEnqueue(&manager->ntfy, &note);
  ESP_LOGI(kTag,
           "ntfy test uses standard note path: type=root_restart "
           "resolved=0 leaf=%" PRIu64 " queued=%u",
           leaf_id,
           queued ? 1u : 0u);
  return queued;
}

/**
 * @brief Queue a lightweight direct ntfy transport/TLS probe message.
 * @param manager Parameter manager.
 * @param now_ms Parameter now_ms.
 * @return True when the direct ntfy probe job was queued.
 */
bool
AlertManagerSendDirectNtfyTest(alert_manager_t* manager, int64_t now_ms)
{
  if (manager == NULL) {
    return false;
  }
  if (!AlertManagerIsConfigured(manager)) {
    return false;
  }

  alert_ntfy_job_t job = { 0 };
  snprintf(job.url, sizeof(job.url), "%s", manager->config.ntfy_url);
  snprintf(job.topic, sizeof(job.topic), "%s", manager->config.ntfy_topic);
  snprintf(job.token, sizeof(job.token), "%s", manager->config.ntfy_token);
  snprintf(job.root_id,
           sizeof(job.root_id),
           "%s",
           manager->root_id_string != NULL ? manager->root_id_string : "");
  snprintf(job.title, sizeof(job.title), "%s", "PT100 ntfy test");
  snprintf(job.body, sizeof(job.body), "%s", "ntfy test");
  snprintf(
    job.sequence_id, sizeof(job.sequence_id), "ntfy-test-%" PRId64, now_ms);
  job.http_timeout_ms = 15000u;
  job.attempt = 0u;
  job.next_attempt_ms = now_ms;
  const bool queued = AlertNtfyEnqueueJob(&manager->ntfy, &job);
  ESP_LOGI(kTag,
           "ntfy direct test queued=%u sequence=%s title=%s",
           queued ? 1u : 0u,
           job.sequence_id,
           job.title);
  return queued;
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
  (void)AlertManagerQueueOneShot(
    manager,
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
  (void)AlertManagerQueueOneShot(
    manager,
    &manager->states[leaf_index][ALERT_SYSTEM_MODE],
    ALERT_SYSTEM_MODE,
    ALERT_SEV_INFO,
    leaf_id,
    &payload,
    now_ms);
}

void
AlertManagerStartupNtfyBegin(alert_manager_t* manager,
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
  if (manager->startup_ntfy_pending_active) {
    return;
  }
  manager->startup_ntfy_pending_active = true;
  manager->startup_ntfy_boot_seen = true;
  manager->startup_ntfy_mode_seen = false;
  manager->startup_ntfy_mode_code = ALERT_SYSTEM_CODE_NONE;
  manager->startup_ntfy_first_ms = now_ms;
  manager->startup_ntfy_first_epoch = now_epoch;
  ESP_LOGI(kTag, "startup ntfy pending opened");
}

void
AlertManagerStartupNtfyUpdateMode(alert_manager_t* manager,
                                  alert_system_code_t mode_code,
                                  int64_t now_ms,
                                  int64_t now_epoch)
{
  if (manager == NULL) {
    return;
  }
  if (!manager->startup_ntfy_pending_active ||
      !manager->startup_ntfy_boot_seen) {
    AlertManagerEmitSystemMode(manager, mode_code, now_ms, now_epoch);
    return;
  }

  (void)AlertManagerStartupNtfyFlush(manager, now_ms, now_epoch, false);
  if (!manager->startup_ntfy_pending_active) {
    AlertManagerEmitSystemMode(manager, mode_code, now_ms, now_epoch);
    return;
  }

  const uint64_t leaf_id = ResolveLeafId(manager, 0);
  const uint32_t mask = EffectiveEnableMask(manager, leaf_id);
  if ((mask & (1u << ALERT_SYSTEM_MODE)) == 0u) {
    return;
  }
  manager->startup_ntfy_mode_seen = true;
  manager->startup_ntfy_mode_code = mode_code;
  if (now_epoch > 0) {
    manager->startup_ntfy_first_epoch = now_epoch;
  }
  ESP_LOGI(kTag,
           "startup ntfy pending updated with mode=%s",
           AlertManagerStartupModeLabel(mode_code));
  (void)AlertManagerStartupNtfyFlush(manager, now_ms, now_epoch, true);
}

void
AlertManagerStartupNtfyTick(alert_manager_t* manager,
                            int64_t now_ms,
                            int64_t now_epoch)
{
  if (manager == NULL || !manager->startup_ntfy_pending_active) {
    return;
  }
  (void)AlertManagerStartupNtfyFlush(manager, now_ms, now_epoch, false);
}

/**
 * @brief Execute AlertManagerProcessSystemError.
 * @param manager Parameter manager.
 * @param error_code Parameter error_code.
 * @param active Parameter active.
 * @param now_ms Parameter now_ms.
 * @param now_epoch Parameter now_epoch.
 */
static alert_state_t*
GetSystemErrorState(alert_manager_t* manager, alert_system_code_t error_code)
{
  if (manager == NULL) {
    return NULL;
  }
  if (error_code >= ALERT_SYSTEM_CODE_ERROR_MIN &&
      error_code <= ALERT_SYSTEM_CODE_ERROR_MAX) {
    return &manager->system_error_state_by_code[error_code];
  }
  return &manager->states[0][ALERT_SYSTEM_ERROR];
}

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

  alert_state_t* state = GetSystemErrorState(manager, error_code);
  if (state == NULL) {
    return;
  }

  alert_notification_payload_t payload = { 0 };
  FillPayloadBase(&payload, &manager->leaves[leaf_index], now_ms, now_epoch);
  payload.event_code = error_code;

  if (active) {
    state->last_seen_ms = now_ms;
    if (!state->active) {
      AlertStateTransition(state, true, now_ms);
      payload.transitions = state->transitions;
      if (AlertManagerQueueNotification(manager,
                                        state,
                                        ALERT_SYSTEM_ERROR,
                                        ALERT_SEV_CRIT,
                                        false,
                                        manager->leaves[leaf_index].leaf_id,
                                        &payload,
                                        now_ms)) {
        state->last_notify_ms = now_ms;
      }
      state->last_severity = ALERT_SEV_CRIT;
    } else if (manager->config.per_key_cooldown_ms > 0 &&
               (now_ms - state->last_notify_ms) >=
                 (int64_t)manager->config.per_key_cooldown_ms) {
      payload.transitions = state->transitions;
      if (AlertManagerQueueNotification(manager,
                                        state,
                                        ALERT_SYSTEM_ERROR,
                                        ALERT_SEV_CRIT,
                                        false,
                                        manager->leaves[leaf_index].leaf_id,
                                        &payload,
                                        now_ms)) {
        state->last_notify_ms = now_ms;
      }
      state->last_severity = ALERT_SEV_CRIT;
    }
  } else if (state->active) {
    AlertStateTransition(state, false, now_ms);
    payload.transitions = state->transitions;
    if (AlertManagerQueueNotification(manager,
                                      state,
                                      ALERT_SYSTEM_ERROR,
                                      ALERT_SEV_CRIT,
                                      true,
                                      manager->leaves[leaf_index].leaf_id,
                                      &payload,
                                      now_ms)) {
      state->last_notify_ms = now_ms;
    }
  }
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
  int64_t min_interval_ms = 300000;
  if (manager != NULL && manager->config.ntfy_min_send_interval_ms > 0) {
    min_interval_ms = manager->config.ntfy_min_send_interval_ms;
  }
  return min_interval_ms;
}

static uint32_t
ResolveNtfyHttpTimeoutMs(const alert_manager_t* manager)
{
  (void)manager;
  return 0;
}

static void
FormatEpoch(int64_t epoch_seconds, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  if (epoch_seconds <= 0) {
    snprintf(out, out_size, "unknown");
    return;
  }
  time_t raw = (time_t)epoch_seconds;
  struct tm timeinfo;
  gmtime_r(&raw, &timeinfo);
  strftime(out, out_size, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
}

static void
FormatMilliC(int32_t milli_c, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  int64_t value = milli_c;
  bool negative = value < 0;
  int64_t abs_value = negative ? -value : value;
  int64_t whole = abs_value / 1000;
  int64_t frac = abs_value % 1000;
  snprintf(out,
           out_size,
           "%s%" PRId64 ".%03" PRId64 "C",
           negative ? "-" : "",
           whole,
           frac);
}

static void
FormatBatchWindow(int64_t window_ms, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  int64_t total_seconds = (window_ms + 999) / 1000;
  if (total_seconds >= 60 && (total_seconds % 60) == 0) {
    snprintf(out, out_size, "%" PRId64 "m", total_seconds / 60);
    return;
  }
  if (total_seconds >= 60) {
    snprintf(out,
             out_size,
             "%" PRId64 "m%" PRId64 "s",
             total_seconds / 60,
             total_seconds % 60);
    return;
  }
  snprintf(out, out_size, "%" PRId64 "s", total_seconds);
}

static bool
AlertNotificationKeyMatches(const alert_notification_t* left,
                            const alert_notification_t* right)
{
  if (left == NULL || right == NULL) {
    return false;
  }
  if (left->type != right->type || left->resolved != right->resolved ||
      left->leaf_id != right->leaf_id || left->severity != right->severity) {
    return false;
  }

  if (left->type == ALERT_SYSTEM_ERROR) {
    return left->payload.event_code == right->payload.event_code;
  }

  return true;
}

static bool
AlertNotificationIsLeafScoped(const alert_notification_t* note)
{
  if (note == NULL) {
    return false;
  }
  switch (note->type) {
    case ALERT_TEMP_HIGH:
    case ALERT_TEMP_LOW:
    case ALERT_MISSING_RECORDS:
    case ALERT_LEAF_OFFLINE:
    case ALERT_LEAF_RESTART:
      return true;
    default:
      return false;
  }
}

static void
AlertNotificationDescribe(const alert_manager_t* manager,
                          const alert_notification_t* note,
                          char* out,
                          size_t out_size)
{
  if (note == NULL || out == NULL || out_size == 0) {
    return;
  }
  (void)manager;
  out[0] = '\0';
  char leaf_id[32] = "";
  if (AlertNotificationIsLeafScoped(note)) {
    AlertManagerFormatLeafId(note->leaf_id, leaf_id, sizeof(leaf_id));
    snprintf(out, out_size, "Leaf %s ", leaf_id);
  }

  switch (note->type) {
    case ALERT_TEMP_HIGH: {
      char temp_str[24];
      char limit_str[24];
      FormatMilliC(
        note->payload.current_temp_milli_c, temp_str, sizeof(temp_str));
      FormatMilliC(note->payload.limit_milli_c, limit_str, sizeof(limit_str));
      snprintf(out + strlen(out),
               out_size - strlen(out),
               "temp high %s > %s",
               temp_str,
               limit_str);
      break;
    }
    case ALERT_TEMP_LOW: {
      char temp_str[24];
      char limit_str[24];
      FormatMilliC(
        note->payload.current_temp_milli_c, temp_str, sizeof(temp_str));
      FormatMilliC(note->payload.limit_milli_c, limit_str, sizeof(limit_str));
      snprintf(out + strlen(out),
               out_size - strlen(out),
               "temp low %s < %s",
               temp_str,
               limit_str);
      break;
    }
    case ALERT_MISSING_RECORDS:
      snprintf(out + strlen(out),
               out_size - strlen(out),
               "missing records gap %" PRIu32 "ms",
               note->payload.duration_ms);
      break;
    case ALERT_LEAF_OFFLINE:
      snprintf(out + strlen(out),
               out_size - strlen(out),
               "offline for %" PRIu32 "ms",
               note->payload.duration_ms);
      break;
    case ALERT_LEAF_RESTART:
      snprintf(out + strlen(out), out_size - strlen(out), "sequence reset");
      break;
    case ALERT_ROOT_RESTART:
      snprintf(out + strlen(out), out_size - strlen(out), "root restart");
      break;
    case ALERT_SYSTEM_BOOT:
      snprintf(out + strlen(out), out_size - strlen(out), "system boot");
      break;
    case ALERT_SYSTEM_MODE: {
      const char* mode = "unknown";
      if (note->payload.event_code == ALERT_SYSTEM_CODE_MODE_RUN) {
        mode = "run";
      } else if (note->payload.event_code == ALERT_SYSTEM_CODE_MODE_DIAG) {
        mode = "diag";
      }
      snprintf(
        out + strlen(out), out_size - strlen(out), "system mode %s", mode);
      break;
    }
    case ALERT_SYSTEM_ERROR: {
      const char* error = "unknown";
      switch (note->payload.event_code) {
        case ALERT_SYSTEM_CODE_ERROR_SD_IO:
          error = "sd io error";
          break;
        case ALERT_SYSTEM_CODE_ERROR_FRAM_OVERRUN:
          error = "fram overrun";
          break;
        case ALERT_SYSTEM_CODE_ERROR_RTD_FAULT:
          error = "rtd fault";
          break;
        case ALERT_SYSTEM_CODE_ERROR_TIME_INVALID:
          error = "time invalid";
          break;
        case ALERT_SYSTEM_CODE_ERROR_FRAM_IO:
          error = "fram io error";
          break;
        case ALERT_SYSTEM_CODE_ERROR_I2C_RECOVERY:
          error = "i2c recovery";
          break;
        case ALERT_SYSTEM_CODE_ERROR_STORAGE_STALL:
          error = "storage stall";
          break;
        case ALERT_SYSTEM_CODE_ERROR_SENSOR_SPI:
          error = "sensor spi";
          break;
        case ALERT_SYSTEM_CODE_ERROR_NTFY_RATE_LIMIT:
          error = "ntfy rate limit";
          break;
        case ALERT_SYSTEM_CODE_ERROR_FRAM_ERRLOG:
          error = "fram errlog corrupt";
          break;
        case ALERT_SYSTEM_CODE_ERROR_NTFY_QUEUE:
          error = "ntfy queue full";
          break;
        case ALERT_SYSTEM_CODE_ERROR_I2C_HANG:
          error = "i2c hang";
          break;
        case ALERT_SYSTEM_CODE_ERROR_SD_OOS:
          error = "sd out of space";
          break;
        default:
          break;
      }
      snprintf(
        out + strlen(out), out_size - strlen(out), "system error %s", error);
      break;
    }
    default:
      snprintf(out + strlen(out), out_size - strlen(out), "unknown alert");
      break;
  }

  if (note->resolved) {
    snprintf(out + strlen(out), out_size - strlen(out), " — cleared");
  } else {
    snprintf(out + strlen(out), out_size - strlen(out), " — active");
  }
}

static void
AlertManagerLogNtfyNoteQueued(const alert_manager_t* manager,
                              const alert_notification_t* note)
{
  if (manager == NULL || note == NULL) {
    return;
  }
  char describe[256] = "";
  AlertNotificationDescribe(manager, note, describe, sizeof(describe));
  ESP_LOGI(kTag,
           "ntfy note queued: %s event_epoch=%" PRId64
           " event_uptime_ms=%" PRId64 " last_rx_epoch=%" PRId64
           " last_rx_uptime_ms=%" PRId64,
           describe,
           note->payload.event_epoch,
           note->payload.event_uptime_ms,
           note->payload.last_rx_epoch,
           note->payload.last_rx_uptime_ms);
}

static void
AlertManagerLogNtfyBodyLines(const char* label, const char* text)
{
  const char* line_label = (label != NULL) ? label : "unknown";
  if (text == NULL || text[0] == '\0') {
    ESP_LOGI(kTag, "ntfy body %s: <empty>", line_label);
    return;
  }

  const size_t raw_len = strnlen(text, ALERT_NTFY_JOB_BODY_LEN);
  const bool truncated = (raw_len == ALERT_NTFY_JOB_BODY_LEN);
  char cleaned[ALERT_NTFY_JOB_BODY_LEN + 1];
  size_t cleaned_len = 0;
  for (size_t i = 0; i < raw_len && cleaned_len < ALERT_NTFY_JOB_BODY_LEN;
       ++i) {
    if (text[i] == '\r') {
      continue;
    }
    cleaned[cleaned_len++] = text[i];
  }
  cleaned[cleaned_len] = '\0';

  ESP_LOGI(kTag,
           "ntfy body %s BEGIN\n%s\nntfy body %s END%s",
           line_label,
           cleaned,
           line_label,
           truncated ? " (truncated)" : "");
}

static void
AlertManagerBatchInit(alert_batch_t* batch)
{
  if (batch == NULL) {
    return;
  }
  memset(batch, 0, sizeof(*batch));
  batch->last_event_epoch = -1;
}

static void
AlertManagerBatchAdd(alert_batch_t* batch, const alert_notification_t* note)
{
  if (batch == NULL || note == NULL) {
    return;
  }
  batch->total_count++;
  for (size_t i = 0; i < batch->entry_count; ++i) {
    if (AlertNotificationKeyMatches(&batch->entries[i].note, note)) {
      batch->entries[i].count++;
      if (note->payload.event_epoch > 0 &&
          note->payload.event_epoch >
            batch->entries[i].note.payload.event_epoch) {
        batch->entries[i].note = *note;
      } else if (note->payload.event_epoch <= 0 &&
                 batch->entries[i].note.payload.event_epoch <= 0 &&
                 note->payload.event_uptime_ms >
                   batch->entries[i].note.payload.event_uptime_ms) {
        batch->entries[i].note = *note;
      }
      goto update_last_event;
    }
  }
  if (batch->entry_count < ALERT_NTFY_QUEUE_LEN) {
    batch->entries[batch->entry_count].note = *note;
    batch->entries[batch->entry_count].count = 1;
    batch->entry_count++;
  }

update_last_event:
  if (note->payload.event_epoch > 0) {
    if (note->payload.event_epoch > batch->last_event_epoch) {
      batch->last_event_epoch = note->payload.event_epoch;
    }
    return;
  }
  if (batch->last_event_epoch <= 0 &&
      note->payload.event_uptime_ms > batch->last_event_uptime_ms) {
    batch->last_event_uptime_ms = note->payload.event_uptime_ms;
  }
}

static size_t
AlertManagerDrainNtfyQueue(alert_manager_t* manager,
                           alert_notification_t* notes,
                           size_t max_notes,
                           uint32_t wait_ms)
{
  if (manager == NULL || notes == NULL || max_notes == 0) {
    return 0;
  }
  size_t count = 0;
  if (wait_ms > 0) {
    if (xQueueReceive(manager->ntfy.queue,
                      &notes[count],
                      pdMS_TO_TICKS(wait_ms)) != pdTRUE) {
      return 0;
    }
    count++;
  }
  while (count < max_notes &&
         xQueueReceive(manager->ntfy.queue, &notes[count], 0) == pdTRUE) {
    count++;
  }
  return count;
}

static bool
AlertManagerBuildBatchMessage(const alert_manager_t* manager,
                              const alert_batch_t* batch,
                              uint32_t msg_seq,
                              char* title,
                              size_t title_size,
                              char* body,
                              size_t body_size)
{
  if (manager == NULL || batch == NULL || title == NULL || body == NULL) {
    return false;
  }
  snprintf(title, title_size, "PT100 Alerts (#%06" PRIu32 ")", msg_seq);
  char window[16];
  FormatBatchWindow(ResolveNtfyMinIntervalMs(manager), window, sizeof(window));
  int written = snprintf(body,
                         body_size,
                         "Message seq: %" PRIu32 "\nBatched %" PRIu32
                         " alerts over last %s:\n\n",
                         msg_seq,
                         batch->total_count,
                         window);
  if (written < 0 || (size_t)written >= body_size) {
    return false;
  }
  size_t used = (size_t)written;
  for (size_t i = 0; i < batch->entry_count; ++i) {
    char line[256];
    AlertNotificationDescribe(
      manager, &batch->entries[i].note, line, sizeof(line));
    if (batch->entries[i].count > 1) {
      snprintf(line + strlen(line),
               sizeof(line) - strlen(line),
               " (x%" PRIu32 ")",
               batch->entries[i].count);
    }
    int line_written = snprintf(body + used, body_size - used, "• %s\n", line);
    if (line_written < 0) {
      break;
    }
    if ((size_t)line_written >= (body_size - used)) {
      used = body_size - 1;
      body[used] = '\0';
      break;
    }
    used += (size_t)line_written;
  }
  if (batch->last_event_epoch > 0) {
    char time_str[32];
    FormatEpoch(batch->last_event_epoch, time_str, sizeof(time_str));
    snprintf(body + used, body_size - used, "\nLast event: %s\n", time_str);
  } else if (batch->last_event_uptime_ms > 0) {
    snprintf(body + used,
             body_size - used,
             "\nLast event: uptime=%" PRIu32 "ms\n",
             (uint32_t)batch->last_event_uptime_ms);
  }
  return true;
}

static int64_t
AlertManagerComputeNextSendMs(const alert_manager_t* manager,
                              int64_t now_ms,
                              int retry_after_seconds)
{
  int64_t min_interval_ms = ResolveNtfyMinIntervalMs(manager);
  int64_t min_send_ms = now_ms;
  if (manager != NULL && manager->ntfy.last_sent_valid) {
    min_send_ms = manager->ntfy.last_sent_ms + min_interval_ms;
  }
  if (retry_after_seconds > 0) {
    int64_t retry_ms = now_ms + ((int64_t)retry_after_seconds * 1000);
    return (retry_ms > min_send_ms) ? retry_ms : min_send_ms;
  }
  return min_send_ms;
}

/**
 * @brief Build a stable ntfy sequence ID for a queued HTTP job.
 * @param boot_nonce Random nonce generated at boot.
 * @param msg_seq Monotonic message sequence number.
 * @param out Output buffer for formatted ID.
 * @param out_size Size of output buffer.
 */
static void
AlertManagerBuildNtfySequenceId(uint32_t boot_nonce,
                                uint32_t msg_seq,
                                char* out,
                                size_t out_size)
{
  if (out == NULL || out_size == 0u) {
    return;
  }
  (void)snprintf(
    out, out_size, "pt100-%08" PRIx32 "-%08" PRIx32, boot_nonce, msg_seq);
}

static bool
AlertManagerSendBatchedNtfy(alert_manager_t* manager,
                            int64_t now_ms,
                            int64_t now_epoch,
                            uint32_t wait_ms,
                            int64_t* next_attempt_ms)
{
  if (manager == NULL || manager->ntfy.queue == NULL ||
      manager->ntfy.job_queue == NULL || manager->ntfy_batch_scratch == NULL) {
    return false;
  }
  (void)now_epoch;

  if (manager->ntfy.cooldown_until_ms > now_ms) {
    if (next_attempt_ms != NULL) {
      *next_attempt_ms = manager->ntfy.cooldown_until_ms;
    }
    return false;
  }

  const size_t max_notes = ALERT_NTFY_QUEUE_LEN;
  alert_ntfy_batch_scratch_t* scratch = manager->ntfy_batch_scratch;
  size_t note_count =
    AlertManagerDrainNtfyQueue(manager, scratch->notes, max_notes, wait_ms);
  if (note_count == 0) {
    return false;
  }

  const uint32_t msg_seq = ++g_ntfy_msg_seq;

  AlertManagerBatchInit(&scratch->batch);
  for (size_t i = 0; i < note_count; ++i) {
    AlertManagerBatchAdd(&scratch->batch, &scratch->notes[i]);
  }

  if (!AlertManagerBuildBatchMessage(manager,
                                     &scratch->batch,
                                     msg_seq,
                                     scratch->title,
                                     sizeof(scratch->title),
                                     scratch->body,
                                     sizeof(scratch->body))) {
    return false;
  }

  alert_ntfy_job_t job = { 0 };
  snprintf(job.url, sizeof(job.url), "%s", manager->config.ntfy_url);
  snprintf(job.topic, sizeof(job.topic), "%s", manager->config.ntfy_topic);
  snprintf(job.token, sizeof(job.token), "%s", manager->config.ntfy_token);
  snprintf(job.root_id,
           sizeof(job.root_id),
           "%s",
           (manager->root_id_string != NULL) ? manager->root_id_string
                                             : "unknown");
  snprintf(job.title, sizeof(job.title), "%s", scratch->title);
  snprintf(job.body, sizeof(job.body), "%s", scratch->body);
  job.http_timeout_ms = ResolveNtfyHttpTimeoutMs(manager);
  AlertManagerBuildNtfySequenceId(manager->ntfy_boot_nonce,
                                  msg_seq,
                                  job.sequence_id,
                                  sizeof(job.sequence_id));
  job.attempt = 0;
  job.next_attempt_ms = now_ms;

  const bool queued = AlertNtfyEnqueueJob(&manager->ntfy, &job);
  if (!queued) {
    for (size_t i = 0; i < note_count; ++i) {
      (void)AlertNtfyEnqueue(&manager->ntfy, &scratch->notes[i]);
    }
    manager->ntfy.cooldown_until_ms = now_ms + 1000;
    if (next_attempt_ms != NULL) {
      *next_attempt_ms = manager->ntfy.cooldown_until_ms;
    }
    return true;
  }

  manager->ntfy.last_attempt_ms = now_ms;
  ESP_LOGI(kTag,
           "ntfy job queued seq=%" PRIu32
           " seq_id=%s status=%d title=\"%s\" total=%" PRIu32 " unique=%u",
           msg_seq,
           job.sequence_id,
           manager->ntfy.last_http_status,
           scratch->title,
           scratch->batch.total_count,
           (unsigned)scratch->batch.entry_count);
  AlertManagerLogNtfyBodyLines("queued", scratch->body);
  return true;
}

static void
AlertManagerStartupNtfyReset(alert_manager_t* manager)
{
  if (manager == NULL) {
    return;
  }
  manager->startup_ntfy_pending_active = false;
  manager->startup_ntfy_boot_seen = false;
  manager->startup_ntfy_mode_seen = false;
  manager->startup_ntfy_mode_code = ALERT_SYSTEM_CODE_NONE;
  manager->startup_ntfy_first_ms = 0;
  manager->startup_ntfy_first_epoch = -1;
}

static bool
AlertManagerStartupNtfyWindowExpired(const alert_manager_t* manager,
                                     int64_t now_ms)
{
  if (manager == NULL || !manager->startup_ntfy_pending_active ||
      manager->startup_ntfy_first_ms <= 0 || now_ms <= 0) {
    return false;
  }
  return (now_ms - manager->startup_ntfy_first_ms) >=
         kStartupNtfyCoalesceWindowMs;
}

static const char*
AlertManagerStartupModeLabel(alert_system_code_t mode_code)
{
  switch (mode_code) {
    case ALERT_SYSTEM_CODE_MODE_RUN:
      return "RUN";
    case ALERT_SYSTEM_CODE_MODE_DIAG:
      return "DIAG";
    default:
      break;
  }
  return "UNKNOWN";
}

static void
AlertManagerStartupNtfyBuildMessage(const alert_manager_t* manager,
                                    char* title,
                                    size_t title_size,
                                    char* body,
                                    size_t body_size)
{
  if (manager == NULL || title == NULL || body == NULL || title_size == 0 ||
      body_size == 0) {
    return;
  }
  if (manager->startup_ntfy_mode_seen) {
    snprintf(title,
             title_size,
             "START: System booted in %s",
             AlertManagerStartupModeLabel(manager->startup_ntfy_mode_code));
  } else {
    snprintf(title, title_size, "%s", "START: System booted");
  }

  const char* root_id =
    (manager->root_id_string != NULL) ? manager->root_id_string : "unknown";
  int written = snprintf(
    body, body_size, "root: %s\n• system boot\n", root_id);
  if (written < 0) {
    body[0] = '\0';
    return;
  }
  size_t used = (size_t)written;
  if (used >= body_size) {
    body[body_size - 1] = '\0';
    return;
  }
  if (manager->startup_ntfy_mode_seen) {
    (void)snprintf(body + used,
                   body_size - used,
                   "• initial mode %s\n",
                   AlertManagerStartupModeLabel(manager->startup_ntfy_mode_code));
  }
}

static bool
AlertManagerStartupNtfyFlush(alert_manager_t* manager,
                             int64_t now_ms,
                             int64_t now_epoch,
                             bool force_flush)
{
  if (manager == NULL || !manager->startup_ntfy_pending_active ||
      !manager->startup_ntfy_boot_seen || manager->ntfy.job_queue == NULL) {
    return false;
  }
  if (!force_flush && !AlertManagerStartupNtfyWindowExpired(manager, now_ms)) {
    return false;
  }

  const uint32_t msg_seq = ++g_ntfy_msg_seq;
  alert_ntfy_job_t job = { 0 };
  AlertManagerStartupNtfyBuildMessage(
    manager, job.title, sizeof(job.title), job.body, sizeof(job.body));
  snprintf(job.url, sizeof(job.url), "%s", manager->config.ntfy_url);
  snprintf(job.topic, sizeof(job.topic), "%s", manager->config.ntfy_topic);
  snprintf(job.token, sizeof(job.token), "%s", manager->config.ntfy_token);
  snprintf(job.root_id,
           sizeof(job.root_id),
           "%s",
           (manager->root_id_string != NULL) ? manager->root_id_string
                                             : "unknown");
  AlertManagerBuildNtfySequenceId(manager->ntfy_boot_nonce,
                                  msg_seq,
                                  job.sequence_id,
                                  sizeof(job.sequence_id));
  job.http_timeout_ms = ResolveNtfyHttpTimeoutMs(manager);
  job.attempt = 0u;
  job.next_attempt_ms = now_ms;

  if (!AlertNtfyEnqueueJob(&manager->ntfy, &job)) {
    ESP_LOGW(kTag, "startup ntfy flush deferred (job queue full)");
    return false;
  }

  manager->ntfy.last_attempt_ms = now_ms;
  if (manager->startup_ntfy_mode_seen) {
    ESP_LOGI(kTag,
             "startup ntfy flushed combined (mode=%s)",
             AlertManagerStartupModeLabel(manager->startup_ntfy_mode_code));
  } else {
    ESP_LOGI(kTag, "startup ntfy flushed boot-only");
  }
  AlertManagerLogNtfyBodyLines("startup", job.body);
  if (now_epoch > 0) {
    manager->startup_ntfy_first_epoch = now_epoch;
  }
  AlertManagerStartupNtfyReset(manager);
  return true;
}

static bool
AppendTextLine(char* body, size_t body_size, size_t* used, const char* text)
{
  if (body == NULL || used == NULL || text == NULL || body_size == 0) {
    return false;
  }
  if (*used >= body_size - 1) {
    body[body_size - 1] = '\0';
    return false;
  }
  const int written = snprintf(body + *used, body_size - *used, "%s\n", text);
  if (written < 0) {
    return false;
  }
  if ((size_t)written >= (body_size - *used)) {
    *used = body_size - 1;
    body[body_size - 1] = '\0';
    return false;
  }
  *used += (size_t)written;
  return true;
}

bool
AlertManagerPumpNtfy(alert_manager_t* manager,
                     int64_t now_ms,
                     int64_t* next_attempt_ms)
{
  if (manager == NULL || manager->ntfy.queue == NULL ||
      manager->ntfy.job_queue == NULL) {
    return false;
  }
  int64_t now_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
  return AlertManagerSendBatchedNtfy(
    manager, now_ms, now_epoch, 0, next_attempt_ms);
}

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
bool
AlertManagerBuildStopFlushNtfyJob(alert_manager_t* manager,
                                  const char* stop_reason,
                                  int64_t now_ms,
                                  alert_ntfy_job_t* out_job,
                                  uint32_t* out_notes_drained,
                                  uint32_t* out_jobs_drained)
{
  if (out_notes_drained != NULL) {
    *out_notes_drained = 0;
  }
  if (out_jobs_drained != NULL) {
    *out_jobs_drained = 0;
  }
  if (manager == NULL || out_job == NULL || manager->ntfy.queue == NULL ||
      manager->ntfy.job_queue == NULL || !AlertManagerIsConfigured(manager)) {
    return false;
  }

  alert_ntfy_batch_scratch_t* scratch = manager->ntfy_batch_scratch;
  if (scratch == NULL) {
    manager->ntfy_batch_scratch =
      heap_caps_calloc(1,
                       sizeof(*manager->ntfy_batch_scratch),
                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    scratch = manager->ntfy_batch_scratch;
    if (scratch == NULL) {
      ESP_LOGE(kTag, "Failed to allocate ntfy scratch storage for STOP flush");
      return false;
    }
  }
  memset(scratch, 0, sizeof(*scratch));

  const size_t notes_max = ALERT_NTFY_QUEUE_LEN;
  const size_t jobs_max = ALERT_NTFY_JOB_QUEUE_LEN;
  char drained_job_titles[ALERT_NTFY_JOB_QUEUE_LEN]
                         [ALERT_NTFY_JOB_TITLE_LEN] = { 0 };

  size_t notes_count = 0;
  while (notes_count < notes_max &&
         xQueueReceive(manager->ntfy.queue, &scratch->notes[notes_count], 0) ==
           pdTRUE) {
    AlertManagerBatchAdd(&scratch->batch, &scratch->notes[notes_count]);
    notes_count++;
  }

  alert_ntfy_job_t drained_job = { 0 };
  size_t jobs_count = 0;
  while (jobs_count < jobs_max &&
         xQueueReceive(manager->ntfy.job_queue, &drained_job, 0) == pdTRUE) {
    snprintf(drained_job_titles[jobs_count],
             sizeof(drained_job_titles[jobs_count]),
             "%s",
             drained_job.title);
    jobs_count++;
  }

  if (out_notes_drained != NULL) {
    *out_notes_drained = (uint32_t)notes_count;
  }
  if (out_jobs_drained != NULL) {
    *out_jobs_drained = (uint32_t)jobs_count;
  }

  memset(out_job, 0, sizeof(*out_job));
  snprintf(out_job->url, sizeof(out_job->url), "%s", manager->config.ntfy_url);
  snprintf(
    out_job->topic, sizeof(out_job->topic), "%s", manager->config.ntfy_topic);
  snprintf(
    out_job->token, sizeof(out_job->token), "%s", manager->config.ntfy_token);
  snprintf(out_job->root_id,
           sizeof(out_job->root_id),
           "%s",
           (manager->root_id_string != NULL) ? manager->root_id_string
                                             : "root");
  snprintf(out_job->title,
           sizeof(out_job->title),
           "STOP: Logger halted (consolidated)");

  size_t used = 0;
  char summary[192];
  snprintf(summary,
           sizeof(summary),
           "STOP requested: %s",
           (stop_reason != NULL && stop_reason[0] != '\0') ? stop_reason
                                                           : "operator stop");
  (void)AppendTextLine(out_job->body, sizeof(out_job->body), &used, summary);

  snprintf(summary,
           sizeof(summary),
           "Consolidated pending ntfy items: %u notes, %u jobs",
           (unsigned)notes_count,
           (unsigned)jobs_count);
  (void)AppendTextLine(out_job->body, sizeof(out_job->body), &used, summary);

  if (jobs_count > 0) {
    (void)AppendTextLine(
      out_job->body, sizeof(out_job->body), &used, "Queued job titles:");
    const size_t max_titles = 6;
    const size_t emit = (jobs_count < max_titles) ? jobs_count : max_titles;
    for (size_t i = 0; i < emit; ++i) {
      const size_t kSummaryPrefixLen = 14; // "- queued job: "
      const size_t max_title_chars =
        (sizeof(summary) > (kSummaryPrefixLen + 1))
          ? (sizeof(summary) - (kSummaryPrefixLen + 1))
          : 0;
      snprintf(summary,
               sizeof(summary),
               "- queued job: %.*s",
               (int)max_title_chars,
               drained_job_titles[i]);
      (void)AppendTextLine(
        out_job->body, sizeof(out_job->body), &used, summary);
    }
    if (jobs_count > emit) {
      snprintf(summary,
               sizeof(summary),
               "- (+%u more jobs)",
               (unsigned)(jobs_count - emit));
      (void)AppendTextLine(
        out_job->body, sizeof(out_job->body), &used, summary);
    }
  }

  if (notes_count > 0) {
    (void)AppendTextLine(
      out_job->body, sizeof(out_job->body), &used, "Pending alert notes:");
    const size_t max_notes = 10;
    const size_t emit = (notes_count < max_notes) ? notes_count : max_notes;
    for (size_t i = 0; i < emit; ++i) {
      char line[256];
      AlertNotificationDescribe(
        manager, &scratch->notes[i], line, sizeof(line));
      const size_t kSummaryPrefixLen = 2; // "- "
      const size_t max_line_chars =
        (sizeof(summary) > (kSummaryPrefixLen + 1))
          ? (sizeof(summary) - (kSummaryPrefixLen + 1))
          : 0;
      snprintf(summary, sizeof(summary), "- %.*s", (int)max_line_chars, line);
      (void)AppendTextLine(
        out_job->body, sizeof(out_job->body), &used, summary);
    }
    if (notes_count > emit) {
      snprintf(summary,
               sizeof(summary),
               "- (+%u more notes)",
               (unsigned)(notes_count - emit));
      (void)AppendTextLine(
        out_job->body, sizeof(out_job->body), &used, summary);
    }
  }

  snprintf(summary,
           sizeof(summary),
           "PT100 - %s",
           (manager->root_id_string != NULL) ? manager->root_id_string
                                             : "root");
  (void)AppendTextLine(out_job->body, sizeof(out_job->body), &used, summary);

  out_job->http_timeout_ms = ResolveNtfyHttpTimeoutMs(manager);
  out_job->attempt = 0;
  out_job->next_attempt_ms = now_ms;

  return true;
}

/**
 * @brief Execute AlertManagerUpdateNtfySendState.
 * @param manager Parameter manager.
 * @param result Parameter result.
 * @param status Parameter status.
 * @param retry_after_seconds Parameter retry_after_seconds.
 * @param err Parameter err.
 * @param now_ms Parameter now_ms.
 * @param next_attempt_ms Parameter next_attempt_ms.
 */
void
AlertManagerUpdateNtfySendState(alert_manager_t* manager,
                                alert_ntfy_result_t result,
                                int status,
                                int retry_after_seconds,
                                esp_err_t err,
                                int64_t now_ms,
                                int64_t* next_attempt_ms)
{
  if (manager == NULL) {
    return;
  }
  manager->ntfy.last_attempt_ms = now_ms;

  if (result == ALERT_NTFY_OK) {
    manager->ntfy.send_success++;
    manager->ntfy.last_http_status = status;
    manager->ntfy.last_err = err;
    manager->ntfy.backoff_ms = 0;
    manager->ntfy.last_sent_ms = now_ms;
    manager->ntfy.last_sent_valid = true;
    manager->ntfy.cooldown_until_ms =
      now_ms + ResolveNtfyMinIntervalMs(manager);
    manager->ntfy.rate_limited_count = 0;
    if (next_attempt_ms != NULL) {
      *next_attempt_ms = manager->ntfy.cooldown_until_ms;
    }
    return;
  }

  if (result == ALERT_NTFY_SKIPPED) {
    manager->ntfy.last_err = err;
    return;
  }

  manager->ntfy.send_fail++;
  manager->ntfy.last_http_status = status;
  manager->ntfy.last_err = err;
  if (status == 429) {
    manager->ntfy.rate_limited_count++;
    const int64_t retry_ms =
      AlertManagerComputeNextSendMs(manager, now_ms, retry_after_seconds);
    manager->ntfy.cooldown_until_ms = retry_ms;
    if (next_attempt_ms != NULL) {
      *next_attempt_ms = manager->ntfy.cooldown_until_ms;
    }
  } else {
    manager->ntfy.rate_limited_count = 0;
    manager->ntfy.backoff_ms =
      (manager->ntfy.backoff_ms == 0) ? 1000 : (manager->ntfy.backoff_ms * 2);
    if (manager->ntfy.backoff_ms > kNtfyFailureMaxBackoffMs) {
      manager->ntfy.backoff_ms = kNtfyFailureMaxBackoffMs;
    }
    int64_t retry_ms = now_ms + (int64_t)manager->ntfy.backoff_ms;
    int64_t min_retry_ms = AlertManagerComputeNextSendMs(manager, now_ms, 0);
    if (retry_ms < min_retry_ms) {
      retry_ms = min_retry_ms;
    }
    manager->ntfy.cooldown_until_ms = retry_ms;
    if (next_attempt_ms != NULL) {
      *next_attempt_ms = manager->ntfy.cooldown_until_ms;
    }
  }
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

  while (!*ctx->stop_requested) {
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t now_epoch = TimeSyncIsSystemTimeValid() ? (int64_t)time(NULL) : -1;
    if (ctx->manager->ntfy.cooldown_until_ms > now_ms) {
      int64_t remaining_ms = ctx->manager->ntfy.cooldown_until_ms - now_ms;
      if (remaining_ms > 0) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS((uint32_t)remaining_ms));
        if (*ctx->stop_requested) {
          break;
        }
      }
      continue;
    }
    (void)AlertManagerSendBatchedNtfy(
      ctx->manager, now_ms, now_epoch, 1000, NULL);
  }

  if (ctx->task_handle != NULL) {
    *ctx->task_handle = NULL;
  }
  vTaskDelete(NULL);
}
