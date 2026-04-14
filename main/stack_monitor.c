#include "stack_monitor.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char* kTag = "stack_monitor";

enum
{
  kStackMonitorMaxSnapshotTasks = 48,
  kStackMonitorMaxReportRows =
    kStackMonitorMaxEntries + kStackMonitorMaxSnapshotTasks,
};

typedef struct
{
  const stack_monitor_entry_t* tracked_entry;
  const TaskStatus_t* snapshot_entry;
  bool is_tracked;
} stack_report_row_t;

static const char* TaskStateToString(eTaskState state);
static int CompareRowsForPrint(const void* lhs, const void* rhs);
static int CompareTaskNames(const char* lhs, const char* rhs);
static void
FormatValueOrNa(char* buffer, size_t buffer_size, bool known, uint32_t value);

/**
 * @brief Round an unsigned byte count up to the requested alignment.
 *
 * If @p alignment is 0, the input @p value is returned unchanged.
 *
 * @param value Byte count to round.
 * @param alignment Alignment in bytes.
 * @return The smallest multiple of @p alignment that is greater than or equal
 *         to @p value.
 */
static uint32_t
RoundUpBytes(uint32_t value, uint32_t alignment)
{
  if (alignment == 0) {
    return value;
  }
  return (value + alignment - 1U) / alignment * alignment;
}

/**
 * @brief Read a task's stack high-water mark in bytes.
 *
 * Uses uxTaskGetStackHighWaterMark2() when enabled by FreeRTOS configuration;
 * otherwise falls back to uxTaskGetStackHighWaterMark().
 *
 * @param handle Task handle to query.
 * @return The high-water mark value returned by the selected FreeRTOS API,
 *         cast to uint32_t.
 */
static uint32_t
GetStackHighWaterMarkBytes(TaskHandle_t handle)
{
#if (defined(INCLUDE_uxTaskGetStackHighWaterMark2) &&                          \
     (INCLUDE_uxTaskGetStackHighWaterMark2 == 1))

  return (uint32_t)uxTaskGetStackHighWaterMark2(handle);
#else
  return (uint32_t)uxTaskGetStackHighWaterMark(handle);
#endif
}

static const char*
TaskStateToString(eTaskState state)
{
  switch (state) {
    case eRunning:
      return "run";
    case eReady:
      return "ready";
    case eBlocked:
      return "block";
    case eSuspended:
      return "susp";
    case eDeleted:
      return "del";
    case eInvalid:
    default:
      return "-";
  }
}

static int
CompareTaskNames(const char* lhs, const char* rhs)
{
  if (lhs == NULL && rhs == NULL) {
    return 0;
  }
  if (lhs == NULL) {
    return 1;
  }
  if (rhs == NULL) {
    return -1;
  }
  while (*lhs != '\0' && *rhs != '\0') {
    const int lhs_ch = tolower((unsigned char)*lhs);
    const int rhs_ch = tolower((unsigned char)*rhs);
    if (lhs_ch != rhs_ch) {
      return lhs_ch - rhs_ch;
    }
    ++lhs;
    ++rhs;
  }
  return (unsigned char)*lhs - (unsigned char)*rhs;
}

static int
CompareRowsForPrint(const void* lhs, const void* rhs)
{
  const stack_report_row_t* const a = (const stack_report_row_t*)lhs;
  const stack_report_row_t* const b = (const stack_report_row_t*)rhs;
  if (a->is_tracked != b->is_tracked) {
    return a->is_tracked ? -1 : 1;
  }

  const char* const a_name =
    (a->is_tracked && a->tracked_entry != NULL) ? a->tracked_entry->task_name
                                                : a->snapshot_entry->pcTaskName;
  const char* const b_name =
    (b->is_tracked && b->tracked_entry != NULL) ? b->tracked_entry->task_name
                                                : b->snapshot_entry->pcTaskName;
  return CompareTaskNames(a_name, b_name);
}

static void
FormatValueOrNa(char* buffer, size_t buffer_size, bool known, uint32_t value)
{
  if (buffer == NULL || buffer_size == 0U) {
    return;
  }
  if (!known) {
    snprintf(buffer, buffer_size, "n/a");
    return;
  }
  snprintf(buffer, buffer_size, "%" PRIu32, value);
}

/**
 * @brief Initialize a stack monitor instance.
 *
 * The monitor structure is zero-initialized and configured with a minimum
 * sampling period.
 *
 * @param monitor Pointer to the monitor to initialize. If NULL, the function
 *                returns without modifying anything.
 * @param sample_period_ms Minimum elapsed time between samples in milliseconds.
 */
void
StackMonitorInit(stack_monitor_t* monitor, uint32_t sample_period_ms)
{
  if (monitor == NULL) {
    return;
  }
  memset(monitor, 0, sizeof(*monitor));
  monitor->sample_period_ms = sample_period_ms;
  monitor->last_sample_ticks = 0;
}

/**
 * @brief Register a task handle pointer for periodic stack sampling.
 *
 * The entry is keyed by the address of @p handle_ptr. If an entry already
 * exists with the same handle pointer address, it is updated in place.
 * Otherwise, a new entry is appended if capacity allows.
 *
 * @param monitor Pointer to the monitor that owns the registry.
 * @param name Task name string to print in reports.
 * @param handle_ptr Pointer to a TaskHandle_t that will be sampled; entries are
 *                   skipped if *handle_ptr is NULL.
 * @param stack_alloc_bytes Configured stack allocation in bytes for the task.
 * @return true if the entry was added or updated; false if arguments are NULL
 *         or the monitor has no remaining capacity.
 */
bool
StackMonitorRegister(stack_monitor_t* monitor,
                     const char* name,
                     TaskHandle_t* handle_ptr,
                     uint32_t stack_alloc_bytes)
{
  if (monitor == NULL || name == NULL || handle_ptr == NULL) {
    return false;
  }
  for (size_t i = 0; i < monitor->entry_count; ++i) {
    stack_monitor_entry_t* entry = &monitor->entries[i];
    if (entry->task_handle_ptr == handle_ptr) {
      entry->task_name = name;
      entry->stack_alloc_bytes = stack_alloc_bytes;
      entry->last_free_bytes = stack_alloc_bytes;
      entry->min_free_bytes = stack_alloc_bytes;
      return true;
    }
  }
  if (monitor->entry_count >= kStackMonitorMaxEntries) {
    return false;
  }

  stack_monitor_entry_t* entry = &monitor->entries[monitor->entry_count++];
  entry->task_name = name;
  entry->task_handle_ptr = handle_ptr;
  entry->stack_alloc_bytes = stack_alloc_bytes;
  entry->last_free_bytes = stack_alloc_bytes;
  entry->min_free_bytes = stack_alloc_bytes;
  return true;
}

/**
 * @brief Sample registered tasks' stack high-water marks when the period
 * elapses.
 *
 * If enough time has elapsed since the last sample, this updates each entry's
 * last_free_bytes and min_free_bytes using the FreeRTOS stack high-water mark
 * API. Entries whose task handle pointer is NULL or whose task handle is NULL
 * are skipped.
 *
 * @param monitor Pointer to the monitor to sample.
 */
void
StackMonitorMaybeSample(stack_monitor_t* monitor)
{
  if (monitor == NULL || monitor->entry_count == 0) {
    return;
  }

  const TickType_t now_ticks = xTaskGetTickCount();
  if (monitor->last_sample_ticks != 0) {
    const TickType_t elapsed_ticks = now_ticks - monitor->last_sample_ticks;
    if (elapsed_ticks < pdMS_TO_TICKS(monitor->sample_period_ms)) {
      return;
    }
  }
  monitor->last_sample_ticks = now_ticks;

  for (size_t i = 0; i < monitor->entry_count; ++i) {
    stack_monitor_entry_t* entry = &monitor->entries[i];
    if (entry->task_handle_ptr == NULL || *entry->task_handle_ptr == NULL) {
      continue;
    }
    const uint32_t hwm_bytes =
      GetStackHighWaterMarkBytes(*entry->task_handle_ptr);
    entry->last_free_bytes = hwm_bytes;
    if (hwm_bytes < entry->min_free_bytes) {
      entry->min_free_bytes = hwm_bytes;
      if (hwm_bytes < 256U) {
        ESP_LOGE(kTag,
                 "task %s stack critically low: %" PRIu32 " bytes free",
                 entry->task_name,
                 hwm_bytes);
      } else if (hwm_bytes < 512U) {
        ESP_LOGW(kTag,
                 "task %s stack low: %" PRIu32 " bytes free",
                 entry->task_name,
                 hwm_bytes);
      }
    }
  }
}

/**
 * @brief Lookup the minimum free stack bytes for a named task.
 *
 * Searches for a matching task name in the monitor registry and copies the
 * entry's minimum observed free bytes into @p out_bytes.
 *
 * @param monitor Pointer to the monitor registry.
 * @param name Task name to search for.
 * @param out_bytes Output pointer for the min free bytes.
 * @return true if a matching entry was found.
 */
bool
StackMonitorGetMinFreeBytes(const stack_monitor_t* monitor,
                            const char* name,
                            uint32_t* out_bytes)
{
  if (monitor == NULL || name == NULL || out_bytes == NULL) {
    return false;
  }
  for (size_t i = 0; i < monitor->entry_count; ++i) {
    const stack_monitor_entry_t* entry = &monitor->entries[i];
    if (entry->task_name == NULL) {
      continue;
    }
    if (strcmp(entry->task_name, name) == 0) {
      *out_bytes = entry->min_free_bytes;
      return true;
    }
  }
  return false;
}

/**
 * @brief Print a fixed-width stack usage report with recommended stack sizes.
 *
 * The report includes each entry's configured allocation, last and minimum
 * observed free stack bytes, peak used bytes, and a recommended stack size
 * computed as (peak_used + headroom) rounded up to 256 bytes.
 *
 * The task name column width is chosen based on the longest task name
 * registered in @p monitor, clamped to 24 characters. Names longer than the
 * chosen width are truncated.
 *
 * @param monitor Pointer to the monitor to print.
 * @param headroom_bytes Extra bytes added to peak usage when computing the
 *                       recommended stack size.
 */
void
StackMonitorPrint(const stack_monitor_t* monitor, uint32_t headroom_bytes)
{
  if (monitor == NULL) {
    return;
  }

  static TaskStatus_t s_task_snapshot[kStackMonitorMaxSnapshotTasks];
  static stack_report_row_t s_rows[kStackMonitorMaxReportRows];
  static bool s_snapshot_used[kStackMonitorMaxSnapshotTasks];

  UBaseType_t snapshot_count = 0;
#if (defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1))
  const UBaseType_t task_count = uxTaskGetNumberOfTasks();
  const UBaseType_t snapshot_capacity =
    (UBaseType_t)kStackMonitorMaxSnapshotTasks;
  snapshot_count = uxTaskGetSystemState(s_task_snapshot, snapshot_capacity, NULL);
  if (task_count > snapshot_capacity) {
    printf("note: task snapshot truncated (%" PRIu32 " of %" PRIu32
           " tasks)\n",
           (uint32_t)snapshot_count,
           (uint32_t)task_count);
  }
#else
  printf("note: FreeRTOS trace facility disabled; showing tracked tasks only\n");
#endif

  memset(s_snapshot_used, 0, sizeof(s_snapshot_used));
  size_t row_count = 0;
  for (size_t i = 0;
       i < monitor->entry_count && row_count < kStackMonitorMaxReportRows;
       ++i) {
    const stack_monitor_entry_t* entry = &monitor->entries[i];
    stack_report_row_t* row = &s_rows[row_count++];
    row->tracked_entry = entry;
    row->snapshot_entry = NULL;
    row->is_tracked = true;

    TaskHandle_t handle = NULL;
    if (entry->task_handle_ptr != NULL) {
      handle = *entry->task_handle_ptr;
    }
    if (handle == NULL) {
      continue;
    }
    for (UBaseType_t j = 0; j < snapshot_count; ++j) {
      if (s_task_snapshot[j].xHandle == handle) {
        row->snapshot_entry = &s_task_snapshot[j];
        s_snapshot_used[j] = true;
        break;
      }
    }
  }

  for (UBaseType_t i = 0;
       i < snapshot_count && row_count < kStackMonitorMaxReportRows;
       ++i) {
    if (s_snapshot_used[i]) {
      continue;
    }
    stack_report_row_t* row = &s_rows[row_count++];
    row->tracked_entry = NULL;
    row->snapshot_entry = &s_task_snapshot[i];
    row->is_tracked = false;
  }

  qsort(s_rows, row_count, sizeof(s_rows[0]), CompareRowsForPrint);

  size_t task_col_width = strlen("task");
  for (size_t i = 0; i < row_count; ++i) {
    const stack_report_row_t* row = &s_rows[i];
    const char* name = NULL;
    if (row->is_tracked && row->tracked_entry != NULL) {
      name = row->tracked_entry->task_name;
    } else if (row->snapshot_entry != NULL) {
      name = row->snapshot_entry->pcTaskName;
    }
    if (name == NULL) {
      continue;
    }
    const size_t name_len = strlen(name);
    if (name_len > task_col_width) {
      task_col_width = name_len;
    }
  }
  if (task_col_width > 24U) {
    task_col_width = 24U;
  }

  const int task_w = (int)task_col_width;
  printf("%-*s %6s %10s %10s %10s %10s %12s %8s %4s\n",
         task_w,
         "task",
         "src",
         "alloc",
         "last_free",
         "min_free",
         "peak_used",
         "recommended",
         "state",
         "pri");
  for (size_t i = 0; i < row_count; ++i) {
    const stack_report_row_t* row = &s_rows[i];
    uint32_t alloc_bytes = 0;
    uint32_t last_free_bytes = 0;
    uint32_t min_free_bytes = 0;
    uint32_t peak_used_bytes = 0;
    uint32_t recommended_bytes = 0;
    bool has_alloc = false;
    bool has_last_free = false;
    bool has_min_free = false;
    bool has_peak_used = false;
    bool has_recommended = false;
    const char* name = "";

    if (row->is_tracked && row->tracked_entry != NULL) {
      const stack_monitor_entry_t* entry = row->tracked_entry;
      name = (entry->task_name != NULL) ? entry->task_name : "";
      alloc_bytes = entry->stack_alloc_bytes;
      has_alloc = true;
      last_free_bytes = entry->last_free_bytes;
      has_last_free = true;
      min_free_bytes = entry->min_free_bytes;
      if (min_free_bytes > alloc_bytes) {
        min_free_bytes = alloc_bytes;
      }
      has_min_free = true;
      peak_used_bytes = alloc_bytes - min_free_bytes;
      has_peak_used = true;
      recommended_bytes = RoundUpBytes(peak_used_bytes + headroom_bytes, 256U);
      has_recommended = true;
    } else if (row->snapshot_entry != NULL) {
      name = row->snapshot_entry->pcTaskName;
    }

    if (row->snapshot_entry != NULL) {
      last_free_bytes = (uint32_t)row->snapshot_entry->usStackHighWaterMark;
      has_last_free = true;
    }

    char alloc_buf[16];
    char last_free_buf[16];
    char min_free_buf[16];
    char peak_buf[16];
    char recommended_buf[16];
    char pri_buf[8];
    FormatValueOrNa(alloc_buf, sizeof(alloc_buf), has_alloc, alloc_bytes);
    FormatValueOrNa(
      last_free_buf, sizeof(last_free_buf), has_last_free, last_free_bytes);
    FormatValueOrNa(
      min_free_buf, sizeof(min_free_buf), has_min_free, min_free_bytes);
    FormatValueOrNa(peak_buf, sizeof(peak_buf), has_peak_used, peak_used_bytes);
    FormatValueOrNa(recommended_buf,
                    sizeof(recommended_buf),
                    has_recommended,
                    recommended_bytes);

    const char* state = "-";
    if (row->snapshot_entry != NULL) {
      state = TaskStateToString(row->snapshot_entry->eCurrentState);
      snprintf(pri_buf,
               sizeof(pri_buf),
               "%u",
               (unsigned)row->snapshot_entry->uxCurrentPriority);
    } else {
      snprintf(pri_buf, sizeof(pri_buf), "%s", "n/a");
    }

    printf("%-*.*s %6s %10s %10s %10s %10s %12s %8s %4s\n",
           task_w,
           task_w,
           name,
           row->is_tracked ? "tracked" : "dynamic",
           alloc_buf,
           last_free_buf,
           min_free_buf,
           peak_buf,
           recommended_buf,
           state,
           pri_buf);
  }
}
