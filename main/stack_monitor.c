#include "stack_monitor.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char* kTag = "stack_monitor";

typedef struct
{
  const char* task_name;
  uint32_t stack_alloc_bytes;
} stack_monitor_template_entry_t;

static uint32_t RoundUpBytes(uint32_t value, uint32_t alignment);
static uint32_t GetStackHighWaterMarkBytes(TaskHandle_t handle);
static bool FindEntryIndexByName(const stack_monitor_t* monitor,
                                 const char* name,
                                 size_t* out_index);
static void
FormatValueOrNa(char* buffer, size_t buffer_size, bool known, uint32_t value);

static const stack_monitor_template_entry_t kStackMonitorEntries[] = {
  { "display", 4608U },
  { "control", 8192U },
  { "sensor", 4608U },
  { "storage", 6144U },
  { "sd_flush", 8192U },
  { "export", 3584U },
  { "net_tx", 8192U },
  { "alert_http", 11264U },
};

static uint32_t
RoundUpBytes(uint32_t value, uint32_t alignment)
{
  uint32_t quotient = 0U;
  uint32_t max_aligned = 0U;

  if (alignment == 0U) {
    return value;
  }
  quotient = UINT32_MAX / alignment;
  max_aligned = quotient * alignment;
  if (value > (UINT32_MAX - (alignment - 1U))) {
    return max_aligned;
  }
  return ((value + alignment - 1U) / alignment) * alignment;
}

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

static bool
FindEntryIndexByName(const stack_monitor_t* monitor,
                     const char* name,
                     size_t* out_index)
{
  if (monitor == NULL || name == NULL) {
    return false;
  }
  for (size_t i = 0; i < monitor->entry_count; ++i) {
    const stack_monitor_entry_t* entry = &monitor->entries[i];
    if (entry->task_name == NULL) {
      continue;
    }
    if (strcmp(entry->task_name, name) == 0) {
      if (out_index != NULL) {
        *out_index = i;
      }
      return true;
    }
  }
  return false;
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

void
StackMonitorInit(stack_monitor_t* monitor, uint32_t sample_period_ms)
{
  if (monitor == NULL) {
    return;
  }

  memset(monitor, 0, sizeof(*monitor));
  monitor->sample_period_ms = sample_period_ms;

  const size_t template_count =
    sizeof(kStackMonitorEntries) / sizeof(kStackMonitorEntries[0]);
  const size_t entry_count =
    (template_count < kStackMonitorMaxEntries) ? template_count
                                               : kStackMonitorMaxEntries;
  for (size_t i = 0; i < entry_count; ++i) {
    const stack_monitor_template_entry_t* source = &kStackMonitorEntries[i];
    stack_monitor_entry_t* entry = &monitor->entries[i];
    entry->task_name = source->task_name;
    entry->task_handle_ptr = NULL;
    entry->stack_alloc_bytes = source->stack_alloc_bytes;
    entry->last_free_bytes = source->stack_alloc_bytes;
    entry->min_free_bytes = source->stack_alloc_bytes;
    entry->sample_valid = false;
  }
  monitor->entry_count = entry_count;
}

bool
StackMonitorRegister(stack_monitor_t* monitor,
                     const char* name,
                     TaskHandle_t* handle_ptr,
                     uint32_t stack_alloc_bytes)
{
  if (monitor == NULL || name == NULL || handle_ptr == NULL) {
    return false;
  }

  size_t entry_index = 0;
  if (!FindEntryIndexByName(monitor, name, &entry_index)) {
    return false;
  }

  stack_monitor_entry_t* entry = &monitor->entries[entry_index];
  entry->task_handle_ptr = handle_ptr;
  if (stack_alloc_bytes != 0U) {
    entry->stack_alloc_bytes = stack_alloc_bytes;
  }
  if (!entry->sample_valid) {
    entry->last_free_bytes = entry->stack_alloc_bytes;
    entry->min_free_bytes = entry->stack_alloc_bytes;
  }
  return true;
}

void
StackMonitorMaybeSample(stack_monitor_t* monitor)
{
  if (monitor == NULL || monitor->entry_count == 0U) {
    return;
  }

  const TickType_t now_ticks = xTaskGetTickCount();
  if (monitor->last_sample_ticks != 0U) {
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
    if (!entry->sample_valid || hwm_bytes < entry->min_free_bytes) {
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
    entry->sample_valid = true;
  }
}

bool
StackMonitorGetMinFreeBytes(const stack_monitor_t* monitor,
                            const char* name,
                            uint32_t* out_bytes)
{
  if (monitor == NULL || name == NULL || out_bytes == NULL) {
    return false;
  }

  size_t entry_index = 0;
  if (!FindEntryIndexByName(monitor, name, &entry_index)) {
    return false;
  }

  const stack_monitor_entry_t* entry = &monitor->entries[entry_index];
  if (!entry->sample_valid) {
    return false;
  }

  *out_bytes = entry->min_free_bytes;
  return true;
}

void
StackMonitorPrint(const stack_monitor_t* monitor, uint32_t headroom_bytes)
{
  if (monitor == NULL) {
    return;
  }

  printf("note: reporting monitored tasks only (static registry)\n");
  printf("note: recommended = round_up(peak_used + headroom, 256)\n");
  printf("%-12s %10s %10s %10s %10s %12s\n",
         "task",
         "alloc",
         "last_free",
         "min_free",
         "peak_used",
         "recommended");

  for (size_t i = 0; i < monitor->entry_count; ++i) {
    const stack_monitor_entry_t* entry = &monitor->entries[i];
    const bool has_runtime =
      (entry->sample_valid && entry->task_handle_ptr != NULL &&
       *entry->task_handle_ptr != NULL);

    uint32_t min_free_bytes = entry->min_free_bytes;
    if (min_free_bytes > entry->stack_alloc_bytes) {
      min_free_bytes = entry->stack_alloc_bytes;
    }

    char alloc_buf[16];
    char last_free_buf[16];
    char min_free_buf[16];
    char peak_buf[16];
    char recommended_buf[16];

    const uint32_t peak_used_bytes = has_runtime
                                       ? (entry->stack_alloc_bytes - min_free_bytes)
                                       : 0U;
    uint32_t recommended_bytes = 0U;
    if (has_runtime) {
      uint32_t recommended_target = peak_used_bytes;
      if (recommended_target > (UINT32_MAX - headroom_bytes)) {
        recommended_target = UINT32_MAX;
      } else {
        recommended_target += headroom_bytes;
      }
      recommended_bytes = RoundUpBytes(recommended_target, 256U);
    }

    FormatValueOrNa(alloc_buf, sizeof(alloc_buf), true, entry->stack_alloc_bytes);
    FormatValueOrNa(
      last_free_buf, sizeof(last_free_buf), has_runtime, entry->last_free_bytes);
    FormatValueOrNa(min_free_buf, sizeof(min_free_buf), has_runtime, min_free_bytes);
    FormatValueOrNa(peak_buf, sizeof(peak_buf), has_runtime, peak_used_bytes);
    FormatValueOrNa(
      recommended_buf, sizeof(recommended_buf), has_runtime, recommended_bytes);

    printf("%-12s %10s %10s %10s %10s %12s\n",
           (entry->task_name != NULL) ? entry->task_name : "",
           alloc_buf,
           last_free_buf,
           min_free_buf,
           peak_buf,
           recommended_buf);
  }
}
