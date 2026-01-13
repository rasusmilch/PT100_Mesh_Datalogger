#include "stack_monitor.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char* kTag = "stack_monitor";

static uint32_t
RoundUpBytes(uint32_t value, uint32_t alignment)
{
  if (alignment == 0) {
    return value;
  }
  return (value + alignment - 1U) / alignment * alignment;
}

static uint32_t
GetStackHighWaterMarkBytes(TaskHandle_t handle)
{
#if defined(INCLUDE_uxTaskGetStackHighWaterMark2) &&
  (INCLUDE_uxTaskGetStackHighWaterMark2 == 1)
  return (uint32_t)uxTaskGetStackHighWaterMark2(handle);
#else
  return (uint32_t)uxTaskGetStackHighWaterMark(handle);
#endif
}

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

void
StackMonitorPrint(const stack_monitor_t* monitor, uint32_t headroom_bytes)
{
  if (monitor == NULL) {
    return;
  }

  printf("task\talloc\tlast_free\tmin_free\tpeak_used\trecommended\n");
  for (size_t i = 0; i < monitor->entry_count; ++i) {
    const stack_monitor_entry_t* entry = &monitor->entries[i];
    uint32_t min_free_bytes = entry->min_free_bytes;
    if (min_free_bytes > entry->stack_alloc_bytes) {
      min_free_bytes = entry->stack_alloc_bytes;
    }
    const uint32_t peak_used_bytes =
      entry->stack_alloc_bytes - min_free_bytes;
    const uint32_t recommended_bytes =
      RoundUpBytes(peak_used_bytes + headroom_bytes, 256U);

    printf("%s\t%" PRIu32 "\t%" PRIu32 "\t%" PRIu32 "\t%" PRIu32
           "\t%" PRIu32 "\n",
           entry->task_name,
           entry->stack_alloc_bytes,
           entry->last_free_bytes,
           min_free_bytes,
           peak_used_bytes,
           recommended_bytes);
  }
}
