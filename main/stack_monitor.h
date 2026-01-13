#ifndef PT100_LOGGER_STACK_MONITOR_H_
#define PT100_LOGGER_STACK_MONITOR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

  enum
  {
    kStackMonitorMaxEntries = 16,
  };

  typedef struct
  {
    const char* task_name;
    TaskHandle_t* task_handle_ptr;
    uint32_t stack_alloc_bytes;
    uint32_t last_free_bytes;
    uint32_t min_free_bytes;
  } stack_monitor_entry_t;

  typedef struct
  {
    stack_monitor_entry_t entries[kStackMonitorMaxEntries];
    size_t entry_count;
    uint32_t sample_period_ms;
    TickType_t last_sample_ticks;
  } stack_monitor_t;

  void StackMonitorInit(stack_monitor_t* monitor, uint32_t sample_period_ms);
  bool StackMonitorRegister(stack_monitor_t* monitor,
                            const char* name,
                            TaskHandle_t* handle_ptr,
                            uint32_t stack_alloc_bytes);
  void StackMonitorMaybeSample(stack_monitor_t* monitor);
  void StackMonitorPrint(const stack_monitor_t* monitor, uint32_t headroom_bytes);

#ifdef __cplusplus
}
#endif

#endif  // PT100_LOGGER_STACK_MONITOR_H_
