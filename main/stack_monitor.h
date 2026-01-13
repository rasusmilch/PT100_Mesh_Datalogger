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

  /**
  * @brief Initialize a stack monitor instance.
   *
   * This zero-initializes the monitor object and stores the sampling period
   * used by StackMonitorMaybeSample().
   *
   * @param monitor Pointer to the monitor to initialize; ignored if NULL.
   * @param sample_period_ms Minimum elapsed time in milliseconds between
   *                         samples.
   */
  void StackMonitorInit(stack_monitor_t* monitor, uint32_t sample_period_ms);
  
  /**
   * @brief Register or update a task entry for stack sampling.
   *
   * Entries are keyed by the address of @p handle_ptr.
   *
   * @param monitor Pointer to the monitor that owns the registry.
   * @param name Task name string used in StackMonitorPrint() output.
   * @param handle_ptr Pointer to a TaskHandle_t variable; sampling is skipped
   *                   when *handle_ptr is NULL.
   * @param stack_alloc_bytes Configured stack allocation in bytes for the
   *                          task.
   * @return true if the entry was added or updated; false if arguments are
   *         NULL or the registry is full.
   */
  bool StackMonitorRegister(stack_monitor_t* monitor,
                            const char* name,
                            TaskHandle_t* handle_ptr,
                            uint32_t stack_alloc_bytes);

  /**
   * @brief Sample registered tasks if the sampling period has elapsed.
   *
   * Updates each entry's last_free_bytes and min_free_bytes using the FreeRTOS
   * stack high-water mark API.
   *
   * @param monitor Pointer to the monitor to sample; ignored if NULL.
   */
  void StackMonitorMaybeSample(stack_monitor_t* monitor);

  /**
   * @brief Print a tab-separated report for registered tasks.
   *
   * The report includes configured allocation, last and minimum observed free
   * bytes, peak used bytes, and a recommended stack size computed as
   * (peak_used + headroom) rounded up to 256 bytes.
   *
   * @param monitor Pointer to the monitor to print; ignored if NULL.
   * @param headroom_bytes Extra bytes added to peak usage when computing the
   *                       recommended stack size.
   */
  void StackMonitorPrint(const stack_monitor_t* monitor, uint32_t headroom_bytes);

#ifdef __cplusplus
}
#endif

#endif  // PT100_LOGGER_STACK_MONITOR_H_
