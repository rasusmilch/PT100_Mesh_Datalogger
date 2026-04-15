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
    bool sample_valid;
  } stack_monitor_entry_t;

  typedef struct
  {
    stack_monitor_entry_t entries[kStackMonitorMaxEntries];
    size_t entry_count;
    uint32_t sample_period_ms;
    TickType_t last_sample_ticks;
  } stack_monitor_t;

  /**
   * @brief Initialize the deterministic monitored-task registry.
   *
   * This zero-initializes the monitor and loads the static monitored-task
   * table used by StackMonitorPrint().
   *
   * @param monitor Pointer to the monitor to initialize; ignored if NULL.
   * @param sample_period_ms Minimum elapsed time in milliseconds between
   *                         samples.
   */
  void StackMonitorInit(stack_monitor_t* monitor, uint32_t sample_period_ms);

  /**
   * @brief Bind a monitored task name to its TaskHandle_t storage pointer.
   *
   * The task name must exist in the built-in monitored-task table.
   *
   * @param monitor Pointer to the monitor that owns the registry.
   * @param name Monitored task name.
   * @param handle_ptr Pointer to a TaskHandle_t variable; sampling is skipped
   *                   when *handle_ptr is NULL.
   * @param stack_alloc_bytes Configured stack allocation in bytes for the
   *                          task. A value of 0 keeps the table default.
   * @return true if the monitored task exists and was bound.
   */
  bool StackMonitorRegister(stack_monitor_t* monitor,
                            const char* name,
                            TaskHandle_t* handle_ptr,
                            uint32_t stack_alloc_bytes);

  /**
   * @brief Sample monitored tasks if the sampling period has elapsed.
   *
   * Updates each monitored entry's last_free_bytes and min_free_bytes using
   * the FreeRTOS stack high-water mark API.
   *
   * @param monitor Pointer to the monitor to sample; ignored if NULL.
   */
  void StackMonitorMaybeSample(stack_monitor_t* monitor);

  /**
   * @brief Lookup the minimum free stack bytes for a monitored task.
   *
   * @param monitor Pointer to the monitor registry; ignored if NULL.
   * @param name Task name to search for; ignored if NULL.
   * @param out_bytes Output pointer for the minimum free bytes.
   * @return true if a matching entry has valid sampled data.
   */
  bool StackMonitorGetMinFreeBytes(const stack_monitor_t* monitor,
                                   const char* name,
                                   uint32_t* out_bytes);

  /**
   * @brief Print a monitored-task stack report.
   *
   * The report includes configured allocation, last and minimum observed free
   * bytes, peak used bytes, and recommended allocation based on observed
   * peak usage plus requested headroom.
   *
   * @param monitor Pointer to the monitor to print; ignored if NULL.
   * @param headroom_bytes Additional margin applied to observed peak usage
   *                       before rounding recommendation to 256-byte boundary.
   */
  void StackMonitorPrint(const stack_monitor_t* monitor,
                         uint32_t headroom_bytes);

#ifdef __cplusplus
}
#endif

#endif  // PT100_LOGGER_STACK_MONITOR_H_
