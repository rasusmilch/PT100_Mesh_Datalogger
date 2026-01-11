#ifndef PT100_LOGGER_HEAP_MONITOR_H_
#define PT100_LOGGER_HEAP_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    uint32_t last_sample_ticks;
    uint32_t internal_min_largest_free_block_bytes;
    uint32_t psram_min_largest_free_block_bytes;

    uint32_t crit_consecutive;
    bool internal_warn_active;
    bool internal_crit_active;
    bool psram_warn_active;
    bool psram_crit_active;
  } heap_monitor_t;

  struct runtime_state_t;

  void HeapMonitorInit(heap_monitor_t* monitor);
  void HeapMonitorMaybeSample(struct runtime_state_t* state,
                              uint32_t now_ticks);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_HEAP_MONITOR_H_
