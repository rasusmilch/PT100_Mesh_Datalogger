#include "heap_monitor.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "runtime_health_publisher.h"
#include "runtime_manager.h"
#include "runtime_state.h"

static const char* kTag __attribute__((unused)) = "heap_monitor";

static void
SampleCaps(uint32_t caps, multi_heap_info_t* out_info)
{
  if (out_info == NULL) {
    return;
  }
  memset(out_info, 0, sizeof(*out_info));
  heap_caps_get_info(out_info, caps);
}

static uint8_t
ComputeFragPercent(const multi_heap_info_t* info)
{
  if (info == NULL || info->total_free_bytes == 0u) {
    return 100u;
  }
  const uint32_t largest = info->largest_free_block;
  const uint32_t total = info->total_free_bytes;
  const uint32_t pct = (largest * 100u) / total;
  if (pct >= 100u) {
    return 0u;
  }
  return (uint8_t)(100u - pct);
}

static void
UpdateCachedU32(runtime_state_t* state, uint32_t* dest, uint32_t value)
{
  if (state == NULL || dest == NULL) {
    return;
  }
  if (*dest != value) {
    *dest = value;
    RuntimeHealthMarkDirty(state);
  }
}

static void
UpdateCachedU8(runtime_state_t* state, uint8_t* dest, uint8_t value)
{
  if (state == NULL || dest == NULL) {
    return;
  }
  if (*dest != value) {
    *dest = value;
    RuntimeHealthMarkDirty(state);
  }
}

static void
UpdateCachedBool(runtime_state_t* state, bool* dest, bool value)
{
  if (state == NULL || dest == NULL) {
    return;
  }
  if (*dest != value) {
    *dest = value;
    RuntimeHealthMarkDirty(state);
  }
}

void
HeapMonitorInit(heap_monitor_t* monitor)
{
  if (monitor == NULL) {
    return;
  }
  memset(monitor, 0, sizeof(*monitor));
}

static void
UpdateInternalHeapStatus(runtime_state_t* state,
                         heap_monitor_t* monitor,
                         const multi_heap_info_t* info)
{
  if (state == NULL || monitor == NULL || info == NULL) {
    return;
  }

  const uint32_t total_free = info->total_free_bytes;
  const uint32_t largest_free = info->largest_free_block;
  const uint32_t min_free = info->minimum_free_bytes;
  const uint8_t frag_pct = ComputeFragPercent(info);

  uint32_t min_largest = monitor->internal_min_largest_free_block_bytes;
  if (min_largest == 0u || largest_free < min_largest) {
    min_largest = largest_free;
    monitor->internal_min_largest_free_block_bytes = min_largest;
  }

  bool warn = false;
  bool crit = false;
  if (CONFIG_APP_HEAP_INTERNAL_LARGEST_WARN_BYTES > 0 &&
      largest_free <
        (uint32_t)CONFIG_APP_HEAP_INTERNAL_LARGEST_WARN_BYTES) {
    warn = true;
  }
  if (CONFIG_APP_HEAP_INTERNAL_LARGEST_CRIT_BYTES > 0 &&
      largest_free <
        (uint32_t)CONFIG_APP_HEAP_INTERNAL_LARGEST_CRIT_BYTES) {
    crit = true;
  }
  if (CONFIG_APP_HEAP_INTERNAL_FRAG_WARN_PCT > 0 &&
      frag_pct >= (uint8_t)CONFIG_APP_HEAP_INTERNAL_FRAG_WARN_PCT) {
    warn = true;
  }
  if (CONFIG_APP_HEAP_INTERNAL_FRAG_CRIT_PCT > 0 &&
      frag_pct >= (uint8_t)CONFIG_APP_HEAP_INTERNAL_FRAG_CRIT_PCT) {
    crit = true;
  }
  if (crit) {
    warn = true;
  }

  monitor->internal_warn_active = warn;
  monitor->internal_crit_active = crit;

  UpdateCachedU32(state, &state->cached_status.heap_internal_free_bytes,
                  total_free);
  UpdateCachedU32(state,
                  &state->cached_status.heap_internal_largest_free_block_bytes,
                  largest_free);
  UpdateCachedU32(state, &state->cached_status.heap_internal_min_free_bytes,
                  min_free);
  UpdateCachedU32(
    state, &state->cached_status.heap_internal_min_largest_free_block_bytes,
    min_largest);
  UpdateCachedU8(state, &state->cached_status.heap_internal_frag_percent,
                 frag_pct);
  UpdateCachedBool(state, &state->cached_status.heap_internal_warn, warn);
  UpdateCachedBool(state, &state->cached_status.heap_internal_crit, crit);

#if CONFIG_APP_HEAP_GUARD_ACTION_RUN_STOP || CONFIG_APP_HEAP_GUARD_ACTION_RESTART
  if (crit) {
    const uint32_t threshold =
      (uint32_t)CONFIG_APP_HEAP_GUARD_CRIT_CONSECUTIVE_SAMPLES;
    if (monitor->crit_consecutive < threshold) {
      monitor->crit_consecutive += 1u;
    }
    if (monitor->crit_consecutive == threshold) {
#if CONFIG_APP_HEAP_GUARD_ACTION_RUN_STOP
      ESP_LOGE(kTag,
               "Heap critical condition persists (%u samples). Requesting run "
               "stop.",
               (unsigned)threshold);
      RuntimeRequestRunStop();
#elif CONFIG_APP_HEAP_GUARD_ACTION_RESTART
      ESP_LOGE(kTag,
               "Heap critical condition persists (%u samples). Restarting.",
               (unsigned)threshold);
      esp_restart();
#endif
      monitor->crit_consecutive += 1u;
    }
  } else {
    monitor->crit_consecutive = 0u;
  }
#else
  monitor->crit_consecutive = 0u;
#endif
}

static void
UpdatePsramHeapStatus(runtime_state_t* state,
                      heap_monitor_t* monitor,
                      const multi_heap_info_t* info)
{
  if (state == NULL || monitor == NULL || info == NULL) {
    return;
  }

  const uint32_t total_free = info->total_free_bytes;
  const uint32_t largest_free = info->largest_free_block;
  const uint32_t min_free = info->minimum_free_bytes;
  const uint8_t frag_pct = ComputeFragPercent(info);

  uint32_t min_largest = monitor->psram_min_largest_free_block_bytes;
  if (min_largest == 0u || largest_free < min_largest) {
    min_largest = largest_free;
    monitor->psram_min_largest_free_block_bytes = min_largest;
  }

  bool warn = false;
  bool crit = false;
  if (CONFIG_APP_HEAP_INTERNAL_LARGEST_WARN_BYTES > 0 &&
      largest_free <
        (uint32_t)CONFIG_APP_HEAP_INTERNAL_LARGEST_WARN_BYTES) {
    warn = true;
  }
  if (CONFIG_APP_HEAP_INTERNAL_LARGEST_CRIT_BYTES > 0 &&
      largest_free <
        (uint32_t)CONFIG_APP_HEAP_INTERNAL_LARGEST_CRIT_BYTES) {
    crit = true;
  }
  if (CONFIG_APP_HEAP_INTERNAL_FRAG_WARN_PCT > 0 &&
      frag_pct >= (uint8_t)CONFIG_APP_HEAP_INTERNAL_FRAG_WARN_PCT) {
    warn = true;
  }
  if (CONFIG_APP_HEAP_INTERNAL_FRAG_CRIT_PCT > 0 &&
      frag_pct >= (uint8_t)CONFIG_APP_HEAP_INTERNAL_FRAG_CRIT_PCT) {
    crit = true;
  }
  if (crit) {
    warn = true;
  }

  monitor->psram_warn_active = warn;
  monitor->psram_crit_active = crit;

  UpdateCachedU32(state, &state->cached_status.heap_psram_free_bytes,
                  total_free);
  UpdateCachedU32(state,
                  &state->cached_status.heap_psram_largest_free_block_bytes,
                  largest_free);
  UpdateCachedU32(state, &state->cached_status.heap_psram_min_free_bytes,
                  min_free);
  UpdateCachedU32(state,
                  &state->cached_status.heap_psram_min_largest_free_block_bytes,
                  min_largest);
  UpdateCachedU8(state, &state->cached_status.heap_psram_frag_percent,
                 frag_pct);
  UpdateCachedBool(state, &state->cached_status.heap_psram_warn, warn);
  UpdateCachedBool(state, &state->cached_status.heap_psram_crit, crit);
}

void
HeapMonitorMaybeSample(runtime_state_t* state, uint32_t now_ticks)
{
  if (state == NULL) {
    return;
  }

  heap_monitor_t* monitor = &state->heap_monitor;
  const uint32_t period_ms = (uint32_t)CONFIG_APP_HEAP_MONITOR_PERIOD_MS;
  const uint32_t elapsed_ms =
    (monitor->last_sample_ticks == 0u)
      ? period_ms
      : (uint32_t)pdTICKS_TO_MS(now_ticks -
                                (TickType_t)monitor->last_sample_ticks);
  if (elapsed_ms < period_ms) {
    return;
  }
  monitor->last_sample_ticks = now_ticks;

  multi_heap_info_t info = { 0 };
  SampleCaps(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT, &info);
  UpdateInternalHeapStatus(state, monitor, &info);

#if CONFIG_APP_HEAP_PSRAM_MONITOR_ENABLE
  multi_heap_info_t psram_info = { 0 };
  SampleCaps(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, &psram_info);
  UpdatePsramHeapStatus(state, monitor, &psram_info);
#else
  monitor->psram_warn_active = false;
  monitor->psram_crit_active = false;
#endif
}
