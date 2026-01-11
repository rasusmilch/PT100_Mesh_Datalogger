#include "mem_guard.h"

#include <stdlib.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const uint32_t kMemGuardCaps = MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT;
static const char* kTag = "mem_guard";

static mem_guard_phase_t s_phase = MEM_GUARD_PHASE_BOOT;
static uint64_t s_alloc_count_since_boot = 0;
static uint64_t s_alloc_count_since_run = 0;
static int64_t s_last_run_alloc_log_ms = 0;

void
MemGuardInit(void)
{
  s_phase = MEM_GUARD_PHASE_BOOT;
  s_alloc_count_since_boot = 0;
  s_alloc_count_since_run = 0;
  s_last_run_alloc_log_ms = 0;
}

void
MemGuardSetPhase(mem_guard_phase_t phase)
{
  s_phase = phase;
  if (phase == MEM_GUARD_PHASE_RUN || phase == MEM_GUARD_PHASE_BOOT) {
    s_alloc_count_since_run = 0;
  }
}

mem_guard_phase_t
MemGuardGetPhase(void)
{
  return s_phase;
}

uint64_t
MemGuardGetAllocCountSinceBoot(void)
{
  return s_alloc_count_since_boot;
}

uint64_t
MemGuardGetAllocCountSinceRun(void)
{
  return s_alloc_count_since_run;
}

#if CONFIG_APP_MEM_GUARD_ENABLE
static void
MemGuardHandleRunAlloc(void)
{
#if CONFIG_APP_MEM_GUARD_LOG_ENABLE
  const int64_t now_ms = esp_timer_get_time() / 1000;
  const int64_t rate_limit_ms = CONFIG_APP_MEM_GUARD_LOG_RATE_LIMIT_MS;
  if (rate_limit_ms <= 0 || s_last_run_alloc_log_ms == 0 ||
      (now_ms - s_last_run_alloc_log_ms) >= rate_limit_ms) {
    ESP_LOGW(kTag, "Allocation during RUN phase detected.");
    s_last_run_alloc_log_ms = now_ms;
  }
#endif

#if CONFIG_APP_MEM_GUARD_STRICT
  ESP_LOGE(kTag, "Strict mem guard active; aborting on RUN allocation.");
  abort();
#endif
}
#endif

static void
MemGuardTrackAlloc(void* ptr)
{
#if CONFIG_APP_MEM_GUARD_ENABLE
  if (ptr == NULL) {
    return;
  }

  s_alloc_count_since_boot += 1;
  if (s_phase == MEM_GUARD_PHASE_RUN) {
    s_alloc_count_since_run += 1;
    MemGuardHandleRunAlloc();
  }
#else
  (void)ptr;
#endif
}

void*
AppMalloc(size_t size)
{
#if CONFIG_APP_MEM_GUARD_ENABLE
  void* ptr = heap_caps_malloc(size, kMemGuardCaps);
  MemGuardTrackAlloc(ptr);
  return ptr;
#else
  return heap_caps_malloc(size, kMemGuardCaps);
#endif
}

void*
AppCalloc(size_t count, size_t size)
{
#if CONFIG_APP_MEM_GUARD_ENABLE
  void* ptr = heap_caps_calloc(count, size, kMemGuardCaps);
  MemGuardTrackAlloc(ptr);
  return ptr;
#else
  return heap_caps_calloc(count, size, kMemGuardCaps);
#endif
}

void*
AppRealloc(void* ptr, size_t size)
{
#if CONFIG_APP_MEM_GUARD_ENABLE
  void* new_ptr = heap_caps_realloc(ptr, size, kMemGuardCaps);
  if (size > 0) {
    MemGuardTrackAlloc(new_ptr);
  }
  return new_ptr;
#else
  return heap_caps_realloc(ptr, size, kMemGuardCaps);
#endif
}

void
AppFree(void* ptr)
{
  if (ptr == NULL) {
    return;
  }
  heap_caps_free(ptr);
}
