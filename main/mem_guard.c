#include "mem_guard.h"

#include "esp_heap_caps.h"

static const uint32_t kMemGuardCaps = MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT;

static mem_guard_phase_t s_phase = MEM_GUARD_PHASE_BOOT;
static uint64_t s_alloc_count_since_boot = 0;
static uint64_t s_alloc_count_since_run = 0;

void
MemGuardInit(void)
{
  s_phase = MEM_GUARD_PHASE_BOOT;
  s_alloc_count_since_boot = 0;
  s_alloc_count_since_run = 0;
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

static void
MemGuardTrackAlloc(void* ptr)
{
  if (ptr == NULL) {
    return;
  }

  s_alloc_count_since_boot += 1;
  if (s_phase == MEM_GUARD_PHASE_RUN) {
    s_alloc_count_since_run += 1;
  }
}

void*
AppMalloc(size_t size)
{
  void* ptr = heap_caps_malloc(size, kMemGuardCaps);
  MemGuardTrackAlloc(ptr);
  return ptr;
}

void*
AppCalloc(size_t count, size_t size)
{
  void* ptr = heap_caps_calloc(count, size, kMemGuardCaps);
  MemGuardTrackAlloc(ptr);
  return ptr;
}

void*
AppRealloc(void* ptr, size_t size)
{
  void* new_ptr = heap_caps_realloc(ptr, size, kMemGuardCaps);
  if (size > 0) {
    MemGuardTrackAlloc(new_ptr);
  }
  return new_ptr;
}

void
AppFree(void* ptr)
{
  if (ptr == NULL) {
    return;
  }
  heap_caps_free(ptr);
}
