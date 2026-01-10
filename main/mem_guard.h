#ifndef PT100_LOGGER_MEM_GUARD_H_
#define PT100_LOGGER_MEM_GUARD_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum
  {
    MEM_GUARD_PHASE_BOOT = 0,
    MEM_GUARD_PHASE_RUN
  } mem_guard_phase_t;

  void MemGuardInit(void);
  void MemGuardSetPhase(mem_guard_phase_t phase);
  mem_guard_phase_t MemGuardGetPhase(void);
  uint64_t MemGuardGetAllocCountSinceBoot(void);
  uint64_t MemGuardGetAllocCountSinceRun(void);

  void* AppMalloc(size_t size);
  void* AppCalloc(size_t count, size_t size);
  void* AppRealloc(void* ptr, size_t size);
  void AppFree(void* ptr);

#ifdef __cplusplus
}
#endif

#endif  // PT100_LOGGER_MEM_GUARD_H_
