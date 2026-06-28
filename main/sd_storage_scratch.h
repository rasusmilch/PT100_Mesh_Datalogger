#ifndef PT100_LOGGER_SD_STORAGE_SCRATCH_H_
#define PT100_LOGGER_SD_STORAGE_SCRATCH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__has_include)
#if __has_include("esp_err.h")
#include "esp_err.h"
#else
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#endif
#else
#include "esp_err.h"
#endif
#include "sd_ptlog_paths.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SD_STORAGE_SCRATCH_SLOT_COUNT
#define SD_STORAGE_SCRATCH_SLOT_COUNT 1u
#endif

/**
 * @brief Named path/candidate scratch for one serialized SD storage workflow.
 *
 * The slot is PSRAM-owned by sd_storage_scratch_owner_t and borrowed by a
 * caller while the SD/logger operation is already serialized by the existing
 * storage task/lock model. It is intentionally typed for PTLOG path and
 * candidate work: it is not an allocator, byte pool, queue, or concurrency
 * framework. Path buffers use SD_PTLOG_MAX_PATH_LEN and must be treated as
 * temporary; callers must copy any data needed after release.
 */
typedef struct sd_storage_scratch_slot_t
{
  char path_a[SD_PTLOG_MAX_PATH_LEN];
  char path_b[SD_PTLOG_MAX_PATH_LEN];
  char path_c[SD_PTLOG_MAX_PATH_LEN];
  char date[16];
  char month[16];
  char name[SD_PTLOG_MAX_NAME_LEN];
  sd_ptlog_candidate_t candidate;
  sd_ptlog_candidate_t best_candidate;
} sd_storage_scratch_slot_t;

/**
 * @brief Centrally owned PSRAM scratch for reusable SD/PTLOG paths.
 *
 * SdStorageScratchInit() allocates the fixed slot array from PSRAM only on
 * target firmware. It never falls back to internal RAM for these generic
 * path/candidate buffers. The default slot count is one; double-borrow and
 * uninitialized use fail closed by returning NULL and recording diagnostics.
 * CSV resume/readback byte buffers and the internal DMA SD host bounce buffer
 * are intentionally outside this owner.
 */
typedef struct
{
  sd_storage_scratch_slot_t* slots;
  size_t slot_count;
  bool in_use[SD_STORAGE_SCRATCH_SLOT_COUNT];
  bool initialized;
  esp_err_t init_result;
  size_t allocation_bytes;
  const char* last_failure;
  uint32_t borrow_failures;
} sd_storage_scratch_owner_t;

/**
 * @brief Allocate the fixed storage scratch slots from PSRAM.
 *
 * @param owner Owner object embedded in the logger lifetime.
 * @return ESP_OK on success, ESP_ERR_NO_MEM on PSRAM allocation failure, or
 * ESP_ERR_INVALID_ARG when owner is NULL.
 *
 * @note Existing application code has no logger deinit path; this owner follows
 * the same application-lifetime allocation pattern as other sd_logger_t buffers.
 */
esp_err_t SdStorageScratchInit(sd_storage_scratch_owner_t* owner);

/**
 * @brief Release heap storage owned by the scratch owner.
 *
 * This is provided for host tests and any future logger deinit path. Current
 * firmware initialization treats the logger as application-lifetime storage.
 */
void SdStorageScratchDeinit(sd_storage_scratch_owner_t* owner);

/**
 * @brief Borrow the single typed storage scratch slot without blocking.
 *
 * @param owner Initialized owner.
 * @param purpose Short static diagnostic label for failure logging/accounting.
 * @return Borrowed slot, or NULL when uninitialized/already in use.
 *
 * @warning Callers must already be in the serialized SD/logger workflow. This
 * function only guards double-borrow and does not provide a semaphore, wait
 * queue, or multi-consumer scheduling.
 */
sd_storage_scratch_slot_t* SdStorageScratchBorrow(
  sd_storage_scratch_owner_t* owner, const char* purpose);

/**
 * @brief Release a previously borrowed scratch slot.
 *
 * Invalid releases fail closed by recording diagnostics and leaving ownership
 * unchanged. The slot contents become invalid for callers immediately after
 * release.
 */
void SdStorageScratchRelease(sd_storage_scratch_owner_t* owner,
                             sd_storage_scratch_slot_t* slot);

#if !defined(ESP_PLATFORM)
/** Host-test seam that forces the next scratch allocation attempts to fail. */
void SdStorageScratchHostSetAllocationFailure(bool fail);
#endif

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_STORAGE_SCRATCH_H_
