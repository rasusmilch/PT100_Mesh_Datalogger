#include "sd_storage_scratch.h"

#include <stdlib.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#include "esp_heap_caps.h"
#include "esp_log.h"
#else
#define ESP_LOGE(tag, fmt, ...) ((void)(tag))
#define ESP_LOGW(tag, fmt, ...) ((void)(tag))
#endif

static const char* kTag = "sd_storage_scratch";

#if !defined(ESP_PLATFORM)
static bool s_host_force_allocation_failure = false;

void
SdStorageScratchHostSetAllocationFailure(bool fail)
{
  s_host_force_allocation_failure = fail;
}
#endif

static void*
StorageScratchAlloc(size_t bytes)
{
#if defined(ESP_PLATFORM)
  return heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
  if (s_host_force_allocation_failure) {
    return NULL;
  }
  return malloc(bytes);
#endif
}

static void
StorageScratchFree(void* ptr)
{
  free(ptr);
}

esp_err_t
SdStorageScratchInit(sd_storage_scratch_owner_t* owner)
{
  if (owner == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(owner, 0, sizeof(*owner));
  owner->slot_count = SD_STORAGE_SCRATCH_SLOT_COUNT;
  owner->init_result = ESP_ERR_INVALID_STATE;

  if (owner->slot_count == 0u) {
    owner->last_failure = "slot_count";
    owner->init_result = ESP_ERR_INVALID_SIZE;
    ESP_LOGE(kTag, "storage scratch slot count is zero");
    return owner->init_result;
  }

  const size_t bytes = owner->slot_count * sizeof(sd_storage_scratch_slot_t);
  owner->slots = (sd_storage_scratch_slot_t*)StorageScratchAlloc(bytes);
  if (owner->slots == NULL) {
    owner->last_failure = "slots_psram";
    owner->allocation_bytes = bytes;
    owner->init_result = ESP_ERR_NO_MEM;
    ESP_LOGE(kTag,
             "failed to allocate storage scratch slots from PSRAM: bytes=%u slots=%u",
             (unsigned)bytes,
             (unsigned)owner->slot_count);
    return owner->init_result;
  }

  memset(owner->slots, 0, bytes);
  owner->allocation_bytes = bytes;
  owner->initialized = true;
  owner->init_result = ESP_OK;
  return ESP_OK;
}

void
SdStorageScratchDeinit(sd_storage_scratch_owner_t* owner)
{
  if (owner == NULL) {
    return;
  }
  StorageScratchFree(owner->slots);
  memset(owner, 0, sizeof(*owner));
}

sd_storage_scratch_slot_t*
SdStorageScratchBorrow(sd_storage_scratch_owner_t* owner, const char* purpose)
{
  if (owner == NULL || !owner->initialized || owner->slots == NULL ||
      owner->slot_count == 0u) {
    if (owner != NULL) {
      owner->last_failure = (purpose != NULL) ? purpose : "borrow_uninitialized";
      owner->borrow_failures++;
    }
    ESP_LOGW(kTag,
             "storage scratch borrow failed before initialization: purpose=%s",
             purpose != NULL ? purpose : "unknown");
    return NULL;
  }
  if (owner->in_use[0]) {
    owner->last_failure = (purpose != NULL) ? purpose : "borrow_in_use";
    owner->borrow_failures++;
    ESP_LOGW(kTag,
             "storage scratch double-borrow rejected: purpose=%s",
             purpose != NULL ? purpose : "unknown");
    return NULL;
  }

  owner->in_use[0] = true;
  memset(&owner->slots[0], 0, sizeof(owner->slots[0]));
  return &owner->slots[0];
}

void
SdStorageScratchRelease(sd_storage_scratch_owner_t* owner,
                        sd_storage_scratch_slot_t* slot)
{
  if (owner == NULL || slot == NULL || owner->slots == NULL ||
      slot != &owner->slots[0] || !owner->in_use[0]) {
    if (owner != NULL) {
      owner->last_failure = "release_invalid";
      owner->borrow_failures++;
    }
    ESP_LOGW(kTag, "invalid storage scratch release rejected");
    return;
  }
  owner->in_use[0] = false;
}
