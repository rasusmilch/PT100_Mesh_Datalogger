#include "runtime_health.h"

#include <string.h>

/**
 * @brief Execute RuntimeHealthInit.
 * @param cache Parameter cache.
 */
void
RuntimeHealthInit(runtime_health_cache_t* cache)
{
  if (cache == NULL) {
    return;
  }
  cache->version = 0;
  memset(&cache->snapshot, 0, sizeof(cache->snapshot));
}

/**
 * @brief Execute RuntimeHealthPublish.
 * @param cache Parameter cache.
 * @param new_snapshot Parameter new_snapshot.
 */
void
RuntimeHealthPublish(runtime_health_cache_t* cache,
                     const runtime_health_snapshot_t* new_snapshot)
{
  if (cache == NULL || new_snapshot == NULL) {
    return;
  }

  cache->version++;
  memcpy(&cache->snapshot, new_snapshot, sizeof(cache->snapshot));
  cache->version++;
}

/**
 * @brief Execute RuntimeHealthRead.
 * @param cache Parameter cache.
 * @param out_snapshot Parameter out_snapshot.
 */
void
RuntimeHealthRead(const runtime_health_cache_t* cache,
                  runtime_health_snapshot_t* out_snapshot)
{
  if (cache == NULL || out_snapshot == NULL) {
    return;
  }

  while (true) {
    const uint32_t start_version = cache->version;
    if ((start_version & 1u) != 0u) {
      continue;
    }
    memcpy(out_snapshot, &cache->snapshot, sizeof(*out_snapshot));
    const uint32_t end_version = cache->version;
    if (start_version == end_version && (end_version & 1u) == 0u) {
      return;
    }
  }
}
