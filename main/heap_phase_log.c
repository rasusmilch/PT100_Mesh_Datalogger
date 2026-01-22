#include "heap_phase_log.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char* kTag = "heap_phase";

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

void
HeapLogPhase(const char* tag)
{
  multi_heap_info_t internal_info;
  memset(&internal_info, 0, sizeof(internal_info));
  heap_caps_get_info(&internal_info, MALLOC_CAP_INTERNAL);
  const uint8_t internal_frag = ComputeFragPercent(&internal_info);
  ESP_LOGI(kTag,
           "phase[%s] internal_free=%u internal_largest=%u internal_frag=%u%%",
           (tag != NULL) ? tag : "unknown",
           (unsigned)internal_info.total_free_bytes,
           (unsigned)internal_info.largest_free_block,
           (unsigned)internal_frag);

#if CONFIG_SPIRAM
  multi_heap_info_t psram_info;
  memset(&psram_info, 0, sizeof(psram_info));
  heap_caps_get_info(&psram_info, MALLOC_CAP_SPIRAM);
  if (psram_info.total_free_bytes > 0u ||
      psram_info.largest_free_block > 0u) {
    const uint8_t psram_frag = ComputeFragPercent(&psram_info);
    ESP_LOGI(kTag,
             "phase[%s] psram_free=%u psram_largest=%u psram_frag=%u%%",
             (tag != NULL) ? tag : "unknown",
             (unsigned)psram_info.total_free_bytes,
             (unsigned)psram_info.largest_free_block,
             (unsigned)psram_frag);
  }
#endif
}
