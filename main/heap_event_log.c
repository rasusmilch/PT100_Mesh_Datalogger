#include "heap_event_log.h"

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char* kTag = "heap_event";

static void SampleCaps(uint32_t caps, multi_heap_info_t* out_info);
static uint8_t ComputeFragPercent(const multi_heap_info_t* info);
static void ComputeWarnCrit(uint32_t largest_free,
                            uint8_t frag_pct,
                            bool* warn_out,
                            bool* crit_out);

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
ComputeWarnCrit(uint32_t largest_free,
                uint8_t frag_pct,
                bool* warn_out,
                bool* crit_out)
{
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
  if (warn_out != NULL) {
    *warn_out = warn;
  }
  if (crit_out != NULL) {
    *crit_out = crit;
  }
}

void
HeapEventLog(const char* event_name, const char* detail, int32_t code)
{
  multi_heap_info_t internal_info;
  SampleCaps(MALLOC_CAP_INTERNAL, &internal_info);

  static uint32_t s_internal_min_largest = 0u;
  if (s_internal_min_largest == 0u ||
      internal_info.largest_free_block < s_internal_min_largest) {
    s_internal_min_largest = internal_info.largest_free_block;
  }

  const uint8_t internal_frag = ComputeFragPercent(&internal_info);
  bool internal_warn = false;
  bool internal_crit = false;
  ComputeWarnCrit(internal_info.largest_free_block,
                  internal_frag,
                  &internal_warn,
                  &internal_crit);

  const char* event_label = (event_name != NULL) ? event_name : "unknown";
  const char* detail_label = (detail != NULL) ? detail : "n/a";
  ESP_LOGI(kTag,
           "event[%s] detail=%s code=%" PRId32
           " internal_free=%u internal_largest=%u internal_min_free=%u "
           "internal_min_largest=%u internal_frag=%u%% internal_warn=%u "
           "internal_crit=%u",
           event_label,
           detail_label,
           code,
           (unsigned)internal_info.total_free_bytes,
           (unsigned)internal_info.largest_free_block,
           (unsigned)internal_info.minimum_free_bytes,
           (unsigned)s_internal_min_largest,
           (unsigned)internal_frag,
           internal_warn ? 1u : 0u,
           internal_crit ? 1u : 0u);

#if CONFIG_SPIRAM
  multi_heap_info_t psram_info;
  SampleCaps(MALLOC_CAP_SPIRAM, &psram_info);

  static uint32_t s_psram_min_largest = 0u;
  if (s_psram_min_largest == 0u ||
      psram_info.largest_free_block < s_psram_min_largest) {
    s_psram_min_largest = psram_info.largest_free_block;
  }

  const uint8_t psram_frag = ComputeFragPercent(&psram_info);
  bool psram_warn = false;
  bool psram_crit = false;
  ComputeWarnCrit(psram_info.largest_free_block,
                  psram_frag,
                  &psram_warn,
                  &psram_crit);

  ESP_LOGI(kTag,
           "event[%s] detail=%s code=%" PRId32
           " psram_free=%u psram_largest=%u psram_min_free=%u "
           "psram_min_largest=%u psram_frag=%u%% psram_warn=%u psram_crit=%u",
           event_label,
           detail_label,
           code,
           (unsigned)psram_info.total_free_bytes,
           (unsigned)psram_info.largest_free_block,
           (unsigned)psram_info.minimum_free_bytes,
           (unsigned)s_psram_min_largest,
           (unsigned)psram_frag,
           psram_warn ? 1u : 0u,
           psram_crit ? 1u : 0u);
#endif
}
