#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"

static const char* kTag = "wifi_scan_wrap";

// Linker wrapper for esp_wifi_scan_start().
//
// Goal (Option B): scan all allowed channels.
//
// In practice, Mesh-Lite's internal scans can reach the Wi-Fi driver with a
// non-zero/garbage channel_bitmap. The driver validates the bitmap against the
// current regulatory/country config and emits warnings such as:
//   - "2g bitmap contains only invalid channels= ..."
//   - "In 2G band, 5G channel bitmap does not take effect"
//
// Without modifying the mesh_lite component, we wrap esp_wifi_scan_start() and
// force bitmap-mode scans to use channel_bitmap == 0 (scan all allowed
// channels) and always clear the 5G bitmap on 2.4 GHz-only platforms.

// NOLINTNEXTLINE(readability-identifier-naming)
esp_err_t
__real_esp_wifi_scan_start(const wifi_scan_config_t* config, bool block);

// NOLINTNEXTLINE(readability-identifier-naming)
esp_err_t
__wrap_esp_wifi_scan_start(const wifi_scan_config_t* config, bool block)
{
  // If caller passed NULL, preserve ESP-IDF default behavior.
  if (config == NULL) {
    return __real_esp_wifi_scan_start(NULL, block);
  }

  // Make a local copy we can safely sanitize.
  wifi_scan_config_t sanitized;
  memcpy(&sanitized, config, sizeof(sanitized));

  // Always clear 5 GHz bitmap.
  sanitized.channel_bitmap.ghz_5_channels = 0;

  // If channel == 0, the driver uses channel_bitmap to decide which channels to
  // scan. Option B: scan all allowed channels by clearing the 2.4 GHz bitmap
  // too.
  if (sanitized.channel == 0) {
    sanitized.channel_bitmap.ghz_2_channels = 0;
  }

  // Only log at DEBUG to avoid clutter.
  if ((config->channel_bitmap.ghz_2_channels !=
       sanitized.channel_bitmap.ghz_2_channels) ||
      (config->channel_bitmap.ghz_5_channels !=
       sanitized.channel_bitmap.ghz_5_channels)) {
    ESP_LOGD(kTag,
             "Sanitized scan bitmap: in(2g=0x%04x,5g=0x%08x,ch=%u) -> "
             "out(2g=0x%04x,5g=0x%08x,ch=%u)",
             (unsigned)config->channel_bitmap.ghz_2_channels,
             (unsigned)config->channel_bitmap.ghz_5_channels,
             (unsigned)config->channel,
             (unsigned)sanitized.channel_bitmap.ghz_2_channels,
             (unsigned)sanitized.channel_bitmap.ghz_5_channels,
             (unsigned)sanitized.channel);
  }

  return __real_esp_wifi_scan_start(&sanitized, block);
}
