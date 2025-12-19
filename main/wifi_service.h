#ifndef PT100_LOGGER_WIFI_SERVICE_H_
#define PT100_LOGGER_WIFI_SERVICE_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum
  {
    WIFI_SERVICE_MODE_NONE = 0,
    WIFI_SERVICE_MODE_DIAGNOSTIC_STA,
    WIFI_SERVICE_MODE_MESH,
  } wifi_service_mode_t;

// Initializes shared Wi-Fi service state and the underlying network stack.
// Safe to call multiple times.
esp_err_t WifiServiceInitOnce(void);

// Acquires the Wi-Fi service for the requested mode, initializing and starting
// esp_wifi on first use. Returns ESP_ERR_INVALID_STATE if a different mode is
// already active.
esp_err_t WifiServiceAcquire(wifi_service_mode_t mode);

// Releases the Wi-Fi service. Stops esp_wifi when the last user releases.
esp_err_t WifiServiceRelease(void);

// Returns the currently active service mode.
wifi_service_mode_t WifiServiceActiveMode(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_WIFI_SERVICE_H_
