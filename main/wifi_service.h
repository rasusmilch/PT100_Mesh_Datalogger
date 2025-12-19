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

// Starts the Wi-Fi service for the requested mode. Returns
// ESP_ERR_INVALID_STATE when another mode is already active.
esp_err_t WifiServiceStart(wifi_service_mode_t mode);

// Stops the active Wi-Fi service mode without deinitializing esp_wifi.
esp_err_t WifiServiceStop(void);

// Returns the currently active service mode.
wifi_service_mode_t WifiServiceActiveMode(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_WIFI_SERVICE_H_
