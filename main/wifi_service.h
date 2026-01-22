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
/**
 * @brief Execute WifiServiceInitOnce.
 * @return Return the function result.
 */
esp_err_t WifiServiceInitOnce(void);

// Acquires the Wi-Fi service for the requested mode, initializing and starting
// esp_wifi on first use. Returns ESP_ERR_INVALID_STATE if a different mode is
// already active.
/**
 * @brief Execute WifiServiceAcquire.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
esp_err_t WifiServiceAcquire(wifi_service_mode_t mode);

// Reserves the Wi-Fi driver early without starting radio/connect.
/**
 * @brief Execute WifiServiceReserveEarly.
 * @return Return the function result.
 */
esp_err_t WifiServiceReserveEarly(void);

// Releases the Wi-Fi service. Stops esp_wifi when the last user releases.
/**
 * @brief Execute WifiServiceRelease.
 * @return Return the function result.
 */
esp_err_t WifiServiceRelease(void);

// Returns the currently active service mode.
/**
 * @brief Execute WifiServiceActiveMode.
 * @return Return the function result.
 */
wifi_service_mode_t WifiServiceActiveMode(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_WIFI_SERVICE_H_
