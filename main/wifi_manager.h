#ifndef PT100_LOGGER_WIFI_MANAGER_H_
#define PT100_LOGGER_WIFI_MANAGER_H_

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    bool sta_netif_present;
    bool owns_sta_netif;
    bool wifi_initialized;
    bool wifi_handler_registered;
    bool ip_handler_registered;
    bool wifi_started;
    bool started_by_manager;
    bool wifi_connected;
  } wifi_manager_status_t;

/**
 * @brief Execute WifiManagerInit.
 * @return Return the function result.
 */
esp_err_t WifiManagerInit(void);

/**
 * @brief Execute WifiManagerDeinit.
 * @return Return the function result.
 */
esp_err_t WifiManagerDeinit(void);

// Stops Wi-Fi without deinitializing esp_wifi or destroying the netif. This
// is useful when ownership is controlled elsewhere (e.g., Wi-Fi service).
/**
 * @brief Execute WifiManagerStop.
 * @return Return the function result.
 */
esp_err_t WifiManagerStop(void);

/**
 * @brief Execute WifiManagerScan.
 * @param out_records Parameter out_records.
 * @param max_records Parameter max_records.
 * @param out_count Parameter out_count.
 * @return Return the function result.
 */
esp_err_t WifiManagerScan(wifi_ap_record_t* out_records,
                          size_t max_records,
                          size_t* out_count);

/**
 * @brief Execute WifiManagerConnectSta.
 * @param ssid Parameter ssid.
 * @param password Parameter password.
 * @param timeout_ms Parameter timeout_ms.
 * @return Return the function result.
 */
esp_err_t WifiManagerConnectSta(const char* ssid,
                                const char* password,
                                int timeout_ms);

/**
 * @brief Execute WifiManagerDisconnectSta.
 * @return Return the function result.
 */
esp_err_t WifiManagerDisconnectSta(void);

/**
 * @brief Execute WifiManagerIsStarted.
 * @return Return the function result.
 */
bool WifiManagerIsStarted(void);

/**
 * @brief Execute WifiManagerIsConnected.
 * @return Return the function result.
 */
bool WifiManagerIsConnected(void);

// Notifies the manager that Wi-Fi has been started elsewhere (e.g., Wi-Fi
// service). This keeps internal state consistent when ownership is shared.
/**
 * @brief Execute WifiManagerNotifyWifiStarted.
 */
void WifiManagerNotifyWifiStarted(void);

/**
 * @brief Execute WifiManagerGetIpInfo.
 * @param out_ip Parameter out_ip.
 * @return Return the function result.
 */
esp_err_t WifiManagerGetIpInfo(esp_netif_ip_info_t* out_ip);

/**
 * @brief Execute WifiManagerLastDisconnectReason.
 * @return Return the function result.
 */
wifi_err_reason_t WifiManagerLastDisconnectReason(void);

/**
 * @brief Execute WifiManagerLastConnectAttempts.
 * @return Return the function result.
 */
int WifiManagerLastConnectAttempts(void);

/**
 * @brief Execute WifiManagerGetStatus.
 * @param out_status Parameter out_status.
 */
void WifiManagerGetStatus(wifi_manager_status_t* out_status);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_WIFI_MANAGER_H_
