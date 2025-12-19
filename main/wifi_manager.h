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

esp_err_t WifiManagerInit(void);

esp_err_t WifiManagerDeinit(void);

// Stops Wi-Fi without deinitializing esp_wifi or destroying the netif. This
// is useful when ownership is controlled elsewhere (e.g., Wi-Fi service).
esp_err_t WifiManagerStop(void);

esp_err_t WifiManagerScan(wifi_ap_record_t* out_records,
                          size_t max_records,
                          size_t* out_count);

esp_err_t WifiManagerConnectSta(const char* ssid,
                                const char* password,
                                int timeout_ms);

esp_err_t WifiManagerDisconnectSta(void);

bool WifiManagerIsStarted(void);

bool WifiManagerIsConnected(void);

// Notifies the manager that Wi-Fi has been started elsewhere (e.g., Wi-Fi
// service). This keeps internal state consistent when ownership is shared.
void WifiManagerNotifyWifiStarted(void);

esp_err_t WifiManagerGetIpInfo(esp_netif_ip_info_t* out_ip);

wifi_err_reason_t WifiManagerLastDisconnectReason(void);

int WifiManagerLastConnectAttempts(void);

void WifiManagerGetStatus(wifi_manager_status_t* out_status);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_WIFI_MANAGER_H_
