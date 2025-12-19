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

esp_err_t WifiManagerInit(void);

esp_err_t WifiManagerDeinit(void);

esp_err_t WifiManagerScan(wifi_ap_record_t* out_records,
                          size_t max_records,
                          size_t* out_count);

esp_err_t WifiManagerConnectSta(const char* ssid,
                                const char* password,
                                int timeout_ms);

esp_err_t WifiManagerDisconnectSta(void);

bool WifiManagerIsStarted(void);

bool WifiManagerIsConnected(void);

esp_err_t WifiManagerGetIpInfo(esp_netif_ip_info_t* out_ip);

wifi_err_reason_t WifiManagerLastDisconnectReason(void);

int WifiManagerLastConnectAttempts(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_WIFI_MANAGER_H_
