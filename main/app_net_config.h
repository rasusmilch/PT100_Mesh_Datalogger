#ifndef PT100_LOGGER_APP_NET_CONFIG_H_
#define PT100_LOGGER_APP_NET_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "mesh_addr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Hostname/IP string cap (not including NUL).
#define APP_NET_SNTP_SERVER_MAX_CHARS (63U)
#define APP_NET_SNTP_SERVER_BUF_LEN (APP_NET_SNTP_SERVER_MAX_CHARS + 1U)

#define APP_NET_SNTP_SERVERS_MAX_COUNT (3U)
#define APP_NET_SNTP_SERVERS_MAX_CHARS (191U)
#define APP_NET_SNTP_SERVERS_BUF_LEN (APP_NET_SNTP_SERVERS_MAX_CHARS + 1U)

esp_err_t AppNetConfigInit(void);

uint8_t AppNetConfigGetMeshChannel(void);
bool AppNetConfigGetMeshId(pt100_mesh_addr_t* out_id);
const char* AppNetConfigGetMeshIdString(void);
const char* AppNetConfigGetMeshApPassword(void);
bool AppNetConfigGetMeshDisableRouter(void);
const char* AppNetConfigGetSntpServer(void);
const char* AppNetConfigGetSntpServersCsv(void);
uint8_t AppNetConfigGetSntpServerCount(void);
const char* AppNetConfigGetSntpServerAt(uint8_t index);
uint32_t AppNetConfigGetSntpFailThresholdN(void);
uint32_t AppNetConfigGetTimeSyncPeriodSeconds(void);

bool AppNetConfigMeshChannelIsOverridden(void);
bool AppNetConfigMeshIdIsOverridden(void);
bool AppNetConfigMeshApPasswordIsOverridden(void);
bool AppNetConfigMeshDisableRouterIsOverridden(void);
bool AppNetConfigSntpServerIsOverridden(void);
bool AppNetConfigSntpFailThresholdIsOverridden(void);
bool AppNetConfigTimeSyncPeriodIsOverridden(void);

esp_err_t AppNetConfigSetMeshChannel(uint8_t channel);
esp_err_t AppNetConfigSetMeshIdString(const char* id_str);
esp_err_t AppNetConfigSetMeshApPassword(const char* password);
esp_err_t AppNetConfigSetMeshDisableRouter(bool disabled);
esp_err_t AppNetConfigSetSntpServer(const char* server);
esp_err_t AppNetConfigSetSntpFailThresholdN(uint32_t n);
esp_err_t AppNetConfigSetTimeSyncPeriodSeconds(uint32_t seconds);
esp_err_t AppNetConfigClearAllOverrides(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_APP_NET_CONFIG_H_
