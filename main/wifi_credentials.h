#ifndef PT100_LOGGER_WIFI_CREDENTIALS_H_
#define PT100_LOGGER_WIFI_CREDENTIALS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    char ssid[33];
    char password[65];
    bool has_ssid;
    bool from_nvs;
  } wifi_credentials_t;

/**
 * @brief Execute WifiCredentialsLoad.
 * @param out Parameter out.
 */
  void WifiCredentialsLoad(wifi_credentials_t* out);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_WIFI_CREDENTIALS_H_
