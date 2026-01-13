#ifndef PT100_LOGGER_WIFI_CREDENTIALS_H_
#define PT100_LOGGER_WIFI_CREDENTIALS_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

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
 * @brief Execute WifiCredentialsSave.
 * @param ssid Parameter ssid.
 * @param password Parameter password.
 * @return Return the function result.
 */
  esp_err_t WifiCredentialsSave(const char* ssid, const char* password);

/**
 * @brief Execute WifiCredentialsClear.
 * @return Return the function result.
 */
  esp_err_t WifiCredentialsClear(void);

/**
 * @brief Execute WifiCredentialsLoad.
 * @param out Parameter out.
 */
  void WifiCredentialsLoad(wifi_credentials_t* out);

/**
 * @brief Execute WifiCredentialsHasSsid.
 * @return Return the function result.
 */
  bool WifiCredentialsHasSsid(void);

/**
 * @brief Execute WifiCredentialsGetRevision.
 * @return Return the function result.
 */
  uint32_t WifiCredentialsGetRevision(void);

/**
 * @brief Execute WifiCredentialsMaskPassword.
 * @param password Parameter password.
 * @param out Parameter out.
 * @param out_len Parameter out_len.
 */
  void WifiCredentialsMaskPassword(const char* password,
                                   char* out,
                                   size_t out_len);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_WIFI_CREDENTIALS_H_
