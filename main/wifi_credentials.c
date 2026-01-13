#include "wifi_credentials.h"

#include <string.h>

#include "nvs.h"
#include "sdkconfig.h"

static const char* kWifiNamespace = "pt100_logger";
static const char* kLegacyWifiNamespace = "app";
static const char* kWifiSsidKey = "wifi_ssid";
static const char* kWifiPassKey = "wifi_pass";
static uint32_t s_wifi_credentials_revision = 0;

static bool
ReadNvsString(nvs_handle_t handle, const char* key, char* out, size_t out_len)
{
  if (out == NULL || out_len == 0) {
    return false;
  }
  size_t value_len = out_len;
  esp_err_t result = nvs_get_str(handle, key, out, &value_len);
  if (result != ESP_OK || out[0] == '\0') {
    out[0] = '\0';
    return false;
  }
  out[out_len - 1] = '\0';
  return true;
}

static bool
LoadCredentialsFromNamespace(const char* name, wifi_credentials_t* out)
{
  nvs_handle_t handle;
  esp_err_t open_result = nvs_open(name, NVS_READONLY, &handle);
  if (open_result != ESP_OK) {
    return false;
  }

  const bool has_ssid =
    ReadNvsString(handle, kWifiSsidKey, out->ssid, sizeof(out->ssid));
  const bool has_password =
    ReadNvsString(handle, kWifiPassKey, out->password, sizeof(out->password));
  if (!has_password) {
    out->password[0] = '\0';
  }
  nvs_close(handle);

  if (!has_ssid) {
    out->ssid[0] = '\0';
    out->password[0] = '\0';
    return false;
  }
  return true;
}

static void
EraseLegacyCredentials(void)
{
  nvs_handle_t handle;
  esp_err_t open_result = nvs_open(kLegacyWifiNamespace, NVS_READWRITE, &handle);
  if (open_result != ESP_OK) {
    return;
  }

  esp_err_t erase_result = nvs_erase_key(handle, kWifiSsidKey);
  if (erase_result != ESP_OK && erase_result != ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(handle);
    return;
  }
  erase_result = nvs_erase_key(handle, kWifiPassKey);
  if (erase_result != ESP_OK && erase_result != ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(handle);
    return;
  }
  (void)nvs_commit(handle);
  nvs_close(handle);
}

/**
 * @brief Execute WifiCredentialsSave.
 * @param ssid Parameter ssid.
 * @param password Parameter password.
 * @return Return the function result.
 */
esp_err_t
WifiCredentialsSave(const char* ssid, const char* password)
{
  if (ssid == NULL || password == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t ssid_len = strlen(ssid);
  if (ssid_len == 0 || ssid_len > 32) {
    return ESP_ERR_INVALID_ARG;
  }

  const size_t password_len = strlen(password);
  if (password_len > 64) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t open_result = nvs_open(kWifiNamespace, NVS_READWRITE, &handle);
  if (open_result != ESP_OK) {
    return open_result;
  }

  esp_err_t result = nvs_set_str(handle, kWifiSsidKey, ssid);
  if (result == ESP_OK) {
    if (password_len > 0) {
      result = nvs_set_str(handle, kWifiPassKey, password);
    } else {
      result = nvs_erase_key(handle, kWifiPassKey);
      if (result == ESP_ERR_NVS_NOT_FOUND) {
        result = ESP_OK;
      }
    }
  }
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }

  nvs_close(handle);
  if (result == ESP_OK) {
    s_wifi_credentials_revision++;
  }
  return result;
}

/**
 * @brief Execute WifiCredentialsClear.
 * @return Return the function result.
 */
esp_err_t
WifiCredentialsClear(void)
{
  nvs_handle_t handle;
  esp_err_t open_result = nvs_open(kWifiNamespace, NVS_READWRITE, &handle);
  if (open_result != ESP_OK) {
    return open_result;
  }

  esp_err_t result = nvs_erase_key(handle, kWifiSsidKey);
  if (result != ESP_OK && result != ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(handle);
    return result;
  }
  result = nvs_erase_key(handle, kWifiPassKey);
  if (result != ESP_OK && result != ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(handle);
    return result;
  }
  result = nvs_commit(handle);
  nvs_close(handle);
  if (result == ESP_OK) {
    s_wifi_credentials_revision++;
  }
  return result;
}

/**
 * @brief Execute WifiCredentialsLoad.
 * @param out Parameter out.
 */
void
WifiCredentialsLoad(wifi_credentials_t* out)
{
  if (out == NULL) {
    return;
  }
  memset(out, 0, sizeof(*out));

  if (LoadCredentialsFromNamespace(kWifiNamespace, out)) {
    out->has_ssid = true;
    out->from_nvs = true;
    return;
  }

  wifi_credentials_t legacy;
  memset(&legacy, 0, sizeof(legacy));
  if (LoadCredentialsFromNamespace(kLegacyWifiNamespace, &legacy)) {
    *out = legacy;
    out->has_ssid = true;
    out->from_nvs = true;
    if (WifiCredentialsSave(out->ssid, out->password) == ESP_OK) {
      EraseLegacyCredentials();
    }
    return;
  }

#if defined(CONFIG_APP_WIFI_ALLOW_KCONFIG_FALLBACK)
  if (CONFIG_APP_WIFI_ALLOW_KCONFIG_FALLBACK != 0 &&
      CONFIG_APP_WIFI_ROUTER_SSID[0] != '\0') {
    strncpy(out->ssid, CONFIG_APP_WIFI_ROUTER_SSID, sizeof(out->ssid) - 1);
    out->ssid[sizeof(out->ssid) - 1] = '\0';
    out->has_ssid = true;
    out->from_nvs = false;
  }

  if (out->has_ssid && CONFIG_APP_WIFI_ROUTER_PASSWORD[0] != '\0') {
    strncpy(out->password,
            CONFIG_APP_WIFI_ROUTER_PASSWORD,
            sizeof(out->password) - 1);
    out->password[sizeof(out->password) - 1] = '\0';
  }
#endif
}

/**
 * @brief Execute WifiCredentialsHasSsid.
 * @return Return the function result.
 */
bool
WifiCredentialsHasSsid(void)
{
  wifi_credentials_t creds;
  memset(&creds, 0, sizeof(creds));

  if (LoadCredentialsFromNamespace(kWifiNamespace, &creds)) {
    return true;
  }
  if (LoadCredentialsFromNamespace(kLegacyWifiNamespace, &creds)) {
    return true;
  }
  return false;
}

/**
 * @brief Execute WifiCredentialsGetRevision.
 * @return Return the function result.
 */
uint32_t
WifiCredentialsGetRevision(void)
{
  return s_wifi_credentials_revision;
}

/**
 * @brief Execute WifiCredentialsMaskPassword.
 * @param password Parameter password.
 * @param out Parameter out.
 * @param out_len Parameter out_len.
 */
void
WifiCredentialsMaskPassword(const char* password, char* out, size_t out_len)
{
  if (out == NULL || out_len == 0) {
    return;
  }
  if (password == NULL || password[0] == '\0') {
    out[0] = '\0';
    return;
  }

  size_t password_len = strlen(password);
  if (password_len >= out_len) {
    password_len = out_len - 1;
  }
  memset(out, '*', password_len);
  out[password_len] = '\0';
}
