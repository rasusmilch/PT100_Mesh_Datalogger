#include "wifi_credentials.h"

#include <string.h>

#include "nvs.h"
#include "sdkconfig.h"

void
WifiCredentialsLoad(wifi_credentials_t* out)
{
  if (out == NULL) {
    return;
  }
  memset(out, 0, sizeof(*out));

  nvs_handle_t handle;
  esp_err_t open_result = nvs_open("app", NVS_READONLY, &handle);
  if (open_result == ESP_OK) {
    size_t ssid_len = sizeof(out->ssid);
    esp_err_t ssid_result =
      nvs_get_str(handle, "wifi_ssid", out->ssid, &ssid_len);
    if (ssid_result != ESP_OK || out->ssid[0] == '\0') {
      memset(out->ssid, 0, sizeof(out->ssid));
    }

    size_t pass_len = sizeof(out->password);
    esp_err_t pass_result =
      nvs_get_str(handle, "wifi_pass", out->password, &pass_len);
    if (pass_result != ESP_OK || out->password[0] == '\0') {
      memset(out->password, 0, sizeof(out->password));
    }
    nvs_close(handle);

    if (out->ssid[0] != '\0') {
      out->has_ssid = true;
      out->from_nvs = true;
      return;
    }
  }

  if (CONFIG_APP_WIFI_ROUTER_SSID[0] != '\0') {
    strncpy(out->ssid, CONFIG_APP_WIFI_ROUTER_SSID, sizeof(out->ssid) - 1);
    out->ssid[sizeof(out->ssid) - 1] = '\0';
    out->has_ssid = true;
    out->from_nvs = false;
  }

  if (out->has_ssid && CONFIG_APP_WIFI_ROUTER_PASSWORD[0] != '\0') {
    strncpy(
      out->password, CONFIG_APP_WIFI_ROUTER_PASSWORD, sizeof(out->password) - 1);
    out->password[sizeof(out->password) - 1] = '\0';
  }
}
