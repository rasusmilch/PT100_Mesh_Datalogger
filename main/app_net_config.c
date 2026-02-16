#include "app_net_config.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char* kTag = "net_cfg";

static const char* kNetNamespace = "pt100_logger";
static const char* kKeyMeshChannel = "mesh_chan";
static const char* kKeyMeshId = "mesh_id";
static const char* kKeyMeshApPassword = "mesh_ap_pass";
static const char* kKeyMeshNoRouter = "mesh_no_router";
static const char* kKeySntpServer = "sntp_srv";
static const char* kKeySntpFailThreshold = "sntp_fail_n";
static const char* kKeyTimeSync = "time_sync_s";

static const uint8_t kMeshChannelMin = 1;
static const uint8_t kMeshChannelMax = 13;
static const uint32_t kTimeSyncMinSeconds = 5;
static const uint32_t kTimeSyncMaxSeconds = 86400;

// Compile-time constants (C-friendly). Avoid file-scope VLAs and "+1" scatter.
// "AA:BB:CC:DD:EE:FF" is 17 chars, plus NUL => 18 bytes.
#define APP_NET_MESH_ID_STRING_MAX_CHARS (17U)
#define APP_NET_MESH_ID_STRING_BUF_LEN (APP_NET_MESH_ID_STRING_MAX_CHARS + 1U)

// WPA2 PSK max is 63 characters (not including NUL).
#define APP_NET_MESH_AP_PASSWORD_MAX_CHARS (63U)
#define APP_NET_MESH_AP_PASSWORD_BUF_LEN                                       \
  (APP_NET_MESH_AP_PASSWORD_MAX_CHARS + 1U)

#define APP_NET_SNTP_FAIL_THRESHOLD_MIN (1U)
#define APP_NET_SNTP_FAIL_THRESHOLD_MAX (20U)

typedef struct
{
  uint8_t mesh_channel;
  bool mesh_channel_from_nvs;
  pt100_mesh_addr_t mesh_id;
  bool mesh_id_valid;
  bool mesh_id_from_nvs;
  char mesh_id_string[APP_NET_MESH_ID_STRING_BUF_LEN];
  char mesh_ap_password[APP_NET_MESH_AP_PASSWORD_BUF_LEN];
  bool mesh_ap_password_from_nvs;
  bool mesh_disable_router;
  bool mesh_disable_router_from_nvs;
  char sntp_servers_csv[APP_NET_SNTP_SERVERS_BUF_LEN];
  char sntp_servers[APP_NET_SNTP_SERVERS_MAX_COUNT][APP_NET_SNTP_SERVER_BUF_LEN];
  uint8_t sntp_server_count;
  bool sntp_server_from_nvs;
  uint32_t sntp_fail_threshold_n;
  bool sntp_fail_threshold_from_nvs;
  uint32_t time_sync_period_s;
  bool time_sync_period_from_nvs;
} app_net_config_t;

static app_net_config_t g_config;
static bool g_initialized = false;

static void TrimAsciiWhitespace(const char* in,
                                size_t in_len,
                                const char** out_start,
                                size_t* out_len);
static bool ParseSntpServersCsv(
  const char* csv,
  char out_servers[APP_NET_SNTP_SERVERS_MAX_COUNT][APP_NET_SNTP_SERVER_BUF_LEN],
  uint8_t* out_count);
static bool IsValidSntpServersCsv(const char* csv);
static void RebuildParsedSntpServers(app_net_config_t* config);
static uint32_t ClampSntpFailThresholdN(uint32_t n);

static bool
ParseMeshIdString(const char* mesh_id_string, pt100_mesh_addr_t* mesh_id_out)
{
  if (mesh_id_string == NULL || mesh_id_out == NULL) {
    return false;
  }

  int values[6] = { 0 };
  if (sscanf(mesh_id_string,
             "%x:%x:%x:%x:%x:%x",
             &values[0],
             &values[1],
             &values[2],
             &values[3],
             &values[4],
             &values[5]) != 6) {
    return false;
  }

  for (int index = 0; index < 6; ++index) {
    if (values[index] < 0 || values[index] > 0xFF) {
      return false;
    }
    mesh_id_out->addr[index] = (uint8_t)values[index];
  }
  return true;
}

static void
FormatMeshIdString(char* out, size_t out_size, const pt100_mesh_addr_t* mesh_id)
{
  if (out == NULL || out_size == 0 || mesh_id == NULL) {
    return;
  }
  snprintf(out, out_size, MACSTR, MAC2STR(mesh_id->addr));
}

static uint32_t
ClampTimeSyncSeconds(uint32_t seconds)
{
  if (seconds < kTimeSyncMinSeconds) {
    return kTimeSyncMinSeconds;
  }
  if (seconds > kTimeSyncMaxSeconds) {
    return kTimeSyncMaxSeconds;
  }
  return seconds;
}

static bool
IsValidMeshChannel(uint8_t channel)
{
  return (channel == 0) ||
         ((channel >= kMeshChannelMin) && (channel <= kMeshChannelMax));
}

static bool
IsValidMeshApPassword(const char* password)
{
  if (password == NULL) {
    return false;
  }
  const size_t length = strlen(password);
  return (length == 0) ||
         (length >= 8 && length <= APP_NET_MESH_AP_PASSWORD_MAX_CHARS);
}

static bool
IsValidSntpServer(const char* server)
{
  if (server == NULL) {
    return false;
  }
  const size_t length = strlen(server);
  return (length > 0) && (length <= APP_NET_SNTP_SERVER_MAX_CHARS);
}

static void
TrimAsciiWhitespace(const char* in,
                    size_t in_len,
                    const char** out_start,
                    size_t* out_len)
{
  if (out_start != NULL) {
    *out_start = in;
  }
  if (out_len != NULL) {
    *out_len = in_len;
  }
  if (in == NULL) {
    return;
  }

  size_t start = 0;
  while (start < in_len) {
    const char c = in[start];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
      break;
    }
    ++start;
  }

  size_t end = in_len;
  while (end > start) {
    const char c = in[end - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
      break;
    }
    --end;
  }

  if (out_start != NULL) {
    *out_start = in + start;
  }
  if (out_len != NULL) {
    *out_len = end - start;
  }
}

static bool
ParseSntpServersCsv(
  const char* csv,
  char out_servers[APP_NET_SNTP_SERVERS_MAX_COUNT][APP_NET_SNTP_SERVER_BUF_LEN],
  uint8_t* out_count)
{
  if (out_count != NULL) {
    *out_count = 0;
  }
  if (out_servers != NULL) {
    for (size_t idx = 0; idx < APP_NET_SNTP_SERVERS_MAX_COUNT; ++idx) {
      out_servers[idx][0] = '\0';
    }
  }
  if (csv == NULL || out_servers == NULL || out_count == NULL) {
    return false;
  }

  const size_t total_len = strlen(csv);
  if (total_len == 0 || total_len > APP_NET_SNTP_SERVERS_MAX_CHARS) {
    return false;
  }

  size_t start = 0;
  uint8_t count = 0;
  while (start <= total_len) {
    size_t end = start;
    while (end < total_len && csv[end] != ',') {
      ++end;
    }

    const char* token_start = NULL;
    size_t token_len = 0;
    TrimAsciiWhitespace(csv + start, end - start, &token_start, &token_len);
    if (token_len == 0 || token_len > APP_NET_SNTP_SERVER_MAX_CHARS) {
      return false;
    }
    if (count >= APP_NET_SNTP_SERVERS_MAX_COUNT) {
      return false;
    }

    memcpy(out_servers[count], token_start, token_len);
    out_servers[count][token_len] = '\0';
    if (!IsValidSntpServer(out_servers[count])) {
      return false;
    }
    ++count;

    if (end >= total_len) {
      break;
    }
    start = end + 1;
    if (start > total_len) {
      return false;
    }
  }

  if (count == 0 || count > APP_NET_SNTP_SERVERS_MAX_COUNT) {
    return false;
  }
  *out_count = count;
  return true;
}

static bool
IsValidSntpServersCsv(const char* csv)
{
  char servers[APP_NET_SNTP_SERVERS_MAX_COUNT][APP_NET_SNTP_SERVER_BUF_LEN] =
    { 0 };
  uint8_t count = 0;
  return ParseSntpServersCsv(csv, servers, &count);
}

static void
RebuildParsedSntpServers(app_net_config_t* config)
{
  if (config == NULL) {
    return;
  }
  uint8_t count = 0;
  if (!ParseSntpServersCsv(
        config->sntp_servers_csv, config->sntp_servers, &count)) {
    for (size_t idx = 0; idx < APP_NET_SNTP_SERVERS_MAX_COUNT; ++idx) {
      config->sntp_servers[idx][0] = '\0';
    }
    config->sntp_server_count = 0;
    return;
  }
  config->sntp_server_count = count;
}

static uint32_t
ClampSntpFailThresholdN(uint32_t n)
{
  if (n < APP_NET_SNTP_FAIL_THRESHOLD_MIN) {
    return APP_NET_SNTP_FAIL_THRESHOLD_MIN;
  }
  if (n > APP_NET_SNTP_FAIL_THRESHOLD_MAX) {
    return APP_NET_SNTP_FAIL_THRESHOLD_MAX;
  }
  return n;
}

static void
LoadDefaults(app_net_config_t* config)
{
  if (config == NULL) {
    return;
  }
  memset(config, 0, sizeof(*config));

  uint8_t channel = (uint8_t)CONFIG_APP_MESH_CHANNEL;
  if (!IsValidMeshChannel(channel)) {
    ESP_LOGW(kTag, "Default mesh channel %u invalid; using 0", channel);
    channel = 0;
  }
  config->mesh_channel = channel;

  if (ParseMeshIdString(CONFIG_APP_MESH_ID_HEX, &config->mesh_id)) {
    config->mesh_id_valid = true;
    FormatMeshIdString(
      config->mesh_id_string, sizeof(config->mesh_id_string), &config->mesh_id);
  } else {
    config->mesh_id_valid = false;
    config->mesh_id_string[0] = '\0';
    ESP_LOGW(kTag, "Default mesh ID invalid: %s", CONFIG_APP_MESH_ID_HEX);
  }

  strlcpy(config->mesh_ap_password,
          CONFIG_APP_MESH_AP_PASSWORD,
          sizeof(config->mesh_ap_password));
  if (!IsValidMeshApPassword(config->mesh_ap_password)) {
    ESP_LOGW(kTag, "Default mesh AP password invalid; using empty");
    config->mesh_ap_password[0] = '\0';
  }

#if defined(CONFIG_APP_MESH_DISABLE_ROUTER)
  config->mesh_disable_router = (CONFIG_APP_MESH_DISABLE_ROUTER != 0);
#else
  config->mesh_disable_router = false;
#endif

  strlcpy(config->sntp_servers_csv,
          CONFIG_APP_SNTP_SERVER,
          sizeof(config->sntp_servers_csv));
  if (!IsValidSntpServersCsv(config->sntp_servers_csv)) {
    ESP_LOGW(kTag, "Default SNTP server list invalid; using empty");
    config->sntp_servers_csv[0] = '\0';
  }
  RebuildParsedSntpServers(config);

  config->sntp_fail_threshold_n = ClampSntpFailThresholdN(3U);
  config->time_sync_period_s =
    ClampTimeSyncSeconds((uint32_t)CONFIG_APP_TIME_SYNC_PERIOD_S);
}

static bool
ReadNvsString(nvs_handle_t handle,
              const char* key,
              char* out,
              size_t out_len,
              bool allow_empty)
{
  if (out == NULL || out_len == 0) {
    return false;
  }
  size_t value_len = out_len;
  esp_err_t result = nvs_get_str(handle, key, out, &value_len);
  if (result != ESP_OK) {
    out[0] = '\0';
    return false;
  }
  out[out_len - 1] = '\0';
  if (!allow_empty && out[0] == '\0') {
    return false;
  }
  return true;
}

static void
ApplyNvsOverrides(app_net_config_t* config)
{
  if (config == NULL) {
    return;
  }

  nvs_handle_t handle;
  esp_err_t open_result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (open_result != ESP_OK) {
    return;
  }

  bool needs_commit = false;
  uint8_t mesh_channel = 0;
  esp_err_t result = nvs_get_u8(handle, kKeyMeshChannel, &mesh_channel);
  if (result == ESP_OK) {
    if (IsValidMeshChannel(mesh_channel)) {
      config->mesh_channel = mesh_channel;
      config->mesh_channel_from_nvs = true;
    } else {
      ESP_LOGW(
        kTag, "Invalid mesh channel override %u; clearing", mesh_channel);
      (void)nvs_erase_key(handle, kKeyMeshChannel);
      needs_commit = true;
    }
  }

  char mesh_id_string[APP_NET_MESH_ID_STRING_BUF_LEN] = { 0 };

  if (ReadNvsString(
        handle, kKeyMeshId, mesh_id_string, sizeof(mesh_id_string), false)) {
    pt100_mesh_addr_t mesh_id;
    if (ParseMeshIdString(mesh_id_string, &mesh_id)) {
      config->mesh_id = mesh_id;
      config->mesh_id_valid = true;
      FormatMeshIdString(
        config->mesh_id_string, sizeof(config->mesh_id_string), &mesh_id);
      config->mesh_id_from_nvs = true;
    } else {
      ESP_LOGW(kTag, "Invalid mesh ID override: %s; clearing", mesh_id_string);
      (void)nvs_erase_key(handle, kKeyMeshId);
      needs_commit = true;
    }
  }

  char mesh_ap_password[APP_NET_MESH_AP_PASSWORD_BUF_LEN] = { 0 };
  if (ReadNvsString(handle,
                    kKeyMeshApPassword,
                    mesh_ap_password,
                    sizeof(mesh_ap_password),
                    true)) {
    if (IsValidMeshApPassword(mesh_ap_password)) {
      strlcpy(config->mesh_ap_password,
              mesh_ap_password,
              sizeof(config->mesh_ap_password));
      config->mesh_ap_password_from_nvs = true;
    } else {
      ESP_LOGW(kTag, "Invalid mesh AP password override; clearing");
      (void)nvs_erase_key(handle, kKeyMeshApPassword);
      needs_commit = true;
    }
  }

  uint8_t no_router = 0;
  result = nvs_get_u8(handle, kKeyMeshNoRouter, &no_router);
  if (result == ESP_OK) {
    config->mesh_disable_router = (no_router != 0);
    config->mesh_disable_router_from_nvs = true;
  }

  char sntp_server[APP_NET_SNTP_SERVERS_BUF_LEN] = { 0 };
  if (ReadNvsString(
        handle, kKeySntpServer, sntp_server, sizeof(sntp_server), false)) {
    if (IsValidSntpServersCsv(sntp_server)) {
      strlcpy(config->sntp_servers_csv,
              sntp_server,
              sizeof(config->sntp_servers_csv));
      config->sntp_server_from_nvs = true;
    } else {
      ESP_LOGW(kTag, "Invalid SNTP server list override; clearing");
      (void)nvs_erase_key(handle, kKeySntpServer);
      needs_commit = true;
    }
  }

  uint32_t sntp_fail_threshold_n = 0;
  result = nvs_get_u32(handle, kKeySntpFailThreshold, &sntp_fail_threshold_n);
  if (result == ESP_OK) {
    const uint32_t clamped = ClampSntpFailThresholdN(sntp_fail_threshold_n);
    config->sntp_fail_threshold_n = clamped;
    config->sntp_fail_threshold_from_nvs = true;
    if (clamped != sntp_fail_threshold_n) {
      (void)nvs_set_u32(handle, kKeySntpFailThreshold, clamped);
      needs_commit = true;
    }
  }

  uint32_t time_sync_s = 0;
  result = nvs_get_u32(handle, kKeyTimeSync, &time_sync_s);
  if (result == ESP_OK) {
    const uint32_t clamped = ClampTimeSyncSeconds(time_sync_s);
    config->time_sync_period_s = clamped;
    config->time_sync_period_from_nvs = true;
    if (clamped != time_sync_s) {
      (void)nvs_set_u32(handle, kKeyTimeSync, clamped);
      needs_commit = true;
    }
  }

  if (needs_commit) {
    (void)nvs_commit(handle);
  }

  nvs_close(handle);
  RebuildParsedSntpServers(config);
}

static void
EnsureInitialized(void)
{
  if (!g_initialized) {
    (void)AppNetConfigInit();
  }
}

esp_err_t
AppNetConfigInit(void)
{
  LoadDefaults(&g_config);
  ApplyNvsOverrides(&g_config);
  g_initialized = true;
  return ESP_OK;
}

uint8_t
AppNetConfigGetMeshChannel(void)
{
  EnsureInitialized();
  return g_config.mesh_channel;
}

bool
AppNetConfigGetMeshId(pt100_mesh_addr_t* out_id)
{
  EnsureInitialized();
  if (out_id == NULL || !g_config.mesh_id_valid) {
    return false;
  }
  *out_id = g_config.mesh_id;
  return true;
}

const char*
AppNetConfigGetMeshIdString(void)
{
  EnsureInitialized();
  return g_config.mesh_id_string;
}

const char*
AppNetConfigGetMeshApPassword(void)
{
  EnsureInitialized();
  return g_config.mesh_ap_password;
}

bool
AppNetConfigGetMeshDisableRouter(void)
{
  EnsureInitialized();
  return g_config.mesh_disable_router;
}

const char*
AppNetConfigGetSntpServer(void)
{
  EnsureInitialized();
  if (g_config.sntp_server_count == 0) {
    return "";
  }
  return g_config.sntp_servers[0];
}

const char*
AppNetConfigGetSntpServersCsv(void)
{
  EnsureInitialized();
  return g_config.sntp_servers_csv;
}

uint8_t
AppNetConfigGetSntpServerCount(void)
{
  EnsureInitialized();
  return g_config.sntp_server_count;
}

const char*
AppNetConfigGetSntpServerAt(uint8_t index)
{
  EnsureInitialized();
  if (index >= g_config.sntp_server_count) {
    return "";
  }
  return g_config.sntp_servers[index];
}

uint32_t
AppNetConfigGetSntpFailThresholdN(void)
{
  EnsureInitialized();
  return g_config.sntp_fail_threshold_n;
}

uint32_t
AppNetConfigGetTimeSyncPeriodSeconds(void)
{
  EnsureInitialized();
  return g_config.time_sync_period_s;
}

bool
AppNetConfigMeshChannelIsOverridden(void)
{
  EnsureInitialized();
  return g_config.mesh_channel_from_nvs;
}

bool
AppNetConfigMeshIdIsOverridden(void)
{
  EnsureInitialized();
  return g_config.mesh_id_from_nvs;
}

bool
AppNetConfigMeshApPasswordIsOverridden(void)
{
  EnsureInitialized();
  return g_config.mesh_ap_password_from_nvs;
}

bool
AppNetConfigMeshDisableRouterIsOverridden(void)
{
  EnsureInitialized();
  return g_config.mesh_disable_router_from_nvs;
}

bool
AppNetConfigSntpServerIsOverridden(void)
{
  EnsureInitialized();
  return g_config.sntp_server_from_nvs;
}

bool
AppNetConfigSntpFailThresholdIsOverridden(void)
{
  EnsureInitialized();
  return g_config.sntp_fail_threshold_from_nvs;
}

bool
AppNetConfigTimeSyncPeriodIsOverridden(void)
{
  EnsureInitialized();
  return g_config.time_sync_period_from_nvs;
}

esp_err_t
AppNetConfigSetMeshChannel(uint8_t channel)
{
  if (!IsValidMeshChannel(channel)) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kKeyMeshChannel, channel);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result == ESP_OK) {
    EnsureInitialized();
    g_config.mesh_channel = channel;
    g_config.mesh_channel_from_nvs = true;
  }
  return result;
}

esp_err_t
AppNetConfigSetMeshIdString(const char* id_str)
{
  if (id_str == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  pt100_mesh_addr_t mesh_id;
  if (!ParseMeshIdString(id_str, &mesh_id)) {
    return ESP_ERR_INVALID_ARG;
  }

  char formatted[APP_NET_MESH_ID_STRING_BUF_LEN] = { 0 };
  FormatMeshIdString(formatted, sizeof(formatted), &mesh_id);

  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_str(handle, kKeyMeshId, formatted);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result == ESP_OK) {
    EnsureInitialized();
    g_config.mesh_id = mesh_id;
    g_config.mesh_id_valid = true;
    strlcpy(
      g_config.mesh_id_string, formatted, sizeof(g_config.mesh_id_string));
    g_config.mesh_id_from_nvs = true;
  }
  return result;
}

esp_err_t
AppNetConfigSetMeshApPassword(const char* password)
{
  if (password == NULL || !IsValidMeshApPassword(password)) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_str(handle, kKeyMeshApPassword, password);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result == ESP_OK) {
    EnsureInitialized();
    strlcpy(
      g_config.mesh_ap_password, password, sizeof(g_config.mesh_ap_password));
    g_config.mesh_ap_password_from_nvs = true;
  }
  return result;
}

esp_err_t
AppNetConfigSetMeshDisableRouter(bool disabled)
{
  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  const uint8_t value = disabled ? 1u : 0u;
  result = nvs_set_u8(handle, kKeyMeshNoRouter, value);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result == ESP_OK) {
    EnsureInitialized();
    g_config.mesh_disable_router = disabled;
    g_config.mesh_disable_router_from_nvs = true;
  }
  return result;
}

esp_err_t
AppNetConfigSetSntpServer(const char* server)
{
  if (server == NULL || !IsValidSntpServersCsv(server)) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_str(handle, kKeySntpServer, server);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result == ESP_OK) {
    EnsureInitialized();
    strlcpy(g_config.sntp_servers_csv,
            server,
            sizeof(g_config.sntp_servers_csv));
    RebuildParsedSntpServers(&g_config);
    g_config.sntp_server_from_nvs = true;
  }
  return result;
}

esp_err_t
AppNetConfigSetSntpFailThresholdN(uint32_t n)
{
  const uint32_t clamped = ClampSntpFailThresholdN(n);

  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u32(handle, kKeySntpFailThreshold, clamped);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result == ESP_OK) {
    EnsureInitialized();
    g_config.sntp_fail_threshold_n = clamped;
    g_config.sntp_fail_threshold_from_nvs = true;
  }
  return result;
}

esp_err_t
AppNetConfigSetTimeSyncPeriodSeconds(uint32_t seconds)
{
  const uint32_t clamped = ClampTimeSyncSeconds(seconds);

  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u32(handle, kKeyTimeSync, clamped);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result == ESP_OK) {
    EnsureInitialized();
    g_config.time_sync_period_s = clamped;
    g_config.time_sync_period_from_nvs = true;
  }
  return result;
}

esp_err_t
AppNetConfigClearAllOverrides(void)
{
  nvs_handle_t handle;
  esp_err_t result = nvs_open(kNetNamespace, NVS_READWRITE, &handle);
  if (result != ESP_OK) {
    return result;
  }

  const char* keys[] = {
    kKeyMeshChannel,  kKeyMeshId,      kKeyMeshApPassword,
    kKeyMeshNoRouter, kKeySntpServer,  kKeySntpFailThreshold,
    kKeyTimeSync,
  };

  for (size_t idx = 0; idx < sizeof(keys) / sizeof(keys[0]); ++idx) {
    esp_err_t erase_result = nvs_erase_key(handle, keys[idx]);
    if (erase_result != ESP_OK && erase_result != ESP_ERR_NVS_NOT_FOUND) {
      nvs_close(handle);
      return erase_result;
    }
  }

  result = nvs_commit(handle);
  nvs_close(handle);
  if (result == ESP_OK) {
    LoadDefaults(&g_config);
    g_config.mesh_channel_from_nvs = false;
    g_config.mesh_id_from_nvs = false;
    g_config.mesh_ap_password_from_nvs = false;
    g_config.mesh_disable_router_from_nvs = false;
    g_config.sntp_server_from_nvs = false;
    g_config.sntp_fail_threshold_from_nvs = false;
    g_config.time_sync_period_from_nvs = false;
    g_initialized = true;
  }
  return result;
}
