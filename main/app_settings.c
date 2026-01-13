#include "app_settings.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "esp_log.h"
#include "max31865_reader.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "pt100_table.h"

static const char* kTag = "settings";

static const char* kNvsNamespace = "pt100_logger";
static const char* kKeyLogPeriodMs = "log_period_ms";
static const char* kKeyFlushWatermark = "flush_wm_rec";
static const char* kKeySdFlushPeriodMs = "sd_flush_ms";
static const char* kKeySdBatchBytes = "sd_batch_bytes";
static const char* kKeyCalDegree = "cal_deg";
static const char* kKeyCalMode = "cal_mode";
static const char* kKeyCalCoeffs = "cal_coeffs";
// NVS key names are limited to 15 characters (not including the NUL).
// Keep this <= 15 to avoid ESP_ERR_NVS_KEY_TOO_LONG.
static const char* kKeyCalPointsCount = "cal_pt_count";
static const char* kKeyCalPoints = "cal_points";
static const char* kKeyCalContextVersion = "cal_ctx_ver";
static const char* kKeyCalContextConversion = "cal_ctx_conv";
static const char* kKeyCalContextWires = "cal_ctx_wires";
static const char* kKeyCalContextFilter = "cal_ctx_filter";
static const char* kKeyCalContextRref = "cal_ctx_rref";
static const char* kKeyCalContextR0 = "cal_ctx_r0";
static const char* kKeyCalContextTableVer = "cal_ctx_table";
static const char* kKeyTzPosix = "tz_posix";
static const char* kKeyDstEnabled = "dst_enabled";
static const char* kKeyNodeRole = "node_role";
static const char* kKeyAllowChildren = "allow_child";
static const char* kKeyAllowChildrenSet = "allow_child_set";
static const char* kKeyDisplayUnits = "disp_units";
static const char* kKeyDisplayAttentionMask = "disp_attn";
static const char* kKeyDisplayAttentionPolicy = "disp_attn_pol";
static const char* kKeyNetMode = "net_mode";
static const char* kKeyMqttEnabled = "mqtt_en";
static const char* kKeyMqttBrokerUri = "mqtt_uri";
static const char* kKeyMqttTopicPrefix = "mqtt_pfx";
static const char* kKeyMqttQos = "mqtt_qos";
static const char* kKeyMqttRetain = "mqtt_ret";
static const char* kKeyMqttBridgeMode = "mqtt_bmode";
static const uint8_t kCalibrationContextVersion = 1;
static uint32_t g_display_attention_policy = 0;
static display_attention_mask_t g_display_attention_mask = 0;
static uint32_t g_net_mode_revision = 0;

/**
 * @brief Execute DefaultNodeRole.
 * @return Return the function result.
 */
static app_node_role_t
DefaultNodeRole(void)
{
#if CONFIG_APP_NODE_IS_ROOT
  return APP_NODE_ROLE_ROOT;
#else
  return APP_NODE_ROLE_SENSOR;
#endif
}

/**
 * @brief Execute AppSettingsRoleDefaultAllowsChildren.
 * @param role Parameter role.
 * @return Return the function result.
 */
bool
AppSettingsRoleDefaultAllowsChildren(app_node_role_t role)
{
  return role != APP_NODE_ROLE_SENSOR;
}

/**
 * @brief Execute AppSettingsRoleToString.
 * @param role Parameter role.
 * @return Return the function result.
 */
const char*
AppSettingsRoleToString(app_node_role_t role)
{
  switch (role) {
    case APP_NODE_ROLE_ROOT:
      return "ROOT";
    case APP_NODE_ROLE_SENSOR:
      return "SENSOR";
    case APP_NODE_ROLE_RELAY:
      return "RELAY";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Execute AppSettingsParseRole.
 * @param value Parameter value.
 * @param role_out Parameter role_out.
 * @return Return the function result.
 */
bool
AppSettingsParseRole(const char* value, app_node_role_t* role_out)
{
  if (value == NULL || role_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "root") == 0) {
    *role_out = APP_NODE_ROLE_ROOT;
    return true;
  }
  if (strcasecmp(value, "sensor") == 0) {
    *role_out = APP_NODE_ROLE_SENSOR;
    return true;
  }
  if (strcasecmp(value, "relay") == 0) {
    *role_out = APP_NODE_ROLE_RELAY;
    return true;
  }
  return false;
}

/**
 * @brief Execute AppSettingsDisplayUnitsToString.
 * @param units Parameter units.
 * @return Return the function result.
 */
const char*
AppSettingsDisplayUnitsToString(app_display_units_t units)
{
  switch (units) {
    case APP_DISPLAY_UNITS_C:
      return "C";
    case APP_DISPLAY_UNITS_F:
      return "F";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Execute AppSettingsParseDisplayUnits.
 * @param value Parameter value.
 * @param units_out Parameter units_out.
 * @return Return the function result.
 */
bool
AppSettingsParseDisplayUnits(const char* value,
                             app_display_units_t* units_out)
{
  if (value == NULL || units_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "c") == 0) {
    *units_out = APP_DISPLAY_UNITS_C;
    return true;
  }
  if (strcasecmp(value, "f") == 0) {
    *units_out = APP_DISPLAY_UNITS_F;
    return true;
  }
  return false;
}

/**
 * @brief Execute AppSettingsNetModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
const char*
AppSettingsNetModeToString(app_net_mode_t mode)
{
  switch (mode) {
    case APP_NET_MODE_MESH:
      return "MESH";
    case APP_NET_MODE_DIRECT_WIFI:
      return "WIFI";
    case APP_NET_MODE_NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Execute AppSettingsParseNetMode.
 * @param value Parameter value.
 * @param mode_out Parameter mode_out.
 * @return Return the function result.
 */
bool
AppSettingsParseNetMode(const char* value, app_net_mode_t* mode_out)
{
  if (value == NULL || mode_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "mesh") == 0) {
    *mode_out = APP_NET_MODE_MESH;
    return true;
  }
  if (strcasecmp(value, "wifi") == 0 || strcasecmp(value, "direct") == 0 ||
      strcasecmp(value, "direct_wifi") == 0) {
    *mode_out = APP_NET_MODE_DIRECT_WIFI;
    return true;
  }
  if (strcasecmp(value, "none") == 0 || strcasecmp(value, "off") == 0) {
    *mode_out = APP_NET_MODE_NONE;
    return true;
  }
  return false;
}

/**
 * @brief Execute AppSettingsMqttBridgeModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
const char*
AppSettingsMqttBridgeModeToString(mqtt_bridge_mode_t mode)
{
  switch (mode) {
    case MQTT_BRIDGE_OFF:
      return "OFF";
    case MQTT_BRIDGE_SERIAL:
      return "SERIAL";
    case MQTT_BRIDGE_BROKER:
      return "BROKER";
    case MQTT_BRIDGE_BOTH:
      return "BOTH";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Execute AppSettingsParseMqttBridgeMode.
 * @param value Parameter value.
 * @param mode_out Parameter mode_out.
 * @return Return the function result.
 */
bool
AppSettingsParseMqttBridgeMode(const char* value, mqtt_bridge_mode_t* mode_out)
{
  if (value == NULL || mode_out == NULL) {
    return false;
  }
  if (strcasecmp(value, "off") == 0) {
    *mode_out = MQTT_BRIDGE_OFF;
    return true;
  }
  if (strcasecmp(value, "serial") == 0) {
    *mode_out = MQTT_BRIDGE_SERIAL;
    return true;
  }
  if (strcasecmp(value, "broker") == 0) {
    *mode_out = MQTT_BRIDGE_BROKER;
    return true;
  }
  if (strcasecmp(value, "both") == 0) {
    *mode_out = MQTT_BRIDGE_BOTH;
    return true;
  }
  return false;
}

/**
 * @brief Execute ApplyDefaults.
 * @param settings Parameter settings.
 */
static void
ApplyDefaults(app_settings_t* settings)
{
  settings->log_period_ms = (uint32_t)CONFIG_APP_LOG_PERIOD_MS_DEFAULT;
  settings->fram_flush_watermark_records =
    (uint32_t)CONFIG_APP_FRAM_FLUSH_WATERMARK_RECORDS_DEFAULT;
  settings->sd_flush_period_ms = (uint32_t)CONFIG_APP_SD_PERIODIC_FLUSH_MS;
  settings->sd_batch_bytes_target = (uint32_t)CONFIG_APP_SD_BATCH_BYTES_TARGET;
  CalibrationModelInitIdentity(&settings->calibration);
  memset(&settings->calibration_context, 0,
         sizeof(settings->calibration_context));
  settings->calibration_context_valid = false;
  settings->calibration_points_count = 0;
  memset(settings->calibration_points, 0, sizeof(settings->calibration_points));
  snprintf(settings->tz_posix,
           sizeof(settings->tz_posix),
           "%s",
           APP_SETTINGS_TZ_DEFAULT_POSIX);
  settings->dst_enabled = true;
  settings->node_role = DefaultNodeRole();
  settings->allow_children =
    AppSettingsRoleDefaultAllowsChildren(settings->node_role);
  settings->allow_children_set = false;
  settings->display_units = APP_DISPLAY_UNITS_F;
  settings->display_attention_policy = AppSettingsDefaultDisplayAttentionPolicy();
  settings->display_attention_mask =
    AppSettingsDefaultDisplayAttentionMask();
  settings->net_mode = APP_NET_MODE_MESH;
  settings->mqtt_enabled = false;
  snprintf(settings->mqtt_broker_uri,
           sizeof(settings->mqtt_broker_uri),
           "%s",
           APP_SETTINGS_MQTT_BROKER_URI_DEFAULT);
  snprintf(settings->mqtt_topic_prefix,
           sizeof(settings->mqtt_topic_prefix),
           "%s",
           APP_SETTINGS_MQTT_TOPIC_PREFIX_DEFAULT);
  settings->mqtt_qos = 0;
  settings->mqtt_retain = false;
  settings->mqtt_bridge_mode = MQTT_BRIDGE_BROKER;
  g_display_attention_policy = settings->display_attention_policy;
  g_display_attention_mask = settings->display_attention_mask;
}

/**
 * @brief Execute DisplayAttentionMaskFromPolicy.
 * @param policy Parameter policy.
 * @return Return the function result.
 */
static display_attention_mask_t
DisplayAttentionMaskFromPolicy(uint32_t policy)
{
  display_attention_mask_t mask = 0;
  for (display_attention_item_t item = kDispAttnItemSdOut;
       item < kDispAttnItemCount;
       item = (display_attention_item_t)(item + 1)) {
    if (DisplayAttentionPolicyGet(policy, item) != DISP_SEV_OFF) {
      mask |= (display_attention_mask_t)(1u << (uint32_t)item);
    }
  }
  return mask;
}

/**
 * @brief Execute DisplayAttentionPolicyFromMask.
 * @param mask Parameter mask.
 * @return Return the function result.
 */
static uint32_t
DisplayAttentionPolicyFromMask(display_attention_mask_t mask)
{
  uint32_t policy = 0;
  for (display_attention_item_t item = kDispAttnItemSdOut;
       item < kDispAttnItemCount;
       item = (display_attention_item_t)(item + 1)) {
    const display_attention_mask_t bit =
      (display_attention_mask_t)(1u << (uint32_t)item);
    const display_attention_severity_t severity =
      ((mask & bit) != 0u) ? DISP_SEV_ERROR : DISP_SEV_OFF;
    policy = DisplayAttentionPolicySet(policy, item, severity);
  }
  return policy;
}

/**
 * @brief Execute ReadDouble.
 * @param handle Parameter handle.
 * @param key Parameter key.
 * @param value_out Parameter value_out.
 * @return Return the function result.
 */
static bool
ReadDouble(nvs_handle_t handle, const char* key, double* value_out)
{
  size_t data_size = sizeof(double);
  esp_err_t result = nvs_get_blob(handle, key, value_out, &data_size);
  return (result == ESP_OK && data_size == sizeof(double));
}

/**
 * @brief Execute LoadCalibrationContext.
 * @param handle Parameter handle.
 * @param context_out Parameter context_out.
 * @return Return the function result.
 */
static bool
LoadCalibrationContext(nvs_handle_t handle, calibration_context_t* context_out)
{
  uint8_t version = 0;
  esp_err_t version_result =
    nvs_get_u8(handle, kKeyCalContextVersion, &version);
  if (version_result != ESP_OK || version != kCalibrationContextVersion) {
    return false;
  }

  uint8_t conversion = 0;
  uint8_t wires = 0;
  uint8_t filter = 0;
  uint32_t table_version = 0;
  if (nvs_get_u8(handle, kKeyCalContextConversion, &conversion) != ESP_OK) {
    return false;
  }
  if (nvs_get_u8(handle, kKeyCalContextWires, &wires) != ESP_OK) {
    return false;
  }
  if (nvs_get_u8(handle, kKeyCalContextFilter, &filter) != ESP_OK) {
    return false;
  }
  if (nvs_get_u32(handle, kKeyCalContextTableVer, &table_version) != ESP_OK) {
    return false;
  }

  double rref_ohm = 0.0;
  double r0_ohm = 0.0;
  if (!ReadDouble(handle, kKeyCalContextRref, &rref_ohm)) {
    return false;
  }
  if (!ReadDouble(handle, kKeyCalContextR0, &r0_ohm)) {
    return false;
  }

  context_out->conversion_mode = conversion;
  context_out->wires = wires;
  context_out->filter_hz = filter;
  context_out->rref_ohm = rref_ohm;
  context_out->r0_ohm = r0_ohm;
  context_out->table_version = table_version;
  return true;
}

/**
 * @brief Execute OpenNvs.
 * @param handle_out Parameter handle_out.
 * @return Return the function result.
 */
static esp_err_t
OpenNvs(nvs_handle_t* handle_out)
{
  return nvs_open(kNvsNamespace, NVS_READWRITE, handle_out);
}

/**
 * @brief Execute AppSettingsLoad.
 * @param settings_out Parameter settings_out.
 * @return Return the function result.
 */
esp_err_t
AppSettingsLoad(app_settings_t* settings_out)
{
  if (settings_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  ApplyDefaults(settings_out);

  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "nvs_open failed: %s", esp_err_to_name(result));
    return result;
  }

  uint32_t log_period_ms = 0;
  result = nvs_get_u32(handle, kKeyLogPeriodMs, &log_period_ms);
  if (result == ESP_OK && log_period_ms >= 100 && log_period_ms <= 3600000) {
    settings_out->log_period_ms = log_period_ms;
  }

  uint32_t flush_wm = 0;
  result = nvs_get_u32(handle, kKeyFlushWatermark, &flush_wm);
  if (result == ESP_OK && flush_wm >= 1) {
    settings_out->fram_flush_watermark_records = flush_wm;
  }

  uint32_t sd_flush_ms = 0;
  result = nvs_get_u32(handle, kKeySdFlushPeriodMs, &sd_flush_ms);
  if (result == ESP_OK && sd_flush_ms >= 1000) {
    settings_out->sd_flush_period_ms = sd_flush_ms;
  }

  uint32_t sd_batch_bytes = 0;
  result = nvs_get_u32(handle, kKeySdBatchBytes, &sd_batch_bytes);
  if (result == ESP_OK && sd_batch_bytes >= 4096) {
    settings_out->sd_batch_bytes_target = sd_batch_bytes;
  }

  uint8_t cal_degree = 0;
  result = nvs_get_u8(handle, kKeyCalDegree, &cal_degree);
  uint8_t cal_mode = (uint8_t)CAL_FIT_MODE_LINEAR;
  esp_err_t mode_result = nvs_get_u8(handle, kKeyCalMode, &cal_mode);
  size_t coeff_bytes = sizeof(double) * CALIBRATION_MAX_POINTS;
  double coeffs[CALIBRATION_MAX_POINTS] = { 0 };

  esp_err_t coeff_result =
    nvs_get_blob(handle, kKeyCalCoeffs, coeffs, &coeff_bytes);
  if (result == ESP_OK && coeff_result == ESP_OK &&
      cal_degree <= CALIBRATION_MAX_DEGREE && coeff_bytes == sizeof(coeffs)) {
    settings_out->calibration.degree = cal_degree;
    memcpy(settings_out->calibration.coefficients, coeffs, sizeof(coeffs));
    settings_out->calibration.is_valid = true;
  } else {
    CalibrationModelInitIdentity(&settings_out->calibration);
  }
  if (mode_result == ESP_OK && cal_mode <= (uint8_t)CAL_FIT_MODE_POLY) {
    settings_out->calibration.mode = (calibration_fit_mode_t)cal_mode;
  } else if (settings_out->calibration.is_valid) {
    settings_out->calibration.mode = (settings_out->calibration.degree > 1)
                                       ? CAL_FIT_MODE_POLY
                                       : CAL_FIT_MODE_LINEAR;
  }

  uint8_t cal_points_count = 0;
  result = nvs_get_u8(handle, kKeyCalPointsCount, &cal_points_count);
  if (result == ESP_OK && cal_points_count <= CALIBRATION_MAX_POINTS) {
    size_t points_bytes =
      sizeof(calibration_point_t) * (size_t)cal_points_count;
    if (points_bytes > 0) {
      size_t points_bytes_copy = points_bytes;
      esp_err_t points_result = nvs_get_blob(
        handle, kKeyCalPoints, settings_out->calibration_points, &points_bytes_copy);
      if (points_result == ESP_OK && points_bytes_copy == points_bytes) {
        settings_out->calibration_points_count = cal_points_count;
      } else {
        settings_out->calibration_points_count = 0;
        memset(settings_out->calibration_points,
               0,
               sizeof(settings_out->calibration_points));
      }
    } else {
      settings_out->calibration_points_count = 0;
    }
  } else {
    settings_out->calibration_points_count = 0;
  }

  calibration_context_t loaded_context;
  if (LoadCalibrationContext(handle, &loaded_context)) {
    settings_out->calibration_context = loaded_context;
    settings_out->calibration_context_valid = true;
  } else {
    memset(&settings_out->calibration_context, 0,
           sizeof(settings_out->calibration_context));
    settings_out->calibration_context_valid = false;
  }

  size_t tz_len = sizeof(settings_out->tz_posix);
  result = nvs_get_str(handle, kKeyTzPosix, settings_out->tz_posix, &tz_len);
  if (result != ESP_OK || tz_len == 0 || tz_len > sizeof(settings_out->tz_posix)) {
    snprintf(settings_out->tz_posix,
             sizeof(settings_out->tz_posix),
             "%s",
             APP_SETTINGS_TZ_DEFAULT_POSIX);
  }

  uint8_t dst_enabled = settings_out->dst_enabled ? 1 : 0;
  result = nvs_get_u8(handle, kKeyDstEnabled, &dst_enabled);
  if (result == ESP_OK && dst_enabled <= 1) {
    settings_out->dst_enabled = (dst_enabled == 1);
  }

  uint8_t node_role = (uint8_t)settings_out->node_role;
  result = nvs_get_u8(handle, kKeyNodeRole, &node_role);
  if (result == ESP_OK && node_role <= (uint8_t)APP_NODE_ROLE_RELAY) {
    settings_out->node_role = (app_node_role_t)node_role;
  }

  uint8_t allow_children_set = settings_out->allow_children_set ? 1 : 0;
  esp_err_t allow_children_set_result =
    nvs_get_u8(handle, kKeyAllowChildrenSet, &allow_children_set);
  const bool allow_children_set_present =
    (allow_children_set_result == ESP_OK);
  if (allow_children_set_present && allow_children_set <= 1) {
    settings_out->allow_children_set = (allow_children_set == 1);
  }

  if (settings_out->allow_children_set) {
    uint8_t allow_children = settings_out->allow_children ? 1 : 0;
    result = nvs_get_u8(handle, kKeyAllowChildren, &allow_children);
    if (result == ESP_OK && allow_children <= 1) {
      settings_out->allow_children = (allow_children == 1);
    } else {
      settings_out->allow_children_set = false;
      settings_out->allow_children =
        AppSettingsRoleDefaultAllowsChildren(settings_out->node_role);
    }
  } else if (!allow_children_set_present) {
    uint8_t allow_children = settings_out->allow_children ? 1 : 0;
    result = nvs_get_u8(handle, kKeyAllowChildren, &allow_children);
    if (result == ESP_OK && allow_children <= 1) {
      settings_out->allow_children = (allow_children == 1);
      settings_out->allow_children_set = true;
    } else {
      settings_out->allow_children =
        AppSettingsRoleDefaultAllowsChildren(settings_out->node_role);
    }
  } else {
    settings_out->allow_children =
      AppSettingsRoleDefaultAllowsChildren(settings_out->node_role);
  }

  uint8_t display_units = (uint8_t)settings_out->display_units;
  result = nvs_get_u8(handle, kKeyDisplayUnits, &display_units);
  if (result == ESP_OK && display_units <= (uint8_t)APP_DISPLAY_UNITS_F) {
    settings_out->display_units = (app_display_units_t)display_units;
  }

  uint8_t net_mode = (uint8_t)settings_out->net_mode;
  result = nvs_get_u8(handle, kKeyNetMode, &net_mode);
  if (result == ESP_OK &&
      net_mode <= (uint8_t)APP_NET_MODE_NONE) {
    settings_out->net_mode = (app_net_mode_t)net_mode;
  }

  uint8_t mqtt_enabled = settings_out->mqtt_enabled ? 1 : 0;
  result = nvs_get_u8(handle, kKeyMqttEnabled, &mqtt_enabled);
  if (result == ESP_OK && mqtt_enabled <= 1) {
    settings_out->mqtt_enabled = (mqtt_enabled == 1);
  }

  size_t broker_len = sizeof(settings_out->mqtt_broker_uri);
  result = nvs_get_str(handle,
                       kKeyMqttBrokerUri,
                       settings_out->mqtt_broker_uri,
                       &broker_len);
  if (result != ESP_OK || broker_len == 0 ||
      broker_len > sizeof(settings_out->mqtt_broker_uri)) {
    snprintf(settings_out->mqtt_broker_uri,
             sizeof(settings_out->mqtt_broker_uri),
             "%s",
             APP_SETTINGS_MQTT_BROKER_URI_DEFAULT);
  }

  size_t prefix_len = sizeof(settings_out->mqtt_topic_prefix);
  result = nvs_get_str(handle,
                       kKeyMqttTopicPrefix,
                       settings_out->mqtt_topic_prefix,
                       &prefix_len);
  if (result != ESP_OK || prefix_len == 0 ||
      prefix_len > sizeof(settings_out->mqtt_topic_prefix)) {
    snprintf(settings_out->mqtt_topic_prefix,
             sizeof(settings_out->mqtt_topic_prefix),
             "%s",
             APP_SETTINGS_MQTT_TOPIC_PREFIX_DEFAULT);
  }

  uint8_t mqtt_qos = settings_out->mqtt_qos;
  result = nvs_get_u8(handle, kKeyMqttQos, &mqtt_qos);
  if (result == ESP_OK && mqtt_qos <= 1) {
    settings_out->mqtt_qos = mqtt_qos;
  }

  uint8_t mqtt_retain = settings_out->mqtt_retain ? 1 : 0;
  result = nvs_get_u8(handle, kKeyMqttRetain, &mqtt_retain);
  if (result == ESP_OK && mqtt_retain <= 1) {
    settings_out->mqtt_retain = (mqtt_retain == 1);
  }

  uint8_t mqtt_bridge_mode = (uint8_t)settings_out->mqtt_bridge_mode;
  result = nvs_get_u8(handle, kKeyMqttBridgeMode, &mqtt_bridge_mode);
  if (result == ESP_OK && mqtt_bridge_mode <= (uint8_t)MQTT_BRIDGE_BOTH) {
    settings_out->mqtt_bridge_mode = (mqtt_bridge_mode_t)mqtt_bridge_mode;
  }

  uint32_t display_attention_policy =
    (uint32_t)settings_out->display_attention_policy;
  esp_err_t policy_result =
    nvs_get_u32(handle, kKeyDisplayAttentionPolicy, &display_attention_policy);
  const bool policy_present = (policy_result == ESP_OK);
  if (policy_present) {
    settings_out->display_attention_policy = display_attention_policy;
  }

  uint32_t display_attention_mask =
    (uint32_t)settings_out->display_attention_mask;
  esp_err_t mask_result =
    nvs_get_u32(handle, kKeyDisplayAttentionMask, &display_attention_mask);
  if (!policy_present && mask_result == ESP_OK) {
    settings_out->display_attention_policy =
      DisplayAttentionPolicyFromMask(
        (display_attention_mask_t)display_attention_mask);
    esp_err_t migrate_result =
      nvs_set_u32(handle,
                  kKeyDisplayAttentionPolicy,
                  settings_out->display_attention_policy);
    if (migrate_result == ESP_OK) {
      migrate_result = nvs_commit(handle);
    }
    if (migrate_result != ESP_OK) {
      ESP_LOGW(kTag,
               "display attention policy migration failed: %s",
               esp_err_to_name(migrate_result));
    } else {
      ESP_LOGI(kTag, "display attention policy migrated from legacy mask");
    }
  } else if (!policy_present && mask_result != ESP_OK &&
             mask_result != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(kTag,
             "display attention mask load failed: %s",
             esp_err_to_name(mask_result));
  } else if (!policy_present && mask_result == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(kTag,
             "display attention policy missing; using default: %s",
             esp_err_to_name(policy_result));
  }

  settings_out->display_attention_mask =
    DisplayAttentionMaskFromPolicy(settings_out->display_attention_policy);
  g_display_attention_policy = settings_out->display_attention_policy;
  g_display_attention_mask = settings_out->display_attention_mask;

  nvs_close(handle);
  ESP_LOGI(
    kTag,
    "Loaded: period=%ums wm=%u sd_flush_ms=%u sd_batch=%u deg=%u cal_points=%u tz=%s dst=%u role=%s allow_children=%u display_units=%s net_mode=%s mqtt_en=%u mqtt_uri=%s mqtt_pfx=%s mqtt_qos=%u mqtt_ret=%u mqtt_bridge=%s disp_attn_pol=0x%08" PRIX32 " disp_attn_mask=0x%08" PRIX32,
    (unsigned)settings_out->log_period_ms,
    (unsigned)settings_out->fram_flush_watermark_records,
    (unsigned)settings_out->sd_flush_period_ms,
    (unsigned)settings_out->sd_batch_bytes_target,
    (unsigned)settings_out->calibration.degree,
    (unsigned)settings_out->calibration_points_count,
    settings_out->tz_posix,
    settings_out->dst_enabled ? 1u : 0u,
    AppSettingsRoleToString(settings_out->node_role),
    settings_out->allow_children ? 1u : 0u,
    AppSettingsDisplayUnitsToString(settings_out->display_units),
    AppSettingsNetModeToString(settings_out->net_mode),
    settings_out->mqtt_enabled ? 1u : 0u,
    settings_out->mqtt_broker_uri,
    settings_out->mqtt_topic_prefix,
    (unsigned)settings_out->mqtt_qos,
    settings_out->mqtt_retain ? 1u : 0u,
    AppSettingsMqttBridgeModeToString(settings_out->mqtt_bridge_mode),
    (uint32_t)settings_out->display_attention_policy,
    (uint32_t)settings_out->display_attention_mask);
  return ESP_OK;
}

/**
 * @brief Execute AppSettingsSaveLogPeriodMs.
 * @param log_period_ms Parameter log_period_ms.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveLogPeriodMs(uint32_t log_period_ms)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u32(handle, kKeyLogPeriodMs, log_period_ms);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveFramFlushWatermarkRecords.
 * @param watermark_records Parameter watermark_records.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveFramFlushWatermarkRecords(uint32_t watermark_records)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u32(handle, kKeyFlushWatermark, watermark_records);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveSdFlushPeriodMs.
 * @param period_ms Parameter period_ms.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveSdFlushPeriodMs(uint32_t period_ms)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u32(handle, kKeySdFlushPeriodMs, period_ms);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveSdBatchBytes.
 * @param batch_bytes Parameter batch_bytes.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveSdBatchBytes(uint32_t batch_bytes)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u32(handle, kKeySdBatchBytes, batch_bytes);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveCalibrationWithContext.
 * @param model Parameter model.
 * @param context Parameter context.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveCalibrationWithContext(const calibration_model_t* model,
                                      const calibration_context_t* context)
{
  if (model == NULL || !model->is_valid || context == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kKeyCalDegree, model->degree);
  if (result == ESP_OK) {
    result = nvs_set_u8(handle, kKeyCalMode, (uint8_t)model->mode);
  }
  if (result == ESP_OK) {
    result = nvs_set_blob(handle,
                          kKeyCalCoeffs,
                          model->coefficients,
                          sizeof(double) * CALIBRATION_MAX_POINTS);
  }
  if (result == ESP_OK) {
    result = nvs_set_u8(handle, kKeyCalContextVersion,
                        kCalibrationContextVersion);
  }
  if (result == ESP_OK) {
    result = nvs_set_u8(handle, kKeyCalContextConversion,
                        context->conversion_mode);
  }
  if (result == ESP_OK) {
    result = nvs_set_u8(handle, kKeyCalContextWires, context->wires);
  }
  if (result == ESP_OK) {
    result = nvs_set_u8(handle, kKeyCalContextFilter, context->filter_hz);
  }
  if (result == ESP_OK) {
    result = nvs_set_blob(
      handle, kKeyCalContextRref, &context->rref_ohm, sizeof(double));
  }
  if (result == ESP_OK) {
    result = nvs_set_blob(
      handle, kKeyCalContextR0, &context->r0_ohm, sizeof(double));
  }
  if (result == ESP_OK) {
    result =
      nvs_set_u32(handle, kKeyCalContextTableVer, context->table_version);
  }
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsBuildCalibrationContextFromReader.
 * @param context Parameter context.
 * @param reader Parameter reader.
 */
void
AppSettingsBuildCalibrationContextFromReader(calibration_context_t* context,
                                             const max31865_reader_t* reader)
{
  if (context == NULL || reader == NULL) {
    return;
  }
  context->conversion_mode = (uint8_t)reader->conversion;
  context->wires = reader->wires;
  context->filter_hz = reader->filter_hz;
  context->rref_ohm = reader->rref_ohm;
  context->r0_ohm = reader->rtd_nominal_ohm;
  context->table_version =
    (reader->conversion == kMax31865ConversionTablePt100)
      ? (uint32_t)PT100_TABLE_LENGTH
      : 0u;
}

/**
 * @brief Execute AppSettingsSaveCalibrationPoints.
 * @param points Parameter points.
 * @param points_count Parameter points_count.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveCalibrationPoints(const calibration_point_t* points,
                                 size_t points_count)
{
  if (points_count > CALIBRATION_MAX_POINTS) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (points_count > 0 && points == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kKeyCalPointsCount, (uint8_t)points_count);
  if (result == ESP_OK) {
    if (points_count > 0) {
      result = nvs_set_blob(handle,
                            kKeyCalPoints,
                            points,
                            sizeof(calibration_point_t) * points_count);
    } else {
      esp_err_t erase_result = nvs_erase_key(handle, kKeyCalPoints);
      if (erase_result != ESP_OK && erase_result != ESP_ERR_NVS_NOT_FOUND) {
        result = erase_result;
      }
    }
  }
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveTimeZone.
 * @param tz_posix Parameter tz_posix.
 * @param dst_enabled Parameter dst_enabled.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveTimeZone(const char* tz_posix, bool dst_enabled)
{
  if (tz_posix == NULL || tz_posix[0] == '\0' ||
      strlen(tz_posix) >= APP_SETTINGS_TZ_POSIX_MAX_LEN) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_str(handle, kKeyTzPosix, tz_posix);
  if (result == ESP_OK) {
    result = nvs_set_u8(handle, kKeyDstEnabled, dst_enabled ? 1 : 0);
  }
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveNodeRole.
 * @param node_role Parameter node_role.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveNodeRole(app_node_role_t node_role)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kKeyNodeRole, (uint8_t)node_role);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveAllowChildren.
 * @param allow_children Parameter allow_children.
 * @param explicit_setting Parameter explicit_setting.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveAllowChildren(bool allow_children, bool explicit_setting)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kKeyAllowChildren, allow_children ? 1 : 0);
  if (result == ESP_OK) {
    result =
      nvs_set_u8(handle, kKeyAllowChildrenSet, explicit_setting ? 1 : 0);
  }
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveDisplayUnits.
 * @param units Parameter units.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveDisplayUnits(app_display_units_t units)
{
  if (units != APP_DISPLAY_UNITS_C && units != APP_DISPLAY_UNITS_F) {
    return ESP_ERR_INVALID_ARG;
  }
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kKeyDisplayUnits, (uint8_t)units);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveNetMode.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveNetMode(app_net_mode_t mode)
{
  if (mode != APP_NET_MODE_MESH && mode != APP_NET_MODE_DIRECT_WIFI &&
      mode != APP_NET_MODE_NONE) {
    return ESP_ERR_INVALID_ARG;
  }
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kKeyNetMode, (uint8_t)mode);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  if (result == ESP_OK) {
    g_net_mode_revision++;
  }
  return result;
}

/**
 * @brief Execute AppSettingsGetNetModeRevision.
 * @return Return the function result.
 */
uint32_t
AppSettingsGetNetModeRevision(void)
{
  return g_net_mode_revision;
}

/**
 * @brief Execute AppSettingsSaveMqttEnabled.
 * @param enabled Parameter enabled.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveMqttEnabled(bool enabled)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u8(handle, kKeyMqttEnabled, enabled ? 1 : 0);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveMqttBrokerUri.
 * @param uri Parameter uri.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveMqttBrokerUri(const char* uri)
{
  if (uri == NULL || uri[0] == '\0' ||
      strlen(uri) >= sizeof(((app_settings_t*)0)->mqtt_broker_uri)) {
    return ESP_ERR_INVALID_ARG;
  }
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_str(handle, kKeyMqttBrokerUri, uri);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveMqttTopicPrefix.
 * @param prefix Parameter prefix.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveMqttTopicPrefix(const char* prefix)
{
  if (prefix == NULL || prefix[0] == '\0' ||
      strlen(prefix) >= sizeof(((app_settings_t*)0)->mqtt_topic_prefix)) {
    return ESP_ERR_INVALID_ARG;
  }
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_str(handle, kKeyMqttTopicPrefix, prefix);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveMqttQos.
 * @param qos Parameter qos.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveMqttQos(uint8_t qos)
{
  if (qos > 1) {
    return ESP_ERR_INVALID_ARG;
  }
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u8(handle, kKeyMqttQos, qos);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveMqttRetain.
 * @param retain Parameter retain.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveMqttRetain(bool retain)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u8(handle, kKeyMqttRetain, retain ? 1 : 0);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsSaveMqttBridgeMode.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveMqttBridgeMode(mqtt_bridge_mode_t mode)
{
  if (mode > MQTT_BRIDGE_BOTH) {
    return ESP_ERR_INVALID_ARG;
  }
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u8(handle, kKeyMqttBridgeMode, (uint8_t)mode);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

/**
 * @brief Execute AppSettingsDefaultDisplayAttentionMask.
 * @return Return the function result.
 */
display_attention_mask_t
AppSettingsDefaultDisplayAttentionMask(void)
{
  return DisplayAttentionMaskFromPolicy(
    AppSettingsDefaultDisplayAttentionPolicy());
}

/**
 * @brief Execute AppSettingsSaveDisplayAttentionMask.
 * @param mask Parameter mask.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveDisplayAttentionMask(display_attention_mask_t mask)
{
  const uint32_t policy = DisplayAttentionPolicyFromMask(mask);
  return AppSettingsSaveDisplayAttentionPolicy(policy);
}

/**
 * @brief Execute AppSettingsGetDisplayAttentionMask.
 * @return Return the function result.
 */
display_attention_mask_t
AppSettingsGetDisplayAttentionMask(void)
{
  return g_display_attention_mask;
}

/**
 * @brief Execute AppSettingsDefaultDisplayAttentionPolicy.
 * @return Return the function result.
 */
uint32_t
AppSettingsDefaultDisplayAttentionPolicy(void)
{
  uint32_t policy = 0;
  policy = DisplayAttentionPolicySet(policy, kDispAttnItemSdOut, DISP_SEV_ERROR);
  policy = DisplayAttentionPolicySet(policy, kDispAttnItemSdIo, DISP_SEV_ERROR);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemFramOvr, DISP_SEV_ERROR);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemRtdFault, DISP_SEV_ERROR);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemTimeBad, DISP_SEV_ERROR);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemMeshDown, DISP_SEV_WARN);
  return policy;
}

/**
 * @brief Execute AppSettingsSaveDisplayAttentionPolicy.
 * @param policy Parameter policy.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveDisplayAttentionPolicy(uint32_t policy)
{
  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }
  result = nvs_set_u32(handle, kKeyDisplayAttentionPolicy, policy);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  if (result == ESP_OK) {
    g_display_attention_policy = policy;
    g_display_attention_mask = DisplayAttentionMaskFromPolicy(policy);
  }
  return result;
}

/**
 * @brief Execute AppSettingsGetDisplayAttentionPolicy.
 * @return Return the function result.
 */
uint32_t
AppSettingsGetDisplayAttentionPolicy(void)
{
  return g_display_attention_policy;
}

/**
 * @brief Execute AppSettingsApplyTimeZone.
 * @param settings Parameter settings.
 */
void
AppSettingsApplyTimeZone(const app_settings_t* settings)
{
  if (settings == NULL) {
    return;
  }
  if (settings->tz_posix[0] == '\0') {
    return;
  }
  setenv("TZ", settings->tz_posix, 1);
  tzset();
  ESP_LOGI(kTag, "Applied TZ=%s", settings->tz_posix);
}
