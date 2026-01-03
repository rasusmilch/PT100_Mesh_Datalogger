#ifndef PT100_LOGGER_APP_SETTINGS_H_
#define PT100_LOGGER_APP_SETTINGS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "calibration.h"
#include "display_attention.h"
#include "esp_err.h"
#include "max31865_reader.h"

#define APP_SETTINGS_TZ_POSIX_MAX_LEN 64
#define APP_SETTINGS_TZ_DEFAULT_POSIX "CST6CDT,M3.2.0/2,M11.1.0/2"
#define APP_SETTINGS_TZ_DEFAULT_STD "CST6"
#define APP_SETTINGS_MQTT_BROKER_URI_DEFAULT "mqtt://192.168.1.50"
#define APP_SETTINGS_MQTT_TOPIC_PREFIX_DEFAULT "pt100"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    APP_NODE_ROLE_ROOT = 0,
    APP_NODE_ROLE_SENSOR = 1,
    APP_NODE_ROLE_RELAY = 2,
  } app_node_role_t;

  typedef enum
  {
    APP_DISPLAY_UNITS_C = 0,
    APP_DISPLAY_UNITS_F = 1,
  } app_display_units_t;

  typedef enum
  {
    APP_NET_MODE_MESH = 0,
    APP_NET_MODE_DIRECT_WIFI = 1,
  } app_net_mode_t;

  typedef enum
  {
    MQTT_BRIDGE_OFF = 0,
    MQTT_BRIDGE_SERIAL = 1,
    MQTT_BRIDGE_BROKER = 2,
    MQTT_BRIDGE_BOTH = 3,
  } mqtt_bridge_mode_t;

  typedef struct
  {
    uint8_t conversion_mode;
    uint8_t wires;
    uint8_t filter_hz;
    double rref_ohm;
    double r0_ohm;
    uint32_t table_version;
  } calibration_context_t;

  // Deployment guidance:
  // - Dense plant/fixed power: enable children on RELAY nodes; selectively enable
  //   on SENSOR nodes only where needed.
  // - Sparse/unknown geometry: allow_children on SENSOR nodes can improve reach
  //   at the cost of more chatter.

  typedef struct
  {
    uint32_t log_period_ms;
    uint32_t fram_flush_watermark_records;
    uint32_t sd_flush_period_ms;
    uint32_t sd_batch_bytes_target;
    calibration_model_t calibration;
    calibration_context_t calibration_context;
    bool calibration_context_valid;
    calibration_point_t calibration_points[CALIBRATION_MAX_POINTS];
    uint8_t calibration_points_count;
    char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
    bool dst_enabled;
    app_node_role_t node_role;
    bool allow_children;
    bool allow_children_set;
    app_display_units_t display_units;
    uint32_t display_attention_policy;
    display_attention_mask_t display_attention_mask;
    app_net_mode_t net_mode;
    bool mqtt_enabled;
    char mqtt_broker_uri[128];
    char mqtt_topic_prefix[64];
    uint8_t mqtt_qos;
    bool mqtt_retain;
    mqtt_bridge_mode_t mqtt_bridge_mode;
  } app_settings_t;

  // Loads settings from NVS. If keys are missing or invalid, applies defaults.
  esp_err_t AppSettingsLoad(app_settings_t* settings_out);

  // Persists updated log interval to NVS.
  esp_err_t AppSettingsSaveLogPeriodMs(uint32_t log_period_ms);

  // Persists updated FRAM flush watermark to NVS.
  esp_err_t AppSettingsSaveFramFlushWatermarkRecords(
    uint32_t watermark_records);

  esp_err_t AppSettingsSaveSdFlushPeriodMs(uint32_t period_ms);
  esp_err_t AppSettingsSaveSdBatchBytes(uint32_t batch_bytes);

  // Persists updated calibration model to NVS.
  esp_err_t AppSettingsSaveCalibrationWithContext(
    const calibration_model_t* model,
    const calibration_context_t* context);

  void AppSettingsBuildCalibrationContextFromReader(
    calibration_context_t* context,
    const max31865_reader_t* reader);

  // Persists updated calibration points to NVS.
  esp_err_t AppSettingsSaveCalibrationPoints(
    const calibration_point_t* points,
    size_t points_count);

  // Persists updated timezone string + DST toggle.
  esp_err_t AppSettingsSaveTimeZone(const char* tz_posix, bool dst_enabled);

  // Persists updated node role.
  esp_err_t AppSettingsSaveNodeRole(app_node_role_t node_role);

  // Persists updated allow_children setting.
  esp_err_t AppSettingsSaveAllowChildren(bool allow_children,
                                         bool explicit_setting);

  // Role helpers.
  const char* AppSettingsRoleToString(app_node_role_t role);
  bool AppSettingsParseRole(const char* value, app_node_role_t* role_out);
  bool AppSettingsRoleDefaultAllowsChildren(app_node_role_t role);

  // Display units helpers.
  const char* AppSettingsDisplayUnitsToString(app_display_units_t units);
  bool AppSettingsParseDisplayUnits(const char* value,
                                    app_display_units_t* units_out);

  // Network mode helpers.
  const char* AppSettingsNetModeToString(app_net_mode_t mode);
  bool AppSettingsParseNetMode(const char* value, app_net_mode_t* mode_out);

  // MQTT bridge mode helpers.
  const char* AppSettingsMqttBridgeModeToString(mqtt_bridge_mode_t mode);
  bool AppSettingsParseMqttBridgeMode(const char* value,
                                      mqtt_bridge_mode_t* mode_out);

  // Persists updated display units.
  esp_err_t AppSettingsSaveDisplayUnits(app_display_units_t units);

  // Persists updated network mode.
  esp_err_t AppSettingsSaveNetMode(app_net_mode_t mode);

  esp_err_t AppSettingsSaveMqttEnabled(bool enabled);

  esp_err_t AppSettingsSaveMqttBrokerUri(const char* uri);

  esp_err_t AppSettingsSaveMqttTopicPrefix(const char* prefix);

  esp_err_t AppSettingsSaveMqttQos(uint8_t qos);

  esp_err_t AppSettingsSaveMqttRetain(bool retain);

  esp_err_t AppSettingsSaveMqttBridgeMode(mqtt_bridge_mode_t mode);

  // Persists updated display attention mask.
  esp_err_t AppSettingsSaveDisplayAttentionMask(
    display_attention_mask_t mask);

  display_attention_mask_t AppSettingsGetDisplayAttentionMask(void);

  display_attention_mask_t AppSettingsDefaultDisplayAttentionMask(void);

  // Persists updated display attention policy.
  esp_err_t AppSettingsSaveDisplayAttentionPolicy(uint32_t policy);

  uint32_t AppSettingsGetDisplayAttentionPolicy(void);

  uint32_t AppSettingsDefaultDisplayAttentionPolicy(void);

  // Applies TZ to the runtime environment.
  void AppSettingsApplyTimeZone(const app_settings_t* settings);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_APP_SETTINGS_H_
