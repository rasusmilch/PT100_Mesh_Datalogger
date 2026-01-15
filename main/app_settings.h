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
    APP_UNITS_GPIO_PULL_NONE = 0,
    APP_UNITS_GPIO_PULL_UP = 1,
    APP_UNITS_GPIO_PULL_DOWN = 2,
  } app_units_gpio_pull_t;

  typedef enum
  {
    APP_NET_MODE_MESH = 0,
    APP_NET_MODE_DIRECT_WIFI = 1,
    APP_NET_MODE_NONE = 2,
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
    // Calibration tracking (UTC-based)
    int64_t cal_last_utc;
    int64_t cal_last_override_utc;
    uint16_t cal_due_count;
    uint8_t cal_due_unit;
    uint16_t cal_due_override_count;
    uint8_t cal_due_override_unit;
    bool rtd_ema_enabled;
    uint16_t rtd_ema_alpha_permille;
    char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
    bool dst_enabled;
    app_node_role_t node_role;
    bool allow_children;
    bool allow_children_set;
    app_display_units_t display_units;
    int32_t units_gpio_pin;
    app_units_gpio_pull_t units_gpio_pull;
    bool units_gpio_c_level_high;
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
/**
 * @brief Execute AppSettingsLoad.
 * @param settings_out Parameter settings_out.
 * @return Return the function result.
 */
  esp_err_t AppSettingsLoad(app_settings_t* settings_out);

  // Persists updated log interval to NVS.
/**
 * @brief Execute AppSettingsSaveLogPeriodMs.
 * @param log_period_ms Parameter log_period_ms.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveLogPeriodMs(uint32_t log_period_ms);

  // Persists updated FRAM flush watermark to NVS.
/**
 * @brief Execute AppSettingsSaveFramFlushWatermarkRecords.
 * @param watermark_records Parameter watermark_records.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveFramFlushWatermarkRecords(
    uint32_t watermark_records);

/**
 * @brief Execute AppSettingsSaveSdFlushPeriodMs.
 * @param period_ms Parameter period_ms.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveSdFlushPeriodMs(uint32_t period_ms);
/**
 * @brief Execute AppSettingsSaveSdBatchBytes.
 * @param batch_bytes Parameter batch_bytes.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveSdBatchBytes(uint32_t batch_bytes);

  // Persists updated calibration model to NVS.
/**
 * @brief Execute AppSettingsSaveCalibrationWithContext.
 * @param model Parameter model.
 * @param context Parameter context.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveCalibrationWithContext(
    const calibration_model_t* model,
    const calibration_context_t* context);

  // Persists calibration tracking/due-date metadata.
/**
 * @brief Execute AppSettingsSaveCalibrationSchedule.
 * @param settings Parameter settings.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveCalibrationSchedule(const app_settings_t* settings);

/**
 * @brief Execute AppSettingsBuildCalibrationContextFromReader.
 * @param context Parameter context.
 * @param reader Parameter reader.
 */
  void AppSettingsBuildCalibrationContextFromReader(
    calibration_context_t* context,
    const max31865_reader_t* reader);

  // Persists updated calibration points to NVS.
/**
 * @brief Execute AppSettingsSaveCalibrationPoints.
 * @param points Parameter points.
 * @param points_count Parameter points_count.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveCalibrationPoints(
    const calibration_point_t* points,
    size_t points_count);

  // Persists updated RTD EMA enabled flag.
/**
 * @brief Execute AppSettingsSaveRtdEmaEnabled.
 * @param enabled Parameter enabled.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveRtdEmaEnabled(bool enabled);

  // Persists updated RTD EMA alpha (permille).
/**
 * @brief Execute AppSettingsSaveRtdEmaAlphaPermille.
 * @param permille Parameter permille.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveRtdEmaAlphaPermille(uint16_t permille);

  // Persists updated timezone string + DST toggle.
/**
 * @brief Execute AppSettingsSaveTimeZone.
 * @param tz_posix Parameter tz_posix.
 * @param dst_enabled Parameter dst_enabled.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveTimeZone(const char* tz_posix, bool dst_enabled);

  // Persists updated node role.
/**
 * @brief Execute AppSettingsSaveNodeRole.
 * @param node_role Parameter node_role.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveNodeRole(app_node_role_t node_role);

  // Persists updated allow_children setting.
/**
 * @brief Execute AppSettingsSaveAllowChildren.
 * @param allow_children Parameter allow_children.
 * @param explicit_setting Parameter explicit_setting.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveAllowChildren(bool allow_children,
                                         bool explicit_setting);

  // Role helpers.
/**
 * @brief Execute AppSettingsRoleToString.
 * @param role Parameter role.
 * @return Return the function result.
 */
  const char* AppSettingsRoleToString(app_node_role_t role);
/**
 * @brief Execute AppSettingsParseRole.
 * @param value Parameter value.
 * @param role_out Parameter role_out.
 * @return Return the function result.
 */
  bool AppSettingsParseRole(const char* value, app_node_role_t* role_out);
/**
 * @brief Execute AppSettingsRoleDefaultAllowsChildren.
 * @param role Parameter role.
 * @return Return the function result.
 */
  bool AppSettingsRoleDefaultAllowsChildren(app_node_role_t role);

  // Display units helpers.
/**
 * @brief Execute AppSettingsDisplayUnitsToString.
 * @param units Parameter units.
 * @return Return the function result.
 */
  const char* AppSettingsDisplayUnitsToString(app_display_units_t units);
/**
 * @brief Execute AppSettingsParseDisplayUnits.
 * @param value Parameter value.
 * @param units_out Parameter units_out.
 * @return Return the function result.
 */
  bool AppSettingsParseDisplayUnits(const char* value,
                                    app_display_units_t* units_out);

  // Network mode helpers.
/**
 * @brief Execute AppSettingsNetModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
  const char* AppSettingsNetModeToString(app_net_mode_t mode);
/**
 * @brief Execute AppSettingsParseNetMode.
 * @param value Parameter value.
 * @param mode_out Parameter mode_out.
 * @return Return the function result.
 */
  bool AppSettingsParseNetMode(const char* value, app_net_mode_t* mode_out);

  // MQTT bridge mode helpers.
/**
 * @brief Execute AppSettingsMqttBridgeModeToString.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
  const char* AppSettingsMqttBridgeModeToString(mqtt_bridge_mode_t mode);
/**
 * @brief Execute AppSettingsParseMqttBridgeMode.
 * @param value Parameter value.
 * @param mode_out Parameter mode_out.
 * @return Return the function result.
 */
  bool AppSettingsParseMqttBridgeMode(const char* value,
                                      mqtt_bridge_mode_t* mode_out);

  // Persists updated display units.
/**
 * @brief Execute AppSettingsSaveDisplayUnits.
 * @param units Parameter units.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveDisplayUnits(app_display_units_t units);

  // Persists updated units GPIO pin override.
/**
 * @brief Execute AppSettingsSaveUnitsGpioPin.
 * @param pin Parameter pin.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveUnitsGpioPin(int32_t pin);

  // Persists updated units GPIO pull override.
/**
 * @brief Execute AppSettingsSaveUnitsGpioPull.
 * @param pull Parameter pull.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveUnitsGpioPull(app_units_gpio_pull_t pull);

  // Persists updated units GPIO level mapping for Celsius.
/**
 * @brief Execute AppSettingsSaveUnitsGpioCLevel.
 * @param c_level_high Parameter c_level_high.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveUnitsGpioCLevel(bool c_level_high);

  // Persists updated network mode.
/**
 * @brief Execute AppSettingsSaveNetMode.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveNetMode(app_net_mode_t mode);

/**
 * @brief Execute AppSettingsGetNetModeRevision.
 * @return Return the function result.
 */
  uint32_t AppSettingsGetNetModeRevision(void);

/**
 * @brief Execute AppSettingsSaveMqttEnabled.
 * @param enabled Parameter enabled.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveMqttEnabled(bool enabled);

/**
 * @brief Execute AppSettingsSaveMqttBrokerUri.
 * @param uri Parameter uri.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveMqttBrokerUri(const char* uri);

/**
 * @brief Execute AppSettingsSaveMqttTopicPrefix.
 * @param prefix Parameter prefix.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveMqttTopicPrefix(const char* prefix);

/**
 * @brief Execute AppSettingsSaveMqttQos.
 * @param qos Parameter qos.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveMqttQos(uint8_t qos);

/**
 * @brief Execute AppSettingsSaveMqttRetain.
 * @param retain Parameter retain.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveMqttRetain(bool retain);

/**
 * @brief Execute AppSettingsSaveMqttBridgeMode.
 * @param mode Parameter mode.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveMqttBridgeMode(mqtt_bridge_mode_t mode);

  // Persists updated display attention mask.
/**
 * @brief Execute AppSettingsSaveDisplayAttentionMask.
 * @param mask Parameter mask.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveDisplayAttentionMask(
    display_attention_mask_t mask);

/**
 * @brief Execute AppSettingsGetDisplayAttentionMask.
 * @return Return the function result.
 */
  display_attention_mask_t AppSettingsGetDisplayAttentionMask(void);

/**
 * @brief Execute AppSettingsDefaultDisplayAttentionMask.
 * @return Return the function result.
 */
  display_attention_mask_t AppSettingsDefaultDisplayAttentionMask(void);

  // Persists updated display attention policy.
/**
 * @brief Execute AppSettingsSaveDisplayAttentionPolicy.
 * @param policy Parameter policy.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveDisplayAttentionPolicy(uint32_t policy);

/**
 * @brief Execute AppSettingsGetDisplayAttentionPolicy.
 * @return Return the function result.
 */
  uint32_t AppSettingsGetDisplayAttentionPolicy(void);

/**
 * @brief Execute AppSettingsDefaultDisplayAttentionPolicy.
 * @return Return the function result.
 */
  uint32_t AppSettingsDefaultDisplayAttentionPolicy(void);

  // Applies TZ to the runtime environment.
/**
 * @brief Execute AppSettingsApplyTimeZone.
 * @param settings Parameter settings.
 */
  void AppSettingsApplyTimeZone(const app_settings_t* settings);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_APP_SETTINGS_H_
