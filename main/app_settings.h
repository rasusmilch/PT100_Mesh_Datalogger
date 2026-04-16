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
#define APP_SETTINGS_UNIT_SERIAL_MAX_LEN 48
#define APP_SETTINGS_CAL_METHOD_MAX_LEN 96
#define APP_SETTINGS_CAL_METAR_RAW_MAX_LEN 192
#define APP_SETTINGS_CAL_METAR_STATION_MAX_LEN 8
#define APP_SETTINGS_CAL_METAR_OBS_TOKEN_MAX_LEN 16
#define APP_SETTINGS_CAL_METAR_TEMP_TOKEN_MAX_LEN 16
#define APP_SETTINGS_CAL_METAR_FLAGS_MAX_LEN 16
#define APP_SETTINGS_CAL_METAR_REMARKS_MAX_LEN 80
#define APP_SETTINGS_CAL_METAR_OBS_ISO_MAX_LEN 32
#define APP_SETTINGS_CAL_METAR_METHOD_NOTE_MAX_LEN 96
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

  typedef struct
  {
    uint8_t valid;
    char source_type[8];
    char raw_metar[APP_SETTINGS_CAL_METAR_RAW_MAX_LEN];
    char station_id[APP_SETTINGS_CAL_METAR_STATION_MAX_LEN];
    char observation_token[APP_SETTINGS_CAL_METAR_OBS_TOKEN_MAX_LEN];
    char observation_iso_utc[APP_SETTINGS_CAL_METAR_OBS_ISO_MAX_LEN];
    uint8_t observation_resolved;
    uint16_t elevation_ft;
    float altimeter_inhg;
    float station_pressure_inhg;
    float satpt_c;
    char auto_or_cor[APP_SETTINGS_CAL_METAR_FLAGS_MAX_LEN];
    char temp_dew_token[APP_SETTINGS_CAL_METAR_TEMP_TOKEN_MAX_LEN];
    char remarks[APP_SETTINGS_CAL_METAR_REMARKS_MAX_LEN];
    int64_t stored_at_utc_epoch;
    char method_note[APP_SETTINGS_CAL_METAR_METHOD_NOTE_MAX_LEN];
  } calibration_metar_reference_t;

  // Deployment guidance:
  // - Dense plant/fixed power: enable children on RELAY nodes; selectively enable
  //   on SENSOR nodes only where needed.
  // - Sparse/unknown geometry: allow_children on SENSOR nodes can improve reach
  //   at the cost of more chatter.

  typedef struct
  {
    uint32_t log_period_ms;
    uint32_t display_sample_period_ms;
    uint32_t fram_flush_watermark_records;
    uint32_t sd_flush_period_ms;
    uint32_t sd_batch_bytes_target;
    bool sd_verify_readback;
    uint32_t rtc_resync_period_ms;
    calibration_model_t calibration;
    calibration_context_t calibration_context;
    bool calibration_context_valid;
    calibration_point_t calibration_points[CALIBRATION_MAX_POINTS];
    uint8_t calibration_points_count;
    calibration_domain_t calibration_domain;
    uint16_t cal_window_duration_s;
    uint16_t cal_trend_ema_alpha_permille;
    // Calibration tracking (UTC-based)
    int64_t cal_last_utc;
    int64_t cal_last_override_utc;
    uint16_t cal_due_count;
    uint8_t cal_due_unit;
    uint16_t cal_due_override_count;
    uint8_t cal_due_override_unit;
    calibration_metar_reference_t cal_metar;
    bool rtd_ema_enabled;
    uint16_t rtd_ema_alpha_permille;
    uint32_t rtd_fault_assert_ms;
    uint32_t rtd_fault_clear_ms;
    char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
    char unit_serial[APP_SETTINGS_UNIT_SERIAL_MAX_LEN];  // Deprecated: do not use for audit identity; MAC is used as device id.
    char cal_method[APP_SETTINGS_CAL_METHOD_MAX_LEN];
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

/**
 * @brief Persist updated display/sample period to NVS.
 * @param display_sample_period_ms Display/sample period in milliseconds.
 * @return ESP_OK on success; otherwise an ESP-IDF error code.
 */
  esp_err_t
  AppSettingsSaveDisplaySamplePeriodMs(uint32_t display_sample_period_ms);

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
  esp_err_t AppSettingsSaveSdVerifyReadback(bool enabled);

/**
 * @brief Execute AppSettingsSaveRtcResyncPeriodMs.
 * @param period_ms Parameter period_ms.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveRtcResyncPeriodMs(uint32_t period_ms);

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

/**
 * @brief Persist the active calibration domain metadata.
 * @param domain Calibration domain enum value.
 * @return ESP_OK on success; otherwise an ESP-IDF error code.
 */
  esp_err_t AppSettingsSaveCalibrationDomain(calibration_domain_t domain);
  esp_err_t AppSettingsSaveCalibrationConfig(uint16_t window_s,
                                             uint16_t ema_alpha_permille);

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

  // Persists RTD fault debounce settings.
/**
 * @brief Execute AppSettingsSaveRtdFaultDebounceMs.
 * @param assert_ms Parameter assert_ms.
 * @param clear_ms Parameter clear_ms.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveRtdFaultDebounceMs(uint32_t assert_ms,
                                              uint32_t clear_ms);

  // Persists updated timezone string + DST toggle.
/**
 * @brief Execute AppSettingsSaveTimeZone.
 * @param tz_posix Parameter tz_posix.
 * @param dst_enabled Parameter dst_enabled.
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveTimeZone(const char* tz_posix, bool dst_enabled);

/**
 * @brief Persist the calibration method metadata string.
 * @param method Calibration method string (may be empty to clear).
 * @return Return the function result.
 */
  esp_err_t AppSettingsSaveCalibrationMethod(const char* method);

/**
 * @brief Persist the calibration METAR steam-reference metadata block.
 * @param metar METAR reference metadata to store.
 * @return Return the function result.
 */
  esp_err_t
  AppSettingsSaveCalibrationMetar(const calibration_metar_reference_t* metar);

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
/**
 * @brief Resolve the runtime-effective network mode from role + configured
 * setting.
 * @param role Active node role.
 * @param configured_mode Stored configured network mode.
 * @return Effective network mode used by runtime/supervisor.
 */
  app_net_mode_t AppSettingsGetEffectiveNetMode(app_node_role_t role,
                                                app_net_mode_t configured_mode);
/**
 * @brief Return a short reason when effective net mode overrides configured.
 * @param role Active node role.
 * @param configured_mode Stored configured network mode.
 * @return Override reason string, or NULL when no override applies.
 */
  const char* AppSettingsGetNetModeOverrideReason(app_node_role_t role,
                                                  app_net_mode_t configured_mode);

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
