#include "app_settings.h"

#include <inttypes.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "max31865_reader.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "pt100_table.h"
#include "time_civil.h"

static const char* kTag = "settings";

static const char* kNvsNamespace = "pt100_logger";
static const char* kKeyLogPeriodMs = "log_period_ms";
static const char* kKeyDisplaySamplePeriodMs = "disp_samp_ms";
static const char* kKeyFlushWatermark = "flush_wm_rec";
static const char* kKeySdFlushPeriodMs = "sd_flush_ms";
static const char* kKeySdBatchBytes = "sd_batch_bytes";
static const char* kKeyRtcResyncPeriodMs = "rtc_resync_ms";
static const char* kKeyCalDegree = "cal_deg";
static const char* kKeyCalMode = "cal_mode";
static const char* kKeyCalCoeffs = "cal_coeffs";
static const char* kKeyCalLastUtc = "cal_last_utc";
static const char* kKeyCalLastOverrideUtc = "cal_last_ovr";
static const char* kKeyCalDueCount = "cal_due_cnt";
static const char* kKeyCalDueUnit = "cal_due_unit";
static const char* kKeyCalDueOverrideCount = "cal_due_ovr_cnt";
static const char* kKeyCalDueOverrideUnit = "cal_due_ovr_u";
static const char* kKeyCalDueOverrideUnitLegacy = "cal_due_ovr_unit";
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
static const char* kKeyRtdEmaEnabled = "rtd_ema_en";
static const char* kKeyRtdEmaAlphaPermille = "rtd_ema_alpha";
static const char* kKeyRtdFaultAssertMs = "rtd_f_as_ms";
static const char* kKeyRtdFaultClearMs = "rtd_f_cl_ms";
static const char* kKeyTzPosix = "tz_posix";
static const char* kKeyDstEnabled = "dst_enabled";
static const char* kKeyNodeRole = "node_role";
static const char* kKeyAllowChildren = "allow_child";
static const char* kKeyAllowChildrenSet = "allow_child_set";
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
static const uint32_t kRtdFaultDebounceMaxMs = 60000u;
static uint32_t g_display_attention_policy = 0;
static display_attention_mask_t g_display_attention_mask = 0;
static uint32_t g_net_mode_revision = 0;
static const char* kKeySettingsBlob0 = "cfg0";
static const char* kKeySettingsBlob1 = "cfg1";

#define APP_SETTINGS_BLOB_MAGIC 0x53455454u // 'SETT'
#define APP_SETTINGS_BLOB_VERSION 7u

#pragma pack(push, 1)
typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t size;
  uint32_t generation;
  uint32_t crc32_le;
} app_settings_blob_header_t;

typedef struct
{
  int32_t raw_avg_mC;
  int32_t actual_mC;
  int32_t raw_stddev_mC;
  int32_t raw_avg_mOhm;
  int32_t raw_stddev_mOhm;
  uint16_t sample_count;
  uint8_t time_valid;
  int64_t timestamp_epoch_sec;
} legacy_calibration_point_v2_t;

typedef struct
{
  int32_t raw_avg_mC;
  int32_t actual_mC;
  int32_t raw_stddev_mC;
  uint16_t sample_count;
  uint8_t time_valid;
  int64_t timestamp_epoch_sec;
} legacy_calibration_point_v1_t;

typedef struct
{
  // v1: baseline blob layout using legacy_calibration_point_v1_t.
  uint32_t log_period_ms;
  uint32_t fram_flush_watermark_records;
  uint32_t sd_flush_period_ms;
  uint32_t sd_batch_bytes_target;
  uint32_t rtc_resync_period_ms;
  calibration_model_t calibration;
  calibration_context_t calibration_context;
  uint8_t calibration_context_valid;
  legacy_calibration_point_v1_t calibration_points[CALIBRATION_MAX_POINTS];
  uint8_t calibration_points_count;
  int64_t cal_last_utc;
  int64_t cal_last_override_utc;
  uint16_t cal_due_count;
  uint8_t cal_due_unit;
  uint16_t cal_due_override_count;
  uint8_t cal_due_override_unit;
  uint8_t rtd_ema_enabled;
  uint16_t rtd_ema_alpha_permille;
  uint32_t rtd_fault_assert_ms;
  uint32_t rtd_fault_clear_ms;
  char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
  char unit_serial[APP_SETTINGS_UNIT_SERIAL_MAX_LEN];
  char cal_method[APP_SETTINGS_CAL_METHOD_MAX_LEN];
  uint8_t dst_enabled;
  uint8_t node_role;
  uint8_t allow_children;
  uint8_t allow_children_set;
  uint32_t display_attention_policy;
  uint8_t net_mode;
  uint8_t mqtt_enabled;
  char mqtt_broker_uri[128];
  char mqtt_topic_prefix[64];
  uint8_t mqtt_qos;
  uint8_t mqtt_retain;
  uint8_t mqtt_bridge_mode;
} app_settings_persist_payload_v1_t;

typedef struct
{
  app_settings_blob_header_t header;
  app_settings_persist_payload_v1_t payload;
} app_settings_blob_v1_t;

typedef struct
{
  // v3: added sd_verify_readback, calibration_domain, and point mOhm fields.
  uint32_t log_period_ms;
  uint32_t fram_flush_watermark_records;
  uint32_t sd_flush_period_ms;
  uint32_t sd_batch_bytes_target;
  uint8_t sd_verify_readback;
  uint32_t rtc_resync_period_ms;
  calibration_model_t calibration;
  calibration_context_t calibration_context;
  uint8_t calibration_context_valid;
  calibration_point_t calibration_points[CALIBRATION_MAX_POINTS];
  uint8_t calibration_points_count;
  uint8_t calibration_domain;
  int64_t cal_last_utc;
  int64_t cal_last_override_utc;
  uint16_t cal_due_count;
  uint8_t cal_due_unit;
  uint16_t cal_due_override_count;
  uint8_t cal_due_override_unit;
  uint8_t rtd_ema_enabled;
  uint16_t rtd_ema_alpha_permille;
  uint32_t rtd_fault_assert_ms;
  uint32_t rtd_fault_clear_ms;
  char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
  char unit_serial[APP_SETTINGS_UNIT_SERIAL_MAX_LEN];
  char cal_method[APP_SETTINGS_CAL_METHOD_MAX_LEN];
  uint8_t dst_enabled;
  uint8_t node_role;
  uint8_t allow_children;
  uint8_t allow_children_set;
  uint32_t display_attention_policy;
  uint8_t net_mode;
  uint8_t mqtt_enabled;
  char mqtt_broker_uri[128];
  char mqtt_topic_prefix[64];
  uint8_t mqtt_qos;
  uint8_t mqtt_retain;
  uint8_t mqtt_bridge_mode;
} app_settings_persist_payload_v3_t;

typedef struct
{
  app_settings_blob_header_t header;
  app_settings_persist_payload_v3_t payload;
} app_settings_blob_v3_t;

typedef struct
{
  // v4: replaced sd_verify_readback with cal_metar and legacy point v2 layout.
  uint32_t log_period_ms;
  uint32_t fram_flush_watermark_records;
  uint32_t sd_flush_period_ms;
  uint32_t sd_batch_bytes_target;
  uint32_t rtc_resync_period_ms;
  calibration_model_t calibration;
  calibration_context_t calibration_context;
  uint8_t calibration_context_valid;
  legacy_calibration_point_v2_t calibration_points[CALIBRATION_MAX_POINTS];
  uint8_t calibration_points_count;
  uint8_t calibration_domain;
  int64_t cal_last_utc;
  int64_t cal_last_override_utc;
  uint16_t cal_due_count;
  uint8_t cal_due_unit;
  uint16_t cal_due_override_count;
  uint8_t cal_due_override_unit;
  calibration_metar_reference_t cal_metar;
  uint8_t rtd_ema_enabled;
  uint16_t rtd_ema_alpha_permille;
  uint32_t rtd_fault_assert_ms;
  uint32_t rtd_fault_clear_ms;
  char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
  char unit_serial[APP_SETTINGS_UNIT_SERIAL_MAX_LEN];
  char cal_method[APP_SETTINGS_CAL_METHOD_MAX_LEN];
  uint8_t dst_enabled;
  uint8_t node_role;
  uint8_t allow_children;
  uint8_t allow_children_set;
  uint32_t display_attention_policy;
  uint8_t net_mode;
  uint8_t mqtt_enabled;
  char mqtt_broker_uri[128];
  char mqtt_topic_prefix[64];
  uint8_t mqtt_qos;
  uint8_t mqtt_retain;
  uint8_t mqtt_bridge_mode;
} app_settings_persist_payload_v4_t;

typedef struct
{
  app_settings_blob_header_t header;
  app_settings_persist_payload_v4_t payload;
} app_settings_blob_v4_t;

typedef struct
{
  // v5: switched calibration_points to calibration_point_t.
  uint32_t log_period_ms;
  uint32_t fram_flush_watermark_records;
  uint32_t sd_flush_period_ms;
  uint32_t sd_batch_bytes_target;
  uint32_t rtc_resync_period_ms;
  calibration_model_t calibration;
  calibration_context_t calibration_context;
  uint8_t calibration_context_valid;
  calibration_point_t calibration_points[CALIBRATION_MAX_POINTS];
  uint8_t calibration_points_count;
  uint8_t calibration_domain;
  int64_t cal_last_utc;
  int64_t cal_last_override_utc;
  uint16_t cal_due_count;
  uint8_t cal_due_unit;
  uint16_t cal_due_override_count;
  uint8_t cal_due_override_unit;
  calibration_metar_reference_t cal_metar;
  uint8_t rtd_ema_enabled;
  uint16_t rtd_ema_alpha_permille;
  uint32_t rtd_fault_assert_ms;
  uint32_t rtd_fault_clear_ms;
  char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
  char unit_serial[APP_SETTINGS_UNIT_SERIAL_MAX_LEN];
  char cal_method[APP_SETTINGS_CAL_METHOD_MAX_LEN];
  uint8_t dst_enabled;
  uint8_t node_role;
  uint8_t allow_children;
  uint8_t allow_children_set;
  uint32_t display_attention_policy;
  uint8_t net_mode;
  uint8_t mqtt_enabled;
  char mqtt_broker_uri[128];
  char mqtt_topic_prefix[64];
  uint8_t mqtt_qos;
  uint8_t mqtt_retain;
  uint8_t mqtt_bridge_mode;
} app_settings_persist_payload_v5_t;

typedef struct
{
  // v6: added cal_window_duration_s and cal_trend_ema_alpha_permille.
  uint32_t log_period_ms;
  uint32_t fram_flush_watermark_records;
  uint32_t sd_flush_period_ms;
  uint32_t sd_batch_bytes_target;
  uint32_t rtc_resync_period_ms;
  calibration_model_t calibration;
  calibration_context_t calibration_context;
  uint8_t calibration_context_valid;
  calibration_point_t calibration_points[CALIBRATION_MAX_POINTS];
  uint8_t calibration_points_count;
  uint8_t calibration_domain;
  uint16_t cal_window_duration_s;
  uint16_t cal_trend_ema_alpha_permille;
  int64_t cal_last_utc;
  int64_t cal_last_override_utc;
  uint16_t cal_due_count;
  uint8_t cal_due_unit;
  uint16_t cal_due_override_count;
  uint8_t cal_due_override_unit;
  calibration_metar_reference_t cal_metar;
  uint8_t rtd_ema_enabled;
  uint16_t rtd_ema_alpha_permille;
  uint32_t rtd_fault_assert_ms;
  uint32_t rtd_fault_clear_ms;
  char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
  char unit_serial[APP_SETTINGS_UNIT_SERIAL_MAX_LEN];
  char cal_method[APP_SETTINGS_CAL_METHOD_MAX_LEN];
  uint8_t dst_enabled;
  uint8_t node_role;
  uint8_t allow_children;
  uint8_t allow_children_set;
  uint32_t display_attention_policy;
  uint8_t net_mode;
  uint8_t mqtt_enabled;
  char mqtt_broker_uri[128];
  char mqtt_topic_prefix[64];
  uint8_t mqtt_qos;
  uint8_t mqtt_retain;
  uint8_t mqtt_bridge_mode;
} app_settings_persist_payload_v6_t;

typedef struct
{
  app_settings_blob_header_t header;
  app_settings_persist_payload_v5_t payload;
} app_settings_blob_v5_t;

typedef struct
{
  app_settings_blob_header_t header;
  app_settings_persist_payload_v6_t payload;
} app_settings_blob_v6_t;

typedef struct
{
  // v7/current: added sd_verify_readback.
  uint32_t log_period_ms;
  uint32_t fram_flush_watermark_records;
  uint32_t sd_flush_period_ms;
  uint32_t sd_batch_bytes_target;
  uint8_t sd_verify_readback;
  uint32_t rtc_resync_period_ms;
  calibration_model_t calibration;
  calibration_context_t calibration_context;
  uint8_t calibration_context_valid;
  calibration_point_t calibration_points[CALIBRATION_MAX_POINTS];
  uint8_t calibration_points_count;
  uint8_t calibration_domain;
  uint16_t cal_window_duration_s;
  uint16_t cal_trend_ema_alpha_permille;
  int64_t cal_last_utc;
  int64_t cal_last_override_utc;
  uint16_t cal_due_count;
  uint8_t cal_due_unit;
  uint16_t cal_due_override_count;
  uint8_t cal_due_override_unit;
  calibration_metar_reference_t cal_metar;
  uint8_t rtd_ema_enabled;
  uint16_t rtd_ema_alpha_permille;
  uint32_t rtd_fault_assert_ms;
  uint32_t rtd_fault_clear_ms;
  char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
  char unit_serial[APP_SETTINGS_UNIT_SERIAL_MAX_LEN];
  char cal_method[APP_SETTINGS_CAL_METHOD_MAX_LEN];
  uint8_t dst_enabled;
  uint8_t node_role;
  uint8_t allow_children;
  uint8_t allow_children_set;
  uint32_t display_attention_policy;
  uint8_t net_mode;
  uint8_t mqtt_enabled;
  char mqtt_broker_uri[128];
  char mqtt_topic_prefix[64];
  uint8_t mqtt_qos;
  uint8_t mqtt_retain;
  uint8_t mqtt_bridge_mode;
} app_settings_persist_payload_t;

typedef struct
{
  app_settings_blob_header_t header;
  app_settings_persist_payload_t payload;
} app_settings_blob_t;
#pragma pack(pop)

_Static_assert(sizeof(app_settings_persist_payload_t) ==
                 (sizeof(app_settings_persist_payload_v6_t) + 1u),
               "v7 payload should differ from v6 only by sd_verify_readback");

static uint32_t g_settings_blob_generation = 0;
static bool g_saved_settings_valid = false;
static app_settings_t g_saved_settings;
// Shared save scratch avoids large app_settings_t stack frames in small RTOS
// tasks (e.g. cal_op) without introducing heap allocation/fragmentation.
static app_settings_t s_settings_save_scratch;
static SemaphoreHandle_t s_settings_save_mutex = NULL;
static StaticSemaphore_t s_settings_save_mutex_buf;
static portMUX_TYPE s_settings_save_mutex_init_lock =
  portMUX_INITIALIZER_UNLOCKED;

typedef void (*app_settings_mutator_fn)(app_settings_t* settings, void* context);

static esp_err_t
AppSettingsSaveBlob(const app_settings_t* settings);
static void
EnsureSettingsSaveMutexInitialized_(void);
static esp_err_t
PersistMutatedSettingsSnapshot_(app_settings_mutator_fn mutator, void* context);
static void
MutateLogPeriodMs_(app_settings_t* settings, void* context);
static void
MutateFramFlushWatermarkRecords_(app_settings_t* settings, void* context);
static void
MutateSdFlushPeriodMs_(app_settings_t* settings, void* context);
static void
MutateSdBatchBytes_(app_settings_t* settings, void* context);
static void
MutateSdVerifyReadback_(app_settings_t* settings, void* context);
static void
MutateRtcResyncPeriodMs_(app_settings_t* settings, void* context);
static void
MutateCalibrationWithContext_(app_settings_t* settings, void* context);
static void
MutateCalibrationSchedule_(app_settings_t* settings, void* context);
static void
MutateCalibrationPoints_(app_settings_t* settings, void* context);
static void
MutateCalibrationDomain_(app_settings_t* settings, void* context);
static void
MutateCalibrationConfig_(app_settings_t* settings, void* context);
static void
MutateRtdEmaEnabled_(app_settings_t* settings, void* context);
static void
MutateRtdEmaAlphaPermille_(app_settings_t* settings, void* context);
static void
MutateRtdFaultDebounceMs_(app_settings_t* settings, void* context);
static void
MutateTimeZone_(app_settings_t* settings, void* context);
static void
MutateCalibrationMethod_(app_settings_t* settings, void* context);
static void
MutateCalibrationMetar_(app_settings_t* settings, void* context);
static void
MutateNodeRole_(app_settings_t* settings, void* context);
static void
MutateAllowChildren_(app_settings_t* settings, void* context);
static void
MutateNetMode_(app_settings_t* settings, void* context);
static void
MutateMqttEnabled_(app_settings_t* settings, void* context);
static void
MutateMqttBrokerUri_(app_settings_t* settings, void* context);
static void
MutateMqttTopicPrefix_(app_settings_t* settings, void* context);
static void
MutateMqttQos_(app_settings_t* settings, void* context);
static void
MutateMqttRetain_(app_settings_t* settings, void* context);
static void
MutateMqttBridgeMode_(app_settings_t* settings, void* context);
static void
MutateDisplayAttentionPolicy_(app_settings_t* settings, void* context);

static esp_err_t
OpenNvs(nvs_handle_t* handle_out);
static void
MarkCalibrationPointDriftLegacyUnavailable_(calibration_point_t* point);
/**
 * @brief Load optional display/sample period from standalone NVS key.
 * @param handle Open NVS handle.
 * @param settings_out Settings snapshot to update when key is valid.
 */
static void
AppSettingsMaybeLoadDisplaySamplePeriodMs(nvs_handle_t handle,
                                          app_settings_t* settings_out);
static bool
IsPrintableSettingString(const char* value, size_t max_len);
static bool
IsPlausibleTzPosixString_(const char* value, size_t max_len);
static bool
IsPlausibleMqttUri_(const char* value, size_t max_len);
static bool
ValidateAndSanitizePersistedPayload_(app_settings_persist_payload_t* payload);

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

static app_display_units_t
DefaultDisplayUnits(void)
{
#if CONFIG_APP_DISPLAY_UNITS_DEFAULT_C
  return APP_DISPLAY_UNITS_C;
#else
  return APP_DISPLAY_UNITS_F;
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
AppSettingsParseDisplayUnits(const char* value, app_display_units_t* units_out)
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
 * @brief Execute AppSettingsGetEffectiveNetMode.
 * @param role Parameter role.
 * @param configured_mode Parameter configured_mode.
 * @return Return the function result.
 */
app_net_mode_t
AppSettingsGetEffectiveNetMode(app_node_role_t role,
                               app_net_mode_t configured_mode)
{
  if (role == APP_NODE_ROLE_ROOT) {
    return APP_NET_MODE_MESH;
  }
  return configured_mode;
}

/**
 * @brief Execute AppSettingsGetNetModeOverrideReason.
 * @param role Parameter role.
 * @param configured_mode Parameter configured_mode.
 * @return Return the function result.
 */
const char*
AppSettingsGetNetModeOverrideReason(app_node_role_t role,
                                    app_net_mode_t configured_mode)
{
  const app_net_mode_t effective_mode =
    AppSettingsGetEffectiveNetMode(role, configured_mode);
  if (effective_mode != configured_mode && role == APP_NODE_ROLE_ROOT) {
    return "role=root forces mesh";
  }
  return NULL;
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

static void
MarkCalibrationPointDriftLegacyUnavailable_(calibration_point_t* point)
{
  if (point == NULL) {
    return;
  }
  point->captured_drift_mC_per_min = CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN;
  point->captured_delta_mC = CAL_CAPTURE_DELTA_UNAVAILABLE_MC;
  point->capture_drift_limit_mC_per_min =
    CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN;
  point->drift_limit_source = (uint8_t)CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE;
  point->captured_window_s = CAL_CAPTURE_WINDOW_S_UNAVAILABLE;
  point->captured_ema_alpha_permille = CAL_CAPTURE_EMA_ALPHA_UNAVAILABLE_PERMILLE;
}

/**
 * @brief Execute ApplyDefaults.
 * @param settings Parameter settings.
 */
static void
ApplyDefaults(app_settings_t* settings)
{
  settings->log_period_ms = (uint32_t)CONFIG_APP_LOG_PERIOD_MS_DEFAULT;
  settings->display_sample_period_ms =
    (uint32_t)CONFIG_APP_DISPLAY_SAMPLE_PERIOD_MS_DEFAULT;
  settings->fram_flush_watermark_records =
    (uint32_t)CONFIG_APP_FRAM_FLUSH_WATERMARK_RECORDS_DEFAULT;
  settings->sd_flush_period_ms = (uint32_t)CONFIG_APP_SD_PERIODIC_FLUSH_MS;
  settings->sd_batch_bytes_target = (uint32_t)CONFIG_APP_SD_BATCH_BYTES_TARGET;
  settings->sd_verify_readback = false;
  settings->rtc_resync_period_ms = 3600000u;
  CalibrationModelInitIdentity(&settings->calibration);
  memset(
    &settings->calibration_context, 0, sizeof(settings->calibration_context));
  settings->calibration_context_valid = false;
  settings->calibration_points_count = 0;
  settings->calibration_domain = CAL_DOMAIN_TEMP_C;
  settings->cal_window_duration_s = CAL_WINDOW_DURATION_DEFAULT_S;
  settings->cal_trend_ema_alpha_permille = CAL_TREND_EMA_ALPHA_DEFAULT_PERMILLE;
  memset(settings->calibration_points, 0, sizeof(settings->calibration_points));
  for (size_t i = 0; i < CALIBRATION_MAX_POINTS; ++i) {
    MarkCalibrationPointDriftLegacyUnavailable_(&settings->calibration_points[i]);
  }
  settings->cal_last_utc = 0;
  settings->cal_last_override_utc = 0;
  settings->cal_due_count = 0;
  settings->cal_due_unit = 0;
  settings->cal_due_override_count = 0;
  settings->cal_due_override_unit = 0;
  memset(&settings->cal_metar, 0, sizeof(settings->cal_metar));
  settings->rtd_ema_enabled = false;
  settings->rtd_ema_alpha_permille = 200;
  settings->rtd_fault_assert_ms = 2000u;
  settings->rtd_fault_clear_ms = 3000u;
  snprintf(settings->tz_posix,
           sizeof(settings->tz_posix),
           "%s",
           APP_SETTINGS_TZ_DEFAULT_POSIX);
  settings->unit_serial[0] = '\0';
  settings->cal_method[0] = '\0';
  settings->dst_enabled = true;
  settings->node_role = DefaultNodeRole();
  settings->allow_children =
    AppSettingsRoleDefaultAllowsChildren(settings->node_role);
  settings->allow_children_set = false;
  settings->display_units = DefaultDisplayUnits();

  settings->display_attention_policy =
    AppSettingsDefaultDisplayAttentionPolicy();
  settings->display_attention_mask = AppSettingsDefaultDisplayAttentionMask();
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

static void
ValidateCalibrationDueSettings(uint16_t* count, uint8_t* unit)
{
  if (count == NULL || unit == NULL) {
    return;
  }
  if (*count == 0) {
    *unit = 0;
    return;
  }
  if (*unit < (uint8_t)CAL_DUE_UNIT_DAYS ||
      *unit > (uint8_t)CAL_DUE_UNIT_YEARS) {
    *count = 0;
    *unit = 0;
  }
}

/**
 * @brief Compute the CRC32 for a settings blob with the CRC field cleared.
 * @param blob Parameter blob.
 * @return Return the computed CRC32 value.
 */
static uint32_t
ComputeSettingsBlobCrc32(const app_settings_blob_t* blob)
{
  if (blob == NULL) {
    return 0;
  }
  app_settings_blob_t temp;
  memcpy(&temp, blob, sizeof(temp));
  temp.header.crc32_le = 0;
  return esp_rom_crc32_le(0, (const uint8_t*)&temp, sizeof(temp));
}

/**
 * @brief Determine if a settings blob passes basic integrity checks.
 * @param blob Parameter blob.
 * @return Return true when the blob header and CRC are valid.
 */
static bool
SettingsBlobLooksValid(const app_settings_blob_t* blob)
{
  if (blob == NULL) {
    return false;
  }
  if (blob->header.magic != APP_SETTINGS_BLOB_MAGIC) {
    return false;
  }
  if (blob->header.version == APP_SETTINGS_BLOB_VERSION) {
    if (blob->header.size != sizeof(app_settings_persist_payload_t)) {
      return false;
    }
  } else if (blob->header.version == 3u) {
    if (blob->header.size != sizeof(app_settings_persist_payload_v3_t)) {
      return false;
    }
  } else if (blob->header.version == 4u) {
    if (blob->header.size != sizeof(app_settings_persist_payload_v4_t)) {
      return false;
    }
  } else if (blob->header.version == 5u) {
    if (blob->header.size != sizeof(app_settings_persist_payload_v5_t)) {
      return false;
    }
  } else if (blob->header.version == 6u) {
    if (blob->header.size != sizeof(app_settings_persist_payload_v6_t)) {
      return false;
    }
  } else if (blob->header.version == 1u) {
    if (blob->header.size != sizeof(app_settings_persist_payload_v1_t)) {
      return false;
    }
  } else {
    return false;
  }
  uint32_t crc_saved = blob->header.crc32_le;
  if (blob->header.version == APP_SETTINGS_BLOB_VERSION) {
    app_settings_blob_t temp;
    memcpy(&temp, blob, sizeof(temp));
    temp.header.crc32_le = 0;
    return esp_rom_crc32_le(0, (const uint8_t*)&temp, sizeof(temp)) == crc_saved;
  }
  if (blob->header.version == 3u) {
    app_settings_blob_v3_t temp_v3;
    memcpy(&temp_v3, blob, sizeof(temp_v3));
    temp_v3.header.crc32_le = 0;
    return esp_rom_crc32_le(0, (const uint8_t*)&temp_v3, sizeof(temp_v3)) ==
           crc_saved;
  }
  if (blob->header.version == 4u) {
    app_settings_blob_v4_t temp_v4;
    memcpy(&temp_v4, blob, sizeof(temp_v4));
    temp_v4.header.crc32_le = 0;
    return esp_rom_crc32_le(0, (const uint8_t*)&temp_v4, sizeof(temp_v4)) ==
           crc_saved;
  }
  if (blob->header.version == 5u) {
    app_settings_blob_header_t header = blob->header;
    app_settings_persist_payload_v5_t payload_v5;
    memcpy(&payload_v5, &blob->payload, sizeof(payload_v5));
    header.crc32_le = 0;
    typedef struct
    {
      app_settings_blob_header_t header;
      app_settings_persist_payload_v5_t payload;
    } app_settings_blob_v5_t;
    app_settings_blob_v5_t temp_v5 = { .header = header, .payload = payload_v5 };
    return esp_rom_crc32_le(0, (const uint8_t*)&temp_v5, sizeof(temp_v5)) ==
           crc_saved;
  }
  if (blob->header.version == 6u) {
    app_settings_blob_header_t header = blob->header;
    app_settings_persist_payload_v6_t payload_v6;
    memcpy(&payload_v6, &blob->payload, sizeof(payload_v6));
    header.crc32_le = 0;
    typedef struct
    {
      app_settings_blob_header_t header;
      app_settings_persist_payload_v6_t payload;
    } app_settings_blob_v6_t;
    app_settings_blob_v6_t temp_v6 = { .header = header, .payload = payload_v6 };
    return esp_rom_crc32_le(0, (const uint8_t*)&temp_v6, sizeof(temp_v6)) ==
           crc_saved;
  }
  app_settings_blob_v1_t temp_v1;
  memcpy(&temp_v1, blob, sizeof(temp_v1));
  temp_v1.header.crc32_le = 0;
  return esp_rom_crc32_le(0, (const uint8_t*)&temp_v1, sizeof(temp_v1)) == crc_saved;
}

/**
 * @brief Read a settings blob from NVS and validate its header/CRC.
 * @param handle Parameter handle.
 * @param key Parameter key.
 * @param blob_out Parameter blob_out.
 * @return Return true when a valid blob was read.
 */
static bool
ReadSettingsBlob(nvs_handle_t handle,
                 const char* key,
                 app_settings_blob_t* blob_out)
{
  if (key == NULL || blob_out == NULL) {
    return false;
  }
  uint8_t raw[sizeof(app_settings_blob_t)] = { 0 };
  size_t blob_size = sizeof(raw);
  esp_err_t result = nvs_get_blob(handle, key, raw, &blob_size);
  if (result != ESP_OK || blob_size < sizeof(app_settings_blob_header_t)) {
    return false;
  }
  const app_settings_blob_header_t* header =
    (const app_settings_blob_header_t*)raw;
  if (header->magic != APP_SETTINGS_BLOB_MAGIC) {
    return false;
  }
  if (header->version == APP_SETTINGS_BLOB_VERSION &&
      blob_size == sizeof(app_settings_blob_t)) {
    memcpy(blob_out, raw, sizeof(app_settings_blob_t));
    return SettingsBlobLooksValid(blob_out);
  }
  if (header->version == 3u && blob_size == sizeof(app_settings_blob_v3_t)) {
    const app_settings_blob_v3_t* old_blob = (const app_settings_blob_v3_t*)raw;
    app_settings_blob_v3_t crc_blob = *old_blob;
    const uint32_t crc_saved = crc_blob.header.crc32_le;
    crc_blob.header.crc32_le = 0;
    const uint32_t crc =
      esp_rom_crc32_le(0, (const uint8_t*)&crc_blob, sizeof(crc_blob));
    if (crc != crc_saved) {
      return false;
    }
    memset(blob_out, 0, sizeof(*blob_out));
    blob_out->header.magic = old_blob->header.magic;
    blob_out->header.version = APP_SETTINGS_BLOB_VERSION;
    blob_out->header.size = sizeof(app_settings_persist_payload_t);
    blob_out->header.generation = old_blob->header.generation;
    const app_settings_persist_payload_v3_t* old = &old_blob->payload;
    app_settings_persist_payload_t* now = &blob_out->payload;
    now->log_period_ms = old->log_period_ms;
    now->fram_flush_watermark_records = old->fram_flush_watermark_records;
    now->sd_flush_period_ms = old->sd_flush_period_ms;
    now->sd_batch_bytes_target = old->sd_batch_bytes_target;
    now->rtc_resync_period_ms = old->rtc_resync_period_ms;
    now->calibration = old->calibration;
    now->calibration_context = old->calibration_context;
    now->calibration_context_valid = old->calibration_context_valid;
    memcpy(now->calibration_points,
           old->calibration_points,
           sizeof(now->calibration_points));
    now->calibration_points_count = old->calibration_points_count;
    now->calibration_domain = old->calibration_domain;
    now->cal_window_duration_s = CAL_WINDOW_DURATION_DEFAULT_S;
    now->cal_trend_ema_alpha_permille = CAL_TREND_EMA_ALPHA_DEFAULT_PERMILLE;
    now->cal_last_utc = old->cal_last_utc;
    now->cal_last_override_utc = old->cal_last_override_utc;
    now->cal_due_count = old->cal_due_count;
    now->cal_due_unit = old->cal_due_unit;
    now->cal_due_override_count = old->cal_due_override_count;
    now->cal_due_override_unit = old->cal_due_override_unit;
    memset(&now->cal_metar, 0, sizeof(now->cal_metar));
    now->rtd_ema_enabled = old->rtd_ema_enabled;
    now->rtd_ema_alpha_permille = old->rtd_ema_alpha_permille;
    now->rtd_fault_assert_ms = old->rtd_fault_assert_ms;
    now->rtd_fault_clear_ms = old->rtd_fault_clear_ms;
    memcpy(now->tz_posix, old->tz_posix, sizeof(now->tz_posix));
    memcpy(now->unit_serial, old->unit_serial, sizeof(now->unit_serial));
    memcpy(now->cal_method, old->cal_method, sizeof(now->cal_method));
    now->dst_enabled = old->dst_enabled;
    now->node_role = old->node_role;
    now->allow_children = old->allow_children;
    now->allow_children_set = old->allow_children_set;
    now->display_attention_policy = old->display_attention_policy;
    now->net_mode = old->net_mode;
    now->mqtt_enabled = old->mqtt_enabled;
    memcpy(
      now->mqtt_broker_uri, old->mqtt_broker_uri, sizeof(now->mqtt_broker_uri));
    memcpy(now->mqtt_topic_prefix,
           old->mqtt_topic_prefix,
           sizeof(now->mqtt_topic_prefix));
    now->mqtt_qos = old->mqtt_qos;
    now->mqtt_retain = old->mqtt_retain;
    now->mqtt_bridge_mode = old->mqtt_bridge_mode;
    blob_out->header.crc32_le = ComputeSettingsBlobCrc32(blob_out);
    ESP_LOGI(kTag,
             "settings blob %s migrated v3 -> v%" PRIu32,
             key,
             (uint32_t)APP_SETTINGS_BLOB_VERSION);
    return true;
  }
  if (header->version == 4u && blob_size == sizeof(app_settings_blob_v4_t)) {
    const app_settings_blob_v4_t* old_blob = (const app_settings_blob_v4_t*)raw;
    app_settings_blob_v4_t crc_blob = *old_blob;
    const uint32_t crc_saved = crc_blob.header.crc32_le;
    crc_blob.header.crc32_le = 0;
    const uint32_t crc =
      esp_rom_crc32_le(0, (const uint8_t*)&crc_blob, sizeof(crc_blob));
    if (crc != crc_saved) {
      return false;
    }
    memset(blob_out, 0, sizeof(*blob_out));
    blob_out->header.magic = old_blob->header.magic;
    blob_out->header.version = APP_SETTINGS_BLOB_VERSION;
    blob_out->header.size = sizeof(app_settings_persist_payload_t);
    blob_out->header.generation = old_blob->header.generation;
    const app_settings_persist_payload_v4_t* old = &old_blob->payload;
    app_settings_persist_payload_t* now = &blob_out->payload;
    now->log_period_ms = old->log_period_ms;
    now->fram_flush_watermark_records = old->fram_flush_watermark_records;
    now->sd_flush_period_ms = old->sd_flush_period_ms;
    now->sd_batch_bytes_target = old->sd_batch_bytes_target;
    now->rtc_resync_period_ms = old->rtc_resync_period_ms;
    now->calibration = old->calibration;
    now->calibration_context = old->calibration_context;
    now->calibration_context_valid = old->calibration_context_valid;
    now->calibration_points_count = old->calibration_points_count;
    now->calibration_domain = old->calibration_domain;
    now->cal_window_duration_s = CAL_WINDOW_DURATION_DEFAULT_S;
    now->cal_trend_ema_alpha_permille = CAL_TREND_EMA_ALPHA_DEFAULT_PERMILLE;
    for (size_t i = 0; i < CALIBRATION_MAX_POINTS; ++i) {
      now->calibration_points[i].raw_avg_mC = old->calibration_points[i].raw_avg_mC;
      now->calibration_points[i].actual_mC = old->calibration_points[i].actual_mC;
      now->calibration_points[i].raw_stddev_mC = old->calibration_points[i].raw_stddev_mC;
      now->calibration_points[i].raw_avg_mOhm = old->calibration_points[i].raw_avg_mOhm;
      now->calibration_points[i].raw_stddev_mOhm = old->calibration_points[i].raw_stddev_mOhm;
      now->calibration_points[i].sample_count = old->calibration_points[i].sample_count;
      now->calibration_points[i].time_valid = old->calibration_points[i].time_valid;
      now->calibration_points[i].timestamp_epoch_sec = old->calibration_points[i].timestamp_epoch_sec;
      MarkCalibrationPointDriftLegacyUnavailable_(&now->calibration_points[i]);
    }
    now->cal_last_utc = old->cal_last_utc;
    now->cal_last_override_utc = old->cal_last_override_utc;
    now->cal_due_count = old->cal_due_count;
    now->cal_due_unit = old->cal_due_unit;
    now->cal_due_override_count = old->cal_due_override_count;
    now->cal_due_override_unit = old->cal_due_override_unit;
    now->cal_metar = old->cal_metar;
    now->rtd_ema_enabled = old->rtd_ema_enabled;
    now->rtd_ema_alpha_permille = old->rtd_ema_alpha_permille;
    now->rtd_fault_assert_ms = old->rtd_fault_assert_ms;
    now->rtd_fault_clear_ms = old->rtd_fault_clear_ms;
    memcpy(now->tz_posix, old->tz_posix, sizeof(now->tz_posix));
    memcpy(now->unit_serial, old->unit_serial, sizeof(now->unit_serial));
    memcpy(now->cal_method, old->cal_method, sizeof(now->cal_method));
    now->dst_enabled = old->dst_enabled;
    now->node_role = old->node_role;
    now->allow_children = old->allow_children;
    now->allow_children_set = old->allow_children_set;
    now->display_attention_policy = old->display_attention_policy;
    now->net_mode = old->net_mode;
    now->mqtt_enabled = old->mqtt_enabled;
    memcpy(now->mqtt_broker_uri, old->mqtt_broker_uri, sizeof(now->mqtt_broker_uri));
    memcpy(now->mqtt_topic_prefix, old->mqtt_topic_prefix, sizeof(now->mqtt_topic_prefix));
    now->mqtt_qos = old->mqtt_qos;
    now->mqtt_retain = old->mqtt_retain;
    now->mqtt_bridge_mode = old->mqtt_bridge_mode;
    blob_out->header.crc32_le = ComputeSettingsBlobCrc32(blob_out);
    ESP_LOGI(kTag,
             "settings blob %s migrated v4 -> v%" PRIu32,
             key,
             (uint32_t)APP_SETTINGS_BLOB_VERSION);
    return true;
  }
  if (header->version == 5u) {
    if (blob_size == sizeof(app_settings_blob_v5_t)) {
      const app_settings_blob_v5_t* old_blob =
        (const app_settings_blob_v5_t*)raw;
      app_settings_blob_v5_t crc_blob = *old_blob;
      const uint32_t crc_saved = crc_blob.header.crc32_le;
      crc_blob.header.crc32_le = 0;
      const uint32_t crc =
        esp_rom_crc32_le(0, (const uint8_t*)&crc_blob, sizeof(crc_blob));
      if (crc != crc_saved) {
        return false;
      }
      memset(blob_out, 0, sizeof(*blob_out));
      blob_out->header = old_blob->header;
      blob_out->header.version = APP_SETTINGS_BLOB_VERSION;
      blob_out->header.size = sizeof(app_settings_persist_payload_t);
      blob_out->payload.log_period_ms = old_blob->payload.log_period_ms;
      blob_out->payload.fram_flush_watermark_records =
        old_blob->payload.fram_flush_watermark_records;
      blob_out->payload.sd_flush_period_ms = old_blob->payload.sd_flush_period_ms;
      blob_out->payload.sd_batch_bytes_target = old_blob->payload.sd_batch_bytes_target;
      blob_out->payload.sd_verify_readback = 0u;
      blob_out->payload.rtc_resync_period_ms = old_blob->payload.rtc_resync_period_ms;
      blob_out->payload.calibration = old_blob->payload.calibration;
      blob_out->payload.calibration_context = old_blob->payload.calibration_context;
      blob_out->payload.calibration_context_valid =
        old_blob->payload.calibration_context_valid;
      memcpy(blob_out->payload.calibration_points,
             old_blob->payload.calibration_points,
             sizeof(blob_out->payload.calibration_points));
      blob_out->payload.calibration_points_count =
        old_blob->payload.calibration_points_count;
      blob_out->payload.calibration_domain = old_blob->payload.calibration_domain;
      blob_out->payload.cal_window_duration_s = CAL_WINDOW_DURATION_DEFAULT_S;
      blob_out->payload.cal_trend_ema_alpha_permille =
        CAL_TREND_EMA_ALPHA_DEFAULT_PERMILLE;
      blob_out->payload.cal_last_utc = old_blob->payload.cal_last_utc;
      blob_out->payload.cal_last_override_utc = old_blob->payload.cal_last_override_utc;
      blob_out->payload.cal_due_count = old_blob->payload.cal_due_count;
      blob_out->payload.cal_due_unit = old_blob->payload.cal_due_unit;
      blob_out->payload.cal_due_override_count =
        old_blob->payload.cal_due_override_count;
      blob_out->payload.cal_due_override_unit =
        old_blob->payload.cal_due_override_unit;
      blob_out->payload.cal_metar = old_blob->payload.cal_metar;
      blob_out->payload.rtd_ema_enabled = old_blob->payload.rtd_ema_enabled;
      blob_out->payload.rtd_ema_alpha_permille =
        old_blob->payload.rtd_ema_alpha_permille;
      blob_out->payload.rtd_fault_assert_ms = old_blob->payload.rtd_fault_assert_ms;
      blob_out->payload.rtd_fault_clear_ms = old_blob->payload.rtd_fault_clear_ms;
      memcpy(blob_out->payload.tz_posix,
             old_blob->payload.tz_posix,
             sizeof(blob_out->payload.tz_posix));
      memcpy(blob_out->payload.unit_serial,
             old_blob->payload.unit_serial,
             sizeof(blob_out->payload.unit_serial));
      memcpy(blob_out->payload.cal_method,
             old_blob->payload.cal_method,
             sizeof(blob_out->payload.cal_method));
      blob_out->payload.dst_enabled = old_blob->payload.dst_enabled;
      blob_out->payload.node_role = old_blob->payload.node_role;
      blob_out->payload.allow_children = old_blob->payload.allow_children;
      blob_out->payload.allow_children_set = old_blob->payload.allow_children_set;
      blob_out->payload.display_attention_policy =
        old_blob->payload.display_attention_policy;
      blob_out->payload.net_mode = old_blob->payload.net_mode;
      blob_out->payload.mqtt_enabled = old_blob->payload.mqtt_enabled;
      memcpy(blob_out->payload.mqtt_broker_uri,
             old_blob->payload.mqtt_broker_uri,
             sizeof(blob_out->payload.mqtt_broker_uri));
      memcpy(blob_out->payload.mqtt_topic_prefix,
             old_blob->payload.mqtt_topic_prefix,
             sizeof(blob_out->payload.mqtt_topic_prefix));
      blob_out->payload.mqtt_qos = old_blob->payload.mqtt_qos;
      blob_out->payload.mqtt_retain = old_blob->payload.mqtt_retain;
      blob_out->payload.mqtt_bridge_mode = old_blob->payload.mqtt_bridge_mode;
      blob_out->header.crc32_le = ComputeSettingsBlobCrc32(blob_out);
      ESP_LOGI(kTag,
               "settings blob %s migrated v5 -> v%" PRIu32,
               key,
               (uint32_t)APP_SETTINGS_BLOB_VERSION);
      return true;
    }
  }
  if (header->version == 6u) {
    if (blob_size == sizeof(app_settings_blob_v6_t)) {
      const app_settings_blob_v6_t* old_blob =
        (const app_settings_blob_v6_t*)raw;
      app_settings_blob_v6_t crc_blob = *old_blob;
      const uint32_t crc_saved = crc_blob.header.crc32_le;
      crc_blob.header.crc32_le = 0;
      const uint32_t crc =
        esp_rom_crc32_le(0, (const uint8_t*)&crc_blob, sizeof(crc_blob));
      if (crc != crc_saved) {
        return false;
      }
      memset(blob_out, 0, sizeof(*blob_out));
      blob_out->header = old_blob->header;
      blob_out->header.version = APP_SETTINGS_BLOB_VERSION;
      blob_out->header.size = sizeof(app_settings_persist_payload_t);
      blob_out->payload.log_period_ms = old_blob->payload.log_period_ms;
      blob_out->payload.fram_flush_watermark_records =
        old_blob->payload.fram_flush_watermark_records;
      blob_out->payload.sd_flush_period_ms = old_blob->payload.sd_flush_period_ms;
      blob_out->payload.sd_batch_bytes_target = old_blob->payload.sd_batch_bytes_target;
      blob_out->payload.sd_verify_readback = 0u;
      blob_out->payload.rtc_resync_period_ms = old_blob->payload.rtc_resync_period_ms;
      blob_out->payload.calibration = old_blob->payload.calibration;
      blob_out->payload.calibration_context = old_blob->payload.calibration_context;
      blob_out->payload.calibration_context_valid =
        old_blob->payload.calibration_context_valid;
      memcpy(blob_out->payload.calibration_points,
             old_blob->payload.calibration_points,
             sizeof(blob_out->payload.calibration_points));
      blob_out->payload.calibration_points_count =
        old_blob->payload.calibration_points_count;
      blob_out->payload.calibration_domain = old_blob->payload.calibration_domain;
      blob_out->payload.cal_window_duration_s = old_blob->payload.cal_window_duration_s;
      blob_out->payload.cal_trend_ema_alpha_permille =
        old_blob->payload.cal_trend_ema_alpha_permille;
      blob_out->payload.cal_last_utc = old_blob->payload.cal_last_utc;
      blob_out->payload.cal_last_override_utc = old_blob->payload.cal_last_override_utc;
      blob_out->payload.cal_due_count = old_blob->payload.cal_due_count;
      blob_out->payload.cal_due_unit = old_blob->payload.cal_due_unit;
      blob_out->payload.cal_due_override_count =
        old_blob->payload.cal_due_override_count;
      blob_out->payload.cal_due_override_unit =
        old_blob->payload.cal_due_override_unit;
      blob_out->payload.cal_metar = old_blob->payload.cal_metar;
      blob_out->payload.rtd_ema_enabled = old_blob->payload.rtd_ema_enabled;
      blob_out->payload.rtd_ema_alpha_permille =
        old_blob->payload.rtd_ema_alpha_permille;
      blob_out->payload.rtd_fault_assert_ms = old_blob->payload.rtd_fault_assert_ms;
      blob_out->payload.rtd_fault_clear_ms = old_blob->payload.rtd_fault_clear_ms;
      memcpy(blob_out->payload.tz_posix,
             old_blob->payload.tz_posix,
             sizeof(blob_out->payload.tz_posix));
      memcpy(blob_out->payload.unit_serial,
             old_blob->payload.unit_serial,
             sizeof(blob_out->payload.unit_serial));
      memcpy(blob_out->payload.cal_method,
             old_blob->payload.cal_method,
             sizeof(blob_out->payload.cal_method));
      blob_out->payload.dst_enabled = old_blob->payload.dst_enabled;
      blob_out->payload.node_role = old_blob->payload.node_role;
      blob_out->payload.allow_children = old_blob->payload.allow_children;
      blob_out->payload.allow_children_set = old_blob->payload.allow_children_set;
      blob_out->payload.display_attention_policy =
        old_blob->payload.display_attention_policy;
      blob_out->payload.net_mode = old_blob->payload.net_mode;
      blob_out->payload.mqtt_enabled = old_blob->payload.mqtt_enabled;
      memcpy(blob_out->payload.mqtt_broker_uri,
             old_blob->payload.mqtt_broker_uri,
             sizeof(blob_out->payload.mqtt_broker_uri));
      memcpy(blob_out->payload.mqtt_topic_prefix,
             old_blob->payload.mqtt_topic_prefix,
             sizeof(blob_out->payload.mqtt_topic_prefix));
      blob_out->payload.mqtt_qos = old_blob->payload.mqtt_qos;
      blob_out->payload.mqtt_retain = old_blob->payload.mqtt_retain;
      blob_out->payload.mqtt_bridge_mode = old_blob->payload.mqtt_bridge_mode;
      blob_out->header.crc32_le = ComputeSettingsBlobCrc32(blob_out);
      ESP_LOGI(kTag,
               "settings blob %s migrated v6 -> v%" PRIu32,
               key,
               (uint32_t)APP_SETTINGS_BLOB_VERSION);
      return true;
    }
  }
  if (header->version == 1u && blob_size == sizeof(app_settings_blob_v1_t)) {
    const app_settings_blob_v1_t* old_blob = (const app_settings_blob_v1_t*)raw;
    app_settings_blob_v1_t crc_blob = *old_blob;
    const uint32_t crc_saved = crc_blob.header.crc32_le;
    crc_blob.header.crc32_le = 0;
    const uint32_t crc = esp_rom_crc32_le(0, (const uint8_t*)&crc_blob, sizeof(crc_blob));
    if (crc != crc_saved) {
      return false;
    }
    memset(blob_out, 0, sizeof(*blob_out));
    blob_out->header.magic = old_blob->header.magic;
    blob_out->header.version = APP_SETTINGS_BLOB_VERSION;
    blob_out->header.size = sizeof(app_settings_persist_payload_t);
    blob_out->header.generation = old_blob->header.generation;
    const app_settings_persist_payload_v1_t* old = &old_blob->payload;
    app_settings_persist_payload_t* now = &blob_out->payload;
    now->log_period_ms = old->log_period_ms;
    now->fram_flush_watermark_records = old->fram_flush_watermark_records;
    now->sd_flush_period_ms = old->sd_flush_period_ms;
    now->sd_batch_bytes_target = old->sd_batch_bytes_target;
    now->rtc_resync_period_ms = old->rtc_resync_period_ms;
    now->calibration = old->calibration;
    now->calibration_context = old->calibration_context;
    now->calibration_context_valid = old->calibration_context_valid;
    now->calibration_points_count = old->calibration_points_count;
    now->calibration_domain = (uint8_t)CAL_DOMAIN_TEMP_C;
    now->cal_window_duration_s = CAL_WINDOW_DURATION_DEFAULT_S;
    now->cal_trend_ema_alpha_permille = CAL_TREND_EMA_ALPHA_DEFAULT_PERMILLE;
    for (size_t i = 0; i < CALIBRATION_MAX_POINTS; ++i) {
      now->calibration_points[i].raw_avg_mC = old->calibration_points[i].raw_avg_mC;
      now->calibration_points[i].actual_mC = old->calibration_points[i].actual_mC;
      now->calibration_points[i].raw_stddev_mC = old->calibration_points[i].raw_stddev_mC;
      now->calibration_points[i].raw_avg_mOhm = 0;
      now->calibration_points[i].raw_stddev_mOhm = 0;
      now->calibration_points[i].sample_count = old->calibration_points[i].sample_count;
      now->calibration_points[i].time_valid = old->calibration_points[i].time_valid;
      now->calibration_points[i].timestamp_epoch_sec = old->calibration_points[i].timestamp_epoch_sec;
      MarkCalibrationPointDriftLegacyUnavailable_(&now->calibration_points[i]);
    }
    now->cal_last_utc = old->cal_last_utc;
    now->cal_last_override_utc = old->cal_last_override_utc;
    now->cal_due_count = old->cal_due_count;
    now->cal_due_unit = old->cal_due_unit;
    now->cal_due_override_count = old->cal_due_override_count;
    now->cal_due_override_unit = old->cal_due_override_unit;
    memset(&now->cal_metar, 0, sizeof(now->cal_metar));
    now->rtd_ema_enabled = old->rtd_ema_enabled;
    now->rtd_ema_alpha_permille = old->rtd_ema_alpha_permille;
    now->rtd_fault_assert_ms = old->rtd_fault_assert_ms;
    now->rtd_fault_clear_ms = old->rtd_fault_clear_ms;
    memcpy(now->tz_posix, old->tz_posix, sizeof(now->tz_posix));
    memcpy(now->unit_serial, old->unit_serial, sizeof(now->unit_serial));
    memcpy(now->cal_method, old->cal_method, sizeof(now->cal_method));
    now->dst_enabled = old->dst_enabled;
    now->node_role = old->node_role;
    now->allow_children = old->allow_children;
    now->allow_children_set = old->allow_children_set;
    now->display_attention_policy = old->display_attention_policy;
    now->net_mode = old->net_mode;
    now->mqtt_enabled = old->mqtt_enabled;
    memcpy(now->mqtt_broker_uri, old->mqtt_broker_uri, sizeof(now->mqtt_broker_uri));
    memcpy(now->mqtt_topic_prefix, old->mqtt_topic_prefix, sizeof(now->mqtt_topic_prefix));
    now->mqtt_qos = old->mqtt_qos;
    now->mqtt_retain = old->mqtt_retain;
    now->mqtt_bridge_mode = old->mqtt_bridge_mode;
    blob_out->header.crc32_le = ComputeSettingsBlobCrc32(blob_out);
    ESP_LOGI(kTag,
             "settings blob %s migrated v1 -> v%" PRIu32,
             key,
             (uint32_t)APP_SETTINGS_BLOB_VERSION);
    return true;
  }
  return false;
}

/**
 * @brief Determine whether a generation counter is newer than a reference.
 * @param generation Parameter generation.
 * @param reference Parameter reference.
 * @return Return true when generation is newer than reference.
 */
static bool
SettingsGenerationIsNewer(uint32_t generation, uint32_t reference)
{
  return (int32_t)(generation - reference) > 0;
}

/**
 * @brief Populate a persisted payload from the current settings.
 * @param settings Parameter settings.
 * @param payload Parameter payload.
 */
static void
SettingsPayloadFromSettings(const app_settings_t* settings,
                            app_settings_persist_payload_t* payload)
{
  if (settings == NULL || payload == NULL) {
    return;
  }
  memset(payload, 0, sizeof(*payload));
  payload->log_period_ms = settings->log_period_ms;
  payload->fram_flush_watermark_records =
    settings->fram_flush_watermark_records;
  payload->sd_flush_period_ms = settings->sd_flush_period_ms;
  payload->sd_batch_bytes_target = settings->sd_batch_bytes_target;
  payload->sd_verify_readback = settings->sd_verify_readback ? 1u : 0u;
  payload->rtc_resync_period_ms = settings->rtc_resync_period_ms;
  payload->calibration = settings->calibration;
  payload->calibration_context = settings->calibration_context;
  payload->calibration_context_valid =
    settings->calibration_context_valid ? 1u : 0u;
  memcpy(payload->calibration_points,
         settings->calibration_points,
         sizeof(payload->calibration_points));
  payload->calibration_points_count = settings->calibration_points_count;
  payload->calibration_domain = (uint8_t)settings->calibration_domain;
  payload->cal_window_duration_s = settings->cal_window_duration_s;
  payload->cal_trend_ema_alpha_permille = settings->cal_trend_ema_alpha_permille;
  payload->cal_last_utc = settings->cal_last_utc;
  payload->cal_last_override_utc = settings->cal_last_override_utc;
  payload->cal_due_count = settings->cal_due_count;
  payload->cal_due_unit = settings->cal_due_unit;
  payload->cal_due_override_count = settings->cal_due_override_count;
  payload->cal_due_override_unit = settings->cal_due_override_unit;
  payload->cal_metar = settings->cal_metar;
  payload->rtd_ema_enabled = settings->rtd_ema_enabled ? 1u : 0u;
  payload->rtd_ema_alpha_permille = settings->rtd_ema_alpha_permille;
  payload->rtd_fault_assert_ms = settings->rtd_fault_assert_ms;
  payload->rtd_fault_clear_ms = settings->rtd_fault_clear_ms;
  snprintf(
    payload->tz_posix, sizeof(payload->tz_posix), "%s", settings->tz_posix);
  snprintf(payload->unit_serial,
           sizeof(payload->unit_serial),
           "%s",
           settings->unit_serial);
  snprintf(payload->cal_method,
           sizeof(payload->cal_method),
           "%s",
           settings->cal_method);
  payload->dst_enabled = settings->dst_enabled ? 1u : 0u;
  payload->node_role = (uint8_t)settings->node_role;
  payload->allow_children = settings->allow_children ? 1u : 0u;
  payload->allow_children_set = settings->allow_children_set ? 1u : 0u;
  payload->display_attention_policy = settings->display_attention_policy;
  payload->net_mode = (uint8_t)settings->net_mode;
  payload->mqtt_enabled = settings->mqtt_enabled ? 1u : 0u;
  snprintf(payload->mqtt_broker_uri,
           sizeof(payload->mqtt_broker_uri),
           "%s",
           settings->mqtt_broker_uri);
  snprintf(payload->mqtt_topic_prefix,
           sizeof(payload->mqtt_topic_prefix),
           "%s",
           settings->mqtt_topic_prefix);
  payload->mqtt_qos = settings->mqtt_qos;
  payload->mqtt_retain = settings->mqtt_retain ? 1u : 0u;
  payload->mqtt_bridge_mode = (uint8_t)settings->mqtt_bridge_mode;
}

/**
 * @brief Overlay persisted values onto defaults with range validation.
 * @param payload Parameter payload.
 * @param settings_out Parameter settings_out.
 */
static void
ApplyPersistedSettings(const app_settings_persist_payload_t* payload,
                       app_settings_t* settings_out)
{
  if (payload == NULL || settings_out == NULL) {
    return;
  }
  if (payload->log_period_ms >= 100 && payload->log_period_ms <= 3600000) {
    settings_out->log_period_ms = payload->log_period_ms;
  }
  if (payload->fram_flush_watermark_records >= 1) {
    settings_out->fram_flush_watermark_records =
      payload->fram_flush_watermark_records;
  }
  if (payload->sd_flush_period_ms >= 1000) {
    settings_out->sd_flush_period_ms = payload->sd_flush_period_ms;
  }
  if (payload->sd_batch_bytes_target >= 4096) {
    settings_out->sd_batch_bytes_target = payload->sd_batch_bytes_target;
  }
  if (payload->sd_verify_readback <= 1u) {
    settings_out->sd_verify_readback = (payload->sd_verify_readback == 1u);
  }
  if (payload->rtc_resync_period_ms <= 86400000u) {
    settings_out->rtc_resync_period_ms = payload->rtc_resync_period_ms;
  }

  if (payload->calibration.degree <= CALIBRATION_MAX_DEGREE) {
    settings_out->calibration = payload->calibration;
  } else {
    CalibrationModelInitIdentity(&settings_out->calibration);
  }
  if (settings_out->calibration.is_valid &&
      payload->calibration.mode <= CAL_FIT_MODE_POLY) {
    settings_out->calibration.mode =
      (calibration_fit_mode_t)payload->calibration.mode;
  } else if (settings_out->calibration.is_valid) {
    settings_out->calibration.mode = (settings_out->calibration.degree > 1)
                                       ? CAL_FIT_MODE_POLY
                                       : CAL_FIT_MODE_LINEAR;
  }

  if (payload->calibration_points_count <= CALIBRATION_MAX_POINTS) {
    settings_out->calibration_points_count = payload->calibration_points_count;
    memcpy(settings_out->calibration_points,
           payload->calibration_points,
           sizeof(settings_out->calibration_points));
    for (size_t i = 0; i < CALIBRATION_MAX_POINTS; ++i) {
      if (settings_out->calibration_points[i].drift_limit_source >
          (uint8_t)CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE) {
        MarkCalibrationPointDriftLegacyUnavailable_(
          &settings_out->calibration_points[i]);
      }
    }
  } else {
    settings_out->calibration_points_count = 0;
    memset(settings_out->calibration_points,
           0,
           sizeof(settings_out->calibration_points));
  }

  if (payload->calibration_domain <= (uint8_t)CAL_DOMAIN_RESISTANCE_OHM) {
    settings_out->calibration_domain =
      (calibration_domain_t)payload->calibration_domain;
  } else {
    settings_out->calibration_domain = CAL_DOMAIN_TEMP_C;
  }
  if (payload->cal_window_duration_s >= CAL_WINDOW_DURATION_MIN_S &&
      payload->cal_window_duration_s <= CAL_WINDOW_DURATION_MAX_S) {
    settings_out->cal_window_duration_s = payload->cal_window_duration_s;
  }
  if (payload->cal_trend_ema_alpha_permille >= 1u &&
      payload->cal_trend_ema_alpha_permille <= 1000u) {
    settings_out->cal_trend_ema_alpha_permille =
      payload->cal_trend_ema_alpha_permille;
  }

  if (payload->calibration_context_valid <= 1) {
    settings_out->calibration_context_valid =
      (payload->calibration_context_valid == 1);
  }
  if (settings_out->calibration_context_valid) {
    settings_out->calibration_context = payload->calibration_context;
  } else {
    memset(&settings_out->calibration_context,
           0,
           sizeof(settings_out->calibration_context));
  }

  if (payload->cal_last_utc >= 0) {
    settings_out->cal_last_utc = payload->cal_last_utc;
  }
  if (payload->cal_last_override_utc >= 0) {
    settings_out->cal_last_override_utc = payload->cal_last_override_utc;
  }

  settings_out->cal_due_count = payload->cal_due_count;
  settings_out->cal_due_unit = payload->cal_due_unit;
  ValidateCalibrationDueSettings(&settings_out->cal_due_count,
                                 &settings_out->cal_due_unit);

  settings_out->cal_due_override_count = payload->cal_due_override_count;
  settings_out->cal_due_override_unit = payload->cal_due_override_unit;
  ValidateCalibrationDueSettings(&settings_out->cal_due_override_count,
                                 &settings_out->cal_due_override_unit);
  settings_out->cal_metar = payload->cal_metar;
  if (settings_out->cal_metar.valid > 1u) {
    memset(&settings_out->cal_metar, 0, sizeof(settings_out->cal_metar));
  } else {
    settings_out->cal_metar.source_type[sizeof(settings_out->cal_metar.source_type) - 1] =
      '\0';
    settings_out->cal_metar
      .raw_metar[sizeof(settings_out->cal_metar.raw_metar) - 1] = '\0';
    settings_out->cal_metar
      .station_id[sizeof(settings_out->cal_metar.station_id) - 1] = '\0';
    settings_out->cal_metar.observation_token
      [sizeof(settings_out->cal_metar.observation_token) - 1] = '\0';
    settings_out->cal_metar.observation_iso_utc
      [sizeof(settings_out->cal_metar.observation_iso_utc) - 1] = '\0';
    settings_out->cal_metar.auto_or_cor
      [sizeof(settings_out->cal_metar.auto_or_cor) - 1] = '\0';
    settings_out->cal_metar.temp_dew_token
      [sizeof(settings_out->cal_metar.temp_dew_token) - 1] = '\0';
    settings_out->cal_metar
      .remarks[sizeof(settings_out->cal_metar.remarks) - 1] = '\0';
    settings_out->cal_metar
      .method_note[sizeof(settings_out->cal_metar.method_note) - 1] = '\0';
    if (settings_out->cal_metar.observation_resolved > 1u) {
      settings_out->cal_metar.observation_resolved = 0u;
    }
  }

  if (payload->rtd_ema_enabled <= 1) {
    settings_out->rtd_ema_enabled = (payload->rtd_ema_enabled == 1);
  }
  if (payload->rtd_ema_alpha_permille >= 1 &&
      payload->rtd_ema_alpha_permille <= 1000) {
    settings_out->rtd_ema_alpha_permille = payload->rtd_ema_alpha_permille;
  }
  if (payload->rtd_fault_assert_ms <= kRtdFaultDebounceMaxMs) {
    settings_out->rtd_fault_assert_ms = payload->rtd_fault_assert_ms;
  }
  if (payload->rtd_fault_clear_ms <= kRtdFaultDebounceMaxMs) {
    settings_out->rtd_fault_clear_ms = payload->rtd_fault_clear_ms;
  }

  const size_t tz_len = strnlen(payload->tz_posix, sizeof(payload->tz_posix));
  if (tz_len > 0 && tz_len < sizeof(payload->tz_posix)) {
    memcpy(settings_out->tz_posix,
           payload->tz_posix,
           sizeof(settings_out->tz_posix));
    settings_out->tz_posix[sizeof(settings_out->tz_posix) - 1] = '\0';
  }

  const size_t serial_len =
    strnlen(payload->unit_serial, sizeof(payload->unit_serial));
  if (serial_len < sizeof(payload->unit_serial) &&
      IsPrintableSettingString(payload->unit_serial,
                               sizeof(payload->unit_serial))) {
    memcpy(settings_out->unit_serial,
           payload->unit_serial,
           sizeof(settings_out->unit_serial));
    settings_out->unit_serial[sizeof(settings_out->unit_serial) - 1] = '\0';
  }

  const size_t method_len =
    strnlen(payload->cal_method, sizeof(payload->cal_method));
  if (method_len < sizeof(payload->cal_method) &&
      IsPrintableSettingString(payload->cal_method,
                               sizeof(payload->cal_method))) {
    memcpy(settings_out->cal_method,
           payload->cal_method,
           sizeof(settings_out->cal_method));
    settings_out->cal_method[sizeof(settings_out->cal_method) - 1] = '\0';
  }

  if (payload->dst_enabled <= 1) {
    settings_out->dst_enabled = (payload->dst_enabled == 1);
  }

  if (payload->node_role <= (uint8_t)APP_NODE_ROLE_RELAY) {
    settings_out->node_role = (app_node_role_t)payload->node_role;
  }

  const bool allow_children_set = (payload->allow_children_set == 1);
  settings_out->allow_children_set = allow_children_set;
  if (allow_children_set) {
    if (payload->allow_children <= 1) {
      settings_out->allow_children = (payload->allow_children == 1);
    }
  } else {
    settings_out->allow_children =
      AppSettingsRoleDefaultAllowsChildren(settings_out->node_role);
  }

  settings_out->display_attention_policy = payload->display_attention_policy;
  settings_out->display_attention_mask =
    DisplayAttentionMaskFromPolicy(settings_out->display_attention_policy);

  if (payload->net_mode <= (uint8_t)APP_NET_MODE_NONE) {
    settings_out->net_mode = (app_net_mode_t)payload->net_mode;
  }

  if (payload->mqtt_enabled <= 1) {
    settings_out->mqtt_enabled = (payload->mqtt_enabled == 1);
  }

  const size_t broker_len =
    strnlen(payload->mqtt_broker_uri, sizeof(payload->mqtt_broker_uri));
  if (broker_len > 0 && broker_len < sizeof(payload->mqtt_broker_uri)) {
    memcpy(settings_out->mqtt_broker_uri,
           payload->mqtt_broker_uri,
           sizeof(settings_out->mqtt_broker_uri));
    settings_out->mqtt_broker_uri[sizeof(settings_out->mqtt_broker_uri) - 1] =
      '\0';
  }

  const size_t prefix_len =
    strnlen(payload->mqtt_topic_prefix, sizeof(payload->mqtt_topic_prefix));
  if (prefix_len > 0 && prefix_len < sizeof(payload->mqtt_topic_prefix)) {
    memcpy(settings_out->mqtt_topic_prefix,
           payload->mqtt_topic_prefix,
           sizeof(settings_out->mqtt_topic_prefix));
    settings_out
      ->mqtt_topic_prefix[sizeof(settings_out->mqtt_topic_prefix) - 1] = '\0';
  }

  if (payload->mqtt_qos <= 1) {
    settings_out->mqtt_qos = payload->mqtt_qos;
  }
  if (payload->mqtt_retain <= 1) {
    settings_out->mqtt_retain = (payload->mqtt_retain == 1);
  }
  if (payload->mqtt_bridge_mode <= (uint8_t)MQTT_BRIDGE_BOTH) {
    settings_out->mqtt_bridge_mode =
      (mqtt_bridge_mode_t)payload->mqtt_bridge_mode;
  }
}

/**
 * @brief Initialize a settings snapshot using the last saved settings or
 * defaults.
 * @param settings_out Parameter settings_out.
 */
static void
InitSettingsSnapshot(app_settings_t* settings_out)
{
  if (settings_out == NULL) {
    return;
  }
  if (g_saved_settings_valid) {
    *settings_out = g_saved_settings;
  } else {
    ApplyDefaults(settings_out);
  }
}

/**
 * @brief Persist a settings snapshot and update the cached copy on success.
 * @param settings Parameter settings.
 * @return Return the function result.
 */
static esp_err_t
PersistSettingsSnapshot(app_settings_t* settings)
{
  if (settings == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  const esp_err_t result = AppSettingsSaveBlob(settings);
  if (result == ESP_OK) {
    g_saved_settings = *settings;
    g_saved_settings_valid = true;
  }
  return result;
}

typedef struct
{
  const calibration_model_t* model;
  const calibration_context_t* context;
} save_calibration_with_context_ctx_t;

typedef struct
{
  const app_settings_t* source;
} save_calibration_schedule_ctx_t;

typedef struct
{
  const calibration_point_t* points;
  size_t points_count;
} save_calibration_points_ctx_t;

typedef struct
{
  uint16_t window_s;
  uint16_t ema_alpha_permille;
} save_calibration_config_ctx_t;

typedef struct
{
  uint32_t assert_ms;
  uint32_t clear_ms;
} save_rtd_fault_debounce_ctx_t;

typedef struct
{
  const char* tz_posix;
  bool dst_enabled;
} save_time_zone_ctx_t;

typedef struct
{
  bool allow_children;
  bool explicit_setting;
} save_allow_children_ctx_t;

static void
EnsureSettingsSaveMutexInitialized_(void)
{
  if (s_settings_save_mutex != NULL) {
    return;
  }
  taskENTER_CRITICAL(&s_settings_save_mutex_init_lock);
  if (s_settings_save_mutex == NULL) {
    s_settings_save_mutex = xSemaphoreCreateMutexStatic(&s_settings_save_mutex_buf);
  }
  taskEXIT_CRITICAL(&s_settings_save_mutex_init_lock);
}

static esp_err_t
PersistMutatedSettingsSnapshot_(app_settings_mutator_fn mutator, void* context)
{
  if (mutator == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  EnsureSettingsSaveMutexInitialized_();
  if (s_settings_save_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }
  if (xSemaphoreTake(s_settings_save_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  InitSettingsSnapshot(&s_settings_save_scratch);
  mutator(&s_settings_save_scratch, context);
  const esp_err_t result = PersistSettingsSnapshot(&s_settings_save_scratch);

  xSemaphoreGive(s_settings_save_mutex);
  return result;
}

/**
 * @brief Save the settings blob with redundant A/B storage.
 * @param settings Parameter settings.
 * @return Return the function result.
 */
static esp_err_t
AppSettingsSaveBlob(const app_settings_t* settings)
{
  if (settings == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  app_settings_blob_t blob;
  memset(&blob, 0, sizeof(blob));
  blob.header.magic = APP_SETTINGS_BLOB_MAGIC;
  blob.header.version = APP_SETTINGS_BLOB_VERSION;
  blob.header.size = sizeof(app_settings_persist_payload_t);
  blob.header.generation = g_settings_blob_generation + 1u;
  SettingsPayloadFromSettings(settings, &blob.payload);
  blob.header.crc32_le = ComputeSettingsBlobCrc32(&blob);

  const char* key = ((blob.header.generation % 2u) == 0u) ? kKeySettingsBlob0
                                                          : kKeySettingsBlob1;

  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_blob(handle, key, &blob, sizeof(blob));
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result == ESP_OK) {
    g_settings_blob_generation = blob.header.generation;
  }
  return result;
}

static void
MutateLogPeriodMs_(app_settings_t* settings, void* context)
{
  settings->log_period_ms = *(uint32_t*)context;
}

static void
MutateFramFlushWatermarkRecords_(app_settings_t* settings, void* context)
{
  settings->fram_flush_watermark_records = *(uint32_t*)context;
}

static void
MutateSdFlushPeriodMs_(app_settings_t* settings, void* context)
{
  settings->sd_flush_period_ms = *(uint32_t*)context;
}

static void
MutateSdBatchBytes_(app_settings_t* settings, void* context)
{
  settings->sd_batch_bytes_target = *(uint32_t*)context;
}

static void
MutateSdVerifyReadback_(app_settings_t* settings, void* context)
{
  settings->sd_verify_readback = *(bool*)context;
}

static void
MutateRtcResyncPeriodMs_(app_settings_t* settings, void* context)
{
  settings->rtc_resync_period_ms = *(uint32_t*)context;
}

static void
MutateCalibrationWithContext_(app_settings_t* settings, void* context)
{
  const save_calibration_with_context_ctx_t* ctx = context;
  settings->calibration = *ctx->model;
  settings->calibration_context = *ctx->context;
  settings->calibration_context_valid = true;
}

static void
MutateCalibrationSchedule_(app_settings_t* settings, void* context)
{
  const save_calibration_schedule_ctx_t* ctx = context;
  *settings = *ctx->source;
  ValidateCalibrationDueSettings(&settings->cal_due_count, &settings->cal_due_unit);
  ValidateCalibrationDueSettings(&settings->cal_due_override_count,
                                 &settings->cal_due_override_unit);
}

static void
MutateCalibrationPoints_(app_settings_t* settings, void* context)
{
  const save_calibration_points_ctx_t* ctx = context;
  memset(settings->calibration_points, 0, sizeof(settings->calibration_points));
  if (ctx->points_count > 0u) {
    memcpy(settings->calibration_points,
           ctx->points,
           sizeof(calibration_point_t) * ctx->points_count);
  }
  settings->calibration_points_count = (uint8_t)ctx->points_count;
}

static void
MutateCalibrationDomain_(app_settings_t* settings, void* context)
{
  settings->calibration_domain = *(calibration_domain_t*)context;
}

static void
MutateCalibrationConfig_(app_settings_t* settings, void* context)
{
  const save_calibration_config_ctx_t* ctx = context;
  settings->cal_window_duration_s = ctx->window_s;
  settings->cal_trend_ema_alpha_permille = ctx->ema_alpha_permille;
}

static void
MutateRtdEmaEnabled_(app_settings_t* settings, void* context)
{
  settings->rtd_ema_enabled = *(bool*)context;
}

static void
MutateRtdEmaAlphaPermille_(app_settings_t* settings, void* context)
{
  settings->rtd_ema_alpha_permille = *(uint16_t*)context;
}

static void
MutateRtdFaultDebounceMs_(app_settings_t* settings, void* context)
{
  const save_rtd_fault_debounce_ctx_t* ctx = context;
  settings->rtd_fault_assert_ms = ctx->assert_ms;
  settings->rtd_fault_clear_ms = ctx->clear_ms;
}

static void
MutateTimeZone_(app_settings_t* settings, void* context)
{
  const save_time_zone_ctx_t* ctx = context;
  snprintf(settings->tz_posix, sizeof(settings->tz_posix), "%s", ctx->tz_posix);
  settings->dst_enabled = ctx->dst_enabled;
}

static void
MutateCalibrationMethod_(app_settings_t* settings, void* context)
{
  const char* method = context;
  snprintf(settings->cal_method, sizeof(settings->cal_method), "%s", method);
}

static void
MutateCalibrationMetar_(app_settings_t* settings, void* context)
{
  const calibration_metar_reference_t* metar = context;
  settings->cal_metar = *metar;
  settings->cal_metar.source_type[sizeof(settings->cal_metar.source_type) - 1] =
    '\0';
  settings->cal_metar.raw_metar[sizeof(settings->cal_metar.raw_metar) - 1] = '\0';
  settings->cal_metar.station_id[sizeof(settings->cal_metar.station_id) - 1] =
    '\0';
  settings->cal_metar
    .observation_token[sizeof(settings->cal_metar.observation_token) - 1] = '\0';
  settings->cal_metar.observation_iso_utc
    [sizeof(settings->cal_metar.observation_iso_utc) - 1] = '\0';
  settings->cal_metar.auto_or_cor[sizeof(settings->cal_metar.auto_or_cor) - 1] =
    '\0';
  settings->cal_metar
    .temp_dew_token[sizeof(settings->cal_metar.temp_dew_token) - 1] = '\0';
  settings->cal_metar.remarks[sizeof(settings->cal_metar.remarks) - 1] = '\0';
  settings->cal_metar
    .method_note[sizeof(settings->cal_metar.method_note) - 1] = '\0';
  settings->cal_metar.valid = (settings->cal_metar.valid != 0u) ? 1u : 0u;
  settings->cal_metar.observation_resolved =
    (settings->cal_metar.observation_resolved != 0u) ? 1u : 0u;
}

static void
MutateNodeRole_(app_settings_t* settings, void* context)
{
  settings->node_role = *(app_node_role_t*)context;
}

static void
MutateAllowChildren_(app_settings_t* settings, void* context)
{
  const save_allow_children_ctx_t* ctx = context;
  settings->allow_children = ctx->allow_children;
  settings->allow_children_set = ctx->explicit_setting;
}

static void
MutateNetMode_(app_settings_t* settings, void* context)
{
  settings->net_mode = *(app_net_mode_t*)context;
}

static void
MutateMqttEnabled_(app_settings_t* settings, void* context)
{
  settings->mqtt_enabled = *(bool*)context;
}

static void
MutateMqttBrokerUri_(app_settings_t* settings, void* context)
{
  const char* uri = context;
  snprintf(settings->mqtt_broker_uri, sizeof(settings->mqtt_broker_uri), "%s", uri);
}

static void
MutateMqttTopicPrefix_(app_settings_t* settings, void* context)
{
  const char* prefix = context;
  snprintf(settings->mqtt_topic_prefix,
           sizeof(settings->mqtt_topic_prefix),
           "%s",
           prefix);
}

static void
MutateMqttQos_(app_settings_t* settings, void* context)
{
  settings->mqtt_qos = *(uint8_t*)context;
}

static void
MutateMqttRetain_(app_settings_t* settings, void* context)
{
  settings->mqtt_retain = *(bool*)context;
}

static void
MutateMqttBridgeMode_(app_settings_t* settings, void* context)
{
  settings->mqtt_bridge_mode = *(mqtt_bridge_mode_t*)context;
}

static void
MutateDisplayAttentionPolicy_(app_settings_t* settings, void* context)
{
  const uint32_t policy = *(uint32_t*)context;
  settings->display_attention_policy = policy;
  settings->display_attention_mask = DisplayAttentionMaskFromPolicy(policy);
}

/**
 * @brief Load settings from legacy per-key storage.
 * @param handle Parameter handle.
 * @param settings_out Parameter settings_out.
 * @param migrated_out Parameter migrated_out.
 * @return Return the function result.
 */
static esp_err_t
AppSettingsLoadLegacy(nvs_handle_t handle,
                      app_settings_t* settings_out,
                      bool* migrated_out)
{
  if (settings_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (migrated_out != NULL) {
    *migrated_out = false;
  }

  uint32_t log_period_ms = 0;
  esp_err_t result = nvs_get_u32(handle, kKeyLogPeriodMs, &log_period_ms);
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

  uint32_t rtc_resync_ms = settings_out->rtc_resync_period_ms;
  result = nvs_get_u32(handle, kKeyRtcResyncPeriodMs, &rtc_resync_ms);
  if (result == ESP_OK && rtc_resync_ms <= 86400000u) {
    settings_out->rtc_resync_period_ms = rtc_resync_ms;
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
      esp_err_t points_result = nvs_get_blob(handle,
                                             kKeyCalPoints,
                                             settings_out->calibration_points,
                                             &points_bytes_copy);
      if (points_result == ESP_OK && points_bytes_copy == points_bytes) {
        settings_out->calibration_points_count = cal_points_count;
      } else {
        legacy_calibration_point_v2_t legacy_points_v2[CALIBRATION_MAX_POINTS] =
          { 0 };
        size_t legacy_v2_bytes =
          sizeof(legacy_calibration_point_v2_t) * (size_t)cal_points_count;
        points_result = nvs_get_blob(
          handle, kKeyCalPoints, legacy_points_v2, &legacy_v2_bytes);
        if (points_result == ESP_OK &&
            legacy_v2_bytes ==
              sizeof(legacy_calibration_point_v2_t) *
                (size_t)cal_points_count) {
          settings_out->calibration_points_count = cal_points_count;
          for (size_t i = 0; i < cal_points_count; ++i) {
            settings_out->calibration_points[i].raw_avg_mC =
              legacy_points_v2[i].raw_avg_mC;
            settings_out->calibration_points[i].actual_mC =
              legacy_points_v2[i].actual_mC;
            settings_out->calibration_points[i].raw_stddev_mC =
              legacy_points_v2[i].raw_stddev_mC;
            settings_out->calibration_points[i].raw_avg_mOhm =
              legacy_points_v2[i].raw_avg_mOhm;
            settings_out->calibration_points[i].raw_stddev_mOhm =
              legacy_points_v2[i].raw_stddev_mOhm;
            settings_out->calibration_points[i].sample_count =
              legacy_points_v2[i].sample_count;
            settings_out->calibration_points[i].time_valid =
              legacy_points_v2[i].time_valid;
            settings_out->calibration_points[i].timestamp_epoch_sec =
              legacy_points_v2[i].timestamp_epoch_sec;
            MarkCalibrationPointDriftLegacyUnavailable_(
              &settings_out->calibration_points[i]);
          }
          goto load_points_done;
        }

        legacy_calibration_point_v1_t legacy_points[CALIBRATION_MAX_POINTS] =
          { 0 };
        size_t legacy_bytes =
          sizeof(legacy_calibration_point_v1_t) * (size_t)cal_points_count;
        points_result = nvs_get_blob(
          handle, kKeyCalPoints, legacy_points, &legacy_bytes);
        if (points_result == ESP_OK &&
            legacy_bytes ==
              sizeof(legacy_calibration_point_v1_t) *
                (size_t)cal_points_count) {
          settings_out->calibration_points_count = cal_points_count;
          for (size_t i = 0; i < cal_points_count; ++i) {
            settings_out->calibration_points[i].raw_avg_mC =
              legacy_points[i].raw_avg_mC;
            settings_out->calibration_points[i].actual_mC =
              legacy_points[i].actual_mC;
            settings_out->calibration_points[i].raw_stddev_mC =
              legacy_points[i].raw_stddev_mC;
            settings_out->calibration_points[i].raw_avg_mOhm = 0;
            settings_out->calibration_points[i].raw_stddev_mOhm = 0;
            settings_out->calibration_points[i].sample_count =
              legacy_points[i].sample_count;
            settings_out->calibration_points[i].time_valid =
              legacy_points[i].time_valid;
            settings_out->calibration_points[i].timestamp_epoch_sec =
              legacy_points[i].timestamp_epoch_sec;
            MarkCalibrationPointDriftLegacyUnavailable_(
              &settings_out->calibration_points[i]);
          }
        } else {
          settings_out->calibration_points_count = 0;
          memset(settings_out->calibration_points,
                 0,
                 sizeof(settings_out->calibration_points));
          for (size_t i = 0; i < CALIBRATION_MAX_POINTS; ++i) {
            MarkCalibrationPointDriftLegacyUnavailable_(
              &settings_out->calibration_points[i]);
          }
        }
      }
    } else {
      settings_out->calibration_points_count = 0;
    }
load_points_done:
  } else {
    settings_out->calibration_points_count = 0;
  }

  settings_out->calibration_domain = CAL_DOMAIN_TEMP_C;

  calibration_context_t loaded_context;
  if (LoadCalibrationContext(handle, &loaded_context)) {
    settings_out->calibration_context = loaded_context;
    settings_out->calibration_context_valid = true;
  } else {
    memset(&settings_out->calibration_context,
           0,
           sizeof(settings_out->calibration_context));
    settings_out->calibration_context_valid = false;
  }

  int64_t cal_last_utc = settings_out->cal_last_utc;
  result = nvs_get_i64(handle, kKeyCalLastUtc, &cal_last_utc);
  if (result == ESP_OK && cal_last_utc >= 0) {
    settings_out->cal_last_utc = cal_last_utc;
  }

  int64_t cal_last_override_utc = settings_out->cal_last_override_utc;
  result = nvs_get_i64(handle, kKeyCalLastOverrideUtc, &cal_last_override_utc);
  if (result == ESP_OK && cal_last_override_utc >= 0) {
    settings_out->cal_last_override_utc = cal_last_override_utc;
  }

  uint16_t cal_due_count = settings_out->cal_due_count;
  result = nvs_get_u16(handle, kKeyCalDueCount, &cal_due_count);
  if (result == ESP_OK) {
    settings_out->cal_due_count = cal_due_count;
  }

  uint8_t cal_due_unit = settings_out->cal_due_unit;
  result = nvs_get_u8(handle, kKeyCalDueUnit, &cal_due_unit);
  if (result == ESP_OK) {
    settings_out->cal_due_unit = cal_due_unit;
  }
  ValidateCalibrationDueSettings(&settings_out->cal_due_count,
                                 &settings_out->cal_due_unit);

  uint16_t cal_due_override_count = settings_out->cal_due_override_count;
  result =
    nvs_get_u16(handle, kKeyCalDueOverrideCount, &cal_due_override_count);
  if (result == ESP_OK) {
    settings_out->cal_due_override_count = cal_due_override_count;
  }

  uint8_t cal_due_override_unit = settings_out->cal_due_override_unit;
  result = nvs_get_u8(handle, kKeyCalDueOverrideUnit, &cal_due_override_unit);
  if (result == ESP_ERR_NVS_NOT_FOUND) {
    result =
      nvs_get_u8(handle, kKeyCalDueOverrideUnitLegacy, &cal_due_override_unit);
    if (result == ESP_ERR_NVS_KEY_TOO_LONG) {
      result = ESP_ERR_NVS_NOT_FOUND;
    }
  }
  if (result == ESP_OK) {
    settings_out->cal_due_override_unit = cal_due_override_unit;
  }
  ValidateCalibrationDueSettings(&settings_out->cal_due_override_count,
                                 &settings_out->cal_due_override_unit);

  uint8_t rtd_ema_enabled = settings_out->rtd_ema_enabled ? 1 : 0;
  result = nvs_get_u8(handle, kKeyRtdEmaEnabled, &rtd_ema_enabled);
  if (result == ESP_OK && rtd_ema_enabled <= 1) {
    settings_out->rtd_ema_enabled = (rtd_ema_enabled == 1);
  }

  uint32_t rtd_ema_alpha = settings_out->rtd_ema_alpha_permille;
  result = nvs_get_u32(handle, kKeyRtdEmaAlphaPermille, &rtd_ema_alpha);
  if (result == ESP_OK && rtd_ema_alpha >= 1 && rtd_ema_alpha <= 1000) {
    settings_out->rtd_ema_alpha_permille = (uint16_t)rtd_ema_alpha;
  }

  uint32_t rtd_fault_assert_ms = settings_out->rtd_fault_assert_ms;
  result = nvs_get_u32(handle, kKeyRtdFaultAssertMs, &rtd_fault_assert_ms);
  if (result == ESP_OK && rtd_fault_assert_ms <= kRtdFaultDebounceMaxMs) {
    settings_out->rtd_fault_assert_ms = rtd_fault_assert_ms;
  }

  uint32_t rtd_fault_clear_ms = settings_out->rtd_fault_clear_ms;
  result = nvs_get_u32(handle, kKeyRtdFaultClearMs, &rtd_fault_clear_ms);
  if (result == ESP_OK && rtd_fault_clear_ms <= kRtdFaultDebounceMaxMs) {
    settings_out->rtd_fault_clear_ms = rtd_fault_clear_ms;
  }

  size_t tz_len = sizeof(settings_out->tz_posix);
  result = nvs_get_str(handle, kKeyTzPosix, settings_out->tz_posix, &tz_len);
  if (result != ESP_OK || tz_len == 0 ||
      tz_len > sizeof(settings_out->tz_posix)) {
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
  const bool allow_children_set_present = (allow_children_set_result == ESP_OK);
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

  uint8_t net_mode = (uint8_t)settings_out->net_mode;
  result = nvs_get_u8(handle, kKeyNetMode, &net_mode);
  if (result == ESP_OK && net_mode <= (uint8_t)APP_NET_MODE_NONE) {
    settings_out->net_mode = (app_net_mode_t)net_mode;
  }

  uint8_t mqtt_enabled = settings_out->mqtt_enabled ? 1 : 0;
  result = nvs_get_u8(handle, kKeyMqttEnabled, &mqtt_enabled);
  if (result == ESP_OK && mqtt_enabled <= 1) {
    settings_out->mqtt_enabled = (mqtt_enabled == 1);
  }

  size_t broker_len = sizeof(settings_out->mqtt_broker_uri);
  result = nvs_get_str(
    handle, kKeyMqttBrokerUri, settings_out->mqtt_broker_uri, &broker_len);
  if (result != ESP_OK || broker_len == 0 ||
      broker_len > sizeof(settings_out->mqtt_broker_uri)) {
    snprintf(settings_out->mqtt_broker_uri,
             sizeof(settings_out->mqtt_broker_uri),
             "%s",
             APP_SETTINGS_MQTT_BROKER_URI_DEFAULT);
  }

  size_t prefix_len = sizeof(settings_out->mqtt_topic_prefix);
  result = nvs_get_str(
    handle, kKeyMqttTopicPrefix, settings_out->mqtt_topic_prefix, &prefix_len);
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
    settings_out->display_attention_policy = DisplayAttentionPolicyFromMask(
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
      if (migrated_out != NULL) {
        *migrated_out = true;
      }
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

  return ESP_OK;
}

/**
 * @brief Load display/sample period override from standalone NVS key.
 * @param handle Open NVS handle.
 * @param settings_out Settings snapshot to update when a valid key is found.
 */
static void
AppSettingsMaybeLoadDisplaySamplePeriodMs(nvs_handle_t handle,
                                          app_settings_t* settings_out)
{
  if (settings_out == NULL) {
    return;
  }

  uint32_t value = 0;
  const esp_err_t result =
    nvs_get_u32(handle, kKeyDisplaySamplePeriodMs, &value);
  if (result == ESP_OK && value >= 100u && value <= 3600000u) {
    settings_out->display_sample_period_ms = value;
  }
}

/**
 * @brief Log a summary of the loaded settings.
 * @param settings Parameter settings.
 */
static void
LogSettingsLoaded(const app_settings_t* settings)
{
  if (settings == NULL) {
    return;
  }
  ESP_LOGI(
    kTag,
    "Loaded: period=%ums disp_samp_ms=%u wm=%u sd_flush_ms=%u sd_batch=%u rtc_resync_ms=%u "
    "deg=%u cal_points=%u tz=%s dst=%u role=%s allow_children=%u "
    "display_units=%s net_mode=%s mqtt_en=%u mqtt_uri=%s mqtt_pfx=%s "
    "mqtt_qos=%u mqtt_ret=%u mqtt_bridge=%s rtd_f_as_ms=%u rtd_f_cl_ms=%u "
    "disp_attn_pol=0x%08" PRIX32 " disp_attn_mask=0x%08" PRIX32,
    (unsigned)settings->log_period_ms,
    (unsigned)settings->display_sample_period_ms,
    (unsigned)settings->fram_flush_watermark_records,
    (unsigned)settings->sd_flush_period_ms,
    (unsigned)settings->sd_batch_bytes_target,
    (unsigned)settings->rtc_resync_period_ms,
    (unsigned)settings->calibration.degree,
    (unsigned)settings->calibration_points_count,
    settings->tz_posix,
    settings->dst_enabled ? 1u : 0u,
    AppSettingsRoleToString(settings->node_role),
    settings->allow_children ? 1u : 0u,
    AppSettingsDisplayUnitsToString(settings->display_units),
    AppSettingsNetModeToString(settings->net_mode),
    settings->mqtt_enabled ? 1u : 0u,
    settings->mqtt_broker_uri,
    settings->mqtt_topic_prefix,
    (unsigned)settings->mqtt_qos,
    settings->mqtt_retain ? 1u : 0u,
    AppSettingsMqttBridgeModeToString(settings->mqtt_bridge_mode),
    (unsigned)settings->rtd_fault_assert_ms,
    (unsigned)settings->rtd_fault_clear_ms,
    (uint32_t)settings->display_attention_policy,
    (uint32_t)settings->display_attention_mask);
}

/**
 * @brief Execute OpenNvs.
 * @param handle_out Parameter handle_out.
 * @return Return the function result.
 */

/**
 * @brief Validate a user-provided metadata string for safe persistence.
 * @param value Candidate string.
 * @param max_len Maximum buffer length including NUL.
 * @return True when string is non-NULL, fits, and contains printable ASCII.
 */
static bool
IsPrintableSettingString(const char* value, size_t max_len)
{
  if (value == NULL) {
    return false;
  }
  const size_t len = strnlen(value, max_len);
  if (len >= max_len) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    if (!isprint((unsigned char)value[i])) {
      return false;
    }
  }
  return true;
}

static bool
IsPlausibleTzPosixString_(const char* value, size_t max_len)
{
  if (!IsPrintableSettingString(value, max_len)) {
    return false;
  }
  const size_t len = strnlen(value, max_len);
  if (len < 3u) {
    return false;
  }
  bool has_digit = false;
  for (size_t i = 0; i < len; ++i) {
    if (isdigit((unsigned char)value[i])) {
      has_digit = true;
      break;
    }
  }
  return has_digit;
}

static bool
IsPlausibleMqttUri_(const char* value, size_t max_len)
{
  if (!IsPrintableSettingString(value, max_len)) {
    return false;
  }
  return (strncmp(value, "mqtt://", 7u) == 0) ||
         (strncmp(value, "mqtts://", 8u) == 0);
}

static bool
ValidateAndSanitizePersistedPayload_(app_settings_persist_payload_t* payload)
{
  if (payload == NULL) {
    return false;
  }
  bool valid = true;
  bool corruption_detected = false;

  payload->tz_posix[sizeof(payload->tz_posix) - 1u] = '\0';
  payload->unit_serial[sizeof(payload->unit_serial) - 1u] = '\0';
  payload->cal_method[sizeof(payload->cal_method) - 1u] = '\0';
  payload->mqtt_broker_uri[sizeof(payload->mqtt_broker_uri) - 1u] = '\0';
  payload->mqtt_topic_prefix[sizeof(payload->mqtt_topic_prefix) - 1u] = '\0';

  if (!IsPlausibleTzPosixString_(payload->tz_posix, sizeof(payload->tz_posix))) {
    snprintf(payload->tz_posix,
             sizeof(payload->tz_posix),
             "%s",
             APP_SETTINGS_TZ_DEFAULT_POSIX);
    valid = false;
    corruption_detected = true;
    ESP_LOGW(kTag, "settings: invalid timezone in blob; using default");
  }
  if (payload->rtc_resync_period_ms > 86400000u) {
    payload->rtc_resync_period_ms = 3600000u;
    valid = false;
    corruption_detected = true;
    ESP_LOGW(kTag, "settings: invalid rtc_resync_ms in blob; using default");
  }
  if (payload->node_role > (uint8_t)APP_NODE_ROLE_RELAY) {
    payload->node_role = (uint8_t)APP_NODE_ROLE_SENSOR;
    payload->allow_children = 0u;
    payload->allow_children_set = 1u;
    valid = false;
    corruption_detected = true;
    ESP_LOGW(kTag, "settings: invalid role in blob; forcing SENSOR");
  }
  if (payload->net_mode > (uint8_t)APP_NET_MODE_NONE) {
    payload->net_mode = (uint8_t)APP_NET_MODE_DIRECT_WIFI;
    valid = false;
    corruption_detected = true;
    ESP_LOGW(kTag, "settings: invalid net_mode in blob; forcing WIFI");
  }
  if (payload->mqtt_enabled > 1u) {
    payload->mqtt_enabled = 0u;
    valid = false;
    corruption_detected = true;
    ESP_LOGW(kTag, "settings: invalid mqtt_enabled in blob; disabling MQTT");
  }
  if (!IsPlausibleMqttUri_(payload->mqtt_broker_uri,
                           sizeof(payload->mqtt_broker_uri))) {
    if (payload->mqtt_enabled == 1u) {
      corruption_detected = true;
    }
    payload->mqtt_enabled = 0u;
    snprintf(payload->mqtt_broker_uri,
             sizeof(payload->mqtt_broker_uri),
             "%s",
             APP_SETTINGS_MQTT_BROKER_URI_DEFAULT);
    valid = false;
    ESP_LOGW(kTag, "settings: invalid MQTT URI in blob; disabling MQTT");
  }
  if (!IsPrintableSettingString(payload->mqtt_topic_prefix,
                                sizeof(payload->mqtt_topic_prefix))) {
    snprintf(payload->mqtt_topic_prefix,
             sizeof(payload->mqtt_topic_prefix),
             "%s",
             APP_SETTINGS_MQTT_TOPIC_PREFIX_DEFAULT);
    valid = false;
    ESP_LOGW(kTag, "settings: invalid MQTT topic prefix in blob; using default");
  }

  if (corruption_detected) {
    payload->node_role = (uint8_t)APP_NODE_ROLE_SENSOR;
    payload->allow_children = 0u;
    payload->allow_children_set = 1u;
    payload->net_mode = (uint8_t)APP_NET_MODE_DIRECT_WIFI;
    payload->mqtt_enabled = 0u;
    ESP_LOGW(
      kTag,
      "settings: corruption detected; enforcing safe SENSOR/WIFI startup");
  }

  return valid;
}

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

  app_settings_blob_t blob0;
  app_settings_blob_t blob1;
  bool valid_blob0 = ReadSettingsBlob(handle, kKeySettingsBlob0, &blob0);
  bool valid_blob1 = ReadSettingsBlob(handle, kKeySettingsBlob1, &blob1);

  const app_settings_blob_t* selected_blob = NULL;
  const char* selected_key = NULL;
  if (valid_blob0 && valid_blob1) {
    if (SettingsGenerationIsNewer(blob0.header.generation,
                                  blob1.header.generation)) {
      selected_blob = &blob0;
      selected_key = kKeySettingsBlob0;
    } else {
      selected_blob = &blob1;
      selected_key = kKeySettingsBlob1;
    }
  } else if (valid_blob0) {
    selected_blob = &blob0;
    selected_key = kKeySettingsBlob0;
  } else if (valid_blob1) {
    selected_blob = &blob1;
    selected_key = kKeySettingsBlob1;
  }

  if (selected_blob != NULL) {
    app_settings_persist_payload_t sanitized_payload = selected_blob->payload;
    const bool payload_fully_valid =
      ValidateAndSanitizePersistedPayload_(&sanitized_payload);
    if (!payload_fully_valid) {
      ESP_LOGW(kTag,
               "settings: validation adjusted invalid persisted values "
               "(blob %s gen=%" PRIu32 ")",
               selected_key,
               selected_blob->header.generation);
    }
    ApplyPersistedSettings(&sanitized_payload, settings_out);
    AppSettingsMaybeLoadDisplaySamplePeriodMs(handle, settings_out);
    g_settings_blob_generation = selected_blob->header.generation;
    g_display_attention_policy = settings_out->display_attention_policy;
    g_display_attention_mask = settings_out->display_attention_mask;
    g_saved_settings = *settings_out;
    g_saved_settings_valid = true;
    nvs_close(handle);
    ESP_LOGI(kTag,
             "settings: loaded from blob %s gen=%" PRIu32,
             selected_key,
             g_settings_blob_generation);
    LogSettingsLoaded(settings_out);
    return ESP_OK;
  }

  bool legacy_migrated = false;
  result = AppSettingsLoadLegacy(handle, settings_out, &legacy_migrated);
  AppSettingsMaybeLoadDisplaySamplePeriodMs(handle, settings_out);
  nvs_close(handle);
  if (result != ESP_OK) {
    return result;
  }

  g_saved_settings = *settings_out;
  g_saved_settings_valid = true;
  g_settings_blob_generation = 0;

  const esp_err_t migrate_result = AppSettingsSaveBlob(settings_out);
  if (migrate_result == ESP_OK) {
    ESP_LOGI(kTag,
             "settings: loaded legacy keys (blob missing/invalid), migrated "
             "to blob gen=%" PRIu32,
             g_settings_blob_generation);
  } else {
    ESP_LOGW(kTag,
             "settings: loaded legacy keys (blob missing/invalid); "
             "migration failed: %s",
             esp_err_to_name(migrate_result));
  }
  LogSettingsLoaded(settings_out);
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
  return PersistMutatedSettingsSnapshot_(MutateLogPeriodMs_, &log_period_ms);
}

/**
 * @brief Persist updated display/sample period to standalone NVS key.
 * @param display_sample_period_ms Display/sample period in milliseconds.
 * @return ESP_OK on success; otherwise an ESP-IDF error code.
 */
esp_err_t
AppSettingsSaveDisplaySamplePeriodMs(uint32_t display_sample_period_ms)
{
  if (display_sample_period_ms < 100u || display_sample_period_ms > 3600000u) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u32(handle,
                       kKeyDisplaySamplePeriodMs,
                       display_sample_period_ms);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);

  if (result != ESP_OK) {
    return result;
  }

  if (!g_saved_settings_valid) {
    ApplyDefaults(&g_saved_settings);
    g_saved_settings_valid = true;
  }
  g_saved_settings.display_sample_period_ms = display_sample_period_ms;
  return ESP_OK;
}

/**
 * @brief Execute AppSettingsSaveFramFlushWatermarkRecords.
 * @param watermark_records Parameter watermark_records.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveFramFlushWatermarkRecords(uint32_t watermark_records)
{
  return PersistMutatedSettingsSnapshot_(MutateFramFlushWatermarkRecords_,
                                         &watermark_records);
}

/**
 * @brief Execute AppSettingsSaveSdFlushPeriodMs.
 * @param period_ms Parameter period_ms.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveSdFlushPeriodMs(uint32_t period_ms)
{
  return PersistMutatedSettingsSnapshot_(MutateSdFlushPeriodMs_, &period_ms);
}

/**
 * @brief Execute AppSettingsSaveSdBatchBytes.
 * @param batch_bytes Parameter batch_bytes.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveSdBatchBytes(uint32_t batch_bytes)
{
  return PersistMutatedSettingsSnapshot_(MutateSdBatchBytes_, &batch_bytes);
}

esp_err_t
AppSettingsSaveSdVerifyReadback(bool enabled)
{
  return PersistMutatedSettingsSnapshot_(MutateSdVerifyReadback_, &enabled);
}

/**
 * @brief Execute AppSettingsSaveRtcResyncPeriodMs.
 * @param period_ms Parameter period_ms.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveRtcResyncPeriodMs(uint32_t period_ms)
{
  return PersistMutatedSettingsSnapshot_(MutateRtcResyncPeriodMs_, &period_ms);
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
  save_calibration_with_context_ctx_t save_ctx = {
    .model = model,
    .context = context,
  };
  return PersistMutatedSettingsSnapshot_(MutateCalibrationWithContext_,
                                         &save_ctx);
}

/**
 * @brief Execute AppSettingsSaveCalibrationSchedule.
 * @param settings Parameter settings.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveCalibrationSchedule(const app_settings_t* settings)
{
  if (settings == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  save_calibration_schedule_ctx_t save_ctx = {
    .source = settings,
  };
  return PersistMutatedSettingsSnapshot_(MutateCalibrationSchedule_, &save_ctx);
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
  context->table_version = (reader->conversion == kMax31865ConversionTablePt100)
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
  save_calibration_points_ctx_t save_ctx = {
    .points = points,
    .points_count = points_count,
  };
  return PersistMutatedSettingsSnapshot_(MutateCalibrationPoints_, &save_ctx);
}

/**
 * @brief Execute AppSettingsSaveRtdEmaEnabled.
 * @param enabled Parameter enabled.
 * @return Return the function result.
 */

/**
 * @brief Persist the active calibration domain metadata.
 * @param domain Calibration domain enum value.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveCalibrationDomain(calibration_domain_t domain)
{
  if (domain != CAL_DOMAIN_TEMP_C && domain != CAL_DOMAIN_RESISTANCE_OHM) {
    return ESP_ERR_INVALID_ARG;
  }
  return PersistMutatedSettingsSnapshot_(MutateCalibrationDomain_, &domain);
}

esp_err_t
AppSettingsSaveCalibrationConfig(uint16_t window_s, uint16_t ema_alpha_permille)
{
  if (window_s < CAL_WINDOW_DURATION_MIN_S ||
      window_s > CAL_WINDOW_DURATION_MAX_S || ema_alpha_permille == 0u ||
      ema_alpha_permille > 1000u) {
    return ESP_ERR_INVALID_ARG;
  }
  save_calibration_config_ctx_t save_ctx = {
    .window_s = window_s,
    .ema_alpha_permille = ema_alpha_permille,
  };
  return PersistMutatedSettingsSnapshot_(MutateCalibrationConfig_, &save_ctx);
}

esp_err_t
AppSettingsSaveRtdEmaEnabled(bool enabled)
{
  return PersistMutatedSettingsSnapshot_(MutateRtdEmaEnabled_, &enabled);
}

/**
 * @brief Execute AppSettingsSaveRtdEmaAlphaPermille.
 * @param permille Parameter permille.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveRtdEmaAlphaPermille(uint16_t permille)
{
  if (permille < 1 || permille > 1000) {
    return ESP_ERR_INVALID_ARG;
  }
  return PersistMutatedSettingsSnapshot_(MutateRtdEmaAlphaPermille_, &permille);
}

/**
 * @brief Execute AppSettingsSaveRtdFaultDebounceMs.
 * @param assert_ms Parameter assert_ms.
 * @param clear_ms Parameter clear_ms.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveRtdFaultDebounceMs(uint32_t assert_ms, uint32_t clear_ms)
{
  if (assert_ms > kRtdFaultDebounceMaxMs || clear_ms > kRtdFaultDebounceMaxMs) {
    return ESP_ERR_INVALID_ARG;
  }
  save_rtd_fault_debounce_ctx_t save_ctx = {
    .assert_ms = assert_ms,
    .clear_ms = clear_ms,
  };
  return PersistMutatedSettingsSnapshot_(MutateRtdFaultDebounceMs_, &save_ctx);
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
  save_time_zone_ctx_t save_ctx = {
    .tz_posix = tz_posix,
    .dst_enabled = dst_enabled,
  };
  return PersistMutatedSettingsSnapshot_(MutateTimeZone_, &save_ctx);
}

/**
 * @brief Persist the calibration method metadata string.
 * @param method Calibration method string (may be empty to clear).
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveCalibrationMethod(const char* method)
{
  if (!IsPrintableSettingString(method, APP_SETTINGS_CAL_METHOD_MAX_LEN)) {
    return ESP_ERR_INVALID_ARG;
  }
  return PersistMutatedSettingsSnapshot_(MutateCalibrationMethod_,
                                         (void*)method);
}

esp_err_t
AppSettingsSaveCalibrationMetar(const calibration_metar_reference_t* metar)
{
  if (metar == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  return PersistMutatedSettingsSnapshot_(MutateCalibrationMetar_, (void*)metar);
}

esp_err_t
AppSettingsSaveNodeRole(app_node_role_t node_role)
{
  return PersistMutatedSettingsSnapshot_(MutateNodeRole_, &node_role);
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
  save_allow_children_ctx_t save_ctx = {
    .allow_children = allow_children,
    .explicit_setting = explicit_setting,
  };
  return PersistMutatedSettingsSnapshot_(MutateAllowChildren_, &save_ctx);
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
  const esp_err_t result =
    PersistMutatedSettingsSnapshot_(MutateNetMode_, &mode);
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
  return PersistMutatedSettingsSnapshot_(MutateMqttEnabled_, &enabled);
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
  return PersistMutatedSettingsSnapshot_(MutateMqttBrokerUri_, (void*)uri);
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
  return PersistMutatedSettingsSnapshot_(MutateMqttTopicPrefix_,
                                         (void*)prefix);
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
  return PersistMutatedSettingsSnapshot_(MutateMqttQos_, &qos);
}

/**
 * @brief Execute AppSettingsSaveMqttRetain.
 * @param retain Parameter retain.
 * @return Return the function result.
 */
esp_err_t
AppSettingsSaveMqttRetain(bool retain)
{
  return PersistMutatedSettingsSnapshot_(MutateMqttRetain_, &retain);
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
  return PersistMutatedSettingsSnapshot_(MutateMqttBridgeMode_, &mode);
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
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemSdOut, DISP_SEV_ERROR);
  policy = DisplayAttentionPolicySet(policy, kDispAttnItemSdIo, DISP_SEV_ERROR);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemFramOvr, DISP_SEV_ERROR);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemRtdFault, DISP_SEV_ERROR);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemTimeBad, DISP_SEV_ERROR);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemMeshDown, DISP_SEV_WARN);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemNtpFail, DISP_SEV_WARN);
  policy =
    DisplayAttentionPolicySet(policy, kDispAttnItemSdSpace, DISP_SEV_WARN);
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
  const display_attention_mask_t mask = DisplayAttentionMaskFromPolicy(policy);
  const esp_err_t result =
    PersistMutatedSettingsSnapshot_(MutateDisplayAttentionPolicy_, &policy);
  if (result == ESP_OK) {
    g_display_attention_policy = policy;
    g_display_attention_mask = mask;
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
