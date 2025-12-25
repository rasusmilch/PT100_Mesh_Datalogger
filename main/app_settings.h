#ifndef PT100_LOGGER_APP_SETTINGS_H_
#define PT100_LOGGER_APP_SETTINGS_H_

#include <stdbool.h>
#include <stdint.h>

#include "calibration.h"
#include "esp_err.h"

#define APP_SETTINGS_TZ_POSIX_MAX_LEN 64
#define APP_SETTINGS_TZ_DEFAULT_POSIX "CST6CDT,M3.2.0/2,M11.1.0/2"
#define APP_SETTINGS_TZ_DEFAULT_STD "CST6"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    uint32_t log_period_ms;
    uint32_t fram_flush_watermark_records;
    uint32_t sd_flush_period_ms;
    uint32_t sd_batch_bytes_target;
    calibration_model_t calibration;
    char tz_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
    bool dst_enabled;
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
  esp_err_t AppSettingsSaveCalibration(const calibration_model_t* model);

  // Persists updated timezone string + DST toggle.
  esp_err_t AppSettingsSaveTimeZone(const char* tz_posix, bool dst_enabled);

  // Applies TZ to the runtime environment.
  void AppSettingsApplyTimeZone(const app_settings_t* settings);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_APP_SETTINGS_H_
