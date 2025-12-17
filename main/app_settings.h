#ifndef PT100_LOGGER_APP_SETTINGS_H_
#define PT100_LOGGER_APP_SETTINGS_H_

#include <stdint.h>

#include "calibration.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    uint32_t log_period_ms;
    uint32_t fram_flush_watermark_records;
    calibration_model_t calibration;
  } app_settings_t;

  // Loads settings from NVS. If keys are missing or invalid, applies defaults.
  esp_err_t AppSettingsLoad(app_settings_t* settings_out);

  // Persists updated log interval to NVS.
  esp_err_t AppSettingsSaveLogPeriodMs(uint32_t log_period_ms);

  // Persists updated FRAM flush watermark to NVS.
  esp_err_t AppSettingsSaveFramFlushWatermarkRecords(
    uint32_t watermark_records);

  // Persists updated calibration model to NVS.
  esp_err_t AppSettingsSaveCalibration(const calibration_model_t* model);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_APP_SETTINGS_H_
