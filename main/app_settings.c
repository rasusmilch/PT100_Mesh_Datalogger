#include "app_settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char* kTag = "settings";

static const char* kNvsNamespace = "pt100_logger";
static const char* kKeyLogPeriodMs = "log_period_ms";
static const char* kKeyFlushWatermark = "flush_wm_rec";
static const char* kKeySdFlushPeriodMs = "sd_flush_ms";
static const char* kKeySdBatchBytes = "sd_batch_bytes";
static const char* kKeyCalDegree = "cal_deg";
static const char* kKeyCalCoeffs = "cal_coeffs";
static const char* kKeyTzPosix = "tz_posix";
static const char* kKeyDstEnabled = "dst_enabled";

static void
ApplyDefaults(app_settings_t* settings)
{
  settings->log_period_ms = (uint32_t)CONFIG_APP_LOG_PERIOD_MS_DEFAULT;
  settings->fram_flush_watermark_records =
    (uint32_t)CONFIG_APP_FRAM_FLUSH_WATERMARK_RECORDS_DEFAULT;
  settings->sd_flush_period_ms = (uint32_t)CONFIG_APP_SD_PERIODIC_FLUSH_MS;
  settings->sd_batch_bytes_target = (uint32_t)CONFIG_APP_SD_BATCH_BYTES_TARGET;
  CalibrationModelInitIdentity(&settings->calibration);
  snprintf(settings->tz_posix,
           sizeof(settings->tz_posix),
           "%s",
           APP_SETTINGS_TZ_DEFAULT_POSIX);
  settings->dst_enabled = true;
}

static esp_err_t
OpenNvs(nvs_handle_t* handle_out)
{
  return nvs_open(kNvsNamespace, NVS_READWRITE, handle_out);
}

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

  nvs_close(handle);
  ESP_LOGI(kTag,
           "Loaded: period=%ums wm=%u sd_flush_ms=%u sd_batch=%u deg=%u tz=%s dst=%u",
           (unsigned)settings_out->log_period_ms,
           (unsigned)settings_out->fram_flush_watermark_records,
           (unsigned)settings_out->sd_flush_period_ms,
           (unsigned)settings_out->sd_batch_bytes_target,
           (unsigned)settings_out->calibration.degree,
           settings_out->tz_posix,
           settings_out->dst_enabled ? 1u : 0u);
  return ESP_OK;
}

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

esp_err_t
AppSettingsSaveCalibration(const calibration_model_t* model)
{
  if (model == NULL || !model->is_valid) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t result = OpenNvs(&handle);
  if (result != ESP_OK) {
    return result;
  }

  result = nvs_set_u8(handle, kKeyCalDegree, model->degree);
  if (result == ESP_OK) {
    result = nvs_set_blob(handle,
                          kKeyCalCoeffs,
                          model->coefficients,
                          sizeof(double) * CALIBRATION_MAX_POINTS);
  }
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

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
