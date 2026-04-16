#ifndef PT100_LOGGER_CALIBRATION_H_
#define PT100_LOGGER_CALIBRATION_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CALIBRATION_MAX_POINTS 4
#define CALIBRATION_MAX_DEGREE 3
#define CAL_WINDOW_MAX_SAMPLES 720
#define CAL_WINDOW_DURATION_DEFAULT_S 60u
#define CAL_WINDOW_DURATION_MIN_S 10u
#define CAL_WINDOW_DURATION_MAX_S 600u
#define CAL_TREND_EMA_ALPHA_DEFAULT_PERMILLE 200u
#define CALIBRATION_MIN_SLOPE 0.8
#define CALIBRATION_MAX_SLOPE 1.2
#define CALIBRATION_GUARD_MIN_C -50.0
#define CALIBRATION_GUARD_MAX_C 200.0
#define CALIBRATION_MAX_CORRECTION_C 20.0
#define CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN INT32_MIN
#define CAL_CAPTURE_DELTA_UNAVAILABLE_MC INT32_MIN
#define CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN INT32_MIN
#define CAL_CAPTURE_WINDOW_S_UNAVAILABLE INT16_MIN
#define CAL_CAPTURE_EMA_ALPHA_UNAVAILABLE_PERMILLE INT16_MIN

  typedef enum
  {
    CAL_DOMAIN_TEMP_C = 0,
    CAL_DOMAIN_RESISTANCE_OHM = 1,
  } calibration_domain_t;

  typedef enum
  {
    CAL_DRIFT_LIMIT_SOURCE_DEFAULT = 0,
    CAL_DRIFT_LIMIT_SOURCE_USER = 1,
    CAL_DRIFT_LIMIT_SOURCE_DISABLED = 2,
    CAL_DRIFT_LIMIT_SOURCE_LEGACY_UNAVAILABLE = 3,
  } calibration_drift_limit_source_t;

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
    int32_t captured_drift_mC_per_min;
    int32_t captured_delta_mC;
    int32_t capture_drift_limit_mC_per_min;
    uint8_t drift_limit_source;
    int16_t captured_window_s;
    int16_t captured_ema_alpha_permille;
  } calibration_point_t;

  typedef enum
  {
    CAL_FIT_MODE_LINEAR = 0,
    CAL_FIT_MODE_PIECEWISE,
    CAL_FIT_MODE_POLY
  } calibration_fit_mode_t;

  typedef struct
  {
    calibration_fit_mode_t mode;
    uint8_t degree;                              // 0..3
    double coefficients[CALIBRATION_MAX_POINTS]; // c0..c3
    bool is_valid;
  } calibration_model_t;

  typedef struct
  {
    calibration_fit_mode_t mode;
    uint8_t poly_degree;
    bool allow_wide_slope;
    double min_slope;
    double max_slope;
    double guard_min_c;
    double guard_max_c;
    double max_abs_correction_c;
  } calibration_fit_options_t;

  typedef struct
  {
    double rms_error_c;
    double max_abs_residual_c;
    double max_abs_correction_c;
  } calibration_fit_diagnostics_t;

  typedef struct
  {
    size_t samples_milli_c_bytes;
    size_t samples_milli_ohm_bytes;
    size_t samples_time_us_bytes;
    bool samples_milli_c_in_psram;
    bool samples_milli_ohm_in_psram;
    bool samples_time_us_in_psram;
  } cal_window_storage_layout_t;

  // Identity mapping (y = x).
  /**
   * @brief Execute CalibrationModelInitIdentity.
   * @param model Parameter model.
   */
  void CalibrationModelInitIdentity(calibration_model_t* model);

  // y = sum_{i=0..degree} c[i] * x^i
  /**
   * @brief Execute CalibrationModelEvaluate.
   * @param model Parameter model.
   * @param raw_c Parameter raw_c.
   * @return Return the function result.
   */
  double CalibrationModelEvaluate(const calibration_model_t* model,
                                  double raw_c);
  /**
   * @brief Execute CalibrationModelEvaluateWithPoints.
   * @param model Parameter model.
   * @param raw_c Parameter raw_c.
   * @param points Parameter points.
   * @param num_points Parameter num_points.
   * @return Return the function result.
   */
  double CalibrationModelEvaluateWithPoints(const calibration_model_t* model,
                                            double raw_c,
                                            const calibration_point_t* points,
                                            size_t num_points);

  // Fit a calibration model using default options.
  // - N=1 => offset-only correction with slope=1 (y=x+offset)
  // - N>=2 => linear least-squares fit (y=a+b*x)
  /**
   * @brief Execute CalibrationModelFitFromPoints.
   * @param points Parameter points.
   * @param num_points Parameter num_points.
   * @param model_out Parameter model_out.
   * @return Return the function result.
   */
  esp_err_t CalibrationModelFitFromPoints(const calibration_point_t* points,
                                          size_t num_points,
                                          calibration_model_t* model_out);

  /**
   * @brief Execute CalibrationFitOptionsInitDefault.
   * @param options Parameter options.
   */
  void CalibrationFitOptionsInitDefault(calibration_fit_options_t* options);

  /**
   * @brief Execute CalibrationModelFitFromPointsWithOptions.
   * @param points Parameter points.
   * @param num_points Parameter num_points.
   * @param options Parameter options.
   * @param model_out Parameter model_out.
   * @param diagnostics_out Parameter diagnostics_out.
   * @return Return the function result.
   */
  esp_err_t CalibrationModelFitFromPointsWithOptions(
    const calibration_point_t* points,
    size_t num_points,
    const calibration_fit_options_t* options,
    calibration_model_t* model_out,
    calibration_fit_diagnostics_t* diagnostics_out);

  /**
   * @brief Execute CalWindowPushRawSample.
   * @param raw_milli_c Parameter raw_milli_c.
   */
  void CalWindowPushRawSample(int32_t raw_milli_c, int32_t raw_milli_ohm);
  /**
   * @brief Execute CalWindowClear.
   */
  void CalWindowClear(void);
  /**
   * @brief Execute CalWindowIsReady.
   * @return Return the function result.
   */
  bool CalWindowIsReady(void);
  /**
   * @brief Execute CalWindowGetSampleCount.
   * @return Return the function result.
   */
  size_t CalWindowGetSampleCount(void);
  /**
   * @brief Execute CalWindowGetStats.
   * @param out_last_raw_mC Parameter out_last_raw_mC.
   * @param out_mean_raw_mC Parameter out_mean_raw_mC.
   * @param out_stddev_mC Parameter out_stddev_mC.
   */
  void CalWindowGetStats(int32_t* out_last_raw_mC,
                         int32_t* out_mean_raw_mC,
                         int32_t* out_stddev_mC);

  /**
   * @brief Read latest/mean/stddev resistance statistics from the calibration
   * capture window.
   * @param out_last_raw_mOhm Receives the most recent sample in milli-ohms.
   * @param out_mean_raw_mOhm Receives the mean sample in milli-ohms.
   * @param out_stddev_mOhm Receives the stddev in milli-ohms.
   */
  void CalWindowGetResistanceStats(int32_t* out_last_raw_mOhm,
                                   int32_t* out_mean_raw_mOhm,
                                   int32_t* out_stddev_mOhm);

  /**
   * @brief Read beginning/end segment means, delta, and regression drift from
   * calibration window samples.
   * @param out_begin_mean_raw_mC Receives beginning-segment mean (milli-C).
   * @param out_end_mean_raw_mC Receives ending-segment mean (milli-C).
   * @param out_delta_raw_mC Receives end-begin delta (milli-C).
   * @param out_elapsed_s Receives elapsed seconds between oldest/newest
   * samples in the current window.
   * @param out_drift_c_per_min Receives signed regression drift in C/min.
   * @param out_abs_drift_c_per_min Receives absolute drift in C/min.
   */
  void CalWindowGetTrendStats(int32_t* out_begin_mean_raw_mC,
                              int32_t* out_end_mean_raw_mC,
                              int32_t* out_delta_raw_mC,
                              double* out_elapsed_s,
                              double* out_drift_c_per_min,
                              double* out_abs_drift_c_per_min);
  void CalWindowSetCalibratedTempStats(double last_calibrated_temp_c,
                                       double mean_calibrated_temp_c,
                                       double stddev_calibrated_temp_c,
                                       bool valid,
                                       size_t sample_count,
                                       uint32_t generation);
  void CalWindowGetCalibratedTempStats(double* out_last_calibrated_temp_c,
                                       double* out_mean_calibrated_temp_c,
                                       double* out_stddev_calibrated_temp_c,
                                       bool* out_valid,
                                       size_t* out_sample_count,
                                       uint32_t* out_generation);
  uint32_t CalWindowGetActiveGeneration(void);
  /**
   * @brief Read one sample from the active calibration window by age.
   * @param index_from_oldest Zero-based index in active window order.
   * @param out_raw_mC Receives raw temperature sample in milli-Celsius.
   * @param out_raw_mOhm Receives raw resistance sample in milli-ohms.
   * @param out_time_us Receives sample timestamp in microseconds.
   * @return true when index is in-range and outputs were populated.
   */
  bool CalWindowGetActiveSampleByIndex(size_t index_from_oldest,
                                       int32_t* out_raw_mC,
                                       int32_t* out_raw_mOhm,
                                       int64_t* out_time_us);
  void CalWindowSetDurationSeconds(uint16_t window_s);
  uint16_t CalWindowGetDurationSeconds(void);
  void CalWindowSetTrendEmaAlphaPermille(uint16_t alpha_permille);
  uint16_t CalWindowGetTrendEmaAlphaPermille(void);
  void CalWindowResetTrendEma(void);
  void CalWindowGetTrendEmaStats(double* out_delta_c_ema,
                                 double* out_drift_c_per_min_ema,
                                 bool* out_initialized);
  void CalWindowGetStorageLayout(cal_window_storage_layout_t* out_layout);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_CALIBRATION_H_
