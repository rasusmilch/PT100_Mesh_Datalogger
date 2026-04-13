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
#define CAL_WINDOW_SIZE 30
#define CALIBRATION_MIN_SLOPE 0.8
#define CALIBRATION_MAX_SLOPE 1.2
#define CALIBRATION_GUARD_MIN_C -50.0
#define CALIBRATION_GUARD_MAX_C 200.0
#define CALIBRATION_MAX_CORRECTION_C 20.0
#define CAL_CAPTURE_DRIFT_UNAVAILABLE_MC_PER_MIN INT32_MIN
#define CAL_CAPTURE_DELTA_UNAVAILABLE_MC INT32_MIN
#define CAL_CAPTURE_DRIFT_LIMIT_UNAVAILABLE_MC_PER_MIN INT32_MIN

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

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_CALIBRATION_H_
