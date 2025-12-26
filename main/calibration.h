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
#define CAL_WINDOW_SIZE 16
#define CALIBRATION_MIN_SLOPE 0.8
#define CALIBRATION_MAX_SLOPE 1.2
#define CALIBRATION_GUARD_MIN_C -50.0
#define CALIBRATION_GUARD_MAX_C 200.0
#define CALIBRATION_MAX_CORRECTION_C 20.0

  typedef struct
  {
    int32_t raw_avg_mC;
    int32_t actual_mC;
    int32_t raw_stddev_mC;
    uint16_t sample_count;
    uint8_t time_valid;
    int64_t timestamp_epoch_sec;
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
  void CalibrationModelInitIdentity(calibration_model_t* model);

  // y = sum_{i=0..degree} c[i] * x^i
  double CalibrationModelEvaluate(const calibration_model_t* model,
                                  double raw_c);
  double CalibrationModelEvaluateWithPoints(const calibration_model_t* model,
                                            double raw_c,
                                            const calibration_point_t* points,
                                            size_t num_points);

  // Fit a calibration model using default options.
  // - N=1 => offset-only correction with slope=1 (y=x+offset)
  // - N>=2 => linear least-squares fit (y=a+b*x)
  esp_err_t CalibrationModelFitFromPoints(const calibration_point_t* points,
                                          size_t num_points,
                                          calibration_model_t* model_out);

  void CalibrationFitOptionsInitDefault(calibration_fit_options_t* options);

  esp_err_t CalibrationModelFitFromPointsWithOptions(
    const calibration_point_t* points,
    size_t num_points,
    const calibration_fit_options_t* options,
    calibration_model_t* model_out,
    calibration_fit_diagnostics_t* diagnostics_out);

  void CalWindowPushRawSample(int32_t raw_milli_c);
  bool CalWindowIsReady(void);
  size_t CalWindowGetSampleCount(void);
  void CalWindowGetStats(int32_t* out_last_raw_mC,
                         int32_t* out_mean_raw_mC,
                         int32_t* out_stddev_mC);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_CALIBRATION_H_
