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

  typedef struct
  {
    double raw_c;
    double actual_c;
  } calibration_point_t;

  typedef struct
  {
    uint8_t degree;                              // 0..3
    double coefficients[CALIBRATION_MAX_POINTS]; // c0..c3
    bool is_valid;
  } calibration_model_t;

  // Identity mapping (y = x).
  void CalibrationModelInitIdentity(calibration_model_t* model);

  // y = sum_{i=0..degree} c[i] * x^i
  double CalibrationModelEvaluate(const calibration_model_t* model,
                                  double raw_c);

  // Fit a polynomial through N points.
  // - N=1 => degree 0 (offset-only), y=c0
  // - N=2 => degree 1 (linear)
  // - N=3 => degree 2 (quadratic)
  // - N=4 => degree 3 (cubic)
  esp_err_t CalibrationModelFitFromPoints(const calibration_point_t* points,
                                          size_t num_points,
                                          calibration_model_t* model_out);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_CALIBRATION_H_
