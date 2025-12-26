#include "calibration.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

static const char* kTag = "calibration";

typedef struct
{
  int32_t samples_milli_c[CAL_WINDOW_SIZE];
  size_t count;
  size_t index;
  int32_t last_raw_milli_c;
  int32_t mean_raw_milli_c;
  int32_t stddev_raw_milli_c;
} cal_window_state_t;

static cal_window_state_t g_cal_window;

static esp_err_t
SolveLinearSystemGauss(
  int dimension,
  double matrix_a[CALIBRATION_MAX_POINTS][CALIBRATION_MAX_POINTS],
  double vector_b[CALIBRATION_MAX_POINTS],
  double vector_x_out[CALIBRATION_MAX_POINTS])
{
  // Augmented matrix: [A | b]
  for (int pivot_index = 0; pivot_index < dimension; ++pivot_index) {
    // Partial pivoting.
    int best_row = pivot_index;
    double best_abs = fabs(matrix_a[pivot_index][pivot_index]);
    for (int row = pivot_index + 1; row < dimension; ++row) {
      const double candidate_abs = fabs(matrix_a[row][pivot_index]);
      if (candidate_abs > best_abs) {
        best_abs = candidate_abs;
        best_row = row;
      }
    }

    if (best_abs < 1e-12) {
      ESP_LOGW(kTag, "Singular matrix (pivot too small)");
      return ESP_ERR_INVALID_STATE;
    }

    if (best_row != pivot_index) {
      // Swap rows in A.
      for (int col = pivot_index; col < dimension; ++col) {
        const double temp = matrix_a[pivot_index][col];
        matrix_a[pivot_index][col] = matrix_a[best_row][col];
        matrix_a[best_row][col] = temp;
      }
      // Swap rows in b.
      const double temp_b = vector_b[pivot_index];
      vector_b[pivot_index] = vector_b[best_row];
      vector_b[best_row] = temp_b;
    }

    // Normalize pivot row.
    const double pivot_value = matrix_a[pivot_index][pivot_index];
    for (int col = pivot_index; col < dimension; ++col) {
      matrix_a[pivot_index][col] /= pivot_value;
    }
    vector_b[pivot_index] /= pivot_value;

    // Eliminate other rows.
    for (int row = 0; row < dimension; ++row) {
      if (row == pivot_index) {
        continue;
      }
      const double factor = matrix_a[row][pivot_index];
      if (fabs(factor) < 1e-18) {
        continue;
      }
      for (int col = pivot_index; col < dimension; ++col) {
        matrix_a[row][col] -= factor * matrix_a[pivot_index][col];
      }
      vector_b[row] -= factor * vector_b[pivot_index];
    }
  }

  for (int index = 0; index < dimension; ++index) {
    vector_x_out[index] = vector_b[index];
  }
  return ESP_OK;
}

static bool
HasDuplicateRawValues(const calibration_point_t* points, size_t num_points)
{
  for (size_t i = 0; i < num_points; ++i) {
    for (size_t j = i + 1; j < num_points; ++j) {
      if (points[i].raw_avg_mC == points[j].raw_avg_mC) {
        return true;
      }
    }
  }
  return false;
}

static esp_err_t
FitLeastSquaresPolynomial(const calibration_point_t* points,
                          size_t num_points,
                          uint8_t degree,
                          calibration_model_t* model_out)
{
  const int dimension = (int)degree + 1;
  double matrix_a[CALIBRATION_MAX_POINTS][CALIBRATION_MAX_POINTS] = { 0 };
  double vector_b[CALIBRATION_MAX_POINTS] = { 0 };

  for (size_t index = 0; index < num_points; ++index) {
    const double x_value = points[index].raw_avg_mC / 1000.0;
    const double y_value = points[index].actual_mC / 1000.0;
    double x_powers[2 * CALIBRATION_MAX_DEGREE + 1] = { 0 };
    x_powers[0] = 1.0;
    for (int power = 1; power <= 2 * degree; ++power) {
      x_powers[power] = x_powers[power - 1] * x_value;
    }

    for (int row = 0; row < dimension; ++row) {
      for (int col = 0; col < dimension; ++col) {
        matrix_a[row][col] += x_powers[row + col];
      }
      vector_b[row] += y_value * x_powers[row];
    }
  }

  double solution[CALIBRATION_MAX_POINTS] = { 0 };
  esp_err_t result =
    SolveLinearSystemGauss(dimension, matrix_a, vector_b, solution);
  if (result != ESP_OK) {
    return result;
  }

  memset(model_out, 0, sizeof(*model_out));
  model_out->degree = degree;
  for (int index = 0; index < dimension; ++index) {
    model_out->coefficients[index] = solution[index];
  }
  model_out->is_valid = true;
  return ESP_OK;
}

static esp_err_t
ComputeDiagnostics(const calibration_point_t* points,
                   size_t num_points,
                   const calibration_model_t* model,
                   calibration_fit_diagnostics_t* diagnostics_out)
{
  if (diagnostics_out == NULL) {
    return ESP_OK;
  }

  double sum_sq = 0.0;
  double max_abs_residual = 0.0;
  for (size_t index = 0; index < num_points; ++index) {
    const double raw_c = points[index].raw_avg_mC / 1000.0;
    const double actual_c = points[index].actual_mC / 1000.0;
    const double predicted_c = CalibrationModelEvaluate(model, raw_c);
    const double residual = actual_c - predicted_c;
    const double abs_residual = fabs(residual);
    sum_sq += residual * residual;
    if (abs_residual > max_abs_residual) {
      max_abs_residual = abs_residual;
    }
  }

  diagnostics_out->rms_error_c =
    (num_points > 0) ? sqrt(sum_sq / (double)num_points) : 0.0;
  diagnostics_out->max_abs_residual_c = max_abs_residual;
  return ESP_OK;
}

static bool
IsSlopeReasonable(const calibration_fit_options_t* options,
                  const calibration_model_t* model)
{
  if (options->allow_wide_slope || model->degree < 1) {
    return true;
  }
  const double slope = model->coefficients[1];
  return slope >= options->min_slope && slope <= options->max_slope;
}

static bool
IsCorrectionReasonable(const calibration_fit_options_t* options,
                       const calibration_model_t* model,
                       calibration_fit_diagnostics_t* diagnostics_out)
{
  if (options->guard_min_c >= options->guard_max_c) {
    return true;
  }
  const double raw_min = options->guard_min_c;
  const double raw_max = options->guard_max_c;
  const double predicted_min = CalibrationModelEvaluate(model, raw_min);
  const double predicted_max = CalibrationModelEvaluate(model, raw_max);
  const double correction_min = predicted_min - raw_min;
  const double correction_max = predicted_max - raw_max;
  const double max_abs_correction =
    fmax(fabs(correction_min), fabs(correction_max));
  if (diagnostics_out != NULL) {
    diagnostics_out->max_abs_correction_c = max_abs_correction;
  }
  return max_abs_correction <= options->max_abs_correction_c;
}

void
CalibrationModelInitIdentity(calibration_model_t* model)
{
  if (model == NULL) {
    return;
  }
  model->degree = 1;
  model->coefficients[0] = 0.0;
  model->coefficients[1] = 1.0;
  model->coefficients[2] = 0.0;
  model->coefficients[3] = 0.0;
  model->is_valid = true;
}

double
CalibrationModelEvaluate(const calibration_model_t* model, double raw_c)
{
  if (model == NULL || !model->is_valid) {
    return raw_c;
  }
  double sum = 0.0;
  double x_pow = 1.0;
  for (uint8_t index = 0;
       index <= model->degree && index <= CALIBRATION_MAX_DEGREE;
       ++index) {
    sum += model->coefficients[index] * x_pow;
    x_pow *= raw_c;
  }
  return sum;
}

esp_err_t
CalibrationModelFitFromPoints(const calibration_point_t* points,
                              size_t num_points,
                              calibration_model_t* model_out)
{
  calibration_fit_options_t options;
  CalibrationFitOptionsInitDefault(&options);
  return CalibrationModelFitFromPointsWithOptions(
    points, num_points, &options, model_out, NULL);
}

void
CalibrationFitOptionsInitDefault(calibration_fit_options_t* options)
{
  if (options == NULL) {
    return;
  }
  options->mode = CAL_FIT_MODE_LINEAR;
  options->poly_degree = 1;
  options->allow_wide_slope = false;
  options->min_slope = CALIBRATION_MIN_SLOPE;
  options->max_slope = CALIBRATION_MAX_SLOPE;
  options->guard_min_c = CALIBRATION_GUARD_MIN_C;
  options->guard_max_c = CALIBRATION_GUARD_MAX_C;
  options->max_abs_correction_c = CALIBRATION_MAX_CORRECTION_C;
}

esp_err_t
CalibrationModelFitFromPointsWithOptions(
  const calibration_point_t* points,
  size_t num_points,
  const calibration_fit_options_t* options,
  calibration_model_t* model_out,
  calibration_fit_diagnostics_t* diagnostics_out)
{
  if (points == NULL || model_out == NULL || options == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (num_points < 1 || num_points > CALIBRATION_MAX_POINTS) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (HasDuplicateRawValues(points, num_points)) {
    ESP_LOGW(kTag, "duplicate raw values in calibration points");
    return ESP_ERR_INVALID_ARG;
  }

  if (num_points == 1) {
    const double offset =
      (points[0].actual_mC - points[0].raw_avg_mC) / 1000.0;
    CalibrationModelInitIdentity(model_out);
    model_out->degree = 1;
    model_out->coefficients[0] = offset;
    model_out->coefficients[1] = 1.0;
    model_out->is_valid = true;
    if (diagnostics_out != NULL) {
      diagnostics_out->rms_error_c = 0.0;
      diagnostics_out->max_abs_residual_c = 0.0;
      diagnostics_out->max_abs_correction_c = fabs(offset);
    }
    return ESP_OK;
  }

  uint8_t degree = 1;
  switch (options->mode) {
    case CAL_FIT_MODE_LINEAR:
      degree = 1;
      break;
    case CAL_FIT_MODE_PIECEWISE:
      ESP_LOGW(kTag, "piecewise fit mode not implemented");
      return ESP_ERR_NOT_SUPPORTED;
    case CAL_FIT_MODE_POLY:
      degree = options->poly_degree;
      if (degree < 1 || degree > CALIBRATION_MAX_DEGREE) {
        ESP_LOGW(kTag, "invalid polynomial degree %u", degree);
        return ESP_ERR_INVALID_ARG;
      }
      break;
    default:
      ESP_LOGW(kTag, "unknown fit mode");
      return ESP_ERR_INVALID_ARG;
  }

  if (degree + 1 > num_points) {
    ESP_LOGW(kTag,
             "not enough points for degree %u (need >=%u)",
             degree,
             (unsigned)(degree + 1));
    return ESP_ERR_INVALID_SIZE;
  }

  esp_err_t result =
    FitLeastSquaresPolynomial(points, num_points, degree, model_out);
  if (result != ESP_OK) {
    return result;
  }

  ComputeDiagnostics(points, num_points, model_out, diagnostics_out);

  if (!IsSlopeReasonable(options, model_out)) {
    ESP_LOGW(kTag,
             "slope out of bounds (%.6f not in [%.3f, %.3f])",
             model_out->coefficients[1],
             options->min_slope,
             options->max_slope);
    return ESP_ERR_INVALID_STATE;
  }

  if (!IsCorrectionReasonable(options, model_out, diagnostics_out)) {
    ESP_LOGW(kTag,
             "correction exceeds max abs %.2fC within [%.1f, %.1f]",
             options->max_abs_correction_c,
             options->guard_min_c,
             options->guard_max_c);
    return ESP_ERR_INVALID_STATE;
  }

  return ESP_OK;
}

void
CalWindowPushRawSample(int32_t raw_milli_c)
{
  g_cal_window.samples_milli_c[g_cal_window.index] = raw_milli_c;
  g_cal_window.index = (g_cal_window.index + 1) % CAL_WINDOW_SIZE;
  if (g_cal_window.count < CAL_WINDOW_SIZE) {
    g_cal_window.count++;
  }

  g_cal_window.last_raw_milli_c = raw_milli_c;

  double sum = 0.0;
  for (size_t i = 0; i < g_cal_window.count; ++i) {
    sum += g_cal_window.samples_milli_c[i];
  }
  const double mean = sum / (double)g_cal_window.count;

  double variance_sum = 0.0;
  for (size_t i = 0; i < g_cal_window.count; ++i) {
    const double delta =
      (double)g_cal_window.samples_milli_c[i] - mean;
    variance_sum += delta * delta;
  }
  const double variance =
    (g_cal_window.count > 0) ? (variance_sum / g_cal_window.count) : 0.0;
  const double stddev = sqrt(variance);

  g_cal_window.mean_raw_milli_c = (int32_t)llround(mean);
  g_cal_window.stddev_raw_milli_c = (int32_t)llround(stddev);
}

bool
CalWindowIsReady(void)
{
  return g_cal_window.count >= CAL_WINDOW_SIZE;
}

size_t
CalWindowGetSampleCount(void)
{
  return g_cal_window.count;
}

void
CalWindowGetStats(int32_t* out_last_raw_mC,
                  int32_t* out_mean_raw_mC,
                  int32_t* out_stddev_mC)
{
  if (out_last_raw_mC != NULL) {
    *out_last_raw_mC = g_cal_window.last_raw_milli_c;
  }
  if (out_mean_raw_mC != NULL) {
    *out_mean_raw_mC = g_cal_window.mean_raw_milli_c;
  }
  if (out_stddev_mC != NULL) {
    *out_stddev_mC = g_cal_window.stddev_raw_milli_c;
  }
}
