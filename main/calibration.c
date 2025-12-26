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
  if (points == NULL || model_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (num_points < 1 || num_points > CALIBRATION_MAX_POINTS) {
    return ESP_ERR_INVALID_SIZE;
  }

  if (num_points == 1) {
    // Deterministic behavior: treat single-point as offset-only correction with
    // slope=1.
    const double offset =
      (points[0].actual_mC - points[0].raw_avg_mC) / 1000.0;
    CalibrationModelInitIdentity(model_out);
    model_out->degree = 1;
    model_out->coefficients[0] = offset;
    model_out->coefficients[1] = 1.0;
    model_out->is_valid = true;
    return ESP_OK;
  }

  // Build Vandermonde matrix for polynomial degree (N-1).
  const int dimension = (int)num_points;
  double matrix_a[CALIBRATION_MAX_POINTS][CALIBRATION_MAX_POINTS];
  double vector_b[CALIBRATION_MAX_POINTS];
  memset(matrix_a, 0, sizeof(matrix_a));
  memset(vector_b, 0, sizeof(vector_b));

  for (int row = 0; row < dimension; ++row) {
    const double x_value = points[row].raw_avg_mC / 1000.0;
    double x_pow = 1.0;
    for (int col = 0; col < dimension; ++col) {
      matrix_a[row][col] = x_pow;
      x_pow *= x_value;
    }
    vector_b[row] = points[row].actual_mC / 1000.0;
  }

  double solution[CALIBRATION_MAX_POINTS] = { 0 };
  esp_err_t result =
    SolveLinearSystemGauss(dimension, matrix_a, vector_b, solution);
  if (result != ESP_OK) {
    return result;
  }

  memset(model_out, 0, sizeof(*model_out));
  model_out->degree = (uint8_t)(dimension - 1);
  for (int index = 0; index < dimension; ++index) {
    model_out->coefficients[index] = solution[index];
  }
  model_out->is_valid = true;
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
