#include "calibration.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* kTag = "calibration";

typedef struct
{
  int32_t* samples_milli_c;
  int32_t* samples_milli_ohm;
  int64_t* samples_time_us;
  size_t count;
  size_t head;
  size_t write_index;
  uint16_t window_duration_s;
  uint16_t trend_ema_alpha_permille;
  bool trend_ema_initialized;
  double trend_ema_delta_c;
  double trend_ema_drift_c_per_min;
  int32_t last_raw_milli_c;
  int32_t mean_raw_milli_c;
  int32_t stddev_raw_milli_c;
  int32_t last_raw_milli_ohm;
  int32_t mean_raw_milli_ohm;
  int32_t stddev_raw_milli_ohm;
  size_t active_count;
  size_t active_oldest_index;
  size_t active_newest_index;
  double active_elapsed_s;
  bool active_is_ready;
  double active_sum_milli_c;
  double active_sum_sq_milli_c;
  double active_sum_milli_ohm;
  double active_sum_sq_milli_ohm;
} cal_window_state_t;

typedef struct
{
  size_t active_count;
  size_t oldest_index;
  size_t newest_index;
  double elapsed_s;
  bool is_ready;
} calibration_active_window_info_t;

static cal_window_state_t g_cal_window = {
  .samples_milli_c = g_cal_window_samples_milli_c_fallback,
  .samples_milli_ohm = g_cal_window_samples_milli_ohm_fallback,
  .samples_time_us = g_cal_window_samples_time_us_fallback,
  .window_duration_s = CAL_WINDOW_DURATION_DEFAULT_S,
  .trend_ema_alpha_permille = CAL_TREND_EMA_ALPHA_DEFAULT_PERMILLE,
};

/**
 * @brief Compute begin/end segment delta for calibration window.
 * @param begin_mean_mC Beginning segment mean in milli-Celsius.
 * @param end_mean_mC Ending segment mean in milli-Celsius.
 * @return End-minus-begin delta in Celsius.
 */
static double ComputeCalibrationWindowDeltaCRaw(double begin_mean_mC,
                                                double end_mean_mC);

static void
ComputeCalibrationWindowTrendStats_(size_t count,
                                    size_t oldest_index,
                                    size_t segment_count,
                                    double* out_begin_mean_mC,
                                    double* out_end_mean_mC,
                                    double* out_drift_c_per_min);
static void ResetCalibrationTrendEma(void);
static void UpdateCalibrationTrendEma(double delta_c_raw,
                                      double drift_c_per_min_raw);
static void RebuildCalibrationWindowState_(void);
static calibration_active_window_info_t ResolveActiveWindowInfo_(void);
static bool CalWindowEnsureStorage_(void);
static bool CalWindowStorageIsPsram_(const void* ptr);

static int32_t g_cal_window_samples_milli_c_fallback[CAL_WINDOW_MAX_SAMPLES];
static int32_t g_cal_window_samples_milli_ohm_fallback[CAL_WINDOW_MAX_SAMPLES];
static int64_t g_cal_window_samples_time_us_fallback[CAL_WINDOW_MAX_SAMPLES];
static bool g_cal_window_storage_initialized = false;
static bool g_cal_window_samples_milli_c_in_psram = false;
static bool g_cal_window_samples_milli_ohm_in_psram = false;
static bool g_cal_window_samples_time_us_in_psram = false;

/**
 * @brief Execute InterpolateResidual.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @param raw_c Parameter raw_c.
 * @return Return the function result.
 */
static double
InterpolateResidual(const calibration_point_t* points,
                    size_t num_points,
                    double raw_c)
{
  if (points == NULL || num_points == 0) {
    return 0.0;
  }

  int lower_index = -1;
  int upper_index = -1;
  double lower_x = 0.0;
  double upper_x = 0.0;

  for (size_t index = 0; index < num_points; ++index) {
    const double x_value = points[index].raw_avg_mC / 1000.0;
    if (x_value <= raw_c) {
      if (lower_index < 0 || x_value > lower_x) {
        lower_index = (int)index;
        lower_x = x_value;
      }
    }
    if (x_value >= raw_c) {
      if (upper_index < 0 || x_value < upper_x) {
        upper_index = (int)index;
        upper_x = x_value;
      }
    }
  }

  if (lower_index < 0 && upper_index < 0) {
    return 0.0;
  }

  if (lower_index < 0) {
    const calibration_point_t* upper = &points[upper_index];
    return (upper->actual_mC - upper->raw_avg_mC) / 1000.0;
  }

  if (upper_index < 0) {
    const calibration_point_t* lower = &points[lower_index];
    return (lower->actual_mC - lower->raw_avg_mC) / 1000.0;
  }

  if (lower_index == upper_index || fabs(upper_x - lower_x) < 1e-12) {
    const calibration_point_t* point = &points[lower_index];
    return (point->actual_mC - point->raw_avg_mC) / 1000.0;
  }

  const calibration_point_t* lower = &points[lower_index];
  const calibration_point_t* upper = &points[upper_index];
  const double lower_residual =
    (lower->actual_mC - lower->raw_avg_mC) / 1000.0;
  const double upper_residual =
    (upper->actual_mC - upper->raw_avg_mC) / 1000.0;
  const double t = (raw_c - lower_x) / (upper_x - lower_x);
  return lower_residual + t * (upper_residual - lower_residual);
}

/**
 * @brief Execute SolveLinearSystemGauss.
 * @param dimension Parameter dimension.
 * @param matrix_a Parameter matrix_a.
 * @param vector_b Parameter vector_b.
 * @param vector_x_out Parameter vector_x_out.
 * @return Return the function result.
 */
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

/**
 * @brief Resolve authoritative active calibration-window bounds/readiness.
 * @return Active-window sample indices/count, elapsed span, and readiness.
 */
static calibration_active_window_info_t
ResolveActiveWindowInfo_(void)
{
  return (calibration_active_window_info_t){
    .active_count = g_cal_window.active_count,
    .oldest_index = g_cal_window.active_oldest_index,
    .newest_index = g_cal_window.active_newest_index,
    .elapsed_s = g_cal_window.active_elapsed_s,
    .is_ready = g_cal_window.active_is_ready,
  };
}

static bool
CalWindowStorageIsPsram_(const void* ptr)
{
  if (ptr == NULL) {
    return false;
  }
  return heap_caps_check_addr(ptr, MALLOC_CAP_SPIRAM);
}

static bool
CalWindowEnsureStorage_(void)
{
  if (g_cal_window_storage_initialized) {
    return (g_cal_window.samples_milli_c != NULL &&
            g_cal_window.samples_milli_ohm != NULL &&
            g_cal_window.samples_time_us != NULL);
  }

  const size_t milli_c_bytes =
    sizeof(int32_t) * (size_t)CAL_WINDOW_MAX_SAMPLES;
  const size_t milli_ohm_bytes =
    sizeof(int32_t) * (size_t)CAL_WINDOW_MAX_SAMPLES;
  const size_t time_us_bytes =
    sizeof(int64_t) * (size_t)CAL_WINDOW_MAX_SAMPLES;

  int32_t* samples_milli_c = (int32_t*)heap_caps_malloc(
    milli_c_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  int32_t* samples_milli_ohm = (int32_t*)heap_caps_malloc(
    milli_ohm_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  int64_t* samples_time_us = (int64_t*)heap_caps_malloc(
    time_us_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (samples_milli_c == NULL || samples_milli_ohm == NULL ||
      samples_time_us == NULL) {
    if (samples_milli_c != NULL) {
      heap_caps_free(samples_milli_c);
    }
    if (samples_milli_ohm != NULL) {
      heap_caps_free(samples_milli_ohm);
    }
    if (samples_time_us != NULL) {
      heap_caps_free(samples_time_us);
    }
    g_cal_window.samples_milli_c = g_cal_window_samples_milli_c_fallback;
    g_cal_window.samples_milli_ohm = g_cal_window_samples_milli_ohm_fallback;
    g_cal_window.samples_time_us = g_cal_window_samples_time_us_fallback;
  } else {
    g_cal_window.samples_milli_c = samples_milli_c;
    g_cal_window.samples_milli_ohm = samples_milli_ohm;
    g_cal_window.samples_time_us = samples_time_us;
  }

  g_cal_window_samples_milli_c_in_psram =
    CalWindowStorageIsPsram_(g_cal_window.samples_milli_c);
  g_cal_window_samples_milli_ohm_in_psram =
    CalWindowStorageIsPsram_(g_cal_window.samples_milli_ohm);
  g_cal_window_samples_time_us_in_psram =
    CalWindowStorageIsPsram_(g_cal_window.samples_time_us);
  g_cal_window_storage_initialized = true;

  ESP_LOGI(kTag,
           "cal window storage: milli_c=%s bytes=%u milli_ohm=%s bytes=%u "
           "time=%s bytes=%u",
           g_cal_window_samples_milli_c_in_psram ? "psram" : "internal",
           (unsigned)milli_c_bytes,
           g_cal_window_samples_milli_ohm_in_psram ? "psram" : "internal",
           (unsigned)milli_ohm_bytes,
           g_cal_window_samples_time_us_in_psram ? "psram" : "internal",
           (unsigned)time_us_bytes);

  return (g_cal_window.samples_milli_c != NULL &&
          g_cal_window.samples_milli_ohm != NULL &&
          g_cal_window.samples_time_us != NULL);
}

static void
RebuildCalibrationWindowState_(void)
{
  g_cal_window.active_count = 0u;
  g_cal_window.active_oldest_index = 0u;
  g_cal_window.active_newest_index = 0u;
  g_cal_window.active_elapsed_s = 0.0;
  g_cal_window.active_is_ready = false;
  g_cal_window.active_sum_milli_c = 0.0;
  g_cal_window.active_sum_sq_milli_c = 0.0;
  g_cal_window.active_sum_milli_ohm = 0.0;
  g_cal_window.active_sum_sq_milli_ohm = 0.0;
  g_cal_window.mean_raw_milli_c = 0;
  g_cal_window.stddev_raw_milli_c = 0;
  g_cal_window.mean_raw_milli_ohm = 0;
  g_cal_window.stddev_raw_milli_ohm = 0;

  if (g_cal_window.count == 0u) {
    return;
  }

  const size_t newest_index =
    (g_cal_window.write_index + CAL_WINDOW_MAX_SAMPLES - 1u) %
    CAL_WINDOW_MAX_SAMPLES;
  const int64_t newest_time_us = g_cal_window.samples_time_us[newest_index];
  const int64_t min_time_us = newest_time_us -
                              (int64_t)g_cal_window.window_duration_s * 1000000LL;

  size_t oldest_index = g_cal_window.head;
  size_t active_count = g_cal_window.count;
  bool dropped_aged_samples = false;
  while (active_count > 1u) {
    const int64_t oldest_time_us = g_cal_window.samples_time_us[oldest_index];
    if (oldest_time_us >= min_time_us) {
      break;
    }
    dropped_aged_samples = true;
    oldest_index = (oldest_index + 1u) % CAL_WINDOW_MAX_SAMPLES;
    --active_count;
  }

  g_cal_window.active_count = active_count;
  g_cal_window.active_oldest_index = oldest_index;
  g_cal_window.active_newest_index =
    (oldest_index + active_count - 1u) % CAL_WINDOW_MAX_SAMPLES;

  for (size_t i = 0; i < active_count; ++i) {
    const size_t idx = (oldest_index + i) % CAL_WINDOW_MAX_SAMPLES;
    const double raw_milli_c = (double)g_cal_window.samples_milli_c[idx];
    const double raw_milli_ohm = (double)g_cal_window.samples_milli_ohm[idx];
    g_cal_window.active_sum_milli_c += raw_milli_c;
    g_cal_window.active_sum_sq_milli_c += raw_milli_c * raw_milli_c;
    g_cal_window.active_sum_milli_ohm += raw_milli_ohm;
    g_cal_window.active_sum_sq_milli_ohm += raw_milli_ohm * raw_milli_ohm;
  }

  if (active_count > 0u) {
    const double inv_count = 1.0 / (double)active_count;
    const double mean_milli_c = g_cal_window.active_sum_milli_c * inv_count;
    const double variance_milli_c = fmax(
      0.0,
      (g_cal_window.active_sum_sq_milli_c * inv_count) -
        (mean_milli_c * mean_milli_c));
    g_cal_window.mean_raw_milli_c = (int32_t)llround(mean_milli_c);
    g_cal_window.stddev_raw_milli_c = (int32_t)llround(sqrt(variance_milli_c));

    const double mean_milli_ohm = g_cal_window.active_sum_milli_ohm * inv_count;
    const double variance_milli_ohm = fmax(
      0.0,
      (g_cal_window.active_sum_sq_milli_ohm * inv_count) -
        (mean_milli_ohm * mean_milli_ohm));
    g_cal_window.mean_raw_milli_ohm = (int32_t)llround(mean_milli_ohm);
    g_cal_window.stddev_raw_milli_ohm =
      (int32_t)llround(sqrt(variance_milli_ohm));
  }

  if (active_count > 1u) {
    const int64_t oldest_time_us = g_cal_window.samples_time_us[oldest_index];
    const int64_t active_newest_time_us =
      g_cal_window.samples_time_us[g_cal_window.active_newest_index];
    if (active_newest_time_us > oldest_time_us) {
      g_cal_window.active_elapsed_s =
        (double)(active_newest_time_us - oldest_time_us) / 1000000.0;
    }

    const bool matured_by_boundary = dropped_aged_samples ||
                                     (oldest_time_us <= min_time_us);
    g_cal_window.active_is_ready = (active_count >= 3u) && matured_by_boundary;
  }
}

static void
ResetCalibrationTrendEma(void)
{
  g_cal_window.trend_ema_initialized = false;
  g_cal_window.trend_ema_delta_c = 0.0;
  g_cal_window.trend_ema_drift_c_per_min = 0.0;
}

static void
UpdateCalibrationTrendEma(double delta_c_raw, double drift_c_per_min_raw)
{
  const double alpha = g_cal_window.trend_ema_alpha_permille / 1000.0;
  if (!g_cal_window.trend_ema_initialized) {
    g_cal_window.trend_ema_delta_c = delta_c_raw;
    g_cal_window.trend_ema_drift_c_per_min = drift_c_per_min_raw;
    g_cal_window.trend_ema_initialized = true;
    return;
  }

  g_cal_window.trend_ema_delta_c =
    (alpha * delta_c_raw) + ((1.0 - alpha) * g_cal_window.trend_ema_delta_c);
  g_cal_window.trend_ema_drift_c_per_min =
    (alpha * drift_c_per_min_raw) +
    ((1.0 - alpha) * g_cal_window.trend_ema_drift_c_per_min);
}

/**
 * @brief Execute HasDuplicateRawValues.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @return Return the function result.
 */
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

/**
 * @brief Execute FitLeastSquaresPolynomial.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @param degree Parameter degree.
 * @param model_out Parameter model_out.
 * @return Return the function result.
 */
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

/**
 * @brief Execute ComputeDiagnostics.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @param model Parameter model.
 * @param diagnostics_out Parameter diagnostics_out.
 * @return Return the function result.
 */
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
    const double predicted_c =
      CalibrationModelEvaluateWithPoints(model, raw_c, points, num_points);
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

/**
 * @brief Execute IsSlopeReasonable.
 * @param options Parameter options.
 * @param model Parameter model.
 * @return Return the function result.
 */
static bool
IsSlopeReasonable(const calibration_fit_options_t* options,
                  const calibration_model_t* model)
{
  if (model->mode == CAL_FIT_MODE_PIECEWISE) {
    return true;
  }
  if (options->allow_wide_slope || model->degree < 1) {
    return true;
  }
  const double slope = model->coefficients[1];
  return slope >= options->min_slope && slope <= options->max_slope;
}

/**
 * @brief Execute IsCorrectionReasonable.
 * @param options Parameter options.
 * @param model Parameter model.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @param diagnostics_out Parameter diagnostics_out.
 * @return Return the function result.
 */
static bool
IsCorrectionReasonable(const calibration_fit_options_t* options,
                       const calibration_model_t* model,
                       const calibration_point_t* points,
                       size_t num_points,
                       calibration_fit_diagnostics_t* diagnostics_out)
{
  if (options->guard_min_c >= options->guard_max_c) {
    return true;
  }
  const double raw_min = options->guard_min_c;
  const double raw_max = options->guard_max_c;
  const double predicted_min =
    CalibrationModelEvaluateWithPoints(model, raw_min, points, num_points);
  const double predicted_max =
    CalibrationModelEvaluateWithPoints(model, raw_max, points, num_points);
  const double correction_min = predicted_min - raw_min;
  const double correction_max = predicted_max - raw_max;
  const double max_abs_correction =
    fmax(fabs(correction_min), fabs(correction_max));
  if (diagnostics_out != NULL) {
    diagnostics_out->max_abs_correction_c = max_abs_correction;
  }
  return max_abs_correction <= options->max_abs_correction_c;
}

/**
 * @brief Execute CalibrationModelInitIdentity.
 * @param model Parameter model.
 */
void
CalibrationModelInitIdentity(calibration_model_t* model)
{
  if (model == NULL) {
    return;
  }
  model->mode = CAL_FIT_MODE_LINEAR;
  model->degree = 1;
  model->coefficients[0] = 0.0;
  model->coefficients[1] = 1.0;
  model->coefficients[2] = 0.0;
  model->coefficients[3] = 0.0;
  model->is_valid = true;
}

/**
 * @brief Execute CalibrationModelEvaluate.
 * @param model Parameter model.
 * @param raw_c Parameter raw_c.
 * @return Return the function result.
 */
double
CalibrationModelEvaluate(const calibration_model_t* model, double raw_c)
{
  if (model == NULL || !model->is_valid) {
    return raw_c;
  }
  if (model->mode == CAL_FIT_MODE_PIECEWISE) {
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

/**
 * @brief Execute CalibrationModelEvaluateWithPoints.
 * @param model Parameter model.
 * @param raw_c Parameter raw_c.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @return Return the function result.
 */
double
CalibrationModelEvaluateWithPoints(const calibration_model_t* model,
                                   double raw_c,
                                   const calibration_point_t* points,
                                   size_t num_points)
{
  if (model == NULL || !model->is_valid) {
    return raw_c;
  }
  if (model->mode != CAL_FIT_MODE_PIECEWISE) {
    return CalibrationModelEvaluate(model, raw_c);
  }
  const double residual = InterpolateResidual(points, num_points, raw_c);
  return raw_c + residual;
}

/**
 * @brief Execute CalibrationModelFitFromPoints.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @param model_out Parameter model_out.
 * @return Return the function result.
 */
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

/**
 * @brief Execute CalibrationFitOptionsInitDefault.
 * @param options Parameter options.
 */
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

/**
 * @brief Execute CalibrationModelFitFromPointsWithOptions.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @param options Parameter options.
 * @param model_out Parameter model_out.
 * @param diagnostics_out Parameter diagnostics_out.
 * @return Return the function result.
 */
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
    model_out->mode = options->mode;
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
      CalibrationModelInitIdentity(model_out);
      model_out->mode = CAL_FIT_MODE_PIECEWISE;
      model_out->degree = 1;
      model_out->is_valid = true;
      ComputeDiagnostics(points, num_points, model_out, diagnostics_out);
      if (!IsCorrectionReasonable(
            options, model_out, points, num_points, diagnostics_out)) {
        ESP_LOGW(kTag,
                 "correction exceeds max abs %.2fC within [%.1f, %.1f]",
                 options->max_abs_correction_c,
                 options->guard_min_c,
                 options->guard_max_c);
        return ESP_ERR_INVALID_STATE;
      }
      return ESP_OK;
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
  model_out->mode = options->mode;

  ComputeDiagnostics(points, num_points, model_out, diagnostics_out);

  if (!IsSlopeReasonable(options, model_out)) {
    ESP_LOGW(kTag,
             "slope out of bounds (%.6f not in [%.3f, %.3f])",
             model_out->coefficients[1],
             options->min_slope,
             options->max_slope);
    return ESP_ERR_INVALID_STATE;
  }

  if (!IsCorrectionReasonable(
        options, model_out, points, num_points, diagnostics_out)) {
    ESP_LOGW(kTag,
             "correction exceeds max abs %.2fC within [%.1f, %.1f]",
             options->max_abs_correction_c,
             options->guard_min_c,
             options->guard_max_c);
    return ESP_ERR_INVALID_STATE;
  }

  return ESP_OK;
}

/**
 * @brief Execute CalWindowPushRawSample.
 * @param raw_milli_c Parameter raw_milli_c.
 */
void
CalWindowPushRawSample(int32_t raw_milli_c, int32_t raw_milli_ohm)
{
  if (!CalWindowEnsureStorage_()) {
    return;
  }
  const int64_t now_us = esp_timer_get_time();
  const size_t write_index = g_cal_window.write_index;
  const bool buffer_full = (g_cal_window.count == CAL_WINDOW_MAX_SAMPLES);
  const size_t overwritten_index = g_cal_window.head;
  const int32_t overwritten_milli_c =
    buffer_full ? g_cal_window.samples_milli_c[overwritten_index] : 0;
  const int32_t overwritten_milli_ohm =
    buffer_full ? g_cal_window.samples_milli_ohm[overwritten_index] : 0;

  g_cal_window.samples_milli_c[write_index] = raw_milli_c;
  g_cal_window.samples_milli_ohm[write_index] = raw_milli_ohm;
  g_cal_window.samples_time_us[write_index] = now_us;
  g_cal_window.write_index =
    (g_cal_window.write_index + 1u) % CAL_WINDOW_MAX_SAMPLES;
  if (!buffer_full) {
    g_cal_window.count++;
  } else {
    g_cal_window.head = (g_cal_window.head + 1u) % CAL_WINDOW_MAX_SAMPLES;
  }

  g_cal_window.last_raw_milli_c = raw_milli_c;
  g_cal_window.last_raw_milli_ohm = raw_milli_ohm;

  // Keep all calibration-window math in fixed in-module state (no heap, no
  // extra stack arrays) to preserve deterministic memory use and reduce CPU
  // churn in this timing-sensitive Wi-Fi/mesh runtime path.
  if (buffer_full && g_cal_window.active_count > 0u &&
      overwritten_index == g_cal_window.active_oldest_index) {
    const double old_milli_c = (double)overwritten_milli_c;
    const double old_milli_ohm = (double)overwritten_milli_ohm;
    g_cal_window.active_sum_milli_c -= old_milli_c;
    g_cal_window.active_sum_sq_milli_c -= old_milli_c * old_milli_c;
    g_cal_window.active_sum_milli_ohm -= old_milli_ohm;
    g_cal_window.active_sum_sq_milli_ohm -= old_milli_ohm * old_milli_ohm;
    g_cal_window.active_oldest_index =
      (g_cal_window.active_oldest_index + 1u) % CAL_WINDOW_MAX_SAMPLES;
    --g_cal_window.active_count;
  }

  const double new_milli_c = (double)raw_milli_c;
  const double new_milli_ohm = (double)raw_milli_ohm;
  if (g_cal_window.active_count == 0u) {
    g_cal_window.active_oldest_index = write_index;
    g_cal_window.active_sum_milli_c = 0.0;
    g_cal_window.active_sum_sq_milli_c = 0.0;
    g_cal_window.active_sum_milli_ohm = 0.0;
    g_cal_window.active_sum_sq_milli_ohm = 0.0;
  }
  g_cal_window.active_sum_milli_c += new_milli_c;
  g_cal_window.active_sum_sq_milli_c += new_milli_c * new_milli_c;
  g_cal_window.active_sum_milli_ohm += new_milli_ohm;
  g_cal_window.active_sum_sq_milli_ohm += new_milli_ohm * new_milli_ohm;
  ++g_cal_window.active_count;
  g_cal_window.active_newest_index = write_index;

  const int64_t min_time_us =
    now_us - (int64_t)g_cal_window.window_duration_s * 1000000LL;
  bool dropped_aged_samples = false;
  while (g_cal_window.active_count > 1u) {
    const size_t oldest_index = g_cal_window.active_oldest_index;
    const int64_t oldest_time_us = g_cal_window.samples_time_us[oldest_index];
    if (oldest_time_us >= min_time_us) {
      break;
    }
    dropped_aged_samples = true;
    const double old_milli_c = (double)g_cal_window.samples_milli_c[oldest_index];
    const double old_milli_ohm =
      (double)g_cal_window.samples_milli_ohm[oldest_index];
    g_cal_window.active_sum_milli_c -= old_milli_c;
    g_cal_window.active_sum_sq_milli_c -= old_milli_c * old_milli_c;
    g_cal_window.active_sum_milli_ohm -= old_milli_ohm;
    g_cal_window.active_sum_sq_milli_ohm -= old_milli_ohm * old_milli_ohm;
    g_cal_window.active_oldest_index =
      (g_cal_window.active_oldest_index + 1u) % CAL_WINDOW_MAX_SAMPLES;
    --g_cal_window.active_count;
  }

  if (g_cal_window.active_count == 0u) {
    g_cal_window.active_elapsed_s = 0.0;
    g_cal_window.active_is_ready = false;
    g_cal_window.mean_raw_milli_c = 0;
    g_cal_window.stddev_raw_milli_c = 0;
    g_cal_window.mean_raw_milli_ohm = 0;
    g_cal_window.stddev_raw_milli_ohm = 0;
    return;
  }

  const double inv_count = 1.0 / (double)g_cal_window.active_count;
  const double mean_milli_c = g_cal_window.active_sum_milli_c * inv_count;
  const double variance_milli_c = fmax(
    0.0,
    (g_cal_window.active_sum_sq_milli_c * inv_count) -
      (mean_milli_c * mean_milli_c));
  g_cal_window.mean_raw_milli_c = (int32_t)llround(mean_milli_c);
  g_cal_window.stddev_raw_milli_c = (int32_t)llround(sqrt(variance_milli_c));

  const double mean_milli_ohm = g_cal_window.active_sum_milli_ohm * inv_count;
  const double variance_milli_ohm = fmax(
    0.0,
    (g_cal_window.active_sum_sq_milli_ohm * inv_count) -
      (mean_milli_ohm * mean_milli_ohm));
  g_cal_window.mean_raw_milli_ohm = (int32_t)llround(mean_milli_ohm);
  g_cal_window.stddev_raw_milli_ohm = (int32_t)llround(sqrt(variance_milli_ohm));

  g_cal_window.active_elapsed_s = 0.0;
  g_cal_window.active_is_ready = false;
  if (g_cal_window.active_count > 1u) {
    const int64_t oldest_time_us =
      g_cal_window.samples_time_us[g_cal_window.active_oldest_index];
    if (now_us > oldest_time_us) {
      g_cal_window.active_elapsed_s = (double)(now_us - oldest_time_us) / 1000000.0;
    }
    const bool matured_by_boundary = dropped_aged_samples ||
                                     (oldest_time_us <= min_time_us);
    g_cal_window.active_is_ready =
      (g_cal_window.active_count >= 3u) && matured_by_boundary;
  }

  int32_t delta_raw_mC = 0;
  double drift_raw_c_per_min = 0.0;
  CalWindowGetTrendStats(NULL,
                         NULL,
                         &delta_raw_mC,
                         NULL,
                         &drift_raw_c_per_min,
                         NULL);
  UpdateCalibrationTrendEma(delta_raw_mC / 1000.0, drift_raw_c_per_min);
}

/**
 * @brief Execute CalWindowClear.
 */
void
CalWindowClear(void)
{
  if (!CalWindowEnsureStorage_()) {
    return;
  }
  int32_t* samples_milli_c = g_cal_window.samples_milli_c;
  int32_t* samples_milli_ohm = g_cal_window.samples_milli_ohm;
  int64_t* samples_time_us = g_cal_window.samples_time_us;
  memset(&g_cal_window, 0, sizeof(g_cal_window));
  g_cal_window.samples_milli_c = samples_milli_c;
  g_cal_window.samples_milli_ohm = samples_milli_ohm;
  g_cal_window.samples_time_us = samples_time_us;
  memset(g_cal_window.samples_milli_c,
         0,
         sizeof(int32_t) * (size_t)CAL_WINDOW_MAX_SAMPLES);
  memset(g_cal_window.samples_milli_ohm,
         0,
         sizeof(int32_t) * (size_t)CAL_WINDOW_MAX_SAMPLES);
  memset(g_cal_window.samples_time_us,
         0,
         sizeof(int64_t) * (size_t)CAL_WINDOW_MAX_SAMPLES);
  g_cal_window.window_duration_s = CAL_WINDOW_DURATION_DEFAULT_S;
  g_cal_window.trend_ema_alpha_permille = CAL_TREND_EMA_ALPHA_DEFAULT_PERMILLE;
  RebuildCalibrationWindowState_();
  ResetCalibrationTrendEma();
}

/**
 * @brief Execute CalWindowIsReady.
 * @return Return the function result.
 */
bool
CalWindowIsReady(void)
{
  return ResolveActiveWindowInfo_().is_ready;
}

/**
 * @brief Execute CalWindowGetSampleCount.
 * @return Return the function result.
 */
size_t
CalWindowGetSampleCount(void)
{
  return ResolveActiveWindowInfo_().active_count;
}

/**
 * @brief Execute CalWindowGetStats.
 * @param out_last_raw_mC Parameter out_last_raw_mC.
 * @param out_mean_raw_mC Parameter out_mean_raw_mC.
 * @param out_stddev_mC Parameter out_stddev_mC.
 */
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

/**
 * @brief Read latest/mean/stddev resistance statistics from calibration window.
 * @param out_last_raw_mOhm Receives the most recent resistance sample.
 * @param out_mean_raw_mOhm Receives the mean resistance sample.
 * @param out_stddev_mOhm Receives resistance sample standard deviation.
 */
void
CalWindowGetResistanceStats(int32_t* out_last_raw_mOhm,
                            int32_t* out_mean_raw_mOhm,
                            int32_t* out_stddev_mOhm)
{
  if (out_last_raw_mOhm != NULL) {
    *out_last_raw_mOhm = g_cal_window.last_raw_milli_ohm;
  }
  if (out_mean_raw_mOhm != NULL) {
    *out_mean_raw_mOhm = g_cal_window.mean_raw_milli_ohm;
  }
  if (out_stddev_mOhm != NULL) {
    *out_stddev_mOhm = g_cal_window.stddev_raw_milli_ohm;
  }
}

void
CalWindowGetTrendStats(int32_t* out_begin_mean_raw_mC,
                       int32_t* out_end_mean_raw_mC,
                       int32_t* out_delta_raw_mC,
                       double* out_elapsed_s,
                       double* out_drift_c_per_min,
                       double* out_abs_drift_c_per_min)
{
  const calibration_active_window_info_t active_window = ResolveActiveWindowInfo_();
  const size_t count = active_window.active_count;
  const size_t oldest_index = active_window.oldest_index;
  const size_t segment_count = (count / 4u >= 3u) ? (count / 4u) : 3u;
  if (count < segment_count || segment_count == 0u) {
    if (out_begin_mean_raw_mC != NULL) {
      *out_begin_mean_raw_mC = 0;
    }
    if (out_end_mean_raw_mC != NULL) {
      *out_end_mean_raw_mC = 0;
    }
    if (out_delta_raw_mC != NULL) {
      *out_delta_raw_mC = 0;
    }
    if (out_elapsed_s != NULL) {
      *out_elapsed_s = 0.0;
    }
    if (out_drift_c_per_min != NULL) {
      *out_drift_c_per_min = 0.0;
    }
    if (out_abs_drift_c_per_min != NULL) {
      *out_abs_drift_c_per_min = 0.0;
    }
    return;
  }

  double begin_mean_mC = 0.0;
  double end_mean_mC = 0.0;
  double drift_c_per_min = 0.0;
  ComputeCalibrationWindowTrendStats_(count,
                                      oldest_index,
                                      segment_count,
                                      &begin_mean_mC,
                                      &end_mean_mC,
                                      &drift_c_per_min);
  const double delta_c =
    ComputeCalibrationWindowDeltaCRaw(begin_mean_mC, end_mean_mC);
  const double delta_mC = delta_c * 1000.0;

  if (out_begin_mean_raw_mC != NULL) {
    *out_begin_mean_raw_mC = (int32_t)llround(begin_mean_mC);
  }
  if (out_end_mean_raw_mC != NULL) {
    *out_end_mean_raw_mC = (int32_t)llround(end_mean_mC);
  }
  if (out_delta_raw_mC != NULL) {
    *out_delta_raw_mC = (int32_t)llround(delta_mC);
  }
  if (out_elapsed_s != NULL) {
    *out_elapsed_s = active_window.elapsed_s;
  }
  if (out_drift_c_per_min != NULL) {
    *out_drift_c_per_min = drift_c_per_min;
  }
  if (out_abs_drift_c_per_min != NULL) {
    *out_abs_drift_c_per_min = fabs(drift_c_per_min);
  }
}

bool
CalWindowGetActiveSampleByIndex(size_t index_from_oldest,
                                int32_t* out_raw_mC,
                                int32_t* out_raw_mOhm,
                                int64_t* out_time_us)
{
  const calibration_active_window_info_t active_window = ResolveActiveWindowInfo_();
  if (index_from_oldest >= active_window.active_count) {
    return false;
  }

  const size_t index =
    (active_window.oldest_index + index_from_oldest) % CAL_WINDOW_MAX_SAMPLES;
  if (out_raw_mC != NULL) {
    *out_raw_mC = g_cal_window.samples_milli_c[index];
  }
  if (out_raw_mOhm != NULL) {
    *out_raw_mOhm = g_cal_window.samples_milli_ohm[index];
  }
  if (out_time_us != NULL) {
    *out_time_us = g_cal_window.samples_time_us[index];
  }
  return true;
}

static void
ComputeCalibrationWindowTrendStats_(size_t count,
                                    size_t oldest_index,
                                    size_t segment_count,
                                    double* out_begin_mean_mC,
                                    double* out_end_mean_mC,
                                    double* out_drift_c_per_min)
{
  if (out_begin_mean_mC != NULL) {
    *out_begin_mean_mC = 0.0;
  }
  if (out_end_mean_mC != NULL) {
    *out_end_mean_mC = 0.0;
  }
  if (out_drift_c_per_min != NULL) {
    *out_drift_c_per_min = 0.0;
  }

  if (count < segment_count || segment_count == 0u) {
    return;
  }

  double begin_sum_mC = 0.0;
  double end_sum_mC = 0.0;
  double sum_t_s = 0.0;
  double sum_y_c = 0.0;
  double sum_t2 = 0.0;
  double sum_ty = 0.0;
  const int64_t first_time_us = g_cal_window.samples_time_us[oldest_index];
  const size_t end_segment_start = count - segment_count;

  for (size_t i = 0; i < count; ++i) {
    const size_t idx = (oldest_index + i) % CAL_WINDOW_MAX_SAMPLES;
    const double raw_mC = (double)g_cal_window.samples_milli_c[idx];
    const double y_c = raw_mC / 1000.0;
    const double t_s =
      (double)(g_cal_window.samples_time_us[idx] - first_time_us) / 1000000.0;

    if (i < segment_count) {
      begin_sum_mC += raw_mC;
    }
    if (i >= end_segment_start) {
      end_sum_mC += raw_mC;
    }

    sum_t_s += t_s;
    sum_y_c += y_c;
    sum_t2 += t_s * t_s;
    sum_ty += t_s * y_c;
  }

  if (out_begin_mean_mC != NULL) {
    *out_begin_mean_mC = begin_sum_mC / (double)segment_count;
  }
  if (out_end_mean_mC != NULL) {
    *out_end_mean_mC = end_sum_mC / (double)segment_count;
  }

  if (out_drift_c_per_min != NULL && count >= 3u) {
    const double n = (double)count;
    const double denominator = (n * sum_t2) - (sum_t_s * sum_t_s);
    if (denominator > 0.0) {
      const double slope_c_per_s =
        ((n * sum_ty) - (sum_t_s * sum_y_c)) / denominator;
      if (isfinite(slope_c_per_s)) {
        *out_drift_c_per_min = slope_c_per_s * 60.0;
      }
    }
  }
}

static double
ComputeCalibrationWindowDeltaCRaw(double begin_mean_mC, double end_mean_mC)
{
  return (end_mean_mC - begin_mean_mC) / 1000.0;
}

void
CalWindowSetDurationSeconds(uint16_t window_s)
{
  if (window_s < CAL_WINDOW_DURATION_MIN_S ||
      window_s > CAL_WINDOW_DURATION_MAX_S) {
    return;
  }
  g_cal_window.window_duration_s = window_s;
  RebuildCalibrationWindowState_();
}

uint16_t
CalWindowGetDurationSeconds(void)
{
  return g_cal_window.window_duration_s;
}

void
CalWindowSetTrendEmaAlphaPermille(uint16_t alpha_permille)
{
  if (alpha_permille == 0u || alpha_permille > 1000u) {
    return;
  }
  g_cal_window.trend_ema_alpha_permille = alpha_permille;
}

uint16_t
CalWindowGetTrendEmaAlphaPermille(void)
{
  return g_cal_window.trend_ema_alpha_permille;
}

void
CalWindowResetTrendEma(void)
{
  ResetCalibrationTrendEma();
}

void
CalWindowGetTrendEmaStats(double* out_delta_c_ema,
                          double* out_drift_c_per_min_ema,
                          bool* out_initialized)
{
  if (out_delta_c_ema != NULL) {
    *out_delta_c_ema = g_cal_window.trend_ema_delta_c;
  }
  if (out_drift_c_per_min_ema != NULL) {
    *out_drift_c_per_min_ema = g_cal_window.trend_ema_drift_c_per_min;
  }
  if (out_initialized != NULL) {
    *out_initialized = g_cal_window.trend_ema_initialized;
  }
}

void
CalWindowGetStorageLayout(cal_window_storage_layout_t* out_layout)
{
  if (out_layout == NULL) {
    return;
  }
  (void)CalWindowEnsureStorage_();
  out_layout->samples_milli_c_bytes =
    sizeof(int32_t) * (size_t)CAL_WINDOW_MAX_SAMPLES;
  out_layout->samples_milli_ohm_bytes =
    sizeof(int32_t) * (size_t)CAL_WINDOW_MAX_SAMPLES;
  out_layout->samples_time_us_bytes =
    sizeof(int64_t) * (size_t)CAL_WINDOW_MAX_SAMPLES;
  out_layout->samples_milli_c_in_psram = g_cal_window_samples_milli_c_in_psram;
  out_layout->samples_milli_ohm_in_psram =
    g_cal_window_samples_milli_ohm_in_psram;
  out_layout->samples_time_us_in_psram = g_cal_window_samples_time_us_in_psram;
}
