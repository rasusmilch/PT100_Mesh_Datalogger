#include "calibration.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_timer.h"
#include "max31865_reader.h"

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
  int32_t trend_begin_mean_raw_milli_c;
  int32_t trend_end_mean_raw_milli_c;
  int32_t trend_delta_raw_milli_c;
  double trend_drift_c_per_min;
  double trend_abs_drift_c_per_min;
  bool trend_stats_valid;
  double last_calibrated_temp_c;
  double mean_calibrated_temp_c;
  double stddev_calibrated_temp_c;
  bool calibrated_temp_stats_valid;
  size_t calibrated_temp_stats_sample_count;
  uint32_t calibrated_temp_stats_generation;
  uint32_t active_window_generation;
} cal_window_state_t;

typedef struct
{
  size_t active_count;
  size_t oldest_index;
  size_t newest_index;
  double elapsed_s;
  bool is_ready;
} calibration_active_window_info_t;

static int32_t g_cal_window_samples_milli_c_fallback[CAL_WINDOW_MAX_SAMPLES];
static int32_t g_cal_window_samples_milli_ohm_fallback[CAL_WINDOW_MAX_SAMPLES];
static int64_t g_cal_window_samples_time_us_fallback[CAL_WINDOW_MAX_SAMPLES];

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
static void InvalidateCalibratedTempStats_(void);
static void RecomputeCachedTrendStats_(void);
static void UpdateCalibrationTrendEma(double delta_c_raw,
                                      double drift_c_per_min_raw);
static void RebuildCalibrationWindowState_(void);
static calibration_active_window_info_t ResolveActiveWindowInfo_(void);
static bool CalWindowEnsureStorage_(void);
static bool CalWindowStorageIsPsram_(const void* ptr);
static void ComputeResidualStats_(double sum,
                                  double sum_abs,
                                  double sum_sq,
                                  double max_abs,
                                  size_t count,
                                  double* mean_signed_out,
                                  double* mean_abs_out,
                                  double* rmse_out,
                                  double* stddev_out,
                                  double* sse_out);
static size_t CalibrationParameterCountForModel_(
  const calibration_model_t* model,
  size_t point_count);
static bool CalibrationCanComputeR2_(const calibration_model_t* model);
static void ComputeR2Metrics_(double sse,
                              double mean_target,
                              double sum_target_sq,
                              size_t point_count,
                              size_t parameter_count,
                              bool* r2_available_out,
                              bool* adjusted_r2_available_out,
                              double* r2_out,
                              double* adjusted_r2_out);

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
  return esp_ptr_external_ram(ptr);
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
  g_cal_window.trend_begin_mean_raw_milli_c = 0;
  g_cal_window.trend_end_mean_raw_milli_c = 0;
  g_cal_window.trend_delta_raw_milli_c = 0;
  g_cal_window.trend_drift_c_per_min = 0.0;
  g_cal_window.trend_abs_drift_c_per_min = 0.0;
  g_cal_window.trend_stats_valid = false;
  InvalidateCalibratedTempStats_();

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
  RecomputeCachedTrendStats_();
  UpdateCalibrationTrendEma(g_cal_window.trend_delta_raw_milli_c / 1000.0,
                            g_cal_window.trend_drift_c_per_min);
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

static void
InvalidateCalibratedTempStats_(void)
{
  g_cal_window.last_calibrated_temp_c = 0.0;
  g_cal_window.mean_calibrated_temp_c = 0.0;
  g_cal_window.stddev_calibrated_temp_c = 0.0;
  g_cal_window.calibrated_temp_stats_valid = false;
  g_cal_window.calibrated_temp_stats_sample_count = 0u;
  g_cal_window.calibrated_temp_stats_generation = 0u;
  ++g_cal_window.active_window_generation;
}

static void
RecomputeCachedTrendStats_(void)
{
  const calibration_active_window_info_t active_window = ResolveActiveWindowInfo_();
  const size_t count = active_window.active_count;
  const size_t oldest_index = active_window.oldest_index;
  const size_t segment_count = (count / 4u >= 3u) ? (count / 4u) : 3u;

  g_cal_window.trend_begin_mean_raw_milli_c = 0;
  g_cal_window.trend_end_mean_raw_milli_c = 0;
  g_cal_window.trend_delta_raw_milli_c = 0;
  g_cal_window.trend_drift_c_per_min = 0.0;
  g_cal_window.trend_abs_drift_c_per_min = 0.0;
  g_cal_window.trend_stats_valid = false;

  if (count < segment_count || segment_count == 0u) {
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
  g_cal_window.trend_begin_mean_raw_milli_c = (int32_t)llround(begin_mean_mC);
  g_cal_window.trend_end_mean_raw_milli_c = (int32_t)llround(end_mean_mC);
  g_cal_window.trend_delta_raw_milli_c = (int32_t)llround(delta_c * 1000.0);
  g_cal_window.trend_drift_c_per_min = drift_c_per_min;
  g_cal_window.trend_abs_drift_c_per_min = fabs(drift_c_per_min);
  g_cal_window.trend_stats_valid = true;
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

static void
ComputeResidualStats_(double sum,
                      double sum_abs,
                      double sum_sq,
                      double max_abs,
                      size_t count,
                      double* mean_signed_out,
                      double* mean_abs_out,
                      double* rmse_out,
                      double* stddev_out,
                      double* sse_out)
{
  if (mean_signed_out != NULL) {
    *mean_signed_out = 0.0;
  }
  if (mean_abs_out != NULL) {
    *mean_abs_out = 0.0;
  }
  if (rmse_out != NULL) {
    *rmse_out = 0.0;
  }
  if (stddev_out != NULL) {
    *stddev_out = 0.0;
  }
  if (sse_out != NULL) {
    *sse_out = 0.0;
  }
  (void)max_abs;
  if (count == 0u) {
    return;
  }
  const double inv_count = 1.0 / (double)count;
  const double mean = sum * inv_count;
  const double variance = fmax(0.0, (sum_sq * inv_count) - (mean * mean));
  if (mean_signed_out != NULL) {
    *mean_signed_out = mean;
  }
  if (mean_abs_out != NULL) {
    *mean_abs_out = sum_abs * inv_count;
  }
  if (rmse_out != NULL) {
    *rmse_out = sqrt(sum_sq * inv_count);
  }
  if (stddev_out != NULL) {
    *stddev_out = sqrt(variance);
  }
  if (sse_out != NULL) {
    *sse_out = sum_sq;
  }
}

static size_t
CalibrationParameterCountForModel_(const calibration_model_t* model,
                                   size_t point_count)
{
  if (model == NULL) {
    return 0u;
  }
  if (model->mode == CAL_FIT_MODE_PIECEWISE) {
    return point_count;
  }
  return (size_t)model->degree + 1u;
}

static bool
CalibrationCanComputeR2_(const calibration_model_t* model)
{
  if (model == NULL) {
    return false;
  }
  return (model->mode == CAL_FIT_MODE_LINEAR || model->mode == CAL_FIT_MODE_POLY);
}

static void
ComputeR2Metrics_(double sse,
                  double mean_target,
                  double sum_target_sq,
                  size_t point_count,
                  size_t parameter_count,
                  bool* r2_available_out,
                  bool* adjusted_r2_available_out,
                  double* r2_out,
                  double* adjusted_r2_out)
{
  if (r2_available_out != NULL) {
    *r2_available_out = false;
  }
  if (adjusted_r2_available_out != NULL) {
    *adjusted_r2_available_out = false;
  }
  if (r2_out != NULL) {
    *r2_out = 0.0;
  }
  if (adjusted_r2_out != NULL) {
    *adjusted_r2_out = 0.0;
  }
  if (point_count == 0u) {
    return;
  }
  const double tss = fmax(0.0, sum_target_sq - ((double)point_count * mean_target * mean_target));
  if (tss <= 1e-18) {
    return;
  }

  const double r2 = 1.0 - (sse / tss);
  if (r2_out != NULL) {
    *r2_out = r2;
  }
  if (r2_available_out != NULL) {
    *r2_available_out = true;
  }

  if (point_count > parameter_count && point_count > 1u) {
    const double numerator = (1.0 - r2) * ((double)point_count - 1.0);
    const double denominator = (double)point_count - (double)parameter_count;
    if (denominator > 0.0) {
      if (adjusted_r2_out != NULL) {
        *adjusted_r2_out = 1.0 - (numerator / denominator);
      }
      if (adjusted_r2_available_out != NULL) {
        *adjusted_r2_available_out = true;
      }
    }
  }
}

esp_err_t
CalibrationBuildFitDomainPoints(const calibration_point_t* stored_points,
                                size_t num_points,
                                calibration_domain_t fit_domain,
                                double rtd_nominal_ohm,
                                calibration_point_t* fit_points_out)
{
  if (stored_points == NULL || fit_points_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (num_points < 1u || num_points > CALIBRATION_MAX_POINTS) {
    return ESP_ERR_INVALID_SIZE;
  }

  for (size_t i = 0; i < num_points; ++i) {
    fit_points_out[i] = stored_points[i];
    if (fit_domain == CAL_DOMAIN_RESISTANCE_OHM) {
      if (stored_points[i].raw_avg_mOhm <= 0) {
        return ESP_ERR_INVALID_ARG;
      }
      const double actual_c = stored_points[i].actual_mC / 1000.0;
      const double target_ohm =
        Max31865TemperatureToResistanceCvd(actual_c, rtd_nominal_ohm);
      if (!isfinite(target_ohm)) {
        return ESP_ERR_INVALID_STATE;
      }
      fit_points_out[i].raw_avg_mC = stored_points[i].raw_avg_mOhm;
      fit_points_out[i].actual_mC = (int32_t)llround(target_ohm * 1000.0);
    }
  }
  return ESP_OK;
}

esp_err_t
CalibrationBuildFitReport(
  const calibration_point_t* stored_points,
  size_t num_points,
  calibration_domain_t fit_domain,
  const calibration_model_t* model,
  double rtd_nominal_ohm,
  calibration_resistance_to_temp_fn_t resistance_to_temp_fn,
  void* resistance_to_temp_context,
  calibration_fit_report_t* report_out)
{
  if (stored_points == NULL || model == NULL || report_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (num_points < 1u || num_points > CALIBRATION_MAX_POINTS) {
    return ESP_ERR_INVALID_SIZE;
  }
  calibration_point_t fit_points[CALIBRATION_MAX_POINTS] = { 0 };
  esp_err_t build_result = CalibrationBuildFitDomainPoints(
    stored_points, num_points, fit_domain, rtd_nominal_ohm, fit_points);
  if (build_result != ESP_OK) {
    return build_result;
  }

  memset(report_out, 0, sizeof(*report_out));
  report_out->point_results_count = num_points;
  calibration_fit_diagnostics_t* summary = &report_out->summary;
  summary->fit_domain = fit_domain;
  summary->fit_mode = model->mode;
  summary->degree = model->degree;
  summary->point_count = num_points;
  summary->parameter_count = CalibrationParameterCountForModel_(model, num_points);
  summary->degrees_of_freedom =
    (int32_t)num_points - (int32_t)summary->parameter_count;
  summary->r_squared_is_meaningful = CalibrationCanComputeR2_(model);

  double temp_sum = 0.0;
  double temp_sum_abs = 0.0;
  double temp_sum_sq = 0.0;
  double temp_max_abs = 0.0;
  double temp_target_sum = 0.0;
  double temp_target_sum_sq = 0.0;

  double res_sum = 0.0;
  double res_sum_abs = 0.0;
  double res_sum_sq = 0.0;
  double res_max_abs = 0.0;
  double res_target_sum = 0.0;
  double res_target_sum_sq = 0.0;
  size_t temp_count = 0u;
  size_t res_count = 0u;

  for (size_t i = 0; i < num_points; ++i) {
    const calibration_point_t* point = &stored_points[i];
    calibration_fit_point_result_t* row = &report_out->point_results[i];
    memset(row, 0, sizeof(*row));
    row->point_index = i + 1u;
    row->fit_domain = fit_domain;
    row->target_temp_c = point->actual_mC / 1000.0;
    row->target_temp_c_available = true;
    row->captured_raw_temp_c = point->raw_avg_mC / 1000.0;
    row->captured_raw_temp_c_available = (point->raw_avg_mC != INT32_MIN);
    row->captured_raw_res_ohm = point->raw_avg_mOhm / 1000.0;
    row->captured_raw_res_ohm_available = (point->raw_avg_mOhm > 0);

    if (fit_domain == CAL_DOMAIN_RESISTANCE_OHM) {
      if (!row->captured_raw_res_ohm_available) {
        continue;
      }
      const double target_res_ohm =
        Max31865TemperatureToResistanceCvd(row->target_temp_c, rtd_nominal_ohm);
      if (!isfinite(target_res_ohm)) {
        continue;
      }
      row->target_res_ohm = target_res_ohm;
      row->target_res_ohm_available = true;

      const double fitted_res_ohm = CalibrationModelEvaluateWithPoints(
        model, row->captured_raw_res_ohm, fit_points, num_points);
      if (!isfinite(fitted_res_ohm)) {
        continue;
      }
      row->fitted_res_ohm = fitted_res_ohm;
      row->fitted_res_ohm_available = true;
      row->res_residual_ohm = row->target_res_ohm - row->fitted_res_ohm;
      row->res_residual_ohm_available = true;
      row->correction_fit_domain = row->fitted_res_ohm - row->captured_raw_res_ohm;
      row->correction_fit_domain_available = true;

      if (resistance_to_temp_fn != NULL) {
        const double fitted_temp_c =
          resistance_to_temp_fn(fitted_res_ohm, resistance_to_temp_context);
        if (isfinite(fitted_temp_c)) {
          row->fitted_temp_c = fitted_temp_c;
          row->fitted_temp_c_available = true;
          row->temp_residual_c = row->target_temp_c - row->fitted_temp_c;
          row->temp_residual_c_available = true;
        }
      }
    } else {
      const double fitted_temp_c = CalibrationModelEvaluateWithPoints(
        model, row->captured_raw_temp_c, fit_points, num_points);
      if (!isfinite(fitted_temp_c)) {
        continue;
      }
      row->fitted_temp_c = fitted_temp_c;
      row->fitted_temp_c_available = true;
      row->temp_residual_c = row->target_temp_c - row->fitted_temp_c;
      row->temp_residual_c_available = true;
      row->correction_fit_domain = row->fitted_temp_c - row->captured_raw_temp_c;
      row->correction_fit_domain_available = row->captured_raw_temp_c_available;
    }

    if (row->temp_residual_c_available) {
      const double residual = row->temp_residual_c;
      temp_sum += residual;
      temp_sum_abs += fabs(residual);
      temp_sum_sq += residual * residual;
      if (fabs(residual) > temp_max_abs) {
        temp_max_abs = fabs(residual);
      }
      temp_target_sum += row->target_temp_c;
      temp_target_sum_sq += row->target_temp_c * row->target_temp_c;
      ++temp_count;
    }
    if (row->res_residual_ohm_available) {
      const double residual = row->res_residual_ohm;
      res_sum += residual;
      res_sum_abs += fabs(residual);
      res_sum_sq += residual * residual;
      if (fabs(residual) > res_max_abs) {
        res_max_abs = fabs(residual);
      }
      res_target_sum += row->target_res_ohm;
      res_target_sum_sq += row->target_res_ohm * row->target_res_ohm;
      ++res_count;
    }
    if (row->correction_fit_domain_available) {
      const double abs_correction = fabs(row->correction_fit_domain);
      if (!summary->max_abs_correction_fit_domain_available ||
          abs_correction > summary->max_abs_correction_fit_domain) {
        summary->max_abs_correction_fit_domain = abs_correction;
        summary->max_abs_correction_fit_domain_available = true;
      }
    }
  }

  summary->mean_signed_residual_c_available = (temp_count > 0u);
  summary->mean_abs_residual_c_available = (temp_count > 0u);
  summary->rmse_c_available = (temp_count > 0u);
  summary->residual_stddev_c_available = (temp_count > 0u);
  summary->max_abs_residual_c_available = (temp_count > 0u);
  summary->sse_c_available = (temp_count > 0u);
  ComputeResidualStats_(temp_sum,
                        temp_sum_abs,
                        temp_sum_sq,
                        temp_max_abs,
                        temp_count,
                        &summary->mean_signed_residual_c,
                        &summary->mean_abs_residual_c,
                        &summary->rmse_c,
                        &summary->residual_stddev_c,
                        &summary->sse_c);
  summary->max_abs_residual_c = (temp_count > 0u) ? temp_max_abs : 0.0;

  if (fit_domain == CAL_DOMAIN_RESISTANCE_OHM) {
    summary->mean_signed_residual_ohm_available = (res_count > 0u);
    summary->mean_abs_residual_ohm_available = (res_count > 0u);
    summary->rmse_ohm_available = (res_count > 0u);
    summary->residual_stddev_ohm_available = (res_count > 0u);
    summary->max_abs_residual_ohm_available = (res_count > 0u);
    summary->sse_ohm_available = (res_count > 0u);
    ComputeResidualStats_(res_sum,
                          res_sum_abs,
                          res_sum_sq,
                          res_max_abs,
                          res_count,
                          &summary->mean_signed_residual_ohm,
                          &summary->mean_abs_residual_ohm,
                          &summary->rmse_ohm,
                          &summary->residual_stddev_ohm,
                          &summary->sse_ohm);
    summary->max_abs_residual_ohm = (res_count > 0u) ? res_max_abs : 0.0;
  }

  if (CalibrationCanComputeR2_(model) && temp_count > 0u) {
    ComputeR2Metrics_(temp_sum_sq,
                      temp_target_sum / (double)temp_count,
                      temp_target_sum_sq,
                      temp_count,
                      summary->parameter_count,
                      &summary->r_squared_available,
                      &summary->adjusted_r_squared_available,
                      &summary->r_squared,
                      &summary->adjusted_r_squared);
  }
  if (summary->max_abs_correction_fit_domain_available) {
    summary->max_abs_correction_c = summary->max_abs_correction_fit_domain;
  }
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
                   calibration_domain_t fit_domain,
                   const calibration_model_t* model,
                   calibration_fit_diagnostics_t* diagnostics_out)
{
  if (diagnostics_out == NULL) {
    return ESP_OK;
  }

  calibration_fit_report_t report;
  memset(&report, 0, sizeof(report));
  esp_err_t report_result =
    CalibrationBuildFitReport(points,
                              num_points,
                              fit_domain,
                              model,
                              100.0,
                              NULL,
                              NULL,
                              &report);
  if (report_result != ESP_OK) {
    return report_result;
  }
  *diagnostics_out = report.summary;
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
 * @brief Execute CalibrationFitApplyCorrectionGuard.
 * @param options Parameter options.
 * @param model Parameter model.
 * @param points Parameter points.
 * @param num_points Parameter num_points.
 * @param diagnostics_out Parameter diagnostics_out.
 * @return Return the function result.
 */
bool
CalibrationFitApplyCorrectionGuard(
  const calibration_fit_options_t* options,
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
    diagnostics_out->max_abs_correction_fit_domain = max_abs_correction;
    diagnostics_out->max_abs_correction_fit_domain_available = true;
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
  calibration_domain_t fit_domain = CAL_DOMAIN_TEMP_C;
  bool all_points_have_resistance = true;
  for (size_t i = 0; i < num_points; ++i) {
    if (points[i].raw_avg_mOhm <= 0) {
      all_points_have_resistance = false;
      break;
    }
  }
  if (all_points_have_resistance) {
    fit_domain = CAL_DOMAIN_RESISTANCE_OHM;
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
      memset(diagnostics_out, 0, sizeof(*diagnostics_out));
      diagnostics_out->fit_domain = fit_domain;
      diagnostics_out->fit_mode = model_out->mode;
      diagnostics_out->degree = model_out->degree;
      diagnostics_out->point_count = num_points;
      diagnostics_out->parameter_count = 1u;
      diagnostics_out->degrees_of_freedom = 0;
      diagnostics_out->mean_signed_residual_c_available = true;
      diagnostics_out->mean_abs_residual_c_available = true;
      diagnostics_out->rmse_c_available = true;
      diagnostics_out->residual_stddev_c_available = true;
      diagnostics_out->max_abs_residual_c_available = true;
      diagnostics_out->sse_c_available = true;
      diagnostics_out->max_abs_correction_fit_domain_available = true;
      diagnostics_out->max_abs_correction_c = fabs(offset);
      diagnostics_out->max_abs_correction_fit_domain = fabs(offset);
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
      ComputeDiagnostics(
        points, num_points, fit_domain, model_out, diagnostics_out);
      if (!CalibrationFitApplyCorrectionGuard(
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

  ComputeDiagnostics(points, num_points, fit_domain, model_out, diagnostics_out);

  if (!IsSlopeReasonable(options, model_out)) {
    ESP_LOGW(kTag,
             "slope out of bounds (%.6f not in [%.3f, %.3f])",
             model_out->coefficients[1],
             options->min_slope,
             options->max_slope);
    return ESP_ERR_INVALID_STATE;
  }

  if (!CalibrationFitApplyCorrectionGuard(
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
    g_cal_window.trend_begin_mean_raw_milli_c = 0;
    g_cal_window.trend_end_mean_raw_milli_c = 0;
    g_cal_window.trend_delta_raw_milli_c = 0;
    g_cal_window.trend_drift_c_per_min = 0.0;
    g_cal_window.trend_abs_drift_c_per_min = 0.0;
    g_cal_window.trend_stats_valid = false;
    InvalidateCalibratedTempStats_();
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
  InvalidateCalibratedTempStats_();
  RecomputeCachedTrendStats_();
  UpdateCalibrationTrendEma(g_cal_window.trend_delta_raw_milli_c / 1000.0,
                            g_cal_window.trend_drift_c_per_min);
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
  if (out_begin_mean_raw_mC != NULL) {
    *out_begin_mean_raw_mC = g_cal_window.trend_begin_mean_raw_milli_c;
  }
  if (out_end_mean_raw_mC != NULL) {
    *out_end_mean_raw_mC = g_cal_window.trend_end_mean_raw_milli_c;
  }
  if (out_delta_raw_mC != NULL) {
    *out_delta_raw_mC = g_cal_window.trend_delta_raw_milli_c;
  }
  if (out_elapsed_s != NULL) {
    *out_elapsed_s = g_cal_window.active_elapsed_s;
  }
  if (out_drift_c_per_min != NULL) {
    *out_drift_c_per_min = g_cal_window.trend_drift_c_per_min;
  }
  if (out_abs_drift_c_per_min != NULL) {
    *out_abs_drift_c_per_min = g_cal_window.trend_abs_drift_c_per_min;
  }
}

void
CalWindowSetCalibratedTempStats(double last_calibrated_temp_c,
                                double mean_calibrated_temp_c,
                                double stddev_calibrated_temp_c,
                                bool valid,
                                size_t sample_count,
                                uint32_t generation)
{
  g_cal_window.last_calibrated_temp_c = last_calibrated_temp_c;
  g_cal_window.mean_calibrated_temp_c = mean_calibrated_temp_c;
  g_cal_window.stddev_calibrated_temp_c = stddev_calibrated_temp_c;
  g_cal_window.calibrated_temp_stats_valid = valid;
  g_cal_window.calibrated_temp_stats_sample_count = sample_count;
  g_cal_window.calibrated_temp_stats_generation = generation;
}

void
CalWindowGetCalibratedTempStats(double* out_last_calibrated_temp_c,
                                double* out_mean_calibrated_temp_c,
                                double* out_stddev_calibrated_temp_c,
                                bool* out_valid,
                                size_t* out_sample_count,
                                uint32_t* out_generation)
{
  if (out_last_calibrated_temp_c != NULL) {
    *out_last_calibrated_temp_c = g_cal_window.last_calibrated_temp_c;
  }
  if (out_mean_calibrated_temp_c != NULL) {
    *out_mean_calibrated_temp_c = g_cal_window.mean_calibrated_temp_c;
  }
  if (out_stddev_calibrated_temp_c != NULL) {
    *out_stddev_calibrated_temp_c = g_cal_window.stddev_calibrated_temp_c;
  }
  if (out_valid != NULL) {
    *out_valid = g_cal_window.calibrated_temp_stats_valid;
  }
  if (out_sample_count != NULL) {
    *out_sample_count = g_cal_window.calibrated_temp_stats_sample_count;
  }
  if (out_generation != NULL) {
    *out_generation = g_cal_window.calibrated_temp_stats_generation;
  }
}

uint32_t
CalWindowGetActiveGeneration(void)
{
  return g_cal_window.active_window_generation;
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
