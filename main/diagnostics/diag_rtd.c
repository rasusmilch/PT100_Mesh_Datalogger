#include "diagnostics/diag_rtd.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "max31865_reader.h"

static const uint8_t kRegConfig = 0x00;
static const uint8_t kRegFaultStatus = 0x07;
static const uint8_t kCfg3Wire = 0x10;
static const uint8_t kCfgFilter50Hz = 0x01;
static const uint8_t kCfgFaultStatusClear = 0x02;

static uint8_t
BuildBaseConfig(const max31865_reader_t* reader)
{
  uint8_t cfg = 0;
  if (reader->wires == 3) {
    cfg |= kCfg3Wire;
  }
  if (reader->filter_hz <= 50) {
    cfg |= kCfgFilter50Hz;
  }
  return cfg;
}

int
RunDiagRtd(const app_runtime_t* runtime,
           bool full,
           int samples,
           int delay_ms,
           diag_verbosity_t verbosity)
{
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "RTD", verbosity);
  const int total_steps = full ? 5 : 2;

  if (runtime == NULL || runtime->sensor == NULL) {
    DiagReportStep(&ctx,
                   1,
                   total_steps,
                   "runtime",
                   ESP_ERR_INVALID_STATE,
                   "runtime or sensor not available");
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  const max31865_reader_t* reader = runtime->sensor;

  char warning[80] = { 0 };
  if (reader->wires == 2) {
    snprintf(warning,
             sizeof(warning),
             "WARNING: 2-wire lead resistance adds large error");
  } else if (reader->wires == 3) {
    snprintf(warning,
             sizeof(warning),
             "NOTE: 3-wire assumes matched lead resistance");
  }
  const char* conversion =
    (reader->conversion == kMax31865ConversionCvdIterative) ? "CVD_ITERATIVE"
                                                            : "TABLE_PT100";

  uint8_t config_reg = 0;
  esp_err_t reg_result = Max31865ReadReg(runtime->sensor, kRegConfig, &config_reg);
  DiagReportStep(&ctx,
                 1,
                 total_steps,
                 "spi probe",
                 reg_result,
                 "cfg=0x%02x wires=%u filter=%uHz conv=%s vbias_settle_ms=%u rref=%.2f r0=%.2f %s",
                 (unsigned)config_reg,
                 (unsigned)reader->wires,
                 (unsigned)reader->filter_hz,
                 conversion,
                 (unsigned)reader->bias_settle_ms,
                 reader->rref_ohm,
                 reader->rtd_nominal_ohm,
                 warning);
  if (warning[0] != '\0' && verbosity == kDiagVerbosity0) {
    printf("[RTD] %s\n", warning);
  }

  max31865_sample_t sample;
  memset(&sample, 0, sizeof(sample));
  esp_err_t sample_result = Max31865ReadOnce(runtime->sensor, &sample);
  char fault_desc[80] = { 0 };
  Max31865FormatFault(sample.fault_status, fault_desc, sizeof(fault_desc));

  DiagReportStep(&ctx,
                 2,
                 total_steps,
                 "single sample",
                 (sample_result == ESP_OK && !sample.fault_present) ? ESP_OK
                                                                    : ESP_ERR_INVALID_RESPONSE,
                 "adc=%u r_ohm=%.3f temp_c=%.3f fault=%s",
                 (unsigned)sample.adc_code,
                 sample.resistance_ohm,
                 sample.temperature_c,
                 fault_desc);
  if (!full) {
    DiagPrintSummary(&ctx, total_steps);
    return (ctx.steps_failed == 0) ? 0 : 1;
  }

  const uint8_t base_config = BuildBaseConfig(reader);
  uint8_t fault_before = 0;
  uint8_t fault_after = 0;
  esp_err_t fault_result = Max31865ReadReg(runtime->sensor,
                                           kRegFaultStatus,
                                           &fault_before);
  if (fault_result == ESP_OK) {
    fault_result = Max31865WriteReg(runtime->sensor,
                                    kRegConfig,
                                    (uint8_t)(base_config | kCfgFaultStatusClear));
  }
  if (fault_result == ESP_OK) {
    fault_result = Max31865ReadReg(runtime->sensor, kRegFaultStatus, &fault_after);
  }
  char fault_before_desc[64] = { 0 };
  char fault_after_desc[64] = { 0 };
  Max31865FormatFault(fault_before, fault_before_desc, sizeof(fault_before_desc));
  Max31865FormatFault(fault_after, fault_after_desc, sizeof(fault_after_desc));
  DiagReportStep(&ctx,
                 3,
                 total_steps,
                 "fault clear",
                 fault_result,
                 "before=%s (0x%02x) after=%s (0x%02x)",
                 fault_before_desc,
                 (unsigned)fault_before,
                 fault_after_desc,
                 (unsigned)fault_after);

  const int requested_samples = (samples > 0) ? samples : 5;
  const int delay_ms_effective = (delay_ms >= 0) ? delay_ms : 20;
  max31865_sample_t averaged;
  max31865_sampling_stats_t stats;
  memset(&averaged, 0, sizeof(averaged));
  memset(&stats, 0, sizeof(stats));
  esp_err_t avg_result = Max31865ReadAveraged(runtime->sensor,
                                              requested_samples,
                                              delay_ms_effective,
                                              &averaged,
                                              &stats);

  if (avg_result == ESP_OK) {
    DiagReportStep(&ctx,
                   4,
                   total_steps,
                   "multi-sample",
                   avg_result,
                   "req=%d valid=%d faulted=%d avg_temp=%.3fC avg_r=%.3fΩ stddev=%.4fC min=%.3fC max=%.3fC",
                   requested_samples,
                   stats.valid_samples,
                   stats.faulted_samples,
                   averaged.temperature_c,
                   averaged.resistance_ohm,
                   stats.stddev_temp_c,
                   stats.min_temp_c,
                   stats.max_temp_c);
  } else {
    DiagReportStep(&ctx,
                   4,
                   total_steps,
                   "multi-sample",
                   avg_result,
                   "req=%d valid=%d faulted=%d (averaging failed)",
                   requested_samples,
                   stats.valid_samples,
                   stats.faulted_samples);
  }

  max31865_sample_t ema_sample;
  double ema_temp = 0.0;
  memset(&ema_sample, 0, sizeof(ema_sample));
  esp_err_t ema_result =
    Max31865ReadEmaUpdate(runtime->sensor, 0.2, &ema_sample, &ema_temp);
  char ema_fault[64] = { 0 };
  Max31865FormatFault(ema_sample.fault_status, ema_fault, sizeof(ema_fault));

  DiagReportStep(&ctx,
                 5,
                 total_steps,
                 "ema update",
                 ema_result,
                 "sample_temp=%.3fC ema_temp=%.3fC fault=%s",
                 ema_sample.temperature_c,
                 ema_temp,
                 ema_fault);

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
