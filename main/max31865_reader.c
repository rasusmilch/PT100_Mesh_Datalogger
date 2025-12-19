#include "max31865_reader.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pt100_table.h"
#include "sdkconfig.h"

static const char* kTag = "max31865";

// MAX31865 register map.
static const uint8_t kRegConfig = 0x00;
static const uint8_t kRegRtdMsb = 0x01;
// static const uint8_t kRegRtdLsb = 0x02;
static const uint8_t kRegHighFaultMsb = 0x03;
static const uint8_t kRegHighFaultLsb = 0x04;
static const uint8_t kRegLowFaultMsb = 0x05;
static const uint8_t kRegLowFaultLsb = 0x06;
static const uint8_t kRegFaultStatus = 0x07;

// Config bits.
static const uint8_t kCfgVbias = 0x80;
static const uint8_t kCfgOneShot = 0x20;
static const uint8_t kCfg3Wire = 0x10;
static const uint8_t kCfgFaultStatusClear = 0x02;
static const uint8_t kCfgFilter50Hz = 0x01;

// Fault status bits.
static const uint8_t kFaultHighThreshold = 0x80;
static const uint8_t kFaultLowThreshold = 0x40;
static const uint8_t kFaultRefInLow = 0x20;
static const uint8_t kFaultRefInHigh = 0x10;
static const uint8_t kFaultRtdInLow = 0x08;
static const uint8_t kFaultOverUnder = 0x04;
static const uint8_t kFaultRtdFlag = 0x01; // Derived from RTD LSB fault bit.

static const double kCvdA = 3.9083e-3;
static const double kCvdB = -5.775e-7;
static const double kCvdC = -4.183e-12;

static esp_err_t
SpiTransfer(spi_device_handle_t device,
            const uint8_t* tx,
            size_t tx_len,
            uint8_t* rx,
            size_t rx_len)
{
  spi_transaction_t transaction;
  memset(&transaction, 0, sizeof(transaction));

  const size_t total_len = (tx_len > rx_len) ? tx_len : rx_len;
  transaction.length = total_len * 8;
  transaction.tx_buffer = tx;
  if (rx != NULL && rx_len > 0) {
    transaction.rxlength = total_len * 8;
    transaction.rx_buffer = rx;
  }
  return spi_device_transmit(device, &transaction);
}

esp_err_t
Max31865WriteReg(max31865_reader_t* reader, uint8_t reg, uint8_t value)
{
  if (reader == NULL || reader->spi_device == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  uint8_t tx[2] = { (uint8_t)(reg | 0x80u), value };
  return SpiTransfer(reader->spi_device, tx, sizeof(tx), NULL, 0);
}

esp_err_t
Max31865ReadRegs(max31865_reader_t* reader,
                 uint8_t reg,
                 uint8_t* data_out,
                 size_t len)
{
  if (reader == NULL || reader->spi_device == NULL || data_out == NULL ||
      len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (len > 8) {
    return ESP_ERR_INVALID_SIZE;
  }

  uint8_t tx[1 + 8] = { 0 };
  uint8_t rx[1 + 8] = { 0 };
  tx[0] = (uint8_t)(reg & 0x7Fu);

  const size_t total_len = 1 + len;
  esp_err_t result =
    SpiTransfer(reader->spi_device, tx, total_len, rx, total_len);
  if (result != ESP_OK) {
    return result;
  }
  memcpy(data_out, &rx[1], len);
  return ESP_OK;
}

esp_err_t
Max31865ReadReg(max31865_reader_t* reader, uint8_t reg, uint8_t* value_out)
{
  if (value_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  return Max31865ReadRegs(reader, reg, value_out, 1);
}

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

static int
ConversionDelayMs(const max31865_reader_t* reader)
{
  return (reader->filter_hz <= 50) ? 65 : 55;
}

double
Max31865AdcCodeToResistance(uint16_t adc_code, double rref_ohm)
{
  return ((double)adc_code * rref_ohm) / 32768.0;
}

static esp_err_t
ClearFaults(max31865_reader_t* reader, uint8_t base_config)
{
  return Max31865WriteReg(
    reader, kRegConfig, (uint8_t)(base_config | kCfgFaultStatusClear));
}

static esp_err_t
WaitForConversionComplete(max31865_reader_t* reader, int timeout_ms)
{
  const int64_t start_us = esp_timer_get_time();
  const int64_t timeout_us = (int64_t)timeout_ms * 1000;
  while ((esp_timer_get_time() - start_us) < timeout_us) {
    uint8_t cfg = 0;
    esp_err_t res = Max31865ReadReg(reader, kRegConfig, &cfg);
    if (res != ESP_OK) {
      return res;
    }
    if ((cfg & kCfgOneShot) == 0) {
      return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return ESP_ERR_TIMEOUT;
}

static double
ConvertTablePt100(double resistance_ohm, double r0_ohm)
{
  if (r0_ohm <= 0.0) {
    return NAN;
  }
  const double scaled_ohm = resistance_ohm * (100.0 / r0_ohm);
  const double ohm_x100 = scaled_ohm * 100.0;

  if (ohm_x100 <= kPt100TableOhmsX100[0]) {
    return PT100_TABLE_MIN_C;
  }
  if (ohm_x100 >= kPt100TableOhmsX100[kPt100TableLength - 1]) {
    return PT100_TABLE_MAX_C;
  }

  size_t low = 0;
  size_t high = kPt100TableLength - 1;
  while ((high - low) > 1) {
    const size_t mid = (low + high) / 2;
    if (kPt100TableOhmsX100[mid] <= ohm_x100) {
      low = mid;
    } else {
      high = mid;
    }
  }

  const double lower_val = (double)kPt100TableOhmsX100[low];
  const double upper_val = (double)kPt100TableOhmsX100[high];
  const double lower_temp = PT100_TABLE_MIN_C + (double)low;

  const double span = upper_val - lower_val;
  const double fraction = (span > 0.0) ? ((ohm_x100 - lower_val) / span) : 0.0;

  return lower_temp + fraction;
}

static double
ConvertCvdIterative(double resistance_ohm, double r0_ohm)
{
  if (r0_ohm <= 0.0) {
    return NAN;
  }
  const double ratio = resistance_ohm / r0_ohm;
  const double discriminant = (kCvdA * kCvdA) - (4.0 * kCvdB * (1.0 - ratio));
  if (discriminant >= 0.0) {
    const double temp = (-kCvdA + sqrt(discriminant)) / (2.0 * kCvdB);
    if (temp >= 0.0) {
      return temp;
    }
  }

  double t = -200.0;
  double f = 0.0;
  for (int i = 0; i < 20; ++i) {
    const double t2 = t * t;
    const double t3 = t2 * t;
    f = 1.0 + kCvdA * t + kCvdB * t2 + kCvdC * (t - 100.0) * t3 - ratio;
    const double df =
      kCvdA + 2.0 * kCvdB * t + 3.0 * kCvdC * t2 * (t - 100.0) + kCvdC * t3;
    if (fabs(df) < 1e-12) {
      break;
    }
    const double next = t - (f / df);
    if (fabs(next - t) < 1e-6) {
      t = next;
      break;
    }
    t = next;
  }
  if (t < PT100_TABLE_MIN_C) {
    t = PT100_TABLE_MIN_C;
  } else if (t > PT100_TABLE_MAX_C) {
    t = PT100_TABLE_MAX_C;
  }
  return t;
}

static double
ResistanceToTemperature(const max31865_reader_t* reader, double resistance_ohm)
{
  return (reader->conversion == kMax31865ConversionCvdIterative)
           ? ConvertCvdIterative(resistance_ohm, reader->rtd_nominal_ohm)
           : ConvertTablePt100(resistance_ohm, reader->rtd_nominal_ohm);
}

void
Max31865FormatFault(uint8_t fault_status, char* out, size_t out_len)
{
  if (out == NULL || out_len == 0) {
    return;
  }
  if (fault_status == 0) {
    snprintf(out, out_len, "none");
    return;
  }

  size_t written = 0;
  const struct
  {
    uint8_t bit;
    const char* label;
  } fault_map[] = {
    { kFaultHighThreshold, "rtd_high" }, { kFaultLowThreshold, "rtd_low" },
    { kFaultRefInLow, "refin_low" },     { kFaultRefInHigh, "refin_high" },
    { kFaultRtdInLow, "rtdin_low" },     { kFaultOverUnder, "ov_uv" },
    { kFaultRtdFlag, "fault_bit" },
  };

  for (size_t i = 0; i < sizeof(fault_map) / sizeof(fault_map[0]); ++i) {
    if ((fault_status & fault_map[i].bit) != 0) {
      const int n = snprintf(out + written,
                             (written < out_len) ? (out_len - written) : 0,
                             "%s%s",
                             (written > 0) ? "|" : "",
                             fault_map[i].label);
      if (n < 0) {
        break;
      }
      written += (size_t)n;
      if (written >= out_len) {
        break;
      }
    }
  }
}

static void
InitializeFaultThresholds(max31865_reader_t* reader)
{
  // Disable comparator faults by setting wide thresholds.
  (void)Max31865WriteReg(reader, kRegHighFaultMsb, 0xFF);
  (void)Max31865WriteReg(reader, kRegHighFaultLsb, 0xFF);
  (void)Max31865WriteReg(reader, kRegLowFaultMsb, 0x00);
  (void)Max31865WriteReg(reader, kRegLowFaultLsb, 0x00);
}

esp_err_t
Max31865ReaderInit(max31865_reader_t* reader,
                   spi_host_device_t host,
                   int cs_gpio)
{
  if (reader == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(reader, 0, sizeof(*reader));

  // MAX31865 uses SPI mode 1 (CPOL=0, CPHA=1).
  spi_device_interface_config_t device_config = {
    .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz conservative default.
    .mode = 1,
    .spics_io_num = cs_gpio,
    .queue_size = 2,
  };

  esp_err_t result =
    spi_bus_add_device(host, &device_config, &reader->spi_device);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "spi_bus_add_device failed: %s", esp_err_to_name(result));
    return result;
  }

  reader->rtd_nominal_ohm = (double)CONFIG_APP_RTD_R0_OHMS;
  reader->rref_ohm = (double)CONFIG_APP_MAX31865_RREF_OHMS;
  reader->bias_settle_ms = (uint32_t)CONFIG_APP_MAX31865_BIAS_SETTLE_MS;
  reader->pulsed_bias = true;
#if CONFIG_APP_MAX31865_WIRES_2
  reader->wires = 2;
#elif CONFIG_APP_MAX31865_WIRES_3
  reader->wires = 3;
#else
  reader->wires = 4;
#endif

#if CONFIG_APP_MAX31865_FILTER_60HZ
  reader->filter_hz = 60;
#else
  reader->filter_hz = 50;
#endif

#if CONFIG_APP_MAX31865_CONVERSION_CVD_ITERATIVE
  reader->conversion = kMax31865ConversionCvdIterative;
#else
  reader->conversion = kMax31865ConversionTablePt100;
#endif

  reader->ema_valid = false;
  reader->ema_temp_c = 0.0;
  reader->ema_resistance_ohm = 0.0;

  const uint8_t base_config = BuildBaseConfig(reader);
  InitializeFaultThresholds(reader);
  result = ClearFaults(reader, base_config);
  if (result == ESP_OK) {
    result = Max31865WriteReg(reader, kRegConfig, base_config);
  }
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "Init write failed: %s", esp_err_to_name(result));
    return result;
  }

  reader->is_initialized = true;
  ESP_LOGI(
    kTag,
    "Initialized MAX31865 (Rref=%.2fΩ R0=%.2fΩ wires=%u filter=%uHz mode=%s)",
    reader->rref_ohm,
    reader->rtd_nominal_ohm,
    (unsigned)reader->wires,
    (unsigned)reader->filter_hz,
    (reader->conversion == kMax31865ConversionCvdIterative) ? "CVD" : "TABLE");
  return ESP_OK;
}

static void
FillSample(max31865_sample_t* sample,
           uint16_t adc_code,
           double resistance,
           double temp_c,
           uint8_t fault_status)
{
  if (sample == NULL) {
    return;
  }
  sample->adc_code = adc_code;
  sample->resistance_ohm = resistance;
  sample->temperature_c = temp_c;
  sample->fault_status = fault_status;
  sample->fault_present = (fault_status != 0);
}

esp_err_t
Max31865ReadOnce(max31865_reader_t* reader, max31865_sample_t* sample_out)
{
  if (reader == NULL || sample_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!reader->is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  const uint8_t base_config = BuildBaseConfig(reader);
  (void)ClearFaults(reader, base_config);

  esp_err_t result =
    Max31865WriteReg(reader, kRegConfig, (uint8_t)(base_config | kCfgVbias));
  if (result != ESP_OK) {
    return result;
  }

  if (reader->bias_settle_ms > 0) {
    vTaskDelay(pdMS_TO_TICKS(reader->bias_settle_ms));
  }

  result = Max31865WriteReg(
    reader, kRegConfig, (uint8_t)(base_config | kCfgVbias | kCfgOneShot));
  if (result != ESP_OK) {
    (void)Max31865WriteReg(reader, kRegConfig, base_config);
    return result;
  }

  const int wait_ms = ConversionDelayMs(reader) + reader->bias_settle_ms;
  result = WaitForConversionComplete(reader, wait_ms + 10);
  if (result == ESP_ERR_TIMEOUT) {
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    result = ESP_OK;
  }

  uint8_t rtd_raw[2] = { 0 };
  if (result == ESP_OK) {
    result = Max31865ReadRegs(reader, kRegRtdMsb, rtd_raw, sizeof(rtd_raw));
  }

  uint8_t fault_reg = 0;
  if (result == ESP_OK) {
    result = Max31865ReadReg(reader, kRegFaultStatus, &fault_reg);
  }

  (void)Max31865WriteReg(reader, kRegConfig, base_config);

  if (result != ESP_OK) {
    return result;
  }

  uint16_t rtd_code = ((uint16_t)rtd_raw[0] << 8) | rtd_raw[1];
  const bool rtd_fault_bit = (rtd_code & 0x01u) != 0;
  rtd_code >>= 1;

  uint8_t combined_faults = fault_reg;
  if (rtd_fault_bit) {
    combined_faults |= kFaultRtdFlag;
  }

  const double resistance =
    Max31865AdcCodeToResistance(rtd_code, reader->rref_ohm);
  const double temp_c = ResistanceToTemperature(reader, resistance);

  FillSample(sample_out, rtd_code, resistance, temp_c, combined_faults);

  if (combined_faults != 0) {
    (void)ClearFaults(reader, base_config);
  }
  return ESP_OK;
}

esp_err_t
Max31865ReadAveraged(max31865_reader_t* reader,
                     int sample_count,
                     int sample_delay_ms,
                     max31865_sample_t* averaged_out,
                     max31865_sampling_stats_t* stats_out)
{
  if (reader == NULL || averaged_out == NULL || sample_count <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  max31865_sampling_stats_t stats;
  memset(&stats, 0, sizeof(stats));
  stats.requested_samples = sample_count;
  stats.min_temp_c = INFINITY;
  stats.max_temp_c = -INFINITY;
  stats.min_resistance_ohm = INFINITY;
  stats.max_resistance_ohm = -INFINITY;

  double mean_temp = 0.0;
  double m2 = 0.0;
  double mean_res = 0.0;
  double mean_code = 0.0;
  esp_err_t last_error = ESP_OK;

  for (int i = 0; i < sample_count; ++i) {
    max31865_sample_t sample;
    esp_err_t res = Max31865ReadOnce(reader, &sample);
    if (res != ESP_OK) {
      last_error = res;
    }
    if (res != ESP_OK || sample.fault_present) {
      stats.faulted_samples++;
    } else {
      stats.valid_samples++;
      const double delta_temp = sample.temperature_c - mean_temp;
      mean_temp += delta_temp / (double)stats.valid_samples;
      m2 += delta_temp * (sample.temperature_c - mean_temp);
      mean_res +=
        (sample.resistance_ohm - mean_res) / (double)stats.valid_samples;
      mean_code +=
        ((double)sample.adc_code - mean_code) / (double)stats.valid_samples;
      if (sample.temperature_c < stats.min_temp_c) {
        stats.min_temp_c = sample.temperature_c;
      }
      if (sample.temperature_c > stats.max_temp_c) {
        stats.max_temp_c = sample.temperature_c;
      }
      if (sample.resistance_ohm < stats.min_resistance_ohm) {
        stats.min_resistance_ohm = sample.resistance_ohm;
      }
      if (sample.resistance_ohm > stats.max_resistance_ohm) {
        stats.max_resistance_ohm = sample.resistance_ohm;
      }
    }
    if (sample_delay_ms > 0 && i + 1 < sample_count) {
      vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
    }
  }

  if (stats.valid_samples == 0) {
    return (last_error != ESP_OK) ? last_error : ESP_ERR_INVALID_RESPONSE;
  }

  averaged_out->adc_code = (uint16_t)llround(mean_code);
  averaged_out->resistance_ohm = mean_res;
  averaged_out->temperature_c = mean_temp;
  averaged_out->fault_status = 0;
  averaged_out->fault_present = false;

  stats.stddev_temp_c = sqrt(
    (stats.valid_samples > 1) ? (m2 / (double)(stats.valid_samples - 1)) : 0.0);

  if (stats_out != NULL) {
    *stats_out = stats;
  }
  return ESP_OK;
}

esp_err_t
Max31865ReadEmaUpdate(max31865_reader_t* reader,
                      double alpha,
                      max31865_sample_t* sample_out,
                      double* ema_temp_out)
{
  if (reader == NULL || sample_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (alpha <= 0.0 || alpha > 1.0) {
    return ESP_ERR_INVALID_ARG;
  }
  max31865_sample_t sample;
  esp_err_t res = Max31865ReadOnce(reader, &sample);
  if (res != ESP_OK || sample.fault_present) {
    if (res == ESP_OK) {
      FillSample(sample_out,
                 sample.adc_code,
                 sample.resistance_ohm,
                 sample.temperature_c,
                 sample.fault_status);
      return ESP_ERR_INVALID_RESPONSE;
    }
    memset(sample_out, 0, sizeof(*sample_out));
    return res;
  }

  if (!reader->ema_valid) {
    reader->ema_temp_c = sample.temperature_c;
    reader->ema_resistance_ohm = sample.resistance_ohm;
    reader->ema_valid = true;
  } else {
    reader->ema_temp_c =
      alpha * sample.temperature_c + (1.0 - alpha) * reader->ema_temp_c;
    reader->ema_resistance_ohm = alpha * sample.resistance_ohm +
                                 (1.0 - alpha) * reader->ema_resistance_ohm;
  }

  FillSample(sample_out,
             sample.adc_code,
             sample.resistance_ohm,
             sample.temperature_c,
             sample.fault_status);

  if (ema_temp_out != NULL) {
    *ema_temp_out = reader->ema_temp_c;
  }
  return ESP_OK;
}

esp_err_t
Max31865ReaderRead(max31865_reader_t* reader,
                   float* raw_temp_c,
                   float* resistance_ohm)
{
  if (reader == NULL || raw_temp_c == NULL || resistance_ohm == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  max31865_sample_t sample;
  esp_err_t res = Max31865ReadOnce(reader, &sample);
  if (res != ESP_OK) {
    return res;
  }
  *raw_temp_c = (float)sample.temperature_c;
  *resistance_ohm = (float)sample.resistance_ohm;
  return sample.fault_present ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}
