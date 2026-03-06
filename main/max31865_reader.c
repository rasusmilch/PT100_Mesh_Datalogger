#include "max31865_reader.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_heap_caps.h"
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
static const uint8_t kFaultRefInAboveBiasThreshold = 0x20;
static const uint8_t kFaultRefInBelowBiasThreshold = 0x10;
static const uint8_t kFaultRtdInLow = 0x08;
static const uint8_t kFaultOverUnder = 0x04;
static const uint8_t kFaultRtdFlag = 0x01; // Derived from RTD LSB fault bit.

static const double kCvdA = 3.9083e-3;
static const double kCvdB = -5.775e-7;
static const double kCvdC = -4.183e-12;



static void FillSample(max31865_sample_t* sample,
                       uint16_t adc_code,
                       double resistance,
                       double temp_c,
                       uint8_t fault_status);

static esp_err_t ConfigureRdyGpio(max31865_reader_t* reader);

static bool
Max31865AcquireSpiBus(max31865_reader_t* reader)
{
  if (reader == NULL || reader->spi_bus_lock == NULL) {
    return true;
  }
  return reader->spi_bus_lock(reader->spi_bus_lock_context,
                              reader->spi_bus_lock_timeout_ticks);
}

static void
Max31865ReleaseSpiBus(max31865_reader_t* reader)
{
  if (reader == NULL || reader->spi_bus_unlock == NULL) {
    return;
  }
  reader->spi_bus_unlock(reader->spi_bus_lock_context);
}

static bool
IsInvalidRegisterPattern(uint8_t fault_status, const uint8_t* rtd_raw)
{
  if (fault_status == 0xFFu) {
    return true;
  }
  if (rtd_raw != NULL && rtd_raw[0] == 0xFFu && rtd_raw[1] == 0xFFu) {
    return true;
  }
  return false;
}

/**
 * @brief Execute SpiTransfer.
 * @param reader Parameter reader.
 * @param tx Parameter tx.
 * @param tx_len Parameter tx_len.
 * @param rx Parameter rx.
 * @param rx_len Parameter rx_len.
 * @return Return the function result.
 */
static esp_err_t
SpiTransfer(max31865_reader_t* reader,
            const uint8_t* tx,
            size_t tx_len,
            uint8_t* rx,
            size_t rx_len)
{
  if (reader == NULL || reader->spi_device == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!Max31865AcquireSpiBus(reader)) {
    return ESP_ERR_TIMEOUT;
  }

  spi_transaction_t transaction;
  memset(&transaction, 0, sizeof(transaction));

  const size_t total_len = (tx_len > rx_len) ? tx_len : rx_len;
  if (total_len == 0) {
    Max31865ReleaseSpiBus(reader);
    return ESP_OK;
  }

  // If the calling task stack lives in PSRAM, stack-local tx/rx buffers are not
  // DMA-capable. The SPI master driver will then allocate internal bounce
  // buffers per transaction. Avoid that allocation path by staging transfers
  // into internal DMA-capable buffers owned by the reader.
  if (reader->dma_tx_buf == NULL || reader->dma_rx_buf == NULL ||
      reader->dma_buf_len < total_len) {
    Max31865ReleaseSpiBus(reader);
    return ESP_ERR_INVALID_STATE;
  }

  memset(reader->dma_tx_buf, 0, total_len);
  if (tx != NULL && tx_len > 0) {
    memcpy(reader->dma_tx_buf, tx, tx_len);
  }

  transaction.length = total_len * 8;
  transaction.tx_buffer = reader->dma_tx_buf;

  if (rx != NULL && rx_len > 0) {
    transaction.rxlength = total_len * 8;
    transaction.rx_buffer = reader->dma_rx_buf;
  }

  esp_err_t result =
    spi_device_polling_transmit(reader->spi_device, &transaction);
  if (result != ESP_OK) {
    Max31865ReleaseSpiBus(reader);
    return result;
  }

  if (rx != NULL && rx_len > 0) {
    memcpy(rx, reader->dma_rx_buf, rx_len);
  }
  Max31865ReleaseSpiBus(reader);
  return ESP_OK;
}

/**
 * @brief Execute Max31865WriteReg.
 * @param reader Parameter reader.
 * @param reg Parameter reg.
 * @param value Parameter value.
 * @return Return the function result.
 */
esp_err_t
Max31865WriteReg(max31865_reader_t* reader, uint8_t reg, uint8_t value)
{
  if (reader == NULL || reader->spi_device == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  uint8_t tx[2] = { (uint8_t)(reg | 0x80u), value };
  return SpiTransfer(reader, tx, sizeof(tx), NULL, 0);
}

/**
 * @brief Execute Max31865ReadRegs.
 * @param reader Parameter reader.
 * @param reg Parameter reg.
 * @param data_out Parameter data_out.
 * @param len Parameter len.
 * @return Return the function result.
 */
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
  esp_err_t result = SpiTransfer(reader, tx, total_len, rx, total_len);
  if (result != ESP_OK) {
    return result;
  }
  memcpy(data_out, &rx[1], len);
  return ESP_OK;
}

/**
 * @brief Execute Max31865ReadReg.
 * @param reader Parameter reader.
 * @param reg Parameter reg.
 * @param value_out Parameter value_out.
 * @return Return the function result.
 */
esp_err_t
Max31865ReadReg(max31865_reader_t* reader, uint8_t reg, uint8_t* value_out)
{
  if (value_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  return Max31865ReadRegs(reader, reg, value_out, 1);
}

/**
 * @brief Execute BuildBaseConfig.
 * @param reader Parameter reader.
 * @return Return the function result.
 */
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

/**
 * @brief Execute ConversionDelayMs.
 * @param reader Parameter reader.
 * @return Return the function result.
 */
static int
ConversionDelayMs(const max31865_reader_t* reader)
{
  return (reader->filter_hz <= 50) ? 65 : 55;
}

/**
 * @brief Execute Max31865AdcCodeToResistance.
 * @param adc_code Parameter adc_code.
 * @param rref_ohm Parameter rref_ohm.
 * @return Return the function result.
 */
double
Max31865AdcCodeToResistance(uint16_t adc_code, double rref_ohm)
{
  return ((double)adc_code * rref_ohm) / 32768.0;
}

/**
 * @brief Execute ClearFaults.
 * @param reader Parameter reader.
 * @param base_config Parameter base_config.
 * @return Return the function result.
 */
static esp_err_t
ClearFaults(max31865_reader_t* reader, uint8_t base_config)
{
  return Max31865WriteReg(
    reader, kRegConfig, (uint8_t)(base_config | kCfgFaultStatusClear));
}

/**
 * @brief Read raw RTD/fault registers and populate a parsed sample.
 * @param reader Parameter reader.
 * @param base_config Parameter base_config.
 * @param sample_out Parameter sample_out.
 * @return Return the function result.
 */
static esp_err_t
ReadAndFinishOneShot(max31865_reader_t* reader,
                     uint8_t base_config,
                     max31865_sample_t* sample_out)
{
  uint8_t rtd_raw[2] = { 0 };
  esp_err_t result =
    Max31865ReadRegs(reader, kRegRtdMsb, rtd_raw, sizeof(rtd_raw));

  uint8_t fault_reg = 0;
  if (result == ESP_OK) {
    result = Max31865ReadReg(reader, kRegFaultStatus, &fault_reg);
  }

  const esp_err_t disable_result = Max31865WriteReg(reader, kRegConfig, base_config);
  if (result != ESP_OK) {
    return result;
  }
  if (disable_result != ESP_OK) {
    return disable_result;
  }

  if (IsInvalidRegisterPattern(fault_reg, rtd_raw)) {
    return ESP_ERR_INVALID_RESPONSE;
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
  const double temp_c = Max31865ResistanceToTemperature(reader, resistance);

  FillSample(sample_out, rtd_code, resistance, temp_c, combined_faults);

  if (combined_faults != 0) {
    (void)ClearFaults(reader, base_config);
  }
  return ESP_OK;
}

/**
 * @brief Execute ConvertTablePt100.
 * @param resistance_ohm Parameter resistance_ohm.
 * @param r0_ohm Parameter r0_ohm.
 * @return Return the function result.
 */
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

/**
 * @brief Execute ConvertCvdIterative.
 * @param resistance_ohm Parameter resistance_ohm.
 * @param r0_ohm Parameter r0_ohm.
 * @return Return the function result.
 */
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

/**
 * @brief Execute ResistanceToTemperature.
 * @param reader Parameter reader.
 * @param resistance_ohm Parameter resistance_ohm.
 * @return Return the function result.
 */
double
Max31865ResistanceToTemperature(const max31865_reader_t* reader,
                                double resistance_ohm)
{
  if (reader == NULL) {
    return NAN;
  }
  return (reader->conversion == kMax31865ConversionCvdIterative)
           ? ConvertCvdIterative(resistance_ohm, reader->rtd_nominal_ohm)
           : ConvertTablePt100(resistance_ohm, reader->rtd_nominal_ohm);
}

/**
 * @brief Convert PT100 temperature to resistance via the CVD polynomial.
 * @param temperature_c Temperature in Celsius.
 * @param r0_ohm Nominal 0C resistance.
 * @return Equivalent resistance in ohms.
 */
double
Max31865TemperatureToResistanceCvd(double temperature_c, double r0_ohm)
{
  if (r0_ohm <= 0.0) {
    return NAN;
  }
  const double t = temperature_c;
  const double t2 = t * t;
  const double t3 = t2 * t;
  if (t >= 0.0) {
    return r0_ohm * (1.0 + kCvdA * t + kCvdB * t2);
  }
  return r0_ohm * (1.0 + kCvdA * t + kCvdB * t2 + kCvdC * (t - 100.0) * t3);
}

/**
 * @brief Execute Max31865FormatFault.
 * @param fault_status Parameter fault_status.
 * @param out Parameter out.
 * @param out_len Parameter out_len.
 */
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
    { kFaultHighThreshold, "RTD high threshold" },
    { kFaultLowThreshold, "RTD low threshold" },
    { kFaultRefInAboveBiasThreshold, "REFIN- > 0.85*VBIAS" },
    { kFaultRefInBelowBiasThreshold, "REFIN- < 0.85*VBIAS (FORCE- open)" },
    { kFaultRtdInLow, "RTDIN- < 0.85*VBIAS (FORCE- open)" },
    { kFaultOverUnder, "Over/undervoltage" },
    { kFaultRtdFlag, "Fault summary bit (RTD LSB)" },
  };
  bool matched = false;

  for (size_t i = 0; i < sizeof(fault_map) / sizeof(fault_map[0]); ++i) {
    if ((fault_status & fault_map[i].bit) != 0) {
      matched = true;
      const int n = snprintf(out + written,
                             (written < out_len) ? (out_len - written) : 0,
                             "%s%s",
                             (written > 0) ? ", " : "",
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
  if (!matched) {
    snprintf(out, out_len, "unknown(0x%02X)", fault_status);
  } else {
    out[out_len - 1] = '\0';
  }
}

/**
 * @brief Execute InitializeFaultThresholds.
 * @param reader Parameter reader.
 */
static void
InitializeFaultThresholds(max31865_reader_t* reader)
{
  // Disable comparator faults by setting wide thresholds.
  (void)Max31865WriteReg(reader, kRegHighFaultMsb, 0xFF);
  (void)Max31865WriteReg(reader, kRegHighFaultLsb, 0xFF);
  (void)Max31865WriteReg(reader, kRegLowFaultMsb, 0x00);
  (void)Max31865WriteReg(reader, kRegLowFaultLsb, 0x00);
}

/**
 * @brief Configure optional MAX31865 RDY GPIO input behavior.
 * @param reader Initialized reader storage to receive RDY configuration.
 * @return ESP_OK when configuration succeeds or RDY is disabled.
 */
static esp_err_t
ConfigureRdyGpio(max31865_reader_t* reader)
{
  if (reader == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  reader->rdy_gpio = CONFIG_APP_MAX31865_RDY_GPIO;
  reader->rdy_enabled = (reader->rdy_gpio >= 0);
#if CONFIG_APP_MAX31865_RDY_READY_HIGH
  reader->rdy_ready_level = 1;
#else
  reader->rdy_ready_level = 0;
#endif

  if (!reader->rdy_enabled) {
    ESP_LOGI(kTag, "RDY GPIO disabled");
    return ESP_OK;
  }

  gpio_config_t io_config = {
    .pin_bit_mask = (1ULL << reader->rdy_gpio),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };

#if CONFIG_APP_MAX31865_RDY_PULLUP
  io_config.pull_up_en = GPIO_PULLUP_ENABLE;
#elif CONFIG_APP_MAX31865_RDY_PULLDOWN
  io_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
#endif

  esp_err_t result = gpio_config(&io_config);
  if (result != ESP_OK) {
    reader->rdy_enabled = false;
    return result;
  }

#if CONFIG_APP_MAX31865_RDY_PULLUP
  const char* pull_mode = "pullup";
#elif CONFIG_APP_MAX31865_RDY_PULLDOWN
  const char* pull_mode = "pulldown";
#else
  const char* pull_mode = "none";
#endif

  ESP_LOGI(kTag,
           "RDY GPIO configured (gpio=%d pull=%s ready_level=%u)",
           reader->rdy_gpio,
           pull_mode,
           (unsigned)reader->rdy_ready_level);
  return ESP_OK;
}

/**
 * @brief Execute Max31865ReaderInit.
 * @param reader Parameter reader.
 * @param host Parameter host.
 * @param cs_gpio Parameter cs_gpio.
 * @return Return the function result.
 */
esp_err_t
Max31865ReaderInit(max31865_reader_t* reader,
                   spi_host_device_t host,
                   int cs_gpio)
{
  if (reader == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (reader->spi_device != NULL || reader->dma_tx_buf != NULL ||
      reader->dma_rx_buf != NULL || reader->is_initialized) {
    ESP_LOGW(kTag, "Reinitializing MAX31865; cleaning stale handles");
    esp_err_t cleanup_result = Max31865ReaderDeinit(reader);
    if (cleanup_result != ESP_OK) {
      ESP_LOGE(
        kTag,
        "MAX31865 deinit failed before reinit: %s",
        esp_err_to_name(cleanup_result));
      return cleanup_result;
    }
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

  // Allocate small DMA-capable staging buffers in internal RAM. This prevents
  // per-transaction bounce-buffer allocations when the calling task stack is in
  // PSRAM (common once we move larger task stacks to external memory).
  reader->dma_buf_len =
    16; // Plenty for MAX31865 register ops (<= 1 + 8 bytes).
  reader->dma_tx_buf = (uint8_t*)heap_caps_aligned_alloc(
    4, reader->dma_buf_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  reader->dma_rx_buf = (uint8_t*)heap_caps_aligned_alloc(
    4, reader->dma_buf_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (reader->dma_tx_buf == NULL || reader->dma_rx_buf == NULL) {
    ESP_LOGE(kTag, "DMA staging buffer alloc failed");
    if (reader->dma_tx_buf != NULL) {
      heap_caps_free(reader->dma_tx_buf);
      reader->dma_tx_buf = NULL;
    }
    if (reader->dma_rx_buf != NULL) {
      heap_caps_free(reader->dma_rx_buf);
      reader->dma_rx_buf = NULL;
    }
    reader->dma_buf_len = 0;
    (void)spi_bus_remove_device(reader->spi_device);
    reader->spi_device = NULL;
    return ESP_ERR_NO_MEM;
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

  result = ConfigureRdyGpio(reader);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "RDY GPIO config failed: %s", esp_err_to_name(result));
    return result;
  }

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
  const int32_t rref_mohm = (int32_t)llround(reader->rref_ohm * 1000.0);
  const int32_t r0_mohm = (int32_t)llround(reader->rtd_nominal_ohm * 1000.0);
  ESP_LOGI(
    kTag,
    "Initialized MAX31865 (Rref_mohm=%" PRId32 " R0_mohm=%" PRId32
    " wires=%u filter=%uHz mode=%s)",
    rref_mohm,
    r0_mohm,
    (unsigned)reader->wires,
    (unsigned)reader->filter_hz,
    (reader->conversion == kMax31865ConversionCvdIterative) ? "CVD" : "TABLE");
  return ESP_OK;
}

/**
 * @brief Execute Max31865ReaderDeinit.
 * @param reader Parameter reader.
 * @return Return the function result.
 */
esp_err_t
Max31865ReaderDeinit(max31865_reader_t* reader)
{
  if (reader == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t result = ESP_OK;
  if (reader->spi_device != NULL) {
    result = spi_bus_remove_device(reader->spi_device);
  }
  if (reader->dma_tx_buf != NULL) {
    heap_caps_free(reader->dma_tx_buf);
  }
  if (reader->dma_rx_buf != NULL) {
    heap_caps_free(reader->dma_rx_buf);
  }

  memset(reader, 0, sizeof(*reader));
  return result;
}

/**
 * @brief Execute FillSample.
 * @param sample Parameter sample.
 * @param adc_code Parameter adc_code.
 * @param resistance Parameter resistance.
 * @param temp_c Parameter temp_c.
 * @param fault_status Parameter fault_status.
 */
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

/**
 * @brief Execute Max31865ReadOnce.
 * @param reader Parameter reader.
 * @param sample_out Parameter sample_out.
 * @return Return the function result.
 */
esp_err_t
Max31865ReadOnce(max31865_reader_t* reader, max31865_sample_t* sample_out)
{
  if (reader == NULL || sample_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!reader->is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  max31865_one_shot_state_t one_shot;
  esp_err_t result = Max31865StartOneShot(reader, &one_shot);
  if (result != ESP_OK) {
    return result;
  }

  while (true) {
    result = Max31865TryReadOneShot(reader, &one_shot, sample_out);
    if (result == ESP_OK) {
      return ESP_OK;
    }
    if (result != ESP_ERR_TIMEOUT) {
      (void)Max31865AbortOneShot(reader, &one_shot);
      return result;
    }

    const int64_t now_us = esp_timer_get_time();
    int64_t delay_ms =
      (one_shot.next_action_deadline_us > now_us)
        ? (one_shot.next_action_deadline_us - now_us + 999) / 1000
        : 1;
    if (delay_ms < 1) {
      delay_ms = 1;
    }
    vTaskDelay(pdMS_TO_TICKS((uint32_t)delay_ms));
  }
}

/**
 * @brief Start a non-blocking one-shot conversion sequence.
 * @param reader Parameter reader.
 * @param state_out Parameter state_out.
 * @return Return the function result.
 */
esp_err_t
Max31865StartOneShot(max31865_reader_t* reader,
                     max31865_one_shot_state_t* state_out)
{
  if (reader == NULL || state_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(state_out, 0, sizeof(*state_out));
  state_out->phase = kMax31865OneShotIdle;

  if (!reader->is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  const uint8_t base_config = BuildBaseConfig(reader);
  esp_err_t result = ClearFaults(reader, base_config);
  if (result != ESP_OK) {
    return result;
  }
  result = Max31865WriteReg(reader, kRegConfig, (uint8_t)(base_config | kCfgVbias));
  if (result != ESP_OK) {
    return result;
  }

  const int64_t now_us = esp_timer_get_time();
  const int64_t bias_us = (int64_t)reader->bias_settle_ms * 1000;
  const int64_t timeout_us =
    (int64_t)(reader->bias_settle_ms + ConversionDelayMs(reader) + 50) * 1000;
  state_out->phase = (reader->bias_settle_ms > 0)
                       ? kMax31865OneShotBiasSettling
                       : kMax31865OneShotConverting;
  state_out->base_config = base_config;
  state_out->next_action_deadline_us = now_us + bias_us;
  state_out->hard_timeout_deadline_us = now_us + timeout_us;
  state_out->conversion_started = false;
  return ESP_OK;
}

/**
 * @brief Advance and optionally finish a started one-shot conversion.
 * @param reader Parameter reader.
 * @param state Parameter state.
 * @param sample_out Parameter sample_out.
 * @return Return the function result.
 */
esp_err_t
Max31865TryReadOneShot(max31865_reader_t* reader,
                       max31865_one_shot_state_t* state,
                       max31865_sample_t* sample_out)
{
  if (reader == NULL || state == NULL || sample_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!reader->is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (state->phase == kMax31865OneShotIdle) {
    return ESP_ERR_INVALID_STATE;
  }

  const int64_t now_us = esp_timer_get_time();
  if (now_us < state->next_action_deadline_us) {
    return ESP_ERR_TIMEOUT;
  }

  if (state->phase == kMax31865OneShotBiasSettling ||
      (state->phase == kMax31865OneShotConverting &&
       !state->conversion_started)) {
    esp_err_t result = Max31865WriteReg(reader,
                                        kRegConfig,
                                        (uint8_t)(state->base_config | kCfgVbias |
                                                  kCfgOneShot));
    if (result != ESP_OK) {
      (void)Max31865AbortOneShot(reader, state);
      return result;
    }

    state->phase = kMax31865OneShotConverting;
    state->conversion_started = true;
    state->next_action_deadline_us =
      now_us + ((int64_t)ConversionDelayMs(reader) * 1000);
    return ESP_ERR_TIMEOUT;
  }

  if (now_us > state->hard_timeout_deadline_us) {
    (void)Max31865AbortOneShot(reader, state);
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t result = ESP_OK;
  if (reader->rdy_enabled) {
    const int rdy_level = gpio_get_level((gpio_num_t)reader->rdy_gpio);
    if (rdy_level < 0 ||
        (uint8_t)rdy_level != reader->rdy_ready_level) {
      state->next_action_deadline_us = now_us + ((int64_t)10 * 1000);
      return ESP_ERR_TIMEOUT;
    }
  } else {
    uint8_t config = 0;
    result = Max31865ReadReg(reader, kRegConfig, &config);
    if (result != ESP_OK) {
      (void)Max31865AbortOneShot(reader, state);
      return result;
    }
    if ((config & kCfgOneShot) != 0) {
      state->next_action_deadline_us = now_us + 2000;
      return ESP_ERR_TIMEOUT;
    }
  }

  result = ReadAndFinishOneShot(reader, state->base_config, sample_out);
  state->phase = kMax31865OneShotIdle;
  state->conversion_started = false;
  return result;
}

/**
 * @brief Abort a one-shot conversion by restoring base configuration.
 * @param reader Parameter reader.
 * @param state Parameter state.
 * @return Return the function result.
 */
esp_err_t
Max31865AbortOneShot(max31865_reader_t* reader, max31865_one_shot_state_t* state)
{
  if (reader == NULL || state == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!reader->is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = ESP_OK;
  if (state->phase != kMax31865OneShotIdle) {
    result = Max31865WriteReg(reader, kRegConfig, state->base_config);
  }
  state->phase = kMax31865OneShotIdle;
  state->conversion_started = false;
  state->next_action_deadline_us = 0;
  state->hard_timeout_deadline_us = 0;
  return result;
}

/**
 * @brief Recover MAX31865 register state to known-good base configuration.
 * @param reader Parameter reader.
 * @return Return the function result.
 */
esp_err_t
Max31865RecoverToBaseConfig(max31865_reader_t* reader)
{
  if (reader == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!reader->is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  const uint8_t base_config = BuildBaseConfig(reader);
  esp_err_t result = Max31865WriteReg(reader, kRegConfig, base_config);
  if (result != ESP_OK) {
    return result;
  }

  result = ClearFaults(reader, base_config);
  if (result != ESP_OK) {
    return result;
  }

  return Max31865WriteReg(reader, kRegConfig, base_config);
}

/**
 * @brief Execute Max31865ReadAveraged.
 * @param reader Parameter reader.
 * @param sample_count Parameter sample_count.
 * @param sample_delay_ms Parameter sample_delay_ms.
 * @param averaged_out Parameter averaged_out.
 * @param stats_out Parameter stats_out.
 * @return Return the function result.
 */
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

/**
 * @brief Execute Max31865ReadEmaUpdate.
 * @param reader Parameter reader.
 * @param alpha Parameter alpha.
 * @param sample_out Parameter sample_out.
 * @param ema_temp_out Parameter ema_temp_out.
 * @return Return the function result.
 */
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
  if (res != ESP_OK) {
    memset(sample_out, 0, sizeof(*sample_out));
    return res;
  }
  esp_err_t ema_result = Max31865ApplyEmaSample(reader, alpha, &sample, ema_temp_out);
  if (ema_result != ESP_OK) {
    memset(sample_out, 0, sizeof(*sample_out));
    return ema_result;
  }

  FillSample(sample_out,
             sample.adc_code,
             sample.resistance_ohm,
             sample.temperature_c,
             sample.fault_status);
  return ESP_OK;
}

/**
 * @brief Update EMA state using a pre-acquired sample.
 * @param reader Parameter reader.
 * @param alpha Parameter alpha.
 * @param sample Parameter sample.
 * @param ema_temp_out Parameter ema_temp_out.
 * @return Return the function result.
 */
esp_err_t
Max31865ApplyEmaSample(max31865_reader_t* reader,
                       double alpha,
                       const max31865_sample_t* sample,
                       double* ema_temp_out)
{
  if (reader == NULL || sample == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (alpha <= 0.0 || alpha > 1.0) {
    return ESP_ERR_INVALID_ARG;
  }

  if (sample->fault_present) {
    if (ema_temp_out != NULL) {
      *ema_temp_out = reader->ema_temp_c;
    }
    return ESP_OK;
  }

  if (!reader->ema_valid) {
    reader->ema_resistance_ohm = sample->resistance_ohm;
    reader->ema_valid = true;
  } else {
    reader->ema_resistance_ohm =
      alpha * sample->resistance_ohm + (1.0 - alpha) * reader->ema_resistance_ohm;
  }
  reader->ema_temp_c =
    Max31865ResistanceToTemperature(reader, reader->ema_resistance_ohm);
  if (ema_temp_out != NULL) {
    *ema_temp_out = reader->ema_temp_c;
  }
  return ESP_OK;
}

/**
 * @brief Execute Max31865ReaderRead.
 * @param reader Parameter reader.
 * @param raw_temp_c Parameter raw_temp_c.
 * @param resistance_ohm Parameter resistance_ohm.
 * @return Return the function result.
 */
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
