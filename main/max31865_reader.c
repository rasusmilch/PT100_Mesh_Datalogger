#include "max31865_reader.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* kTag = "max31865";

// MAX31865 register map.
static const uint8_t kRegConfig = 0x00;
static const uint8_t kRegRtdMsb = 0x01;
// static const uint8_t kRegRtdLsb = 0x02;
// static const uint8_t kRegFaultStatus = 0x07;

// Config bits.
static const uint8_t kCfgVbias = 0x80;
static const uint8_t kCfgConversionModeAuto = 0x40;
static const uint8_t kCfg1Shot = 0x20;
static const uint8_t kCfg3Wire = 0x10;
static const uint8_t kCfgFaultStatusClear = 0x02;
static const uint8_t kCfgFilter50Hz = 0x01;

static esp_err_t
SpiTransfer(spi_device_handle_t device,
            const uint8_t* tx,
            size_t tx_len,
            uint8_t* rx,
            size_t rx_len)
{
  // Perform a single transaction with shared clock.
  // For reads, caller sets tx_len = 1 + rx_len and provides rx buffer of same
  // size.
  spi_transaction_t transaction;
  memset(&transaction, 0, sizeof(transaction));

  const size_t total_len = (rx_len > 0) ? (tx_len) : tx_len;
  transaction.length = total_len * 8;
  transaction.tx_buffer = tx;
  if (rx != NULL && rx_len > 0) {
    transaction.rxlength = total_len * 8;
    transaction.rx_buffer = rx;
  }
  return spi_device_transmit(device, &transaction);
}

static esp_err_t
WriteReg8(spi_device_handle_t device, uint8_t reg, uint8_t value)
{
  // Write: set MSB of register address.
  uint8_t tx[2] = { (uint8_t)(reg | 0x80u), value };
  return SpiTransfer(device, tx, sizeof(tx), NULL, 0);
}

static esp_err_t
ReadRegs(spi_device_handle_t device, uint8_t reg, uint8_t* data_out, size_t len)
{
  // Read: MSB clear.
  uint8_t tx[1 + 8] = { 0 };
  uint8_t rx[1 + 8] = { 0 };
  if (len > 8) {
    return ESP_ERR_INVALID_SIZE;
  }
  tx[0] = (uint8_t)(reg & 0x7Fu);
  esp_err_t result = SpiTransfer(device, tx, 1 + len, rx, 1 + len);
  if (result != ESP_OK) {
    return result;
  }
  memcpy(data_out, &rx[1], len);
  return ESP_OK;
}

// Convert RTD resistance (ohms) to temperature (C) using Callendar-Van Dusen.
// - For T >= 0: closed-form.
// - For T < 0: polynomial approximation (common MAX31865 approach).
static float
Pt100ResistanceToTemperatureC(float resistance_ohm, float r0_ohm)
{
  // IEC 60751 coefficients for alpha=0.00385.
  const float A = 3.9083e-3f;
  const float B = -5.775e-7f;

  const float ratio = resistance_ohm / r0_ohm;
  const float temp_guess =
    (-A + sqrtf(A * A - 4.0f * B * (1.0f - ratio))) / (2.0f * B);
  if (temp_guess >= 0.0f) {
    return temp_guess;
  }

  // Polynomial for negative temps:
  // Let Rt = (R/R0)*100, so that Rt=100 at 0C for PT100.
  const float Rt = ratio * 100.0f;
  const float Rt2 = Rt * Rt;
  const float Rt3 = Rt2 * Rt;
  const float Rt4 = Rt3 * Rt;
  const float Rt5 = Rt4 * Rt;

  // Coefficients from a common approximation used in embedded libraries.
  const float temp = -242.02f + 2.2228f * Rt + 2.5859e-3f * Rt2 -
                     4.8260e-6f * Rt3 - 2.8183e-8f * Rt4 + 1.5243e-10f * Rt5;

  return temp;
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
    .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz safe default.
    .mode = 1,
    .spics_io_num = cs_gpio,
    .queue_size = 1,
  };

  esp_err_t result =
    spi_bus_add_device(host, &device_config, &reader->spi_device);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "spi_bus_add_device failed: %s", esp_err_to_name(result));
    return result;
  }

  reader->rtd_nominal_ohm = 100.0f;
  reader->rref_ohm = 430.0f;
  reader->is_3wire = false;
  reader->filter_50hz = true;

  uint8_t config = 0;
  config |= kCfgVbias;
  config |= kCfgConversionModeAuto;
  if (reader->is_3wire) {
    config |= kCfg3Wire;
  }
  config |= kCfgFaultStatusClear;
  if (reader->filter_50hz) {
    config |= kCfgFilter50Hz;
  }

  result = WriteReg8(reader->spi_device, kRegConfig, config);
  if (result != ESP_OK) {
    return result;
  }

  // Allow bias to settle.
  vTaskDelay(pdMS_TO_TICKS(10));

  reader->is_initialized = true;
  ESP_LOGI(kTag, "Initialized MAX31865");
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
  if (!reader->is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Read RTD MSB/LSB.
  uint8_t data[2] = { 0 };
  esp_err_t result =
    ReadRegs(reader->spi_device, kRegRtdMsb, data, sizeof(data));
  if (result != ESP_OK) {
    return result;
  }

  // 15-bit RTD code in bits 15..1; bit0 is fault flag.
  uint16_t rtd_code = ((uint16_t)data[0] << 8) | data[1];
  const bool fault = (rtd_code & 0x0001u) != 0u;
  rtd_code >>= 1;

  const float ratio = (float)rtd_code / 32768.0f;
  const float rtd_resistance = ratio * reader->rref_ohm;

  *resistance_ohm = rtd_resistance;
  *raw_temp_c =
    Pt100ResistanceToTemperatureC(rtd_resistance, reader->rtd_nominal_ohm);

  if (fault) {
    // Clear fault in config register (write with clear bit set).
    uint8_t config = 0;
    config |= kCfgVbias;
    config |= kCfgConversionModeAuto;
    config |= kCfgFaultStatusClear;
    if (reader->filter_50hz) {
      config |= kCfgFilter50Hz;
    }
    (void)WriteReg8(reader->spi_device, kRegConfig, config);
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}
