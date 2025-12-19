#ifndef PT100_LOGGER_MAX31865_READER_H_
#define PT100_LOGGER_MAX31865_READER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    kMax31865ConversionTablePt100 = 0,
    kMax31865ConversionCvdIterative = 1,
  } max31865_conversion_t;

  typedef struct
  {
    uint16_t adc_code;
    double resistance_ohm;
    double temperature_c;
    uint8_t fault_status;
    bool fault_present;
  } max31865_sample_t;

  typedef struct
  {
    int requested_samples;
    int valid_samples;
    int faulted_samples;
    double min_temp_c;
    double max_temp_c;
    double min_resistance_ohm;
    double max_resistance_ohm;
    double stddev_temp_c;
  } max31865_sampling_stats_t;

  typedef struct
  {
    spi_device_handle_t spi_device;
    double rtd_nominal_ohm; // e.g. 100.0 for PT100
    double rref_ohm;        // e.g. 430.0 on common breakout boards
    uint8_t wires;          // 2, 3, or 4
    uint8_t filter_hz;      // 50 or 60
    uint32_t bias_settle_ms;
    max31865_conversion_t conversion;
    bool pulsed_bias;
    bool is_initialized;
    double ema_temp_c;
    double ema_resistance_ohm;
    bool ema_valid;
  } max31865_reader_t;

  // Initialize MAX31865 on an already-initialized SPI bus.
  esp_err_t Max31865ReaderInit(max31865_reader_t* reader,
                               spi_host_device_t host,
                               int cs_gpio);

  // Register helpers.
  esp_err_t Max31865ReadReg(max31865_reader_t* reader,
                            uint8_t reg,
                            uint8_t* value_out);
  esp_err_t Max31865ReadRegs(max31865_reader_t* reader,
                             uint8_t reg,
                             uint8_t* data_out,
                             size_t len);
  esp_err_t Max31865WriteReg(max31865_reader_t* reader,
                             uint8_t reg,
                             uint8_t value);

  // Read one conversion using pulsed bias + one-shot mode.
  esp_err_t Max31865ReadOnce(max31865_reader_t* reader,
                             max31865_sample_t* sample_out);

  // Read multiple samples and report the mean (faulted samples are skipped).
  // stats_out is optional; if provided stddev_temp_c will be populated for the
  // non-faulted samples.
  esp_err_t Max31865ReadAveraged(max31865_reader_t* reader,
                                 int sample_count,
                                 int sample_delay_ms,
                                 max31865_sample_t* averaged_out,
                                 max31865_sampling_stats_t* stats_out);

  // Update an EMA filter with the latest reading. ema_temp_out may be NULL.
  esp_err_t Max31865ReadEmaUpdate(max31865_reader_t* reader,
                                  double alpha,
                                  max31865_sample_t* sample_out,
                                  double* ema_temp_out);

  // Legacy convenience wrapper returning floats.
  esp_err_t Max31865ReaderRead(max31865_reader_t* reader,
                               float* raw_temp_c,
                               float* resistance_ohm);

  // Helpers for diagnostics.
  void Max31865FormatFault(uint8_t fault_status, char* out, size_t out_len);
  double Max31865AdcCodeToResistance(uint16_t adc_code, double rref_ohm);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_MAX31865_READER_H_
