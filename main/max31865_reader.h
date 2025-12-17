#ifndef PT100_LOGGER_MAX31865_READER_H_
#define PT100_LOGGER_MAX31865_READER_H_

#include <stdbool.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    spi_device_handle_t spi_device;
    float rtd_nominal_ohm; // e.g. 100.0 for PT100
    float rref_ohm;        // e.g. 430.0 on common breakout boards
    bool is_3wire;
    bool filter_50hz;
    bool is_initialized;
  } max31865_reader_t;

  // Initialize MAX31865 on an already-initialized SPI bus.
  // Defaults: PT100 (100 ohm nominal), Rref=430 ohm, 4-wire, 50Hz filter, auto
  // conversion.
  esp_err_t Max31865ReaderInit(max31865_reader_t* reader,
                               spi_host_device_t host,
                               int cs_gpio);

  // Read one measurement.
  // - raw_temp_c: temperature derived from RTD conversion (uncalibrated)
  // - resistance_ohm: RTD resistance in ohms
  esp_err_t Max31865ReaderRead(max31865_reader_t* reader,
                               float* raw_temp_c,
                               float* resistance_ohm);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_MAX31865_READER_H_
