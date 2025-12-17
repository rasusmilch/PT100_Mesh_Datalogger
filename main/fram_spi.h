#ifndef PT100_LOGGER_FRAM_SPI_H_
#define PT100_LOGGER_FRAM_SPI_H_

#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    spi_device_handle_t device;
    int address_bytes; // 2 or 3
  } fram_spi_t;

  esp_err_t FramSpiInit(fram_spi_t* fram,
                        spi_host_device_t host,
                        int cs_gpio,
                        int address_bytes);

  esp_err_t FramSpiRead(const fram_spi_t* fram,
                        uint32_t address,
                        void* data_out,
                        size_t length_bytes);

  esp_err_t FramSpiWrite(const fram_spi_t* fram,
                         uint32_t address,
                         const void* data,
                         size_t length_bytes);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_SPI_H_
