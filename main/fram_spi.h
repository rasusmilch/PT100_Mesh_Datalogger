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

/**
 * @brief Execute FramSpiInit.
 * @param fram Parameter fram.
 * @param host Parameter host.
 * @param cs_gpio Parameter cs_gpio.
 * @param address_bytes Parameter address_bytes.
 * @return Return the function result.
 */
  esp_err_t FramSpiInit(fram_spi_t* fram,
                        spi_host_device_t host,
                        int cs_gpio,
                        int address_bytes);
/**
 * @brief Execute FramSpiDeinit.
 * @param fram Parameter fram.
 * @return Return the function result.
 */
  esp_err_t FramSpiDeinit(fram_spi_t* fram);

/**
 * @brief Execute FramSpiRead.
 * @param fram Parameter fram.
 * @param address Parameter address.
 * @param data_out Parameter data_out.
 * @param length_bytes Parameter length_bytes.
 * @return Return the function result.
 */
  esp_err_t FramSpiRead(const fram_spi_t* fram,
                        uint32_t address,
                        void* data_out,
                        size_t length_bytes);

/**
 * @brief Execute FramSpiWrite.
 * @param fram Parameter fram.
 * @param address Parameter address.
 * @param data Parameter data.
 * @param length_bytes Parameter length_bytes.
 * @return Return the function result.
 */
  esp_err_t FramSpiWrite(const fram_spi_t* fram,
                         uint32_t address,
                         const void* data,
                         size_t length_bytes);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_SPI_H_
