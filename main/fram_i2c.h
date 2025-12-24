#ifndef PT100_LOGGER_FRAM_I2C_H_
#define PT100_LOGGER_FRAM_I2C_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    uint16_t manufacturer_id;
    uint16_t product_id;
    uint8_t raw[3];
  } fram_device_id_t;

  typedef struct
  {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;
    uint8_t i2c_addr_7bit;
    uint32_t scl_speed_hz;
    size_t fram_size_bytes;
    bool initialized;
  } fram_i2c_t;

  esp_err_t FramI2cInit(fram_i2c_t* fram,
                        i2c_master_bus_handle_t bus,
                        uint8_t i2c_addr_7bit,
                        size_t fram_size_bytes,
                        uint32_t scl_speed_hz);

  esp_err_t FramI2cRead(const fram_i2c_t* fram,
                        uint16_t addr,
                        void* out,
                        size_t len);

  esp_err_t FramI2cWrite(const fram_i2c_t* fram,
                         uint16_t addr,
                         const void* data,
                         size_t len);

  esp_err_t FramI2cReadDeviceId(const fram_i2c_t* fram, fram_device_id_t* out);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_I2C_H_
