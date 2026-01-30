#ifndef PT100_LOGGER_FRAM_I2C_H_
#define PT100_LOGGER_FRAM_I2C_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "i2c_bus.h"

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
    const i2c_bus_t* bus_state;
    uint8_t i2c_addr_7bit;
    uint32_t scl_speed_hz;
    size_t fram_size_bytes;
    bool initialized;
  } fram_i2c_t;

/**
 * @brief Execute FramI2cInit.
 * @param fram Parameter fram.
 * @param bus Parameter bus.
 * @param i2c_addr_7bit Parameter i2c_addr_7bit.
 * @param fram_size_bytes Parameter fram_size_bytes.
 * @param scl_speed_hz Parameter scl_speed_hz.
 * @return Return the function result.
 */
  esp_err_t FramI2cInit(fram_i2c_t* fram,
                        i2c_master_bus_handle_t bus,
                        const i2c_bus_t* bus_state,
                        uint8_t i2c_addr_7bit,
                        size_t fram_size_bytes,
                        uint32_t scl_speed_hz);

/**
 * @brief Execute FramI2cRead.
 * @param fram Parameter fram.
 * @param addr Parameter addr.
 * @param out Parameter out.
 * @param len Parameter len.
 * @return Return the function result.
 */
  esp_err_t FramI2cRead(const fram_i2c_t* fram,
                        uint16_t addr,
                        void* out,
                        size_t len);

/**
 * @brief Execute FramI2cWrite.
 * @param fram Parameter fram.
 * @param addr Parameter addr.
 * @param data Parameter data.
 * @param len Parameter len.
 * @return Return the function result.
 */
  esp_err_t FramI2cWrite(const fram_i2c_t* fram,
                         uint16_t addr,
                         const void* data,
                         size_t len);

/**
 * @brief Execute FramI2cReadDeviceId.
 * @param fram Parameter fram.
 * @param out Parameter out.
 * @return Return the function result.
 */
  esp_err_t FramI2cReadDeviceId(const fram_i2c_t* fram, fram_device_id_t* out);

/**
 * @brief Execute FramI2cDeinit.
 * @param fram Parameter fram.
 * @return Return the function result.
 */
  esp_err_t FramI2cDeinit(fram_i2c_t* fram);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_I2C_H_
