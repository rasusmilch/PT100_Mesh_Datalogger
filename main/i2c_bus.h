#ifndef PT100_LOGGER_I2C_BUS_H_
#define PT100_LOGGER_I2C_BUS_H_

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
    i2c_master_bus_handle_t handle;
    i2c_port_t port;
    int sda_gpio;
    int scl_gpio;
    uint32_t frequency_hz;
    bool initialized;
  } i2c_bus_t;

  esp_err_t I2cBusInit(i2c_bus_t* bus,
                       i2c_port_t port,
                       int sda_gpio,
                       int scl_gpio,
                       uint32_t frequency_hz);

  esp_err_t I2cBusAddDevice(i2c_bus_t* bus,
                            uint16_t address,
                            uint32_t scl_speed_hz,
                            i2c_master_dev_handle_t* out_device);

  esp_err_t I2cBusReadRegister(i2c_master_dev_handle_t device,
                               uint8_t start_register,
                               uint8_t* data_out,
                               size_t length);

  esp_err_t I2cBusWriteRegister(i2c_master_dev_handle_t device,
                                uint8_t start_register,
                                const uint8_t* data,
                                size_t length);

  esp_err_t I2cBusScan(const i2c_bus_t* bus,
                       uint8_t start_addr,
                       uint8_t end_addr,
                       uint8_t* found_addrs,
                       size_t max_found,
                       size_t* found_count);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_I2C_BUS_H_
