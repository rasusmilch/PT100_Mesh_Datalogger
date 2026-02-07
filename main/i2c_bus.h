#ifndef PT100_LOGGER_I2C_BUS_H_
#define PT100_LOGGER_I2C_BUS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_BUS_TRANSACTION_TIMEOUT_MS 25

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

  typedef struct
  {
    int scl_level;
    int sda_level;
    bool controller_busy;
  } i2c_bus_state_t;

/**
 * @brief Execute I2cBusInit.
 * @param bus Parameter bus.
 * @param port Parameter port.
 * @param sda_gpio Parameter sda_gpio.
 * @param scl_gpio Parameter scl_gpio.
 * @param frequency_hz Parameter frequency_hz.
 * @return Return the function result.
 */
  esp_err_t I2cBusInit(i2c_bus_t* bus,
                       i2c_port_t port,
                       int sda_gpio,
                       int scl_gpio,
                       uint32_t frequency_hz);

/**
 * @brief Execute I2cBusDeinit.
 * @param bus Parameter bus.
 * @return Return the function result.
 */
  esp_err_t I2cBusDeinit(i2c_bus_t* bus);

/**
 * @brief Execute I2cBusRecoverLines.
 * @param sda_gpio Parameter sda_gpio.
 * @param scl_gpio Parameter scl_gpio.
 * @return Return the function result.
 */
  esp_err_t I2cBusRecoverLines(int sda_gpio, int scl_gpio);

/**
 * @brief Execute I2cBusLinesLookIdle.
 * @param bus Parameter bus.
 * @return Return the function result.
 */
  bool I2cBusLinesLookIdle(const i2c_bus_t* bus);

/**
 * @brief Capture I2C bus GPIO and controller state.
 * @param bus Parameter bus.
 * @return Return the function result.
 */
  i2c_bus_state_t I2cBusGetState(const i2c_bus_t* bus);

/**
 * @brief Check if I2C bus lines and controller look idle.
 * @param bus Parameter bus.
 * @return Return the function result.
 */
  bool I2cBusLooksIdle(const i2c_bus_t* bus);

/**
 * @brief Log I2C bus GPIO and controller state.
 * @param bus Parameter bus.
 * @param reason Parameter reason.
 */
  void I2cBusLogState(const i2c_bus_t* bus, const char* reason);

/**
 * @brief Execute I2cBusAddDevice.
 * @param bus Parameter bus.
 * @param address Parameter address.
 * @param scl_speed_hz Parameter scl_speed_hz.
 * @param out_device Parameter out_device.
 * @return Return the function result.
 */
  esp_err_t I2cBusAddDevice(i2c_bus_t* bus,
                            uint16_t address,
                            uint32_t scl_speed_hz,
                            i2c_master_dev_handle_t* out_device);

/**
 * @brief Execute I2cBusReadRegister.
 * @param device Parameter device.
 * @param start_register Parameter start_register.
 * @param data_out Parameter data_out.
 * @param length Parameter length.
 * @return Return the function result.
 */
  esp_err_t I2cBusReadRegister(i2c_master_dev_handle_t device,
                               uint8_t start_register,
                               uint8_t* data_out,
                               size_t length);

/**
 * @brief Execute I2cBusWriteRegister.
 * @param device Parameter device.
 * @param start_register Parameter start_register.
 * @param data Parameter data.
 * @param length Parameter length.
 * @return Return the function result.
 */
  esp_err_t I2cBusWriteRegister(i2c_master_dev_handle_t device,
                                uint8_t start_register,
                                const uint8_t* data,
                                size_t length);

/**
 * @brief Execute I2cBusScan.
 * @param bus Parameter bus.
 * @param start_addr Parameter start_addr.
 * @param end_addr Parameter end_addr.
 * @param found_addrs Parameter found_addrs.
 * @param max_found Parameter max_found.
 * @param found_count Parameter found_count.
 * @return Return the function result.
 */
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
