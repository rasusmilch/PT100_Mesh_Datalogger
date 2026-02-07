#include "i2c_bus.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "hal/i2c_ll.h"

static const char* kTag = "i2c_bus";
static const int kI2cRecoveryDelayUs = 5;

/**
 * @brief Execute I2cBusInit.
 * @param bus Parameter bus.
 * @param port Parameter port.
 * @param sda_gpio Parameter sda_gpio.
 * @param scl_gpio Parameter scl_gpio.
 * @param frequency_hz Parameter frequency_hz.
 * @return Return the function result.
 */
esp_err_t
I2cBusInit(i2c_bus_t* bus,
           i2c_port_t port,
           int sda_gpio,
           int scl_gpio,
           uint32_t frequency_hz)
{
  if (bus == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(bus, 0, sizeof(*bus));
  bus->port = port;
  bus->sda_gpio = sda_gpio;
  bus->scl_gpio = scl_gpio;
  bus->frequency_hz = frequency_hz;

  const i2c_master_bus_config_t config = {
    .i2c_port = port,
    .scl_io_num = scl_gpio,
    .sda_io_num = sda_gpio,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags = {
      .enable_internal_pullup = true,
    },
  };

  esp_err_t result = i2c_new_master_bus(&config, &bus->handle);
  if (result != ESP_OK) {
    return result;
  }
  bus->initialized = true;
  return ESP_OK;
}

/**
 * @brief Execute I2cBusDeinit.
 * @param bus Parameter bus.
 * @return Return the function result.
 */
esp_err_t
I2cBusDeinit(i2c_bus_t* bus)
{
  if (bus == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t result = ESP_OK;
  if (bus->initialized && bus->handle != NULL) {
    result = i2c_del_master_bus(bus->handle);
  }

  memset(bus, 0, sizeof(*bus));
  return result;
}

/**
 * @brief Execute I2cBusRecoverLines.
 * @param sda_gpio Parameter sda_gpio.
 * @param scl_gpio Parameter scl_gpio.
 * @return Return the function result.
 */
esp_err_t
I2cBusRecoverLines(int sda_gpio, int scl_gpio)
{
  if (sda_gpio < 0 || scl_gpio < 0) {
    return ESP_ERR_INVALID_ARG;
  }

  gpio_config_t config = {
    .pin_bit_mask = (1ULL << sda_gpio) | (1ULL << scl_gpio),
    .mode = GPIO_MODE_INPUT_OUTPUT_OD,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t result = gpio_config(&config);
  if (result != ESP_OK) {
    return result;
  }

  gpio_set_level(sda_gpio, 1);
  gpio_set_level(scl_gpio, 1);
  esp_rom_delay_us(kI2cRecoveryDelayUs);

  for (int pulse = 0; pulse < 9; ++pulse) {
    gpio_set_level(scl_gpio, 0);
    esp_rom_delay_us(kI2cRecoveryDelayUs);
    gpio_set_level(scl_gpio, 1);
    esp_rom_delay_us(kI2cRecoveryDelayUs);
  }

  gpio_set_level(sda_gpio, 0);
  esp_rom_delay_us(kI2cRecoveryDelayUs);
  gpio_set_level(scl_gpio, 1);
  esp_rom_delay_us(kI2cRecoveryDelayUs);
  gpio_set_level(sda_gpio, 1);
  esp_rom_delay_us(kI2cRecoveryDelayUs);

  return ESP_OK;
}

/**
 * @brief Execute I2cBusLinesLookIdle.
 * @param bus Parameter bus.
 * @return Return the function result.
 */
bool
I2cBusLinesLookIdle(const i2c_bus_t* bus)
{
  if (bus == NULL) {
    return false;
  }
  if (bus->sda_gpio < 0 || bus->scl_gpio < 0) {
    return false;
  }
  const int sda_level = gpio_get_level(bus->sda_gpio);
  const int scl_level = gpio_get_level(bus->scl_gpio);
  return (sda_level != 0) && (scl_level != 0);
}

/**
 * @brief Capture I2C bus GPIO and controller state.
 * @param bus Parameter bus.
 * @return Return the function result.
 */
i2c_bus_state_t
I2cBusGetState(const i2c_bus_t* bus)
{
  i2c_bus_state_t state = {
    .scl_level = -1,
    .sda_level = -1,
    .controller_busy = false,
  };
  if (bus == NULL) {
    return state;
  }
  if (bus->scl_gpio >= 0) {
    state.scl_level = gpio_get_level(bus->scl_gpio);
  }
  if (bus->sda_gpio >= 0) {
    state.sda_level = gpio_get_level(bus->sda_gpio);
  }
  if (bus->initialized) {
    i2c_dev_t* hw = I2C_LL_GET_HW(bus->port);
    state.controller_busy = i2c_ll_is_bus_busy(hw);
  }
  return state;
}

/**
 * @brief Check if I2C bus lines and controller look idle.
 * @param bus Parameter bus.
 * @return Return the function result.
 */
bool
I2cBusLooksIdle(const i2c_bus_t* bus)
{
  if (bus == NULL || !bus->initialized) {
    return false;
  }
  const i2c_bus_state_t state = I2cBusGetState(bus);
  return state.sda_level > 0 && state.scl_level > 0 &&
         !state.controller_busy;
}

/**
 * @brief Log I2C bus GPIO and controller state.
 * @param bus Parameter bus.
 * @param reason Parameter reason.
 */
void
I2cBusLogState(const i2c_bus_t* bus, const char* reason)
{
  const i2c_bus_state_t state = I2cBusGetState(bus);
  const char* label = (reason != NULL) ? reason : "unknown";
  const int port = (bus != NULL) ? (int)bus->port : -1;
  const int sda_pin = (bus != NULL) ? bus->sda_gpio : -1;
  const int scl_pin = (bus != NULL) ? bus->scl_gpio : -1;
  ESP_LOGW(kTag,
           "I2C bus state(%s): port=%d sda_pin=%d scl_pin=%d sda_level=%d "
           "scl_level=%d controller_busy=%u",
           label,
           port,
           sda_pin,
           scl_pin,
           state.sda_level,
           state.scl_level,
           state.controller_busy ? 1u : 0u);
}

/**
 * @brief Execute I2cBusAddDevice.
 * @param bus Parameter bus.
 * @param address Parameter address.
 * @param scl_speed_hz Parameter scl_speed_hz.
 * @param out_device Parameter out_device.
 * @return Return the function result.
 */
esp_err_t
I2cBusAddDevice(i2c_bus_t* bus,
                uint16_t address,
                uint32_t scl_speed_hz,
                i2c_master_dev_handle_t* out_device)
{
  if (bus == NULL || out_device == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!bus->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  const i2c_device_config_t config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = address,
    .scl_speed_hz = (scl_speed_hz > 0) ? scl_speed_hz : bus->frequency_hz,
  };
  return i2c_master_bus_add_device(bus->handle, &config, out_device);
}

/**
 * @brief Execute I2cBusReadRegister.
 * @param device Parameter device.
 * @param start_register Parameter start_register.
 * @param data_out Parameter data_out.
 * @param length Parameter length.
 * @return Return the function result.
 */
esp_err_t
I2cBusReadRegister(i2c_master_dev_handle_t device,
                   uint8_t start_register,
                   uint8_t* data_out,
                   size_t length)
{
  if (device == NULL || data_out == NULL || length == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  return i2c_master_transmit_receive(device,
                                     &start_register,
                                     1,
                                     data_out,
                                     length,
                                     I2C_BUS_TRANSACTION_TIMEOUT_MS);
}

/**
 * @brief Execute I2cBusWriteRegister.
 * @param device Parameter device.
 * @param start_register Parameter start_register.
 * @param data Parameter data.
 * @param length Parameter length.
 * @return Return the function result.
 */
esp_err_t
I2cBusWriteRegister(i2c_master_dev_handle_t device,
                    uint8_t start_register,
                    const uint8_t* data,
                    size_t length)
{
  if (device == NULL || data == NULL || length == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t buffer[1 + length];
  buffer[0] = start_register;
  memcpy(&buffer[1], data, length);
  return i2c_master_transmit(
    device, buffer, sizeof(buffer), I2C_BUS_TRANSACTION_TIMEOUT_MS);
}

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
esp_err_t
I2cBusScan(const i2c_bus_t* bus,
           uint8_t start_addr,
           uint8_t end_addr,
           uint8_t* found_addrs,
           size_t max_found,
           size_t* found_count)
{
  if (bus == NULL || !bus->initialized || start_addr > end_addr) {
    return ESP_ERR_INVALID_ARG;
  }
  if (found_count != NULL) {
    *found_count = 0;
  }

  esp_err_t result = ESP_OK;
  size_t found = 0;
  for (uint16_t addr = start_addr; addr <= end_addr; ++addr) {
    esp_err_t probe_result =
      i2c_master_probe(bus->handle, addr, I2C_BUS_TRANSACTION_TIMEOUT_MS);
    if (probe_result == ESP_OK) {
      if (found < max_found && found_addrs != NULL) {
        found_addrs[found] = (uint8_t)addr;
      }
      found++;
      continue;
    }
    if (probe_result != ESP_ERR_NOT_FOUND) {
      result = probe_result;
      break;
    }
  }
  if (found_count != NULL) {
    *found_count = found;
  }
  return result;
}
