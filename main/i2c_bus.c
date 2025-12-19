#include "i2c_bus.h"

#include <string.h>

#include "esp_err.h"

static const int kI2cTimeoutMs = 100;

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

esp_err_t
I2cBusReadRegister(i2c_master_dev_handle_t device,
                   uint8_t start_register,
                   uint8_t* data_out,
                   size_t length)
{
  if (device == NULL || data_out == NULL || length == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  return i2c_master_transmit_receive(
    device, &start_register, 1, data_out, length, kI2cTimeoutMs);
}

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
  return i2c_master_transmit(device, buffer, sizeof(buffer), kI2cTimeoutMs);
}

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
    esp_err_t probe_result = i2c_master_probe(bus->handle, addr, kI2cTimeoutMs);
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
