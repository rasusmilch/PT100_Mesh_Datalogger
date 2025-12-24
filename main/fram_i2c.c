#include "fram_i2c.h"

#include <string.h>

#include "esp_log.h"

static const char* kTag = "fram_i2c";
static const int kI2cTimeoutMs = 100;
static const size_t kMaxChunk = 96;
static const uint8_t kReservedIdAddr7bit = 0x7C; // 0xF8/0xF9 in 8-bit form.

static void
EncodeAddress(uint16_t addr, uint8_t out[2])
{
  out[0] = (uint8_t)((addr >> 8) & 0x7Fu);
  out[1] = (uint8_t)(addr & 0xFFu);
}

static bool
BoundsOk(const fram_i2c_t* fram, uint16_t addr, size_t len)
{
  if (fram == NULL || !fram->initialized) {
    return false;
  }
  if (len > fram->fram_size_bytes) {
    return false;
  }
  if ((size_t)addr > fram->fram_size_bytes - len) {
    return false;
  }
  const size_t end = (size_t)addr + len;
  return (addr < 0x8000u) && (end <= 0x8000u);
}

esp_err_t
FramI2cInit(fram_i2c_t* fram,
            i2c_master_bus_handle_t bus,
            uint8_t i2c_addr_7bit,
            size_t fram_size_bytes,
            uint32_t scl_speed_hz)
{
  if (fram == NULL || bus == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (i2c_addr_7bit > 0x7Fu || fram_size_bytes == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(fram, 0, sizeof(*fram));
  fram->bus = bus;
  fram->i2c_addr_7bit = i2c_addr_7bit;
  fram->fram_size_bytes = fram_size_bytes;
  if (scl_speed_hz == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  fram->scl_speed_hz = scl_speed_hz;

  const i2c_device_config_t config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = i2c_addr_7bit,
    .scl_speed_hz = scl_speed_hz,
  };
  esp_err_t result = i2c_master_bus_add_device(bus, &config, &fram->device);
  if (result != ESP_OK) {
    ESP_LOGE(kTag,
             "i2c_master_bus_add_device addr=0x%02x failed: %s",
             (unsigned)i2c_addr_7bit,
             esp_err_to_name(result));
    return result;
  }
  fram->initialized = true;
  return ESP_OK;
}

esp_err_t
FramI2cRead(const fram_i2c_t* fram, uint16_t addr, void* out, size_t len)
{
  if (fram == NULL || out == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!fram->initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!BoundsOk(fram, addr, len)) {
    return ESP_ERR_INVALID_SIZE;
  }

  uint8_t* dest = (uint8_t*)out;
  size_t remaining = len;
  uint16_t offset = addr;

  while (remaining > 0) {
    const size_t chunk = (remaining > kMaxChunk) ? kMaxChunk : remaining;
    uint8_t address_bytes[2] = { 0 };
    EncodeAddress(offset, address_bytes);
    const esp_err_t result = i2c_master_transmit_receive(
      fram->device, address_bytes, sizeof(address_bytes), dest, chunk, kI2cTimeoutMs);
    if (result != ESP_OK) {
      return result;
    }
    dest += chunk;
    remaining -= chunk;
    offset = (uint16_t)(offset + chunk);
  }
  return ESP_OK;
}

esp_err_t
FramI2cWrite(const fram_i2c_t* fram,
             uint16_t addr,
             const void* data,
             size_t len)
{
  if (fram == NULL || data == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!fram->initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!BoundsOk(fram, addr, len)) {
    return ESP_ERR_INVALID_SIZE;
  }

  const uint8_t* src = (const uint8_t*)data;
  size_t remaining = len;
  uint16_t offset = addr;

  uint8_t buffer[2 + kMaxChunk];
  while (remaining > 0) {
    const size_t chunk = (remaining > kMaxChunk) ? kMaxChunk : remaining;
    EncodeAddress(offset, buffer);
    memcpy(&buffer[2], src, chunk);

    const esp_err_t result =
      i2c_master_transmit(fram->device, buffer, 2 + chunk, kI2cTimeoutMs);
    if (result != ESP_OK) {
      return result;
    }
    src += chunk;
    remaining -= chunk;
    offset = (uint16_t)(offset + chunk);
  }
  return ESP_OK;
}

esp_err_t
FramI2cReadDeviceId(const fram_i2c_t* fram, fram_device_id_t* out)
{
  if (fram == NULL || out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!fram->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  i2c_master_dev_handle_t id_device = NULL;
  const i2c_device_config_t config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = kReservedIdAddr7bit,
    .scl_speed_hz = fram->scl_speed_hz,
  };
  esp_err_t result =
    i2c_master_bus_add_device(fram->bus, &config, &id_device);
  if (result != ESP_OK) {
    return result;
  }

  const uint8_t device_addr_word = (uint8_t)(fram->i2c_addr_7bit << 1);
  memset(out, 0, sizeof(*out));
  result = i2c_master_transmit_receive(
    id_device, &device_addr_word, 1, out->raw, sizeof(out->raw), kI2cTimeoutMs);
  (void)i2c_master_bus_rm_device(id_device);
  if (result != ESP_OK) {
    return result;
  }

  out->manufacturer_id =
    (uint16_t)((out->raw[0] << 4) | (uint16_t)(out->raw[1] >> 4));
  out->product_id =
    (uint16_t)(((uint16_t)(out->raw[1] & 0x0Fu) << 8) | out->raw[2]);
  return ESP_OK;
}
