#include "fram_spi.h"

#include "freertos/FreeRTOS.h"
#include <string.h>

#include "esp_log.h"

static const char* kTag = "fram_spi";

// Common SPI FRAM opcodes (MB85RS / FM25* families).
static const uint8_t kOpcodeWren = 0x06;
static const uint8_t kOpcodeRead = 0x03;
static const uint8_t kOpcodeWrite = 0x02;

static esp_err_t
SpiTx(const fram_spi_t* fram,
      const void* tx_data,
      size_t tx_len,
      void* rx_data,
      size_t rx_len)
{
  spi_transaction_t transaction;
  memset(&transaction, 0, sizeof(transaction));
  transaction.length = tx_len * 8;
  transaction.tx_buffer = tx_data;

  if (rx_len > 0) {
    transaction.rxlength = rx_len * 8;
    transaction.rx_buffer = rx_data;
  }
  return spi_device_transmit(fram->device, &transaction);
}

static esp_err_t
WriteEnable(const fram_spi_t* fram)
{
  uint8_t opcode = kOpcodeWren;
  return SpiTx(fram, &opcode, 1, NULL, 0);
}

static void
EncodeAddress(uint8_t* out, int address_bytes, uint32_t address)
{
  // Big-endian address.
  for (int index = 0; index < address_bytes; ++index) {
    const int shift = 8 * (address_bytes - 1 - index);
    out[index] = (uint8_t)((address >> shift) & 0xFFu);
  }
}

esp_err_t
FramSpiInit(fram_spi_t* fram,
            spi_host_device_t host,
            int cs_gpio,
            int address_bytes)
{
  if (fram == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (address_bytes != 2 && address_bytes != 3) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(fram, 0, sizeof(*fram));
  fram->address_bytes = address_bytes;

  spi_device_interface_config_t device_config = {
    .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz (safe default).
    .mode = 0,
    .spics_io_num = cs_gpio,
    .queue_size = 1,
  };

  esp_err_t result = spi_bus_add_device(host, &device_config, &fram->device);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "spi_bus_add_device failed: %s", esp_err_to_name(result));
    return result;
  }
  return ESP_OK;
}

esp_err_t
FramSpiRead(const fram_spi_t* fram,
            uint32_t address,
            void* data_out,
            size_t length_bytes)
{
  if (fram == NULL || data_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  // Tx header: [READ][addr...]
  uint8_t header[1 + 3] = { 0 };
  header[0] = kOpcodeRead;
  EncodeAddress(&header[1], fram->address_bytes, address);

  // Use a two-step transaction: send header, then read data.
  spi_transaction_t transaction_header;
  memset(&transaction_header, 0, sizeof(transaction_header));
  transaction_header.length = (1 + fram->address_bytes) * 8;
  transaction_header.tx_buffer = header;

  spi_transaction_t transaction_data;
  memset(&transaction_data, 0, sizeof(transaction_data));
  transaction_data.length = length_bytes * 8;
  transaction_data.rxlength = length_bytes * 8;
  transaction_data.rx_buffer = data_out;

  esp_err_t result = spi_device_acquire_bus(fram->device, portMAX_DELAY);
  if (result != ESP_OK) {
    return result;
  }
  result = spi_device_transmit(fram->device, &transaction_header);
  if (result == ESP_OK) {
    result = spi_device_transmit(fram->device, &transaction_data);
  }
  spi_device_release_bus(fram->device);
  return result;
}

esp_err_t
FramSpiWrite(const fram_spi_t* fram,
             uint32_t address,
             const void* data,
             size_t length_bytes)
{
  if (fram == NULL || data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  // Header: [WRITE][addr...]
  uint8_t header[1 + 3] = { 0 };
  header[0] = kOpcodeWrite;
  EncodeAddress(&header[1], fram->address_bytes, address);

  spi_transaction_t transaction_header;
  memset(&transaction_header, 0, sizeof(transaction_header));
  transaction_header.length = (1 + fram->address_bytes) * 8;
  transaction_header.tx_buffer = header;

  spi_transaction_t transaction_data;
  memset(&transaction_data, 0, sizeof(transaction_data));
  transaction_data.length = length_bytes * 8;
  transaction_data.tx_buffer = data;

  esp_err_t result = spi_device_acquire_bus(fram->device, portMAX_DELAY);
  if (result != ESP_OK) {
    return result;
  }
  result = WriteEnable(fram);
  if (result == ESP_OK) {
    result = spi_device_transmit(fram->device, &transaction_header);
  }
  if (result == ESP_OK) {
    result = spi_device_transmit(fram->device, &transaction_data);
  }
  spi_device_release_bus(fram->device);
  return result;
}
