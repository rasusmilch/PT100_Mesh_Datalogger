#include "data_port.h"

#include "driver/uart.h"
#include "esp_log.h"

static const char* kTag = "data_port";
static bool g_initialized = false;
static const int kRxBufferLen = 256;
static const int kTxBufferLen = 2048;

esp_err_t
DataPortInit(void)
{
  if (g_initialized) {
    return ESP_OK;
  }

  const uart_config_t config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  // UART0 is reserved for data CSV streaming (USB-to-UART bridge). Keep
  // console/log output on USB Serial/JTAG when available to avoid mixing.
  esp_err_t result = uart_param_config(UART_NUM_0, &config);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "uart_param_config failed: %s", esp_err_to_name(result));
    return result;
  }

  result = uart_set_pin(UART_NUM_0,
                        UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "uart_set_pin failed: %s", esp_err_to_name(result));
    return result;
  }

  // ESP-IDF requires rx_buffer_size > 0 in uart_driver_install(), even if
  // the application never reads from RX (TX-only data stream).
  result =
    uart_driver_install(UART_NUM_0, kRxBufferLen, kTxBufferLen, 0, NULL, 0);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "uart_driver_install failed: %s", esp_err_to_name(result));
    return result;
  }

  g_initialized = true;
  return ESP_OK;
}

esp_err_t
DataPortWrite(const char* bytes, size_t len, size_t* bytes_written)
{
  if (bytes_written != NULL) {
    *bytes_written = 0;
  }
  if (bytes == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!g_initialized) {
    esp_err_t init_result = DataPortInit();
    if (init_result != ESP_OK) {
      return init_result;
    }
  }
  const int written = uart_write_bytes(UART_NUM_0, bytes, len);
  if (written < 0) {
    return ESP_FAIL;
  }
  if (bytes_written != NULL) {
    *bytes_written = (size_t)written;
  }
  if ((size_t)written != len) {
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}
