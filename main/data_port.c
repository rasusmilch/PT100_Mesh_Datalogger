#include "data_port.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"

#if defined(SOC_USB_SERIAL_JTAG_SUPPORTED)
#include "soc/soc_caps.h"
#endif

#if defined(SOC_USB_SERIAL_JTAG_SUPPORTED) && SOC_USB_SERIAL_JTAG_SUPPORTED
#include "driver/usb_serial_jtag.h"
#endif

static const char* kTag = "data_port";
static bool g_initialized = false;
static const int kRxBufferLen = 256;
static const int kTxBufferLen = 2048;

static int
MapConfiguredUartGpio_(int32_t configured_gpio);

#if defined(SOC_USB_SERIAL_JTAG_SUPPORTED) && SOC_USB_SERIAL_JTAG_SUPPORTED && \
  !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
static const DataPortBackend kBackend = DATA_PORT_BACKEND_USB_SERIAL_JTAG;
#else
static const DataPortBackend kBackend = DATA_PORT_BACKEND_UART0;
#endif

DataPortBackend
DataPortGetBackend(void)
{
  return kBackend;
}

const char*
DataPortBackendToString(DataPortBackend backend)
{
  switch (backend) {
    case DATA_PORT_BACKEND_UART0:
      return "uart0";
    case DATA_PORT_BACKEND_USB_SERIAL_JTAG:
      return "usb_jtag";
    default:
      return "unknown";
  }
}

/**
 * @brief Returns the configured UART0 pins used for the data stream backend.
 *
 * @param[out] tx_gpio GPIO number for UART0 TX, or -1 if not configured.
 * @param[out] rx_gpio GPIO number for UART0 RX, or -1 if not configured.
 */
void
DataPortGetUart0Pins(int32_t* tx_gpio, int32_t* rx_gpio)
{
  if (tx_gpio != NULL) {
    *tx_gpio = CONFIG_APP_DATA_STREAM_UART_TX_GPIO;
  }
  if (rx_gpio != NULL) {
    *rx_gpio = CONFIG_APP_DATA_STREAM_UART_RX_GPIO;
  }
}

/**
 * @brief Execute DataPortInit.
 * @return Return the function result.
 */
esp_err_t
DataPortInit(void)
{
  if (g_initialized) {
    return ESP_OK;
  }

  if (kBackend == DATA_PORT_BACKEND_USB_SERIAL_JTAG) {
#if defined(SOC_USB_SERIAL_JTAG_SUPPORTED) && SOC_USB_SERIAL_JTAG_SUPPORTED
    usb_serial_jtag_driver_config_t usb_jtag_config = {
      .tx_buffer_size = kTxBufferLen,
      .rx_buffer_size = kRxBufferLen,
    };
    esp_err_t result = usb_serial_jtag_driver_install(&usb_jtag_config);
    if (result != ESP_OK) {
      ESP_LOGE(kTag,
               "usb_serial_jtag_driver_install failed: %s",
               esp_err_to_name(result));
      return result;
    }
    g_initialized = true;
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
  }

  const uart_config_t config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t result = uart_param_config(UART_NUM_0, &config);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "uart_param_config failed: %s", esp_err_to_name(result));
    return result;
  }

  int32_t configured_tx_gpio = -1;
  int32_t configured_rx_gpio = -1;
  DataPortGetUart0Pins(&configured_tx_gpio, &configured_rx_gpio);

  result = uart_set_pin(UART_NUM_0,
                        MapConfiguredUartGpio_(configured_tx_gpio),
                        MapConfiguredUartGpio_(configured_rx_gpio),
                        UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "uart_set_pin failed: %s", esp_err_to_name(result));
    return result;
  }

  result =
    uart_driver_install(UART_NUM_0, kRxBufferLen, kTxBufferLen, 0, NULL, 0);
  if (result != ESP_OK) {
    ESP_LOGE(kTag, "uart_driver_install failed: %s", esp_err_to_name(result));
    return result;
  }

  g_initialized = true;
  return ESP_OK;
}

static int
MapConfiguredUartGpio_(int32_t configured_gpio)
{
  if (configured_gpio < 0) {
    return UART_PIN_NO_CHANGE;
  }
  return configured_gpio;
}

/**
 * @brief Execute DataPortWrite.
 * @param bytes Parameter bytes.
 * @param len Parameter len.
 * @param bytes_written Parameter bytes_written.
 * @return Return the function result.
 */
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

  int written = -1;
  if (kBackend == DATA_PORT_BACKEND_USB_SERIAL_JTAG) {
#if defined(SOC_USB_SERIAL_JTAG_SUPPORTED) && SOC_USB_SERIAL_JTAG_SUPPORTED
    written = usb_serial_jtag_write_bytes(bytes, len, 0);
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
  } else {
    written = uart_write_bytes(UART_NUM_0, bytes, len);
  }
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
