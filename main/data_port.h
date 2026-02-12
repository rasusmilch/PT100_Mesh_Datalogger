#ifndef PT100_LOGGER_DATA_PORT_H_
#define PT100_LOGGER_DATA_PORT_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum
{
  DATA_PORT_BACKEND_UART0 = 0,
  DATA_PORT_BACKEND_USB_SERIAL_JTAG,
} DataPortBackend;

/**
 * @brief Execute DataPortInit.
 * @return Return the function result.
 */
esp_err_t DataPortInit(void);

DataPortBackend DataPortGetBackend(void);

const char* DataPortBackendToString(DataPortBackend backend);

/**
 * @brief Returns the configured UART0 pins used for the data stream backend.
 *
 * @param[out] tx_gpio GPIO number for UART0 TX, or -1 if not configured.
 * @param[out] rx_gpio GPIO number for UART0 RX, or -1 if not configured.
 */
void DataPortGetUart0Pins(int32_t* tx_gpio, int32_t* rx_gpio);

/**
 * @brief Execute DataPortWrite.
 * @param bytes Parameter bytes.
 * @param len Parameter len.
 * @param bytes_written Parameter bytes_written.
 * @return Return the function result.
 */
esp_err_t DataPortWrite(const char* bytes, size_t len, size_t* bytes_written);

#endif // PT100_LOGGER_DATA_PORT_H_
