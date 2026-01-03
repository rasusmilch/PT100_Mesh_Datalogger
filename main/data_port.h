#ifndef PT100_LOGGER_DATA_PORT_H_
#define PT100_LOGGER_DATA_PORT_H_

#include <stddef.h>

#include "esp_err.h"

/**
 * @brief Execute DataPortInit.
 * @return Return the function result.
 */
esp_err_t DataPortInit(void);
/**
 * @brief Execute DataPortWrite.
 * @param bytes Parameter bytes.
 * @param len Parameter len.
 * @param bytes_written Parameter bytes_written.
 * @return Return the function result.
 */
esp_err_t DataPortWrite(const char* bytes, size_t len, size_t* bytes_written);

#endif // PT100_LOGGER_DATA_PORT_H_
