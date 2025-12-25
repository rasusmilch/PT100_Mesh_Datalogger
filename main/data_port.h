#ifndef PT100_LOGGER_DATA_PORT_H_
#define PT100_LOGGER_DATA_PORT_H_

#include <stddef.h>

#include "esp_err.h"

esp_err_t DataPortInit(void);
void DataPortWrite(const char* bytes, size_t len);

#endif // PT100_LOGGER_DATA_PORT_H_
