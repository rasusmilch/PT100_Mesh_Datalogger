#ifndef PT100_LOGGER_FRAM_IO_H_
#define PT100_LOGGER_FRAM_IO_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    void* context;
    esp_err_t (*read)(void* context, uint32_t addr, void* out, size_t len);
    esp_err_t (*write)(void* context,
                       uint32_t addr,
                       const void* data,
                       size_t len);
  } fram_io_t;

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_IO_H_
