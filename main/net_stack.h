#ifndef PT100_LOGGER_NET_STACK_H_
#define PT100_LOGGER_NET_STACK_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the ESP net stack (esp_netif + default event loop) exactly once.
// Subsequent calls are idempotent; ESP_ERR_INVALID_STATE from the underlying
// calls is treated as success.
esp_err_t NetStackInitOnce(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_NET_STACK_H_
