#ifndef PT100_LOGGER_RUNTIME_MANAGER_H_
#define PT100_LOGGER_RUNTIME_MANAGER_H_

#include <stdbool.h>

#include "app_settings.h"
#include "esp_err.h"
#include "fram_log.h"
#include "i2c_bus.h"
#include "max31865_reader.h"
#include "mesh_transport.h"
#include "sd_logger.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    app_settings_t* settings;
    fram_log_t* fram_log;
    sd_logger_t* sd_logger;
    max31865_reader_t* sensor;
    mesh_transport_t* mesh;
    time_sync_t* time_sync;
    i2c_bus_t* i2c_bus;
    const char* node_id_string;
    esp_err_t (*flush_callback)(void* context);
    void* flush_context;
    bool* fram_full;
  } app_runtime_t;

  esp_err_t RuntimeManagerInit(void);

  const app_runtime_t* RuntimeGetRuntime(void);

  esp_err_t RuntimeStart(void);

  esp_err_t RuntimeStop(void);

  bool RuntimeIsRunning(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUNTIME_MANAGER_H_
