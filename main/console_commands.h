#ifndef PT100_LOGGER_CONSOLE_COMMANDS_H_
#define PT100_LOGGER_CONSOLE_COMMANDS_H_

#include "app_settings.h"
#include "esp_err.h"
#include "fram_log.h"
#include "max31865_reader.h"
#include "mesh_transport.h"
#include "sd_logger.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    app_settings_t* settings;
    fram_log_t* fram_log;
    sd_logger_t* sd_logger;
    max31865_reader_t* sensor;
    mesh_transport_t* mesh;
    time_sync_t* time_sync;
    const char* node_id_string;
    esp_err_t (*flush_callback)(void* context);
    void* flush_context;
  } app_runtime_t;

  // Initializes and starts the serial console.
  esp_err_t ConsoleCommandsStart(app_runtime_t* runtime);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_CONSOLE_COMMANDS_H_
