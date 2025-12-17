#ifndef PT100_LOGGER_SD_LOGGER_H_
#define PT100_LOGGER_SD_LOGGER_H_

#include <stdbool.h>

#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "log_record.h"
#include "sdmmc_cmd.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    bool is_mounted;
    sdmmc_card_t* card;
    char mount_point[16]; // e.g. "/sdcard"
  } sd_logger_t;

  // Mount SD card over SPI (FATFS).
  // Assumes SPI bus is already initialized.
  esp_err_t SdLoggerMount(sd_logger_t* logger,
                          spi_host_device_t host,
                          int cs_gpio);

  // Append one record to a YYYYMMDD CSV file.
  esp_err_t SdLoggerAppendCsv(sd_logger_t* logger,
                              const char* node_id,
                              const log_record_t* record);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_LOGGER_H_
