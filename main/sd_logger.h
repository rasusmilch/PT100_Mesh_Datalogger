#ifndef PT100_LOGGER_SD_LOGGER_H_
#define PT100_LOGGER_SD_LOGGER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "sd_csv_verify.h"
#include "sdmmc_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  size_t batch_target_bytes;
  size_t tail_scan_bytes;
  size_t file_buffer_bytes;
} sd_logger_config_t;

typedef struct
{
  bool is_mounted;
  sdmmc_card_t* card;
  char mount_point[16]; // e.g. "/sdcard"

  FILE* file;
  char current_date[16]; // YYYY-MM-DD
  uint64_t last_record_id_on_sd;
  uint8_t* file_buffer;

  sd_logger_config_t config;

  // Saved slot configuration so we can retry mounting on hot-insert.
  spi_host_device_t host_id;
  int cs_gpio;
  bool slot_config_valid;
} sd_logger_t;

void SdLoggerInit(sd_logger_t* logger, const sd_logger_config_t* config);

// Mount SD card over SPI (FATFS).
// Assumes SPI bus is already initialized.
esp_err_t SdLoggerMount(sd_logger_t* logger,
                        spi_host_device_t host,
                        int cs_gpio);

// Retry mount using the last host/cs passed to SdLoggerMount().
// If format_if_mount_failed is true, the card may be formatted (destructive).
esp_err_t SdLoggerTryRemount(sd_logger_t* logger, bool format_if_mount_failed);

// Close any open file and unmount the SD card.
esp_err_t SdLoggerUnmount(sd_logger_t* logger);

// Open/create the UTC daily CSV for the provided epoch. Repairs tail and
// updates last_record_id_on_sd.
esp_err_t SdLoggerEnsureDailyFile(sd_logger_t* logger, int64_t epoch_utc);

// Append a verified batch (already formatted CSV) and update last_record_id_on_sd.
esp_err_t SdLoggerAppendVerifiedBatch(sd_logger_t* logger,
                                      const uint8_t* batch_bytes,
                                      size_t batch_length_bytes,
                                      uint64_t last_record_id_in_batch,
                                      SdCsvAppendDiagnostics* diag_out);

void SdLoggerClose(sd_logger_t* logger);

static inline uint64_t
SdLoggerLastRecordIdOnSd(const sd_logger_t* logger)
{
  return (logger == NULL) ? 0 : logger->last_record_id_on_sd;
}

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_LOGGER_H_
