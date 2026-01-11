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
extern "C"
{
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
    uint8_t* resume_tail_bytes;
    size_t resume_tail_capacity;

    sd_logger_config_t config;
    uint8_t* io_bounce_bytes;
    size_t io_bounce_capacity;
    uint8_t* verify_readback_bytes;
    size_t verify_readback_capacity;

    // Saved slot configuration so we can retry mounting on hot-insert.
    spi_host_device_t host_id;
    int cs_gpio;
    bool slot_config_valid;
  } sd_logger_t;

  typedef enum
  {
    SD_APPEND_VERIFY_NONE = 0,
    SD_APPEND_VERIFY_READBACK_SHA256,
  } sd_append_verify_t;

  typedef enum
  {
    SD_APPEND_FLUSH_NEVER = 0,
    SD_APPEND_FLUSH_ALWAYS,
  } sd_append_flush_t;

  typedef struct
  {
    size_t bytes_appended;
    size_t write_calls;
    SdCsvAppendDiagnostics diag;
  } sd_csv_append_stats_t;

/**
 * @brief Execute SdLoggerInit.
 * @param logger Parameter logger.
 * @param config Parameter config.
 */
  void SdLoggerInit(sd_logger_t* logger, const sd_logger_config_t* config);

  // Mount SD card over SPI (FATFS).
  // Assumes SPI bus is already initialized.
/**
 * @brief Execute SdLoggerMount.
 * @param logger Parameter logger.
 * @param host Parameter host.
 * @param cs_gpio Parameter cs_gpio.
 * @return Return the function result.
 */
  esp_err_t SdLoggerMount(sd_logger_t* logger,
                          spi_host_device_t host,
                          int cs_gpio);

  // Retry mount using the last host/cs passed to SdLoggerMount().
  // If format_if_mount_failed is true, the card may be formatted (destructive).
/**
 * @brief Execute SdLoggerTryRemount.
 * @param logger Parameter logger.
 * @param format_if_mount_failed Parameter format_if_mount_failed.
 * @return Return the function result.
 */
  esp_err_t SdLoggerTryRemount(sd_logger_t* logger,
                               bool format_if_mount_failed);

  // Close any open file and unmount the SD card.
/**
 * @brief Execute SdLoggerUnmount.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
  esp_err_t SdLoggerUnmount(sd_logger_t* logger);

  // Destructively format the SD card (create a fresh FAT filesystem) and leave
  // it mounted.
  //
  // Handles blank/unformatted cards, corrupted filesystems, and healthy cards
  // that need to be reset to a fresh state.
  //
  // NOTE: This recreates filesystem structures (FAT tables, root dir, boot
  // sector). It is not a secure erase of all flash blocks (SD wear leveling).
/**
 * @brief Execute SdLoggerFormatDestructive.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
  esp_err_t SdLoggerFormatDestructive(sd_logger_t* logger);

  // Open/create the UTC daily CSV for the provided epoch. Repairs tail and
  // updates last_record_id_on_sd.
/**
 * @brief Execute SdLoggerEnsureDailyFile.
 * @param logger Parameter logger.
 * @param epoch_utc Parameter epoch_utc.
 * @return Return the function result.
 */
  esp_err_t SdLoggerEnsureDailyFile(sd_logger_t* logger, int64_t epoch_utc);

  esp_err_t SdLoggerAppendBatchEx(
    sd_logger_t* logger,
    const uint8_t* batch_bytes,
    size_t batch_length_bytes,
    uint64_t last_record_id,
    sd_append_verify_t verify_mode,
    sd_append_flush_t flush_mode,
    const sd_csv_append_scratch_t* scratch,
    sd_csv_append_stats_t* out_stats);

  // Append a verified batch (already formatted CSV) and update
  // last_record_id_on_sd.
/**
 * @brief Execute SdLoggerAppendVerifiedBatch.
 * @param logger Parameter logger.
 * @param batch_bytes Parameter batch_bytes.
 * @param batch_length_bytes Parameter batch_length_bytes.
 * @param last_record_id_in_batch Parameter last_record_id_in_batch.
 * @param diag_out Parameter diag_out.
 * @return Return the function result.
 */
  esp_err_t SdLoggerAppendVerifiedBatch(sd_logger_t* logger,
                                        const uint8_t* batch_bytes,
                                        size_t batch_length_bytes,
                                        uint64_t last_record_id_in_batch,
                                        SdCsvAppendDiagnostics* diag_out);

/**
 * @brief Execute SdLoggerClose.
 * @param logger Parameter logger.
 */
  void SdLoggerClose(sd_logger_t* logger);

/**
 * @brief Execute SdLoggerFlushAndSync.
 * @param logger Parameter logger.
 * @param diag_out Parameter diag_out.
 * @return Return the function result.
 */
  esp_err_t SdLoggerFlushAndSync(sd_logger_t* logger,
                                 SdCsvAppendDiagnostics* diag_out);

/**
 * @brief Execute SdLoggerLastRecordIdOnSd.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
  static inline uint64_t SdLoggerLastRecordIdOnSd(const sd_logger_t* logger)
  {
    return (logger == NULL) ? 0 : logger->last_record_id_on_sd;
  }

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_LOGGER_H_
