#ifndef PT100_LOGGER_SD_LOGGER_H_
#define PT100_LOGGER_SD_LOGGER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "ptlog_format.h"
#include "sd_csv_verify.h"
#include "sd_ptlog_paths.h"
#include "sdmmc_cmd.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SD_LOGGER_DAILY_DIAG_PATH_LEN 128u
#define SD_LOGGER_DAILY_DIAG_DATE_LEN 16u
#define SD_LOGGER_DAILY_DIAG_MONTH_LEN 16u

  /** Stage labels for the last daily PTLOG path/directory/open failure. */
  typedef enum
  {
    SD_LOGGER_DAILY_STAGE_NONE = 0,
    SD_LOGGER_DAILY_STAGE_PATH_BUILD,
    SD_LOGGER_DAILY_STAGE_LOG_ROOT_BUILD,
    SD_LOGGER_DAILY_STAGE_LOG_ROOT_STAT,
    SD_LOGGER_DAILY_STAGE_LOG_ROOT_MKDIR,
    SD_LOGGER_DAILY_STAGE_LOG_ROOT_VERIFY,
    SD_LOGGER_DAILY_STAGE_MONTH_DIR_BUILD,
    SD_LOGGER_DAILY_STAGE_MONTH_DIR_STAT,
    SD_LOGGER_DAILY_STAGE_MONTH_DIR_MKDIR,
    SD_LOGGER_DAILY_STAGE_MONTH_DIR_VERIFY,
    SD_LOGGER_DAILY_STAGE_REVISION_DIR_BUILD,
    SD_LOGGER_DAILY_STAGE_REVISION_DIR_OPEN,
    SD_LOGGER_DAILY_STAGE_REVISION_FILE_STAT,
    SD_LOGGER_DAILY_STAGE_REVISION_OVERFLOW,
    SD_LOGGER_DAILY_STAGE_ACCESS_EXISTING,
    SD_LOGGER_DAILY_STAGE_FOPEN_DAILY,
    SD_LOGGER_DAILY_STAGE_RESUME,
    SD_LOGGER_DAILY_STAGE_HEADER_COMMIT,
    SD_LOGGER_DAILY_STAGE_EMPTY_FILE_UNLINK,
  } sd_logger_daily_stage_t;

  /** Fixed-size daily PTLOG diagnostics; strings are copied, never borrowed. */
  typedef struct
  {
    sd_logger_daily_stage_t stage;
    char path[SD_LOGGER_DAILY_DIAG_PATH_LEN];
    char date[SD_LOGGER_DAILY_DIAG_DATE_LEN];
    char month[SD_LOGGER_DAILY_DIAG_MONTH_LEN];
    uint32_t revision;
    int errno_value;
    esp_err_t result;
    bool file_existed_before_open;
    bool file_was_empty;
    bool empty_file_unlink_attempted;
    bool empty_file_unlink_succeeded;
  } SdLoggerDailyDiagnostics;

  typedef struct
  {
    size_t batch_target_bytes;
    size_t tail_scan_bytes;
    size_t file_buffer_bytes;
    uint32_t max_freq_khz;
  } sd_logger_config_t;

  typedef struct
  {
    bool is_mounted;
    sdmmc_card_t* card;
    char mount_point[16]; // e.g. "/sdcard"
    bool space_reclaim_active;
    uint32_t space_reclaim_deleted_total;

    FILE* file;
    char current_date[16]; // YYYY-MM-DDZ
    uint32_t current_header_signature;
    const ptlog_header_t* pending_header;
    uint64_t last_record_id_on_sd;
    uint8_t* file_buffer;
    uint8_t* resume_tail_bytes;
    size_t resume_tail_capacity;

    sd_logger_config_t config;
    uint8_t* io_bounce_bytes;
    size_t io_bounce_capacity;
    uint8_t* verify_readback_bytes;
    size_t verify_readback_capacity;
    /**
     * Logger-owned SD/PTLOG path workspace.
     *
     * Target firmware allocates this once from PSRAM-capable 8-bit heap in
     * SdLoggerInit and runtime PTLOG helpers fail closed when it is absent.
     * Access is serialized by the logger/SD operation; pointers into the
     * workspace must not escape the active operation.
     */
    sd_ptlog_path_workspace_t* ptlog_workspace;

    // Saved slot configuration so we can retry mounting on hot-insert.
    spi_host_device_t host_id;
    int cs_gpio;
    bool slot_config_valid;

    SdLoggerDailyDiagnostics last_daily_diag;
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
    uint32_t space_reclaim_deleted_files;
    uint64_t space_free_bytes_before;
    uint64_t space_free_bytes_after;
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

/**
 * @brief Reset SD logger mount-derived state without touching durable metadata.
 * @param logger Parameter logger.
 */
  void SdLoggerResetMountState(sd_logger_t* logger);

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

  // Open/create the UTC daily PTLOG for the provided epoch. Repairs tail and
  // updates last_record_id_on_sd.
/**
 * @brief Execute SdLoggerEnsureDailyFile.
 * @param logger Parameter logger.
 * @param epoch_utc Parameter epoch_utc.
 * @return Return the function result.
 */
  esp_err_t SdLoggerEnsureDailyFile(sd_logger_t* logger, int64_t epoch_utc);

/**
 * @brief Execute SdLoggerEnsureDailyFileWithHeader.
 * @param logger Parameter logger.
 * @param epoch_utc Parameter epoch_utc.
 * @param header Parameter header.
 * @param header_signature Parameter header_signature.
 * @return Return the function result.
 */
  esp_err_t SdLoggerEnsureDailyFileWithHeader(sd_logger_t* logger,
                                              int64_t epoch_utc,
                                              const ptlog_header_t* header,
                                              uint32_t header_signature);

  const char* SdLoggerDailyStageName(sd_logger_daily_stage_t stage);

  /** Copy the last daily PTLOG failure diagnostics, if any, into diag_out. */
  bool SdLoggerGetLastDailyDiagnostics(
    const sd_logger_t* logger, SdLoggerDailyDiagnostics* diag_out);

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

/**
 * @brief Return true if SD log space reclaim (oldest-file rotation) has occurred.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
  bool SdLoggerSpaceReclaimActive(const sd_logger_t* logger);

/**
 * @brief Return total number of daily PTLOG files deleted to reclaim space.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
  uint32_t SdLoggerSpaceReclaimDeletedTotal(const sd_logger_t* logger);

/**
 * @brief Query SD card filesystem capacity and currently available free bytes.
 * @param logger Parameter logger.
 * @param total_bytes Parameter total_bytes.
 * @param free_bytes Parameter free_bytes.
 * @return Return the function result.
 */
  esp_err_t SdLoggerGetSpaceInfo(const sd_logger_t* logger,
                                 uint64_t* total_bytes,
                                 uint64_t* free_bytes);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_LOGGER_H_
