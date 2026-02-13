#include "sd_logger.h"

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "data_csv.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

static const char* kTag = "sd_logger";

static bool IsStrictDailyCsvName(const char* name);
static esp_err_t SdLoggerGetSpaceInfoLocked(const sd_logger_t* logger,
                                             uint64_t* total_bytes,
                                             uint64_t* free_bytes);
static bool SdLoggerFindOldestDailyCsvLocked(const sd_logger_t* logger,
                                              char* oldest_path,
                                              size_t oldest_path_size);
static esp_err_t SdLoggerReclaimSpaceLocked(sd_logger_t* logger,
                                             uint64_t required_free_bytes,
                                             uint32_t* deleted_files);

static uint8_t*
AllocatePreferPsram(size_t bytes)
{
  uint8_t* buffer = (uint8_t*)heap_caps_malloc(
    bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (buffer != NULL) {
    return buffer;
  }
  return (uint8_t*)heap_caps_malloc(
    bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

/**
 * @brief Execute DefaultOr.
 * @param value Parameter value.
 * @param fallback Parameter fallback.
 * @return Return the function result.
 */
static size_t
DefaultOr(const size_t value, const size_t fallback)
{
  return (value == 0) ? fallback : value;
}

/**
 * @brief Execute SetAppendDiagnostics.
 * @param diag Parameter diag.
 * @param operation Parameter operation.
 * @param errno_value Parameter errno_value.
 */
static void
SetAppendDiagnostics(SdCsvAppendDiagnostics* diag,
                     const char* operation,
                     int errno_value)
{
  if (diag == NULL) {
    return;
  }
  diag->operation = operation;
  diag->errno_value = errno_value;
}

/**
 * @brief Execute ResetAppendStats.
 * @param stats Parameter stats.
 */
static void
ResetAppendStats(sd_csv_append_stats_t* stats)
{
  if (stats == NULL) {
    return;
  }
  memset(stats, 0, sizeof(*stats));
  SetAppendDiagnostics(&stats->diag, NULL, 0);
}

/**
 * @brief Execute CsvFileWriter.
 * @param bytes Parameter bytes.
 * @param len Parameter len.
 * @param context Parameter context.
 * @return Return the function result.
 */
static bool
CsvFileWriter(const char* bytes, size_t len, void* context)
{
  FILE* file = (FILE*)context;
  return SdCsvAppendBatchWithReadbackVerify(
           file, (const uint8_t*)bytes, len, NULL) == ESP_OK;
}

/**
 * @brief Execute SdLoggerInit.
 * @param logger Parameter logger.
 * @param config Parameter config.
 */
void
SdLoggerInit(sd_logger_t* logger, const sd_logger_config_t* config)
{
  if (logger == NULL) {
    return;
  }
  memset(logger, 0, sizeof(*logger));
  strncpy(logger->mount_point, "/sdcard", sizeof(logger->mount_point) - 1);

  const size_t default_batch = 128 * 1024;
  const size_t default_tail_scan = 256 * 1024;
  const size_t default_buffer = 64 * 1024;
  const uint32_t default_max_freq_khz = 10000;

  logger->config.batch_target_bytes =
    DefaultOr(config ? config->batch_target_bytes : 0, default_batch);
  logger->config.tail_scan_bytes =
    DefaultOr(config ? config->tail_scan_bytes : 0, default_tail_scan);
  logger->config.file_buffer_bytes =
    DefaultOr(config ? config->file_buffer_bytes : 0, default_buffer);
  logger->config.max_freq_khz =
    DefaultOr(config ? config->max_freq_khz : 0, default_max_freq_khz);

  if (logger->config.file_buffer_bytes > 0) {
    logger->file_buffer = AllocatePreferPsram(
      logger->config.file_buffer_bytes);
  }
  if (logger->config.tail_scan_bytes > 0) {
    const size_t resume_bytes = logger->config.tail_scan_bytes + 1;
    logger->resume_tail_bytes = AllocatePreferPsram(resume_bytes);
    if (logger->resume_tail_bytes != NULL) {
      logger->resume_tail_capacity = resume_bytes;
    }
  }

  logger->host_id = (spi_host_device_t)0;
  logger->cs_gpio = -1;
  logger->slot_config_valid = false;
}

/**
 * @brief Execute BuildDailyCsvPath.
 * @param logger Parameter logger.
 * @param epoch_seconds Parameter epoch_seconds.
 * @param date_out Parameter date_out.
 * @param date_out_size Parameter date_out_size.
 * @param path_out Parameter path_out.
 * @param path_out_size Parameter path_out_size.
 */
static void
BuildDailyCsvPath(const sd_logger_t* logger,
                  int64_t epoch_seconds,
                  char* date_out,
                  size_t date_out_size,
                  char* path_out,
                  size_t path_out_size)
{
  time_t time_seconds = (time_t)epoch_seconds;
  struct tm time_info;
  gmtime_r(&time_seconds, &time_info);

  strftime(date_out, date_out_size, "%Y-%m-%dZ", &time_info);

  snprintf(path_out, path_out_size, "%s/%s.csv", logger->mount_point, date_out);
}

static bool
IsStrictDailyCsvName(const char* name)
{
  if (name == NULL) {
    return false;
  }
  const size_t length = strlen(name);
  if (length != 14 && length != 15) {
    return false;
  }
  const bool has_z = (length == 15);
  if ((name[4] != '-') || (name[7] != '-')) {
    return false;
  }
  for (size_t i = 0; i < 10; ++i) {
    if (i == 4 || i == 7) {
      continue;
    }
    if (name[i] < '0' || name[i] > '9') {
      return false;
    }
  }
  if (has_z && name[10] != 'Z') {
    return false;
  }
  if (strcmp(name + length - 4, ".csv") != 0) {
    return false;
  }
  return true;
}

static esp_err_t
SdLoggerGetSpaceInfoLocked(const sd_logger_t* logger,
                           uint64_t* total_bytes,
                           uint64_t* free_bytes)
{
  if (logger == NULL || total_bytes == NULL || free_bytes == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!logger->is_mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  struct statvfs fs = { 0 };
  if (statvfs(logger->mount_point, &fs) != 0) {
    return ESP_FAIL;
  }
  const uint64_t block_size =
    (fs.f_frsize != 0u) ? (uint64_t)fs.f_frsize : (uint64_t)fs.f_bsize;
  *total_bytes = ((uint64_t)fs.f_blocks) * block_size;
  *free_bytes = ((uint64_t)fs.f_bavail) * block_size;
  return ESP_OK;
}

static bool
SdLoggerFindOldestDailyCsvLocked(const sd_logger_t* logger,
                                 char* oldest_path,
                                 size_t oldest_path_size)
{
  if (logger == NULL || oldest_path == NULL || oldest_path_size == 0) {
    return false;
  }

  DIR* dir = opendir(logger->mount_point);
  if (dir == NULL) {
    return false;
  }

  char current_path[128] = "";
  if (logger->current_date[0] != '\0') {
    snprintf(current_path,
             sizeof(current_path),
             "%s/%s.csv",
             logger->mount_point,
             logger->current_date);
  }

  bool found = false;
  char oldest_name[32] = "";
  struct dirent* entry = NULL;
  while ((entry = readdir(dir)) != NULL) {
    if (!IsStrictDailyCsvName(entry->d_name)) {
      continue;
    }
    char candidate_path[128];
    snprintf(candidate_path,
             sizeof(candidate_path),
             "%s/%s",
             logger->mount_point,
             entry->d_name);
    if (current_path[0] != '\0' && strcmp(candidate_path, current_path) == 0) {
      continue;
    }
    if (!found || strcmp(entry->d_name, oldest_name) < 0) {
      snprintf(oldest_name, sizeof(oldest_name), "%s", entry->d_name);
      snprintf(oldest_path, oldest_path_size, "%s", candidate_path);
      found = true;
    }
  }

  closedir(dir);
  return found;
}

static esp_err_t
SdLoggerReclaimSpaceLocked(sd_logger_t* logger,
                           uint64_t required_free_bytes,
                           uint32_t* deleted_files)
{
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (deleted_files != NULL) {
    *deleted_files = 0;
  }

  static const uint32_t kSdReclaimMaxDeletesPerAttempt = 16;
  uint64_t total_bytes = 0;
  uint64_t free_bytes = 0;
  esp_err_t space_result =
    SdLoggerGetSpaceInfoLocked(logger, &total_bytes, &free_bytes);
  if (space_result != ESP_OK) {
    return space_result;
  }

  uint32_t deletes = 0;
  while (free_bytes < required_free_bytes &&
         deletes < kSdReclaimMaxDeletesPerAttempt) {
    char oldest_path[128] = "";
    if (!SdLoggerFindOldestDailyCsvLocked(
          logger, oldest_path, sizeof(oldest_path))) {
      break;
    }
    if (unlink(oldest_path) != 0) {
      ESP_LOGW(kTag,
               "Failed to delete old log %s: %s (%d)",
               oldest_path,
               strerror(errno),
               errno);
      break;
    }
    deletes++;
    ESP_LOGW(kTag, "Deleted old log to reclaim space: %s", oldest_path);

    space_result = SdLoggerGetSpaceInfoLocked(logger, &total_bytes, &free_bytes);
    if (space_result != ESP_OK) {
      return space_result;
    }
  }

  if (deleted_files != NULL) {
    *deleted_files = deletes;
  }
  return ESP_OK;
}

/**
 * @brief Execute WriteHeaderIfEmpty.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
static esp_err_t
WriteHeaderIfEmpty(sd_logger_t* logger)
{
  struct stat stat_buffer;
  if (fstat(fileno(logger->file), &stat_buffer) != 0) {
    return ESP_FAIL;
  }
  if (stat_buffer.st_size > 0) {
    return ESP_OK;
  }

  const bool wrote_header = CsvWriteHeader(CsvFileWriter, logger->file);
  return wrote_header ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Execute SdLoggerMountInternal.
 * @param logger Parameter logger.
 * @param host Parameter host.
 * @param cs_gpio Parameter cs_gpio.
 * @param format_if_mount_failed Parameter format_if_mount_failed.
 * @return Return the function result.
 */
static esp_err_t
SdLoggerMountInternal(sd_logger_t* logger,
                      spi_host_device_t host,
                      int cs_gpio,
                      bool format_if_mount_failed)
{
  sdmmc_host_t sd_host = SDSPI_HOST_DEFAULT();
  sd_host.slot = host;
  // Hot-insert and longer wiring runs are significantly more reliable at a
  // lower SPI clock. If you want maximum throughput later, make this
  // configurable and/or raise it after a successful probe.
  sd_host.max_freq_khz = logger->config.max_freq_khz;
  if (logger->io_bounce_bytes != NULL && logger->io_bounce_capacity > 0) {
#if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    if (logger->io_bounce_capacity >= 512) {
      sd_host.dma_aligned_buffer = logger->io_bounce_bytes;
    } else {
      ESP_LOGW(kTag,
               "IO bounce buffer too small (%zu bytes); skipping DMA buffer",
               logger->io_bounce_capacity);
    }
#endif
  }

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = cs_gpio;
  slot_config.host_id = host;

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = format_if_mount_failed,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024,
  };

  sdmmc_card_t* card = NULL;
  esp_err_t result = esp_vfs_fat_sdspi_mount(
    logger->mount_point, &sd_host, &slot_config, &mount_config, &card);

  if (result != ESP_OK) {
    ESP_LOGW(kTag, "SD mount failed: %s", esp_err_to_name(result));
    return result;
  }

  logger->is_mounted = true;
  logger->card = card;
  ESP_LOGI(kTag, "SD mounted at %s", logger->mount_point);

  // Give the card a brief settle window after mount, especially if it was
  // inserted while the system was already running.
  vTaskDelay(pdMS_TO_TICKS(150));
  return ESP_OK;
}

/**
 * @brief Execute SdLoggerMount.
 * @param logger Parameter logger.
 * @param host Parameter host.
 * @param cs_gpio Parameter cs_gpio.
 * @return Return the function result.
 */
esp_err_t
SdLoggerMount(sd_logger_t* logger, spi_host_device_t host, int cs_gpio)
{
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  logger->host_id = host;
  logger->cs_gpio = cs_gpio;
  logger->slot_config_valid = true;

  return SdLoggerMountInternal(logger, host, cs_gpio, false);
}

/**
 * @brief Execute SdLoggerTryRemount.
 * @param logger Parameter logger.
 * @param format_if_mount_failed Parameter format_if_mount_failed.
 * @return Return the function result.
 */
esp_err_t
SdLoggerTryRemount(sd_logger_t* logger, bool format_if_mount_failed)
{
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (logger->is_mounted) {
    return ESP_OK;
  }
  if (!logger->slot_config_valid) {
    return ESP_ERR_INVALID_STATE;
  }
  return SdLoggerMountInternal(
    logger, logger->host_id, logger->cs_gpio, format_if_mount_failed);
}

/**
 * @brief Execute SdLoggerUnmount.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
esp_err_t
SdLoggerUnmount(sd_logger_t* logger)
{
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  SdLoggerClose(logger);

  if (logger->is_mounted && logger->card != NULL) {
    esp_err_t unmount_result =
      esp_vfs_fat_sdcard_unmount(logger->mount_point, logger->card);
    if (unmount_result != ESP_OK) {
      ESP_LOGW(kTag,
               "SD unmount failed (%s): %s",
               logger->mount_point,
               esp_err_to_name(unmount_result));
      // Continue clearing state regardless.
    }
  }

  logger->is_mounted = false;
  logger->card = NULL;
  return ESP_OK;
}

/**
 * @brief Execute ApplyResumeInfo.
 * @param logger Parameter logger.
 * @param file Parameter file.
 * @param path Parameter path.
 * @return Return the function result.
 */
static esp_err_t
ApplyResumeInfo(sd_logger_t* logger, FILE* file, const char* path)
{
  SdCsvResumeInfo resume_info = { 0 };
  sd_csv_resume_scratch_t scratch = {
    .tail_bytes = logger->resume_tail_bytes,
    .tail_capacity = logger->resume_tail_capacity,
  };
  esp_err_t resume_result = SdCsvFindLastRecordIdAndRepairTail(
    file, logger->config.tail_scan_bytes, &scratch, &resume_info);
  if (resume_result != ESP_OK) {
    ESP_LOGE(kTag,
             "Failed to scan/repair %s: %s",
             path,
             esp_err_to_name(resume_result));
    return resume_result;
  }
  if (resume_info.file_was_truncated) {
    ESP_LOGW(kTag, "%s tail repaired after power loss", path);
  }
  if (resume_info.found_last_record_id) {
    logger->last_record_id_on_sd = resume_info.last_record_id;
    ESP_LOGI(kTag,
             "Resume: last record id on %s = %" PRIu64,
             path,
             resume_info.last_record_id);
  }
  return ESP_OK;
}

/**
 * @brief Execute SdLoggerEnsureDailyFile.
 * @param logger Parameter logger.
 * @param epoch_utc Parameter epoch_utc.
 * @return Return the function result.
 */
esp_err_t
SdLoggerEnsureDailyFile(sd_logger_t* logger, int64_t epoch_utc)
{
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!logger->is_mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  char date_string[16];
  char path[128];
  BuildDailyCsvPath(
    logger, epoch_utc, date_string, sizeof(date_string), path, sizeof(path));

  if (logger->file != NULL && logger->current_date[0] != '\0') {
    const int date_cmp = strcmp(date_string, logger->current_date);
    if (date_cmp == 0) {
      return ESP_OK; // already open for today
    }
    if (date_cmp < 0) {
      return ESP_OK; // never roll backward to an earlier date
    }
  }

  SdLoggerClose(logger);
  logger->last_record_id_on_sd = 0;

  logger->file = fopen(path, "a+b");
  if (logger->file == NULL) {
    ESP_LOGE(
      kTag, "fopen failed for %s: %s (%d)", path, strerror(errno), errno);
    return ESP_FAIL;
  }

  if (logger->file_buffer != NULL) {
    setvbuf((FILE*)logger->file,
            (char*)logger->file_buffer,
            _IOFBF,
            logger->config.file_buffer_bytes);
  }

  esp_err_t resume_result = ApplyResumeInfo(logger, logger->file, path);
  if (resume_result != ESP_OK) {
    return resume_result;
  }

  esp_err_t header_result = WriteHeaderIfEmpty(logger);
  if (header_result != ESP_OK) {
    ESP_LOGE(kTag, "Failed to write header to %s", path);
    return header_result;
  }

  strncpy(logger->current_date, date_string, sizeof(logger->current_date) - 1);
  logger->current_date[sizeof(logger->current_date) - 1] = '\0';
  return ESP_OK;
}

/**
 * @brief Execute SdLoggerAppendVerifiedBatch.
 * @param logger Parameter logger.
 * @param batch_bytes Parameter batch_bytes.
 * @param batch_length_bytes Parameter batch_length_bytes.
 * @param last_record_id_in_batch Parameter last_record_id_in_batch.
 * @param diag_out Parameter diag_out.
 * @return Return the function result.
 */
esp_err_t
SdLoggerAppendVerifiedBatch(sd_logger_t* logger,
                            const uint8_t* batch_bytes,
                            size_t batch_length_bytes,
                            uint64_t last_record_id_in_batch,
                            SdCsvAppendDiagnostics* diag_out)
{
  sd_csv_append_stats_t stats = { 0 };
  esp_err_t result = SdLoggerAppendBatchEx(logger,
                                           batch_bytes,
                                           batch_length_bytes,
                                           last_record_id_in_batch,
                                           SD_APPEND_VERIFY_READBACK_SHA256,
                                           SD_APPEND_FLUSH_ALWAYS,
                                           NULL,
                                           &stats);
  if (diag_out != NULL) {
    *diag_out = stats.diag;
  }
  return result;
}

/**
 * @brief Execute SdLoggerAppendBatchEx.
 * @param logger Parameter logger.
 * @param batch_bytes Parameter batch_bytes.
 * @param batch_length_bytes Parameter batch_length_bytes.
 * @param last_record_id Parameter last_record_id.
 * @param verify_mode Parameter verify_mode.
 * @param flush_mode Parameter flush_mode.
 * @param scratch Parameter scratch.
 * @param out_stats Parameter out_stats.
 * @return Return the function result.
 */
esp_err_t
SdLoggerAppendBatchEx(sd_logger_t* logger,
                      const uint8_t* batch_bytes,
                      size_t batch_length_bytes,
                      uint64_t last_record_id,
                      sd_append_verify_t verify_mode,
                      sd_append_flush_t flush_mode,
                      const sd_csv_append_scratch_t* scratch,
                      sd_csv_append_stats_t* out_stats)
{
  static const uint64_t kSdMinFreeReserveBytes = 64 * 1024;
  sd_csv_append_stats_t stats = { 0 };
  sd_csv_append_stats_t* stats_out = (out_stats != NULL) ? out_stats : &stats;
  ResetAppendStats(stats_out);

  if (logger == NULL || logger->file == NULL) {
    SetAppendDiagnostics(&stats_out->diag, "append", 0);
    return ESP_ERR_INVALID_STATE;
  }
  if (batch_bytes == NULL || batch_length_bytes == 0) {
    SetAppendDiagnostics(&stats_out->diag, "append", 0);
    return ESP_ERR_INVALID_ARG;
  }

  const uint64_t required_free_bytes =
    (uint64_t)batch_length_bytes + kSdMinFreeReserveBytes;
  uint64_t total_bytes = 0;
  uint64_t free_bytes_before = 0;
  if (SdLoggerGetSpaceInfoLocked(logger, &total_bytes, &free_bytes_before) == ESP_OK) {
    stats_out->space_free_bytes_before = free_bytes_before;
    if (free_bytes_before < required_free_bytes) {
      uint32_t deleted_files = 0;
      if (SdLoggerReclaimSpaceLocked(logger, required_free_bytes, &deleted_files) == ESP_OK &&
          deleted_files > 0) {
        logger->space_reclaim_active = true;
        logger->space_reclaim_deleted_total += deleted_files;
        stats_out->space_reclaim_deleted_files += deleted_files;
      }
    }
  }

  fseek(logger->file, 0, SEEK_END);

  bool retry_after_reclaim = false;
write_retry:
  if (verify_mode == SD_APPEND_VERIFY_READBACK_SHA256) {
    esp_err_t result = SdCsvAppendBatchWithReadbackVerifyEx(
      logger->file,
      batch_bytes,
      batch_length_bytes,
      &stats_out->diag,
      scratch,
      flush_mode == SD_APPEND_FLUSH_ALWAYS);
    if (result == ESP_OK) {
      stats_out->bytes_appended = batch_length_bytes;
      logger->last_record_id_on_sd = last_record_id;
      (void)SdLoggerGetSpaceInfoLocked(
        logger, &total_bytes, &stats_out->space_free_bytes_after);
      return ESP_OK;
    }
    if (!retry_after_reclaim && stats_out->diag.errno_value == ENOSPC) {
      uint32_t deleted_files = 0;
      if (SdLoggerReclaimSpaceLocked(logger, required_free_bytes, &deleted_files) == ESP_OK &&
          deleted_files > 0) {
        logger->space_reclaim_active = true;
        logger->space_reclaim_deleted_total += deleted_files;
        stats_out->space_reclaim_deleted_files += deleted_files;
      }
      retry_after_reclaim = true;
      clearerr(logger->file);
      goto write_retry;
    }
    (void)SdLoggerGetSpaceInfoLocked(
      logger, &total_bytes, &stats_out->space_free_bytes_after);
    return result;
  }

  size_t write_calls = 0;
  if (scratch != NULL && scratch->io_bounce_bytes != NULL &&
      scratch->io_bounce_capacity > 0) {
    size_t offset = 0;
    while (offset < batch_length_bytes) {
      size_t chunk = batch_length_bytes - offset;
      if (chunk > scratch->io_bounce_capacity) {
        chunk = scratch->io_bounce_capacity;
      }
      memcpy(scratch->io_bounce_bytes, batch_bytes + offset, chunk);
      const size_t written =
        fwrite(scratch->io_bounce_bytes, 1, chunk, logger->file);
      write_calls++;
      if (written != chunk) {
        ESP_LOGE(kTag,
                 "fwrite() short write: wrote=%u expected=%u",
                 (unsigned)written,
                 (unsigned)chunk);
        SetAppendDiagnostics(&stats_out->diag, "append", errno);
        break;
      }
      offset += chunk;
    }
    if (offset != batch_length_bytes) {
      if (!retry_after_reclaim && stats_out->diag.errno_value == ENOSPC) {
        uint32_t deleted_files = 0;
        if (SdLoggerReclaimSpaceLocked(logger, required_free_bytes, &deleted_files) == ESP_OK &&
            deleted_files > 0) {
          logger->space_reclaim_active = true;
          logger->space_reclaim_deleted_total += deleted_files;
          stats_out->space_reclaim_deleted_files += deleted_files;
        }
        retry_after_reclaim = true;
        clearerr(logger->file);
        goto write_retry;
      }
      (void)SdLoggerGetSpaceInfoLocked(
        logger, &total_bytes, &stats_out->space_free_bytes_after);
      return ESP_FAIL;
    }
  } else {
    const size_t written =
      fwrite(batch_bytes, 1, batch_length_bytes, logger->file);
    write_calls++;
    if (written != batch_length_bytes) {
      ESP_LOGE(kTag,
               "fwrite() short write: wrote=%u expected=%u",
               (unsigned)written,
               (unsigned)batch_length_bytes);
      SetAppendDiagnostics(&stats_out->diag, "append", errno);
      if (!retry_after_reclaim && stats_out->diag.errno_value == ENOSPC) {
        uint32_t deleted_files = 0;
        if (SdLoggerReclaimSpaceLocked(logger, required_free_bytes, &deleted_files) == ESP_OK &&
            deleted_files > 0) {
          logger->space_reclaim_active = true;
          logger->space_reclaim_deleted_total += deleted_files;
          stats_out->space_reclaim_deleted_files += deleted_files;
        }
        retry_after_reclaim = true;
        clearerr(logger->file);
        goto write_retry;
      }
      (void)SdLoggerGetSpaceInfoLocked(
        logger, &total_bytes, &stats_out->space_free_bytes_after);
      return ESP_FAIL;
    }
  }

  if (flush_mode == SD_APPEND_FLUSH_ALWAYS) {
    if (fflush(logger->file) != 0) {
      ESP_LOGE(kTag, "fflush() failed: errno=%d (%s)", errno, strerror(errno));
      SetAppendDiagnostics(&stats_out->diag, "fflush", errno);
      if (!retry_after_reclaim && stats_out->diag.errno_value == ENOSPC) {
        uint32_t deleted_files = 0;
        if (SdLoggerReclaimSpaceLocked(logger, required_free_bytes, &deleted_files) == ESP_OK &&
            deleted_files > 0) {
          logger->space_reclaim_active = true;
          logger->space_reclaim_deleted_total += deleted_files;
          stats_out->space_reclaim_deleted_files += deleted_files;
        }
        retry_after_reclaim = true;
        clearerr(logger->file);
        goto write_retry;
      }
      (void)SdLoggerGetSpaceInfoLocked(
        logger, &total_bytes, &stats_out->space_free_bytes_after);
      return ESP_FAIL;
    }
  }

  stats_out->bytes_appended = batch_length_bytes;
  stats_out->write_calls = write_calls;
  logger->last_record_id_on_sd = last_record_id;
  (void)SdLoggerGetSpaceInfoLocked(logger, &total_bytes, &stats_out->space_free_bytes_after);
  return ESP_OK;
}

/**
 * @brief Execute SdLoggerClose.
 * @param logger Parameter logger.
 */
void
SdLoggerClose(sd_logger_t* logger)
{
  if (logger == NULL) {
    return;
  }
  if (logger->file != NULL) {
    fclose(logger->file);
    logger->file = NULL;
  }
  logger->current_date[0] = '\0';
}

/**
 * @brief Execute SdLoggerFlushAndSync.
 * @param logger Parameter logger.
 * @param diag_out Parameter diag_out.
 * @return Return the function result.
 */
esp_err_t
SdLoggerFlushAndSync(sd_logger_t* logger, SdCsvAppendDiagnostics* diag_out)
{
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (logger->file == NULL) {
    return ESP_OK;
  }
  return SdCsvFlushAndSync(logger->file, diag_out);
}

bool
SdLoggerSpaceReclaimActive(const sd_logger_t* logger)
{
  return (logger != NULL) ? logger->space_reclaim_active : false;
}

uint32_t
SdLoggerSpaceReclaimDeletedTotal(const sd_logger_t* logger)
{
  return (logger != NULL) ? logger->space_reclaim_deleted_total : 0;
}

esp_err_t
SdLoggerGetSpaceInfo(const sd_logger_t* logger,
                     uint64_t* total_bytes,
                     uint64_t* free_bytes)
{
  return SdLoggerGetSpaceInfoLocked(logger, total_bytes, free_bytes);
}

/**
 * @brief Execute SdLoggerFormatDestructive.
 * @param logger Parameter logger.
 * @return Return the function result.
 */
esp_err_t
SdLoggerFormatDestructive(sd_logger_t* logger)
{
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Close any open file handles before formatting.
  SdLoggerClose(logger);

  // Ensure we have a mounted/registered card context.
  // format_if_mount_failed=true handles blank or corrupted cards.
  esp_err_t mount_result = SdLoggerTryRemount(logger, true);
  if (mount_result != ESP_OK) {
    ESP_LOGE(kTag,
             "SD format: failed to mount/init card: %s",
             esp_err_to_name(mount_result));
    return mount_result;
  }
  if (!logger->is_mounted || logger->card == NULL) {
    ESP_LOGE(kTag, "SD format: card not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  // Destructive format (mkfs). This recreates the filesystem even if already
  // mounted.
  ESP_LOGW(kTag, "Formatting SD card at %s (destructive)", logger->mount_point);
  esp_err_t format_result =
    esp_vfs_fat_sdcard_format(logger->mount_point, logger->card);
  if (format_result != ESP_OK) {
    ESP_LOGE(kTag, "SD format failed: %s", esp_err_to_name(format_result));
    return format_result;
  }

  // Sanity-check that the mount point is usable after format.
  struct stat stat_buffer;
  if (stat(logger->mount_point, &stat_buffer) != 0) {
    ESP_LOGE(kTag,
             "SD format succeeded but mount point is not accessible: %s (%d)",
             strerror(errno),
             errno);
    return ESP_FAIL;
  }

  logger->current_date[0] = '\0';
  logger->last_record_id_on_sd = 0;
  return ESP_OK;
}
