#include "sd_logger.h"

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "data_csv.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

static const char* kTag = "sd_logger";

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

  logger->config.batch_target_bytes =
    DefaultOr(config ? config->batch_target_bytes : 0, default_batch);
  logger->config.tail_scan_bytes =
    DefaultOr(config ? config->tail_scan_bytes : 0, default_tail_scan);
  logger->config.file_buffer_bytes =
    DefaultOr(config ? config->file_buffer_bytes : 0, default_buffer);

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
  sd_host.max_freq_khz = 10000; // 10 MHz

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
  esp_err_t resume_result = SdCsvFindLastRecordIdAndRepairTail(
    file, logger->config.tail_scan_bytes, &resume_info);
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

  if (logger->file != NULL && strcmp(logger->current_date, date_string) == 0) {
    return ESP_OK; // already open for today
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
    free(logger->file_buffer);
    logger->file_buffer = NULL;
  }
  // File buffer is used only for VFS I/O (no DMA); PSRAM is safe here.
  logger->file_buffer = (uint8_t*)heap_caps_malloc(
    logger->config.file_buffer_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (logger->file_buffer == NULL) {
    logger->file_buffer = (uint8_t*)heap_caps_malloc(
      logger->config.file_buffer_bytes,
      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
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
  if (logger == NULL || logger->file == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  if (batch_bytes == NULL || batch_length_bytes == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  fseek(logger->file, 0, SEEK_END);
  esp_err_t result = SdCsvAppendBatchWithReadbackVerify(
    logger->file, batch_bytes, batch_length_bytes, diag_out);
  if (result == ESP_OK) {
    logger->last_record_id_on_sd = last_record_id_in_batch;
  }
  return result;
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
  if (logger->file_buffer != NULL) {
    free(logger->file_buffer);
    logger->file_buffer = NULL;
  }
  logger->current_date[0] = '\0';
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
