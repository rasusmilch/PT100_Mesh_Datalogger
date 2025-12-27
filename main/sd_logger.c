#include "sd_logger.h"

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "data_csv.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

static const char* kTag = "sd_logger";

static size_t
DefaultOr(const size_t value, const size_t fallback)
{
  return (value == 0) ? fallback : value;
}

static bool
CsvFileWriter(const char* bytes, size_t len, void* context)
{
  FILE* file = (FILE*)context;
  return SdCsvAppendBatchWithReadbackVerify(
           file, (const uint8_t*)bytes, len, NULL) == ESP_OK;
}

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
  localtime_r(&time_seconds, &time_info);

  strftime(date_out, date_out_size, "%Y-%m-%d", &time_info);

  snprintf(path_out,
           path_out_size,
           "%s/%s.csv",
           logger->mount_point,
           date_out);
}

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

static esp_err_t
SdLoggerMountInternal(sd_logger_t* logger,
                      spi_host_device_t host,
                      int cs_gpio,
                      bool format_if_mount_failed)
{
  sdmmc_host_t sd_host = SDSPI_HOST_DEFAULT();
  sd_host.slot = host;

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
  return ESP_OK;
}

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
  return SdLoggerMountInternal(logger,
                               logger->host_id,
                               logger->cs_gpio,
                               format_if_mount_failed);
}

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


static esp_err_t
ApplyResumeInfo(sd_logger_t* logger, FILE* file, const char* path)
{
  SdCsvResumeInfo resume_info = { 0 };
  esp_err_t resume_result = SdCsvFindLastRecordIdAndRepairTail(
    file, logger->config.tail_scan_bytes, &resume_info);
  if (resume_result != ESP_OK) {
    ESP_LOGE(kTag, "Failed to scan/repair %s: %s", path, esp_err_to_name(resume_result));
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
  BuildDailyCsvPath(logger, epoch_utc, date_string, sizeof(date_string), path, sizeof(path));

  if (logger->file != NULL && strcmp(logger->current_date, date_string) == 0) {
    return ESP_OK; // already open for today
  }

  SdLoggerClose(logger);
  logger->last_record_id_on_sd = 0;

  logger->file = fopen(path, "a+b");
  if (logger->file == NULL) {
    ESP_LOGE(kTag, "fopen failed for %s: %s (%d)", path, strerror(errno), errno);
    return ESP_FAIL;
  }

  if (logger->file_buffer != NULL) {
    free(logger->file_buffer);
    logger->file_buffer = NULL;
  }
  logger->file_buffer = (uint8_t*)malloc(logger->config.file_buffer_bytes);
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
  logger->last_record_id_on_sd = 0;
}
