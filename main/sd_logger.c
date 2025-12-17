#include "sd_logger.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

static const char* kTag = "sd_logger";

static void
BuildDailyCsvPath(const sd_logger_t* logger,
                  int64_t epoch_seconds,
                  char* path_out,
                  size_t path_out_size)
{
  // UTC date.
  time_t time_seconds = (time_t)epoch_seconds;
  struct tm time_info;
  gmtime_r(&time_seconds, &time_info);

  char date_string[16];
  strftime(date_string, sizeof(date_string), "%Y%m%d", &time_info);

  snprintf(path_out,
           path_out_size,
           "%s/pt100_log_%s.csv",
           logger->mount_point,
           date_string);
}

static bool
FileExists(const char* path)
{
  struct stat stat_buffer;
  return stat(path, &stat_buffer) == 0;
}

esp_err_t
SdLoggerMount(sd_logger_t* logger, spi_host_device_t host, int cs_gpio)
{
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(logger, 0, sizeof(*logger));
  strncpy(logger->mount_point, "/sdcard", sizeof(logger->mount_point) - 1);

  sdmmc_host_t sd_host = SDSPI_HOST_DEFAULT();
  sd_host.slot = host;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = cs_gpio;
  slot_config.host_id = host;

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
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
SdLoggerAppendCsv(sd_logger_t* logger,
                  const char* node_id,
                  const log_record_t* record)
{
  if (logger == NULL || node_id == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!logger->is_mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  const int64_t epoch_seconds = record->timestamp_epoch_sec;
  char path[128];
  BuildDailyCsvPath(logger,
                    epoch_seconds == 0 ? (int64_t)time(NULL) : epoch_seconds,
                    path,
                    sizeof(path));

  const bool file_exists = FileExists(path);

  FILE* file = fopen(path, "a");
  if (file == NULL) {
    ESP_LOGW(kTag, "fopen failed: %s (%d)", strerror(errno), errno);
    return ESP_FAIL;
  }

  if (!file_exists) {
    // Header row.
    fprintf(
      file,
      "node_id,epoch_sec,ms,raw_temp_c,temp_c,resistance_ohm,seq,flags\n");
  }

  const double raw_c = record->raw_temp_milli_c / 1000.0;
  const double temp_c = record->temp_milli_c / 1000.0;
  const double resistance_ohm = record->resistance_milli_ohm / 1000.0;

  fprintf(file,
          "%s,%" PRId64 ",%d,%.3f,%.3f,%.3f,%u,0x%04x\n",
          node_id,
          record->timestamp_epoch_sec,
          record->timestamp_millis,
          raw_c,
          temp_c,
          resistance_ohm,
          (unsigned)record->sequence,
          (unsigned)record->flags);

  if (fflush(file) != 0) {
    ESP_LOGW(kTag, "fflush failed: %s (%d)", strerror(errno), errno);
    fclose(file);
    return ESP_FAIL;
  }
  if (fsync(fileno(file)) != 0) {
    ESP_LOGW(kTag, "fsync failed: %s (%d)", strerror(errno), errno);
    fclose(file);
    return ESP_FAIL;
  }
  fclose(file);
  return ESP_OK;
}
