#include "sd_logger.h"

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "ptlog_format.h"
#include "sd_ptlog_paths.h"

static const char* kTag = "sd_logger";
static const char kPtlogSuffix[] = ".ptlog";

static bool
IsDailyPtlogName(const char* name, char* date_out, size_t date_out_size);
static bool
SdLoggerJoinPath(const char* dir_path,
                 const char* child_name,
                 char* out_path,
                 size_t out_path_size);
static esp_err_t
SdLoggerEnsurePtlogDirectoryLocked(const sd_logger_t* logger,
                                  const char* month_string);
static esp_err_t
SdLoggerFindNextRevisionInMonthLocked(const sd_logger_t* logger,
                                      const char* date_string,
                                      const char* month_string,
                                      uint32_t* revision_out);
static bool
SdLoggerCopyString(const char* source, char* dest, size_t dest_size);
static esp_err_t
SdLoggerGetSpaceInfoLocked(const sd_logger_t* logger,
                           uint64_t* total_bytes,
                           uint64_t* free_bytes);
static bool
SdLoggerFindOldestDailyPtlogDateLocked(const sd_logger_t* logger,
                                       char* oldest_date,
                                       size_t oldest_date_size);
static uint32_t
SdLoggerDeleteDailyPtlogDateLocked(const sd_logger_t* logger,
                                   const char* date_string);
static esp_err_t
SdLoggerReclaimSpaceLocked(sd_logger_t* logger,
                           uint64_t required_free_bytes,
                           uint32_t* deleted_files);
static esp_err_t
CommitHeaderIfEmptyAndSync_(FILE* file,
                            const ptlog_header_t* header,
                            const char* path,
                            bool* file_was_empty_out,
                            bool* header_written_out);
static bool
SdLoggerUnlinkEmptyFileIfPresent_(const char* path);
static void
SdLoggerResetMountState_(sd_logger_t* logger);

static uint8_t*
AllocatePreferPsram(size_t bytes)
{
  uint8_t* buffer =
    (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (buffer != NULL) {
    return buffer;
  }
  return (uint8_t*)heap_caps_malloc(bytes,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
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
  const uint32_t default_max_freq_khz = CONFIG_APP_SD_SPI_MAX_FREQ_KHZ;

  logger->config.batch_target_bytes =
    DefaultOr(config ? config->batch_target_bytes : 0, default_batch);
  logger->config.tail_scan_bytes =
    DefaultOr(config ? config->tail_scan_bytes : 0, default_tail_scan);
  logger->config.file_buffer_bytes =
    DefaultOr(config ? config->file_buffer_bytes : 0, default_buffer);
  logger->config.max_freq_khz =
    DefaultOr(config ? config->max_freq_khz : 0, default_max_freq_khz);

  if (logger->config.file_buffer_bytes > 0) {
    logger->file_buffer = AllocatePreferPsram(logger->config.file_buffer_bytes);
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
 * @brief Build the FAT16-safe nested daily PTLOG path.
 *
 * New writes avoid FAT16 root-directory pressure by targeting
 * /logs/YYYY-MM/YYYY-MM-DDZ[-revision].ptlog.  The helper fails closed on
 * truncation and leaves path_out empty so callers do not open partial paths.
 */
static void
BuildDailyPtlogPath(const sd_logger_t* logger,
                    int64_t epoch_seconds,
                    uint32_t revision,
                    char* date_out,
                    size_t date_out_size,
                    char* month_out,
                    size_t month_out_size,
                    char* path_out,
                    size_t path_out_size)
{
  if (path_out != NULL && path_out_size > 0) {
    path_out[0] = '\0';
  }
  if (date_out != NULL && date_out_size > 0) {
    date_out[0] = '\0';
  }
  if (month_out != NULL && month_out_size > 0) {
    month_out[0] = '\0';
  }
  if (logger == NULL) {
    return;
  }

  if (!SdPtlogBuildNestedPath(logger->mount_point,
                              epoch_seconds,
                              revision,
                              date_out,
                              date_out_size,
                              month_out,
                              month_out_size,
                              path_out,
                              path_out_size)) {
    if (path_out != NULL && path_out_size > 0) {
      path_out[0] = '\0';
    }
  }
}

/**
 * @brief Join a directory path and child name into a destination buffer.
 * @param dir_path Base directory path.
 * @param child_name Child entry name.
 * @param out_path Destination buffer.
 * @param out_path_size Destination buffer size.
 * @return True when the full joined path fits, otherwise false.
 */
static bool
SdLoggerJoinPath(const char* dir_path,
                 const char* child_name,
                 char* out_path,
                 size_t out_path_size)
{
  if (dir_path == NULL || child_name == NULL || out_path == NULL ||
      out_path_size == 0) {
    return false;
  }

  const size_t dir_len = strnlen(dir_path, out_path_size);
  if (dir_len == out_path_size) {
    return false;
  }
  const size_t child_len = strnlen(child_name, out_path_size);
  if (child_len == out_path_size) {
    return false;
  }
  const bool needs_separator = (dir_len > 0 && dir_path[dir_len - 1] != '/');
  const size_t total_len =
    dir_len + (needs_separator ? 1U : 0U) + child_len + 1U;
  if (total_len > out_path_size) {
    return false;
  }

  memcpy(out_path, dir_path, dir_len);
  size_t write_index = dir_len;
  if (needs_separator) {
    out_path[write_index++] = '/';
  }
  memcpy(out_path + write_index, child_name, child_len);
  out_path[write_index + child_len] = '\0';
  return true;
}

/**
 * @brief Create or verify one PTLOG directory without overwriting conflicts.
 *
 * Existing directories are accepted, missing directories are created, and
 * existing non-directory paths fail closed so later file opens cannot target a
 * malformed FAT path.
 */
static esp_err_t
SdLoggerEnsureDirectoryPath_(const char* path, const char* label)
{
  if (path == NULL || path[0] == '\0') {
    ESP_LOGE(kTag, "Invalid %s directory path", label != NULL ? label : "PTLOG");
    return ESP_ERR_INVALID_ARG;
  }

  struct stat stat_buffer;
  if (stat(path, &stat_buffer) == 0) {
    if (S_ISDIR(stat_buffer.st_mode)) {
      return ESP_OK;
    }
    ESP_LOGE(kTag,
             "%s path exists but is not a directory: %s",
             label != NULL ? label : "PTLOG",
             path);
    return ESP_ERR_INVALID_STATE;
  }

  const int stat_errno = errno;
  if (stat_errno != ENOENT) {
    ESP_LOGE(kTag,
             "stat failed for %s directory %s: %s (%d)",
             label != NULL ? label : "PTLOG",
             path,
             strerror(stat_errno),
             stat_errno);
    return ESP_FAIL;
  }

  if (mkdir(path, 0777) != 0) {
    const int mkdir_errno = errno;
    if (mkdir_errno != EEXIST) {
      ESP_LOGE(kTag,
               "mkdir failed for %s directory %s: %s (%d)",
               label != NULL ? label : "PTLOG",
               path,
               strerror(mkdir_errno),
               mkdir_errno);
      return ESP_FAIL;
    }
  }

  if (stat(path, &stat_buffer) != 0) {
    const int verify_errno = errno;
    ESP_LOGE(kTag,
             "stat verify failed for %s directory %s: %s (%d)",
             label != NULL ? label : "PTLOG",
             path,
             strerror(verify_errno),
             verify_errno);
    return ESP_FAIL;
  }
  if (!S_ISDIR(stat_buffer.st_mode)) {
    ESP_LOGE(kTag,
             "%s path is not a directory after mkdir: %s",
             label != NULL ? label : "PTLOG",
             path);
    return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

/**
 * @brief Ensure the FAT16-safe PTLOG log root and target month directories.
 *
 * New daily files are opened only after /logs and /logs/YYYY-MM are verified
 * as directories. Existing files at those paths fail closed; nothing is
 * deleted or renamed here.
 */
static esp_err_t
SdLoggerEnsurePtlogDirectoryLocked(const sd_logger_t* logger,
                                  const char* month_string)
{
  if (logger == NULL || month_string == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  char log_root[SD_PTLOG_MAX_PATH_LEN];
  if (!SdPtlogBuildLogRootPath(
        logger->mount_point, log_root, sizeof(log_root))) {
    ESP_LOGE(kTag, "Failed to build PTLOG log root path");
    return ESP_FAIL;
  }
  esp_err_t result = SdLoggerEnsureDirectoryPath_(log_root, "PTLOG log root");
  if (result != ESP_OK) {
    return result;
  }

  char month_dir[SD_PTLOG_MAX_PATH_LEN];
  if (!SdPtlogBuildMonthDirPath(
        logger->mount_point, month_string, month_dir, sizeof(month_dir))) {
    ESP_LOGE(kTag, "Failed to build PTLOG month directory for %s", month_string);
    return ESP_FAIL;
  }
  return SdLoggerEnsureDirectoryPath_(month_dir, "PTLOG month");
}

/**
 * @brief Find the next same-day header revision in the target month directory.
 *
 * Revision rollover is scoped to /logs/YYYY-MM so nested files do not consume
 * FAT16 root entries. Malformed names, non-regular entries, wrong dates, and
 * overflowing revision values are ignored or fail closed before wraparound.
 */
static esp_err_t
SdLoggerFindNextRevisionInMonthLocked(const sd_logger_t* logger,
                                      const char* date_string,
                                      const char* month_string,
                                      uint32_t* revision_out)
{
  if (logger == NULL || date_string == NULL || month_string == NULL ||
      revision_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  *revision_out = 1u;
  char month_dir[SD_PTLOG_MAX_PATH_LEN];
  if (!SdPtlogBuildMonthDirPath(
        logger->mount_point, month_string, month_dir, sizeof(month_dir))) {
    ESP_LOGE(kTag, "Failed to build PTLOG revision scan directory for %s", month_string);
    return ESP_FAIL;
  }

  DIR* dir = opendir(month_dir);
  if (dir == NULL) {
    if (errno == ENOENT) {
      return ESP_OK;
    }
    const int dir_errno = errno;
    ESP_LOGE(kTag,
             "Failed to open PTLOG revision directory %s: %s (%d)",
             month_dir,
             strerror(dir_errno),
             dir_errno);
    return ESP_FAIL;
  }

  struct dirent* entry = NULL;
  while ((entry = readdir(dir)) != NULL) {
    char parsed_date[SD_PTLOG_DATE_LEN + 1u];
    uint32_t parsed_revision = 0;
    if (!SdPtlogParseName(
          entry->d_name, parsed_date, sizeof(parsed_date), &parsed_revision)) {
      continue;
    }
    if (strcmp(parsed_date, date_string) != 0) {
      continue;
    }

    char candidate_path[SD_PTLOG_MAX_PATH_LEN];
    if (!SdLoggerJoinPath(
          month_dir, entry->d_name, candidate_path, sizeof(candidate_path))) {
      continue;
    }
    struct stat stat_buffer;
    if (stat(candidate_path, &stat_buffer) != 0 ||
        !S_ISREG(stat_buffer.st_mode)) {
      continue;
    }
    if (parsed_revision == UINT32_MAX) {
      ESP_LOGE(kTag,
               "PTLOG revision overflow in %s; refusing to wrap",
               candidate_path);
      closedir(dir);
      return ESP_ERR_INVALID_SIZE;
    }
    if (parsed_revision >= *revision_out) {
      *revision_out = parsed_revision + 1u;
    }
  }

  closedir(dir);
  return ESP_OK;
}

/**
 * @brief Copy a C string to a destination buffer when it fully fits.
 * @param source Source string.
 * @param dest Destination buffer.
 * @param dest_size Destination buffer size.
 * @return True when copied successfully, otherwise false.
 */
static bool
SdLoggerCopyString(const char* source, char* dest, size_t dest_size)
{
  if (source == NULL || dest == NULL || dest_size == 0) {
    return false;
  }

  const size_t source_len = strnlen(source, dest_size);
  if (source_len == dest_size) {
    return false;
  }

  memcpy(dest, source, source_len + 1);
  return true;
}

static bool
IsDailyPtlogName(const char* name, char* date_out, size_t date_out_size)
{
  if (name == NULL) {
    return false;
  }
  const size_t suffix_len = sizeof(kPtlogSuffix) - 1;
  const size_t length = strlen(name);
  if (length < 11 + suffix_len) {
    return false;
  }
  if (strcmp(name + length - suffix_len, kPtlogSuffix) != 0) {
    return false;
  }

  const size_t prefix_len = length - suffix_len;
  if (prefix_len < 11 || prefix_len > 22) {
    return false;
  }
  if (name[4] != '-' || name[7] != '-' || name[10] != 'Z') {
    return false;
  }
  for (size_t i = 0; i < 11; ++i) {
    if (i == 4 || i == 7) {
      continue;
    }
    if (i == 10) {
      continue;
    }
    if (name[i] < '0' || name[i] > '9') {
      return false;
    }
  }

  if (prefix_len > 11) {
    if (name[11] != '-') {
      return false;
    }
    for (size_t i = 12; i < prefix_len; ++i) {
      if (name[i] < '0' || name[i] > '9') {
        return false;
      }
    }
  }

  if (date_out != NULL && date_out_size > 0) {
    if (date_out_size < 12) {
      return false;
    }
    memcpy(date_out, name, 11);
    date_out[11] = '\0';
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

  uint64_t total_tmp_bytes = 0;
  uint64_t free_tmp_bytes = 0;
  const esp_err_t result =
    esp_vfs_fat_info(logger->mount_point, &total_tmp_bytes, &free_tmp_bytes);
  if (result != ESP_OK) {
    return result;
  }
  *total_bytes = total_tmp_bytes;
  *free_bytes = free_tmp_bytes;
  return ESP_OK;
}

static bool
SdLoggerFindOldestDailyPtlogDateLocked(const sd_logger_t* logger,
                                       char* oldest_date,
                                       size_t oldest_date_size)
{
  if (logger == NULL || oldest_date == NULL || oldest_date_size < 12) {
    return false;
  }

  DIR* dir = opendir(logger->mount_point);
  if (dir == NULL) {
    return false;
  }
  bool found = false;
  oldest_date[0] = '\0';
  struct dirent* entry = NULL;
  while ((entry = readdir(dir)) != NULL) {
    char date_string[16] = "";
    if (!IsDailyPtlogName(entry->d_name, date_string, sizeof(date_string))) {
      continue;
    }

    if (logger->current_date[0] != '\0' &&
        strcmp(date_string, logger->current_date) == 0) {
      continue;
    }

    if (!found || strcmp(date_string, oldest_date) < 0) {
      if (!SdLoggerCopyString(date_string, oldest_date, oldest_date_size)) {
        continue;
      }
      found = true;
    }
  }

  closedir(dir);
  return found;
}

static uint32_t
SdLoggerDeleteDailyPtlogDateLocked(const sd_logger_t* logger,
                                   const char* date_string)
{
  if (logger == NULL || date_string == NULL) {
    return 0;
  }

  DIR* dir = opendir(logger->mount_point);
  if (dir == NULL) {
    return 0;
  }

  uint32_t deleted = 0;
  struct dirent* entry = NULL;
  while ((entry = readdir(dir)) != NULL) {
    char candidate_date[16] = "";
    if (!IsDailyPtlogName(entry->d_name, candidate_date, sizeof(candidate_date))) {
      continue;
    }
    if (strcmp(candidate_date, date_string) != 0) {
      continue;
    }

    char candidate_path[128] = "";
    if (!SdLoggerJoinPath(
          logger->mount_point, entry->d_name, candidate_path, sizeof(candidate_path))) {
      continue;
    }
    if (unlink(candidate_path) != 0) {
      ESP_LOGW(kTag,
               "Failed to delete old log %s: %s (%d)",
               candidate_path,
               strerror(errno),
               errno);
      continue;
    }
    deleted++;
    ESP_LOGW(kTag, "Deleted old log to reclaim space: %s", candidate_path);
  }

  closedir(dir);
  return deleted;
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
  static bool reclaim_scope_logged = false;
  if (!reclaim_scope_logged) {
    ESP_LOGI(kTag, "SD reclaim targets ptlog only");
    reclaim_scope_logged = true;
  }
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
    char oldest_date[16] = "";
    if (!SdLoggerFindOldestDailyPtlogDateLocked(
          logger, oldest_date, sizeof(oldest_date))) {
      break;
    }
    const uint32_t deleted_for_day =
      SdLoggerDeleteDailyPtlogDateLocked(logger, oldest_date);
    if (deleted_for_day == 0) {
      break;
    }
    deletes += deleted_for_day;

    space_result =
      SdLoggerGetSpaceInfoLocked(logger, &total_bytes, &free_bytes);
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
 * @brief Execute CommitHeaderIfEmptyAndSync_.
 * @param file Parameter file.
 * @param header Parameter header.
 * @param path Parameter path.
 * @param file_was_empty_out Parameter file_was_empty_out.
 * @param header_written_out Parameter header_written_out.
 * @return Return the function result.
 */
static esp_err_t
CommitHeaderIfEmptyAndSync_(FILE* file,
                            const ptlog_header_t* header,
                            const char* path,
                            bool* file_was_empty_out,
                            bool* header_written_out)
{
  if (file == NULL || header == NULL || path == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (file_was_empty_out != NULL) {
    *file_was_empty_out = false;
  }
  if (header_written_out != NULL) {
    *header_written_out = false;
  }

  struct stat stat_buffer;
  const int file_descriptor = fileno(file);
  if (file_descriptor < 0 || fstat(file_descriptor, &stat_buffer) != 0) {
    ESP_LOGE(kTag, "fstat() failed for %s: errno=%d (%s)", path, errno, strerror(errno));
    return ESP_FAIL;
  }
  if (stat_buffer.st_size > 0) {
    return ESP_OK;
  }

  if (file_was_empty_out != NULL) {
    *file_was_empty_out = true;
  }
  if (!PtlogWriteHeader(file, header)) {
    ESP_LOGE(kTag, "Failed to write PTLOG header to %s", path);
    return ESP_FAIL;
  }
  ESP_LOGI(kTag, "PTLOG header written to %s", path);
  if (header_written_out != NULL) {
    *header_written_out = true;
  }

  if (fflush(file) != 0) {
    ESP_LOGE(kTag,
             "Header fflush() failed for %s: errno=%d (%s)",
             path,
             errno,
             strerror(errno));
    return ESP_FAIL;
  }
  if (fsync(file_descriptor) != 0) {
    ESP_LOGE(kTag,
             "Header fsync() failed for %s: errno=%d (%s)",
             path,
             errno,
             strerror(errno));
    return ESP_FAIL;
  }
  ESP_LOGI(kTag, "PTLOG header sync committed for %s", path);
  return ESP_OK;
}

static bool
SdLoggerUnlinkEmptyFileIfPresent_(const char* path)
{
  if (path == NULL) {
    return false;
  }

  struct stat stat_buffer;
  if (stat(path, &stat_buffer) != 0) {
    return false;
  }
  if (stat_buffer.st_size != 0) {
    return false;
  }

  if (unlink(path) != 0) {
    ESP_LOGW(kTag,
             "Failed to remove empty PTLOG placeholder %s: errno=%d (%s)",
             path,
             errno,
             strerror(errno));
    return false;
  }
  ESP_LOGW(kTag, "Removed empty PTLOG placeholder %s", path);
  return true;
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
#if defined(ESP_IDF_VERSION) &&                                                \
  (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
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
    SdLoggerResetMountState_(logger);
    ESP_LOGW(kTag, "SD mount failed: %s", esp_err_to_name(result));
    return result;
  }

  logger->is_mounted = true;
  logger->card = card;
  ESP_LOGI(kTag,
           "SD mounted at %s (SDSPI max clock=%u kHz)",
           logger->mount_point,
           (unsigned)logger->config.max_freq_khz);

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
  if (logger->is_mounted && logger->card != NULL) {
    return ESP_OK;
  }
  if (!logger->slot_config_valid) {
    return ESP_ERR_INVALID_STATE;
  }

  if (logger->card != NULL) {
    const esp_err_t stale_unmount_result =
      esp_vfs_fat_sdcard_unmount(logger->mount_point, logger->card);
    if (stale_unmount_result != ESP_OK &&
        stale_unmount_result != ESP_ERR_INVALID_STATE) {
      ESP_LOGW(kTag,
               "SD stale unmount failed (%s): %s",
               logger->mount_point,
               esp_err_to_name(stale_unmount_result));
    }
  }
  SdLoggerResetMountState_(logger);

  esp_err_t result = SdLoggerMountInternal(
    logger, logger->host_id, logger->cs_gpio, format_if_mount_failed);
  if (result == ESP_ERR_INVALID_STATE) {
    ESP_LOGW(kTag, "SD remount invalid-state; resetting and retrying once");
    SdLoggerResetMountState_(logger);
    vTaskDelay(pdMS_TO_TICKS(20));
    result = SdLoggerMountInternal(
      logger, logger->host_id, logger->cs_gpio, format_if_mount_failed);
  }
  if (result != ESP_OK) {
    SdLoggerResetMountState_(logger);
  }
  return result;
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

  if (logger->card != NULL) {
    esp_err_t unmount_result =
      esp_vfs_fat_sdcard_unmount(logger->mount_point, logger->card);
    if (unmount_result != ESP_OK) {
      ESP_LOGW(kTag,
               "SD unmount failed (%s): %s",
               logger->mount_point,
               esp_err_to_name(unmount_result));
    } else {
      logger->card = NULL;
    }
  }

  SdLoggerResetMountState_(logger);
  return ESP_OK;
}

void
SdLoggerResetMountState(sd_logger_t* logger)
{
  SdLoggerResetMountState_(logger);
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
SdLoggerEnsureDailyFileWithHeader(sd_logger_t* logger,
                                  int64_t epoch_utc,
                                  const ptlog_header_t* header,
                                  uint32_t header_signature)
{
  if (logger == NULL || header == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!logger->is_mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  char date_string[16];
  char month_string[16];
  char path[128];
  const bool same_signature = (logger->current_header_signature == header_signature);
  BuildDailyPtlogPath(logger,
                      epoch_utc,
                      0,
                      date_string,
                      sizeof(date_string),
                      month_string,
                      sizeof(month_string),
                      path,
                      sizeof(path));
  if (path[0] == '\0') {
    ESP_LOGE(kTag, "Failed to build nested daily PTLOG path");
    return ESP_FAIL;
  }

  if (logger->file != NULL && logger->current_date[0] != '\0') {
    const int date_cmp = strcmp(date_string, logger->current_date);
    if (date_cmp == 0 && same_signature) {
      return ESP_OK; // already open for today
    }
    if (date_cmp < 0) {
      return ESP_OK; // never roll backward to an earlier date
    }
  }

  uint32_t revision = 0;
  if (logger->current_date[0] != '\0' &&
      strcmp(date_string, logger->current_date) == 0 && !same_signature) {
    esp_err_t revision_result = SdLoggerFindNextRevisionInMonthLocked(
      logger, date_string, month_string, &revision);
    if (revision_result != ESP_OK) {
      return revision_result;
    }
  }

  BuildDailyPtlogPath(logger,
                      epoch_utc,
                      revision,
                      date_string,
                      sizeof(date_string),
                      month_string,
                      sizeof(month_string),
                      path,
                      sizeof(path));
  if (path[0] == '\0') {
    ESP_LOGE(kTag, "Failed to build nested daily PTLOG path for revision %" PRIu32, revision);
    return ESP_FAIL;
  }

  esp_err_t dir_result =
    SdLoggerEnsurePtlogDirectoryLocked(logger, month_string);
  if (dir_result != ESP_OK) {
    return dir_result;
  }

  SdLoggerClose(logger);
  logger->last_record_id_on_sd = 0;

  const bool file_existed_before_open = (access(path, F_OK) == 0);

  logger->file = fopen(path, "a+b");
  if (logger->file == NULL) {
    ESP_LOGE(
      kTag, "fopen failed for %s: %s (%d)", path, strerror(errno), errno);
    return ESP_FAIL;
  }
  ESP_LOGI(kTag,
           "%s daily PTLOG file %s",
           file_existed_before_open ? "Opened existing" : "Created",
           path);

  if (logger->file_buffer != NULL) {
    setvbuf((FILE*)logger->file,
            (char*)logger->file_buffer,
            _IOFBF,
            logger->config.file_buffer_bytes);
  }

  esp_err_t resume_result = ApplyResumeInfo(logger, logger->file, path);
  if (resume_result != ESP_OK) {
    fclose(logger->file);
    logger->file = NULL;
    return resume_result;
  }

  bool file_was_empty = false;
  bool header_written = false;
  esp_err_t header_result = CommitHeaderIfEmptyAndSync_(
    logger->file, header, path, &file_was_empty, &header_written);
  if (header_result != ESP_OK) {
    ESP_LOGE(kTag, "Failed to commit header for %s", path);
    fclose(logger->file);
    logger->file = NULL;
    if (!file_existed_before_open && file_was_empty) {
      (void)SdLoggerUnlinkEmptyFileIfPresent_(path);
    }
    return header_result;
  }
  if (header_written) {
    ESP_LOGI(kTag, "Daily PTLOG header ready for %s", path);
  }

  strncpy(logger->current_date, date_string, sizeof(logger->current_date) - 1);
  logger->current_date[sizeof(logger->current_date) - 1] = '\0';
  logger->current_header_signature = header_signature;
  logger->pending_header = NULL;
  return ESP_OK;
}

esp_err_t
SdLoggerEnsureDailyFile(sd_logger_t* logger, int64_t epoch_utc)
{
  (void)epoch_utc;
  if (logger == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_ERR_INVALID_STATE;
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
  if (SdLoggerGetSpaceInfoLocked(logger, &total_bytes, &free_bytes_before) ==
      ESP_OK) {
    stats_out->space_free_bytes_before = free_bytes_before;
    if (free_bytes_before < required_free_bytes) {
      uint32_t deleted_files = 0;
      if (SdLoggerReclaimSpaceLocked(
            logger, required_free_bytes, &deleted_files) == ESP_OK &&
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
      if (SdLoggerReclaimSpaceLocked(
            logger, required_free_bytes, &deleted_files) == ESP_OK &&
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
        if (SdLoggerReclaimSpaceLocked(
              logger, required_free_bytes, &deleted_files) == ESP_OK &&
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
        if (SdLoggerReclaimSpaceLocked(
              logger, required_free_bytes, &deleted_files) == ESP_OK &&
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
      const int errno_first = errno;
      clearerr(logger->file);
      vTaskDelay(pdMS_TO_TICKS(50));
      if (fflush(logger->file) == 0) {
        ESP_LOGW(kTag,
                 "fflush() recovered on retry (first errno=%d: %s)",
                 errno_first,
                 strerror(errno_first));
      } else {
        ESP_LOGE(kTag, "fflush() failed: errno=%d (%s)", errno, strerror(errno));
        SetAppendDiagnostics(&stats_out->diag, "fflush", errno);
        if (!retry_after_reclaim && stats_out->diag.errno_value == ENOSPC) {
          uint32_t deleted_files = 0;
          if (SdLoggerReclaimSpaceLocked(
                logger, required_free_bytes, &deleted_files) == ESP_OK &&
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
  }

  stats_out->bytes_appended = batch_length_bytes;
  stats_out->write_calls = write_calls;
  logger->last_record_id_on_sd = last_record_id;
  (void)SdLoggerGetSpaceInfoLocked(
    logger, &total_bytes, &stats_out->space_free_bytes_after);
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
  logger->current_header_signature = 0;
  logger->pending_header = NULL;
}

static void
SdLoggerResetMountState_(sd_logger_t* logger)
{
  if (logger == NULL) {
    return;
  }
  SdLoggerClose(logger);
  logger->is_mounted = false;
  logger->card = NULL;
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
