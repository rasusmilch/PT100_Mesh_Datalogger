#include "diagnostics/diag_storage.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "data_csv.h"
#include "esp_err.h"
#include "fram_log.h"
#include "nvs.h"
#include "sd_logger.h"
#include "sd_ptlog_paths.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* kNvsNamespace = "pt100_logger";
static const char* kRecordIdKey = "diag_recid";

/**
 * @brief Build the compact nested PTLOG path used by storage diagnostics.
 *
 * The diagnostic path is delegated to SdPtlogBuildNestedPathWithWorkspace()
 * with revision 0 so self-tests exercise the same /logs/YYYY-MM/YYYYMMDD.RRR
 * layout as normal firmware-created PTLOG files. The caller must hold
 * RuntimeSdIoLock() because the helper uses the shared logger-owned PTLOG path
 * workspace. The final path is copied into the caller-owned path_out buffer;
 * no workspace pointer escapes. Invalid arguments or a missing logger-owned
 * workspace leave path_out empty.
 */
static void
BuildDailyPtlogPath(const sd_logger_t* logger,
                  int64_t epoch_seconds,
                  char* path_out,
                  size_t path_out_size)
{
  if (path_out != NULL && path_out_size > 0) {
    path_out[0] = '\0';
  }
  if (logger == NULL || logger->ptlog_workspace == NULL || path_out == NULL ||
      path_out_size == 0) {
    return;
  }

  char date_string[SD_PTLOG_DATE_LEN + 1u];
  char month_string[SD_PTLOG_MONTH_LEN + 1u];
  (void)SdPtlogBuildNestedPathWithWorkspace(logger->ptlog_workspace,
                                            logger->mount_point,
                                            epoch_seconds,
                                            0,
                                            date_string,
                                            sizeof(date_string),
                                            month_string,
                                            sizeof(month_string),
                                            path_out,
                                            path_out_size);
}

/**
 * @brief Create a deterministic synthetic PT100 record for storage diagnostics.
 *
 * The fixed temperature/resistance values keep power-loss and flush self-tests
 * focused on persistence behavior rather than sensor sampling state.
 */
static log_record_t
BuildDiagRecord(int64_t epoch_seconds)
{
  log_record_t record;
  memset(&record, 0, sizeof(record));
  record.timestamp_epoch_sec = epoch_seconds;
  record.timestamp_millis = 0;
  record.raw_temp_milli_c = 25000;
  record.temp_milli_c = 25000;
  record.resistance_milli_ohm = 100000;
  record.flags =
    (uint16_t)(LOG_RECORD_FLAG_TIME_VALID | LOG_RECORD_FLAG_CAL_VALID);
  return record;
}

/**
 * @brief Assign a record id and append one diagnostic record to FRAM.
 *
 * The helper preserves the production ordering of id assignment before append so
 * storage diagnostics exercise the same FRAM queue semantics as normal logging.
 */
static esp_err_t
AppendRecordToFram(fram_log_t* fram, log_record_t* record)
{
  if (fram == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t id_result = FramLogAssignRecordIds(fram, record);
  if (id_result != ESP_OK) {
    return id_result;
  }
  return FramLogAppend(fram, record);
}

/**
 * @brief Write and sync a deliberately truncated CSV row to the active PTLOG.
 *
 * Power-loss diagnostics use this helper to create an incomplete tail record.
 * The expected recovery behavior is preservation: firmware must leave the old
 * partial revision untouched and select a new same-day revision for replay.
 * full_len_out receives the intended full row length when requested.
 */
static esp_err_t
WritePartialCsvLine(sd_logger_t* logger,
                    const log_record_t* record,
                    const char* node_id,
                    size_t* full_len_out)
{
  if (logger == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (logger->file == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  char line[256];
  size_t line_len = 0;
  if (!CsvFormatRow(record, node_id, line, sizeof(line), &line_len)) {
    return ESP_FAIL;
  }

  if (full_len_out != NULL) {
    *full_len_out = line_len;
  }

  size_t partial_len = line_len / 2;
  if (partial_len == 0 && line_len > 1) {
    partial_len = line_len - 1;
  }
  if (partial_len == 0 || partial_len >= line_len) {
    return ESP_ERR_INVALID_SIZE;
  }

  const size_t written = fwrite(line, 1, partial_len, logger->file);
  if (written != partial_len) {
    return ESP_FAIL;
  }

  if (fflush(logger->file) != 0) {
    return ESP_FAIL;
  }

  const int fd = fileno(logger->file);
  if (fd >= 0 && fsync(fd) != 0) {
    return ESP_FAIL;
  }

  return ESP_OK;
}

static bool
CopyActivePtlogPath(const sd_logger_t* logger, char* path_out, size_t path_out_size)
{
  /*
   * The logger-owned PTLOG workspace is shared scratch storage; callers must
   * hold RuntimeSdIoLock() while copying the active path out of it.
   */
  if (path_out != NULL && path_out_size > 0) {
    path_out[0] = '\0';
  }
  if (logger == NULL || logger->ptlog_workspace == NULL || path_out == NULL ||
      path_out_size == 0 || logger->ptlog_workspace->daily_path[0] == '\0') {
    return false;
  }
  const int written = snprintf(path_out,
                               path_out_size,
                               "%s",
                               logger->ptlog_workspace->daily_path);
  return written >= 0 && (size_t)written < path_out_size;
}

static bool
GetFileSizeBytes(const char* path, off_t* size_out)
{
  if (path == NULL || size_out == NULL) {
    return false;
  }
  struct stat stat_buffer;
  if (stat(path, &stat_buffer) != 0 || !S_ISREG(stat_buffer.st_mode)) {
    return false;
  }
  *size_out = stat_buffer.st_size;
  return true;
}

static const char*
PathLeaf(const char* path)
{
  if (path == NULL) {
    return "";
  }
  const char* leaf = strrchr(path, '/');
  return (leaf == NULL) ? path : leaf + 1;
}

/**
 * @brief Verify power-loss preservation selects a new revision.
 *
 * The diagnostic deliberately leaves a partial CSV row in the active PTLOG,
 * closes the logger to simulate uncertainty, and reopens through the normal
 * daily-open path. Passing requires a different same-day revision, unchanged old
 * file size, and successful replay of the pending FRAM record to the new active
 * revision. Duplicate records across revisions are acceptable until host-side
 * recovery and record_id dedupe are implemented.
 */
static bool
RunPowerLossPreserveTest(const app_runtime_t* runtime,
                         bool full,
                         char* details,
                         size_t details_len)
{
  if (runtime == NULL || runtime->sd_logger == NULL ||
      runtime->fram_log == NULL || runtime->flush_callback == NULL) {
    snprintf(
      details, details_len, "runtime missing (sd/fram/flush unavailable)");
    return false;
  }

  sd_logger_t* logger = runtime->sd_logger;
  fram_log_t* fram = runtime->fram_log;
  const int records = full ? 8 : 3;
  const int64_t base_time = (int64_t)time(NULL);

  for (int i = 0; i < records; ++i) {
    log_record_t record = BuildDiagRecord(base_time + i);
    esp_err_t append_result = AppendRecordToFram(fram, &record);
    if (append_result != ESP_OK) {
      snprintf(details,
               details_len,
               "fram append failed: %s",
               esp_err_to_name(append_result));
      return false;
    }
  }

  esp_err_t flush_result = runtime->flush_callback(runtime->flush_context);
  if (flush_result != ESP_OK) {
    snprintf(
      details, details_len, "flush failed: %s", esp_err_to_name(flush_result));
    return false;
  }

  const uint64_t last_sd_before = SdLoggerLastRecordIdOnSd(logger);

  log_record_t pending_record = BuildDiagRecord(base_time + records);
  esp_err_t pending_append = AppendRecordToFram(fram, &pending_record);
  if (pending_append != ESP_OK) {
    snprintf(details,
             details_len,
             "fram append pending failed: %s",
             esp_err_to_name(pending_append));
    return false;
  }

  runtime_state_t* state = RuntimeGetState();
  if (state == NULL || !RuntimeSdIoLock(state, pdMS_TO_TICKS(2000))) {
    snprintf(details, details_len, "sd io lock failed");
    return false;
  }

  ptlog_header_t ptlog_header;
  uint32_t header_signature = 0;
  if (!RuntimeBuildPtlogHeader(
        pending_record.timestamp_epoch_sec, &ptlog_header, &header_signature)) {
    RuntimeSdIoUnlock(state);
    snprintf(details, details_len, "ptlog header build failed");
    return false;
  }

  esp_err_t ensure_result =
    SdLoggerEnsureDailyFileWithHeader(logger,
                                      pending_record.timestamp_epoch_sec,
                                      &ptlog_header,
                                      header_signature);
  if (ensure_result != ESP_OK) {
    RuntimeSdIoUnlock(state);
    snprintf(details,
             details_len,
             "ensure daily file failed: %s",
             esp_err_to_name(ensure_result));
    return false;
  }

  size_t line_len = 0;
  esp_err_t partial_result = WritePartialCsvLine(
    logger, &pending_record, runtime->node_id_string, &line_len);
  if (partial_result != ESP_OK) {
    RuntimeSdIoUnlock(state);
    snprintf(details,
             details_len,
             "partial write failed: %s",
             esp_err_to_name(partial_result));
    return false;
  }

  char old_path[SD_PTLOG_MAX_PATH_LEN];
  if (!CopyActivePtlogPath(logger, old_path, sizeof(old_path))) {
    RuntimeSdIoUnlock(state);
    snprintf(details, details_len, "active PTLOG path unavailable");
    return false;
  }
  off_t old_size_before = 0;
  if (!GetFileSizeBytes(old_path, &old_size_before)) {
    RuntimeSdIoUnlock(state);
    snprintf(details, details_len, "old PTLOG stat failed");
    return false;
  }

  SdLoggerClose(logger);
  esp_err_t reopen_result =
    SdLoggerEnsureDailyFileWithHeader(logger,
                                      pending_record.timestamp_epoch_sec,
                                      &ptlog_header,
                                      header_signature);
  char new_path[SD_PTLOG_MAX_PATH_LEN];
  const bool copied_new_path =
    CopyActivePtlogPath(logger, new_path, sizeof(new_path));
  off_t old_size_after = 0;
  const bool old_stat_after = GetFileSizeBytes(old_path, &old_size_after);
  const uint64_t last_sd_after_reopen = SdLoggerLastRecordIdOnSd(logger);
  RuntimeSdIoUnlock(state);
  if (reopen_result != ESP_OK) {
    snprintf(details,
             details_len,
             "reopen failed: %s",
             esp_err_to_name(reopen_result));
    return false;
  }
  if (!copied_new_path) {
    snprintf(details, details_len, "new PTLOG path unavailable");
    return false;
  }
  if (!old_stat_after) {
    snprintf(details, details_len, "old PTLOG stat after reopen failed");
    return false;
  }

  esp_err_t replay_result = runtime->flush_callback(runtime->flush_context);
  const uint64_t last_sd_after_flush = SdLoggerLastRecordIdOnSd(logger);
  const uint32_t fram_buffered = FramLogGetBufferedRecords(fram);

  const bool new_revision = strcmp(old_path, new_path) != 0;
  const bool old_preserved = (old_size_before == old_size_after);
  const bool no_reopen_resume = (last_sd_after_reopen == 0);
  const bool replay_ok = (replay_result == ESP_OK &&
                          last_sd_after_flush == pending_record.record_id);

  snprintf(details,
           details_len,
           "old_preserved=%s new_revision=%s no_reopen_resume=%s"
           " old_size_before=%" PRIdMAX " old_size_after=%" PRIdMAX
           " last_sd_before=%" PRIu64 " last_sd_after_reopen=%" PRIu64
           " pending_id=%" PRIu64 " last_sd_after_flush=%" PRIu64
           " line_len=%u fram_buffered=%u old=%s new=%s",
           old_preserved ? "yes" : "no",
           new_revision ? "yes" : "no",
           no_reopen_resume ? "yes" : "no",
           (intmax_t)old_size_before,
           (intmax_t)old_size_after,
           last_sd_before,
           last_sd_after_reopen,
           pending_record.record_id,
           last_sd_after_flush,
           (unsigned)line_len,
           (unsigned)fram_buffered,
           PathLeaf(old_path),
           PathLeaf(new_path));

  return old_preserved && new_revision && no_reopen_resume && replay_ok;
}

/**
 * @brief Execute RunSdPullTest.
 * @param runtime Parameter runtime.
 * @param full Parameter full.
 * @param details Parameter details.
 * @param details_len Parameter details_len.
 * @return Return the function result.
 */
static bool
RunSdPullTest(const app_runtime_t* runtime,
              bool full,
              char* details,
              size_t details_len)
{
  if (runtime == NULL || runtime->sd_logger == NULL ||
      runtime->fram_log == NULL || runtime->flush_callback == NULL) {
    snprintf(
      details, details_len, "runtime missing (sd/fram/flush unavailable)");
    return false;
  }

  sd_logger_t* logger = runtime->sd_logger;
  fram_log_t* fram = runtime->fram_log;
  const int records = full ? 12 : 4;
  const int64_t base_time = (int64_t)time(NULL);
  uint64_t last_record_id = 0;

  for (int i = 0; i < records; ++i) {
    log_record_t record = BuildDiagRecord(base_time + i);
    esp_err_t append_result = AppendRecordToFram(fram, &record);
    if (append_result != ESP_OK) {
      snprintf(details,
               details_len,
               "fram append failed: %s",
               esp_err_to_name(append_result));
      return false;
    }
    last_record_id = record.record_id;
  }

  RuntimeSetSdAppendFailureOnce(true);
  esp_err_t flush_result = runtime->flush_callback(runtime->flush_context);

  const uint32_t buffered_after_fail = FramLogGetBufferedRecords(fram);
  const TickType_t now_ticks = xTaskGetTickCount();
  const uint32_t backoff_until = RuntimeSdBackoffUntilTicks();
  const bool backoff_active =
    RuntimeSdIsDegraded() && backoff_until > (uint32_t)now_ticks;

  esp_err_t remount_result = ESP_ERR_TIMEOUT;
  runtime_state_t* state = RuntimeGetState();
  if (state != NULL && RuntimeSdIoLock(state, pdMS_TO_TICKS(2000))) {
    remount_result = SdLoggerTryRemount(logger, false);
    RuntimeSdIoUnlock(state);
  }
  esp_err_t replay_result = runtime->flush_callback(runtime->flush_context);

  const uint32_t buffered_after_replay = FramLogGetBufferedRecords(fram);
  const uint64_t last_sd = SdLoggerLastRecordIdOnSd(logger);

  const bool expected_fail = (flush_result != ESP_OK);
  const bool buffered_ok = (buffered_after_fail >= (uint32_t)records);
  const bool replay_ok =
    (replay_result == ESP_OK && buffered_after_replay == 0 &&
     last_sd >= last_record_id);

  snprintf(details,
           details_len,
           "flush_failed=%s backoff=%s backoff_until=%u remount=%s "
           "buffered_after_fail=%u buffered_after_replay=%u last_sd=%" PRIu64
           " last_expected=%" PRIu64,
           expected_fail ? "yes" : "no",
           backoff_active ? "yes" : "no",
           (unsigned)backoff_until,
           esp_err_to_name(remount_result),
           (unsigned)buffered_after_fail,
           (unsigned)buffered_after_replay,
           last_sd,
           last_record_id);

  return expected_fail && backoff_active && buffered_ok && replay_ok;
}

static bool
ParseRecordIdFromCsvLine(const char* line, uint64_t* record_id_out)
{
  if (line == NULL || record_id_out == NULL) {
    return false;
  }
  const char* comma = strchr(line, ',');
  if (comma == NULL || comma[1] < '0' || comma[1] > '9') {
    return false;
  }
  uint64_t value = 0;
  for (const char* cursor = comma + 1; *cursor != '\0' && *cursor != ',';
       ++cursor) {
    if (*cursor < '0' || *cursor > '9') {
      return false;
    }
    value = value * 10u + (uint64_t)(*cursor - '0');
  }
  *record_id_out = value;
  return true;
}

/**
 * @brief Read the last complete CSV record id without repairing the file.
 *
 * Storage diagnostics use this helper for observation only. It opens PTLOG files
 * read-only, ignores any incomplete trailing line, and never calls the
 * destructive tail-repair helper, preserving partial revisions for host-side
 * recovery.
 */
static bool
ReadLastRecordId(const char* path,
                 size_t tail_scan_bytes,
                 bool* found_out,
                 uint64_t* record_id_out)
{
  if (path == NULL || found_out == NULL || record_id_out == NULL) {
    return false;
  }
  *found_out = false;
  *record_id_out = 0;
  FILE* file = fopen(path, "rb");
  if (file == NULL) {
    return false;
  }
  if (fseeko(file, 0, SEEK_END) != 0) {
    fclose(file);
    return false;
  }
  const off_t file_size = ftello(file);
  if (file_size <= 0) {
    fclose(file);
    return true;
  }
  const off_t scan_start =
    (file_size > (off_t)tail_scan_bytes) ? file_size - (off_t)tail_scan_bytes : 0;
  const size_t scan_length = (size_t)(file_size - scan_start);
  char* tail_bytes = (char*)malloc(scan_length + 1u);
  if (tail_bytes == NULL) {
    fclose(file);
    return false;
  }
  if (fseeko(file, scan_start, SEEK_SET) != 0 ||
      fread(tail_bytes, 1, scan_length, file) != scan_length) {
    free(tail_bytes);
    fclose(file);
    return false;
  }
  fclose(file);
  tail_bytes[scan_length] = '\0';

  ssize_t line_end = (ssize_t)scan_length - 1;
  if (line_end >= 0 && tail_bytes[(size_t)line_end] != '\n') {
    while (line_end >= 0 && tail_bytes[(size_t)line_end] != '\n') {
      --line_end;
    }
    --line_end;
  } else if (line_end >= 0) {
    --line_end;
  }

  while (line_end >= 0) {
    ssize_t line_start = line_end;
    while (line_start >= 0 && tail_bytes[(size_t)line_start] != '\n') {
      --line_start;
    }
    const size_t line_offset = (size_t)(line_start + 1);
    const size_t line_length = (size_t)(line_end - line_start);
    tail_bytes[line_offset + line_length] = '\0';
    uint64_t record_id = 0;
    if (ParseRecordIdFromCsvLine(&tail_bytes[line_offset], &record_id)) {
      *found_out = true;
      *record_id_out = record_id;
      free(tail_bytes);
      return true;
    }
    line_end = line_start - 1;
  }

  free(tail_bytes);
  return true;
}

/**
 * @brief Execute RunMidnightSplitTest.
 * @param runtime Parameter runtime.
 * @param details Parameter details.
 * @param details_len Parameter details_len.
 * @return Return the function result.
 */
static bool
RunMidnightSplitTest(const app_runtime_t* runtime,
                     char* details,
                     size_t details_len)
{
  if (runtime == NULL || runtime->sd_logger == NULL ||
      runtime->fram_log == NULL || runtime->flush_callback == NULL) {
    snprintf(
      details, details_len, "runtime missing (sd/fram/flush unavailable)");
    return false;
  }

  sd_logger_t* logger = runtime->sd_logger;
  fram_log_t* fram = runtime->fram_log;
  const int64_t epoch_day1 = (int64_t)time(NULL);
  const int64_t epoch_day2 = epoch_day1 + 86400 + 5;

  log_record_t record_day1 = BuildDiagRecord(epoch_day1);
  log_record_t record_day2 = BuildDiagRecord(epoch_day2);

  esp_err_t append1 = AppendRecordToFram(fram, &record_day1);
  esp_err_t append2 = AppendRecordToFram(fram, &record_day2);
  if (append1 != ESP_OK || append2 != ESP_OK) {
    snprintf(details,
             details_len,
             "fram append failed: %s/%s",
             esp_err_to_name(append1),
             esp_err_to_name(append2));
    return false;
  }

  esp_err_t flush_result = runtime->flush_callback(runtime->flush_context);
  if (flush_result != ESP_OK) {
    snprintf(
      details, details_len, "flush failed: %s", esp_err_to_name(flush_result));
    return false;
  }

  char path_day1[128];
  char path_day2[128];
  runtime_state_t* state = RuntimeGetState();
  if (state == NULL || !RuntimeSdIoLock(state, pdMS_TO_TICKS(2000))) {
    snprintf(details,
             details_len,
             "sd io lock failed while building diagnostic PTLOG paths");
    return false;
  }
  BuildDailyPtlogPath(logger, epoch_day1, path_day1, sizeof(path_day1));
  BuildDailyPtlogPath(logger, epoch_day2, path_day2, sizeof(path_day2));
  RuntimeSdIoUnlock(state);

  bool found_day1 = false;
  bool found_day2 = false;
  uint64_t last_day1 = 0;
  uint64_t last_day2 = 0;
  const bool read_day1 = ReadLastRecordId(
    path_day1, logger->config.tail_scan_bytes, &found_day1, &last_day1);
  const bool read_day2 = ReadLastRecordId(
    path_day2, logger->config.tail_scan_bytes, &found_day2, &last_day2);

  const bool distinct_files = strcmp(path_day1, path_day2) != 0;
  const bool day1_ok =
    read_day1 && found_day1 && last_day1 >= record_day1.record_id;
  const bool day2_ok =
    read_day2 && found_day2 && last_day2 >= record_day2.record_id;

  snprintf(details,
           details_len,
           "files=%s day1=%s day2=%s id1=%" PRIu64 " id2=%" PRIu64
           " last1=%" PRIu64 " last2=%" PRIu64 " path1=%s path2=%s",
           distinct_files ? "split" : "same",
           day1_ok ? "ok" : "fail",
           day2_ok ? "ok" : "fail",
           record_day1.record_id,
           record_day2.record_id,
           last_day1,
           last_day2,
           path_day1,
           path_day2);

  return distinct_files && day1_ok && day2_ok;
}

/**
 * @brief Execute RunRecordIdContinuityTest.
 * @param runtime Parameter runtime.
 * @param details Parameter details.
 * @param details_len Parameter details_len.
 * @return Return the function result.
 */
static bool
RunRecordIdContinuityTest(const app_runtime_t* runtime,
                          char* details,
                          size_t details_len)
{
  if (runtime == NULL || runtime->fram_log == NULL) {
    snprintf(details, details_len, "runtime missing (fram unavailable)");
    return false;
  }

  nvs_handle_t handle;
  esp_err_t open_result = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
  if (open_result != ESP_OK) {
    snprintf(details,
             details_len,
             "nvs open failed: %s",
             esp_err_to_name(open_result));
    return false;
  }

  uint64_t previous = 0;
  esp_err_t read_result = nvs_get_u64(handle, kRecordIdKey, &previous);
  if (read_result == ESP_ERR_NVS_NOT_FOUND) {
    previous = 0;
  } else if (read_result != ESP_OK) {
    nvs_close(handle);
    snprintf(details,
             details_len,
             "nvs read failed: %s",
             esp_err_to_name(read_result));
    return false;
  }

  const uint64_t current = FramLogNextRecordId(runtime->fram_log);
  const bool monotonic = (previous == 0 || current >= previous);

  esp_err_t set_result = nvs_set_u64(handle, kRecordIdKey, current);
  esp_err_t commit_result =
    (set_result == ESP_OK) ? nvs_commit(handle) : set_result;
  nvs_close(handle);

  snprintf(details,
           details_len,
           "prev=%" PRIu64 " current=%" PRIu64 " monotonic=%s commit=%s",
           previous,
           current,
           monotonic ? "yes" : "no",
           esp_err_to_name(commit_result));

  return monotonic && commit_result == ESP_OK;
}

/**
 * @brief Execute RunDiagStorageImpl.
 * @param runtime Parameter runtime.
 * @param full Parameter full.
 * @param verbosity Parameter verbosity.
 * @return Return the function result.
 */
static int
RunDiagStorageImpl(const app_runtime_t* runtime,
                   bool full,
                   diag_verbosity_t verbosity)
{
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "Storage", verbosity);
  const int total_steps = 4;
  int step_index = 1;

  if (runtime == NULL || runtime->sd_logger == NULL ||
      runtime->fram_log == NULL) {
    DiagReportStep(&ctx,
                   step_index++,
                   total_steps,
                   "runtime available",
                   ESP_ERR_INVALID_STATE,
                   "runtime not initialized");
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  if (!runtime->sd_logger->is_mounted) {
    runtime_state_t* state = RuntimeGetState();
    if (state != NULL && RuntimeSdIoLock(state, pdMS_TO_TICKS(2000))) {
      (void)SdLoggerTryRemount(runtime->sd_logger, false);
      RuntimeSdIoUnlock(state);
    }
  }

  char details[256];
  bool pass = RunPowerLossPreserveTest(runtime, full, details, sizeof(details));
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "revision preserve",
                 pass ? ESP_OK : ESP_FAIL,
                 "%s (last_record_id=%" PRIu64 " fram_buffered=%u)",
                 details,
                 SdLoggerLastRecordIdOnSd(runtime->sd_logger),
                 (unsigned)FramLogGetBufferedRecords(runtime->fram_log));

  pass = RunSdPullTest(runtime, full, details, sizeof(details));
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "sd pull",
                 pass ? ESP_OK : ESP_FAIL,
                 "%s (last_record_id=%" PRIu64 " fram_buffered=%u)",
                 details,
                 SdLoggerLastRecordIdOnSd(runtime->sd_logger),
                 (unsigned)FramLogGetBufferedRecords(runtime->fram_log));

  pass = RunMidnightSplitTest(runtime, details, sizeof(details));
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "midnight split",
                 pass ? ESP_OK : ESP_FAIL,
                 "%s (last_record_id=%" PRIu64 " fram_buffered=%u)",
                 details,
                 SdLoggerLastRecordIdOnSd(runtime->sd_logger),
                 (unsigned)FramLogGetBufferedRecords(runtime->fram_log));

  pass = RunRecordIdContinuityTest(runtime, details, sizeof(details));
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "record_id continuity",
                 pass ? ESP_OK : ESP_FAIL,
                 "%s (last_record_id=%" PRIu64 " fram_buffered=%u)",
                 details,
                 SdLoggerLastRecordIdOnSd(runtime->sd_logger),
                 (unsigned)FramLogGetBufferedRecords(runtime->fram_log));

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}

typedef struct
{
  bool full;
  diag_verbosity_t verbosity;
  int result;
} diag_storage_ctx_t;

/**
 * @brief Execute RunDiagStorageWithMount.
 * @param runtime Parameter runtime.
 * @param ctx Parameter ctx.
 * @return Return the function result.
 */
static esp_err_t
RunDiagStorageWithMount(app_runtime_t* runtime, void* ctx)
{
  diag_storage_ctx_t* args = (diag_storage_ctx_t*)ctx;
  args->result = RunDiagStorageImpl(runtime, args->full, args->verbosity);
  return (args->result == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Execute RunDiagStorage.
 * @param runtime Parameter runtime.
 * @param full Parameter full.
 * @param verbosity Parameter verbosity.
 * @return Return the function result.
 */
int
RunDiagStorage(const app_runtime_t* runtime,
               bool full,
               diag_verbosity_t verbosity)
{
  if (!RuntimeIsRunning()) {
    diag_storage_ctx_t ctx = {
      .full = full,
      .verbosity = verbosity,
      .result = 1,
    };
    esp_err_t result =
      RuntimeWithTemporarySdMount(&RunDiagStorageWithMount, &ctx);
    if (result != ESP_OK) {
      return 1;
    }
    return ctx.result;
  }

  return RunDiagStorageImpl(runtime, full, verbosity);
}
