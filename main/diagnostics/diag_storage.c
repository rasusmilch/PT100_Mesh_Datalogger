#include "diagnostics/diag_storage.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "data_csv.h"
#include "esp_err.h"
#include "fram_log.h"
#include "nvs.h"
#include "sd_csv_verify.h"
#include "sd_logger.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* kNvsNamespace = "pt100_logger";
static const char* kRecordIdKey = "diag_recid";

static void
BuildDailyCsvPath(const sd_logger_t* logger,
                  int64_t epoch_seconds,
                  char* path_out,
                  size_t path_out_size)
{
  if (logger == NULL || path_out == NULL || path_out_size == 0) {
    return;
  }
  time_t time_seconds = (time_t)epoch_seconds;
  struct tm time_info;
  gmtime_r(&time_seconds, &time_info);

  char date_string[16];
  strftime(date_string, sizeof(date_string), "%Y-%m-%dZ", &time_info);
  snprintf(path_out,
           path_out_size,
           "%s/%s.csv",
           logger->mount_point,
           date_string);
}

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
  record.flags = (uint16_t)(LOG_RECORD_FLAG_TIME_VALID | LOG_RECORD_FLAG_CAL_VALID);
  return record;
}

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
RunPowerLossTailTest(const app_runtime_t* runtime,
                     bool full,
                     char* details,
                     size_t details_len)
{
  if (runtime == NULL || runtime->sd_logger == NULL || runtime->fram_log == NULL ||
      runtime->flush_callback == NULL) {
    snprintf(details,
             details_len,
             "runtime missing (sd/fram/flush unavailable)");
    return false;
  }

  sd_logger_t* logger = runtime->sd_logger;
  fram_log_t* fram = runtime->fram_log;
  const int records = full ? 8 : 3;
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

  esp_err_t flush_result = runtime->flush_callback(runtime->flush_context);
  if (flush_result != ESP_OK) {
    snprintf(details,
             details_len,
             "flush failed: %s",
             esp_err_to_name(flush_result));
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

  esp_err_t ensure_result =
    SdLoggerEnsureDailyFile(logger, pending_record.timestamp_epoch_sec);
  if (ensure_result != ESP_OK) {
    snprintf(details,
             details_len,
             "ensure daily file failed: %s",
             esp_err_to_name(ensure_result));
    return false;
  }

  size_t line_len = 0;
  esp_err_t partial_result = WritePartialCsvLine(logger,
                                                 &pending_record,
                                                 runtime->node_id_string,
                                                 &line_len);
  if (partial_result != ESP_OK) {
    snprintf(details,
             details_len,
             "partial write failed: %s",
             esp_err_to_name(partial_result));
    return false;
  }

  SdLoggerClose(logger);
  esp_err_t reopen_result =
    SdLoggerEnsureDailyFile(logger, pending_record.timestamp_epoch_sec);
  if (reopen_result != ESP_OK) {
    snprintf(details,
             details_len,
             "reopen failed: %s",
             esp_err_to_name(reopen_result));
    return false;
  }

  const uint64_t last_sd_after_reopen = SdLoggerLastRecordIdOnSd(logger);
  esp_err_t replay_result = runtime->flush_callback(runtime->flush_context);
  const uint64_t last_sd_after_flush = SdLoggerLastRecordIdOnSd(logger);
  const uint32_t fram_buffered = FramLogGetBufferedRecords(fram);

  const bool tail_ok = (last_sd_after_reopen == last_sd_before);
  const bool replay_ok =
    (replay_result == ESP_OK && last_sd_after_flush == pending_record.record_id);

  snprintf(details,
           details_len,
           "tail_repaired=%s last_sd_before=%" PRIu64
           " last_sd_after_reopen=%" PRIu64 " pending_id=%" PRIu64
           " last_sd_after_flush=%" PRIu64 " line_len=%u fram_buffered=%u",
           tail_ok ? "yes" : "no",
           last_sd_before,
           last_sd_after_reopen,
           pending_record.record_id,
           last_sd_after_flush,
           (unsigned)line_len,
           (unsigned)fram_buffered);

  return tail_ok && replay_ok;
}

static bool
RunSdPullTest(const app_runtime_t* runtime,
              bool full,
              char* details,
              size_t details_len)
{
  if (runtime == NULL || runtime->sd_logger == NULL || runtime->fram_log == NULL ||
      runtime->flush_callback == NULL) {
    snprintf(details,
             details_len,
             "runtime missing (sd/fram/flush unavailable)");
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

  esp_err_t remount_result = SdLoggerTryRemount(logger, false);
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
ReadLastRecordId(const char* path,
                 size_t tail_scan_bytes,
                 bool* found_out,
                 uint64_t* record_id_out)
{
  if (path == NULL || found_out == NULL || record_id_out == NULL) {
    return false;
  }
  FILE* file = fopen(path, "r+b");
  if (file == NULL) {
    *found_out = false;
    *record_id_out = 0;
    return false;
  }
  SdCsvResumeInfo info = { 0 };
  esp_err_t result =
    SdCsvFindLastRecordIdAndRepairTail(file, tail_scan_bytes, &info);
  fclose(file);
  if (result != ESP_OK) {
    *found_out = false;
    *record_id_out = 0;
    return false;
  }
  *found_out = info.found_last_record_id;
  *record_id_out = info.last_record_id;
  return true;
}

static bool
RunMidnightSplitTest(const app_runtime_t* runtime,
                     char* details,
                     size_t details_len)
{
  if (runtime == NULL || runtime->sd_logger == NULL || runtime->fram_log == NULL ||
      runtime->flush_callback == NULL) {
    snprintf(details,
             details_len,
             "runtime missing (sd/fram/flush unavailable)");
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
    snprintf(details,
             details_len,
             "flush failed: %s",
             esp_err_to_name(flush_result));
    return false;
  }

  char path_day1[128];
  char path_day2[128];
  BuildDailyCsvPath(logger, epoch_day1, path_day1, sizeof(path_day1));
  BuildDailyCsvPath(logger, epoch_day2, path_day2, sizeof(path_day2));

  bool found_day1 = false;
  bool found_day2 = false;
  uint64_t last_day1 = 0;
  uint64_t last_day2 = 0;
  const bool read_day1 =
    ReadLastRecordId(path_day1, logger->config.tail_scan_bytes, &found_day1, &last_day1);
  const bool read_day2 =
    ReadLastRecordId(path_day2, logger->config.tail_scan_bytes, &found_day2, &last_day2);

  const bool distinct_files = strcmp(path_day1, path_day2) != 0;
  const bool day1_ok = read_day1 && found_day1 && last_day1 >= record_day1.record_id;
  const bool day2_ok = read_day2 && found_day2 && last_day2 >= record_day2.record_id;

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

int
RunDiagStorage(const app_runtime_t* runtime, bool full, diag_verbosity_t verbosity)
{
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "Storage", verbosity);
  const int total_steps = 4;
  int step_index = 1;

  if (runtime == NULL || runtime->sd_logger == NULL || runtime->fram_log == NULL) {
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
    (void)SdLoggerTryRemount(runtime->sd_logger, false);
  }

  char details[256];
  bool pass = RunPowerLossTailTest(runtime, full, details, sizeof(details));
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "powerloss tail",
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
