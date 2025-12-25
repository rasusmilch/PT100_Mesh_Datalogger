#include "data_csv.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char* kCsvHeader =
  "schema_ver,seq,epoch_utc,iso8601_local,raw_rtd_ohms,raw_temp_c,cal_temp_c,flags,node_id\n";

static void
FormatIso8601Offset(const struct tm* time_info, char* out, size_t out_size)
{
  if (out_size == 0) {
    return;
  }

  char raw_offset[8] = "";
  strftime(raw_offset, sizeof(raw_offset), "%z", time_info);
  if (strlen(raw_offset) == 5) {
    snprintf(out,
             out_size,
             "%c%c%c:%c%c",
             raw_offset[0],
             raw_offset[1],
             raw_offset[2],
             raw_offset[3],
             raw_offset[4]);
  } else {
    snprintf(out, out_size, "+00:00");
  }
}

static void
BuildIso8601LocalWithMillis(int64_t epoch_seconds,
                            int32_t millis,
                            char* out,
                            size_t out_size)
{
  if (epoch_seconds <= 0) {
    if (out_size > 0) {
      out[0] = '\0';
    }
    return;
  }

  time_t time_seconds = (time_t)epoch_seconds;
  struct tm time_info;
  localtime_r(&time_seconds, &time_info);

  if (millis < 0) {
    millis = 0;
  }
  if (millis > 999) {
    millis = 999;
  }

  char offset[8] = "";
  FormatIso8601Offset(&time_info, offset, sizeof(offset));

  snprintf(out,
           out_size,
           "%04d-%02d-%02dT%02d:%02d:%02d.%03d%s",
           time_info.tm_year + 1900,
           time_info.tm_mon + 1,
           time_info.tm_mday,
           time_info.tm_hour,
           time_info.tm_min,
           time_info.tm_sec,
           (int)millis,
           offset);
}

bool
CsvFormatHeader(char* out, size_t out_size, size_t* written_out)
{
  if (out == NULL || out_size == 0) {
    return false;
  }
  const size_t header_len = strlen(kCsvHeader);
  if (header_len >= out_size) {
    return false;
  }
  memcpy(out, kCsvHeader, header_len + 1);
  if (written_out != NULL) {
    *written_out = header_len;
  }
  return true;
}

bool
CsvFormatRow(const log_record_t* record,
             const char* node_id,
             char* out,
             size_t out_size,
             size_t* written_out)
{
  if (record == NULL || out == NULL || out_size == 0) {
    return false;
  }
  const char* node = (node_id != NULL) ? node_id : "";

  char iso8601[40];
  BuildIso8601LocalWithMillis(record->timestamp_epoch_sec,
                              record->timestamp_millis,
                              iso8601,
                              sizeof(iso8601));

  const double raw_c = record->raw_temp_milli_c / 1000.0;
  const double temp_c = record->temp_milli_c / 1000.0;
  const double resistance_ohm = record->resistance_milli_ohm / 1000.0;

  const int length = snprintf(out,
                              out_size,
                              "%u,%u,%" PRId64 ",%s,%.3f,%.3f,%.3f,0x%04x,%s\n",
                              CSV_SCHEMA_VERSION,
                              (unsigned)record->sequence,
                              record->timestamp_epoch_sec,
                              iso8601,
                              resistance_ohm,
                              raw_c,
                              temp_c,
                              (unsigned)record->flags,
                              node);
  if (length < 0 || (size_t)length >= out_size) {
    return false;
  }
  if (written_out != NULL) {
    *written_out = (size_t)length;
  }
  return true;
}

bool
CsvWriteHeader(csv_write_fn_t writer, void* context)
{
  if (writer == NULL) {
    return false;
  }
  return writer(kCsvHeader, strlen(kCsvHeader), context);
}

bool
CsvWriteRow(csv_write_fn_t writer,
            void* context,
            const log_record_t* record,
            const char* node_id)
{
  if (writer == NULL || record == NULL) {
    return false;
  }
  char line[208];
  size_t line_len = 0;
  if (!CsvFormatRow(record, node_id, line, sizeof(line), &line_len)) {
    return false;
  }
  return writer(line, line_len, context);
}
