#include "data_csv.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "sdkconfig.h"

static const char* kCsvHeader =
  "schema_ver,record_id,seq,epoch_utc,iso8601_local,raw_rtd_ohms,raw_temp_c,"
  "cal_temp_c,flags,fault_status,node_id\n";

static char g_csv_line_buffer[CONFIG_APP_MAX_CSV_LINE_BYTES];

static void
FormatSignedMilliFixed3(int64_t milli_value, char* out, size_t out_size);

/**
 * @brief Format a signed milli-unit value as fixed-point with 3 decimals or as nan on sensor faults.
 *
 * @param record Record to format from.
 * @param milli_value Signed milli-unit value to format when not faulted.
 * @param out Output buffer.
 * @param out_size Output buffer size in bytes.
 */
static void
FormatSignedMilliOrNan(const log_record_t* record,
                       int64_t milli_value,
                       char* out,
                       size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  if (record != NULL &&
      (record->flags & LOG_RECORD_FLAG_SENSOR_FAULT) != 0) {
    (void)snprintf(out, out_size, "nan");
    return;
  }
  FormatSignedMilliFixed3(milli_value, out, out_size);
}

/**
 * @brief Format a signed milli-unit value as fixed-point with 3 decimals.
 *
 * Example: -1 -> "-0.001", 12345 -> "12.345"
 */
static void
FormatSignedMilliFixed3(int64_t milli_value, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }

  const bool negative = (milli_value < 0);
  // Avoid overflow for INT64_MIN (not expected in practice, but safe).
  const uint64_t abs_milli =
    negative ? (uint64_t)(-(milli_value + 1)) + 1 : (uint64_t)milli_value;

  const uint64_t whole = abs_milli / 1000ULL;
  const uint64_t frac = abs_milli % 1000ULL;

  if (negative) {
    (void)snprintf(out, out_size, "-%" PRIu64 ".%03" PRIu64, whole, frac);
  } else {
    (void)snprintf(out, out_size, "%" PRIu64 ".%03" PRIu64, whole, frac);
  }
}

/**
 * @brief Execute FormatIso8601Offset.
 * @param time_info Parameter time_info.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
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

/**
 * @brief Execute BuildIso8601LocalWithMillis.
 * @param epoch_seconds Parameter epoch_seconds.
 * @param millis Parameter millis.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
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

/**
 * @brief Execute CsvFormatHeader.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 * @param written_out Parameter written_out.
 * @return Return the function result.
 */
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

/**
 * @brief Execute CsvFormatRow.
 * @param record Parameter record.
 * @param node_id Parameter node_id.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 * @param written_out Parameter written_out.
 * @return Return the function result.
 */
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

  // Avoid floating-point formatting in snprintf; it is stack-heavy and can
  // overflow small task stacks. Format fixed-point using integer math instead.
  char raw_c_str[24] = { 0 };
  char temp_c_str[24] = { 0 };
  char resistance_ohm_str[24] = { 0 };
  FormatSignedMilliOrNan(
    record, (int64_t)record->raw_temp_milli_c, raw_c_str, sizeof(raw_c_str));
  FormatSignedMilliOrNan(
    record, (int64_t)record->temp_milli_c, temp_c_str, sizeof(temp_c_str));
  FormatSignedMilliOrNan(record,
                         (int64_t)record->resistance_milli_ohm,
                         resistance_ohm_str,
                         sizeof(resistance_ohm_str));

  const uint8_t fault_status =
    ((record->flags & LOG_RECORD_FLAG_SENSOR_FAULT) != 0) ? record->fault_status
                                                           : 0u;

  const int length =
    snprintf(out,
             out_size,
             "%u,%" PRIu64 ",%u,%" PRId64 ",%s,%s,%s,%s,0x%04x,0x%02x,%s\n",
             CSV_SCHEMA_VERSION,
             record->record_id,
             (unsigned)record->sequence,
             record->timestamp_epoch_sec,
             iso8601,
             resistance_ohm_str,
             raw_c_str,
             temp_c_str,
             (unsigned)record->flags,
             (unsigned)fault_status,
             node);
  if (length < 0 || (size_t)length >= out_size) {
    return false;
  }
  if (written_out != NULL) {
    *written_out = (size_t)length;
  }
  return true;
}

/**
 * @brief Execute CsvWriteHeader.
 * @param writer Parameter writer.
 * @param context Parameter context.
 * @return Return the function result.
 */
bool
CsvWriteHeader(csv_write_fn_t writer, void* context)
{
  if (writer == NULL) {
    return false;
  }
  return writer(kCsvHeader, strlen(kCsvHeader), context);
}

/**
 * @brief Execute CsvWriteRow.
 * @param writer Parameter writer.
 * @param context Parameter context.
 * @param record Parameter record.
 * @param node_id Parameter node_id.
 * @return Return the function result.
 */
bool
CsvWriteRow(csv_write_fn_t writer,
            void* context,
            const log_record_t* record,
            const char* node_id)
{
  if (writer == NULL || record == NULL) {
    return false;
  }
  size_t line_len = 0;
  if (!CsvFormatRow(record,
                    node_id,
                    g_csv_line_buffer,
                    sizeof(g_csv_line_buffer),
                    &line_len)) {
    return false;
  }
  return writer(g_csv_line_buffer, line_len, context);
}
