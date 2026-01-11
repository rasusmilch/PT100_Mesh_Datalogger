#ifndef PT100_LOGGER_SD_CSV_VERIFY_H_
#define PT100_LOGGER_SD_CSV_VERIFY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  bool file_was_truncated;
  bool found_last_record_id;
  uint64_t last_record_id;
} SdCsvResumeInfo;

typedef struct
{
  const char* operation;
  int errno_value;
} SdCsvAppendDiagnostics;

typedef struct
{
  uint8_t* io_bounce_bytes;
  size_t io_bounce_capacity;
  uint8_t* verify_readback_bytes;
  size_t verify_readback_capacity;
} sd_csv_append_scratch_t;

typedef struct
{
  uint8_t* tail_bytes;
  size_t tail_capacity;
} sd_csv_resume_scratch_t;

// Repairs a power-loss tail (truncates to the last '\n' if needed) and returns
// the last successfully written record_id found in the file.
//
// Requirements:
// - Data lines must begin with an unsigned integer schema_ver followed by a
//   comma, and include record_id as the second field.
// - Header line should be "schema_ver,record_id,..." and will be ignored.
// - Comment lines beginning with '#' are ignored.
/**
 * @brief Execute SdCsvFindLastRecordIdAndRepairTail.
 * @param file_handle Parameter file_handle.
 * @param tail_scan_max_bytes Parameter tail_scan_max_bytes.
 * @param scratch Parameter scratch.
 * @param resume_info_out Parameter resume_info_out.
 * @return Return the function result.
 */
esp_err_t SdCsvFindLastRecordIdAndRepairTail(FILE* file_handle,
                                             size_t tail_scan_max_bytes,
                                             const sd_csv_resume_scratch_t* scratch,
                                             SdCsvResumeInfo* resume_info_out);

// Appends a large buffer to the CSV file, then verifies by reading back the
// appended region and comparing SHA-256 hashes.
//
// On verification failure, the function truncates the file back to its original
// size and returns an error. FRAM must NOT be consumed unless this returns OK.
/**
 * @brief Execute SdCsvAppendBatchWithReadbackVerify.
 * @param file_handle Parameter file_handle.
 * @param batch_bytes Parameter batch_bytes.
 * @param batch_length_bytes Parameter batch_length_bytes.
 * @param diag_out Parameter diag_out.
 * @return Return the function result.
 */
esp_err_t SdCsvAppendBatchWithReadbackVerify(FILE* file_handle,
                                             const uint8_t* batch_bytes,
                                             size_t batch_length_bytes,
                                             SdCsvAppendDiagnostics* diag_out);

// Same as SdCsvAppendBatchWithReadbackVerify, but allows caller-provided scratch
// buffers and configurable flush behavior. When flush_per_append is false, the
// write is still fflush()'d for readback verification, but fsync() is skipped.
esp_err_t SdCsvAppendBatchWithReadbackVerifyEx(
  FILE* file_handle,
  const uint8_t* batch_bytes,
  size_t batch_length_bytes,
  SdCsvAppendDiagnostics* diag_out,
  const sd_csv_append_scratch_t* scratch,
  bool flush_per_append);

// Flush and fsync a CSV file handle, reporting diagnostics.
esp_err_t SdCsvFlushAndSync(FILE* file_handle,
                            SdCsvAppendDiagnostics* diag_out);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_CSV_VERIFY_H_
