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

// Repairs a power-loss tail (truncates to the last '\n' if needed) and returns
// the last successfully written record_id found in the file.
//
// Requirements:
// - Data lines must begin with an unsigned integer schema_ver followed by a
//   comma, and include record_id as the second field.
// - Header line should be "schema_ver,record_id,..." and will be ignored.
// - Comment lines beginning with '#' are ignored.
esp_err_t SdCsvFindLastRecordIdAndRepairTail(FILE* file_handle,
                                             size_t tail_scan_max_bytes,
                                             SdCsvResumeInfo* resume_info_out);

// Appends a large buffer to the CSV file, then verifies by reading back the
// appended region and comparing SHA-256 hashes.
//
// On verification failure, the function truncates the file back to its original
// size and returns an error. FRAM must NOT be consumed unless this returns OK.
esp_err_t SdCsvAppendBatchWithReadbackVerify(FILE* file_handle,
                                             const uint8_t* batch_bytes,
                                             size_t batch_length_bytes);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_CSV_VERIFY_H_
