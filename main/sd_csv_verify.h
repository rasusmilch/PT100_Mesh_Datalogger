#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "esp_err.h"

struct SdCsvResumeInfo {
  bool file_was_truncated = false;
  bool found_last_sequence = false;
  uint32_t last_sequence = 0;  // 0 means "no samples present"
};

// Repairs a power-loss tail (truncates to the last '\n' if needed) and returns
// the last successfully written sequence number found in the file.
//
// Requirements:
// - Data lines must begin with an unsigned integer "seq" followed by a comma.
// - Header line should be "seq,..." and will be ignored.
// - Comment lines beginning with '#' are ignored.
esp_err_t SdCsvFindLastSequenceAndRepairTail(FILE* file_handle,
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

// Minimal FRAM interface expected by FramConsumeUpToSequence.
// Your FRAM ring buffer should provide "peek next seq" and "pop one record".
class FramLogInterface {
 public:
  virtual ~FramLogInterface() = default;

  // Returns true if a record exists and sets *sequence_out.
  virtual bool PeekNextSequence(uint32_t* sequence_out) = 0;

  // Pops exactly one record. Returns true on success.
  virtual bool PopOneRecord() = 0;
};

// Pops records from FRAM while seq <= last_sequence_on_sd.
// Use this after SdCsvFindLastSequenceAndRepairTail() at boot.
esp_err_t FramConsumeUpToSequence(FramLogInterface* fram_log,
                                 uint32_t last_sequence_on_sd,
                                 uint32_t* records_consumed_out);
