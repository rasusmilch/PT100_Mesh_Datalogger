#include "sd_csv_verify.h"

#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "esp_log.h"
#include "mbedtls/sha256.h"

namespace
{

  static const char* const kTag = "sd_csv_verify";

  struct Sha256Digest
  {
    uint8_t bytes[32];
  };

  Sha256Digest ComputeSha256(const uint8_t* data, size_t length_bytes)
  {
    Sha256Digest digest{};
    mbedtls_sha256_context sha_context;
    mbedtls_sha256_init(&sha_context);

    const int use_sha256 = 0; // 0 => SHA-256, 1 => SHA-224
    mbedtls_sha256_starts_ret(&sha_context, use_sha256);
    mbedtls_sha256_update_ret(&sha_context, data, length_bytes);
    mbedtls_sha256_finish_ret(&sha_context, digest.bytes);

    mbedtls_sha256_free(&sha_context);
    return digest;
  }

  bool DigestsEqual(const Sha256Digest& a, const Sha256Digest& b)
  {
    return std::memcmp(a.bytes, b.bytes, sizeof(a.bytes)) == 0;
  }

  esp_err_t GetFileSizeBytes(int file_descriptor, off_t* size_out)
  {
    struct stat file_stat{};
    if (fstat(file_descriptor, &file_stat) != 0) {
      ESP_LOGE(
        kTag, "fstat() failed: errno=%d (%s)", errno, std::strerror(errno));
      return ESP_FAIL;
    }
    *size_out = file_stat.st_size;
    return ESP_OK;
  }

  esp_err_t ReadExactly(
    int file_descriptor, off_t offset, uint8_t* buffer, size_t length_bytes)
  {
    if (lseek(file_descriptor, offset, SEEK_SET) < 0) {
      ESP_LOGE(
        kTag, "lseek() failed: errno=%d (%s)", errno, std::strerror(errno));
      return ESP_FAIL;
    }

    size_t total_read = 0;
    while (total_read < length_bytes) {
      const ssize_t read_result =
        read(file_descriptor, buffer + total_read, length_bytes - total_read);
      if (read_result < 0) {
        ESP_LOGE(
          kTag, "read() failed: errno=%d (%s)", errno, std::strerror(errno));
        return ESP_FAIL;
      }
      if (read_result == 0) {
        ESP_LOGE(kTag, "read() EOF before expected length");
        return ESP_FAIL;
      }
      total_read += static_cast<size_t>(read_result);
    }
    return ESP_OK;
  }

  bool ParseSequenceFromCsvLine(const std::string& line, uint32_t* sequence_out)
  {
    if (line.empty()) {
      return false;
    }
    if (line[0] == '#') {
      return false;
    }
    // Skip header (e.g., "seq,iso8601,...")
    if (line.rfind("seq,", 0) == 0) {
      return false;
    }

    // Parse leading uint32 up to the first comma.
    size_t comma_position = line.find(',');
    if (comma_position == std::string::npos) {
      return false;
    }

    const std::string first_field = line.substr(0, comma_position);
    char* end_pointer = nullptr;
    errno = 0;
    const unsigned long parsed =
      std::strtoul(first_field.c_str(), &end_pointer, 10);
    if (errno != 0 || end_pointer == first_field.c_str() ||
        *end_pointer != '\0') {
      return false;
    }
    if (parsed > UINT32_MAX) {
      return false;
    }

    *sequence_out = static_cast<uint32_t>(parsed);
    return true;
  }

  // Finds the last '\n' in the file and truncates to that point+1 if needed.
  // If there is no '\n' at all, truncates to 0.
  esp_err_t RepairTailToLastNewline(int file_descriptor,
                                    off_t file_size,
                                    size_t tail_scan_max_bytes,
                                    bool* truncated_out)
  {
    *truncated_out = false;

    if (file_size <= 0) {
      return ESP_OK;
    }

    uint8_t last_byte = 0;
    if (ReadExactly(file_descriptor, file_size - 1, &last_byte, 1) != ESP_OK) {
      return ESP_FAIL;
    }

    if (last_byte == '\n') {
      return ESP_OK; // already line-complete
    }

    const off_t scan_start =
      (file_size > static_cast<off_t>(tail_scan_max_bytes))
        ? (file_size - static_cast<off_t>(tail_scan_max_bytes))
        : 0;
    const size_t scan_length = static_cast<size_t>(file_size - scan_start);

    std::vector<uint8_t> tail_bytes(scan_length);
    if (ReadExactly(
          file_descriptor, scan_start, tail_bytes.data(), scan_length) !=
        ESP_OK) {
      return ESP_FAIL;
    }

    // Find last '\n' in tail buffer.
    ssize_t last_newline_index = -1;
    for (ssize_t index = static_cast<ssize_t>(scan_length) - 1; index >= 0;
         --index) {
      if (tail_bytes[static_cast<size_t>(index)] == '\n') {
        last_newline_index = index;
        break;
      }
    }

    off_t new_size = 0;
    if (last_newline_index >= 0) {
      new_size = scan_start + static_cast<off_t>(last_newline_index) + 1;
    } else {
      new_size = 0;
    }

    if (ftruncate(file_descriptor, new_size) != 0) {
      ESP_LOGE(
        kTag, "ftruncate() failed: errno=%d (%s)", errno, std::strerror(errno));
      return ESP_FAIL;
    }

    // Ensure truncation is committed.
    if (fsync(file_descriptor) != 0) {
      ESP_LOGE(kTag,
               "fsync() after truncate failed: errno=%d (%s)",
               errno,
               std::strerror(errno));
      return ESP_FAIL;
    }

    *truncated_out = true;
    ESP_LOGW(kTag,
             "Repaired tail by truncating file from %lld to %lld",
             static_cast<long long>(file_size),
             static_cast<long long>(new_size));
    return ESP_OK;
  }

  esp_err_t FindLastSequenceInFile(int file_descriptor,
                                   off_t file_size,
                                   size_t tail_scan_max_bytes,
                                   bool* found_out,
                                   uint32_t* last_sequence_out)
  {
    *found_out = false;
    *last_sequence_out = 0;

    if (file_size <= 0) {
      return ESP_OK;
    }

    const off_t scan_start =
      (file_size > static_cast<off_t>(tail_scan_max_bytes))
        ? (file_size - static_cast<off_t>(tail_scan_max_bytes))
        : 0;
    const size_t scan_length = static_cast<size_t>(file_size - scan_start);

    std::vector<uint8_t> tail_bytes(scan_length);
    if (ReadExactly(
          file_descriptor, scan_start, tail_bytes.data(), scan_length) !=
        ESP_OK) {
      return ESP_FAIL;
    }

    // We assume file ends with '\n' at this point.
    // Extract lines and find the last data line that parses.
    // Work backwards: find line boundaries by '\n'.
    ssize_t line_end = static_cast<ssize_t>(scan_length) - 1;

    // Skip trailing '\n' if present.
    if (line_end >= 0 && tail_bytes[static_cast<size_t>(line_end)] == '\n') {
      --line_end;
    }

    while (line_end >= 0) {
      ssize_t line_start = line_end;
      while (line_start >= 0 &&
             tail_bytes[static_cast<size_t>(line_start)] != '\n') {
        --line_start;
      }
      const size_t line_offset = static_cast<size_t>(line_start + 1);
      const size_t line_length = static_cast<size_t>(line_end - line_start);

      std::string line(
        reinterpret_cast<const char*>(tail_bytes.data() + line_offset),
        line_length);

      uint32_t parsed_sequence = 0;
      if (ParseSequenceFromCsvLine(line, &parsed_sequence)) {
        *found_out = true;
        *last_sequence_out = parsed_sequence;
        return ESP_OK;
      }

      // Move to previous line (skip the '\n').
      line_end = line_start - 1;
    }

    return ESP_OK;
  }

} // namespace

esp_err_t
SdCsvFindLastSequenceAndRepairTail(FILE* file_handle,
                                   size_t tail_scan_max_bytes,
                                   SdCsvResumeInfo* resume_info_out)
{
  if (file_handle == nullptr || resume_info_out == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  const int file_descriptor = fileno(file_handle);
  if (file_descriptor < 0) {
    ESP_LOGE(kTag, "fileno() failed");
    return ESP_FAIL;
  }

  // Get current size.
  off_t file_size = 0;
  if (GetFileSizeBytes(file_descriptor, &file_size) != ESP_OK) {
    return ESP_FAIL;
  }

  // Repair partial last line if needed.
  bool file_was_truncated = false;
  if (RepairTailToLastNewline(
        file_descriptor, file_size, tail_scan_max_bytes, &file_was_truncated) !=
      ESP_OK) {
    return ESP_FAIL;
  }
  resume_info_out->file_was_truncated = file_was_truncated;

  // Re-check size after potential truncation.
  if (GetFileSizeBytes(file_descriptor, &file_size) != ESP_OK) {
    return ESP_FAIL;
  }

  // Find last sequence.
  bool found_last_sequence = false;
  uint32_t last_sequence = 0;
  if (FindLastSequenceInFile(file_descriptor,
                             file_size,
                             tail_scan_max_bytes,
                             &found_last_sequence,
                             &last_sequence) != ESP_OK) {
    return ESP_FAIL;
  }

  resume_info_out->found_last_sequence = found_last_sequence;
  resume_info_out->last_sequence = last_sequence;
  return ESP_OK;
}

esp_err_t
SdCsvAppendBatchWithReadbackVerify(FILE* file_handle,
                                   const uint8_t* batch_bytes,
                                   size_t batch_length_bytes)
{
  if (file_handle == nullptr || batch_bytes == nullptr ||
      batch_length_bytes == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  const int file_descriptor = fileno(file_handle);
  if (file_descriptor < 0) {
    ESP_LOGE(kTag, "fileno() failed");
    return ESP_FAIL;
  }

  // Determine append offset (current file size).
  off_t original_size = 0;
  if (GetFileSizeBytes(file_descriptor, &original_size) != ESP_OK) {
    return ESP_FAIL;
  }
  const off_t append_offset = original_size;

  const Sha256Digest digest_before =
    ComputeSha256(batch_bytes, batch_length_bytes);

  // Append in a few big writes. (Caller should already provide a big buffer.)
  const size_t written =
    fwrite(batch_bytes, 1, batch_length_bytes, file_handle);
  if (written != batch_length_bytes) {
    ESP_LOGE(kTag,
             "fwrite() short write: wrote=%u expected=%u",
             static_cast<unsigned>(written),
             static_cast<unsigned>(batch_length_bytes));
    // Best effort: truncate back.
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  if (fflush(file_handle) != 0) {
    ESP_LOGE(
      kTag, "fflush() failed: errno=%d (%s)", errno, std::strerror(errno));
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  if (fsync(file_descriptor) != 0) {
    ESP_LOGE(
      kTag, "fsync() failed: errno=%d (%s)", errno, std::strerror(errno));
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  // Read back the appended region and verify hash.
  std::vector<uint8_t> readback_bytes(batch_length_bytes);
  if (ReadExactly(file_descriptor,
                  append_offset,
                  readback_bytes.data(),
                  batch_length_bytes) != ESP_OK) {
    ESP_LOGE(kTag, "Read-back failed; truncating to original size.");
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  const Sha256Digest digest_after =
    ComputeSha256(readback_bytes.data(), readback_bytes.size());

  if (!DigestsEqual(digest_before, digest_after)) {
    ESP_LOGE(kTag, "SD verify failed (SHA mismatch). Truncating append.");
    if (ftruncate(file_descriptor, original_size) != 0) {
      ESP_LOGE(kTag,
               "ftruncate() rollback failed: errno=%d (%s)",
               errno,
               std::strerror(errno));
    }
    fsync(file_descriptor);
    return ESP_ERR_INVALID_CRC;
  }

  return ESP_OK;
}

esp_err_t
FramConsumeUpToSequence(FramLogInterface* fram_log,
                        uint32_t last_sequence_on_sd,
                        uint32_t* records_consumed_out)
{
  if (fram_log == nullptr || records_consumed_out == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t consumed_count = 0;

  while (true) {
    uint32_t next_sequence = 0;
    if (!fram_log->PeekNextSequence(&next_sequence)) {
      break; // no records
    }
    if (next_sequence > last_sequence_on_sd) {
      break;
    }
    if (!fram_log->PopOneRecord()) {
      ESP_LOGE(kTag,
               "FRAM PopOneRecord() failed while consuming up to seq=%u",
               static_cast<unsigned>(last_sequence_on_sd));
      return ESP_FAIL;
    }
    ++consumed_count;
  }

  *records_consumed_out = consumed_count;
  return ESP_OK;
}
