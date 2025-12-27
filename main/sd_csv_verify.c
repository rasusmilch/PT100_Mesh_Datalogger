#include "sd_csv_verify.h"

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "data_csv.h"
#include "esp_log.h"
#include "mbedtls/sha256.h"

static const char* kTag = "sd_csv_verify";

typedef struct
{
  uint8_t bytes[32];
} sha256_digest_t;

static sha256_digest_t
ComputeSha256(const uint8_t* data, size_t length_bytes)
{
  sha256_digest_t digest = { 0 };
  mbedtls_sha256_context sha_context;
  mbedtls_sha256_init(&sha_context);

  const int use_sha256 = 0; // 0 => SHA-256, 1 => SHA-224
  mbedtls_sha256_starts(&sha_context, use_sha256);
  mbedtls_sha256_update(&sha_context, data, length_bytes);
  mbedtls_sha256_finish(&sha_context, digest.bytes);

  mbedtls_sha256_free(&sha_context);
  return digest;
}

static bool
DigestsEqual(const sha256_digest_t* a, const sha256_digest_t* b)
{
  return memcmp(a->bytes, b->bytes, sizeof(a->bytes)) == 0;
}

static esp_err_t
GetFileSizeBytes(int file_descriptor, off_t* size_out)
{
  struct stat file_stat;
  if (fstat(file_descriptor, &file_stat) != 0) {
    ESP_LOGE(kTag, "fstat() failed: errno=%d (%s)", errno, strerror(errno));
    return ESP_FAIL;
  }
  *size_out = file_stat.st_size;
  return ESP_OK;
}

static esp_err_t
ReadExactly(int file_descriptor,
            off_t offset,
            uint8_t* buffer,
            size_t length_bytes)
{
  if (lseek(file_descriptor, offset, SEEK_SET) < 0) {
    ESP_LOGE(kTag, "lseek() failed: errno=%d (%s)", errno, strerror(errno));
    return ESP_FAIL;
  }

  size_t total_read = 0;
  while (total_read < length_bytes) {
    const ssize_t read_result =
      read(file_descriptor, buffer + total_read, length_bytes - total_read);
    if (read_result < 0) {
      ESP_LOGE(kTag, "read() failed: errno=%d (%s)", errno, strerror(errno));
      return ESP_FAIL;
    }
    if (read_result == 0) {
      ESP_LOGE(kTag, "read() EOF before expected length");
      return ESP_FAIL;
    }
    total_read += (size_t)read_result;
  }
  return ESP_OK;
}

static bool
ParseRecordIdFromCsvLine(const char* line, uint64_t* record_id_out)
{
  if (line == NULL || record_id_out == NULL) {
    return false;
  }
  if (line[0] == '\0' || line[0] == '#') {
    return false;
  }
  if (strncmp(line, "schema_ver,", 11) == 0) {
    return false;
  }

  const char* first_comma = strchr(line, ',');
  if (first_comma == NULL) {
    return false;
  }

  char* end_pointer = NULL;
  errno = 0;
  const unsigned long parsed_schema = strtoul(line, &end_pointer, 10);
  if (errno != 0 || end_pointer != first_comma || parsed_schema == 0) {
    return false;
  }
  if (parsed_schema < CSV_SCHEMA_VERSION) {
    return false;
  }

  const char* second_comma = strchr(first_comma + 1, ',');
  if (second_comma == NULL) {
    return false;
  }

  errno = 0;
  end_pointer = NULL;
  const unsigned long long parsed_record_id =
    strtoull(first_comma + 1, &end_pointer, 10);
  if (errno != 0 || end_pointer != second_comma) {
    return false;
  }

  *record_id_out = (uint64_t)parsed_record_id;
  return true;
}

// Finds the last '\n' in the file and truncates to that point+1 if needed.
// If there is no '\n' at all, truncates to 0.
static esp_err_t
RepairTailToLastNewline(int file_descriptor,
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

  const off_t scan_start = (file_size > (off_t)tail_scan_max_bytes)
                             ? (file_size - (off_t)tail_scan_max_bytes)
                             : 0;
  const size_t scan_length = (size_t)(file_size - scan_start);

  uint8_t* tail_bytes = (uint8_t*)malloc(scan_length);
  if (tail_bytes == NULL) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t read_result =
    ReadExactly(file_descriptor, scan_start, tail_bytes, scan_length);
  if (read_result != ESP_OK) {
    free(tail_bytes);
    return read_result;
  }

  ssize_t last_newline_index = -1;
  for (ssize_t index = (ssize_t)scan_length - 1; index >= 0; --index) {
    if (tail_bytes[(size_t)index] == '\n') {
      last_newline_index = index;
      break;
    }
  }

  off_t new_size = 0;
  if (last_newline_index >= 0) {
    new_size = scan_start + (off_t)last_newline_index + 1;
  } else {
    new_size = 0;
  }

  free(tail_bytes);

  if (ftruncate(file_descriptor, new_size) != 0) {
    ESP_LOGE(kTag, "ftruncate() failed: errno=%d (%s)", errno, strerror(errno));
    return ESP_FAIL;
  }

  if (fsync(file_descriptor) != 0) {
    ESP_LOGE(kTag,
             "fsync() after truncate failed: errno=%d (%s)",
             errno,
             strerror(errno));
    return ESP_FAIL;
  }

  *truncated_out = true;
  ESP_LOGW(kTag,
           "Repaired tail by truncating file from %lld to %lld",
           (long long)file_size,
           (long long)new_size);
  return ESP_OK;
}

static esp_err_t
FindLastRecordIdInFile(int file_descriptor,
                       off_t file_size,
                       size_t tail_scan_max_bytes,
                       bool* found_out,
                       uint64_t* last_record_id_out)
{
  *found_out = false;
  *last_record_id_out = 0;

  if (file_size <= 0) {
    return ESP_OK;
  }

  const off_t scan_start = (file_size > (off_t)tail_scan_max_bytes)
                             ? (file_size - (off_t)tail_scan_max_bytes)
                             : 0;
  const size_t scan_length = (size_t)(file_size - scan_start);

  uint8_t* tail_bytes = (uint8_t*)malloc(scan_length + 1);
  if (tail_bytes == NULL) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t read_result =
    ReadExactly(file_descriptor, scan_start, tail_bytes, scan_length);
  if (read_result != ESP_OK) {
    free(tail_bytes);
    return read_result;
  }
  tail_bytes[scan_length] = '\0';

  ssize_t line_end = (ssize_t)scan_length - 1;
  if (line_end >= 0 && tail_bytes[(size_t)line_end] == '\n') {
    --line_end;
  }

  while (line_end >= 0) {
    ssize_t line_start = line_end;
    while (line_start >= 0 && tail_bytes[(size_t)line_start] != '\n') {
      --line_start;
    }
    const size_t line_offset = (size_t)(line_start + 1);
    const size_t line_length = (size_t)(line_end - line_start);

    char* line_copy = (char*)malloc(line_length + 1);
    if (line_copy == NULL) {
      free(tail_bytes);
      return ESP_ERR_NO_MEM;
    }
    memcpy(line_copy, &tail_bytes[line_offset], line_length);
    line_copy[line_length] = '\0';

    uint64_t parsed_record_id = 0;
    const bool parsed_ok =
      ParseRecordIdFromCsvLine(line_copy, &parsed_record_id);
    free(line_copy);

    if (parsed_ok) {
      *found_out = true;
      *last_record_id_out = parsed_record_id;
      free(tail_bytes);
      return ESP_OK;
    }

    line_end = line_start - 1;
  }

  free(tail_bytes);
  return ESP_OK;
}

esp_err_t
SdCsvFindLastRecordIdAndRepairTail(FILE* file_handle,
                                   size_t tail_scan_max_bytes,
                                   SdCsvResumeInfo* resume_info_out)
{
  if (file_handle == NULL || resume_info_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const int file_descriptor = fileno(file_handle);
  if (file_descriptor < 0) {
    ESP_LOGE(kTag, "fileno() failed");
    return ESP_FAIL;
  }

  off_t file_size = 0;
  if (GetFileSizeBytes(file_descriptor, &file_size) != ESP_OK) {
    return ESP_FAIL;
  }

  bool file_was_truncated = false;
  if (RepairTailToLastNewline(
        file_descriptor, file_size, tail_scan_max_bytes, &file_was_truncated) !=
      ESP_OK) {
    return ESP_FAIL;
  }
  resume_info_out->file_was_truncated = file_was_truncated;

  if (GetFileSizeBytes(file_descriptor, &file_size) != ESP_OK) {
    return ESP_FAIL;
  }

  bool found_last_record_id = false;
  uint64_t last_record_id = 0;
  if (FindLastRecordIdInFile(file_descriptor,
                             file_size,
                             tail_scan_max_bytes,
                             &found_last_record_id,
                             &last_record_id) != ESP_OK) {
    return ESP_FAIL;
  }

  resume_info_out->found_last_record_id = found_last_record_id;
  resume_info_out->last_record_id = last_record_id;
  return ESP_OK;
}

esp_err_t
SdCsvAppendBatchWithReadbackVerify(FILE* file_handle,
                                   const uint8_t* batch_bytes,
                                   size_t batch_length_bytes)
{
  if (file_handle == NULL || batch_bytes == NULL || batch_length_bytes == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  const int file_descriptor = fileno(file_handle);
  if (file_descriptor < 0) {
    ESP_LOGE(kTag, "fileno() failed");
    return ESP_FAIL;
  }

  off_t original_size = 0;
  if (GetFileSizeBytes(file_descriptor, &original_size) != ESP_OK) {
    return ESP_FAIL;
  }
  const off_t append_offset = original_size;

  const sha256_digest_t digest_before =
    ComputeSha256(batch_bytes, batch_length_bytes);

  const size_t written =
    fwrite(batch_bytes, 1, batch_length_bytes, file_handle);
  if (written != batch_length_bytes) {
    ESP_LOGE(kTag,
             "fwrite() short write: wrote=%u expected=%u",
             (unsigned)written,
             (unsigned)batch_length_bytes);
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  if (fflush(file_handle) != 0) {
    ESP_LOGE(kTag, "fflush() failed: errno=%d (%s)", errno, strerror(errno));
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  if (fsync(file_descriptor) != 0) {
    ESP_LOGE(kTag, "fsync() failed: errno=%d (%s)", errno, strerror(errno));
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  uint8_t* readback_bytes = (uint8_t*)malloc(batch_length_bytes);
  if (readback_bytes == NULL) {
    return ESP_ERR_NO_MEM;
  }
  if (ReadExactly(
        file_descriptor, append_offset, readback_bytes, batch_length_bytes) !=
      ESP_OK) {
    ESP_LOGE(kTag, "Read-back failed; truncating to original size.");
    free(readback_bytes);
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  const sha256_digest_t digest_after =
    ComputeSha256(readback_bytes, batch_length_bytes);
  free(readback_bytes);

  if (!DigestsEqual(&digest_before, &digest_after)) {
    ESP_LOGE(kTag, "SD verify failed (SHA mismatch). Truncating append.");
    if (ftruncate(file_descriptor, original_size) != 0) {
      ESP_LOGE(kTag,
               "ftruncate() rollback failed: errno=%d (%s)",
               errno,
               strerror(errno));
    }
    fsync(file_descriptor);
    return ESP_ERR_INVALID_CRC;
  }

  return ESP_OK;
}
