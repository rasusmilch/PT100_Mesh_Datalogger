#include "sd_csv_verify.h"

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "data_csv.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mem_guard.h"
#include "mbedtls/sha256.h"

static const char* kTag = "sd_csv_verify";
static const uint32_t kVerifyYieldMaxMs = 30;

typedef struct
{
  uint8_t bytes[32];
} sha256_digest_t;

static esp_err_t
ReadExactly(int file_descriptor,
            off_t offset,
            uint8_t* buffer,
            size_t length_bytes);

/**
 * @brief Execute SetAppendDiagnostics.
 * @param diag_out Parameter diag_out.
 * @param operation Parameter operation.
 * @param errno_value Parameter errno_value.
 */
static void
SetAppendDiagnostics(SdCsvAppendDiagnostics* diag_out,
                     const char* operation,
                     int errno_value)
{
  if (diag_out == NULL) {
    return;
  }
  diag_out->operation = operation;
  diag_out->errno_value = errno_value;
}

/**
 * @brief Execute ComputeSha256.
 * @param data Parameter data.
 * @param length_bytes Parameter length_bytes.
 * @return Return the function result.
 */
static sha256_digest_t
ComputeSha256(const uint8_t* data, size_t length_bytes)
{
  sha256_digest_t digest = { 0 };
  mbedtls_sha256_context sha_context;
  mbedtls_sha256_init(&sha_context);

  const int use_sha256 = 0; // 0 => SHA-256, 1 => SHA-224
  mbedtls_sha256_starts(&sha_context, use_sha256);
  const size_t chunk_size = 1024;
  TickType_t slice_start = xTaskGetTickCount();
  size_t offset = 0;
  while (offset < length_bytes) {
    size_t chunk = length_bytes - offset;
    if (chunk > chunk_size) {
      chunk = chunk_size;
    }
    mbedtls_sha256_update(&sha_context, data + offset, chunk);
    offset += chunk;

    if (pdTICKS_TO_MS(xTaskGetTickCount() - slice_start) >=
        kVerifyYieldMaxMs) {
      vTaskDelay(1);
      slice_start = xTaskGetTickCount();
    }
  }
  mbedtls_sha256_finish(&sha_context, digest.bytes);

  mbedtls_sha256_free(&sha_context);
  return digest;
}

/**
 * @brief Execute ComputeSha256FromFileRegion.
 * @param file_descriptor Parameter file_descriptor.
 * @param offset Parameter offset.
 * @param length_bytes Parameter length_bytes.
 * @param buffer Parameter buffer.
 * @param buffer_capacity Parameter buffer_capacity.
 * @param digest_out Parameter digest_out.
 * @return Return the function result.
 */
static esp_err_t
ComputeSha256FromFileRegion(int file_descriptor,
                            off_t offset,
                            size_t length_bytes,
                            uint8_t* buffer,
                            size_t buffer_capacity,
                            sha256_digest_t* digest_out)
{
  if (buffer == NULL || buffer_capacity == 0 || digest_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  mbedtls_sha256_context sha_context;
  mbedtls_sha256_init(&sha_context);
  const int use_sha256 = 0; // 0 => SHA-256, 1 => SHA-224
  mbedtls_sha256_starts(&sha_context, use_sha256);

  size_t remaining = length_bytes;
  off_t read_offset = offset;
  while (remaining > 0) {
    size_t chunk = remaining;
    if (chunk > buffer_capacity) {
      chunk = buffer_capacity;
    }
    if (ReadExactly(file_descriptor, read_offset, buffer, chunk) != ESP_OK) {
      mbedtls_sha256_free(&sha_context);
      return ESP_FAIL;
    }
    mbedtls_sha256_update(&sha_context, buffer, chunk);
    read_offset += (off_t)chunk;
    remaining -= chunk;
  }

  mbedtls_sha256_finish(&sha_context, digest_out->bytes);
  mbedtls_sha256_free(&sha_context);
  return ESP_OK;
}

/**
 * @brief Execute DigestsEqual.
 * @param a Parameter a.
 * @param b Parameter b.
 * @return Return the function result.
 */
static bool
DigestsEqual(const sha256_digest_t* a, const sha256_digest_t* b)
{
  return memcmp(a->bytes, b->bytes, sizeof(a->bytes)) == 0;
}

/**
 * @brief Execute GetFileSizeBytes.
 * @param file_descriptor Parameter file_descriptor.
 * @param size_out Parameter size_out.
 * @return Return the function result.
 */
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

/**
 * @brief Execute ReadExactly.
 * @param file_descriptor Parameter file_descriptor.
 * @param offset Parameter offset.
 * @param buffer Parameter buffer.
 * @param length_bytes Parameter length_bytes.
 * @return Return the function result.
 */
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
  TickType_t slice_start = xTaskGetTickCount();
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
    if (pdTICKS_TO_MS(xTaskGetTickCount() - slice_start) >=
        kVerifyYieldMaxMs) {
      vTaskDelay(1);
      slice_start = xTaskGetTickCount();
    }
  }
  return ESP_OK;
}

/**
 * @brief Execute ParseRecordIdFromCsvLine.
 * @param line Parameter line.
 * @param record_id_out Parameter record_id_out.
 * @return Return the function result.
 */
static bool
ParseRecordIdFromCsvLine(const char* line, uint64_t* record_id_out)
{
  static bool logged_legacy_schema = false;

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
    if (!logged_legacy_schema) {
      ESP_LOGW(kTag,
               "Unsupported CSV schema_ver=%lu (expected >=%u). "
               "record_id resume disabled for legacy files.",
               parsed_schema,
               CSV_SCHEMA_VERSION);
      logged_legacy_schema = true;
    }
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

static uint8_t*
AcquireTailBuffer(size_t required_bytes,
                  const sd_csv_resume_scratch_t* scratch,
                  bool* needs_free_out)
{
  if (needs_free_out == NULL) {
    return NULL;
  }
  *needs_free_out = false;
  if (scratch != NULL) {
    if (scratch->tail_bytes != NULL &&
        scratch->tail_capacity >= required_bytes) {
      return scratch->tail_bytes;
    }
    return NULL;
  }

  uint8_t* tail_bytes = (uint8_t*)heap_caps_malloc(
    required_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (tail_bytes == NULL) {
    tail_bytes = (uint8_t*)heap_caps_malloc(
      required_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (tail_bytes == NULL) {
    return NULL;
  }
  *needs_free_out = true;
  return tail_bytes;
}

// Finds the last '\n' in the file and truncates to that point+1 if needed.
// If there is no '\n' at all, truncates to 0.
/**
 * @brief Execute RepairTailToLastNewline.
 * @param file_descriptor Parameter file_descriptor.
 * @param file_size Parameter file_size.
 * @param tail_scan_max_bytes Parameter tail_scan_max_bytes.
 * @param truncated_out Parameter truncated_out.
 * @return Return the function result.
 */
static esp_err_t
RepairTailToLastNewline(int file_descriptor,
                        off_t file_size,
                        size_t tail_scan_max_bytes,
                        const sd_csv_resume_scratch_t* scratch,
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

  bool needs_free = false;
  uint8_t* tail_bytes =
    AcquireTailBuffer(scan_length, scratch, &needs_free);
  if (tail_bytes == NULL) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t read_result =
    ReadExactly(file_descriptor, scan_start, tail_bytes, scan_length);
  if (read_result != ESP_OK) {
    if (needs_free) {
      AppFree(tail_bytes);
    }
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

  if (needs_free) {
    AppFree(tail_bytes);
  }

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

/**
 * @brief Execute FindLastRecordIdInFile.
 * @param file_descriptor Parameter file_descriptor.
 * @param file_size Parameter file_size.
 * @param tail_scan_max_bytes Parameter tail_scan_max_bytes.
 * @param found_out Parameter found_out.
 * @param last_record_id_out Parameter last_record_id_out.
 * @return Return the function result.
 */
static esp_err_t
FindLastRecordIdInFile(int file_descriptor,
                       off_t file_size,
                       size_t tail_scan_max_bytes,
                       const sd_csv_resume_scratch_t* scratch,
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

  const size_t buffer_needed = scan_length + 1;
  bool needs_free = false;
  uint8_t* tail_bytes =
    AcquireTailBuffer(buffer_needed, scratch, &needs_free);
  if (tail_bytes == NULL) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t read_result =
    ReadExactly(file_descriptor, scan_start, tail_bytes, scan_length);
  if (read_result != ESP_OK) {
    if (needs_free) {
      AppFree(tail_bytes);
    }
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

    uint64_t parsed_record_id = 0;
    tail_bytes[line_offset + line_length] = '\0';
    const bool parsed_ok = ParseRecordIdFromCsvLine(
      (const char*)&tail_bytes[line_offset], &parsed_record_id);

    if (parsed_ok) {
      *found_out = true;
      *last_record_id_out = parsed_record_id;
      if (needs_free) {
        AppFree(tail_bytes);
      }
      return ESP_OK;
    }

    line_end = line_start - 1;
  }

  if (needs_free) {
    AppFree(tail_bytes);
  }
  return ESP_OK;
}

/**
 * @brief Execute SdCsvFindLastRecordIdAndRepairTail.
 * @param file_handle Parameter file_handle.
 * @param tail_scan_max_bytes Parameter tail_scan_max_bytes.
 * @param resume_info_out Parameter resume_info_out.
 * @return Return the function result.
 */
esp_err_t
SdCsvFindLastRecordIdAndRepairTail(FILE* file_handle,
                                   size_t tail_scan_max_bytes,
                                   const sd_csv_resume_scratch_t* scratch,
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
        file_descriptor,
        file_size,
        tail_scan_max_bytes,
        scratch,
        &file_was_truncated) != ESP_OK) {
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
                             scratch,
                             &found_last_record_id,
                             &last_record_id) != ESP_OK) {
    return ESP_FAIL;
  }

  resume_info_out->found_last_record_id = found_last_record_id;
  resume_info_out->last_record_id = last_record_id;
  return ESP_OK;
}

/**
 * @brief Execute SdCsvAppendBatchWithReadbackVerify.
 * @param file_handle Parameter file_handle.
 * @param batch_bytes Parameter batch_bytes.
 * @param batch_length_bytes Parameter batch_length_bytes.
 * @param diag_out Parameter diag_out.
 * @return Return the function result.
 */
esp_err_t
SdCsvAppendBatchWithReadbackVerify(FILE* file_handle,
                                   const uint8_t* batch_bytes,
                                   size_t batch_length_bytes,
                                   SdCsvAppendDiagnostics* diag_out)
{
  return SdCsvAppendBatchWithReadbackVerifyEx(file_handle,
                                              batch_bytes,
                                              batch_length_bytes,
                                              diag_out,
                                              NULL,
                                              true);
}

/**
 * @brief Execute SdCsvAppendBatchWithReadbackVerifyEx.
 * @param file_handle Parameter file_handle.
 * @param batch_bytes Parameter batch_bytes.
 * @param batch_length_bytes Parameter batch_length_bytes.
 * @param diag_out Parameter diag_out.
 * @param scratch Parameter scratch.
 * @param flush_per_append Parameter flush_per_append.
 * @return Return the function result.
 */
esp_err_t
SdCsvAppendBatchWithReadbackVerifyEx(FILE* file_handle,
                                     const uint8_t* batch_bytes,
                                     size_t batch_length_bytes,
                                     SdCsvAppendDiagnostics* diag_out,
                                     const sd_csv_append_scratch_t* scratch,
                                     bool flush_per_append)
{
  if (file_handle == NULL || batch_bytes == NULL || batch_length_bytes == 0) {
    SetAppendDiagnostics(diag_out, "append", 0);
    return ESP_ERR_INVALID_ARG;
  }

  const int file_descriptor = fileno(file_handle);
  if (file_descriptor < 0) {
    ESP_LOGE(kTag, "fileno() failed");
    SetAppendDiagnostics(diag_out, "append", errno);
    return ESP_FAIL;
  }

  off_t original_size = 0;
  if (GetFileSizeBytes(file_descriptor, &original_size) != ESP_OK) {
    SetAppendDiagnostics(diag_out, "append", errno);
    return ESP_FAIL;
  }
  const off_t append_offset = original_size;

  SetAppendDiagnostics(diag_out, NULL, 0);
  const sha256_digest_t digest_before =
    ComputeSha256(batch_bytes, batch_length_bytes);

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
        fwrite(scratch->io_bounce_bytes, 1, chunk, file_handle);
      if (written != chunk) {
        ESP_LOGE(kTag,
                 "fwrite() short write: wrote=%u expected=%u",
                 (unsigned)written,
                 (unsigned)chunk);
        SetAppendDiagnostics(diag_out, "append", errno);
        ftruncate(file_descriptor, original_size);
        fsync(file_descriptor);
        return ESP_FAIL;
      }
      offset += chunk;
    }
  } else {
    const size_t written =
      fwrite(batch_bytes, 1, batch_length_bytes, file_handle);
    if (written != batch_length_bytes) {
      ESP_LOGE(kTag,
               "fwrite() short write: wrote=%u expected=%u",
               (unsigned)written,
               (unsigned)batch_length_bytes);
      SetAppendDiagnostics(diag_out, "append", errno);
      ftruncate(file_descriptor, original_size);
      fsync(file_descriptor);
      return ESP_FAIL;
    }
  }

  if (fflush(file_handle) != 0) {
    ESP_LOGE(kTag, "fflush() failed: errno=%d (%s)", errno, strerror(errno));
    SetAppendDiagnostics(diag_out, "fflush", errno);
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  if (flush_per_append) {
    if (fsync(file_descriptor) != 0) {
      ESP_LOGE(kTag, "fsync() failed: errno=%d (%s)", errno, strerror(errno));
      SetAppendDiagnostics(diag_out, "fsync", errno);
      ftruncate(file_descriptor, original_size);
      fsync(file_descriptor);
      return ESP_FAIL;
    }
  }

  uint8_t* readback_bytes = NULL;
  size_t readback_capacity = 0;
  bool readback_owned = false;
  if (scratch != NULL && scratch->verify_readback_bytes != NULL &&
      scratch->verify_readback_capacity >= batch_length_bytes) {
    readback_bytes = scratch->verify_readback_bytes;
    readback_capacity = scratch->verify_readback_capacity;
  } else if (scratch != NULL && scratch->io_bounce_bytes != NULL &&
             scratch->io_bounce_capacity > 0) {
    readback_bytes = scratch->io_bounce_bytes;
    readback_capacity = scratch->io_bounce_capacity;
  } else if (scratch == NULL) {
    // Read-back buffer is used for file I/O only; PSRAM is safe.
    readback_bytes = (uint8_t*)heap_caps_malloc(
      batch_length_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (readback_bytes == NULL) {
      readback_bytes = (uint8_t*)heap_caps_malloc(
        batch_length_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (readback_bytes == NULL) {
      return ESP_ERR_NO_MEM;
    }
    readback_capacity = batch_length_bytes;
    readback_owned = true;
  } else {
    return ESP_ERR_NO_MEM;
  }

  sha256_digest_t digest_after = { 0 };
  if (ComputeSha256FromFileRegion(file_descriptor,
                                  append_offset,
                                  batch_length_bytes,
                                  readback_bytes,
                                  readback_capacity,
                                  &digest_after) != ESP_OK) {
    ESP_LOGE(kTag, "Read-back failed; truncating to original size.");
    SetAppendDiagnostics(diag_out, "verify", errno);
    if (readback_owned) {
      AppFree(readback_bytes);
    }
    ftruncate(file_descriptor, original_size);
    fsync(file_descriptor);
    return ESP_FAIL;
  }

  if (readback_owned) {
    AppFree(readback_bytes);
  }

  if (!DigestsEqual(&digest_before, &digest_after)) {
    ESP_LOGE(kTag, "SD verify failed (SHA mismatch). Truncating append.");
    SetAppendDiagnostics(diag_out, "verify", 0);
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

/**
 * @brief Execute SdCsvFlushAndSync.
 * @param file_handle Parameter file_handle.
 * @param diag_out Parameter diag_out.
 * @return Return the function result.
 */
esp_err_t
SdCsvFlushAndSync(FILE* file_handle, SdCsvAppendDiagnostics* diag_out)
{
  if (file_handle == NULL) {
    SetAppendDiagnostics(diag_out, "fflush", 0);
    return ESP_ERR_INVALID_ARG;
  }

  const int file_descriptor = fileno(file_handle);
  if (file_descriptor < 0) {
    ESP_LOGE(kTag, "fileno() failed");
    SetAppendDiagnostics(diag_out, "fflush", errno);
    return ESP_FAIL;
  }

  if (fflush(file_handle) != 0) {
    ESP_LOGE(kTag, "fflush() failed: errno=%d (%s)", errno, strerror(errno));
    SetAppendDiagnostics(diag_out, "fflush", errno);
    return ESP_FAIL;
  }

  if (fsync(file_descriptor) != 0) {
    ESP_LOGE(kTag, "fsync() failed: errno=%d (%s)", errno, strerror(errno));
    SetAppendDiagnostics(diag_out, "fsync", errno);
    return ESP_FAIL;
  }

  SetAppendDiagnostics(diag_out, NULL, 0);
  return ESP_OK;
}
