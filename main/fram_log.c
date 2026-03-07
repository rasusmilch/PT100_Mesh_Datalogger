#include "fram_log.h"

#include <inttypes.h>
#include <string.h>

#include "crc16.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* kTag = "fram_log";

#define FRAM_LOG_MAGIC 0x46524C47u // 'FRLG'
#define FRAM_LOG_VERSION 4u

// Reserve some bytes for two header copies.
static const uint32_t kHeaderCopy0Address = 0;
static const uint32_t kHeaderCopy1Address = 128;
static const uint32_t kRecordRegionOffset = 256;

/**
 * @brief Execute IoRead.
 * @param log Parameter log.
 * @param address Parameter address.
 * @param out Parameter out.
 * @param len Parameter len.
 * @return Return the function result.
 */
static esp_err_t
IoRead(const fram_log_t* log, uint32_t address, void* out, size_t len)
{
  if (log == NULL || log->io.read == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return log->io.read(log->io.context, address, out, len);
}

/**
 * @brief Execute IoWrite.
 * @param log Parameter log.
 * @param address Parameter address.
 * @param data Parameter data.
 * @param len Parameter len.
 * @return Return the function result.
 */
static esp_err_t
IoWrite(const fram_log_t* log, uint32_t address, const void* data, size_t len)
{
  if (log == NULL || log->io.write == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return log->io.write(log->io.context, address, data, len);
}

#pragma pack(push, 1)
typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t generation_counter;
  uint32_t write_index;
  uint32_t read_index;
  uint32_t record_count;
  uint32_t next_sequence;
  uint64_t next_record_id;
  uint32_t crc32_le;
} fram_log_header_t;
#pragma pack(pop)

/**
 * @brief Execute ComputeHeaderCrc32.
 * @param header Parameter header.
 * @return Return the function result.
 */
static uint32_t
ComputeHeaderCrc32(const fram_log_header_t* header)
{
  // Compute CRC over everything except the crc field itself.
  fram_log_header_t temp;
  memcpy(&temp, header, sizeof(temp));
  temp.crc32_le = 0;
  return esp_rom_crc32_le(0, (const uint8_t*)&temp, sizeof(temp));
}

/**
 * @brief Execute HeaderLooksValid.
 * @param header Parameter header.
 * @return Return the function result.
 */
static bool
HeaderLooksValid(const fram_log_header_t* header)
{
  if (header->magic != FRAM_LOG_MAGIC) {
    return false;
  }
  if (header->version != FRAM_LOG_VERSION) {
    return false;
  }
  const uint32_t crc = ComputeHeaderCrc32(header);
  return crc == header->crc32_le;
}

/**
 * @brief Execute ReadHeaderAt.
 * @param log Parameter log.
 * @param address Parameter address.
 * @param header_out Parameter header_out.
 * @return Return the function result.
 */
static esp_err_t
ReadHeaderAt(const fram_log_t* log,
             uint32_t address,
             fram_log_header_t* header_out)
{
  return IoRead(log, address, header_out, sizeof(*header_out));
}

/**
 * @brief Execute WriteHeaderAt.
 * @param log Parameter log.
 * @param address Parameter address.
 * @param header Parameter header.
 * @return Return the function result.
 */
static esp_err_t
WriteHeaderAt(const fram_log_t* log,
              uint32_t address,
              const fram_log_header_t* header)
{
  return IoWrite(log, address, header, sizeof(*header));
}

/**
 * @brief Execute ApplyHeaderToState.
 * @param header Parameter header.
 * @param log Parameter log.
 */
static void
ApplyHeaderToState(const fram_log_header_t* header, fram_log_t* log)
{
  log->header_generation = header->generation_counter;
  log->write_index = header->write_index;
  log->read_index = header->read_index;
  log->record_count = header->record_count;
  log->next_sequence = header->next_sequence;
  log->next_record_id = header->next_record_id;
}

/**
 * @brief Execute BuildHeaderFromState.
 * @param log Parameter log.
 * @param generation_counter Parameter generation_counter.
 * @param header_out Parameter header_out.
 */
static void
BuildHeaderFromState(const fram_log_t* log,
                     uint32_t generation_counter,
                     fram_log_header_t* header_out)
{
  memset(header_out, 0, sizeof(*header_out));
  header_out->magic = FRAM_LOG_MAGIC;
  header_out->version = FRAM_LOG_VERSION;
  header_out->generation_counter = generation_counter;
  header_out->write_index = log->write_index;
  header_out->read_index = log->read_index;
  header_out->record_count = log->record_count;
  header_out->next_sequence = log->next_sequence;
  header_out->next_record_id = log->next_record_id;
  header_out->crc32_le = ComputeHeaderCrc32(header_out);
}

/**
 * @brief Execute RecordAddressForIndex.
 * @param log Parameter log.
 * @param record_index Parameter record_index.
 * @return Return the function result.
 */
static uint32_t
RecordAddressForIndex(const fram_log_t* log, uint32_t record_index)
{
  const uint32_t slot = record_index % log->capacity_records;
  return log->record_region_offset + slot * sizeof(log_record_t);
}

/**
 * @brief Execute FramLogValidateResultToString.
 * @param result Parameter result.
 * @return Return the function result.
 */
const char*
FramLogValidateResultToString(fram_log_validate_result_t result)
{
  switch (result) {
    case OK:
      return "ok";
    case MAGIC_MISMATCH:
      return "bad-magic";
    case SCHEMA_MISMATCH:
      return "bad-schema";
    case CRC_MISMATCH:
      return "bad-crc";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute FramLogValidateRecord.
 * @param record Parameter record.
 * @param actual_crc_out Parameter actual_crc_out.
 * @return Return the function result.
 */
fram_log_validate_result_t
FramLogValidateRecord(const log_record_t* record, uint16_t* actual_crc_out)
{
  if (record == NULL) {
    if (actual_crc_out != NULL) {
      *actual_crc_out = 0;
    }
    return CRC_MISMATCH;
  }
  if (record->magic != LOG_RECORD_MAGIC) {
    if (actual_crc_out != NULL) {
      *actual_crc_out = 0;
    }
    return MAGIC_MISMATCH;
  }
  if (record->schema_version != LOG_RECORD_SCHEMA_VER) {
    if (actual_crc_out != NULL) {
      *actual_crc_out = 0;
    }
    return SCHEMA_MISMATCH;
  }

  const uint16_t expected_crc = record->crc16_ccitt;
  log_record_t temp_record;
  memcpy(&temp_record, record, sizeof(temp_record));
  temp_record.crc16_ccitt = 0;
  const uint16_t actual_crc = Crc16CcittFalse(
    &temp_record, sizeof(temp_record) - sizeof(temp_record.crc16_ccitt));

  if (actual_crc_out != NULL) {
    *actual_crc_out = actual_crc;
  }
  if (expected_crc != actual_crc) {
    return CRC_MISMATCH;
  }
  return OK;
}

/**
 * @brief Execute WriteRecord.
 * @param log Parameter log.
 * @param record_index Parameter record_index.
 * @param record Parameter record.
 * @return Return the function result.
 */
static esp_err_t
WriteRecord(const fram_log_t* log, uint32_t record_index, log_record_t record)
{
  record.magic = LOG_RECORD_MAGIC;
  record.schema_version = LOG_RECORD_SCHEMA_VER;
  record.crc16_ccitt = 0;
  record.crc16_ccitt =
    Crc16CcittFalse(&record, sizeof(record) - sizeof(record.crc16_ccitt));

  const uint32_t address = RecordAddressForIndex(log, record_index);

  // Verify on write to catch bus noise/torn transactions early (prevents
  // persisting bad records that would later be dropped during SD flush).
  static const int kWriteVerifyRetries = 2;
  esp_err_t last_error = ESP_OK;
  for (int attempt = 0; attempt <= kWriteVerifyRetries; ++attempt) {
    const esp_err_t write_result =
      IoWrite(log, address, &record, sizeof(record));
    if (write_result != ESP_OK) {
      last_error = write_result;
      continue;
    }

    log_record_t verify;
    const esp_err_t read_result = IoRead(log, address, &verify, sizeof(verify));
    if (read_result != ESP_OK) {
      last_error = read_result;
      continue;
    }

    uint16_t actual_crc = 0;
    const fram_log_validate_result_t validate_result =
      FramLogValidateRecord(&verify, &actual_crc);
    if (validate_result == OK) {
      return ESP_OK;
    }
    last_error = ESP_ERR_INVALID_RESPONSE;
  }

  ((fram_log_t*)log)->saw_corruption = true;
  ESP_LOGE(kTag,
           "FRAM write verify failed at index=%u slot=%u addr=0x%04x: %s",
           (unsigned)record_index,
           (unsigned)(record_index % ((fram_log_t*)log)->capacity_records),
           (unsigned)address,
           esp_err_to_name(last_error));
  return last_error;
}

/**
 * @brief Execute ReadRecord.
 * @param log Parameter log.
 * @param record_index Parameter record_index.
 * @param record_out Parameter record_out.
 * @return Return the function result.
 */
static esp_err_t
ReadRecord(const fram_log_t* log,
           uint32_t record_index,
           log_record_t* record_out)
{
  const uint32_t address = RecordAddressForIndex(log, record_index);
  esp_err_t result = IoRead(log, address, record_out, sizeof(*record_out));
  if (result != ESP_OK) {
    return result;
  }
  if (FramLogValidateRecord(record_out, NULL) != OK) {
    ((fram_log_t*)log)->saw_corruption = true;
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

/**
 * @brief Execute FramLogInit.
 * @param log Parameter log.
 * @param io Parameter io.
 * @param fram_size_bytes Parameter fram_size_bytes.
 * @return Return the function result.
 */
esp_err_t
FramLogInit(fram_log_t* log, fram_io_t io, uint32_t fram_size_bytes)
{
  if (log == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (fram_size_bytes == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (io.read == NULL || io.write == NULL || io.context == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(log, 0, sizeof(*log));
  log->io = io;
  log->fram_size_bytes = fram_size_bytes;
  log->record_region_offset = kRecordRegionOffset;
  log->mounted = false;

  if (fram_size_bytes <= kRecordRegionOffset + sizeof(log_record_t)) {
    return ESP_ERR_INVALID_SIZE;
  }

  log->capacity_records =
    (fram_size_bytes - kRecordRegionOffset) / sizeof(log_record_t);
  if (log->capacity_records == 0) {
    return ESP_ERR_INVALID_SIZE;
  }

  fram_log_header_t header0;
  fram_log_header_t header1;
  esp_err_t result0 = ReadHeaderAt(log, kHeaderCopy0Address, &header0);
  esp_err_t result1 = ReadHeaderAt(log, kHeaderCopy1Address, &header1);

  const bool header0_valid = (result0 == ESP_OK) && HeaderLooksValid(&header0);
  const bool header1_valid = (result1 == ESP_OK) && HeaderLooksValid(&header1);

  if (!header0_valid && !header1_valid) {
    ESP_LOGW(kTag, "No valid FRAM header; scanning for latest record_id");
    log->header_generation = 0;
    log->header_copy_index = 1;
    log->write_index = 0;
    log->read_index = 0;
    log->record_count = 0;
    log->next_sequence = 1;
    log->next_record_id = 1;
    log->records_since_header_persist = 0;
    log->saw_corruption = false;
    uint64_t max_record_id = 0;
    for (uint32_t idx = 0; idx < log->capacity_records; ++idx) {
      log_record_t record;
      const uint32_t address = RecordAddressForIndex(log, idx);
      if (IoRead(log, address, &record, sizeof(record)) != ESP_OK) {
        continue;
      }
      if (record.magic != LOG_RECORD_MAGIC ||
          record.schema_version != LOG_RECORD_SCHEMA_VER) {
        continue;
      }
      const uint16_t expected_crc = record.crc16_ccitt;
      record.crc16_ccitt = 0;
      const uint16_t actual_crc =
        Crc16CcittFalse(&record, sizeof(record) - sizeof(uint16_t));
      record.crc16_ccitt = expected_crc;
      if (expected_crc != actual_crc) {
        continue;
      }
      if (record.record_id > max_record_id) {
        max_record_id = record.record_id;
      }
    }
    if (max_record_id > 0) {
      log->next_record_id = max_record_id + 1;
      ESP_LOGW(kTag,
               "Recovered next_record_id=%" PRIu64 " from FRAM scan",
               log->next_record_id);
    }
    esp_err_t persist_result = FramLogPersistHeader(log);
    if (persist_result == ESP_OK) {
      log->mounted = true;
    }
    return persist_result;
  }

  const fram_log_header_t* chosen = NULL;
  uint8_t chosen_index = 0;
  if (header0_valid && header1_valid) {
    if (header1.generation_counter >= header0.generation_counter) {
      chosen = &header1;
      chosen_index = 1;
    } else {
      chosen = &header0;
      chosen_index = 0;
    }
  } else if (header0_valid) {
    chosen = &header0;
    chosen_index = 0;
  } else {
    chosen = &header1;
    chosen_index = 1;
  }

  ApplyHeaderToState(chosen, log);
  log->header_copy_index = chosen_index;
  log->saw_corruption = false;
  log->records_since_header_persist = 0;

  // Sanity clamp.
  if (log->record_count > log->capacity_records) {
    log->record_count = log->capacity_records;
  }

  uint32_t max_sequence = log->next_sequence;
  uint64_t max_record_id = log->next_record_id;
  for (uint32_t idx = 0; idx < log->record_count; ++idx) {
    log_record_t record;
    if (FramLogPeekOffset(log, idx, &record) != ESP_OK) {
      break;
    }
    if (record.sequence >= max_sequence) {
      max_sequence = record.sequence + 1u;
    }
    if (record.record_id >= max_record_id) {
      max_record_id = record.record_id + 1u;
    }
  }
  if (max_sequence == 0) {
    max_sequence = 1;
  }
  if (max_record_id == 0) {
    max_record_id = 1;
  }
  log->next_sequence = max_sequence;
  log->next_record_id = max_record_id;

  ESP_LOGI(kTag,
           "FRAM log: cap=%u rec write=%u read=%u count=%u seq=%u id=%" PRIu64,
           (unsigned)log->capacity_records,
           (unsigned)log->write_index,
           (unsigned)log->read_index,
           (unsigned)log->record_count,
           (unsigned)log->next_sequence,
           log->next_record_id);
  log->mounted = true;
  return ESP_OK;
}

/**
 * @brief Execute FramLogGetCapacityRecords.
 * @param log Parameter log.
 * @return Return the function result.
 */
size_t
FramLogGetCapacityRecords(const fram_log_t* log)
{
  return (log == NULL) ? 0 : (size_t)log->capacity_records;
}

/**
 * @brief Execute FramLogGetBufferedRecords.
 * @param log Parameter log.
 * @return Return the function result.
 */
uint32_t
FramLogGetBufferedRecords(const fram_log_t* log)
{
  return (log == NULL) ? 0 : log->record_count;
}

/**
 * @brief Execute FramLogGetCountRecords.
 * @param log Parameter log.
 * @return Return the function result.
 */
size_t
FramLogGetCountRecords(const fram_log_t* log)
{
  return (log == NULL) ? 0 : (size_t)log->record_count;
}

/**
 * @brief Execute FramLogGetOverrunRecordsTotal.
 * @param log Parameter log.
 * @return Return the function result.
 */
uint64_t
FramLogGetOverrunRecordsTotal(const fram_log_t* log)
{
  return (log == NULL) ? 0 : log->overrun_records_total;
}

/**
 * @brief Execute FramLogIsOverwriting.
 * @param log Parameter log.
 * @return Return the function result.
 */
bool
FramLogIsOverwriting(const fram_log_t* log)
{
  return (log != NULL) && (log->overrun_records_total > 0);
}

/**
 * @brief Execute FramLogNextSequence.
 * @param log Parameter log.
 * @return Return the function result.
 */
uint32_t
FramLogNextSequence(const fram_log_t* log)
{
  return (log == NULL) ? 0 : log->next_sequence;
}

/**
 * @brief Execute FramLogNextRecordId.
 * @param log Parameter log.
 * @return Return the function result.
 */
uint64_t
FramLogNextRecordId(const fram_log_t* log)
{
  return (log == NULL) ? 0 : log->next_record_id;
}

/**
 * @brief Execute FramLogGetStatus.
 * @param log Parameter log.
 * @param out_status Parameter out_status.
 * @return Return the function result.
 */
esp_err_t
FramLogGetStatus(const fram_log_t* log, fram_log_status_t* out_status)
{
  if (log == NULL || out_status == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!log->mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  out_status->capacity_records = log->capacity_records;
  out_status->record_size_bytes = sizeof(log_record_t);
  out_status->flush_watermark_records = 0;
  out_status->buffered_count = log->record_count;
  out_status->write_index_abs = log->write_index;
  out_status->read_index_abs = log->read_index;
  out_status->next_sequence = log->next_sequence;
  out_status->next_record_id = log->next_record_id;
  out_status->mounted = log->mounted;
  out_status->full = (log->record_count >= log->capacity_records);
  return ESP_OK;
}

/**
 * @brief Execute FramLogReset.
 * @param log Parameter log.
 * @return Return the function result.
 */
esp_err_t
FramLogReset(fram_log_t* log)
{
  if (log == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!log->mounted) {
    return ESP_ERR_INVALID_STATE;
  }

  log->write_index = 0;
  log->read_index = 0;
  log->record_count = 0;
  log->next_sequence = 1;
  log->next_record_id = 1;
  log->overrun_records_total = 0;
  log->overrun_events_total = 0;
  log->records_since_header_persist = 0;
  log->saw_corruption = false;

  return FramLogPersistHeader(log);
}

/**
 * @brief Execute FramLogAssignRecordIds.
 * @param log Parameter log.
 * @param record Parameter record.
 * @return Return the function result.
 */
esp_err_t
FramLogAssignRecordIds(fram_log_t* log, log_record_t* record)
{
  if (log == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  record->sequence = log->next_sequence;
  record->record_id = log->next_record_id;
  record->schema_version = LOG_RECORD_SCHEMA_VER;

  log->next_sequence++;
  log->next_record_id++;
  log->records_since_header_persist++;

  if (log->records_since_header_persist >=
      (uint32_t)CONFIG_APP_FRAM_HEADER_UPDATE_EVERY_N_RECORDS) {
    return FramLogPersistHeader(log);
  }
  return ESP_OK;
}

/**
 * @brief Execute FramLogPersistHeader.
 * @param log Parameter log.
 * @return Return the function result.
 */
esp_err_t
FramLogPersistHeader(fram_log_t* log)
{
  if (log == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  const uint32_t next_generation = log->header_generation + 1u;
  fram_log_header_t header;
  BuildHeaderFromState(log, next_generation, &header);

  // Alternate header copies to reduce single-address pounding (even though FRAM
  // tolerates it well).
  const uint8_t next_copy_index = (uint8_t)((log->header_copy_index + 1u) % 2u);
  const uint32_t address =
    (next_copy_index == 0u) ? kHeaderCopy0Address : kHeaderCopy1Address;

  esp_err_t result = WriteHeaderAt(log, address, &header);
  if (result != ESP_OK) {
    return result;
  }

  fram_log_header_t verify;
  result = ReadHeaderAt(log, address, &verify);
  if (result != ESP_OK) {
    return result;
  }
  if (!HeaderLooksValid(&verify) ||
      verify.generation_counter != next_generation) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  log->header_generation = next_generation;
  log->header_copy_index = next_copy_index;
  log->records_since_header_persist = 0;
  return ESP_OK;
}

/**
 * @brief Execute FramLogAppend.
 * @param log Parameter log.
 * @param record Parameter record.
 * @return Return the function result.
 */
esp_err_t
FramLogAppend(fram_log_t* log, const log_record_t* record)
{
  if (log == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (log->record_count >= log->capacity_records) {
    log->read_index++;
    if (log->record_count > 0) {
      log->record_count--;
    }
    log->overrun_records_total++;
    if (log->overrun_records_total == 1u) {
      log->overrun_events_total++;
    }
  }

  log_record_t record_copy;
  memcpy(&record_copy, record, sizeof(record_copy));
  esp_err_t result = WriteRecord(log, log->write_index, record_copy);
  if (result != ESP_OK) {
    return result;
  }

  log->write_index++;
  if (log->record_count < log->capacity_records) {
    log->record_count++;
  } else {
    log->record_count = log->capacity_records;
  }

  if (record->sequence >= log->next_sequence) {
    log->next_sequence = record->sequence + 1u;
  }
  if (record->record_id >= log->next_record_id) {
    log->next_record_id = record->record_id + 1u;
  }

  return ESP_OK;
}

/**
 * @brief Execute FramLogPeekOldest.
 * @param log Parameter log.
 * @param record_out Parameter record_out.
 * @return Return the function result.
 */
esp_err_t
FramLogPeekOldest(const fram_log_t* log, log_record_t* record_out)
{
  if (log == NULL || record_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (log->record_count == 0) {
    return ESP_ERR_NOT_FOUND;
  }
  // Read raw bytes and validate. Always return populated bytes.
  const uint32_t address = RecordAddressForIndex(log, log->read_index);
  const uint32_t slot = log->read_index % log->capacity_records;
  const int kMaxAttempts = 3;
  fram_log_validate_result_t last_reason = OK;
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    log_record_t candidate;
    const esp_err_t read_result =
      IoRead(log, address, &candidate, sizeof(candidate));
    if (read_result != ESP_OK) {
      return read_result;
    }
    *record_out = candidate;
    uint16_t actual_crc = 0;
    last_reason = FramLogValidateRecord(record_out, &actual_crc);
    if (last_reason == OK) {
      return ESP_OK;
    }
    if (attempt < (kMaxAttempts - 1)) {
      ESP_LOGW(kTag,
               "PeekOldest retry attempt=%d slot=%u addr=0x%04x last_reason=%s",
               attempt + 1,
               (unsigned)slot,
               (unsigned)address,
               FramLogValidateResultToString(last_reason));
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
  ((fram_log_t*)log)->saw_corruption = true;
  return ESP_ERR_INVALID_RESPONSE;
}

/**
 * @brief Execute FramLogPeekOffset.
 * @param log Parameter log.
 * @param offset Parameter offset.
 * @param record_out Parameter record_out.
 * @return Return the function result.
 */
esp_err_t
FramLogPeekOffset(const fram_log_t* log,
                  uint32_t offset,
                  log_record_t* record_out)
{
  if (log == NULL || record_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (offset >= log->record_count) {
    return ESP_ERR_NOT_FOUND;
  }
  const uint32_t record_index = log->read_index + offset;
  const uint32_t address = RecordAddressForIndex(log, record_index);
  esp_err_t result = IoRead(log, address, record_out, sizeof(*record_out));
  if (result != ESP_OK) {
    return result;
  }
  if (FramLogValidateRecord(record_out, NULL) != OK) {
    for (int attempt = 0; attempt < 2; ++attempt) {
      log_record_t retry_record;
      esp_err_t retry_result =
        IoRead(log, address, &retry_record, sizeof(retry_record));
      if (retry_result == ESP_OK &&
          FramLogValidateRecord(&retry_record, NULL) == OK) {
        *record_out = retry_record;
        return ESP_OK;
      }
    }
    ((fram_log_t*)log)->saw_corruption = true;
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

/**
 * @brief Execute FramLogDiscardOldest.
 * @param log Parameter log.
 * @return Return the function result.
 */
esp_err_t
FramLogDiscardOldest(fram_log_t* log)
{
  if (log == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (log->record_count == 0) {
    return ESP_ERR_NOT_FOUND;
  }
  log->read_index++;
  log->record_count--;
  log->records_since_header_persist++;

  // Persist periodically to avoid holding the FRAM log mutex for long durations
  // during bulk discards. Callers that require best durability should
  // explicitly call FramLogPersistHeader() after completing a bulk discard.
  if (log->records_since_header_persist >=
      (uint32_t)CONFIG_APP_FRAM_HEADER_UPDATE_EVERY_N_RECORDS) {
    return FramLogPersistHeader(log);
  }
  return ESP_OK;
}

/**
 * @brief Execute FramLogPopOldest.
 * @param log Parameter log.
 * @param record_out Parameter record_out.
 * @return Return the function result.
 */
esp_err_t
FramLogPopOldest(fram_log_t* log, log_record_t* record_out)
{
  if (log == NULL || record_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (log->record_count == 0) {
    return ESP_ERR_NOT_FOUND;
  }

  esp_err_t result = ReadRecord(log, log->read_index, record_out);
  if (result != ESP_OK) {
    log->saw_corruption = true;
    ESP_LOGW(kTag,
             "Bad record at index=%u: %s; refusing to consume further",
             (unsigned)log->read_index,
             esp_err_to_name(result));
    return result;
  }

  log->read_index++;
  log->record_count--;
  log->records_since_header_persist++;

  if (log->records_since_header_persist >=
      (uint32_t)CONFIG_APP_FRAM_HEADER_UPDATE_EVERY_N_RECORDS) {
    return FramLogPersistHeader(log);
  }
  return ESP_OK;
}

/**
 * @brief Execute FramLogSkipCorruptedRecord.
 * @param log Parameter log.
 * @return Return the function result.
 */
esp_err_t
FramLogSkipCorruptedRecord(fram_log_t* log)
{
  if (log == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (log->record_count == 0) {
    return ESP_ERR_NOT_FOUND;
  }

  // Best-effort detail to help diagnose why corruption occurred.
  const uint32_t address = RecordAddressForIndex(log, log->read_index);
  log_record_t record;
  const esp_err_t read_result = IoRead(log, address, &record, sizeof(record));
  if (read_result == ESP_OK) {
    uint16_t actual_crc = 0;
    const fram_log_validate_result_t validate_result =
      FramLogValidateRecord(&record, &actual_crc);
    const uint16_t expected_crc = record.crc16_ccitt;
    ESP_LOGW(kTag,
             "Skipping corrupted record at index=%u slot=%u addr=0x%04x "
             "reason=%s magic=0x%08" PRIx32 " schema=%" PRIu32 " "
             "exp_crc=0x%04x act_crc=0x%04x",
             (unsigned)log->read_index,
             (unsigned)(log->read_index % log->capacity_records),
             (unsigned)address,
             FramLogValidateResultToString(validate_result),
             record.magic,
             record.schema_version,
             (unsigned)expected_crc,
             (unsigned)actual_crc);
  } else {
    ESP_LOGW(kTag,
             "Skipping corrupted record at index=%u slot=%u addr=0x%04x "
             "(read failed: %s)",
             (unsigned)log->read_index,
             (unsigned)(log->read_index % log->capacity_records),
             (unsigned)address,
             esp_err_to_name(read_result));
  }

  log->saw_corruption = true;
  log->read_index++;
  log->record_count--;
  log->records_since_header_persist++;
  return FramLogPersistHeader(log);
}

/**
 * @brief Execute FramLogConsumeUpToRecordId.
 * @param log Parameter log.
 * @param max_record_id_inclusive Parameter max_record_id_inclusive.
 * @param consumed_out Parameter consumed_out.
 * @return Return the function result.
 */
esp_err_t
FramLogConsumeUpToRecordId(fram_log_t* log,
                           uint64_t max_record_id_inclusive,
                           uint32_t* consumed_out)
{
  if (log == NULL || consumed_out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t consumed = 0;
  uint32_t invalid_discards = 0;
  const uint32_t kMaxInvalidDiscards = 8;
  esp_err_t status = ESP_OK;

  while (FramLogGetBufferedRecords(log) > 0) {
    log_record_t peeked;
    esp_err_t peek_result = FramLogPeekOldest(log, &peeked);
    if (peek_result == ESP_ERR_INVALID_RESPONSE) {
      if (invalid_discards >= kMaxInvalidDiscards) {
        ESP_LOGE(kTag,
                 "Exceeded invalid record discard limit while consuming up to "
                 "id=%" PRIu64,
                 max_record_id_inclusive);
        status = ESP_ERR_INVALID_RESPONSE;
        break;
      }
      ESP_LOGW(
        kTag,
        "Discarding invalid FRAM record while consuming up to id=%" PRIu64
        " (discarded=%u)",
        max_record_id_inclusive,
        (unsigned)(invalid_discards + 1u));
      esp_err_t skip_result = FramLogSkipCorruptedRecord(log);
      if (skip_result != ESP_OK) {
        status = skip_result;
        break;
      }
      invalid_discards++;
      continue;
    }
    if (peek_result != ESP_OK) {
      status = peek_result;
      break;
    }
    if (peeked.record_id > max_record_id_inclusive) {
      break;
    }
    esp_err_t pop_result = FramLogPopOldest(log, &peeked);
    if (pop_result != ESP_OK) {
      status = pop_result;
      break;
    }
    consumed++;
  }

  *consumed_out = consumed;
  return status;
}
