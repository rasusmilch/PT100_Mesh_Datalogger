#include "fram_log.h"

#include <string.h>

#include "crc16.h"
#include "esp_log.h"
#include "esp_rom_crc.h"

static const char* kTag = "fram_log";

#define FRAM_LOG_MAGIC 0x46524C47u // 'FRLG'
#define FRAM_LOG_VERSION 1u

// Reserve some bytes for two header copies.
static const uint32_t kHeaderCopy0Address = 0;
static const uint32_t kHeaderCopy1Address = 128;
static const uint32_t kRecordRegionOffset = 256;

#pragma pack(push, 1)
typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t header_sequence;
  uint32_t write_index;
  uint32_t read_index;
  uint32_t record_count;
  uint32_t crc32_le;
} fram_log_header_t;
#pragma pack(pop)

static uint32_t
ComputeHeaderCrc32(const fram_log_header_t* header)
{
  // Compute CRC over everything except the crc field itself.
  fram_log_header_t temp;
  memcpy(&temp, header, sizeof(temp));
  temp.crc32_le = 0;
  return esp_rom_crc32_le(0, (const uint8_t*)&temp, sizeof(temp));
}

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

static esp_err_t
ReadHeaderAt(const fram_spi_t* fram,
             uint32_t address,
             fram_log_header_t* header_out)
{
  return FramSpiRead(fram, address, header_out, sizeof(*header_out));
}

static esp_err_t
WriteHeaderAt(const fram_spi_t* fram,
              uint32_t address,
              const fram_log_header_t* header)
{
  return FramSpiWrite(fram, address, header, sizeof(*header));
}

static void
ApplyHeaderToState(const fram_log_header_t* header, fram_log_t* log)
{
  log->header_sequence = header->header_sequence;
  log->write_index = header->write_index;
  log->read_index = header->read_index;
  log->record_count = header->record_count;
}

static void
BuildHeaderFromState(const fram_log_t* log, fram_log_header_t* header_out)
{
  memset(header_out, 0, sizeof(*header_out));
  header_out->magic = FRAM_LOG_MAGIC;
  header_out->version = FRAM_LOG_VERSION;
  header_out->header_sequence = log->header_sequence;
  header_out->write_index = log->write_index;
  header_out->read_index = log->read_index;
  header_out->record_count = log->record_count;
  header_out->crc32_le = ComputeHeaderCrc32(header_out);
}

static uint32_t
RecordAddressForIndex(const fram_log_t* log, uint32_t record_index)
{
  const uint32_t slot = record_index % log->capacity_records;
  return log->record_region_offset + slot * sizeof(log_record_t);
}

static esp_err_t
WriteRecord(const fram_log_t* log, uint32_t record_index, log_record_t record)
{
  record.magic = LOG_RECORD_MAGIC;
  record.crc16_ccitt = 0;
  record.crc16_ccitt =
    Crc16CcittFalse(&record, sizeof(record) - sizeof(record.crc16_ccitt));

  const uint32_t address = RecordAddressForIndex(log, record_index);
  return FramSpiWrite(log->fram, address, &record, sizeof(record));
}

static esp_err_t
ReadRecord(const fram_log_t* log,
           uint32_t record_index,
           log_record_t* record_out)
{
  const uint32_t address = RecordAddressForIndex(log, record_index);
  esp_err_t result =
    FramSpiRead(log->fram, address, record_out, sizeof(*record_out));
  if (result != ESP_OK) {
    return result;
  }
  if (record_out->magic != LOG_RECORD_MAGIC) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  const uint16_t expected_crc = record_out->crc16_ccitt;
  record_out->crc16_ccitt = 0;
  const uint16_t actual_crc =
    Crc16CcittFalse(record_out, sizeof(*record_out) - sizeof(uint16_t));
  record_out->crc16_ccitt = expected_crc;
  if (expected_crc != actual_crc) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

esp_err_t
FramLogInit(fram_log_t* log, fram_spi_t* fram, uint32_t fram_size_bytes)
{
  if (log == NULL || fram == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(log, 0, sizeof(*log));
  log->fram = fram;
  log->fram_size_bytes = fram_size_bytes;
  log->record_region_offset = kRecordRegionOffset;

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
  esp_err_t result0 = ReadHeaderAt(fram, kHeaderCopy0Address, &header0);
  esp_err_t result1 = ReadHeaderAt(fram, kHeaderCopy1Address, &header1);

  const bool header0_valid = (result0 == ESP_OK) && HeaderLooksValid(&header0);
  const bool header1_valid = (result1 == ESP_OK) && HeaderLooksValid(&header1);

  if (!header0_valid && !header1_valid) {
    ESP_LOGW(kTag, "No valid FRAM header; initializing new log");
    log->header_sequence = 1;
    log->write_index = 0;
    log->read_index = 0;
    log->record_count = 0;
    return FramLogPersistHeader(log);
  }

  const fram_log_header_t* chosen = NULL;
  if (header0_valid && header1_valid) {
    chosen = (header1.header_sequence >= header0.header_sequence) ? &header1
                                                                  : &header0;
  } else if (header0_valid) {
    chosen = &header0;
  } else {
    chosen = &header1;
  }

  ApplyHeaderToState(chosen, log);

  // Sanity clamp.
  if (log->record_count > log->capacity_records) {
    log->record_count = log->capacity_records;
  }

  ESP_LOGI(kTag,
           "FRAM log: cap=%u rec write=%u read=%u count=%u seq=%u",
           (unsigned)log->capacity_records,
           (unsigned)log->write_index,
           (unsigned)log->read_index,
           (unsigned)log->record_count,
           (unsigned)log->header_sequence);
  return ESP_OK;
}

uint32_t
FramLogGetCapacityRecords(const fram_log_t* log)
{
  return (log == NULL) ? 0 : log->capacity_records;
}

uint32_t
FramLogGetBufferedRecords(const fram_log_t* log)
{
  return (log == NULL) ? 0 : log->record_count;
}

esp_err_t
FramLogPersistHeader(fram_log_t* log)
{
  if (log == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  fram_log_header_t header;
  BuildHeaderFromState(log, &header);

  // Alternate header copies to reduce single-address pounding (even though FRAM
  // tolerates it well).
  const uint32_t address = (log->header_sequence % 2u == 0u)
                             ? kHeaderCopy0Address
                             : kHeaderCopy1Address;

  esp_err_t result = WriteHeaderAt(log->fram, address, &header);
  if (result == ESP_OK) {
    log->records_since_header_persist = 0;
  }
  return result;
}

esp_err_t
FramLogAppend(fram_log_t* log, const log_record_t* record)
{
  if (log == NULL || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // If full, drop the oldest record to make room.
  if (log->record_count >= log->capacity_records) {
    log->read_index++;
    log->record_count--;
  }

  log_record_t record_copy;
  memcpy(&record_copy, record, sizeof(record_copy));
  esp_err_t result = WriteRecord(log, log->write_index, record_copy);
  if (result != ESP_OK) {
    return result;
  }

  log->write_index++;
  log->record_count++;
  log->records_since_header_persist++;

  if (log->records_since_header_persist >=
      (uint32_t)CONFIG_APP_FRAM_HEADER_UPDATE_EVERY_N_RECORDS) {
    log->header_sequence++;
    return FramLogPersistHeader(log);
  }
  return ESP_OK;
}

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
  esp_err_t result =
    FramSpiRead(log->fram, address, record_out, sizeof(*record_out));
  if (result != ESP_OK) {
    return result;
  }
  if (record_out->magic != LOG_RECORD_MAGIC) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  const uint16_t expected_crc = record_out->crc16_ccitt;
  record_out->crc16_ccitt = 0;
  const uint16_t actual_crc =
    Crc16CcittFalse(record_out, sizeof(*record_out) - sizeof(uint16_t));
  record_out->crc16_ccitt = expected_crc;
  if (expected_crc != actual_crc) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

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

  // During SD flush we want best durability; persist header eagerly.
  log->header_sequence++;
  return FramLogPersistHeader(log);
}

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
    // Corruption in one slot: drop it and continue.
    ESP_LOGW(kTag,
             "Bad record at index=%u: %s; dropping",
             (unsigned)log->read_index,
             esp_err_to_name(result));
  }

  log->read_index++;
  log->record_count--;
  log->records_since_header_persist++;

  if (log->records_since_header_persist >=
      (uint32_t)CONFIG_APP_FRAM_HEADER_UPDATE_EVERY_N_RECORDS) {
    log->header_sequence++;
    return FramLogPersistHeader(log);
  }
  return ESP_OK;
}
