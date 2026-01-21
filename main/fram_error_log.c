#include "fram_error_log.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "crc16.h"
#include "esp_err.h"
#include "esp_log.h"

static const char* kTag = "fram_errlog";
static const uint32_t kFramErrorLogMagic = 0x474f4c45u; // 'ELOG'
static const uint16_t kFramErrorLogSchema = 1;

static const char* FramErrorLogStateToStringInternal(fram_errlog_state_t state);
static fram_errlog_header_status_t
FramErrorLogHeaderCheck(const fram_error_log_header_t* header);
static bool FramErrlogBufferIsBlank(const uint8_t* buffer, size_t length);
static bool FramErrlogRegionIsBlank(const fram_error_log_t* log);

typedef struct
{
  fram_error_log_header_t header;
  fram_errlog_header_status_t status;
} fram_error_log_header_state_t;

/**
 * @brief Convert errlog state to string (internal helper).
 * @param state Parameter state.
 * @return Return the function result.
 */
static const char*
FramErrorLogStateToStringInternal(fram_errlog_state_t state)
{
  switch (state) {
    case FRAM_ERRLOG_STATE_UNINIT:
      return "uninit";
    case FRAM_ERRLOG_STATE_OK:
      return "ok";
    case FRAM_ERRLOG_STATE_BLANK_INITED:
      return "blank_inited";
    case FRAM_ERRLOG_STATE_CORRUPT:
      return "corrupt";
    default:
      return "unknown";
  }
}

static size_t
FramErrorLogHeaderSize(void)
{
  return sizeof(fram_error_log_header_t);
}

static size_t
FramErrorLogEntrySize(void)
{
  return sizeof(fram_error_log_entry_t);
}

static uint16_t
FramErrorLogComputeHeaderCrc(const fram_error_log_header_t* header)
{
  fram_error_log_header_t temp = *header;
  temp.crc16 = 0;
  return Crc16CcittFalse(&temp, sizeof(temp));
}

static uint16_t
FramErrorLogComputeEntryCrc(const fram_error_log_entry_t* entry)
{
  fram_error_log_entry_t temp = *entry;
  temp.crc16 = 0;
  return Crc16CcittFalse(&temp, sizeof(temp));
}

/**
 * @brief Validate a header and return detailed status.
 * @param header Parameter header.
 * @return Return the function result.
 */
static fram_errlog_header_status_t
FramErrorLogHeaderCheck(const fram_error_log_header_t* header)
{
  fram_errlog_header_status_t status = {
    .valid = false,
    .reason = FRAM_ERRLOG_HDR_BAD_MAGIC,
  };
  if (header == NULL) {
    status.reason = FRAM_ERRLOG_HDR_IO_FAIL;
    return status;
  }
  if (header->magic != kFramErrorLogMagic) {
    status.reason = FRAM_ERRLOG_HDR_BAD_MAGIC;
    return status;
  }
  if (header->schema_ver != kFramErrorLogSchema) {
    status.reason = FRAM_ERRLOG_HDR_BAD_SCHEMA;
    return status;
  }
  const uint16_t crc = FramErrorLogComputeHeaderCrc(header);
  if (crc != header->crc16) {
    status.reason = FRAM_ERRLOG_HDR_BAD_CRC;
    return status;
  }
  status.valid = true;
  status.reason = FRAM_ERRLOG_HDR_OK;
  return status;
}

static esp_err_t
FramErrorLogReadHeaderCopy(const fram_error_log_t* log,
                           uint32_t copy_index,
                           fram_error_log_header_state_t* out)
{
  if (log == NULL || out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t header_size = FramErrorLogHeaderSize();
  const uint32_t addr = log->base_addr + (uint32_t)(copy_index * header_size);
  esp_err_t result =
    log->io.read(log->io.context, addr, &out->header, header_size);
  if (result != ESP_OK) {
    out->status.valid = false;
    out->status.reason = FRAM_ERRLOG_HDR_IO_FAIL;
    return result;
  }
  out->status = FramErrorLogHeaderCheck(&out->header);
  return ESP_OK;
}

static esp_err_t
FramErrorLogWriteHeaderCopies(const fram_error_log_t* log,
                              const fram_error_log_header_t* header)
{
  if (log == NULL || header == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t header_size = FramErrorLogHeaderSize();
  for (uint32_t copy_index = 0; copy_index < 2; ++copy_index) {
    const uint32_t addr =
      log->base_addr + (uint32_t)(copy_index * header_size);
    esp_err_t result =
      log->io.write(log->io.context, addr, header, header_size);
    if (result != ESP_OK) {
      return result;
    }
  }
  return ESP_OK;
}

static fram_error_log_header_t
FramErrorLogDefaultHeader(void)
{
  fram_error_log_header_t header = { 0 };
  header.magic = kFramErrorLogMagic;
  header.schema_ver = kFramErrorLogSchema;
  header.write_index = 0;
  header.count = 0;
  header.active_bitmap_low = 0;
  header.active_bitmap_high = 0;
  header.reserved = 0;
  header.crc16 = FramErrorLogComputeHeaderCrc(&header);
  return header;
}

/**
 * @brief Check whether a buffer is blank (0xFF).
 * @param buffer Parameter buffer.
 * @param length Parameter length.
 * @return Return the function result.
 */
static bool
FramErrlogBufferIsBlank(const uint8_t* buffer, size_t length)
{
  if (buffer == NULL || length == 0) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    if (buffer[i] != 0xFFu) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check whether the errlog region appears blank.
 * @param log Parameter log.
 * @return Return the function result.
 */
static bool
FramErrlogRegionIsBlank(const fram_error_log_t* log)
{
  if (log == NULL) {
    return false;
  }
  const size_t header_size = FramErrorLogHeaderSize();
  const size_t entry_size = FramErrorLogEntrySize();
  const size_t sample_bytes = 16;
  uint8_t sample[16];
  const size_t header_sample = (header_size < sample_bytes) ? header_size
                                                            : sample_bytes;
  const size_t entry_sample = (entry_size < sample_bytes) ? entry_size
                                                          : sample_bytes;
  const uint32_t header0_addr = log->base_addr;
  const uint32_t header1_addr = log->base_addr + (uint32_t)header_size;
  const uint32_t entry_base =
    log->base_addr + (uint32_t)(2u * header_size);
  const uint32_t last_entry_addr =
    entry_base + (uint32_t)((log->entry_capacity - 1u) * entry_size);

  if (log->io.read(log->io.context, header0_addr, sample, header_sample) !=
      ESP_OK) {
    return false;
  }
  if (!FramErrlogBufferIsBlank(sample, header_sample)) {
    return false;
  }
  if (log->io.read(log->io.context, header1_addr, sample, header_sample) !=
      ESP_OK) {
    return false;
  }
  if (!FramErrlogBufferIsBlank(sample, header_sample)) {
    return false;
  }
  if (log->io.read(log->io.context, entry_base, sample, entry_sample) !=
      ESP_OK) {
    return false;
  }
  if (!FramErrlogBufferIsBlank(sample, entry_sample)) {
    return false;
  }
  if (log->io.read(log->io.context, last_entry_addr, sample, entry_sample) !=
      ESP_OK) {
    return false;
  }
  return FramErrlogBufferIsBlank(sample, entry_sample);
}

static uint64_t
FramErrorLogActiveBitmap(const fram_error_log_header_t* header)
{
  return ((uint64_t)header->active_bitmap_high << 32) |
         (uint64_t)header->active_bitmap_low;
}

static void
FramErrorLogSetActiveBitmap(fram_error_log_header_t* header, uint64_t bitmap)
{
  header->active_bitmap_low = (uint32_t)(bitmap & 0xFFFFFFFFu);
  header->active_bitmap_high = (uint32_t)(bitmap >> 32);
}

bool
FramErrorLogGetCodeMask(uint16_t code, uint64_t* mask_out)
{
  if (mask_out == NULL) {
    return false;
  }
  if (code >= 64u) {
    return false;
  }
  *mask_out = (uint64_t)1u << code;
  return true;
}

static esp_err_t
FramErrorLogAppendInternal(fram_error_log_t* log,
                           uint16_t code,
                           int32_t detail0,
                           int32_t detail1,
                           uint16_t flags,
                           uint32_t epoch_sec,
                           uint16_t millis,
                           bool* logged_out,
                           bool want_active)
{
  if (logged_out != NULL) {
    *logged_out = false;
  }
  if (log == NULL || !log->initialized || log->entry_capacity == 0) {
    return ESP_ERR_INVALID_STATE;
  }
  if (log->state == FRAM_ERRLOG_STATE_CORRUPT) {
    return ESP_ERR_INVALID_STATE;
  }

  fram_error_log_header_t header = log->header;
  uint64_t mask = 0;
  const bool has_bit = FramErrorLogGetCodeMask(code, &mask);
  const uint64_t active_bitmap = FramErrorLogActiveBitmap(&header);

  if (has_bit) {
    const bool is_active = (active_bitmap & mask) != 0;
    if (want_active && is_active) {
      return ESP_OK;
    }
    if (!want_active && !is_active) {
      return ESP_OK;
    }
  } else {
    return ESP_ERR_INVALID_ARG;
  }

  const uint32_t header_bytes = (uint32_t)(2u * FramErrorLogHeaderSize());
  const uint32_t slot =
    (log->entry_capacity > 0)
      ? (header.write_index % log->entry_capacity)
      : 0u;
  const uint32_t addr = header_bytes + log->base_addr +
                        (uint32_t)(slot * FramErrorLogEntrySize());

  fram_error_log_entry_t entry = { 0 };
  entry.epoch_sec = epoch_sec;
  entry.millis = (millis > 999u) ? 999u : millis;
  entry.code = code;
  entry.detail0 = detail0;
  entry.detail1 = detail1;
  entry.flags = flags;
  entry.crc16 = FramErrorLogComputeEntryCrc(&entry);

  esp_err_t result =
    log->io.write(log->io.context, addr, &entry, sizeof(entry));
  if (result != ESP_OK) {
    return result;
  }

  header.write_index++;
  if (header.count < log->entry_capacity) {
    header.count++;
  }
  if (has_bit) {
    uint64_t next_bitmap = active_bitmap;
    if (want_active) {
      next_bitmap |= mask;
    } else {
      next_bitmap &= ~mask;
    }
    FramErrorLogSetActiveBitmap(&header, next_bitmap);
  }

  header.crc16 = FramErrorLogComputeHeaderCrc(&header);
  result = FramErrorLogWriteHeaderCopies(log, &header);
  if (result != ESP_OK) {
    return result;
  }

  log->header = header;
  if (logged_out != NULL) {
    *logged_out = true;
  }
  return ESP_OK;
}

static void
FramErrorLogFormatUtc(uint32_t epoch_sec, char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) {
    return;
  }
  if (epoch_sec == 0) {
    snprintf(buffer, buffer_size, "n/a");
    return;
  }
  const time_t raw = (time_t)epoch_sec;
  struct tm utc_time;
  if (gmtime_r(&raw, &utc_time) == NULL) {
    snprintf(buffer, buffer_size, "n/a");
    return;
  }
  if (strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%SZ", &utc_time) == 0) {
    snprintf(buffer, buffer_size, "n/a");
  }
}

esp_err_t
FramErrorLogInit(fram_error_log_t* log,
                 fram_io_t io,
                 uint32_t base_addr,
                 uint32_t region_bytes)
{
  if (log == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(log, 0, sizeof(*log));
  log->state = FRAM_ERRLOG_STATE_UNINIT;
  log->io = io;
  log->base_addr = base_addr;
  log->region_bytes = region_bytes;

  const size_t header_size = FramErrorLogHeaderSize();
  const size_t entry_size = FramErrorLogEntrySize();
  if (region_bytes < (2u * header_size) || entry_size == 0) {
    return ESP_ERR_INVALID_SIZE;
  }

  const uint32_t capacity =
    (uint32_t)((region_bytes - (2u * header_size)) / entry_size);
  if (capacity == 0) {
    return ESP_ERR_INVALID_SIZE;
  }
  log->entry_capacity = capacity;

  fram_error_log_header_state_t copy0 = { 0 };
  fram_error_log_header_state_t copy1 = { 0 };
  const esp_err_t read0 = FramErrorLogReadHeaderCopy(log, 0, &copy0);
  const esp_err_t read1 = FramErrorLogReadHeaderCopy(log, 1, &copy1);

  const bool copy0_valid = (read0 == ESP_OK && copy0.status.valid);
  const bool copy1_valid = (read1 == ESP_OK && copy1.status.valid);
  log->copy0_status = copy0.status;
  log->copy1_status = copy1.status;

  if (copy0_valid || copy1_valid) {
    if (copy0_valid && copy1_valid) {
      log->header = (copy0.header.write_index >= copy1.header.write_index)
                      ? copy0.header
                      : copy1.header;
    } else {
      log->header = copy0_valid ? copy0.header : copy1.header;
    }
    if (log->header.count > log->entry_capacity) {
      log->header.count = log->entry_capacity;
      log->header.crc16 = FramErrorLogComputeHeaderCrc(&log->header);
      (void)FramErrorLogWriteHeaderCopies(log, &log->header);
    }
    log->initialized = true;
    log->state = FRAM_ERRLOG_STATE_OK;
    log->region_blank = false;
    ESP_LOGW(kTag,
             "errlog init: copy0=%s copy1=%s blank=%s state=%s",
             FramErrorLogHeaderReasonToString(log->copy0_status.reason),
             FramErrorLogHeaderReasonToString(log->copy1_status.reason),
             "no",
             FramErrorLogStateToStringInternal(log->state));
    return ESP_OK;
  }

  if (read0 != ESP_OK && read1 != ESP_OK) {
    ESP_LOGW(kTag, "error log header read failed (%s/%s)",
             esp_err_to_name(read0),
             esp_err_to_name(read1));
  }

  log->region_blank = FramErrlogRegionIsBlank(log);
  if (log->region_blank) {
    log->header = FramErrorLogDefaultHeader();
    const esp_err_t write_result =
      FramErrorLogWriteHeaderCopies(log, &log->header);
    if (write_result != ESP_OK) {
      return write_result;
    }
    log->initialized = true;
    log->state = FRAM_ERRLOG_STATE_BLANK_INITED;
    ESP_LOGW(kTag,
             "errlog init: copy0=%s copy1=%s blank=%s state=%s",
             FramErrorLogHeaderReasonToString(log->copy0_status.reason),
             FramErrorLogHeaderReasonToString(log->copy1_status.reason),
             "yes",
             FramErrorLogStateToStringInternal(log->state));
    return ESP_OK;
  }

  log->header = FramErrorLogDefaultHeader();
  log->initialized = true;
  log->state = FRAM_ERRLOG_STATE_CORRUPT;
  ESP_LOGW(kTag,
           "errlog init: copy0=%s copy1=%s blank=%s state=%s",
           FramErrorLogHeaderReasonToString(log->copy0_status.reason),
           FramErrorLogHeaderReasonToString(log->copy1_status.reason),
           "no",
           FramErrorLogStateToStringInternal(log->state));
  return ESP_OK;
}

esp_err_t
FramErrorLogGetStats(const fram_error_log_t* log, fram_error_log_stats_t* out)
{
  if (log == NULL || out == NULL || !log->initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (log->state == FRAM_ERRLOG_STATE_CORRUPT) {
    return ESP_ERR_INVALID_STATE;
  }
  out->write_index = log->header.write_index;
  out->count = log->header.count;
  out->capacity = log->entry_capacity;
  out->active_bitmap_low = log->header.active_bitmap_low;
  out->active_bitmap_high = log->header.active_bitmap_high;
  return ESP_OK;
}

esp_err_t
FramErrorLogReadEntry(const fram_error_log_t* log,
                      uint32_t entry_index,
                      fram_error_log_entry_t* out,
                      bool* crc_ok_out)
{
  if (crc_ok_out != NULL) {
    *crc_ok_out = false;
  }
  if (log == NULL || out == NULL || !log->initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (log->state == FRAM_ERRLOG_STATE_CORRUPT) {
    return ESP_ERR_INVALID_STATE;
  }
  if (log->entry_capacity == 0 || entry_index >= log->header.count) {
    return ESP_ERR_INVALID_ARG;
  }

  const uint32_t header_bytes = (uint32_t)(2u * FramErrorLogHeaderSize());
  const uint32_t oldest_index =
    (log->header.count == 0)
      ? 0u
      : (log->header.write_index - log->header.count);
  const uint32_t slot =
    (log->entry_capacity > 0)
      ? ((oldest_index + entry_index) % log->entry_capacity)
      : 0u;
  const uint32_t addr = header_bytes + log->base_addr +
                        (uint32_t)(slot * FramErrorLogEntrySize());

  esp_err_t result =
    log->io.read(log->io.context, addr, out, sizeof(*out));
  if (result != ESP_OK) {
    return result;
  }
  const uint16_t crc = FramErrorLogComputeEntryCrc(out);
  if (crc_ok_out != NULL) {
    *crc_ok_out = (crc == out->crc16);
  }
  return ESP_OK;
}

esp_err_t
FramErrorLogAppendActive(fram_error_log_t* log,
                         uint16_t code,
                         int32_t detail0,
                         int32_t detail1,
                         uint32_t epoch_sec,
                         uint16_t millis,
                         bool* logged_out)
{
  return FramErrorLogAppendInternal(log,
                                    code,
                                    detail0,
                                    detail1,
                                    kFramErrorLogEntryFlagActive,
                                    epoch_sec,
                                    millis,
                                    logged_out,
                                    true);
}

esp_err_t
FramErrorLogAppendResolved(fram_error_log_t* log,
                           uint16_t code,
                           int32_t detail0,
                           int32_t detail1,
                           uint32_t epoch_sec,
                           uint16_t millis,
                           bool* logged_out)
{
  return FramErrorLogAppendInternal(log,
                                    code,
                                    detail0,
                                    detail1,
                                    kFramErrorLogEntryFlagResolved,
                                    epoch_sec,
                                    millis,
                                    logged_out,
                                    false);
}

esp_err_t
FramErrorLogClear(fram_error_log_t* log)
{
  if (log == NULL || !log->initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  log->header = FramErrorLogDefaultHeader();
  const esp_err_t result = FramErrorLogWriteHeaderCopies(log, &log->header);
  if (result == ESP_OK) {
    log->state = FRAM_ERRLOG_STATE_OK;
    log->region_blank = false;
    log->copy0_status.valid = true;
    log->copy0_status.reason = FRAM_ERRLOG_HDR_OK;
    log->copy1_status.valid = true;
    log->copy1_status.reason = FRAM_ERRLOG_HDR_OK;
  }
  return result;
}

esp_err_t
FramErrorLogDump(const fram_error_log_t* log, uint32_t max_entries)
{
  if (log == NULL || !log->initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (log->state == FRAM_ERRLOG_STATE_CORRUPT) {
    return ESP_ERR_INVALID_STATE;
  }
  const uint32_t count = log->header.count;
  if (count == 0) {
    return ESP_OK;
  }
  const uint32_t to_show =
    (max_entries == 0 || max_entries > count) ? count : max_entries;

  ESP_LOGI(kTag,
           "stored error log entries: count=%" PRIu32 " capacity=%" PRIu32
           " write_index=%" PRIu32,
           count,
           log->entry_capacity,
           log->header.write_index);

  for (uint32_t i = 0; i < to_show; ++i) {
    fram_error_log_entry_t entry = { 0 };
    bool crc_ok = false;
    const esp_err_t result = FramErrorLogReadEntry(log, i, &entry, &crc_ok);
    if (result != ESP_OK) {
      ESP_LOGW(kTag, "entry[%" PRIu32 "] read failed: %s", i,
               esp_err_to_name(result));
      continue;
    }
    char utc_buffer[32];
    FramErrorLogFormatUtc(entry.epoch_sec, utc_buffer, sizeof(utc_buffer));
    ESP_LOGI(kTag,
             "entry[%" PRIu32 "] time=%s.%03u code=%u flags=0x%04x d0=%" PRId32
             " d1=%" PRId32 " crc=%s",
             i,
             utc_buffer,
             (unsigned)entry.millis,
             (unsigned)entry.code,
             (unsigned)entry.flags,
             entry.detail0,
             entry.detail1,
             crc_ok ? "ok" : "bad");
  }
  return ESP_OK;
}

/**
 * @brief Execute FramErrorLogGetStatus.
 * @param log Parameter log.
 * @param out Parameter out.
 * @return Return the function result.
 */
esp_err_t
FramErrorLogGetStatus(const fram_error_log_t* log,
                      fram_error_log_status_t* out)
{
  if (log == NULL || out == NULL || !log->initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  out->state = log->state;
  out->copy0 = log->copy0_status;
  out->copy1 = log->copy1_status;
  out->region_blank = log->region_blank;
  return ESP_OK;
}

/**
 * @brief Execute FramErrorLogHeaderReasonToString.
 * @param reason Parameter reason.
 * @return Return the function result.
 */
const char*
FramErrorLogHeaderReasonToString(fram_errlog_header_reason_t reason)
{
  switch (reason) {
    case FRAM_ERRLOG_HDR_OK:
      return "ok";
    case FRAM_ERRLOG_HDR_BAD_MAGIC:
      return "bad_magic";
    case FRAM_ERRLOG_HDR_BAD_SCHEMA:
      return "bad_schema";
    case FRAM_ERRLOG_HDR_BAD_CRC:
      return "bad_crc";
    case FRAM_ERRLOG_HDR_IO_FAIL:
      return "io_fail";
    case FRAM_ERRLOG_HDR_SHORT_READ:
      return "short_read";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute FramErrorLogStateToString.
 * @param state Parameter state.
 * @return Return the function result.
 */
const char*
FramErrorLogStateToString(fram_errlog_state_t state)
{
  return FramErrorLogStateToStringInternal(state);
}
