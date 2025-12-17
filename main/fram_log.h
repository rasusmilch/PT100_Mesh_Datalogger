#ifndef PT100_LOGGER_FRAM_LOG_H_
#define PT100_LOGGER_FRAM_LOG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "fram_spi.h"
#include "log_record.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    fram_spi_t* fram;
    uint32_t fram_size_bytes;
    uint32_t record_region_offset;
    uint32_t capacity_records;

    uint32_t header_sequence;
    uint32_t write_index;
    uint32_t read_index;
    uint32_t record_count;

    uint32_t records_since_header_persist;
  } fram_log_t;

  esp_err_t FramLogInit(fram_log_t* log,
                        fram_spi_t* fram,
                        uint32_t fram_size_bytes);

  uint32_t FramLogGetCapacityRecords(const fram_log_t* log);
  uint32_t FramLogGetBufferedRecords(const fram_log_t* log);

  // Append record (writes to FRAM). May drop the oldest record if buffer is
  // full.
  esp_err_t FramLogAppend(fram_log_t* log, const log_record_t* record);

  // Peek the oldest record into `record_out` without removing it.
  // Returns ESP_ERR_NOT_FOUND if empty. May return ESP_ERR_INVALID_RESPONSE
  // if CRC/magic validation fails, but `record_out` is still populated with raw
  // bytes.
  esp_err_t FramLogPeekOldest(const fram_log_t* log, log_record_t* record_out);

  // Discard the oldest record without reading it.
  // Returns ESP_ERR_NOT_FOUND if empty.
  esp_err_t FramLogDiscardOldest(fram_log_t* log);

  // Pop the oldest record into `record_out`. Returns ESP_ERR_NOT_FOUND if
  // empty.
  esp_err_t FramLogPopOldest(fram_log_t* log, log_record_t* record_out);

  // Force persist header to FRAM immediately.
  esp_err_t FramLogPersistHeader(fram_log_t* log);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_LOG_H_
