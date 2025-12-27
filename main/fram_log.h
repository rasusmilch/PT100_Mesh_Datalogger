#ifndef PT100_LOGGER_FRAM_LOG_H_
#define PT100_LOGGER_FRAM_LOG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "fram_io.h"
#include "log_record.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    fram_io_t io;
    uint32_t fram_size_bytes;
    uint32_t record_region_offset;
    uint32_t capacity_records;

    uint32_t header_generation;
    uint8_t header_copy_index;
    uint32_t write_index;
    uint32_t read_index;
    uint32_t record_count;
    uint32_t next_sequence;
    uint64_t next_record_id;
    uint64_t overrun_records_total;
    uint32_t overrun_events_total;

    uint32_t records_since_header_persist;
    bool saw_corruption;
    bool mounted;
  } fram_log_t;

  typedef struct
  {
    uint32_t capacity_records;
    uint32_t record_size_bytes;
    uint32_t flush_watermark_records;
    uint32_t buffered_count;
    uint32_t write_index_abs;
    uint32_t read_index_abs;
    uint32_t next_sequence;
    uint64_t next_record_id;
    bool mounted;
    bool full;
  } fram_log_status_t;

  esp_err_t FramLogInit(fram_log_t* log,
                        fram_io_t io,
                        uint32_t fram_size_bytes);

  uint32_t FramLogNextSequence(const fram_log_t* log);
  uint64_t FramLogNextRecordId(const fram_log_t* log);
  size_t FramLogGetCapacityRecords(const fram_log_t* log);
  uint32_t FramLogGetBufferedRecords(const fram_log_t* log);
  size_t FramLogGetCountRecords(const fram_log_t* log);
  uint64_t FramLogGetOverrunRecordsTotal(const fram_log_t* log);
  bool FramLogIsOverwriting(const fram_log_t* log);
  esp_err_t FramLogGetStatus(const fram_log_t* log,
                             fram_log_status_t* out_status);

  // Assign the next record_id/sequence values to the record and advance the
  // persistent counters, guaranteeing monotonicity across reboots.
  esp_err_t FramLogAssignRecordIds(fram_log_t* log, log_record_t* record);

  // Append record (writes to FRAM). Overwrites oldest when buffer is full.
  esp_err_t FramLogAppend(fram_log_t* log, const log_record_t* record);

  // Peek the oldest record into `record_out` without removing it.
  // Returns ESP_ERR_NOT_FOUND if empty. May return ESP_ERR_INVALID_RESPONSE
  // if CRC/magic validation fails, but `record_out` is still populated with raw
  // bytes.
  esp_err_t FramLogPeekOldest(const fram_log_t* log, log_record_t* record_out);

  // Peek with an offset from the oldest record (0=oldest). Returns
  // ESP_ERR_NOT_FOUND if offset exceeds buffered records.
  esp_err_t FramLogPeekOffset(const fram_log_t* log,
                              uint32_t offset,
                              log_record_t* record_out);

  // Discard the oldest record without reading it.
  // Returns ESP_ERR_NOT_FOUND if empty.
  esp_err_t FramLogDiscardOldest(fram_log_t* log);

  // Pop the oldest record into `record_out`. Returns ESP_ERR_NOT_FOUND if
  // empty.
  esp_err_t FramLogPopOldest(fram_log_t* log, log_record_t* record_out);

  // Force persist header to FRAM immediately.
  esp_err_t FramLogPersistHeader(fram_log_t* log);

  // Consume records while their record_id <= max_record_id_inclusive.
  // Returns ESP_OK with consumed_out set, even if zero records were dropped.
  esp_err_t FramLogConsumeUpToRecordId(fram_log_t* log,
                                       uint64_t max_record_id_inclusive,
                                       uint32_t* consumed_out);

  // Explicitly skip one record when corruption is detected, logging the
  // recovery attempt.
  esp_err_t FramLogSkipCorruptedRecord(fram_log_t* log);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_LOG_H_
