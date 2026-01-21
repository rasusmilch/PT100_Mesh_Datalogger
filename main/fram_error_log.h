#ifndef PT100_LOGGER_FRAM_ERROR_LOG_H_
#define PT100_LOGGER_FRAM_ERROR_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "fram_io.h"

#ifdef __cplusplus
extern "C" {
#endif

  enum
  {
    kFramErrorLogEntryFlagActive = 1 << 0,
    kFramErrorLogEntryFlagResolved = 1 << 1,
  };

  typedef struct __attribute__((packed))
  {
    uint32_t epoch_sec;
    uint16_t millis;
    uint16_t code;
    int32_t detail0;
    int32_t detail1;
    uint16_t flags;
    uint16_t crc16;
  } fram_error_log_entry_t;

  typedef struct
  {
    uint32_t write_index;
    uint32_t count;
    uint32_t capacity;
    uint32_t active_bitmap_low;
    uint32_t active_bitmap_high;
  } fram_error_log_stats_t;

  typedef struct __attribute__((packed))
  {
    uint32_t magic;
    uint16_t schema_ver;
    uint16_t crc16;
    uint32_t write_index;
    uint32_t count;
    uint32_t active_bitmap_low;
    uint32_t active_bitmap_high;
    uint32_t reserved;
  } fram_error_log_header_t;

  typedef enum
  {
    FRAM_ERRLOG_STATE_UNINIT = 0,
    FRAM_ERRLOG_STATE_OK = 1,
    FRAM_ERRLOG_STATE_BLANK_INITED = 2,
    FRAM_ERRLOG_STATE_CORRUPT = 3,
  } fram_errlog_state_t;

  typedef enum
  {
    FRAM_ERRLOG_HDR_OK = 0,
    FRAM_ERRLOG_HDR_BAD_MAGIC = 1,
    FRAM_ERRLOG_HDR_BAD_SCHEMA = 2,
    FRAM_ERRLOG_HDR_BAD_CRC = 3,
    FRAM_ERRLOG_HDR_IO_FAIL = 4,
    FRAM_ERRLOG_HDR_SHORT_READ = 5,
  } fram_errlog_header_reason_t;

  typedef struct
  {
    bool valid;
    fram_errlog_header_reason_t reason;
  } fram_errlog_header_status_t;

  typedef struct
  {
    fram_errlog_state_t state;
    fram_errlog_header_status_t copy0;
    fram_errlog_header_status_t copy1;
    bool region_blank;
  } fram_error_log_status_t;

  typedef struct
  {
    fram_io_t io;
    uint32_t base_addr;
    uint32_t region_bytes;
    uint32_t entry_capacity;
    bool initialized;
    fram_error_log_header_t header;
    fram_errlog_state_t state;
    fram_errlog_header_status_t copy0_status;
    fram_errlog_header_status_t copy1_status;
    bool region_blank;
  } fram_error_log_t;

  esp_err_t FramErrorLogInit(fram_error_log_t* log,
                             fram_io_t io,
                             uint32_t base_addr,
                             uint32_t region_bytes);

  esp_err_t FramErrorLogGetStats(const fram_error_log_t* log,
                                 fram_error_log_stats_t* out);

  esp_err_t FramErrorLogReadEntry(const fram_error_log_t* log,
                                  uint32_t entry_index,
                                  fram_error_log_entry_t* out,
                                  bool* crc_ok_out);

  esp_err_t FramErrorLogAppendActive(fram_error_log_t* log,
                                     uint16_t code,
                                     int32_t detail0,
                                     int32_t detail1,
                                     uint32_t epoch_sec,
                                     uint16_t millis,
                                     bool* logged_out);

  esp_err_t FramErrorLogAppendResolved(fram_error_log_t* log,
                                       uint16_t code,
                                       int32_t detail0,
                                       int32_t detail1,
                                       uint32_t epoch_sec,
                                       uint16_t millis,
                                       bool* logged_out);

  esp_err_t FramErrorLogClear(fram_error_log_t* log);

  esp_err_t FramErrorLogDump(const fram_error_log_t* log, uint32_t max_entries);

  bool FramErrorLogGetCodeMask(uint16_t code, uint64_t* mask_out);

  /**
   * @brief Get errlog status from the last initialization pass.
   * @param log Parameter log.
   * @param out Parameter out.
   * @return Return the function result.
   */
  esp_err_t FramErrorLogGetStatus(const fram_error_log_t* log,
                                  fram_error_log_status_t* out);

  /**
   * @brief Convert errlog header validation reason to string.
   * @param reason Parameter reason.
   * @return Return the function result.
   */
  const char* FramErrorLogHeaderReasonToString(
    fram_errlog_header_reason_t reason);

  /**
   * @brief Convert errlog state to string.
   * @param state Parameter state.
   * @return Return the function result.
   */
  const char* FramErrorLogStateToString(fram_errlog_state_t state);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_ERROR_LOG_H_
