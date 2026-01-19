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

  typedef struct
  {
    fram_io_t io;
    uint32_t base_addr;
    uint32_t region_bytes;
    uint32_t entry_capacity;
    bool initialized;
    fram_error_log_header_t header;
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

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_FRAM_ERROR_LOG_H_
