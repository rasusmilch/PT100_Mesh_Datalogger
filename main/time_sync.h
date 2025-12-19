#ifndef PT100_LOGGER_TIME_SYNC_H_
#define PT100_LOGGER_TIME_SYNC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"
#include "i2c_bus.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    i2c_bus_t* bus;
    i2c_master_dev_handle_t ds3231_device;
    uint8_t ds3231_addr;
    bool is_ds3231_ready;
  } time_sync_t;

  esp_err_t TimeSyncInit(time_sync_t* time_sync,
                         i2c_bus_t* i2c_bus,
                         uint8_t ds3231_addr);

  // If DS3231 has a plausible time, set system clock from RTC.
  esp_err_t TimeSyncSetSystemFromRtc(const time_sync_t* time_sync);

  // Write system clock (UTC) back to DS3231.
  esp_err_t TimeSyncSetRtcFromSystem(const time_sync_t* time_sync);

  // Root-only: start SNTP and block until time is synced (or timeout).
  esp_err_t TimeSyncStartSntpAndWait(const char* sntp_server, int timeout_ms);

  // Set system time from an epoch value (UTC seconds). Optionally updates
  // DS3231.
  esp_err_t TimeSyncSetSystemEpoch(int64_t epoch_seconds,
                                   bool update_rtc,
                                   const time_sync_t* time_sync);

  // Check if system clock is plausibly set (year >= 2023).
  bool TimeSyncIsSystemTimeValid(void);

  // Get current epoch seconds and milliseconds.
  void TimeSyncGetNow(int64_t* epoch_seconds_out, int32_t* millis_out);

  // Read DS3231 registers starting at the given address.
  esp_err_t TimeSyncReadRtcRegisters(const time_sync_t* time_sync,
                                     uint8_t start_reg,
                                     uint8_t* data_out,
                                     size_t length);

  // Read DS3231 time into a struct tm (UTC).
  esp_err_t TimeSyncReadRtcTime(const time_sync_t* time_sync,
                                struct tm* time_out);

  // Write a struct tm (UTC) into the DS3231.
  esp_err_t TimeSyncWriteRtcTime(const time_sync_t* time_sync,
                                 const struct tm* time_value);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_TIME_SYNC_H_
