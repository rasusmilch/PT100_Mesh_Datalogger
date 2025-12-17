#ifndef PT100_LOGGER_TIME_SYNC_H_
#define PT100_LOGGER_TIME_SYNC_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    i2c_port_t port;
    uint8_t ds3231_addr;
    bool is_i2c_initialized;
  } time_sync_t;

  esp_err_t TimeSyncInit(time_sync_t* time_sync,
                         i2c_port_t port,
                         int sda_gpio,
                         int scl_gpio,
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

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_TIME_SYNC_H_
