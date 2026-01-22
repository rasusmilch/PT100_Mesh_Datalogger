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
    // Backoff for probing an absent/unresponsive DS3231 to avoid repeated I2C
    // errors/log spam.
    // Stored as FreeRTOS ticks (uint32) to avoid including FreeRTOS headers
    // here.
    uint32_t ds3231_next_probe_ticks;
    uint32_t ds3231_consecutive_failures;
  } time_sync_t;

  typedef struct
  {
    char last_server[64];
    int64_t last_attempt_epoch;
    esp_err_t last_result;
    int64_t last_success_epoch;
  } time_sntp_status_t;

  /**
   * @brief Execute TimeSyncInit.
   * @param time_sync Parameter time_sync.
   * @param i2c_bus Parameter i2c_bus.
   * @param ds3231_addr Parameter ds3231_addr.
   * @return Return the function result.
   */
  esp_err_t TimeSyncInit(time_sync_t* time_sync,
                         i2c_bus_t* i2c_bus,
                         uint8_t ds3231_addr);

  /**
   * @brief Execute TimeSyncDeinit.
   * @param time_sync Parameter time_sync.
   * @return Return the function result.
   */
  esp_err_t TimeSyncDeinit(time_sync_t* time_sync);

  // If DS3231 has a plausible time, set system clock from RTC.
  /**
   * @brief Execute TimeSyncSetSystemFromRtc.
   * @param time_sync Parameter time_sync.
   * @return Return the function result.
   */
  esp_err_t TimeSyncSetSystemFromRtc(time_sync_t* time_sync);

  // Write system clock (UTC) back to DS3231.
  /**
   * @brief Execute TimeSyncSetRtcFromSystem.
   * @param time_sync Parameter time_sync.
   * @return Return the function result.
   */
  esp_err_t TimeSyncSetRtcFromSystem(time_sync_t* time_sync);

  /**
   * @brief Resync the system time from the DS3231 RTC if the delta exceeds a
   *        small deadband.
   * @param time_sync Time sync context for RTC access.
   * @param delta_seconds_out Optional output for rtc_epoch - system_epoch.
   * @param jumped_back_out Optional output set true when a backward time step
   *        was applied.
   * @return ESP_OK on success; error code on RTC read or time conversion
   *         failure.
   */
  esp_err_t TimeSyncResyncSystemFromRtc(time_sync_t* time_sync,
                                        int64_t* delta_seconds_out,
                                        bool* jumped_back_out);

  // Root-only: start SNTP and block until time is synced (or timeout).
  /**
   * @brief Execute TimeSyncStartSntpAndWait.
   * @param sntp_server Parameter sntp_server.
   * @param timeout_ms Parameter timeout_ms.
   * @return Return the function result.
   */
  esp_err_t TimeSyncStartSntpAndWait(const char* sntp_server, int timeout_ms);

  // Set system time from an epoch value (UTC seconds). Optionally updates
  // DS3231.
    /**
   * @brief Set the system clock to a specified UTC epoch time and optionally update the DS3231.
   * @param epoch_seconds UTC epoch seconds to set as the system time.
   * @param update_rtc If true and time_sync is non-NULL, write the system time to the DS3231 RTC.
   * @param time_sync Time sync context used for RTC access when update_rtc is true.
   * @return ESP_OK on success.
   */
  esp_err_t TimeSyncSetSystemEpoch(int64_t epoch_seconds,
                                   bool update_rtc,
                                   time_sync_t* time_sync);

  // Check if system clock is plausibly set (year >= 2023).
  /**
   * @brief Execute TimeSyncIsSystemTimeValid.
   * @return Return the function result.
   */
  bool TimeSyncIsSystemTimeValid(void);

  // Get current epoch seconds and milliseconds.
  /**
   * @brief Execute TimeSyncGetNow.
   * @param epoch_seconds_out Parameter epoch_seconds_out.
   * @param millis_out Parameter millis_out.
   */
  void TimeSyncGetNow(int64_t* epoch_seconds_out, int32_t* millis_out);

  /**
   * @brief Execute TimeSyncGetSntpStatus.
   * @param out Parameter out.
   */
  void TimeSyncGetSntpStatus(time_sntp_status_t* out);

  // Returns the last epoch (UTC) written to the RTC by this boot instance (0 if
  // never).
  int64_t TimeSyncGetLastRtcSetEpoch(void);

  // Read the RTC time and return it as UTC epoch seconds.
  esp_err_t TimeSyncReadRtcEpoch(const time_sync_t* time_sync,
                                 int64_t* out_epoch_seconds);

  // Parse an ISO-like local time string into struct tm (local).
  // Accepts "YYYY-MM-DD HH:MM:SS" or "YYYY-MM-DDTHH:MM:SS".
  /**
   * @brief Execute TimeParseLocalIso.
   * @param iso Parameter iso.
   * @param out_tm_local Parameter out_tm_local.
   * @return Return the function result.
   */
  esp_err_t TimeParseLocalIso(const char* iso, struct tm* out_tm_local);

  // Convert a local struct tm to UTC epoch seconds using current TZ/DST.
  // Returns ESP_ERR_INVALID_STATE for invalid (gap) times and
  // ESP_ERR_NOT_SUPPORTED if the time is ambiguous and tm_isdst is unset.
  /**
   * @brief Execute TimeLocalTmToEpochUtc.
   * @param tm_local Parameter tm_local.
   * @param out_epoch_utc Parameter out_epoch_utc.
   * @param out_ambiguous Parameter out_ambiguous.
   * @return Return the function result.
   */
  esp_err_t TimeLocalTmToEpochUtc(const struct tm* tm_local,
                                  time_t* out_epoch_utc,
                                  bool* out_ambiguous);

  // Read DS3231 registers starting at the given address.
  /**
   * @brief Execute TimeSyncReadRtcRegisters.
   * @param time_sync Parameter time_sync.
   * @param start_reg Parameter start_reg.
   * @param data_out Parameter data_out.
   * @param length Parameter length.
   * @return Return the function result.
   */
  esp_err_t TimeSyncReadRtcRegisters(const time_sync_t* time_sync,
                                     uint8_t start_reg,
                                     uint8_t* data_out,
                                     size_t length);

  // Read DS3231 time into a struct tm (UTC).
  /**
   * @brief Execute TimeSyncReadRtcTime.
   * @param time_sync Parameter time_sync.
   * @param time_out Parameter time_out.
   * @return Return the function result.
   */
  esp_err_t TimeSyncReadRtcTime(const time_sync_t* time_sync,
                                struct tm* time_out);

  // Write a struct tm (UTC) into the DS3231.
  /**
   * @brief Execute TimeSyncWriteRtcTime.
   * @param time_sync Parameter time_sync.
   * @param time_value Parameter time_value.
   * @return Return the function result.
   */
  esp_err_t TimeSyncWriteRtcTime(const time_sync_t* time_sync,
                                 const struct tm* time_value);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_TIME_SYNC_H_
