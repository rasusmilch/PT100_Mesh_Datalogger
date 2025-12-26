
#include "time_sync.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "esp_log.h"
#include "i2c_bus.h"

#if __has_include("esp_netif_sntp.h")
#include "esp_netif_sntp.h"
#define APP_USE_ESP_NETIF_SNTP 1
#else
#include "lwip/apps/sntp.h"
#define APP_USE_ESP_NETIF_SNTP 0
#endif
#include "esp_system.h"

static const char* kTag = "time_sync";

static uint8_t
BcdToBinary(uint8_t bcd)
{
  return (uint8_t)(((bcd >> 4) * 10) + (bcd & 0x0F));
}

static uint8_t
BinaryToBcd(uint8_t value)
{
  return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static bool
YearLooksValid(int year_since_1900)
{
  const int year = year_since_1900 + 1900;
  return year >= 2023 && year <= 2100;
}

static time_t
UtcTmToEpochSeconds(struct tm* tm_utc)
{
  if (tm_utc == NULL) {
    return (time_t)-1;
  }

  // DS3231 values are UTC in this application (see TimeSyncSetRtcFromSystem).
  //
  // mktime() interprets its input as local time under the currently configured TZ.
  // When a non-UTC TZ is active (e.g. CST6CDT,...), using mktime() directly will
  // incorrectly apply the TZ offset and skew the epoch (typically by 6 hours).
  //
  // Convert in a TZ-agnostic way by temporarily forcing TZ=UTC0.
  const char* previous_tz = getenv("TZ");
  char previous_tz_copy[80] = { 0 };
  if (previous_tz != NULL) {
    strncpy(previous_tz_copy, previous_tz, sizeof(previous_tz_copy) - 1);
  }

  setenv("TZ", "UTC0", 1);
  tzset();
  const time_t epoch_seconds = mktime(tm_utc);

  if (previous_tz == NULL) {
    unsetenv("TZ");
  } else {
    setenv("TZ", previous_tz_copy, 1);
  }
  tzset();
  return epoch_seconds;
}

esp_err_t
TimeSyncInit(time_sync_t* time_sync,
             i2c_bus_t* i2c_bus,
             uint8_t ds3231_addr)
{
  if (time_sync == NULL || i2c_bus == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(time_sync, 0, sizeof(*time_sync));
  time_sync->bus = i2c_bus;
  time_sync->ds3231_addr = ds3231_addr;

  esp_err_t result = I2cBusAddDevice(
    i2c_bus, ds3231_addr, i2c_bus->frequency_hz, &time_sync->ds3231_device);
  if (result != ESP_OK) {
    return result;
  }

  time_sync->is_ds3231_ready = true;
  return ESP_OK;
}

static esp_err_t
Ds3231ReadTime(const time_sync_t* time_sync, struct tm* time_out)
{
  uint8_t regs[7] = { 0 };
  esp_err_t result = I2cBusReadRegister(
    time_sync->ds3231_device, 0x00, regs, sizeof(regs));
  if (result != ESP_OK) {
    return result;
  }

  // DS3231 registers are BCD.
  memset(time_out, 0, sizeof(*time_out));
  time_out->tm_sec = BcdToBinary(regs[0] & 0x7F);
  time_out->tm_min = BcdToBinary(regs[1] & 0x7F);
  time_out->tm_hour = BcdToBinary(regs[2] & 0x3F); // 24-hour mode expected.
  time_out->tm_mday = BcdToBinary(regs[4] & 0x3F);
  time_out->tm_mon = (int)BcdToBinary(regs[5] & 0x1F) - 1;
  time_out->tm_year =
    (int)BcdToBinary(regs[6]) +
    100; // years since 1900; DS3231 stores 00..99 (assume 2000+)
  return ESP_OK;
}

static esp_err_t
Ds3231WriteTime(const time_sync_t* time_sync, const struct tm* time_value)
{
  uint8_t regs[7] = { 0 };
  regs[0] = BinaryToBcd((uint8_t)time_value->tm_sec);
  regs[1] = BinaryToBcd((uint8_t)time_value->tm_min);
  regs[2] = BinaryToBcd((uint8_t)time_value->tm_hour); // 24-hour
  regs[3] = BinaryToBcd(
    (uint8_t)((time_value->tm_wday == 0) ? 7 : time_value->tm_wday)); // 1..7
  regs[4] = BinaryToBcd((uint8_t)time_value->tm_mday);
  regs[5] = BinaryToBcd((uint8_t)(time_value->tm_mon + 1));
  regs[6] = BinaryToBcd((uint8_t)(time_value->tm_year - 100)); // store 00..99
  return I2cBusWriteRegister(
    time_sync->ds3231_device, 0x00, regs, sizeof(regs));
}

esp_err_t
TimeSyncSetSystemFromRtc(const time_sync_t* time_sync)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  struct tm rtc_time;
  esp_err_t result = Ds3231ReadTime(time_sync, &rtc_time);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "DS3231 read failed: %s", esp_err_to_name(result));
    return result;
  }
  if (!YearLooksValid(rtc_time.tm_year)) {
    ESP_LOGW(kTag, "RTC time not plausible (year=%d)", rtc_time.tm_year + 1900);
    return ESP_ERR_INVALID_STATE;
  }

  // Convert the DS3231's UTC calendar fields to epoch seconds.
  rtc_time.tm_isdst = 0;
  const time_t epoch_seconds = UtcTmToEpochSeconds(&rtc_time);
  if (epoch_seconds == (time_t)-1) {
    return ESP_ERR_INVALID_STATE;
  }
  struct timeval tv = {
    .tv_sec = epoch_seconds,
    .tv_usec = 0,
  };
  settimeofday(&tv, NULL);

  ESP_LOGI(kTag,
           "System time set from RTC: %04d-%02d-%02d %02d:%02d:%02dZ",
           rtc_time.tm_year + 1900,
           rtc_time.tm_mon + 1,
           rtc_time.tm_mday,
           rtc_time.tm_hour,
           rtc_time.tm_min,
           rtc_time.tm_sec);
  return ESP_OK;
}

esp_err_t
TimeSyncSetRtcFromSystem(const time_sync_t* time_sync)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  time_t now_seconds = time(NULL);
  struct tm now_utc;
  gmtime_r(&now_seconds, &now_utc);

  if (!YearLooksValid(now_utc.tm_year)) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = Ds3231WriteTime(time_sync, &now_utc);
  if (result == ESP_OK) {
    ESP_LOGI(kTag, "RTC updated from system time");
  }
  return result;
}

bool
TimeSyncIsSystemTimeValid(void)
{
  time_t now_seconds = time(NULL);
  struct tm now_utc;
  gmtime_r(&now_seconds, &now_utc);
  return YearLooksValid(now_utc.tm_year);
}

void
TimeSyncGetNow(int64_t* epoch_seconds_out, int32_t* millis_out)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (epoch_seconds_out != NULL) {
    *epoch_seconds_out = (int64_t)tv.tv_sec;
  }
  if (millis_out != NULL) {
    *millis_out = (int32_t)(tv.tv_usec / 1000);
  }
}

static bool
LocalTmFieldsMatch(const struct tm* left, const struct tm* right)
{
  return left != NULL && right != NULL && left->tm_year == right->tm_year &&
         left->tm_mon == right->tm_mon && left->tm_mday == right->tm_mday &&
         left->tm_hour == right->tm_hour && left->tm_min == right->tm_min &&
         left->tm_sec == right->tm_sec;
}

esp_err_t
TimeParseLocalIso(const char* iso, struct tm* out_tm_local)
{
  if (iso == NULL || out_tm_local == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  while (*iso != '\0' && isspace((unsigned char)*iso)) {
    ++iso;
  }
  if (*iso == '\0') {
    return ESP_ERR_INVALID_ARG;
  }

  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  int consumed = 0;
  int matched = sscanf(
    iso, "%d-%d-%d %d:%d:%d %n", &year, &month, &day, &hour, &minute, &second, &consumed);
  if (matched != 6) {
    consumed = 0;
    matched = sscanf(
      iso, "%d-%d-%dT%d:%d:%d %n", &year, &month, &day, &hour, &minute, &second, &consumed);
  }
  if (matched != 6 || consumed <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  const char* tail = iso + consumed;
  while (*tail != '\0' && isspace((unsigned char)*tail)) {
    ++tail;
  }
  if (*tail != '\0') {
    return ESP_ERR_INVALID_ARG;
  }

  if (year < 1970 || year > 2100 || month < 1 || month > 12 || day < 1 ||
      day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
      second < 0 || second > 59) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(out_tm_local, 0, sizeof(*out_tm_local));
  out_tm_local->tm_year = year - 1900;
  out_tm_local->tm_mon = month - 1;
  out_tm_local->tm_mday = day;
  out_tm_local->tm_hour = hour;
  out_tm_local->tm_min = minute;
  out_tm_local->tm_sec = second;
  out_tm_local->tm_isdst = -1;
  return ESP_OK;
}

esp_err_t
TimeLocalTmToEpochUtc(const struct tm* tm_local,
                      time_t* out_epoch_utc,
                      bool* out_ambiguous)
{
  if (tm_local == NULL || out_epoch_utc == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (out_ambiguous != NULL) {
    *out_ambiguous = false;
  }

  tzset();

  const struct tm input = *tm_local;

  if (input.tm_isdst == 0 || input.tm_isdst == 1) {
    struct tm normalized = input;
    const time_t epoch = mktime(&normalized);
    if (epoch == (time_t)-1) {
      return ESP_ERR_INVALID_STATE;
    }
    struct tm round_trip;
    localtime_r(&epoch, &round_trip);
    if (!LocalTmFieldsMatch(&round_trip, &input)) {
      return ESP_ERR_INVALID_STATE;
    }
    *out_epoch_utc = epoch;
    return ESP_OK;
  }

  struct tm standard = input;
  standard.tm_isdst = 0;
  const time_t epoch_std = mktime(&standard);
  bool match_std = false;
  if (epoch_std != (time_t)-1) {
    struct tm round_std;
    localtime_r(&epoch_std, &round_std);
    match_std = LocalTmFieldsMatch(&round_std, &input);
  }

  struct tm daylight = input;
  daylight.tm_isdst = 1;
  const time_t epoch_dst = mktime(&daylight);
  bool match_dst = false;
  if (epoch_dst != (time_t)-1) {
    struct tm round_dst;
    localtime_r(&epoch_dst, &round_dst);
    match_dst = LocalTmFieldsMatch(&round_dst, &input);
  }

  if (!match_std && !match_dst) {
    return ESP_ERR_INVALID_STATE;
  }
  if (match_std && match_dst) {
    if (out_ambiguous != NULL) {
      *out_ambiguous = true;
    }
    return ESP_ERR_NOT_SUPPORTED;
  }

  *out_epoch_utc = match_std ? epoch_std : epoch_dst;
  return ESP_OK;
}

esp_err_t
TimeSyncStartSntpAndWait(const char* sntp_server, int timeout_ms)
{
  if (sntp_server == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

#if APP_USE_ESP_NETIF_SNTP
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(sntp_server);
  config.smooth_sync = false;
  esp_netif_sntp_init(&config);

  esp_err_t wait_result = ESP_OK;
  const TickType_t start_ticks = xTaskGetTickCount();
  while (true) {
    // Prefer sync_wait when available, but poll to stay robust across versions.
#ifdef esp_netif_sntp_sync_wait
    wait_result = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(250));
    if (wait_result == ESP_OK) {
      break;
    }
#else
    // No sync_wait symbol; poll for valid time instead.
    wait_result = ESP_ERR_TIMEOUT;
#endif
    if (TimeSyncIsSystemTimeValid()) {
      wait_result = ESP_OK;
      break;
    }
    const int elapsed_ms =
      (int)pdTICKS_TO_MS(xTaskGetTickCount() - start_ticks);
    if (elapsed_ms >= timeout_ms) {
      wait_result = ESP_ERR_TIMEOUT;
      break;
    }
  }

  if (wait_result != ESP_OK) {
    ESP_LOGW(kTag, "SNTP timeout/failure: %s", esp_err_to_name(wait_result));
    return wait_result;
  }
#else
  // Legacy SNTP API (lwIP-based).
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, sntp_server);
  sntp_init();

  const TickType_t start_ticks = xTaskGetTickCount();
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) {
    vTaskDelay(pdMS_TO_TICKS(200));
    const int elapsed_ms =
      (int)pdTICKS_TO_MS(xTaskGetTickCount() - start_ticks);
    if (elapsed_ms >= timeout_ms) {
      ESP_LOGW(kTag, "SNTP timeout after %dms", elapsed_ms);
      return ESP_ERR_TIMEOUT;
    }
  }
#endif

  if (!TimeSyncIsSystemTimeValid()) {
    ESP_LOGW(kTag, "SNTP completed, but system time still not plausible");
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(kTag, "SNTP synced");
  return ESP_OK;
}

esp_err_t
TimeSyncSetSystemEpoch(int64_t epoch_seconds,
                       bool update_rtc,
                       const time_sync_t* time_sync)
{
  struct timeval tv = {
    .tv_sec = (time_t)epoch_seconds,
    .tv_usec = 0,
  };
  settimeofday(&tv, NULL);

  if (update_rtc && time_sync != NULL) {
    (void)TimeSyncSetRtcFromSystem(time_sync);
  }
  return ESP_OK;
}

esp_err_t
TimeSyncReadRtcRegisters(const time_sync_t* time_sync,
                         uint8_t start_reg,
                         uint8_t* data_out,
                         size_t length)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  return I2cBusReadRegister(time_sync->ds3231_device, start_reg, data_out, length);
}

esp_err_t
TimeSyncReadRtcTime(const time_sync_t* time_sync, struct tm* time_out)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  return Ds3231ReadTime(time_sync, time_out);
}

esp_err_t
TimeSyncWriteRtcTime(const time_sync_t* time_sync, const struct tm* time_value)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  return Ds3231WriteTime(time_sync, time_value);
}
