
#include "time_sync.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
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
static char s_last_sntp_server[64] = "";
static int64_t s_last_sntp_attempt_epoch = 0;
static esp_err_t s_last_sntp_result = ESP_ERR_INVALID_STATE;
static int64_t s_last_sntp_success_epoch = 0;
static int64_t s_last_rtc_set_epoch = 0;
static bool s_esp_netif_sntp_initialized = false;

/**
 * @brief Execute BcdToBinary.
 * @param bcd Parameter bcd.
 * @return Return the function result.
 */
static uint8_t
BcdToBinary(uint8_t bcd)
{
  return (uint8_t)(((bcd >> 4) * 10) + (bcd & 0x0F));
}

/**
 * @brief Execute BinaryToBcd.
 * @param value Parameter value.
 * @return Return the function result.
 */
static uint8_t
BinaryToBcd(uint8_t value)
{
  return (uint8_t)(((value / 10) << 4) | (value % 10));
}

/**
 * @brief Execute YearLooksValid.
 * @param year_since_1900 Parameter year_since_1900.
 * @return Return the function result.
 */
static bool
YearLooksValid(int year_since_1900)
{
  const int year = year_since_1900 + 1900;
  return year >= 2023 && year <= 2100;
}

/**
 * @brief Execute UtcTmToEpochSeconds.
 * @param tm_utc Parameter tm_utc.
 * @return Return the function result.
 */
static int64_t
DaysFromCivil(int year, unsigned month, unsigned day)
{
  // Howard Hinnant's algorithm: days since 1970-01-01.
  // month: 1-12, day: 1-31.
  year -= (month <= 2);
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400); // [0, 399]
  const unsigned doy =
    (153U * (month + (month > 2 ? (unsigned)-3 : 9)) + 2U) / 5U + day - 1U;
  const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy; // [0, 146096]
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

int64_t
UtcTmToEpochSeconds(const struct tm* utc_tm)
{
  // Convert a UTC tm to epoch seconds without touching the process TZ.
  // (Avoids setenv()/tzset(), which are global and can be risky once multiple
  // tasks are running.)
  if (utc_tm == NULL) {
    return 0;
  }

  const int year = utc_tm->tm_year + 1900;
  const unsigned month = (unsigned)(utc_tm->tm_mon + 1);
  const unsigned day = (unsigned)utc_tm->tm_mday;

  const int64_t days = DaysFromCivil(year, month, day);
  const int64_t seconds = days * 86400LL + (int64_t)utc_tm->tm_hour * 3600LL +
                          (int64_t)utc_tm->tm_min * 60LL +
                          (int64_t)utc_tm->tm_sec;

  return seconds;
}

/**
 * @brief Execute TimeSyncInit.
 * @param time_sync Parameter time_sync.
 * @param i2c_bus Parameter i2c_bus.
 * @param ds3231_addr Parameter ds3231_addr.
 * @return Return the function result.
 */
esp_err_t
TimeSyncInit(time_sync_t* time_sync, i2c_bus_t* i2c_bus, uint8_t ds3231_addr)
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

/**
 * @brief Execute Ds3231ReadTime.
 * @param time_sync Parameter time_sync.
 * @param time_out Parameter time_out.
 * @return Return the function result.
 */
static esp_err_t
Ds3231ReadTime(const time_sync_t* time_sync, struct tm* time_out)
{
  uint8_t regs[7] = { 0 };
  esp_err_t result =
    I2cBusReadRegister(time_sync->ds3231_device, 0x00, regs, sizeof(regs));
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

/**
 * @brief Execute Ds3231WriteTime.
 * @param time_sync Parameter time_sync.
 * @param time_value Parameter time_value.
 * @return Return the function result.
 */
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

/**
 * @brief Execute TimeSyncSetSystemFromRtc.
 * @param time_sync Parameter time_sync.
 * @return Return the function result.
 */
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

/**
 * @brief Execute TimeSyncSetRtcFromSystem.
 * @param time_sync Parameter time_sync.
 * @return Return the function result.
 */
esp_err_t
TimeSyncSetRtcFromSystem(const time_sync_t* time_sync)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  // The DS3231 is stored as UTC in this firmware. Local TZ/DST is applied only
  // when rendering timestamps.
  time_t now_seconds = time(NULL);
  struct tm now_utc;
  gmtime_r(&now_seconds, &now_utc);

  if (!YearLooksValid(now_utc.tm_year)) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = Ds3231WriteTime(time_sync, &now_utc);
  if (result == ESP_OK) {
    s_last_rtc_set_epoch = (int64_t)now_seconds;
    ESP_LOGI(kTag, "RTC updated from system time");
  }
  return result;
}

/**
 * @brief Execute TimeSyncIsSystemTimeValid.
 * @return Return the function result.
 */
bool
TimeSyncIsSystemTimeValid(void)
{
  time_t now_seconds = time(NULL);
  struct tm now_utc;
  gmtime_r(&now_seconds, &now_utc);
  return YearLooksValid(now_utc.tm_year);
}

/**
 * @brief Execute TimeSyncGetNow.
 * @param epoch_seconds_out Parameter epoch_seconds_out.
 * @param millis_out Parameter millis_out.
 */
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

/**
 * @brief Execute TimeSyncGetSntpStatus.
 * @param out Parameter out.
 */
void
TimeSyncGetSntpStatus(time_sntp_status_t* out)
{
  if (out == NULL) {
    return;
  }
  memset(out, 0, sizeof(*out));
  strncpy(out->last_server, s_last_sntp_server, sizeof(out->last_server) - 1);
  out->last_server[sizeof(out->last_server) - 1] = '\0';
  out->last_attempt_epoch = s_last_sntp_attempt_epoch;
  out->last_result = s_last_sntp_result;
  out->last_success_epoch = s_last_sntp_success_epoch;
}

int64_t
TimeSyncGetLastRtcSetEpoch(void)
{
  return s_last_rtc_set_epoch;
}

esp_err_t
TimeSyncReadRtcEpoch(const time_sync_t* time_sync, int64_t* out_epoch_seconds)
{
  if (out_epoch_seconds == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  *out_epoch_seconds = 0;

  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }

  struct tm rtc_tm = { 0 };
  const esp_err_t read_result = Ds3231ReadTime(time_sync, &rtc_tm);
  if (read_result != ESP_OK) {
    return read_result;
  }

  // DS3231 is stored as UTC. Ensure plausibility before converting.
  if (!YearLooksValid(rtc_tm.tm_year)) {
    return ESP_ERR_INVALID_STATE;
  }
  rtc_tm.tm_isdst = 0;

  *out_epoch_seconds = UtcTmToEpochSeconds(&rtc_tm);
  return ESP_OK;
}

/**
 * @brief Execute LocalTmFieldsMatch.
 * @param left Parameter left.
 * @param right Parameter right.
 * @return Return the function result.
 */
static bool
LocalTmFieldsMatch(const struct tm* left, const struct tm* right)
{
  return left != NULL && right != NULL && left->tm_year == right->tm_year &&
         left->tm_mon == right->tm_mon && left->tm_mday == right->tm_mday &&
         left->tm_hour == right->tm_hour && left->tm_min == right->tm_min &&
         left->tm_sec == right->tm_sec;
}

/**
 * @brief Execute TimeParseLocalIso.
 * @param iso Parameter iso.
 * @param out_tm_local Parameter out_tm_local.
 * @return Return the function result.
 */
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
  int matched = sscanf(iso,
                       "%d-%d-%d %d:%d:%d %n",
                       &year,
                       &month,
                       &day,
                       &hour,
                       &minute,
                       &second,
                       &consumed);
  if (matched != 6) {
    consumed = 0;
    matched = sscanf(iso,
                     "%d-%d-%dT%d:%d:%d %n",
                     &year,
                     &month,
                     &day,
                     &hour,
                     &minute,
                     &second,
                     &consumed);
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

/**
 * @brief Execute TimeLocalTmToEpochUtc.
 * @param tm_local Parameter tm_local.
 * @param out_epoch_utc Parameter out_epoch_utc.
 * @param out_ambiguous Parameter out_ambiguous.
 * @return Return the function result.
 */
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

/**
 * @brief Execute TimeSyncStartSntpAndWait.
 * @param sntp_server Parameter sntp_server.
 * @param timeout_ms Parameter timeout_ms.
 * @return Return the function result.
 */
esp_err_t
TimeSyncStartSntpAndWait(const char* sntp_server, int timeout_ms)
{
  if (sntp_server == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  strncpy(s_last_sntp_server, sntp_server, sizeof(s_last_sntp_server) - 1);
  s_last_sntp_server[sizeof(s_last_sntp_server) - 1] = '\0';
  s_last_sntp_attempt_epoch = (int64_t)time(NULL);
  s_last_sntp_result = ESP_ERR_INVALID_STATE;

#if APP_USE_ESP_NETIF_SNTP
  // esp_netif_sntp_init() may only be called once unless the service is
  // destroyed with esp_netif_sntp_deinit(). Deinit between one-off syncs to
  // avoid "already initialized" warnings and to allow server changes.
  if (s_esp_netif_sntp_initialized) {
    esp_netif_sntp_deinit();
    s_esp_netif_sntp_initialized = false;
  }

  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(sntp_server);
  config.smooth_sync = false;
  const esp_err_t init_result = esp_netif_sntp_init(&config);
  if (init_result != ESP_OK) {
    ESP_LOGW(kTag, "SNTP init failed: %s", esp_err_to_name(init_result));
    s_last_sntp_result = init_result;
    return init_result;
  }
  s_esp_netif_sntp_initialized = true;

  esp_err_t wait_result = ESP_ERR_TIMEOUT;
  const TickType_t start_ticks = xTaskGetTickCount();
  while (true) {
#ifdef esp_netif_sntp_sync_wait
    wait_result = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(250));
    if (wait_result == ESP_OK && TimeSyncIsSystemTimeValid()) {
      break;
    }
#else
    // No sync_wait symbol; poll for valid time instead.

    if (TimeSyncIsSystemTimeValid()) {
      wait_result = ESP_OK;
      break;
    }
    wait_result = ESP_ERR_TIMEOUT;
    vTaskDelay(pdMS_TO_TICKS(200));
#endif

    const int elapsed_ms =
      (int)pdTICKS_TO_MS(xTaskGetTickCount() - start_ticks);
    if (elapsed_ms >= timeout_ms) {
      wait_result = ESP_ERR_TIMEOUT;
      break;
    }
  }

  // Stop + destroy the SNTP service; we run it only for one-off sync requests.
  esp_netif_sntp_deinit();
  s_esp_netif_sntp_initialized = false;

  if (wait_result != ESP_OK) {
    ESP_LOGW(kTag, "SNTP timeout/failure: %s", esp_err_to_name(wait_result));
    s_last_sntp_result = wait_result;
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
      s_last_sntp_result = ESP_ERR_TIMEOUT;
      return ESP_ERR_TIMEOUT;
    }
  }
#endif

  if (!TimeSyncIsSystemTimeValid()) {
    ESP_LOGW(kTag, "SNTP completed, but system time still not plausible");
    s_last_sntp_result = ESP_ERR_INVALID_STATE;
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(kTag, "SNTP synced");
  s_last_sntp_result = ESP_OK;
  s_last_sntp_success_epoch = (int64_t)time(NULL);
  return ESP_OK;
}

/**
 * @brief Execute TimeSyncSetSystemEpoch.
 * @param epoch_seconds Parameter epoch_seconds.
 * @param update_rtc Parameter update_rtc.
 * @param time_sync Parameter time_sync.
 * @return Return the function result.
 */
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

/**
 * @brief Execute TimeSyncReadRtcRegisters.
 * @param time_sync Parameter time_sync.
 * @param start_reg Parameter start_reg.
 * @param data_out Parameter data_out.
 * @param length Parameter length.
 * @return Return the function result.
 */
esp_err_t
TimeSyncReadRtcRegisters(const time_sync_t* time_sync,
                         uint8_t start_reg,
                         uint8_t* data_out,
                         size_t length)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  return I2cBusReadRegister(
    time_sync->ds3231_device, start_reg, data_out, length);
}

/**
 * @brief Execute TimeSyncReadRtcTime.
 * @param time_sync Parameter time_sync.
 * @param time_out Parameter time_out.
 * @return Return the function result.
 */
esp_err_t
TimeSyncReadRtcTime(const time_sync_t* time_sync, struct tm* time_out)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  return Ds3231ReadTime(time_sync, time_out);
}

/**
 * @brief Execute TimeSyncWriteRtcTime.
 * @param time_sync Parameter time_sync.
 * @param time_value Parameter time_value.
 * @return Return the function result.
 */
esp_err_t
TimeSyncWriteRtcTime(const time_sync_t* time_sync, const struct tm* time_value)
{
  if (time_sync == NULL || !time_sync->is_ds3231_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  return Ds3231WriteTime(time_sync, time_value);
}
