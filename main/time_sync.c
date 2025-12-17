
#include "time_sync.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_log.h"

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

static esp_err_t
I2cReadRegisters(i2c_port_t port,
                 uint8_t device_addr,
                 uint8_t reg,
                 uint8_t* data_out,
                 size_t length)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
  if (length > 1) {
    i2c_master_read(cmd, data_out, length - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, &data_out[length - 1], I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t result = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(250));
  i2c_cmd_link_delete(cmd);
  return result;
}

static esp_err_t
I2cWriteRegisters(i2c_port_t port,
                  uint8_t device_addr,
                  uint8_t reg,
                  const uint8_t* data,
                  size_t length)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write(cmd, (uint8_t*)data, length, true);
  i2c_master_stop(cmd);
  esp_err_t result = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(250));
  i2c_cmd_link_delete(cmd);
  return result;
}

esp_err_t
TimeSyncInit(time_sync_t* time_sync,
             i2c_port_t port,
             int sda_gpio,
             int scl_gpio,
             uint8_t ds3231_addr)
{
  if (time_sync == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(time_sync, 0, sizeof(*time_sync));
  time_sync->port = port;
  time_sync->ds3231_addr = ds3231_addr;

  // Configure I2C (legacy driver for compatibility and simplicity).
  const i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sda_gpio,
    .scl_io_num = scl_gpio,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,
  };

  esp_err_t result = i2c_param_config(port, &config);
  if (result != ESP_OK) {
    return result;
  }
  result = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    return result;
  }

  time_sync->is_i2c_initialized = true;
  return ESP_OK;
}

static esp_err_t
Ds3231ReadTime(const time_sync_t* time_sync, struct tm* time_out)
{
  uint8_t regs[7] = { 0 };
  esp_err_t result = I2cReadRegisters(
    time_sync->port, time_sync->ds3231_addr, 0x00, regs, sizeof(regs));
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
  return I2cWriteRegisters(
    time_sync->port, time_sync->ds3231_addr, 0x00, regs, sizeof(regs));
}

esp_err_t
TimeSyncSetSystemFromRtc(const time_sync_t* time_sync)
{
  if (time_sync == NULL || !time_sync->is_i2c_initialized) {
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

  // ESP-IDF defaults to UTC unless TZ is configured.
  // Use mktime() with tm_isdst=0 as a practical UTC conversion.
  rtc_time.tm_isdst = 0;
  time_t epoch_seconds = mktime(&rtc_time);
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
  if (time_sync == NULL || !time_sync->is_i2c_initialized) {
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
