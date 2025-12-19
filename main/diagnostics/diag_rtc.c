#include "diagnostics/diag_rtc.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "time_sync.h"

static int64_t
TmToEpochUtc(struct tm* tm_value)
{
  if (tm_value == NULL) {
    return -1;
  }
  tm_value->tm_isdst = 0;
  time_t epoch = mktime(tm_value);
  return (epoch >= 0) ? (int64_t)epoch : -1;
}

static void
FormatTmUtc(const struct tm* tm_value, char* out, size_t out_len)
{
  if (out == NULL || out_len == 0) {
    return;
  }
  if (tm_value == NULL) {
    out[0] = '\0';
    return;
  }
  snprintf(out,
           out_len,
           "%04d-%02d-%02d %02d:%02d:%02dZ",
           tm_value->tm_year + 1900,
           tm_value->tm_mon + 1,
           tm_value->tm_mday,
           tm_value->tm_hour,
           tm_value->tm_min,
           tm_value->tm_sec);
}

int
RunDiagRtc(const app_runtime_t* runtime,
           bool full,
           bool set_known,
           diag_verbosity_t verbosity)
{
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "RTC", verbosity);

  const bool do_scan = full;
  const bool do_set_known = full && set_known;
  const int total_steps = 3 + (do_scan ? 1 : 0) + (do_set_known ? 1 : 0);
  int step_index = 1;

  if (runtime == NULL || runtime->time_sync == NULL || runtime->i2c_bus == NULL) {
    DiagReportStep(&ctx,
                   step_index++,
                   total_steps,
                   "runtime available",
                   ESP_ERR_INVALID_STATE,
                   "runtime/time_sync missing");
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  const time_sync_t* time_sync = runtime->time_sync;
  const i2c_bus_t* bus = runtime->i2c_bus;

  const esp_err_t bus_result = bus->initialized ? ESP_OK : ESP_ERR_INVALID_STATE;
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "bus init",
                 bus_result,
                 "port=%d sda=%d scl=%d freq=%luHz addr=0x%02x ds_ready=%d",
                 (int)bus->port,
                 bus->sda_gpio,
                 bus->scl_gpio,
                 (unsigned long)bus->frequency_hz,
                 (unsigned)time_sync->ds3231_addr,
                 time_sync->is_ds3231_ready ? 1 : 0);

  uint8_t registers[0x13] = { 0 };
  esp_err_t registers_result = ESP_ERR_INVALID_STATE;

  if (do_scan) {
    uint8_t found[16] = { 0 };
    size_t found_count = 0;
    esp_err_t scan_result =
      I2cBusScan(bus, 0x03, 0x77, found, sizeof(found), &found_count);
    char details[160];
    if (scan_result == ESP_OK) {
      if (found_count == 0) {
        snprintf(details, sizeof(details), "no devices found");
      } else {
        char list[96] = { 0 };
        size_t pos = 0;
        const size_t max_list = sizeof(found) / sizeof(found[0]);
        for (size_t i = 0; i < found_count && i < max_list && pos + 6 < sizeof(list); ++i) {
          pos += (size_t)snprintf(&list[pos],
                                  sizeof(list) - pos,
                                  "%s0x%02x",
                                  (i == 0) ? "" : " ",
                                  found[i]);
        }
        snprintf(details,
                 sizeof(details),
                 "found %u device(s)%s: %s",
                 (unsigned)found_count,
                 (found_count > max_list) ? " (truncated)" : "",
                 list);
      }
    } else {
      snprintf(details,
               sizeof(details),
               "scan error after %u devices: %s",
               (unsigned)found_count,
               esp_err_to_name(scan_result));
    }
    DiagReportStep(
      &ctx, step_index++, total_steps, "i2c scan", scan_result, "%s", details);
  }

  registers_result = TimeSyncReadRtcRegisters(
    time_sync, 0x00, registers, sizeof(registers));
  char probe_details[160];
  if (registers_result == ESP_OK) {
    snprintf(probe_details,
             sizeof(probe_details),
             "read 0x00-0x12 ok (control=0x%02x status=0x%02x)",
             registers[0x0E],
             registers[0x0F]);
  } else {
    snprintf(probe_details,
             sizeof(probe_details),
             "DS3231 addr=0x%02x probe failed: %s",
             (unsigned)time_sync->ds3231_addr,
             esp_err_to_name(registers_result));
  }
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "ds3231 probe",
                 registers_result,
                 "%s",
                 probe_details);
  DiagHexdump(&ctx, "DS3231 registers", registers, sizeof(registers));

  uint8_t status_reg = registers[0x0F];
  bool status_valid = false;
  if (registers_result == ESP_OK) {
    status_valid = true;
  } else {
    const esp_err_t status_result =
      TimeSyncReadRtcRegisters(time_sync, 0x0F, &status_reg, 1);
    status_valid = (status_result == ESP_OK);
  }

  struct tm rtc_time;
  esp_err_t rtc_result = TimeSyncReadRtcTime(time_sync, &rtc_time);
  char time_details[200];
  if (rtc_result == ESP_OK) {
    char time_string[64];
    FormatTmUtc(&rtc_time, time_string, sizeof(time_string));
    const bool osf = status_valid && ((status_reg & 0x80) != 0);
    char status_string[96];
    if (status_valid) {
      snprintf(status_string,
               sizeof(status_string),
               "0x%02x (OSF=%d %s)",
               status_reg,
               osf ? 1 : 0,
               osf ? "oscillator stopped/unknown" : "clock running");
    } else {
      snprintf(status_string, sizeof(status_string), "unknown");
    }
    snprintf(time_details,
             sizeof(time_details),
             "time=%s status=%s",
             time_string,
             status_string);
  } else {
    snprintf(time_details,
             sizeof(time_details),
             "RTC read failed: %s",
             esp_err_to_name(rtc_result));
  }
  DiagReportStep(
    &ctx, step_index++, total_steps, "time read", rtc_result, "%s", time_details);

  if (do_set_known) {
    struct tm known_time = {
      .tm_year = 124, // 2024
      .tm_mon = 0,
      .tm_mday = 1,
      .tm_hour = 0,
      .tm_min = 0,
      .tm_sec = 0,
    };
    const int64_t known_epoch = TmToEpochUtc(&known_time);
    esp_err_t write_result = TimeSyncWriteRtcTime(time_sync, &known_time);
    if (write_result == ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(3000));
    }

    struct tm readback_time;
    esp_err_t readback_result = (write_result == ESP_OK)
                                  ? TimeSyncReadRtcTime(time_sync, &readback_time)
                                  : write_result;
    char set_details[240];
    if (readback_result == ESP_OK) {
      const int64_t readback_epoch = TmToEpochUtc(&readback_time);
      const int64_t delta =
        (known_epoch >= 0 && readback_epoch >= 0) ? (readback_epoch - known_epoch)
                                                  : -1;
      char readback_string[64];
      FormatTmUtc(&readback_time, readback_string, sizeof(readback_string));
      snprintf(set_details,
               sizeof(set_details),
               "set %04d-%02d-%02d %02d:%02d:%02dZ, readback=%s delta=%" PRId64 "s",
               known_time.tm_year + 1900,
               known_time.tm_mon + 1,
               known_time.tm_mday,
               known_time.tm_hour,
               known_time.tm_min,
               known_time.tm_sec,
               readback_string,
               delta);
    } else {
      snprintf(set_details,
               sizeof(set_details),
               "set/read failed: %s",
               esp_err_to_name(readback_result));
    }
    DiagReportStep(&ctx,
                   step_index++,
                   total_steps,
                   "set-known time",
                   readback_result,
                   "%s",
                   set_details);
  }

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
