#include "diagnostics/diag_fram.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "fram_i2c.h"
#include "fram_log.h"
#include "i2c_bus.h"

static void
FormatAddrList(char* out,
               size_t out_len,
               const uint8_t* addrs,
               size_t count,
               size_t max_count)
{
  if (out == NULL || out_len == 0) {
    return;
  }
  out[0] = '\0';
  size_t pos = 0;
  for (size_t i = 0; i < count && i < max_count && pos + 6 < out_len; ++i) {
    pos += (size_t)snprintf(&out[pos],
                            out_len - pos,
                            "%s0x%02x",
                            (i == 0) ? "" : " ",
                            addrs[i]);
  }
}

static void
FillPattern(uint8_t* buffer, size_t len, uint32_t seed)
{
  if (buffer == NULL) {
    return;
  }
  for (size_t i = 0; i < len; ++i) {
    buffer[i] = (uint8_t)((seed + 31u * i) & 0xFFu);
  }
}

static const fram_io_t*
SelectFramIo(const app_runtime_t* runtime)
{
  if (runtime == NULL) {
    return NULL;
  }
  if (runtime->fram_io != NULL) {
    return runtime->fram_io;
  }
  if (runtime->fram_log != NULL) {
    return &runtime->fram_log->io;
  }
  return NULL;
}

int
RunDiagFram(const app_runtime_t* runtime,
            bool full,
            int bytes,
            diag_verbosity_t verbosity)
{
  diag_ctx_t ctx;
  DiagInitCtx(&ctx, "FRAM", verbosity);
  const int total_steps = full ? 4 : 3;
  int step_index = 1;

  if (runtime == NULL || runtime->i2c_bus == NULL) {
    DiagReportStep(&ctx,
                   step_index++,
                   total_steps,
                   "runtime available",
                   ESP_ERR_INVALID_STATE,
                   "runtime or bus missing");
    DiagPrintSummary(&ctx, total_steps);
    return 1;
  }

  const i2c_bus_t* bus = runtime->i2c_bus;
  const fram_log_t* log = runtime->fram_log;
  const fram_i2c_t* fram = runtime->fram_i2c;
  const fram_io_t* io = SelectFramIo(runtime);
  const uint8_t fram_addr =
    (fram != NULL && fram->initialized) ? fram->i2c_addr_7bit
                                        : (uint8_t)CONFIG_APP_FRAM_I2C_ADDR;

  uint8_t found[32] = { 0 };
  size_t found_count = 0;
  const esp_err_t scan_result =
    I2cBusScan(bus, 0x08, 0x77, found, sizeof(found), &found_count);
  char scan_details[196];
  if (scan_result == ESP_OK) {
    if (found_count == 0) {
      snprintf(scan_details,
               sizeof(scan_details),
               "bus init=%d sda=%d scl=%d freq=%luHz; no devices found",
               bus->initialized ? 1 : 0,
               bus->sda_gpio,
               bus->scl_gpio,
               (unsigned long)bus->frequency_hz);
    } else {
      char list[120] = { 0 };
      FormatAddrList(list, sizeof(list), found, found_count, sizeof(found));
      snprintf(scan_details,
               sizeof(scan_details),
               "bus init=%d sda=%d scl=%d freq=%luHz; found %u%s: %s",
               bus->initialized ? 1 : 0,
               bus->sda_gpio,
               bus->scl_gpio,
               (unsigned long)bus->frequency_hz,
               (unsigned)found_count,
               (found_count > (sizeof(found) / sizeof(found[0]))) ? " (truncated)" : "",
               list);
    }
  } else {
    snprintf(scan_details,
             sizeof(scan_details),
             "scan failed after %u devices: %s",
             (unsigned)found_count,
             esp_err_to_name(scan_result));
  }
  DiagReportStep(
    &ctx, step_index++, total_steps, "i2c scan", scan_result, "%s", scan_details);

  const esp_err_t probe_result =
    bus->initialized ? i2c_master_probe(bus->handle, fram_addr, 100)
                     : ESP_ERR_INVALID_STATE;
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "fram probe",
                 probe_result,
                 "addr=0x%02x result=%s",
                 (unsigned)fram_addr,
                 esp_err_to_name(probe_result));

  fram_device_id_t device_id;
  const esp_err_t id_result = FramI2cReadDeviceId(fram, &device_id);
  bool id_match = false;
  char id_details[196];
  if (id_result == ESP_OK) {
    id_match = (device_id.manufacturer_id == 0x00Au) &&
               (device_id.product_id == 0x510u);
    snprintf(id_details,
             sizeof(id_details),
             "raw=%02x %02x %02x mfg=0x%03x prod=0x%03x (%s expected=00A/510)",
             device_id.raw[0],
             device_id.raw[1],
             device_id.raw[2],
             (unsigned)device_id.manufacturer_id,
             (unsigned)device_id.product_id,
             id_match ? "match" : "mismatch");
  } else {
    snprintf(id_details,
             sizeof(id_details),
             "device-id failed: %s (addr=0x%02x)",
             esp_err_to_name(id_result),
             (unsigned)fram_addr);
  }
  DiagReportStep(&ctx,
                 step_index++,
                 total_steps,
                 "device id",
                 id_result,
                 "%s",
                 id_details);
  if (id_result == ESP_OK && verbosity > kDiagVerbosity0) {
    DiagHexdump(&ctx, "device-id raw", device_id.raw, sizeof(device_id.raw));
  }

  if (full) {
    const uint32_t fram_size =
      (log != NULL && log->fram_size_bytes > 0) ? log->fram_size_bytes
                                                : CONFIG_APP_FRAM_SIZE_BYTES;
    const size_t header_guard = 256;
    size_t scratch_size =
      (bytes > 0) ? (size_t)bytes : (size_t)((fram_size >= 512) ? 512 : 256);
    if (fram_size <= header_guard + 16) {
      scratch_size = 0;
    } else if (scratch_size > fram_size - header_guard) {
      scratch_size = fram_size - header_guard;
    }
    uint32_t scratch_addr =
      (fram_size > scratch_size) ? (uint32_t)(fram_size - scratch_size) : 0;
    if (scratch_addr < header_guard) {
      scratch_addr = header_guard;
    }
    if (fram_size <= header_guard + 16 || scratch_size < 16) {
      DiagReportStep(&ctx,
                     step_index++,
                     total_steps,
                     "rw test",
                     ESP_ERR_INVALID_SIZE,
                     "fram_size=%u scratch=%u (need space past header %u)",
                     (unsigned)fram_size,
                     (unsigned)scratch_size,
                     (unsigned)header_guard);
    } else if (io == NULL || io->read == NULL || io->write == NULL) {
      DiagReportStep(&ctx,
                     step_index++,
                     total_steps,
                     "rw test",
                     ESP_ERR_INVALID_STATE,
                     "fram io unavailable");
    } else {
      const size_t max_test_len =
        (scratch_size > 96) ? 96 : (scratch_size > 32 ? scratch_size - 16 : scratch_size / 2);
      const size_t test_lengths[3] = {
        (max_test_len > 32) ? 32 : max_test_len,
        (max_test_len > 48) ? 48 : (max_test_len > 0 ? max_test_len : 1),
        (max_test_len > 0) ? max_test_len : 1,
      };
      const uint32_t test_offsets[3] = {
        0,
        3,
        (scratch_size > (test_lengths[2] + 8))
          ? (uint32_t)(scratch_size - test_lengths[2] - 1)
          : 1,
      };

      uint8_t* original = (uint8_t*)malloc(scratch_size);
      uint8_t* verify = (uint8_t*)malloc(scratch_size);
      uint8_t* pattern = (uint8_t*)malloc(max_test_len);
      esp_err_t rw_result = ESP_OK;
      char rw_details[240];
      uint32_t mismatch_addr = 0;
      uint8_t mismatch_expected = 0;
      uint8_t mismatch_actual = 0;

      if (original == NULL || verify == NULL || pattern == NULL) {
        rw_result = ESP_ERR_NO_MEM;
        snprintf(rw_details,
                 sizeof(rw_details),
                 "alloc failed for %u-byte scratch buffers",
                 (unsigned)scratch_size);
      } else {
        rw_result = io->read(io->context, scratch_addr, original, scratch_size);
        if (rw_result != ESP_OK) {
          snprintf(rw_details,
                   sizeof(rw_details),
                   "read scratch 0x%04x len=%u failed: %s",
                   (unsigned)scratch_addr,
                   (unsigned)scratch_size,
                   esp_err_to_name(rw_result));
        }
      }

      const size_t test_count = (sizeof(test_lengths) / sizeof(test_lengths[0]));
      for (size_t i = 0; rw_result == ESP_OK && i < test_count; ++i) {
        const size_t len = test_lengths[i];
        const uint32_t offset = test_offsets[i];
        const uint32_t target_addr = scratch_addr + offset;
        if ((offset + len) > scratch_size) {
          continue;
        }
        FillPattern(pattern, len, target_addr);
        rw_result = io->write(io->context, target_addr, pattern, len);
        if (rw_result != ESP_OK) {
          snprintf(rw_details,
                   sizeof(rw_details),
                   "write addr=0x%04x len=%u failed: %s",
                   (unsigned)target_addr,
                   (unsigned)len,
                   esp_err_to_name(rw_result));
          break;
        }
        rw_result = io->read(io->context, target_addr, verify, len);
        if (rw_result != ESP_OK) {
          snprintf(rw_details,
                   sizeof(rw_details),
                   "read-back addr=0x%04x len=%u failed: %s",
                   (unsigned)target_addr,
                   (unsigned)len,
                   esp_err_to_name(rw_result));
          break;
        }
        for (size_t b = 0; b < len; ++b) {
          if (verify[b] != pattern[b]) {
            mismatch_addr = target_addr + (uint32_t)b;
            mismatch_expected = pattern[b];
            mismatch_actual = verify[b];
            rw_result = ESP_ERR_INVALID_RESPONSE;
            snprintf(rw_details,
                     sizeof(rw_details),
                     "verify mismatch at 0x%04x exp=0x%02x got=0x%02x "
                     "(WP high=write protect; floating WP is pulled low)",
                     (unsigned)mismatch_addr,
                     (unsigned)mismatch_expected,
                     (unsigned)mismatch_actual);
            break;
          }
        }
      }

      if (original != NULL) {
        esp_err_t restore_result =
          io->write(io->context, scratch_addr, original, scratch_size);
        if (restore_result != ESP_OK && rw_result == ESP_OK) {
          rw_result = restore_result;
          snprintf(rw_details,
                   sizeof(rw_details),
                   "restore failed at 0x%04x len=%u: %s",
                   (unsigned)scratch_addr,
                   (unsigned)scratch_size,
                   esp_err_to_name(restore_result));
        }
      }

      if (rw_result == ESP_OK) {
        snprintf(rw_details,
                 sizeof(rw_details),
                 "scratch 0x%04x len=%u patterns ok",
                 (unsigned)scratch_addr,
                 (unsigned)scratch_size);
      }
      free(original);
      free(verify);
      free(pattern);

      DiagReportStep(&ctx,
                     step_index++,
                     total_steps,
                     "rw test",
                     rw_result,
                     "%s",
                     rw_details);
    }
  }

  DiagPrintSummary(&ctx, total_steps);
  return (ctx.steps_failed == 0) ? 0 : 1;
}
