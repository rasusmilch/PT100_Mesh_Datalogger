#include "ptlog_format.h"

#include <inttypes.h>
#include <string.h>

#include "data_csv.h"
#include "esp_rom_crc.h"

static bool
WriteLine(FILE* file, const char* line)
{
  return (file != NULL && line != NULL && fputs(line, file) >= 0);
}

static bool
CsvHeaderWriter(const char* bytes, size_t len, void* context)
{
  FILE* file = (FILE*)context;
  return (file != NULL && fwrite(bytes, 1, len, file) == len);
}

static uint32_t
CrcAddString(uint32_t crc, const char* value)
{
  const char* text = (value != NULL) ? value : "";
  crc = esp_rom_crc32_le(crc, (const uint8_t*)text, strlen(text));
  const uint8_t sep = '\n';
  return esp_rom_crc32_le(crc, &sep, 1);
}

/**
 * @brief Write a PTLOG V1 metadata header and trailing CSV header row.
 * @param file Open destination file.
 * @param header Header metadata values.
 * @return True when all lines were written successfully.
 */
bool
PtlogWriteHeader(FILE* file, const ptlog_header_t* header)
{
  if (file == NULL || header == NULL) {
    return false;
  }

  char line[256] = { 0 };
  if (!WriteLine(file, PTLOG_MAGIC_LINE) ||
      !WriteLine(file, "# header_version=1\n")) {
    return false;
  }

#define PTLOG_WRITE_KV(key, value)                                             \
  do {                                                                         \
    (void)snprintf(line, sizeof(line), "# %s=%s\n", (key), (value));           \
    if (!WriteLine(file, line)) {                                              \
      return false;                                                            \
    }                                                                          \
  } while (0)

  PTLOG_WRITE_KV("created_utc", header->created_utc);
  PTLOG_WRITE_KV("device_serial", header->device_serial);
  PTLOG_WRITE_KV("device_mac", header->device_mac);
  PTLOG_WRITE_KV("device_role", header->device_role);
  PTLOG_WRITE_KV("firmware_project", header->firmware_project);
  PTLOG_WRITE_KV("firmware_version", header->firmware_version);
  PTLOG_WRITE_KV("firmware_build_date", header->firmware_build_date);
  PTLOG_WRITE_KV("firmware_build_time", header->firmware_build_time);
  PTLOG_WRITE_KV("esp_idf_ver", header->esp_idf_ver);
  PTLOG_WRITE_KV("timezone_posix", header->timezone_posix);
  PTLOG_WRITE_KV("dst_enabled", header->dst_enabled ? "1" : "0");
  PTLOG_WRITE_KV("cal_last_utc", header->cal_last_utc);
  PTLOG_WRITE_KV("cal_due_rule", header->cal_due_rule);
  PTLOG_WRITE_KV("cal_due_utc", header->cal_due_utc);
  PTLOG_WRITE_KV("cal_points_count", header->cal_points_count);
  PTLOG_WRITE_KV("cal_applied", header->cal_applied);
  PTLOG_WRITE_KV("cal_method", header->cal_method);
  PTLOG_WRITE_KV("cal_context", header->cal_context);
#undef PTLOG_WRITE_KV

  if (!WriteLine(file, "#END_HEADER\n")) {
    return false;
  }
  return CsvWriteHeader(CsvHeaderWriter, file);
}

bool
PtlogIsMagicLine(const char* line)
{
  return line != NULL && strcmp(line, PTLOG_MAGIC_LINE) == 0;
}

/**
 * @brief Compute a deterministic signature over header-relevant metadata.
 * @param header Header metadata values.
 * @return CRC32 signature value.
 */
uint32_t
PtlogComputeHeaderSignature(const ptlog_header_t* header)
{
  if (header == NULL) {
    return 0;
  }
  uint32_t crc = 0;
  crc = CrcAddString(crc, header->device_mac);
  crc = CrcAddString(crc, header->device_serial);
  crc = CrcAddString(crc, header->timezone_posix);
  crc = CrcAddString(crc, header->dst_enabled ? "1" : "0");
  crc = CrcAddString(crc, header->cal_last_utc);
  crc = CrcAddString(crc, header->cal_due_rule);
  crc = CrcAddString(crc, header->cal_due_utc);
  crc = CrcAddString(crc, header->cal_points_count);
  crc = CrcAddString(crc, header->cal_applied);
  crc = CrcAddString(crc, header->cal_method);
  crc = CrcAddString(crc, header->cal_context);
  crc = CrcAddString(crc, header->firmware_version);
  crc = CrcAddString(crc, header->firmware_build_date);
  crc = CrcAddString(crc, header->firmware_build_time);
  return crc;
}
