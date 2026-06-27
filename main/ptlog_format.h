#ifndef PT100_LOGGER_PTLOG_FORMAT_H_
#define PT100_LOGGER_PTLOG_FORMAT_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_settings.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PTLOG_TEXT_FIELD_MAX_LEN 96
#define PTLOG_CAL_CONTEXT_MAX_LEN 160
#define PTLOG_MAGIC_TEXT "#PT100_LOG_V1"
#define PTLOG_MAGIC_LINE PTLOG_MAGIC_TEXT "\n"

typedef struct
{
  char created_utc[32];
  char device_serial[PTLOG_TEXT_FIELD_MAX_LEN];
  char device_mac[24];
  char device_role[16];
  char firmware_project[48];
  char firmware_version[48];
  char firmware_build_date[24];
  char firmware_build_time[24];
  char esp_idf_ver[32];
  char timezone_posix[APP_SETTINGS_TZ_POSIX_MAX_LEN];
  bool dst_enabled;
  char cal_last_utc[32];
  char cal_due_rule[32];
  char cal_due_utc[32];
  char cal_points_count[16];
  char cal_applied[8];
  char cal_method[PTLOG_TEXT_FIELD_MAX_LEN];
  char cal_context[PTLOG_CAL_CONTEXT_MAX_LEN];
} ptlog_header_t;

/**
 * @brief Write a PTLOG V1 metadata header and trailing CSV header row.
 * @param file Open destination file.
 * @param header Header metadata values.
 * @return True when all lines were written successfully.
 */
bool PtlogWriteHeader(FILE* file, const ptlog_header_t* header);

/**
 * @brief Return true when line exactly matches the PTLOG V1 magic line.
 *
 * @param line Null-terminated line buffer to compare.
 * @return true only for "#PT100_LOG_V1\n"; false for NULL, short, or partial input.
 */
bool PtlogIsMagicLine(const char* line);

/**
 * @brief Compute a deterministic signature over header-relevant metadata.
 * @param header Header metadata values.
 * @return CRC32 signature value.
 */
uint32_t PtlogComputeHeaderSignature(const ptlog_header_t* header);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_PTLOG_FORMAT_H_
