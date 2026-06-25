#ifndef PT100_LOGGER_SD_PTLOG_PATHS_H_
#define PT100_LOGGER_SD_PTLOG_PATHS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SD_PTLOG_DATE_LEN 11u
#define SD_PTLOG_MONTH_LEN 7u
#define SD_PTLOG_MAX_PATH_LEN 128u
#define SD_PTLOG_MAX_NAME_LEN 40u

/** Regular PTLOG file discovered by the bounded FAT16-safe scanner. */
typedef struct
{
  char path[SD_PTLOG_MAX_PATH_LEN];
  char name[SD_PTLOG_MAX_NAME_LEN];
  char date[SD_PTLOG_DATE_LEN + 1u];
  uint32_t revision;
  bool legacy_root;
  bool current_open;
} sd_ptlog_candidate_t;

/** Build /<mount>/logs for the FAT16-safe nested PTLOG layout. */
bool SdPtlogBuildLogRootPath(const char* mount_point,
                             char* out_path,
                             size_t out_path_size);

/** Build /<mount>/logs/YYYY-MM from a month string already validated as YYYY-MM. */
bool SdPtlogBuildMonthDirPath(const char* mount_point,
                              const char* month_string,
                              char* out_path,
                              size_t out_path_size);

/** Build date/month strings and nested /logs/YYYY-MM/YYYY-MM-DDZ[-rev].ptlog path. */
bool SdPtlogBuildNestedPath(const char* mount_point,
                            int64_t epoch_seconds,
                            uint32_t revision,
                            char* date_out,
                            size_t date_out_size,
                            char* month_out,
                            size_t month_out_size,
                            char* path_out,
                            size_t path_out_size);

/** Parse legacy/nested PTLOG basename YYYY-MM-DDZ[revision].ptlog. */
bool SdPtlogParseName(const char* name,
                      char* date_out,
                      size_t date_out_size,
                      uint32_t* revision_out);

/** Return true only for bounded traversal month directory names in YYYY-MM form. */
bool SdPtlogIsMonthDirectoryName(const char* name);

/** Find the oldest safe PTLOG candidate in root or /logs/YYYY-MM. */
bool SdPtlogFindOldestCandidate(const char* mount_point,
                                const char* current_path,
                                const char* current_date,
                                sd_ptlog_candidate_t* candidate_out);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_PTLOG_PATHS_H_
