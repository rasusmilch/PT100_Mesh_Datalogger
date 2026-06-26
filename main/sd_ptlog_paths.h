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

/**
 * @brief Read-only counts from the bounded PTLOG retention scan.
 *
 * Stats include only parsed regular PTLOG files found in approved automatic
 * retention locations: the mount root for legacy files and /logs/YYYY-MM for
 * nested monthly files.  Unknown directories, malformed month names, system
 * directories, PTLOG-looking directories, and non-PTLOG files are ignored.
 *
 * Current-date and current-path files still contribute to total pressure
 * counts, but they are excluded from eligible_ptlog_files because automatic
 * deletion policy must remain outside this read-only helper.  These counts are
 * host-testable traversal facts; they do not prove FAT16 directory-entry
 * availability on target hardware.
 */
typedef struct
{
  uint32_t total_ptlog_files;        /**< Parsed regular PTLOG files counted. */
  uint32_t legacy_root_ptlog_files;  /**< Parsed regular PTLOG files in root. */
  uint32_t nested_month_ptlog_files; /**< Parsed regular PTLOG files in /logs/YYYY-MM. */
  uint32_t current_date_ptlog_files; /**< Counted PTLOGs whose parsed date matches current_date. */
  uint32_t eligible_ptlog_files;     /**< Counted PTLOGs not protected by current_path/current_date. */
  uint32_t valid_month_directories;  /**< Valid YYYY-MM directories seen directly under /logs. */
  uint32_t max_month_ptlog_files;    /**< Largest parsed regular PTLOG count in one month directory. */
  char max_month_name[SD_PTLOG_MONTH_LEN + 1u]; /**< Month name for max_month_ptlog_files, or empty. */
} sd_ptlog_stats_t;

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

/**
 * @brief Collect read-only PTLOG counts using the bounded retention traversal.
 *
 * @param mount_point Mounted SD root path, such as "/sdcard".
 * @param current_path Optional exact PTLOG path to exclude from eligibility.
 * @param current_date Optional current UTC date string in "YYYY-MM-DDZ" form.
 * @param stats_out Receives zero-initialized counts before scanning begins.
 *
 * @return true when arguments are valid and the bounded scan completed.
 * @return false when required arguments are invalid, the mount root cannot be
 * opened, or a required approved path cannot be built without truncation.
 *
 * @note Traversal is intentionally limited to the mount root, /logs, and
 * /logs/YYYY-MM.  Missing /logs and unreadable month directories are treated
 * like the candidate scanner: they produce no nested counts rather than a
 * threshold policy decision.
 * @warning This function never deletes files and never decides retention
 * thresholds.  Future policy code must compare these facts against approved
 * thresholds outside the PTLOG path/scanner layer.
 */
bool SdPtlogCollectStats(const char* mount_point,
                         const char* current_path,
                         const char* current_date,
                         sd_ptlog_stats_t* stats_out);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_PTLOG_PATHS_H_
