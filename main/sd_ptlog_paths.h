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
#define SD_PTLOG_MAX_REVISION 999u

/** Regular nested compact PTLOG file discovered by the bounded scanner. */
typedef struct
{
  char path[SD_PTLOG_MAX_PATH_LEN];
  char name[SD_PTLOG_MAX_NAME_LEN];
  char date[SD_PTLOG_DATE_LEN + 1u];
  uint32_t revision;
  bool legacy_root; /**< Obsolete compatibility field; always false in compact-only scans. */
  bool current_open;
} sd_ptlog_candidate_t;

/**
 * @brief Stable SD/PTLOG path workspace for bounded path and candidate workflows.
 *
 * Runtime logger paths own one instance through sd_logger_t and allocate it once
 * from PSRAM-capable 8-bit heap storage on target firmware.  The workspace has
 * non-stack lifetime for those runtime paths, fixed capacity from the SD_PTLOG
 * macros above, and named fields only for the nested PTLOG path workflow.
 *
 * Callers must serialize access with the SD/logger operation that owns the
 * workspace.  Helpers may overwrite any field before returning, so pointers into
 * this object must not escape the active locked operation.  Results that outlive
 * a helper call must be copied to caller-owned output buffers or persistent
 * logger-owned state.  Path truncation, invalid names, and missing workspace
 * arguments fail closed: helpers return false and clear final path outputs when
 * practical.  These helpers may block on bounded directory/stat traversal but
 * never delete files or mutate persistent state.
 */
typedef struct
{
  char log_root[SD_PTLOG_MAX_PATH_LEN];       /**< /mount/logs path. */
  char month_dir[SD_PTLOG_MAX_PATH_LEN];      /**< /mount/logs/YYYY-MM path. */
  char candidate_path[SD_PTLOG_MAX_PATH_LEN]; /**< Per-entry joined candidate path. */
  char daily_path[SD_PTLOG_MAX_PATH_LEN];     /**< Daily PTLOG path used by logger open. */
  char daily_name[SD_PTLOG_MAX_NAME_LEN];     /**< Compact YYYYMMDD.RRR filename. */
  char date[16];                              /**< Daily canonical date YYYY-MM-DDZ. */
  char month[16];                             /**< Daily canonical month YYYY-MM. */
  char parsed_date[SD_PTLOG_DATE_LEN + 1u];   /**< Parsed candidate/stat date. */
  sd_ptlog_candidate_t candidate;             /**< Current candidate under comparison. */
  sd_ptlog_candidate_t best;                  /**< Best eligible candidate found so far. */
} sd_ptlog_path_workspace_t;

#ifdef __cplusplus
static_assert(sizeof(((sd_ptlog_path_workspace_t*)0)->log_root) >= SD_PTLOG_MAX_PATH_LEN,
              "PTLOG workspace log_root must fit maximum PTLOG paths");
static_assert(sizeof(((sd_ptlog_path_workspace_t*)0)->daily_name) >= SD_PTLOG_MAX_NAME_LEN,
              "PTLOG workspace daily_name must fit maximum PTLOG names");
static_assert(sizeof(((sd_ptlog_path_workspace_t*)0)->date) >= SD_PTLOG_DATE_LEN + 1u,
              "PTLOG workspace date must fit canonical PTLOG dates");
static_assert(sizeof(((sd_ptlog_path_workspace_t*)0)->month) >= SD_PTLOG_MONTH_LEN + 1u,
              "PTLOG workspace month must fit canonical PTLOG months");
#else
_Static_assert(sizeof(((sd_ptlog_path_workspace_t*)0)->log_root) >= SD_PTLOG_MAX_PATH_LEN,
               "PTLOG workspace log_root must fit maximum PTLOG paths");
_Static_assert(sizeof(((sd_ptlog_path_workspace_t*)0)->daily_name) >= SD_PTLOG_MAX_NAME_LEN,
               "PTLOG workspace daily_name must fit maximum PTLOG names");
_Static_assert(sizeof(((sd_ptlog_path_workspace_t*)0)->date) >= SD_PTLOG_DATE_LEN + 1u,
               "PTLOG workspace date must fit canonical PTLOG dates");
_Static_assert(sizeof(((sd_ptlog_path_workspace_t*)0)->month) >= SD_PTLOG_MONTH_LEN + 1u,
               "PTLOG workspace month must fit canonical PTLOG months");
#endif

/**
 * @brief Read-only counts from the bounded PTLOG retention scan.
 *
 * Stats include only parsed regular compact PTLOG files found in approved
 * automatic retention locations: /logs/YYYY-MM month directories.  Root files,
 * old long .ptlog names, unknown directories, malformed month names, system
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
  uint32_t legacy_root_ptlog_files;  /**< Obsolete; always zero because root PTLOGs are ignored. */
  uint32_t nested_month_ptlog_files; /**< Parsed regular PTLOG files in /logs/YYYY-MM. */
  uint32_t current_date_ptlog_files; /**< Counted PTLOGs whose parsed date matches current_date. */
  uint32_t eligible_ptlog_files;     /**< Counted PTLOGs not protected by current_path/current_date. */
  uint32_t valid_month_directories;  /**< Openable YYYY-MM directories seen directly under /logs. */
  uint32_t max_month_ptlog_files;    /**< Largest parsed regular PTLOG count in one month directory. */
  char max_month_name[SD_PTLOG_MONTH_LEN + 1u]; /**< Month name for max_month_ptlog_files, or empty. */
} sd_ptlog_stats_t;

/** Build /<mount>/logs for the FAT16-safe nested PTLOG layout. */
bool SdPtlogBuildLogRootPath(const char* mount_point,
                             char* out_path,
                             size_t out_path_size);

/**
 * @brief Workspace-backed /<mount>/logs/YYYY-MM builder.
 *
 * @param workspace Caller-owned serialized PTLOG path workspace.
 *
 * Uses only named workspace fields for intermediate paths and fails closed on
 * invalid month names or truncation.  The returned path is copied into
 * caller-owned out_path; no workspace pointer escapes.
 */
bool SdPtlogBuildMonthDirPathWithWorkspace(sd_ptlog_path_workspace_t* workspace,
                                           const char* mount_point,
                                           const char* month_string,
                                           char* out_path,
                                           size_t out_path_size);

/**
 * @brief Workspace-backed canonical nested PTLOG path builder.
 *
 * The workspace must have non-stack lifetime for runtime logger calls and must
 * be serialized by the caller.  The helper may log no messages, performs no SD
 * I/O, mutates no persistent state, and fails closed by clearing path_out on
 * invalid revision, invalid output capacity, or truncation.
 */
bool SdPtlogBuildNestedPathWithWorkspace(sd_ptlog_path_workspace_t* workspace,
                                         const char* mount_point,
                                         int64_t epoch_seconds,
                                         uint32_t revision,
                                         char* date_out,
                                         size_t date_out_size,
                                         char* month_out,
                                         size_t month_out_size,
                                         char* path_out,
                                         size_t path_out_size);

/** Parse only compact YYYYMMDD.RRR basenames into canonical date/revision fields. */
bool SdPtlogParseName(const char* name,
                      char* date_out,
                      size_t date_out_size,
                      uint32_t* revision_out);

/**
 * @brief Fold one existing same-day compact revision into a next-revision value.
 *
 * Callers initialize revision_out to 0 before scanning. Each regular compact
 * same-day file updates it to max(existing_revision + 1) while rejecting
 * SD_PTLOG_MAX_REVISION so firmware fails closed instead of wrapping past .999.
 */
bool SdPtlogAccumulateNextRevision(uint32_t existing_revision,
                                   uint32_t* revision_out);

/** Return true only for bounded traversal month directory names in YYYY-MM form. */
bool SdPtlogIsMonthDirectoryName(const char* name);

/**
 * @brief Workspace-backed bounded oldest-candidate scan.
 *
 * The caller owns and serializes workspace for the full scan.  Traversal is
 * bounded to /logs/YYYY-MM month directories, may block on directory/stat I/O,
 * never deletes files, and copies any result into candidate_out before return.
 */
bool SdPtlogFindOldestCandidateWithWorkspace(sd_ptlog_path_workspace_t* workspace,
                                             const char* mount_point,
                                             const char* current_path,
                                             const char* current_date,
                                             sd_ptlog_candidate_t* candidate_out);

/**
 * @brief Workspace-backed bounded PTLOG stats scan.
 *
 * @param workspace Caller-owned serialized PTLOG path workspace.
 * @param mount_point Mounted SD root path, such as "/sdcard".
 * @param current_path Optional exact PTLOG path to exclude from eligibility.
 * @param current_date Optional current UTC date string in "YYYY-MM-DDZ" form.
 * @param stats_out Receives zero-initialized counts before scanning begins.
 *
 * @return true when arguments are valid and the bounded scan completed.
 * @return false when required arguments are invalid or an approved path cannot
 * be built without truncation.
 *
 * @note Traversal is intentionally limited to /logs and /logs/YYYY-MM.
 * Missing /logs and unreadable month directories produce no counts rather than
 * a threshold policy decision. Root-level PTLOG-looking files are ignored.
 * @warning The caller owns and serializes workspace for the full scan. This
 * helper may block on directory/stat I/O, mutates only stats_out/workspace,
 * never deletes files, and never decides retention thresholds.
 */
bool SdPtlogCollectStatsWithWorkspace(sd_ptlog_path_workspace_t* workspace,
                                      const char* mount_point,
                                      const char* current_path,
                                      const char* current_date,
                                      sd_ptlog_stats_t* stats_out);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_PTLOG_PATHS_H_
