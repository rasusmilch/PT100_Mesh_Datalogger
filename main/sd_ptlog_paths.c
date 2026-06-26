#include "sd_ptlog_paths.h"

/*
 * FAT16 keeps the root directory in a fixed-size table, and long PTLOG
 * filenames consume multiple directory entries.  These helpers build the
 * future /logs/YYYY-MM layout and scan only explicitly approved locations so
 * later retention code can relieve root-directory pressure without treating
 * arbitrary files or host-created system directories as log candidates.
 */

#include <dirent.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <time.h>

static const char kPtlogSuffix[] = ".ptlog";
static const char kLogsDirName[] = "logs";

/* Fail closed on truncation: callers must never use partial FAT paths. */
static bool
JoinPath(const char* dir_path, const char* child_name, char* out_path, size_t out_path_size)
{
  if (dir_path == NULL || child_name == NULL || out_path == NULL || out_path_size == 0) {
    return false;
  }
  out_path[0] = '\0';
  const size_t dir_len = strlen(dir_path);
  const size_t child_len = strlen(child_name);
  const bool needs_slash = (dir_len > 0 && dir_path[dir_len - 1u] != '/');
  const size_t required = dir_len + (needs_slash ? 1u : 0u) + child_len + 1u;
  if (required > out_path_size) {
    return false;
  }
  memcpy(out_path, dir_path, dir_len);
  size_t used = dir_len;
  if (needs_slash) {
    out_path[used++] = '/';
  }
  memcpy(out_path + used, child_name, child_len);
  out_path[used + child_len] = '\0';
  return true;
}

static bool
CopyString(const char* source, char* dest, size_t dest_size)
{
  if (source == NULL || dest == NULL || dest_size == 0) {
    return false;
  }
  const size_t len = strlen(source);
  if (len + 1u > dest_size) {
    dest[0] = '\0';
    return false;
  }
  memcpy(dest, source, len + 1u);
  return true;
}

/* Host operating systems may create these directories; they are never PTLOG candidates. */
static bool
IsSystemDirectoryName(const char* name)
{
  return name == NULL || strcmp(name, ".") == 0 || strcmp(name, "..") == 0 ||
         strcmp(name, ".Trash-1000") == 0 || strcmp(name, "FOUND.000") == 0 ||
         strcmp(name, "System Volume Information") == 0;
}

bool
SdPtlogBuildLogRootPath(const char* mount_point, char* out_path, size_t out_path_size)
{
  return JoinPath(mount_point, kLogsDirName, out_path, out_path_size);
}

bool
SdPtlogBuildMonthDirPath(const char* mount_point,
                         const char* month_string,
                         char* out_path,
                         size_t out_path_size)
{
  char log_root[SD_PTLOG_MAX_PATH_LEN];
  if (!SdPtlogIsMonthDirectoryName(month_string) ||
      !SdPtlogBuildLogRootPath(mount_point, log_root, sizeof(log_root))) {
    if (out_path != NULL && out_path_size > 0) out_path[0] = '\0';
    return false;
  }
  return JoinPath(log_root, month_string, out_path, out_path_size);
}

bool
SdPtlogBuildNestedPath(const char* mount_point,
                       int64_t epoch_seconds,
                       uint32_t revision,
                       char* date_out,
                       size_t date_out_size,
                       char* month_out,
                       size_t month_out_size,
                       char* path_out,
                       size_t path_out_size)
{
  if (date_out == NULL || month_out == NULL || path_out == NULL ||
      date_out_size < SD_PTLOG_DATE_LEN + 1u || month_out_size < SD_PTLOG_MONTH_LEN + 1u) {
    if (path_out != NULL && path_out_size > 0) path_out[0] = '\0';
    return false;
  }
  path_out[0] = '\0';
  time_t time_seconds = (time_t)epoch_seconds;
  struct tm time_info;
  gmtime_r(&time_seconds, &time_info);
  if (strftime(date_out, date_out_size, "%Y-%m-%dZ", &time_info) != SD_PTLOG_DATE_LEN ||
      strftime(month_out, month_out_size, "%Y-%m", &time_info) != SD_PTLOG_MONTH_LEN) {
    path_out[0] = '\0';
    return false;
  }
  char daily_name[SD_PTLOG_MAX_NAME_LEN];
  int written = (revision == 0u)
                  ? snprintf(daily_name, sizeof(daily_name), "%s%s", date_out, kPtlogSuffix)
                  : snprintf(daily_name, sizeof(daily_name), "%s-%" PRIu32 "%s", date_out, revision, kPtlogSuffix);
  if (written < 0 || written >= (int)sizeof(daily_name)) {
    path_out[0] = '\0';
    return false;
  }
  char month_dir[SD_PTLOG_MAX_PATH_LEN];
  if (!SdPtlogBuildMonthDirPath(mount_point, month_out, month_dir, sizeof(month_dir))) {
    path_out[0] = '\0';
    return false;
  }
  return JoinPath(month_dir, daily_name, path_out, path_out_size);
}

bool
SdPtlogParseName(const char* name,
                 char* date_out,
                 size_t date_out_size,
                 uint32_t* revision_out)
{
  if (name == NULL) return false;
  const size_t suffix_len = sizeof(kPtlogSuffix) - 1u;
  const size_t length = strlen(name);
  if (length < SD_PTLOG_DATE_LEN + suffix_len || strcmp(name + length - suffix_len, kPtlogSuffix) != 0) {
    return false;
  }
  const size_t prefix_len = length - suffix_len;
  if (prefix_len < SD_PTLOG_DATE_LEN || prefix_len > 22u) return false;
  if (name[4] != '-' || name[7] != '-' || name[10] != 'Z') return false;
  for (size_t i = 0; i < SD_PTLOG_DATE_LEN; ++i) {
    if (i == 4u || i == 7u || i == 10u) continue;
    if (name[i] < '0' || name[i] > '9') return false;
  }
  uint32_t revision = 0;
  if (prefix_len > SD_PTLOG_DATE_LEN) {
    if (name[11] != '-' || prefix_len == 12u) return false;
    for (size_t i = 12u; i < prefix_len; ++i) {
      if (name[i] < '0' || name[i] > '9') return false;
      const uint32_t digit = (uint32_t)(name[i] - '0');
      if (revision > (UINT32_MAX - digit) / 10u) return false;
      revision = revision * 10u + digit;
    }
  }
  if (date_out != NULL && date_out_size > 0) {
    if (date_out_size < SD_PTLOG_DATE_LEN + 1u) return false;
    memcpy(date_out, name, SD_PTLOG_DATE_LEN);
    date_out[SD_PTLOG_DATE_LEN] = '\0';
  }
  if (revision_out != NULL) *revision_out = revision;
  return true;
}

bool
SdPtlogIsMonthDirectoryName(const char* name)
{
  if (name == NULL || strlen(name) != SD_PTLOG_MONTH_LEN) return false;
  return name[0] >= '0' && name[0] <= '9' && name[1] >= '0' && name[1] <= '9' &&
         name[2] >= '0' && name[2] <= '9' && name[3] >= '0' && name[3] <= '9' &&
         name[4] == '-' &&
         ((name[5] == '0' && name[6] >= '1' && name[6] <= '9') ||
          (name[5] == '1' && name[6] >= '0' && name[6] <= '2'));
}

static bool
CandidateIsOlder(const sd_ptlog_candidate_t* candidate, const sd_ptlog_candidate_t* best)
{
  const int date_cmp = strcmp(candidate->date, best->date);
  if (date_cmp != 0) return date_cmp < 0;
  if (candidate->revision != best->revision) return candidate->revision < best->revision;
  if (candidate->legacy_root != best->legacy_root) return candidate->legacy_root;
  return strcmp(candidate->path, best->path) < 0;
}

static void
ConsiderCandidate(const char* dir_path,
                  const char* name,
                  bool legacy_root,
                  const char* current_path,
                  const char* current_date,
                  bool* found,
                  sd_ptlog_candidate_t* best)
{
  char date[SD_PTLOG_DATE_LEN + 1u];
  uint32_t revision = 0;
  if (!SdPtlogParseName(name, date, sizeof(date), &revision)) return;
  char path[SD_PTLOG_MAX_PATH_LEN];
  if (!JoinPath(dir_path, name, path, sizeof(path))) return;
  struct stat stat_buffer;
  if (stat(path, &stat_buffer) != 0 || !S_ISREG(stat_buffer.st_mode)) return;
  /* Later retention code may delete candidates, so PTLOG-looking directories
   * or special files must fail closed even when their basenames parse. */
  const bool current_open = (current_path != NULL && strcmp(path, current_path) == 0);
  if (current_open) return;
  if (current_date != NULL && current_date[0] != '\0' && strcmp(date, current_date) == 0) return;
  sd_ptlog_candidate_t candidate;
  memset(&candidate, 0, sizeof(candidate));
  if (!CopyString(path, candidate.path, sizeof(candidate.path)) ||
      !CopyString(name, candidate.name, sizeof(candidate.name)) ||
      !CopyString(date, candidate.date, sizeof(candidate.date))) {
    return;
  }
  candidate.revision = revision;
  candidate.legacy_root = legacy_root;
  candidate.current_open = false;
  if (!*found || CandidateIsOlder(&candidate, best)) {
    *best = candidate;
    *found = true;
  }
}

static void
ScanPtlogDirectory(const char* dir_path,
                   bool legacy_root,
                   const char* current_path,
                   const char* current_date,
                   bool* found,
                   sd_ptlog_candidate_t* best)
{
  DIR* dir = opendir(dir_path);
  if (dir == NULL) return;
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    if (IsSystemDirectoryName(entry->d_name)) continue;
    ConsiderCandidate(dir_path, entry->d_name, legacy_root, current_path, current_date, found, best);
  }
  closedir(dir);
}

static bool
StatsCountPtlogFile(const char* dir_path,
                    const char* name,
                    bool legacy_root,
                    const char* current_path,
                    const char* current_date,
                    sd_ptlog_stats_t* stats)
{
  char date[SD_PTLOG_DATE_LEN + 1u];
  uint32_t revision = 0;
  (void)revision;
  if (!SdPtlogParseName(name, date, sizeof(date), &revision)) return true;
  char path[SD_PTLOG_MAX_PATH_LEN];
  if (!JoinPath(dir_path, name, path, sizeof(path))) return false;
  struct stat stat_buffer;
  if (stat(path, &stat_buffer) != 0 || !S_ISREG(stat_buffer.st_mode)) return true;

  stats->total_ptlog_files++;
  if (legacy_root) {
    stats->legacy_root_ptlog_files++;
  } else {
    stats->nested_month_ptlog_files++;
  }

  const bool current_open = (current_path != NULL && strcmp(path, current_path) == 0);
  const bool current_date_match =
    (current_date != NULL && current_date[0] != '\0' && strcmp(date, current_date) == 0);
  if (current_date_match) {
    stats->current_date_ptlog_files++;
  }
  if (!current_open && !current_date_match) {
    stats->eligible_ptlog_files++;
  }
  return true;
}

static bool
ScanPtlogStatsDirectory(const char* dir_path,
                        bool legacy_root,
                        const char* current_path,
                        const char* current_date,
                        sd_ptlog_stats_t* stats,
                        uint32_t* directory_ptlog_files)
{
  DIR* dir = opendir(dir_path);
  if (dir == NULL) return legacy_root ? false : true;
  uint32_t local_count = 0;
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    if (IsSystemDirectoryName(entry->d_name)) continue;
    const uint32_t before = stats->total_ptlog_files;
    if (!StatsCountPtlogFile(
          dir_path, entry->d_name, legacy_root, current_path, current_date, stats)) {
      closedir(dir);
      return false;
    }
    if (!legacy_root && stats->total_ptlog_files > before) {
      local_count++;
    }
  }
  closedir(dir);
  if (directory_ptlog_files != NULL) {
    *directory_ptlog_files = local_count;
  }
  return true;
}

/*
 * Bounded traversal foundation: scan /sdcard, /sdcard/logs, and one level of
 * YYYY-MM month directories only.  Current/open and current-date files are
 * protected here so later deletion policies cannot select them accidentally.
 */
bool
SdPtlogFindOldestCandidate(const char* mount_point,
                           const char* current_path,
                           const char* current_date,
                           sd_ptlog_candidate_t* candidate_out)
{
  if (mount_point == NULL || candidate_out == NULL) return false;
  bool found = false;
  sd_ptlog_candidate_t best;
  memset(&best, 0, sizeof(best));

  ScanPtlogDirectory(mount_point, true, current_path, current_date, &found, &best);

  char log_root[SD_PTLOG_MAX_PATH_LEN];
  if (SdPtlogBuildLogRootPath(mount_point, log_root, sizeof(log_root))) {
    DIR* logs = opendir(log_root);
    if (logs != NULL) {
      struct dirent* entry;
      while ((entry = readdir(logs)) != NULL) {
        if (IsSystemDirectoryName(entry->d_name) || !SdPtlogIsMonthDirectoryName(entry->d_name)) continue;
        char month_dir[SD_PTLOG_MAX_PATH_LEN];
        if (!JoinPath(log_root, entry->d_name, month_dir, sizeof(month_dir))) continue;
        ScanPtlogDirectory(month_dir, false, current_path, current_date, &found, &best);
      }
      closedir(logs);
    }
  }

  if (found) *candidate_out = best;
  return found;
}

bool
SdPtlogCollectStats(const char* mount_point,
                    const char* current_path,
                    const char* current_date,
                    sd_ptlog_stats_t* stats_out)
{
  if (mount_point == NULL || stats_out == NULL) return false;
  memset(stats_out, 0, sizeof(*stats_out));

  if (!ScanPtlogStatsDirectory(
        mount_point, true, current_path, current_date, stats_out, NULL)) {
    return false;
  }

  char log_root[SD_PTLOG_MAX_PATH_LEN];
  if (!SdPtlogBuildLogRootPath(mount_point, log_root, sizeof(log_root))) {
    return false;
  }

  DIR* logs = opendir(log_root);
  if (logs == NULL) return true;

  struct dirent* entry;
  while ((entry = readdir(logs)) != NULL) {
    if (IsSystemDirectoryName(entry->d_name) ||
        !SdPtlogIsMonthDirectoryName(entry->d_name)) {
      continue;
    }
    char month_dir[SD_PTLOG_MAX_PATH_LEN];
    if (!JoinPath(log_root, entry->d_name, month_dir, sizeof(month_dir))) {
      closedir(logs);
      return false;
    }
    stats_out->valid_month_directories++;
    uint32_t month_count = 0;
    if (!ScanPtlogStatsDirectory(
          month_dir, false, current_path, current_date, stats_out, &month_count)) {
      closedir(logs);
      return false;
    }
    if (month_count > 0u &&
        (month_count > stats_out->max_month_ptlog_files ||
        (month_count == stats_out->max_month_ptlog_files &&
         stats_out->max_month_name[0] != '\0' &&
         strcmp(entry->d_name, stats_out->max_month_name) < 0))) {
      stats_out->max_month_ptlog_files = month_count;
      if (!CopyString(entry->d_name,
                      stats_out->max_month_name,
                      sizeof(stats_out->max_month_name))) {
        closedir(logs);
        return false;
      }
    }
  }
  closedir(logs);
  return true;
}
