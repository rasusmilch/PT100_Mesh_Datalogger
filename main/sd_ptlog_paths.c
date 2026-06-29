#include "sd_ptlog_paths.h"

/*
 * FAT16 keeps the root directory in a fixed-size table.  Firmware PTLOG files
 * now use only the nested one-entry YYYYMMDD.RRR layout under /logs/YYYY-MM.
 * Root-level files and old long .ptlog names are intentionally ignored.
 */

#include <dirent.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <time.h>

static const char kLogsDirName[] = "logs";

static bool
IsDigit(char c)
{
  return c >= '0' && c <= '9';
}

static bool
IsLeapYear(unsigned year)
{
  return (year % 4u == 0u && year % 100u != 0u) || (year % 400u == 0u);
}

static unsigned
DaysInMonth(unsigned year, unsigned month)
{
  static const unsigned kDaysPerMonth[] = { 31u, 28u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u };
  if (month < 1u || month > 12u) return 0u;
  if (month == 2u && IsLeapYear(year)) return 29u;
  return kDaysPerMonth[month - 1u];
}

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
SdPtlogBuildMonthDirPathWithWorkspace(sd_ptlog_path_workspace_t* workspace,
                                      const char* mount_point,
                                      const char* month_string,
                                      char* out_path,
                                      size_t out_path_size)
{
  if (workspace == NULL || !SdPtlogIsMonthDirectoryName(month_string) ||
      !SdPtlogBuildLogRootPath(
        mount_point, workspace->log_root, sizeof(workspace->log_root))) {
    if (out_path != NULL && out_path_size > 0) out_path[0] = '\0';
    return false;
  }
  return JoinPath(workspace->log_root, month_string, out_path, out_path_size);
}

bool
SdPtlogBuildNestedPathWithWorkspace(sd_ptlog_path_workspace_t* workspace,
                                    const char* mount_point,
                                    int64_t epoch_seconds,
                                    uint32_t revision,
                                    char* date_out,
                                    size_t date_out_size,
                                    char* month_out,
                                    size_t month_out_size,
                                    char* path_out,
                                    size_t path_out_size)
{
  if (workspace == NULL || date_out == NULL || month_out == NULL ||
      path_out == NULL || date_out_size < SD_PTLOG_DATE_LEN + 1u ||
      month_out_size < SD_PTLOG_MONTH_LEN + 1u) {
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
  if (revision > SD_PTLOG_MAX_REVISION) {
    path_out[0] = '\0';
    return false;
  }
  char compact_date[9];
  int compact_written = snprintf(compact_date,
                                 sizeof(compact_date),
                                 "%04d%02d%02d",
                                 time_info.tm_year + 1900,
                                 time_info.tm_mon + 1,
                                 time_info.tm_mday);
  if (compact_written != 8) {
    path_out[0] = '\0';
    return false;
  }
  int written = snprintf(workspace->daily_name,
                         sizeof(workspace->daily_name),
                         "%s.%03" PRIu32,
                         compact_date,
                         revision);
  if (written < 0 || written >= (int)sizeof(workspace->daily_name)) {
    path_out[0] = '\0';
    return false;
  }
  if (!SdPtlogBuildMonthDirPathWithWorkspace(
        workspace, mount_point, month_out, workspace->month_dir, sizeof(workspace->month_dir))) {
    path_out[0] = '\0';
    return false;
  }
  return JoinPath(workspace->month_dir, workspace->daily_name, path_out, path_out_size);
}

bool
SdPtlogParseName(const char* name,
                 char* date_out,
                 size_t date_out_size,
                 uint32_t* revision_out)
{
  if (name == NULL) return false;
  if (strlen(name) != 12u || name[8] != '.') return false;
  for (size_t i = 0; i < 8u; ++i) {
    if (!IsDigit(name[i])) return false;
  }
  for (size_t i = 9u; i < 12u; ++i) {
    if (!IsDigit(name[i])) return false;
  }
  const unsigned year = (unsigned)(name[0] - '0') * 1000u +
                        (unsigned)(name[1] - '0') * 100u +
                        (unsigned)(name[2] - '0') * 10u +
                        (unsigned)(name[3] - '0');
  const unsigned month = (unsigned)(name[4] - '0') * 10u +
                         (unsigned)(name[5] - '0');
  const unsigned day = (unsigned)(name[6] - '0') * 10u +
                       (unsigned)(name[7] - '0');
  const unsigned max_day = DaysInMonth(year, month);
  if (max_day == 0u || day < 1u || day > max_day) return false;
  if (date_out != NULL && date_out_size > 0) {
    if (date_out_size < SD_PTLOG_DATE_LEN + 1u) return false;
    int written = snprintf(date_out, date_out_size, "%04u-%02u-%02uZ", year, month, day);
    if (written != (int)SD_PTLOG_DATE_LEN) return false;
  }
  if (revision_out != NULL) {
    *revision_out = (uint32_t)(name[9] - '0') * 100u +
                    (uint32_t)(name[10] - '0') * 10u +
                    (uint32_t)(name[11] - '0');
  }
  return true;
}

bool
SdPtlogAccumulateNextRevision(uint32_t existing_revision, uint32_t* revision_out)
{
  if (revision_out == NULL || existing_revision >= SD_PTLOG_MAX_REVISION) {
    return false;
  }
  if (existing_revision >= *revision_out) {
    *revision_out = existing_revision + 1u;
  }
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
  return strcmp(candidate->path, best->path) < 0;
}

static void
ConsiderCandidate(const char* dir_path,
                  const char* name,
                  const char* current_path,
                  const char* current_date,
                  sd_ptlog_path_workspace_t* workspace,
                  bool* found,
                  sd_ptlog_candidate_t* best)
{
  uint32_t revision = 0;
  if (workspace == NULL ||
      !SdPtlogParseName(
        name, workspace->parsed_date, sizeof(workspace->parsed_date), &revision)) return;
  if (!JoinPath(
        dir_path, name, workspace->candidate_path, sizeof(workspace->candidate_path))) return;
  struct stat stat_buffer;
  if (stat(workspace->candidate_path, &stat_buffer) != 0 ||
      !S_ISREG(stat_buffer.st_mode)) return;
  /* Later retention code may delete candidates, so PTLOG-looking directories
   * or special files must fail closed even when their basenames parse. */
  const bool current_open =
    (current_path != NULL && strcmp(workspace->candidate_path, current_path) == 0);
  if (current_open) return;
  if (current_date != NULL && current_date[0] != '\0' &&
      strcmp(workspace->parsed_date, current_date) == 0) return;
  memset(&workspace->candidate, 0, sizeof(workspace->candidate));
  if (!CopyString(workspace->candidate_path,
                  workspace->candidate.path,
                  sizeof(workspace->candidate.path)) ||
      !CopyString(name, workspace->candidate.name, sizeof(workspace->candidate.name)) ||
      !CopyString(workspace->parsed_date,
                  workspace->candidate.date,
                  sizeof(workspace->candidate.date))) {
    return;
  }
  workspace->candidate.revision = revision;
  workspace->candidate.legacy_root = false;
  workspace->candidate.current_open = false;
  if (!*found || CandidateIsOlder(&workspace->candidate, best)) {
    *best = workspace->candidate;
    *found = true;
  }
}

static void
ScanPtlogDirectory(const char* dir_path,
                   const char* current_path,
                   const char* current_date,
                   sd_ptlog_path_workspace_t* workspace,
                   bool* found,
                   sd_ptlog_candidate_t* best)
{
  DIR* dir = opendir(dir_path);
  if (dir == NULL) return;
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    if (IsSystemDirectoryName(entry->d_name)) continue;
    ConsiderCandidate(
      dir_path, entry->d_name, current_path, current_date, workspace, found, best);
  }
  closedir(dir);
}

static bool
StatsCountPtlogFile(const char* dir_path,
                    const char* name,
                    const char* current_path,
                    const char* current_date,
                    sd_ptlog_path_workspace_t* workspace,
                    sd_ptlog_stats_t* stats)
{
  uint32_t revision = 0;
  (void)revision;
  if (workspace == NULL) return false;
  if (!SdPtlogParseName(
        name, workspace->parsed_date, sizeof(workspace->parsed_date), &revision)) return true;
  if (!JoinPath(
        dir_path, name, workspace->candidate_path, sizeof(workspace->candidate_path))) return false;
  struct stat stat_buffer;
  if (stat(workspace->candidate_path, &stat_buffer) != 0 ||
      !S_ISREG(stat_buffer.st_mode)) return true;

  stats->total_ptlog_files++;
  stats->nested_month_ptlog_files++;

  const bool current_open =
    (current_path != NULL && strcmp(workspace->candidate_path, current_path) == 0);
  const bool current_date_match =
    (current_date != NULL && current_date[0] != '\0' &&
     strcmp(workspace->parsed_date, current_date) == 0);
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
                        const char* current_path,
                        const char* current_date,
                        sd_ptlog_path_workspace_t* workspace,
                        sd_ptlog_stats_t* stats,
                        uint32_t* directory_ptlog_files,
                        bool* opened_out)
{
  if (opened_out != NULL) {
    *opened_out = false;
  }
  DIR* dir = opendir(dir_path);
  if (dir == NULL) return true;
  if (opened_out != NULL) {
    *opened_out = true;
  }
  uint32_t local_count = 0;
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    if (IsSystemDirectoryName(entry->d_name)) continue;
    const uint32_t before = stats->total_ptlog_files;
    if (!StatsCountPtlogFile(
          dir_path, entry->d_name, current_path, current_date, workspace, stats)) {
      closedir(dir);
      return false;
    }
    if (stats->total_ptlog_files > before) {
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
 * Bounded traversal foundation: scan /sdcard/logs and one level of YYYY-MM
 * month directories only.  Current/open and current-date files are
 * protected here so later deletion policies cannot select them accidentally.
 */
bool
SdPtlogFindOldestCandidateWithWorkspace(sd_ptlog_path_workspace_t* workspace,
                                        const char* mount_point,
                                        const char* current_path,
                                        const char* current_date,
                                        sd_ptlog_candidate_t* candidate_out)
{
  if (workspace == NULL || mount_point == NULL || candidate_out == NULL) return false;
  bool found = false;
  memset(&workspace->best, 0, sizeof(workspace->best));

  if (SdPtlogBuildLogRootPath(
        mount_point, workspace->log_root, sizeof(workspace->log_root))) {
    DIR* logs = opendir(workspace->log_root);
    if (logs != NULL) {
      struct dirent* entry;
      while ((entry = readdir(logs)) != NULL) {
        if (IsSystemDirectoryName(entry->d_name) || !SdPtlogIsMonthDirectoryName(entry->d_name)) continue;
        if (!JoinPath(
              workspace->log_root,
              entry->d_name,
              workspace->month_dir,
              sizeof(workspace->month_dir))) continue;
        ScanPtlogDirectory(
          workspace->month_dir, current_path, current_date, workspace, &found, &workspace->best);
      }
      closedir(logs);
    }
  }

  if (found) *candidate_out = workspace->best;
  return found;
}

bool
SdPtlogCollectStatsWithWorkspace(sd_ptlog_path_workspace_t* workspace,
                                 const char* mount_point,
                                 const char* current_path,
                                 const char* current_date,
                                 sd_ptlog_stats_t* stats_out)
{
  if (workspace == NULL || mount_point == NULL || stats_out == NULL) return false;
  memset(stats_out, 0, sizeof(*stats_out));

  if (!SdPtlogBuildLogRootPath(
        mount_point, workspace->log_root, sizeof(workspace->log_root))) {
    return false;
  }

  DIR* logs = opendir(workspace->log_root);
  if (logs == NULL) return true;

  struct dirent* entry;
  while ((entry = readdir(logs)) != NULL) {
    if (IsSystemDirectoryName(entry->d_name) ||
        !SdPtlogIsMonthDirectoryName(entry->d_name)) {
      continue;
    }
    if (!JoinPath(
          workspace->log_root, entry->d_name, workspace->month_dir, sizeof(workspace->month_dir))) {
      closedir(logs);
      return false;
    }
    uint32_t month_count = 0;
    bool month_opened = false;
    if (!ScanPtlogStatsDirectory(
          workspace->month_dir,
          current_path,
          current_date,
          workspace,
          stats_out,
          &month_count,
          &month_opened)) {
      closedir(logs);
      return false;
    }
    if (!month_opened) {
      continue;
    }
    stats_out->valid_month_directories++;
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
