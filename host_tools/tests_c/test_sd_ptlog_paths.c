#include "sd_ptlog_paths.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static void make_dir(const char* path) { assert(mkdir(path, 0777) == 0 || access(path, F_OK) == 0); }
static void make_file(const char* path) { FILE* f = fopen(path, "wb"); assert(f != NULL); fclose(f); }
static bool exists_path(const char* path) { return access(path, F_OK) == 0; }
static void assert_zero_stats(const sd_ptlog_stats_t* stats)
{
  assert(stats->total_ptlog_files == 0);
  assert(stats->legacy_root_ptlog_files == 0);
  assert(stats->nested_month_ptlog_files == 0);
  assert(stats->current_date_ptlog_files == 0);
  assert(stats->eligible_ptlog_files == 0);
  assert(stats->valid_month_directories == 0);
  assert(stats->max_month_ptlog_files == 0);
  assert(stats->max_month_name[0] == '\0');
}

static unsigned test_reclaim_candidates(const char* root,
                                        const char* current_date,
                                        unsigned max_deletes,
                                        bool fail_first_unlink,
                                        unsigned* free_checks_out)
{
  unsigned deleted = 0;
  unsigned fake_free_bytes = 0;
  const unsigned required_free_bytes = max_deletes;
  if (free_checks_out != NULL) *free_checks_out = 0;

  while (fake_free_bytes < required_free_bytes && deleted < max_deletes) {
    sd_ptlog_candidate_t candidate;
    if (!SdPtlogFindOldestCandidate(root, NULL, current_date, &candidate)) break;
    if (fail_first_unlink) break;
    assert(unlink(candidate.path) == 0);
    deleted++;
    fake_free_bytes++;
    if (free_checks_out != NULL) (*free_checks_out)++;
  }
  return deleted;
}

static void test_paths(void)
{
  char date[16], month[16], path[128];
  assert(SdPtlogBuildNestedPath("/sdcard", 1782432000, 0, date, sizeof(date), month, sizeof(month), path, sizeof(path)));
  assert(strcmp(date, "2026-06-26Z") == 0);
  assert(strcmp(month, "2026-06") == 0);
  assert(strcmp(path, "/sdcard/logs/2026-06/20260626.000") == 0);
  assert(SdPtlogBuildNestedPath("/sdcard", 1782432000, 1, date, sizeof(date), month, sizeof(month), path, sizeof(path)));
  assert(strcmp(path, "/sdcard/logs/2026-06/20260626.001") == 0);
  assert(SdPtlogBuildNestedPath("/sdcard", 1782432000, 12, date, sizeof(date), month, sizeof(month), path, sizeof(path)));
  assert(strcmp(path, "/sdcard/logs/2026-06/20260626.012") == 0);
  assert(SdPtlogBuildNestedPath("/sdcard", 1782432000, 999, date, sizeof(date), month, sizeof(month), path, sizeof(path)));
  assert(strcmp(path, "/sdcard/logs/2026-06/20260626.999") == 0);
  strcpy(path, "unchanged");
  assert(!SdPtlogBuildNestedPath("/sdcard", 1782432000, 1000, date, sizeof(date), month, sizeof(month), path, sizeof(path)));
  assert(path[0] == '\0');
  char tiny[10] = "unchanged";
  assert(!SdPtlogBuildNestedPath("/sdcard", 1782432000, 0, date, sizeof(date), month, sizeof(month), tiny, sizeof(tiny)));
  assert(tiny[0] == '\0');
}

static void test_parse(void)
{
  char date[16];
  uint32_t revision = 99;
  assert(SdPtlogParseName("20260626.000", date, sizeof(date), &revision));
  assert(strcmp(date, "2026-06-26Z") == 0 && revision == 0);
  assert(SdPtlogParseName("20260626.001", date, sizeof(date), &revision));
  assert(revision == 1);
  assert(SdPtlogParseName("20260626.012", date, sizeof(date), &revision));
  assert(revision == 12);
  assert(SdPtlogParseName("20260626.999", date, sizeof(date), &revision));
  assert(revision == 999);
  assert(SdPtlogParseName("20240229.000", date, sizeof(date), &revision));
  assert(strcmp(date, "2024-02-29Z") == 0 && revision == 0);
  assert(!SdPtlogParseName("20260626.00", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("20260626.0000", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026062.000", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("202606260.000", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026-06-26.000", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("20260626.A00", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("20260626.0X1", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("20261326.000", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("20260600.000", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("20260230.000", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("20230229.000", date, sizeof(date), &revision));
  assert(SdPtlogParseName("2026-06-26Z.ptlog", date, sizeof(date), &revision));
  assert(strcmp(date, "2026-06-26Z") == 0 && revision == 0);
  assert(SdPtlogParseName("2026-06-26Z-123.ptlog", date, sizeof(date), &revision));
  assert(revision == 123);
  assert(SdPtlogParseName("2026-06-23Z-4294967295.ptlog", date, sizeof(date), &revision));
  assert(revision == UINT32_MAX);
  assert(!SdPtlogParseName("2026-06-23Z-4294967296.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("random.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026-6-23Z.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026-06-23.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026-06-23Z-.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026-06-23Z-x.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026-06-23Z.csv", date, sizeof(date), &revision));
  assert(SdPtlogIsMonthDirectoryName("2026-06"));
  assert(!SdPtlogIsMonthDirectoryName("2026-6"));
  assert(!SdPtlogIsMonthDirectoryName("2026-AA"));
}

static void test_traversal(void)
{
  char templ[] = "/tmp/ptlog_paths_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/2024-01-01Z.ptlog", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/2026-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2026-01-01Z-1.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/random.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/.Trash-1000", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/FOUND.000", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/System Volume Information", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/unknown", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/unknown/2025-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month/2024-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/2024-01-01Z.ptlog", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/20250623.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/deeper", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/deeper/2020-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06/20260623.000", root); make_file(path);

  sd_ptlog_candidate_t candidate;
  assert(SdPtlogFindOldestCandidate(root, NULL, "2026-06-23Z", &candidate));
  assert(strcmp(candidate.date, "2025-06-23Z") == 0);
  assert(!candidate.legacy_root);

  char current_path[256];
  snprintf(current_path, sizeof(current_path), "%s/logs/2025-06/20250623.000", root);
  assert(SdPtlogFindOldestCandidate(root, current_path, "2026-06-23Z", &candidate));
  assert(strcmp(candidate.date, "2026-01-01Z") == 0);
  assert(candidate.legacy_root);
  assert(candidate.revision == 0);

}

static void test_current_date_only_is_protected(void)
{
  char templ[] = "/tmp/ptlog_paths_current_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/2026-06-23Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06/20260623.001", root); make_file(path);
  sd_ptlog_candidate_t candidate;
  assert(!SdPtlogFindOldestCandidate(root, NULL, "2026-06-23Z", &candidate));
}

static void test_reclaim_deletes_nested_when_root_has_no_candidate(void)
{
  char templ[] = "/tmp/ptlog_reclaim_nested_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/random.bin", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06", root); make_dir(path);
  char nested[256];
  snprintf(nested, sizeof(nested), "%s/logs/2025-06/20250623.000", root); make_file(nested);

  unsigned free_checks = 0;
  assert(test_reclaim_candidates(root, "2026-06-23Z", 1, false, &free_checks) == 1);
  assert(free_checks == 1);
  assert(!exists_path(nested));
  snprintf(path, sizeof(path), "%s/random.bin", root);
  assert(exists_path(path));
}

static void test_reclaim_prefers_older_legacy_root_candidate(void)
{
  char templ[] = "/tmp/ptlog_reclaim_root_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char root_candidate[256];
  snprintf(root_candidate, sizeof(root_candidate), "%s/2024-01-01Z.ptlog", root); make_file(root_candidate);
  char path[256];
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06", root); make_dir(path);
  char nested[256];
  snprintf(nested, sizeof(nested), "%s/logs/2025-06/20250623.000", root); make_file(nested);

  assert(test_reclaim_candidates(root, "2026-06-23Z", 1, false, NULL) == 1);
  assert(!exists_path(root_candidate));
  assert(exists_path(nested));
}

static void test_reclaim_safety_and_limits(void)
{
  char templ[] = "/tmp/ptlog_reclaim_limits_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/2023-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2023-01-02Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2026-06-23Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/keep.txt", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2022-01-01Z.ptlog", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/.Trash-1000", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/.Trash-1000/2020-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month/2021-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  char current_revision[256];
  snprintf(current_revision, sizeof(current_revision), "%s/logs/2026-06/2026-06-23Z-1.ptlog", root); make_file(current_revision);

  unsigned free_checks = 0;
  assert(test_reclaim_candidates(root, "2026-06-23Z", 2, false, &free_checks) == 2);
  assert(free_checks == 2);
  snprintf(path, sizeof(path), "%s/2023-01-01Z.ptlog", root);
  assert(!exists_path(path));
  snprintf(path, sizeof(path), "%s/2023-01-02Z.ptlog", root);
  assert(!exists_path(path));
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240101.000", root);
  assert(exists_path(path));
  snprintf(path, sizeof(path), "%s/keep.txt", root);
  assert(exists_path(path));
  snprintf(path, sizeof(path), "%s/2022-01-01Z.ptlog", root);
  assert(exists_path(path));
  snprintf(path, sizeof(path), "%s/.Trash-1000/2020-01-01Z.ptlog", root);
  assert(exists_path(path));
  snprintf(path, sizeof(path), "%s/logs/not-a-month/2021-01-01Z.ptlog", root);
  assert(exists_path(path));
  snprintf(path, sizeof(path), "%s/2026-06-23Z.ptlog", root);
  assert(exists_path(path));
  assert(exists_path(current_revision));
}

static void test_reclaim_unlink_failure_does_not_count(void)
{
  char templ[] = "/tmp/ptlog_reclaim_fail_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char candidate[256];
  snprintf(candidate, sizeof(candidate), "%s/2024-01-01Z.ptlog", root); make_file(candidate);

  assert(test_reclaim_candidates(root, "2026-06-23Z", 1, true, NULL) == 0);
  assert(exists_path(candidate));
}

static void test_stats_empty_and_missing_layout(void)
{
  char templ[] = "/tmp/ptlog_stats_empty_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);

  sd_ptlog_stats_t stats;
  memset(&stats, 0xA5, sizeof(stats));
  assert(SdPtlogCollectStats(root, NULL, "2026-06-23Z", &stats));
  assert_zero_stats(&stats);

  char logs[256];
  snprintf(logs, sizeof(logs), "%s/logs", root); make_dir(logs);
  assert(SdPtlogCollectStats(root, NULL, "2026-06-23Z", &stats));
  assert_zero_stats(&stats);

  char long_root[256];
  char long_name[120];
  memset(long_name, 'a', sizeof(long_name) - 1);
  long_name[sizeof(long_name) - 1] = '\0';
  snprintf(long_root, sizeof(long_root), "%s/%s", root, long_name);
  make_dir(long_root);
  memset(&stats, 0xA5, sizeof(stats));
  assert(!SdPtlogCollectStats(long_root, NULL, "2026-06-23Z", &stats));
  assert_zero_stats(&stats);
}

static void test_stats_legacy_root_counting(void)
{
  char templ[] = "/tmp/ptlog_stats_root_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/2024-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2024-01-02Z-2.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2024-01-03Z.ptlog", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/2024-1-03Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/keep.txt", root); make_file(path);
  snprintf(path, sizeof(path), "%s/.Trash-1000", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/.Trash-1000/2020-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/unknown", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/unknown/2020-01-01Z.ptlog", root); make_file(path);

  sd_ptlog_stats_t stats;
  assert(SdPtlogCollectStats(root, NULL, "2026-06-23Z", &stats));
  assert(stats.total_ptlog_files == 2);
  assert(stats.legacy_root_ptlog_files == 2);
  assert(stats.nested_month_ptlog_files == 0);
  assert(stats.current_date_ptlog_files == 0);
  assert(stats.eligible_ptlog_files == 2);
  assert(stats.valid_month_directories == 0);
  assert(stats.max_month_ptlog_files == 0);
  assert(stats.max_month_name[0] == '\0');
}

static void test_stats_nested_month_counting_and_ignored_paths(void)
{
  char templ[] = "/tmp/ptlog_stats_nested_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/2023-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2020-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month/2020-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/2024-01-02Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240102.001", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/notes.txt", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/deeper", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/deeper/2020-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-02", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-02/2024-02-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-02/2024-02-02Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-03", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-03/2024-03-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-03/2024-03-02Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-03/2024-03-03Z.ptlog", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/FOUND.000", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/FOUND.000/2020-01-01Z.ptlog", root); make_file(path);

  sd_ptlog_stats_t stats;
  assert(SdPtlogCollectStats(root, NULL, "2026-06-23Z", &stats));
  assert(stats.total_ptlog_files == 8);
  assert(stats.legacy_root_ptlog_files == 1);
  assert(stats.nested_month_ptlog_files == 7);
  assert(stats.current_date_ptlog_files == 0);
  assert(stats.eligible_ptlog_files == 8);
  assert(stats.valid_month_directories == 3);
  assert(stats.max_month_ptlog_files == 3);
  assert(strcmp(stats.max_month_name, "2024-01") == 0);
}

static void test_stats_valid_month_regular_file_is_ignored(void)
{
  char templ[] = "/tmp/ptlog_stats_month_file_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-04", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-05", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-05/20240501.000", root); make_file(path);

  sd_ptlog_stats_t stats;
  assert(SdPtlogCollectStats(root, NULL, "2026-06-23Z", &stats));
  assert(stats.total_ptlog_files == 1);
  assert(stats.legacy_root_ptlog_files == 0);
  assert(stats.nested_month_ptlog_files == 1);
  assert(stats.current_date_ptlog_files == 0);
  assert(stats.eligible_ptlog_files == 1);
  assert(stats.valid_month_directories == 1);
  assert(stats.max_month_ptlog_files == 1);
  assert(strcmp(stats.max_month_name, "2024-05") == 0);
}

static void test_stats_current_date_and_path_separation(void)
{
  char templ[] = "/tmp/ptlog_stats_current_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/2024-01-01Z.ptlog", root); make_file(path);
  char current_path[256];
  snprintf(current_path, sizeof(current_path), "%s/2024-01-02Z.ptlog", root); make_file(current_path);
  snprintf(path, sizeof(path), "%s/2026-06-23Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06/20260623.001", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/20250623.000", root); make_file(path);

  sd_ptlog_stats_t stats;
  assert(SdPtlogCollectStats(root, current_path, "2026-06-23Z", &stats));
  assert(stats.total_ptlog_files == 5);
  assert(stats.legacy_root_ptlog_files == 3);
  assert(stats.nested_month_ptlog_files == 2);
  assert(stats.current_date_ptlog_files == 2);
  assert(stats.eligible_ptlog_files == 2);
  assert(stats.valid_month_directories == 2);
  assert(stats.max_month_ptlog_files == 1);
  assert(strcmp(stats.max_month_name, "2025-06") == 0);
}

int main(void)
{
  test_paths();
  test_parse();
  test_traversal();
  test_current_date_only_is_protected();
  test_reclaim_deletes_nested_when_root_has_no_candidate();
  test_reclaim_prefers_older_legacy_root_candidate();
  test_reclaim_safety_and_limits();
  test_reclaim_unlink_failure_does_not_count();
  test_stats_empty_and_missing_layout();
  test_stats_legacy_root_counting();
  test_stats_nested_month_counting_and_ignored_paths();
  test_stats_valid_month_regular_file_is_ignored();
  test_stats_current_date_and_path_separation();
  puts("sd_ptlog_paths tests passed");
  return 0;
}
