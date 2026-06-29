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
static sd_ptlog_path_workspace_t* make_workspace(void)
{
  sd_ptlog_path_workspace_t* workspace =
    (sd_ptlog_path_workspace_t*)calloc(1, sizeof(*workspace));
  assert(workspace != NULL);
  return workspace;
}
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

static void test_workspace_paths(void)
{
  sd_ptlog_path_workspace_t* workspace = make_workspace();
  char date[16];
  char month[16];
  char path[128];
  assert(SdPtlogBuildNestedPathWithWorkspace(workspace,
                                             "/sdcard",
                                             1782432000,
                                             12,
                                             date,
                                             sizeof(date),
                                             month,
                                             sizeof(month),
                                             path,
                                             sizeof(path)));
  assert(strcmp(date, "2026-06-26Z") == 0);
  assert(strcmp(month, "2026-06") == 0);
  assert(strcmp(path, "/sdcard/logs/2026-06/20260626.012") == 0);

  strcpy(path, "unchanged");
  assert(!SdPtlogBuildNestedPathWithWorkspace(workspace,
                                              "/sdcard",
                                              1782432000,
                                              1000,
                                              date,
                                              sizeof(date),
                                              month,
                                              sizeof(month),
                                              path,
                                              sizeof(path)));
  assert(path[0] == '\0');

  strcpy(path, "unchanged");
  assert(!SdPtlogBuildNestedPathWithWorkspace(NULL,
                                              "/sdcard",
                                              1782432000,
                                              0,
                                              date,
                                              sizeof(date),
                                              month,
                                              sizeof(month),
                                              path,
                                              sizeof(path)));
  assert(path[0] == '\0');
  free(workspace);
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
  assert(!SdPtlogParseName("2026-06-26Z.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026-06-26Z-123.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("random.ptlog", date, sizeof(date), &revision));
  assert(!SdPtlogParseName("2026-06-23Z.csv", date, sizeof(date), &revision));
  assert(SdPtlogIsMonthDirectoryName("2026-06"));
  assert(!SdPtlogIsMonthDirectoryName("2026-6"));
  assert(!SdPtlogIsMonthDirectoryName("2026-AA"));
}

static void test_traversal_compact_nested_only(void)
{
  char templ[] = "/tmp/ptlog_paths_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/20240101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2024-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/random.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/unknown", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/unknown/20200101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month/20200101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/2024-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20230101.000", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/deeper", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/deeper/20200101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/20250623.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/20250623.001", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06/20260623.000", root); make_file(path);

  sd_ptlog_candidate_t candidate;
  assert(SdPtlogFindOldestCandidate(root, NULL, "2026-06-23Z", &candidate));
  assert(strcmp(candidate.date, "2024-01-01Z") == 0);
  assert(strcmp(candidate.name, "20240101.000") == 0);
  assert(!candidate.legacy_root);

  char current_path[256];
  snprintf(current_path, sizeof(current_path), "%s/logs/2024-01/20240101.000", root);
  assert(SdPtlogFindOldestCandidate(root, current_path, "2026-06-23Z", &candidate));
  assert(strcmp(candidate.date, "2025-06-23Z") == 0);
  assert(candidate.revision == 0);
}

static void test_workspace_traversal_and_stats(void)
{
  char templ[] = "/tmp/ptlog_workspace_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/20230101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2023-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month/20200101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240102.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06/20260623.000", root); make_file(path);

  sd_ptlog_path_workspace_t* workspace = make_workspace();
  sd_ptlog_candidate_t candidate;
  assert(SdPtlogFindOldestCandidateWithWorkspace(
    workspace, root, NULL, "2026-06-23Z", &candidate));
  assert(strcmp(candidate.date, "2024-01-01Z") == 0);
  assert(strcmp(candidate.name, "20240101.000") == 0);

  sd_ptlog_stats_t stats;
  assert(SdPtlogCollectStatsWithWorkspace(
    workspace, root, NULL, "2026-06-23Z", &stats));
  assert(stats.total_ptlog_files == 3);
  assert(stats.current_date_ptlog_files == 1);
  assert(stats.eligible_ptlog_files == 2);
  assert(stats.valid_month_directories == 2);

  assert(!SdPtlogFindOldestCandidateWithWorkspace(
    NULL, root, NULL, "2026-06-23Z", &candidate));
  assert(!SdPtlogCollectStatsWithWorkspace(
    NULL, root, NULL, "2026-06-23Z", &stats));
  free(workspace);
}

static void test_current_date_only_is_protected(void)
{
  char templ[] = "/tmp/ptlog_paths_current_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/2026-06-23Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/20260623.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06/20260623.001", root); make_file(path);
  sd_ptlog_candidate_t candidate;
  assert(!SdPtlogFindOldestCandidate(root, NULL, "2026-06-23Z", &candidate));
}

static void test_reclaim_deletes_only_nested_compact(void)
{
  char templ[] = "/tmp/ptlog_reclaim_nested_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/20240101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2024-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/2024-01-01Z.ptlog", root); make_file(path);
  char nested[256];
  snprintf(nested, sizeof(nested), "%s/logs/2024-01/20240101.000", root); make_file(nested);

  unsigned free_checks = 0;
  assert(test_reclaim_candidates(root, "2026-06-23Z", 1, false, &free_checks) == 1);
  assert(free_checks == 1);
  assert(!exists_path(nested));
  snprintf(path, sizeof(path), "%s/20240101.000", root);
  assert(exists_path(path));
  snprintf(path, sizeof(path), "%s/2024-01-01Z.ptlog", root);
  assert(exists_path(path));
  snprintf(path, sizeof(path), "%s/logs/2024-01/2024-01-01Z.ptlog", root);
  assert(exists_path(path));
}

static void test_reclaim_unlink_failure_does_not_count(void)
{
  char templ[] = "/tmp/ptlog_reclaim_fail_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  char candidate[256];
  snprintf(candidate, sizeof(candidate), "%s/logs/2024-01/20240101.000", root); make_file(candidate);

  assert(test_reclaim_candidates(root, "2026-06-23Z", 1, true, NULL) == 0);
  assert(exists_path(candidate));
}


static void test_next_revision_accumulator(void)
{
  uint32_t next_revision = 0;
  assert(next_revision == 0);
  assert(SdPtlogAccumulateNextRevision(0, &next_revision));
  assert(next_revision == 1);
  assert(SdPtlogAccumulateNextRevision(998, &next_revision));
  assert(next_revision == 999);
  assert(!SdPtlogAccumulateNextRevision(999, &next_revision));
  assert(next_revision == 999);
  assert(!SdPtlogAccumulateNextRevision(0, NULL));
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
}

static void test_stats_nested_compact_only(void)
{
  char templ[] = "/tmp/ptlog_stats_nested_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/20230101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2023-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2020-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/not-a-month/20200101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/20240102.001", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/2024-01-02Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/notes.txt", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/deeper", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01/deeper/20200101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-02", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-02/20240201.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2024-03", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-03/20240301.000", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/FOUND.000", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/FOUND.000/20200101.000", root); make_file(path);

  sd_ptlog_stats_t stats;
  assert(SdPtlogCollectStats(root, NULL, "2026-06-23Z", &stats));
  assert(stats.total_ptlog_files == 3);
  assert(stats.legacy_root_ptlog_files == 0);
  assert(stats.nested_month_ptlog_files == 3);
  assert(stats.current_date_ptlog_files == 0);
  assert(stats.eligible_ptlog_files == 3);
  assert(stats.valid_month_directories == 3);
  assert(stats.max_month_ptlog_files == 2);
  assert(strcmp(stats.max_month_name, "2024-01") == 0);
}

static void test_stats_current_date_and_path_separation(void)
{
  char templ[] = "/tmp/ptlog_stats_current_XXXXXX";
  char* root = mkdtemp(templ);
  assert(root != NULL);
  char path[256];
  snprintf(path, sizeof(path), "%s/20240101.000", root); make_file(path);
  snprintf(path, sizeof(path), "%s/2024-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2024-01", root); make_dir(path);
  char current_path[256];
  snprintf(current_path, sizeof(current_path), "%s/logs/2024-01/20240102.000", root); make_file(current_path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06/20260623.001", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/20250623.000", root); make_file(path);

  sd_ptlog_stats_t stats;
  assert(SdPtlogCollectStats(root, current_path, "2026-06-23Z", &stats));
  assert(stats.total_ptlog_files == 3);
  assert(stats.legacy_root_ptlog_files == 0);
  assert(stats.nested_month_ptlog_files == 3);
  assert(stats.current_date_ptlog_files == 1);
  assert(stats.eligible_ptlog_files == 1);
  assert(stats.valid_month_directories == 3);
  assert(stats.max_month_ptlog_files == 1);
  assert(strcmp(stats.max_month_name, "2024-01") == 0);
}

int main(void)
{
  test_paths();
  test_workspace_paths();
  test_parse();
  test_traversal_compact_nested_only();
  test_workspace_traversal_and_stats();
  test_current_date_only_is_protected();
  test_reclaim_deletes_only_nested_compact();
  test_reclaim_unlink_failure_does_not_count();
  test_next_revision_accumulator();
  test_stats_empty_and_missing_layout();
  test_stats_nested_compact_only();
  test_stats_current_date_and_path_separation();
  puts("sd_ptlog_paths tests passed");
  return 0;
}
