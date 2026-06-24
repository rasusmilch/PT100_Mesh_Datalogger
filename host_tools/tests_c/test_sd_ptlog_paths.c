#include "sd_ptlog_paths.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static void make_dir(const char* path) { assert(mkdir(path, 0777) == 0 || access(path, F_OK) == 0); }
static void make_file(const char* path) { FILE* f = fopen(path, "wb"); assert(f != NULL); fclose(f); }

static void test_paths(void)
{
  char date[16], month[16], path[128];
  assert(SdPtlogBuildNestedPath("/sdcard", 1782172800, 0, date, sizeof(date), month, sizeof(month), path, sizeof(path)));
  assert(strcmp(date, "2026-06-23Z") == 0);
  assert(strcmp(month, "2026-06") == 0);
  assert(strcmp(path, "/sdcard/logs/2026-06/2026-06-23Z.ptlog") == 0);
  assert(SdPtlogBuildNestedPath("/sdcard", 1782172800, 12, date, sizeof(date), month, sizeof(month), path, sizeof(path)));
  assert(strcmp(path, "/sdcard/logs/2026-06/2026-06-23Z-12.ptlog") == 0);
  char tiny[10] = "unchanged";
  assert(!SdPtlogBuildNestedPath("/sdcard", 1782172800, 0, date, sizeof(date), month, sizeof(month), tiny, sizeof(tiny)));
  assert(tiny[0] == '\0');
}

static void test_parse(void)
{
  char date[16];
  uint32_t revision = 99;
  assert(SdPtlogParseName("2026-06-23Z.ptlog", date, sizeof(date), &revision));
  assert(strcmp(date, "2026-06-23Z") == 0 && revision == 0);
  assert(SdPtlogParseName("2026-06-23Z-123.ptlog", date, sizeof(date), &revision));
  assert(revision == 123);
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
  snprintf(path, sizeof(path), "%s/logs/2025-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/2025-06-23Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/deeper", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2025-06/deeper/2020-01-01Z.ptlog", root); make_file(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06", root); make_dir(path);
  snprintf(path, sizeof(path), "%s/logs/2026-06/2026-06-23Z.ptlog", root); make_file(path);

  sd_ptlog_candidate_t candidate;
  assert(SdPtlogFindOldestCandidate(root, NULL, "2026-06-23Z", &candidate));
  assert(strcmp(candidate.date, "2025-06-23Z") == 0);
  assert(!candidate.legacy_root);

  char current_path[256];
  snprintf(current_path, sizeof(current_path), "%s/logs/2025-06/2025-06-23Z.ptlog", root);
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
  snprintf(path, sizeof(path), "%s/logs/2026-06/2026-06-23Z-1.ptlog", root); make_file(path);
  sd_ptlog_candidate_t candidate;
  assert(!SdPtlogFindOldestCandidate(root, NULL, "2026-06-23Z", &candidate));
}

int main(void)
{
  test_paths();
  test_parse();
  test_traversal();
  test_current_date_only_is_protected();
  puts("sd_ptlog_paths tests passed");
  return 0;
}
