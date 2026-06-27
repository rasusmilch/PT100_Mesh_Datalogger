#include "ptlog_format.h"
#include "data_csv.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

bool CsvWriteHeader(csv_write_fn_t writer, void* context)
{
  return writer("csv_header\n", strlen("csv_header\n"), context);
}

static ptlog_header_t make_header(void)
{
  ptlog_header_t header;
  memset(&header, 0, sizeof(header));
  strcpy(header.created_utc, "2026-06-26T00:00:00Z");
  strcpy(header.device_serial, "serial");
  strcpy(header.device_mac, "00:00:00:00:00:00");
  strcpy(header.device_role, "root");
  strcpy(header.firmware_project, "pt100");
  strcpy(header.firmware_version, "test");
  strcpy(header.firmware_build_date, "2026-06-26");
  strcpy(header.firmware_build_time, "00:00:00");
  strcpy(header.esp_idf_ver, "host");
  strcpy(header.timezone_posix, "UTC0");
  strcpy(header.cal_last_utc, "never");
  strcpy(header.cal_due_rule, "none");
  strcpy(header.cal_due_utc, "never");
  strcpy(header.cal_points_count, "0");
  strcpy(header.cal_applied, "0");
  strcpy(header.cal_method, "none");
  strcpy(header.cal_context, "host-test");
  return header;
}

static void test_magic_line(void)
{
  assert(strcmp(PTLOG_MAGIC_TEXT, "#PT100_LOG_V1") == 0);
  assert(PtlogIsMagicLine("#PT100_LOG_V1\n"));
  assert(!PtlogIsMagicLine(NULL));
  assert(!PtlogIsMagicLine("#PT100_LOG_V1"));
  assert(!PtlogIsMagicLine("#PT100_LOG_V2\n"));
  assert(!PtlogIsMagicLine("#PT100\n"));
}

static void test_write_header_magic(void)
{
  FILE* file = tmpfile();
  assert(file != NULL);
  ptlog_header_t header = make_header();
  assert(PtlogWriteHeader(file, &header));
  rewind(file);
  char line[128];
  assert(fgets(line, sizeof(line), file) != NULL);
  assert(strcmp(line, PTLOG_MAGIC_LINE) == 0);
  assert(fgets(line, sizeof(line), file) != NULL);
  assert(strcmp(line, "# header_version=1\n") == 0);
  bool saw_end = false;
  bool saw_csv = false;
  while (fgets(line, sizeof(line), file) != NULL) {
    if (strcmp(line, "#END_HEADER\n") == 0) saw_end = true;
    if (strcmp(line, "csv_header\n") == 0) saw_csv = true;
  }
  assert(saw_end);
  assert(saw_csv);
  fclose(file);
}

int main(void)
{
  test_magic_line();
  test_write_header_magic();
  puts("ptlog_format tests passed");
  return 0;
}
