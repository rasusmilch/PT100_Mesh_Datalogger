#include "time_civil.h"

#include <string.h>
#include <time.h>

static bool
IsLeapYear(int year)
{
  return ((year % 4) == 0 && (year % 100) != 0) || ((year % 400) == 0);
}

static int
DaysInMonth(int year, int month)
{
  static const int kDaysByMonth[12] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
  };
  if (month < 1 || month > 12) {
    return 0;
  }
  if (month == 2 && IsLeapYear(year)) {
    return 29;
  }
  return kDaysByMonth[month - 1];
}

static int64_t
DaysFromCivil(int year, unsigned month, unsigned day)
{
  // Howard Hinnant's algorithm: days since 1970-01-01.
  year -= (month <= 2);
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400);
  const unsigned doy =
    (153U * (month + (month > 2 ? (unsigned)-3 : 9)) + 2U) / 5U + day - 1U;
  const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static int64_t
UtcTmToEpochSeconds(const struct tm* utc_tm)
{
  if (utc_tm == NULL) {
    return 0;
  }

  const int year = utc_tm->tm_year + 1900;
  const unsigned month = (unsigned)(utc_tm->tm_mon + 1);
  const unsigned day = (unsigned)utc_tm->tm_mday;

  const int64_t days = DaysFromCivil(year, month, day);
  return days * 86400LL + (int64_t)utc_tm->tm_hour * 3600LL +
         (int64_t)utc_tm->tm_min * 60LL + (int64_t)utc_tm->tm_sec;
}

bool
TimeCivilUtcEpochFromDate(int year, int month, int day, int64_t* out_epoch)
{
  if (out_epoch == NULL) {
    return false;
  }
  const int dim = DaysInMonth(year, month);
  if (dim == 0 || day < 1 || day > dim) {
    return false;
  }
  struct tm utc_tm;
  memset(&utc_tm, 0, sizeof(utc_tm));
  utc_tm.tm_year = year - 1900;
  utc_tm.tm_mon = month - 1;
  utc_tm.tm_mday = day;
  *out_epoch = UtcTmToEpochSeconds(&utc_tm);
  return true;
}

int64_t
CalComputeDueDateUtc(int64_t last_utc_epoch,
                     uint16_t count,
                     cal_due_unit_t unit)
{
  if (last_utc_epoch <= 0 || count == 0 ||
      unit < CAL_DUE_UNIT_DAYS || unit > CAL_DUE_UNIT_YEARS) {
    return 0;
  }

  time_t last_time = (time_t)last_utc_epoch;
  struct tm last_tm;
  if (gmtime_r(&last_time, &last_tm) == NULL) {
    return 0;
  }

  int year = last_tm.tm_year + 1900;
  int month = last_tm.tm_mon + 1;
  int day = last_tm.tm_mday;

  if (unit == CAL_DUE_UNIT_DAYS) {
    int64_t base_epoch = 0;
    if (!TimeCivilUtcEpochFromDate(year, month, day, &base_epoch)) {
      return 0;
    }
    return base_epoch + (int64_t)count * 86400LL;
  }

  if (unit == CAL_DUE_UNIT_MONTHS) {
    int total_months = (year * 12) + (month - 1) + (int)count;
    year = total_months / 12;
    month = (total_months % 12) + 1;
  } else if (unit == CAL_DUE_UNIT_YEARS) {
    year += (int)count;
  }

  const int dim = DaysInMonth(year, month);
  if (dim == 0) {
    return 0;
  }
  if (day > dim) {
    day = dim;
  }

  int64_t due_epoch = 0;
  if (!TimeCivilUtcEpochFromDate(year, month, day, &due_epoch)) {
    return 0;
  }
  return due_epoch;
}

int64_t
GetUtcMidnightEpochNow(void)
{
  const time_t now = time(NULL);
  struct tm utc_tm;
  if (gmtime_r(&now, &utc_tm) == NULL) {
    return 0;
  }
  utc_tm.tm_hour = 0;
  utc_tm.tm_min = 0;
  utc_tm.tm_sec = 0;
  return UtcTmToEpochSeconds(&utc_tm);
}
