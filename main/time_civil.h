#ifndef PT100_LOGGER_TIME_CIVIL_H_
#define PT100_LOGGER_TIME_CIVIL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum
  {
    CAL_DUE_UNIT_DAYS = 1,
    CAL_DUE_UNIT_MONTHS = 2,
    CAL_DUE_UNIT_YEARS = 3,
  } cal_due_unit_t;

  int64_t CalComputeDueDateUtc(int64_t last_utc_epoch,
                               uint16_t count,
                               cal_due_unit_t unit);
  int64_t GetUtcMidnightEpochNow(void);
  bool TimeCivilUtcEpochFromDate(int year,
                                 int month,
                                 int day,
                                 int64_t* out_epoch);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_TIME_CIVIL_H_
