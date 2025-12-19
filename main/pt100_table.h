#ifndef PT100_LOGGER_PT100_TABLE_H_
#define PT100_LOGGER_PT100_TABLE_H_

#include <stddef.h>
#include <stdint.h>

#define PT100_TABLE_LENGTH 1051
#define PT100_TABLE_MIN_C (-200.0)
#define PT100_TABLE_MAX_C (850.0)

extern const uint16_t kPt100TableOhmsX100[PT100_TABLE_LENGTH];
extern const size_t kPt100TableLength;

#endif // PT100_LOGGER_PT100_TABLE_H_
