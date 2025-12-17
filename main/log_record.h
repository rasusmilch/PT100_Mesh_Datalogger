#ifndef PT100_LOGGER_LOG_RECORD_H_
#define PT100_LOGGER_LOG_RECORD_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Fixed on-media record format. Keep this stable once you start generating log
// files.
#define LOG_RECORD_MAGIC 0x544C4F47u // 'TLOG'

  // Record flags.
  typedef enum
  {
    LOG_RECORD_FLAG_TIME_VALID = 1u << 0,
    LOG_RECORD_FLAG_CAL_VALID = 1u << 1,
    LOG_RECORD_FLAG_SD_ERROR = 1u << 2,
    LOG_RECORD_FLAG_MESH_CONNECTED = 1u << 3,
    LOG_RECORD_FLAG_SENSOR_FAULT = 1u << 4,
    LOG_RECORD_FLAG_FRAM_FULL = 1u << 5,
  } log_record_flags_t;

#pragma pack(push, 1)
  typedef struct
  {
    uint32_t magic;               // LOG_RECORD_MAGIC
    uint32_t sequence;            // Monotonic counter (wrap ok).
    int64_t timestamp_epoch_sec;  // UNIX epoch seconds (UTC). 0 if unknown.
    int32_t timestamp_millis;     // 0..999
    int32_t raw_temp_milli_c;     // Uncalibrated temperature (milli-°C)
    int32_t temp_milli_c;         // Calibrated temperature (milli-°C)
    int32_t resistance_milli_ohm; // PT100 resistance (milli-ohm)
    uint16_t flags;               // log_record_flags_t
    uint16_t crc16_ccitt;         // CRC16-CCITT over all prior bytes.
  } log_record_t;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_LOG_RECORD_H_
