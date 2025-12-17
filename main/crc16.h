#ifndef PT100_LOGGER_CRC16_H_
#define PT100_LOGGER_CRC16_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  // CRC-16/CCITT-FALSE:
  // - poly 0x1021
  // - init 0xFFFF
  // - xorout 0x0000
  uint16_t Crc16CcittFalse(const void* data, size_t length_bytes);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_CRC16_H_
