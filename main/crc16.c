#include "crc16.h"

/**
 * @brief Execute Crc16CcittFalse.
 * @param data Parameter data.
 * @param length_bytes Parameter length_bytes.
 * @return Return the function result.
 */
uint16_t
Crc16CcittFalse(const void* data, size_t length_bytes)
{
  const uint8_t* bytes = (const uint8_t*)data;
  uint16_t crc = 0xFFFFu;

  for (size_t index = 0; index < length_bytes; ++index) {
    crc ^= (uint16_t)bytes[index] << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}
