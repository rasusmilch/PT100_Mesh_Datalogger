#ifndef PT100_LOGGER_DISPLAY_ATTENTION_H_
#define PT100_LOGGER_DISPLAY_ATTENTION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  kDispAttnNone = 0,
  kDispAttnSdOut = 1u << 0,
  kDispAttnSdIo = 1u << 1,
  kDispAttnFramOvr = 1u << 2,
  kDispAttnRtdFault = 1u << 3,
  kDispAttnTimeBad = 1u << 4,
  kDispAttnMeshDown = 1u << 5,
} display_attention_bit_t;

typedef uint32_t display_attention_mask_t;

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DISPLAY_ATTENTION_H_
