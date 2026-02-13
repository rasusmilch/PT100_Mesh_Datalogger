#ifndef PT100_LOGGER_DISPLAY_ATTENTION_H_
#define PT100_LOGGER_DISPLAY_ATTENTION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  kDispAttnItemSdOut = 0,
  kDispAttnItemSdIo = 1,
  kDispAttnItemFramOvr = 2,
  kDispAttnItemRtdFault = 3,
  kDispAttnItemTimeBad = 4,
  kDispAttnItemMeshDown = 5,
  kDispAttnItemHeap = 6,
  kDispAttnItemNtpFail = 7,
  kDispAttnItemSdSpace = 8,
  kDispAttnItemCount,
} display_attention_item_t;

typedef enum
{
  DISP_SEV_OFF = 0,
  DISP_SEV_WARN = 1,
  DISP_SEV_ERROR = 2,
} display_attention_severity_t;

typedef enum
{
  kDispAttnNone = 0,
  kDispAttnSdOut = 1u << kDispAttnItemSdOut,
  kDispAttnSdIo = 1u << kDispAttnItemSdIo,
  kDispAttnFramOvr = 1u << kDispAttnItemFramOvr,
  kDispAttnRtdFault = 1u << kDispAttnItemRtdFault,
  kDispAttnTimeBad = 1u << kDispAttnItemTimeBad,
  kDispAttnMeshDown = 1u << kDispAttnItemMeshDown,
  kDispAttnHeap = 1u << kDispAttnItemHeap,
  kDispAttnNtpFail = 1u << kDispAttnItemNtpFail,
  kDispAttnSdSpace = 1u << kDispAttnItemSdSpace,
} display_attention_bit_t;

typedef uint32_t display_attention_mask_t;

#define DISPLAY_ATTENTION_POLICY_BITS_PER_ITEM 2u
#define DISPLAY_ATTENTION_POLICY_MASK 0x03u

/**
 * @brief Execute DisplayAttentionPolicyGet.
 * @param policy Parameter policy.
 * @param item Parameter item.
 * @return Return the function result.
 */
static inline display_attention_severity_t
DisplayAttentionPolicyGet(uint32_t policy, display_attention_item_t item)
{
  const uint32_t shift =
    DISPLAY_ATTENTION_POLICY_BITS_PER_ITEM * (uint32_t)item;
  return (display_attention_severity_t)(
    (policy >> shift) & DISPLAY_ATTENTION_POLICY_MASK);
}

/**
 * @brief Execute DisplayAttentionPolicySet.
 * @param policy Parameter policy.
 * @param item Parameter item.
 * @param severity Parameter severity.
 * @return Return the function result.
 */
static inline uint32_t
DisplayAttentionPolicySet(uint32_t policy,
                          display_attention_item_t item,
                          display_attention_severity_t severity)
{
  const uint32_t shift =
    DISPLAY_ATTENTION_POLICY_BITS_PER_ITEM * (uint32_t)item;
  const uint32_t clear_mask =
    DISPLAY_ATTENTION_POLICY_MASK << shift;
  policy &= ~clear_mask;
  policy |= ((uint32_t)severity & DISPLAY_ATTENTION_POLICY_MASK) << shift;
  return policy;
}

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DISPLAY_ATTENTION_H_
