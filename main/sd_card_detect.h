#ifndef PT100_LOGGER_SD_CARD_DETECT_H_
#define PT100_LOGGER_SD_CARD_DETECT_H_

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    int gpio;
    bool enabled;
    bool present_level_high;
    uint32_t debounce_ms;
    bool present_debounced;
    bool last_raw_present;
    TickType_t last_raw_change_ticks;
    bool initialized;
  } sd_card_detect_t;

/**
 * @brief Execute SdCardDetectInit.
 * @param detect Parameter detect.
 */
  void SdCardDetectInit(sd_card_detect_t* detect);

/**
 * @brief Execute SdCardDetectPoll.
 * @param detect Parameter detect.
 * @param out_changed Parameter out_changed.
 * @return Return the function result.
 */
  bool SdCardDetectPoll(sd_card_detect_t* detect, bool* out_changed);

/**
 * @brief Execute SdCardDetectIsEnabled.
 * @param detect Parameter detect.
 * @return Return the function result.
 */
  bool SdCardDetectIsEnabled(const sd_card_detect_t* detect);

/**
 * @brief Execute SdCardDetectIsPresent.
 * @param detect Parameter detect.
 * @return Return the function result.
 */
  bool SdCardDetectIsPresent(const sd_card_detect_t* detect);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_SD_CARD_DETECT_H_
