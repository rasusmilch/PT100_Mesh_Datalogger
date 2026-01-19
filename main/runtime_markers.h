#ifndef PT100_LOGGER_RUNTIME_MARKERS_H_
#define PT100_LOGGER_RUNTIME_MARKERS_H_

#include <stdint.h>

#include "esp_system.h"
#include "runtime_state.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum
  {
    RUNTIME_MARKER_NONE = 0,
    STORAGE_010_BEFORE_QUEUE_RECV = 10,
    STORAGE_020_AFTER_QUEUE_RECV = 20,
    STORAGE_030_ASSIGN_IDS = 30,
    STORAGE_040_ALERT_MANAGER = 40,
    STORAGE_050_FRAM_APPEND = 50,
    STORAGE_060_EXPORT_UART_ENQUEUE = 60,
    STORAGE_070_MQTT_ENQUEUE = 70,
    SD_010_DETECT_POLL = 110,
    SD_020_MAINTENANCE = 120,
    SD_030_SCHEDULER = 130,
    SD_040_FLUSH_WORKER_ENTER = 140,
    SD_050_FLUSH_WORKER_EXIT = 150,
  } runtime_marker_id_t;

/**
 * @brief Update StorageTask marker state.
 * @param state Runtime state pointer.
 * @param marker Marker ID to record.
 */
  void RuntimeMarkersSetStorage(runtime_state_t* state,
                                runtime_marker_id_t marker);

/**
 * @brief Update SdFlushTask marker state.
 * @param state Runtime state pointer.
 * @param marker Marker ID to record.
 */
  void RuntimeMarkersSetSdFlush(runtime_state_t* state,
                                runtime_marker_id_t marker);

/**
 * @brief Dump persistent markers on boot.
 */
  void RuntimeMarkersDumpOnBoot(void);

/**
 * @brief Dump persistent and live markers.
 * @param state Runtime state pointer (may be NULL).
 */
  void RuntimeMarkersDump(const runtime_state_t* state);

/**
 * @brief Clear persistent and live markers.
 */
  void RuntimeMarkersClear(void);

/**
 * @brief Convert reset reason to a human-readable string.
 * @param reason Reset reason enum value.
 * @return String label for reset reason.
 */
  const char* RuntimeMarkersResetReasonToString(esp_reset_reason_t reason);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUNTIME_MARKERS_H_
