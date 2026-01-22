#ifndef PT100_LOGGER_HEAP_EVENT_LOG_H_
#define PT100_LOGGER_HEAP_EVENT_LOG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute HeapEventLog.
 * @param event_name Parameter event_name.
 * @param detail Parameter detail.
 * @param code Parameter code.
 */
void HeapEventLog(const char* event_name, const char* detail, int32_t code);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_HEAP_EVENT_LOG_H_
