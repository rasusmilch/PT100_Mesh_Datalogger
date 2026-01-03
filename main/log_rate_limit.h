#ifndef PT100_LOGGER_LOG_RATE_LIMIT_H_
#define PT100_LOGGER_LOG_RATE_LIMIT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute LogRateLimitAllow.
 * @param last_ms Parameter last_ms.
 * @param period_ms Parameter period_ms.
 * @return Return the function result.
 */
bool LogRateLimitAllow(uint32_t* last_ms, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_LOG_RATE_LIMIT_H_
