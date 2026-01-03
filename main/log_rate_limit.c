#include "log_rate_limit.h"

#include "esp_timer.h"

bool
LogRateLimitAllow(uint32_t* last_ms, uint32_t period_ms)
{
  if (period_ms == 0) {
    return true;
  }
  if (last_ms == NULL) {
    return false;
  }
  const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
  if (*last_ms == 0 || (uint32_t)(now_ms - *last_ms) >= period_ms) {
    *last_ms = now_ms;
    return true;
  }
  return false;
}
