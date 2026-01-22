#include "net_stack.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "heap_phase_log.h"

static const char* kTag = "net_stack";
static bool s_initialized = false;

/**
 * @brief Execute NetStackInitOnce.
 * @return Return the function result.
 */
esp_err_t
NetStackInitOnce(void)
{
  if (s_initialized) {
    return ESP_OK;
  }

  HeapLogPhase("wifi_netif_init_before");
  esp_err_t result = esp_netif_init();
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "esp_netif_init failed: %s", esp_err_to_name(result));
    return result;
  }
  HeapLogPhase("wifi_netif_init_after");

  HeapLogPhase("wifi_event_loop_before");
  result = esp_event_loop_create_default();
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "event loop create failed: %s", esp_err_to_name(result));
    return result;
  }
  HeapLogPhase("wifi_event_loop_after");

  s_initialized = true;
  return ESP_OK;
}
