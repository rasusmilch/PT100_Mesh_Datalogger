#ifndef PT100_LOGGER_ALERT_NTFY_H_
#define PT100_LOGGER_ALERT_NTFY_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALERT_NTFY_QUEUE_LEN 16

typedef enum
{
  ALERT_NTFY_OK = 0,
  ALERT_NTFY_SKIPPED = 1,
  ALERT_NTFY_FAILED = 2,
} alert_ntfy_result_t;

typedef struct
{
  int32_t current_temp_milli_c;
  int32_t limit_milli_c;
  int32_t hysteresis_milli_c;
  uint32_t duration_ms;
  uint32_t last_seq;
  int64_t last_rx_epoch;
  int64_t last_rx_uptime_ms;
  int64_t event_epoch;
  int64_t event_uptime_ms;
  uint32_t transitions;
} alert_notification_payload_t;

typedef struct
{
  int type;
  int severity;
  bool resolved;
  uint64_t leaf_id;
  alert_notification_payload_t payload;
} alert_notification_t;

typedef struct
{
  QueueHandle_t queue;
  StaticQueue_t queue_buffer;
  uint8_t queue_storage[sizeof(alert_notification_t) * ALERT_NTFY_QUEUE_LEN];
  uint32_t dropped;
  uint32_t send_success;
  uint32_t send_fail;
  int last_http_status;
  esp_err_t last_err;
  uint32_t backoff_ms;
} alert_ntfy_t;

typedef struct
{
  const char* url;
  const char* topic;
  const char* token;
  const char* root_id;
} alert_ntfy_config_t;

void AlertNtfyInit(alert_ntfy_t* ntfy);

bool AlertNtfyEnqueue(alert_ntfy_t* ntfy, const alert_notification_t* note);

alert_ntfy_result_t AlertNtfySend(const alert_ntfy_t* ntfy,
                                  const alert_ntfy_config_t* cfg,
                                  const alert_notification_t* note,
                                  int* out_status,
                                  esp_err_t* out_err);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_ALERT_NTFY_H_
