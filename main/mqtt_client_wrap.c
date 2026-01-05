#include "mqtt_client_wrap.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"

static const char* kTag = "mqtt";

/**
 * @brief Handle MQTT client events and update wrapper state.
 *
 * This uses the ESP event loop handler signature required by esp-mqtt's
 * `esp_mqtt_client_register_event()`.
 *
 * @note Logs disconnect diagnostics when available via
 * esp_mqtt_event_handle_t::error_handle.
 */
static void
MqttEventHandler(void* handler_args,
                 esp_event_base_t event_base,
                 int32_t event_id,
                 void* event_data)
{
  (void)event_base;

  mqtt_client_wrap_t* wrap = (mqtt_client_wrap_t*)handler_args;
  if (wrap == NULL) {
    return;
  }

  // esp-mqtt passes esp_mqtt_event_handle_t as event_data.
  esp_mqtt_event_handle_t mqtt_event = (esp_mqtt_event_handle_t)event_data;

  switch (event_id) {
    case MQTT_EVENT_CONNECTED:
      wrap->connected = true;
      ESP_LOGI(kTag, "MQTT connected");
      break;

    case MQTT_EVENT_DISCONNECTED: {
      if (wrap->connected) {
        ESP_LOGW(kTag, "MQTT disconnected");
      } else {
        ESP_LOGW(kTag, "MQTT disconnected (was not marked connected)");
      }
      wrap->connected = false;

      // Best-effort diagnostic details. Do not assume the pointer is present.
      if (mqtt_event == NULL || mqtt_event->error_handle == NULL) {
        ESP_LOGW(kTag, "MQTT disconnect details unavailable");
        break;
      }

      const esp_mqtt_error_codes_t* error_handle = mqtt_event->error_handle;
      ESP_LOGW(kTag,
               "MQTT error: type=%d tls_last_esp_err=0x%x tls_stack_err=0x%x "
               "tls_cert_flags=0x%x",
               (int)error_handle->error_type,
               (unsigned)error_handle->esp_tls_last_esp_err,
               (unsigned)error_handle->esp_tls_stack_err,
               (unsigned)error_handle->esp_tls_cert_verify_flags);

      // Provide a more human-meaningful hint when the error type is a TCP
      // transport error.
      if (error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        // esp_transport_sock_errno is only meaningful for TCP transport errors.
        const int socket_errno = error_handle->esp_transport_sock_errno;
        if (socket_errno != 0) {
          ESP_LOGW(kTag,
                   "MQTT TCP transport errno=%d (%s)",
                   socket_errno,
                   strerror(socket_errno));
        } else {
          ESP_LOGW(kTag,
                   "MQTT TCP transport error with errno=0 (no errno set)");
        }
      }
      break;
    }

    default:
      break;
  }
}

/**
 * @brief Execute MqttClientWrapInit.
 * @param wrap Parameter wrap.
 */
void
MqttClientWrapInit(mqtt_client_wrap_t* wrap)
{
  if (wrap == NULL) {
    return;
  }
  memset(wrap, 0, sizeof(*wrap));
}

/**
 * @brief Execute MqttClientWrapStart.
 * @param wrap Parameter wrap.
 * @param broker_uri Parameter broker_uri.
 * @return Return the function result.
 */
esp_err_t
MqttClientWrapStart(mqtt_client_wrap_t* wrap, const char* broker_uri)
{
  if (wrap == NULL || broker_uri == NULL || broker_uri[0] == '\0') {
    return ESP_ERR_INVALID_ARG;
  }

  if (wrap->started && strcmp(wrap->broker_uri, broker_uri) == 0) {
    return ESP_OK;
  }

  if (wrap->started) {
    MqttClientWrapStop(wrap);
  }

  esp_mqtt_client_config_t config = {
    .broker.address.uri = broker_uri,
  };
  wrap->client = esp_mqtt_client_init(&config);
  if (wrap->client == NULL) {
    return ESP_ERR_NO_MEM;
  }
  strlcpy(wrap->broker_uri, broker_uri, sizeof(wrap->broker_uri));
  esp_mqtt_client_register_event(
    wrap->client, MQTT_EVENT_ANY, &MqttEventHandler, wrap);
  const esp_err_t start_result = esp_mqtt_client_start(wrap->client);
  if (start_result != ESP_OK) {
    esp_mqtt_client_destroy(wrap->client);
    wrap->client = NULL;
    return start_result;
  }
  wrap->started = true;
  return ESP_OK;
}

/**
 * @brief Execute MqttClientWrapStop.
 * @param wrap Parameter wrap.
 */
void
MqttClientWrapStop(mqtt_client_wrap_t* wrap)
{
  if (wrap == NULL || !wrap->started) {
    return;
  }
  (void)esp_mqtt_client_stop(wrap->client);
  (void)esp_mqtt_client_destroy(wrap->client);
  wrap->client = NULL;
  wrap->started = false;
  wrap->connected = false;
  wrap->broker_uri[0] = '\0';
}

/**
 * @brief Execute MqttClientWrapIsConnected.
 * @param wrap Parameter wrap.
 * @return Return the function result.
 */
bool
MqttClientWrapIsConnected(const mqtt_client_wrap_t* wrap)
{
  if (wrap == NULL) {
    return false;
  }
  return wrap->connected;
}

/**
 * @brief Execute MqttClientWrapPublish.
 * @param wrap Parameter wrap.
 * @param topic Parameter topic.
 * @param payload Parameter payload.
 * @param len Parameter len.
 * @param qos Parameter qos.
 * @param retain Parameter retain.
 * @return Return the function result.
 */
esp_err_t
MqttClientWrapPublish(mqtt_client_wrap_t* wrap,
                      const char* topic,
                      const char* payload,
                      int len,
                      int qos,
                      int retain)
{
  if (wrap == NULL || !wrap->started || topic == NULL || payload == NULL ||
      len < 0) {
    return ESP_ERR_INVALID_ARG;
  }
  const int msg_id =
    esp_mqtt_client_publish(wrap->client, topic, payload, len, qos, retain);
  return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}
