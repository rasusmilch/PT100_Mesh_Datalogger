#include "mqtt_client_wrap.h"

#include <string.h>

#include "esp_log.h"

static const char* kTag = "mqtt";

static esp_err_t
MqttEventHandler(esp_mqtt_event_handle_t event)
{
  if (event == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  mqtt_client_wrap_t* wrap = (mqtt_client_wrap_t*)event->user_context;
  if (wrap == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      wrap->connected = true;
      ESP_LOGI(kTag, "MQTT connected");
      break;
    case MQTT_EVENT_DISCONNECTED:
      if (wrap->connected) {
        ESP_LOGW(kTag, "MQTT disconnected");
      }
      wrap->connected = false;
      break;
    default:
      break;
  }
  return ESP_OK;
}

void
MqttClientWrapInit(mqtt_client_wrap_t* wrap)
{
  if (wrap == NULL) {
    return;
  }
  memset(wrap, 0, sizeof(*wrap));
}

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
    wrap->client, MQTT_EVENT_ANY, MqttEventHandler, wrap);
  const esp_err_t start_result = esp_mqtt_client_start(wrap->client);
  if (start_result != ESP_OK) {
    esp_mqtt_client_destroy(wrap->client);
    wrap->client = NULL;
    return start_result;
  }
  wrap->started = true;
  return ESP_OK;
}

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

bool
MqttClientWrapIsConnected(const mqtt_client_wrap_t* wrap)
{
  if (wrap == NULL) {
    return false;
  }
  return wrap->connected;
}

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
