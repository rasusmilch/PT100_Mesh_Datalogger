#ifndef PT100_LOGGER_MQTT_CLIENT_WRAP_H_
#define PT100_LOGGER_MQTT_CLIENT_WRAP_H_

#include <stdbool.h>

#include "esp_err.h"
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    esp_mqtt_client_handle_t client;
    bool started;
    bool connected;
    char broker_uri[128];
  } mqtt_client_wrap_t;

  void MqttClientWrapInit(mqtt_client_wrap_t* wrap);

  esp_err_t MqttClientWrapStart(mqtt_client_wrap_t* wrap,
                                const char* broker_uri);

  void MqttClientWrapStop(mqtt_client_wrap_t* wrap);

  bool MqttClientWrapIsConnected(const mqtt_client_wrap_t* wrap);

  esp_err_t MqttClientWrapPublish(mqtt_client_wrap_t* wrap,
                                  const char* topic,
                                  const char* payload,
                                  int len,
                                  int qos,
                                  int retain);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_MQTT_CLIENT_WRAP_H_
