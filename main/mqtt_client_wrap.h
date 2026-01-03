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

/**
 * @brief Execute MqttClientWrapInit.
 * @param wrap Parameter wrap.
 */
  void MqttClientWrapInit(mqtt_client_wrap_t* wrap);

/**
 * @brief Execute MqttClientWrapStart.
 * @param wrap Parameter wrap.
 * @param broker_uri Parameter broker_uri.
 * @return Return the function result.
 */
  esp_err_t MqttClientWrapStart(mqtt_client_wrap_t* wrap,
                                const char* broker_uri);

/**
 * @brief Execute MqttClientWrapStop.
 * @param wrap Parameter wrap.
 */
  void MqttClientWrapStop(mqtt_client_wrap_t* wrap);

/**
 * @brief Execute MqttClientWrapIsConnected.
 * @param wrap Parameter wrap.
 * @return Return the function result.
 */
  bool MqttClientWrapIsConnected(const mqtt_client_wrap_t* wrap);

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
