#ifndef PT100_LOGGER_ALERT_NTFY_H_
#define PT100_LOGGER_ALERT_NTFY_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ALERT_NTFY_QUEUE_LEN 32
#define ALERT_NTFY_JOB_QUEUE_LEN 6
#define ALERT_NTFY_JOB_TITLE_LEN 64
#define ALERT_NTFY_JOB_BODY_LEN 1024
#define ALERT_NTFY_JOB_URL_LEN 128
#define ALERT_NTFY_JOB_TOPIC_LEN 64
#define ALERT_NTFY_JOB_TOKEN_LEN 128
#define ALERT_NTFY_JOB_ROOT_ID_LEN 32

  typedef enum
  {
    ALERT_NTFY_OK = 0,
    ALERT_NTFY_SKIPPED = 1,
    ALERT_NTFY_FAILED = 2,
  } alert_ntfy_result_t;

  typedef enum
  {
    ALERT_SYSTEM_CODE_NONE = 0,
    ALERT_SYSTEM_CODE_BOOT = 1,
    ALERT_SYSTEM_CODE_MODE_RUN = 2,
    ALERT_SYSTEM_CODE_MODE_DIAG = 3,
    ALERT_SYSTEM_CODE_ERROR_SD_IO = 10,
    ALERT_SYSTEM_CODE_ERROR_FRAM_OVERRUN = 11,
    ALERT_SYSTEM_CODE_ERROR_RTD_FAULT = 12,
    ALERT_SYSTEM_CODE_ERROR_TIME_INVALID = 13,
    ALERT_SYSTEM_CODE_ERROR_FRAM_IO = 14,
    ALERT_SYSTEM_CODE_ERROR_I2C_RECOVERY = 15,
    ALERT_SYSTEM_CODE_ERROR_STORAGE_STALL = 16,
    ALERT_SYSTEM_CODE_ERROR_SENSOR_SPI = 17,
    ALERT_SYSTEM_CODE_ERROR_NTFY_RATE_LIMIT = 18,
    ALERT_SYSTEM_CODE_ERROR_FRAM_ERRLOG = 19,
    ALERT_SYSTEM_CODE_ERROR_NTFY_QUEUE = 20,
  } alert_system_code_t;

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
    uint32_t event_code;
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
    char url[ALERT_NTFY_JOB_URL_LEN];
    char topic[ALERT_NTFY_JOB_TOPIC_LEN];
    char token[ALERT_NTFY_JOB_TOKEN_LEN];
    char root_id[ALERT_NTFY_JOB_ROOT_ID_LEN];
    char title[ALERT_NTFY_JOB_TITLE_LEN];
    char body[ALERT_NTFY_JOB_BODY_LEN];
    uint32_t http_timeout_ms;
    uint32_t attempt;
    int64_t next_attempt_ms;
  } alert_ntfy_job_t;

  typedef struct
  {
    QueueHandle_t queue;
    StaticQueue_t* queue_buffer;
    uint8_t* queue_storage;
    uint32_t dropped;
    uint32_t send_success;
    uint32_t send_fail;
    int last_http_status;
    esp_err_t last_err;
    uint32_t backoff_ms;
    int64_t cooldown_until_ms;
    uint32_t rate_limited_count;
    uint32_t suppressed_count;
    bool pending_valid;
    alert_notification_t pending_note;
    int64_t last_attempt_ms;
    int64_t last_sent_ms;
    bool last_sent_valid;
    alert_notification_t last_sent;
    QueueHandle_t job_queue;
    StaticQueue_t* job_queue_buffer;
    uint8_t* job_queue_storage;
    uint32_t job_dropped;
    uint32_t job_last_drop_log_ms;
    uint32_t last_send_log_ms;
    uint32_t last_send_fail_log_ms;
  } alert_ntfy_t;

  typedef struct
  {
    const char* url;
    const char* topic;
    const char* token;
    const char* root_id;
    uint32_t http_timeout_ms;
  } alert_ntfy_config_t;

  /**
   * @brief Execute AlertNtfyInit.
   * @param ntfy Parameter ntfy.
   */
  void AlertNtfyInit(alert_ntfy_t* ntfy);

  /**
   * @brief Execute AlertNtfyEnqueue.
   * @param ntfy Parameter ntfy.
   * @param note Parameter note.
   * @return Return the function result.
   */
  bool AlertNtfyEnqueue(alert_ntfy_t* ntfy, const alert_notification_t* note);

  /**
   * @brief Execute AlertNtfyEnqueueJob.
   * @param ntfy Parameter ntfy.
   * @param job Parameter job.
   * @return Return the function result.
   */
  bool AlertNtfyEnqueueJob(alert_ntfy_t* ntfy, const alert_ntfy_job_t* job);

  /**
   * @brief Execute AlertNtfySend.
   * @param ntfy Parameter ntfy.
   * @param cfg Parameter cfg.
   * @param note Parameter note.
   * @param out_retry_after_seconds Parameter out_retry_after_seconds.
   * @param out_status Parameter out_status.
   * @param out_err Parameter out_err.
   * @return Return the function result.
   */
  alert_ntfy_result_t AlertNtfySend(alert_ntfy_t* ntfy,
                                    const alert_ntfy_config_t* cfg,
                                    const alert_notification_t* note,
                                    int* out_retry_after_seconds,
                                    int* out_status,
                                    esp_err_t* out_err);

  /**
   * @brief Execute AlertNtfySendText.
   * @param ntfy Parameter ntfy.
   * @param cfg Parameter cfg.
   * @param title Parameter title.
   * @param body Parameter body.
   * @param out_retry_after_seconds Parameter out_retry_after_seconds.
   * @param out_status Parameter out_status.
   * @param out_err Parameter out_err.
   * @return Return the function result.
   */
  alert_ntfy_result_t AlertNtfySendText(alert_ntfy_t* ntfy,
                                        const alert_ntfy_config_t* cfg,
                                        const char* title,
                                        const char* body,
                                        int* out_retry_after_seconds,
                                        int* out_status,
                                        esp_err_t* out_err);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_ALERT_NTFY_H_
