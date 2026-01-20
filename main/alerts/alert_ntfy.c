#include "alerts/alert_ntfy.h"
#include "alerts/alert_manager.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_http_client.h"
#include "esp_log.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

static const char* kTag = "alert_ntfy";

/**
 * @brief Reduce log noise from ESP-IDF certificate bundle validation.
 *
 * This suppresses repetitive INFO logs (e.g. "Certificate validated") while
 * retaining WARN/ERROR logs for real TLS/certificate problems.
 */
static void
AlertNtfyApplyTlsLogPolicy(void)
{
  esp_log_level_set("esp-x509-crt-bundle", ESP_LOG_WARN);
}

/**
 * @brief Execute FormatEpoch.
 * @param epoch_seconds Parameter epoch_seconds.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
static void
FormatEpoch(int64_t epoch_seconds, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  if (epoch_seconds <= 0) {
    snprintf(out, out_size, "unknown");
    return;
  }
  time_t raw = (time_t)epoch_seconds;
  struct tm timeinfo;
  gmtime_r(&raw, &timeinfo);
  strftime(out, out_size, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
}

/**
 * @brief Execute SeverityToPriority.
 * @param severity Parameter severity.
 * @return Return the function result.
 */
static const char*
SeverityToPriority(int severity)
{
  switch (severity) {
    case 2:
      return "high";
    case 1:
      return "default";
    default:
      return "low";
  }
}

/**
 * @brief Execute FormatLeafId.
 * @param leaf_id Parameter leaf_id.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
static void
FormatLeafId(uint64_t leaf_id, char* out, size_t out_size)
{
  if (out == NULL || out_size < 18) {
    return;
  }
  uint8_t mac[6];
  for (int i = 5; i >= 0; --i) {
    mac[i] = (uint8_t)(leaf_id & 0xFFu);
    leaf_id >>= 8;
  }
  snprintf(out,
           out_size,
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],
           mac[1],
           mac[2],
           mac[3],
           mac[4],
           mac[5]);
}

/**
 * @brief Execute FormatMilliC.
 * @param milli_c Parameter milli_c.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 */
static void
FormatMilliC(int32_t milli_c, char* out, size_t out_size)
{
  if (out == NULL || out_size == 0) {
    return;
  }
  int64_t value = milli_c;
  bool negative = value < 0;
  int64_t abs_value = negative ? -value : value;
  int64_t whole = abs_value / 1000;
  int64_t frac = abs_value % 1000;
  snprintf(out,
           out_size,
           "%s%" PRId64 ".%03" PRId64 "C",
           negative ? "-" : "",
           whole,
           frac);
}

/**
 * @brief Execute AppendTimeLine.
 * @param payload Parameter payload.
 * @param body Parameter body.
 * @param body_size Parameter body_size.
 */
static void
AppendTimeLine(const alert_notification_payload_t* payload,
               char* body,
               size_t body_size)
{
  if (payload == NULL || body == NULL || body_size == 0) {
    return;
  }
  char time_str[32];
  if (payload->event_epoch > 0) {
    FormatEpoch(payload->event_epoch, time_str, sizeof(time_str));
    snprintf(body + strlen(body),
             body_size - strlen(body),
             "time: %s\n",
             time_str);
  } else {
    snprintf(body + strlen(body),
             body_size - strlen(body),
             "time: uptime=%" PRIu32 "ms (time_invalid)\n",
             (uint32_t)payload->event_uptime_ms);
  }
}

/**
 * @brief Execute AlertNtfyInit.
 * @param ntfy Parameter ntfy.
 */
void
AlertNtfyInit(alert_ntfy_t* ntfy)
{
  AlertNtfyApplyTlsLogPolicy();

  if (ntfy == NULL) {
    return;
  }
  memset(ntfy, 0, sizeof(*ntfy));
  ntfy->queue = xQueueCreateStatic(ALERT_NTFY_QUEUE_LEN,
                                   sizeof(alert_notification_t),
                                   ntfy->queue_storage,
                                   &ntfy->queue_buffer);
}

/**
 * @brief Execute AlertNtfyEnqueue.
 * @param ntfy Parameter ntfy.
 * @param note Parameter note.
 * @return Return the function result.
 */
bool
AlertNtfyEnqueue(alert_ntfy_t* ntfy, const alert_notification_t* note)
{
  if (ntfy == NULL || note == NULL || ntfy->queue == NULL) {
    return false;
  }
  if (xQueueSend(ntfy->queue, note, 0) == pdTRUE) {
    return true;
  }
  alert_notification_t dropped_note;
  if (xQueueReceive(ntfy->queue, &dropped_note, 0) == pdTRUE) {
    ntfy->dropped++;
  }
  if (xQueueSend(ntfy->queue, note, 0) == pdTRUE) {
    return true;
  }
  ntfy->dropped++;
  return false;
}

/**
 * @brief Execute AlertNtfySend.
 * @param ntfy Parameter ntfy.
 * @param cfg Parameter cfg.
 * @param note Parameter note.
 * @param out_status Parameter out_status.
 * @param out_err Parameter out_err.
 * @return Return the function result.
 */
alert_ntfy_result_t
AlertNtfySend(const alert_ntfy_t* ntfy,
              const alert_ntfy_config_t* cfg,
              const alert_notification_t* note,
              int* out_status,
              esp_err_t* out_err)
{
  (void)ntfy;
  if (out_status != NULL) {
    *out_status = 0;
  }
  if (out_err != NULL) {
    *out_err = ESP_OK;
  }
  if (cfg == NULL || note == NULL || ntfy == NULL) {
    if (out_err != NULL) {
      *out_err = ESP_ERR_INVALID_ARG;
    }
    return ALERT_NTFY_FAILED;
  }
  if (cfg->url == NULL || cfg->url[0] == '\0' || cfg->topic == NULL ||
      cfg->topic[0] == '\0') {
    if (out_err != NULL) {
      *out_err = ESP_ERR_INVALID_STATE;
    }
    return ALERT_NTFY_SKIPPED;
  }

  char url[256];
  if (cfg->url[strlen(cfg->url) - 1] == '/') {
    snprintf(url, sizeof(url), "%s%s", cfg->url, cfg->topic);
  } else {
    snprintf(url, sizeof(url), "%s/%s", cfg->url, cfg->topic);
  }

  char leaf_id[32] = "";
  FormatLeafId(note->leaf_id, leaf_id, sizeof(leaf_id));

  char title[64];
  snprintf(title,
           sizeof(title),
           "PT100 %s",
           note->resolved ? "RESOLVED" : "ALERT");

  char body[512];
  snprintf(body,
           sizeof(body),
           "root: %s\nleaf: %s\n",
           (cfg->root_id != NULL) ? cfg->root_id : "unknown",
           leaf_id);

  switch (note->type) {
    case ALERT_TEMP_HIGH: {
      char temp_str[24];
      char limit_str[24];
      FormatMilliC(note->payload.current_temp_milli_c,
                   temp_str,
                   sizeof(temp_str));
      FormatMilliC(note->payload.limit_milli_c, limit_str, sizeof(limit_str));
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: temp_high\nvalue: %s\nthreshold: %s\n",
               temp_str,
               limit_str);
      break;
    }
    case ALERT_TEMP_LOW: {
      char temp_str[24];
      char limit_str[24];
      FormatMilliC(note->payload.current_temp_milli_c,
                   temp_str,
                   sizeof(temp_str));
      FormatMilliC(note->payload.limit_milli_c, limit_str, sizeof(limit_str));
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: temp_low\nvalue: %s\nthreshold: %s\n",
               temp_str,
               limit_str);
      break;
    }
    case ALERT_MISSING_RECORDS:
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: missing_records\ngap_ms: %" PRIu32
               "\nthreshold_ms: %" PRIu32 "\nlast_seq: %" PRIu32 "\n",
               note->payload.duration_ms,
               (uint32_t)note->payload.limit_milli_c,
               note->payload.last_seq);
      break;
    case ALERT_LEAF_OFFLINE:
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: leaf_offline\noffline_ms: %" PRIu32
               "\nthreshold_ms: %" PRIu32 "\n",
               note->payload.duration_ms,
               (uint32_t)note->payload.limit_milli_c);
      break;
    case ALERT_LEAF_RESTART:
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: leaf_restart\nlast_seq: %" PRIu32 "\n",
               note->payload.last_seq);
      break;
    case ALERT_ROOT_RESTART:
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: root_restart\n");
      break;
    case ALERT_SYSTEM_BOOT:
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: system_boot\n");
      break;
    case ALERT_SYSTEM_MODE: {
      const char* mode = "unknown";
      if (note->payload.event_code == ALERT_SYSTEM_CODE_MODE_RUN) {
        mode = "run";
      } else if (note->payload.event_code == ALERT_SYSTEM_CODE_MODE_DIAG) {
        mode = "diag";
      }
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: system_mode\nmode: %s\n",
               mode);
      break;
    }
    case ALERT_SYSTEM_ERROR: {
      const char* error = "unknown";
      bool known = true;
      switch (note->payload.event_code) {
        case ALERT_SYSTEM_CODE_ERROR_SD_IO:
          error = "sd_io";
          break;
        case ALERT_SYSTEM_CODE_ERROR_FRAM_OVERRUN:
          error = "fram_overrun";
          break;
        case ALERT_SYSTEM_CODE_ERROR_RTD_FAULT:
          error = "rtd_fault";
          break;
        case ALERT_SYSTEM_CODE_ERROR_TIME_INVALID:
          error = "time_invalid";
          break;
        case ALERT_SYSTEM_CODE_ERROR_FRAM_IO:
          error = "fram_io";
          break;
        case ALERT_SYSTEM_CODE_ERROR_I2C_RECOVERY:
          error = "i2c_recovery";
          break;
        case ALERT_SYSTEM_CODE_ERROR_STORAGE_STALL:
          error = "storage_stall";
          break;
        default:
          known = false;
          break;
      }
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: system_error\nerror: %s\n",
               error);
      if (!known) {
        snprintf(body + strlen(body),
                 sizeof(body) - strlen(body),
                 "error_code: %" PRIu32 "\n",
                 note->payload.event_code);
      }
      break;
    }
    default:
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: unknown\n");
      break;
  }

  AppendTimeLine(&note->payload, body, sizeof(body));

  const uint32_t timeout_ms =
    (cfg->http_timeout_ms > 0) ? cfg->http_timeout_ms : 5000;
  esp_http_client_config_t config = {
    .url = url,
    .method = HTTP_METHOD_POST,
    .timeout_ms = (int)timeout_ms,
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    .crt_bundle_attach = esp_crt_bundle_attach,
#endif
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    if (out_err != NULL) {
      *out_err = ESP_ERR_NO_MEM;
    }
    return ALERT_NTFY_FAILED;
  }

  (void)esp_http_client_set_header(client, "Title", title);
  (void)esp_http_client_set_header(
    client, "Priority", SeverityToPriority(note->severity));
  (void)esp_http_client_set_header(client, "Tags", "pt100,mesh,alarm");
  if (cfg->token != NULL && cfg->token[0] != '\0') {
    char auth[160];
    snprintf(auth, sizeof(auth), "Bearer %s", cfg->token);
    (void)esp_http_client_set_header(client, "Authorization", auth);
  }

  esp_http_client_set_post_field(client, body, (int)strlen(body));

  esp_err_t err = esp_http_client_perform(client);
  int status = esp_http_client_get_status_code(client);
  esp_http_client_cleanup(client);

  if (out_status != NULL) {
    *out_status = status;
  }
  if (out_err != NULL) {
    *out_err = err;
  }

  if (err == ESP_OK && status >= 200 && status < 300) {
    return ALERT_NTFY_OK;
  }

  ESP_LOGW(kTag, "ntfy post failed: err=%s status=%d", esp_err_to_name(err), status);
  return ALERT_NTFY_FAILED;
}
