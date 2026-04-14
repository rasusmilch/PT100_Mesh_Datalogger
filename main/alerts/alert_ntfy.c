#include "alerts/alert_ntfy.h"
#include "alerts/alert_manager.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include <arpa/inet.h>
#include <ctype.h>
#include <netdb.h>
#include <sys/socket.h>

#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "log_rate_limit.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

static void
AlertNtfyApplyTlsLogPolicy(void);
static void
FormatEpoch(int64_t epoch_seconds, char* out, size_t out_size);
static const char*
SeverityToPriority(int severity);
static int
AlertNtfyParseRetryAfterSeconds(const char* value);
static esp_err_t
AlertNtfyHttpEventHandler(esp_http_client_event_t* evt);
static bool
ContainsWhitespace(const char* text);
static bool
ExtractHostnameFromUrl(const char* url, char* out_host, size_t out_host_size);
static void
LogDnsAddresses(const struct addrinfo* result,
                char* out_first_ip,
                size_t out_first_ip_size);
static void
LogNtfyFailure(alert_ntfy_t* ntfy,
               const char* base_url,
               const char* hostname,
               esp_err_t err,
               int status,
               uint32_t timeout_ms,
               uint32_t attempt);
static void
FormatLeafId(uint64_t leaf_id, char* out, size_t out_size);
static void
FormatMilliC(int32_t milli_c, char* out, size_t out_size);
static void
AppendTimeLine(const alert_notification_payload_t* payload,
               char* body,
               size_t body_size);
static uint8_t
AlertNtfyComputeFragPercent(const multi_heap_info_t* info);
static void
AlertNtfySnapshotResources(uint32_t* out_internal_free,
                           uint32_t* out_internal_largest,
                           uint8_t* out_internal_frag,
                           uint32_t* out_psram_free,
                           uint32_t* out_psram_largest,
                           uint8_t* out_psram_frag);
static void
AlertNtfyLogAttemptResources(const char* phase,
                             const alert_ntfy_config_t* cfg,
                             esp_err_t err,
                             int status,
                             bool cleanup_performed,
                             bool will_retry);
/**
 * @brief Convert URL sanitize result code to a printable string.
 * @param reason Sanitizer result code.
 * @return Static reason string.
 */
const char*
AlertNtfyUrlSanitizeResultToString(alert_ntfy_url_sanitize_result_t reason)
{
  switch (reason) {
    case ALERT_NTFY_URL_SANITIZE_OK:
      return "ok";
    case ALERT_NTFY_URL_SANITIZE_EMPTY:
      return "empty";
    case ALERT_NTFY_URL_SANITIZE_TOO_LONG:
      return "too_long";
    case ALERT_NTFY_URL_SANITIZE_BAD_SCHEME:
      return "bad_scheme";
    case ALERT_NTFY_URL_SANITIZE_MALFORMED_SCHEME:
      return "malformed_scheme";
    case ALERT_NTFY_URL_SANITIZE_EMBEDDED_WHITESPACE:
      return "embedded_whitespace";
    case ALERT_NTFY_URL_SANITIZE_MISSING_HOST:
      return "missing_host";
    default:
      return "unknown";
  }
}

/**
 * @brief Validate and normalize operator ntfy base URL input.
 * @param input_url Raw operator input URL.
 * @param output_url Destination for normalized URL.
 * @param output_url_size Size of output_url in bytes.
 * @param out_reason Optional output reason enum.
 * @return True when sanitized output_url is valid.
 */
bool
AlertNtfySanitizeBaseUrl(const char* input_url,
                         char* output_url,
                         size_t output_url_size,
                         alert_ntfy_url_sanitize_result_t* out_reason)
{
  if (out_reason != NULL) {
    *out_reason = ALERT_NTFY_URL_SANITIZE_OK;
  }
  if (output_url == NULL || output_url_size == 0) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_TOO_LONG;
    }
    return false;
  }
  output_url[0] = '\0';

  if (input_url == NULL) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_EMPTY;
    }
    return false;
  }

  const char* start = input_url;
  while (*start != '\0' && isspace((unsigned char)*start)) {
    ++start;
  }
  const char* end = input_url + strlen(input_url);
  while (end > start && isspace((unsigned char)*(end - 1))) {
    --end;
  }
  const size_t trimmed_len = (size_t)(end - start);
  if (trimmed_len == 0) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_EMPTY;
    }
    return false;
  }

  char candidate[160] = { 0 };
  if (trimmed_len >= sizeof(candidate)) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_TOO_LONG;
    }
    return false;
  }
  memcpy(candidate, start, trimmed_len);
  candidate[trimmed_len] = '\0';

  if (ContainsWhitespace(candidate)) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_EMBEDDED_WHITESPACE;
    }
    return false;
  }

  if (strncmp(candidate, "http:/", 6) == 0 && strncmp(candidate, "http://", 7) != 0) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_MALFORMED_SCHEME;
    }
    return false;
  }
  if (strncmp(candidate, "https:/", 7) == 0 && strncmp(candidate, "https://", 8) != 0) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_MALFORMED_SCHEME;
    }
    return false;
  }

  const bool has_http = (strncmp(candidate, "http://", 7) == 0);
  const bool has_https = (strncmp(candidate, "https://", 8) == 0);
  if (strstr(candidate, "://") != NULL && !has_http && !has_https) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_BAD_SCHEME;
    }
    return false;
  }

  if (!has_http && !has_https) {
    snprintf(output_url, output_url_size, "https://%s", candidate);
  } else {
    strlcpy(output_url, candidate, output_url_size);
  }

  if (output_url[0] == '\0' || strlen(output_url) >= output_url_size) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_TOO_LONG;
    }
    return false;
  }

  char hostname[96] = { 0 };
  if (!ExtractHostnameFromUrl(output_url, hostname, sizeof(hostname))) {
    if (out_reason != NULL) {
      *out_reason = ALERT_NTFY_URL_SANITIZE_MISSING_HOST;
    }
    output_url[0] = '\0';
    return false;
  }
  return true;
}

static alert_ntfy_result_t
AlertNtfySendHttp(alert_ntfy_t* ntfy,
                  const alert_ntfy_config_t* cfg,
                  const char* title,
                  const char* body,
                  const char* priority,
                  const char* sequence_id,
                  int* out_retry_after_seconds,
                  int* out_status,
                  esp_err_t* out_err);

static const char* kTag = "alert_ntfy";
static const uint32_t kNtfyJobDropLogRateLimitMs = 60000;
static const uint32_t kNtfyHttpDefaultTimeoutMs = 15000;

typedef struct
{
  int retry_after_seconds;
} alert_ntfy_http_context_t;

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

static int
AlertNtfyParseRetryAfterSeconds(const char* value)
{
  if (value == NULL || value[0] == '\0') {
    return -1;
  }
  char* end = NULL;
  long seconds = strtol(value, &end, 10);
  if (end == value || seconds <= 0) {
    return -1;
  }
  if (seconds > INT_MAX) {
    return INT_MAX;
  }
  return (int)seconds;
}

static esp_err_t
AlertNtfyHttpEventHandler(esp_http_client_event_t* evt)
{
  if (evt == NULL || evt->user_data == NULL) {
    return ESP_OK;
  }
  if (evt->event_id != HTTP_EVENT_ON_HEADER) {
    return ESP_OK;
  }
  if (evt->header_key == NULL || evt->header_value == NULL) {
    return ESP_OK;
  }
  if (strcasecmp(evt->header_key, "Retry-After") != 0) {
    return ESP_OK;
  }
  alert_ntfy_http_context_t* ctx = (alert_ntfy_http_context_t*)evt->user_data;
  int seconds = AlertNtfyParseRetryAfterSeconds(evt->header_value);
  if (seconds > 0) {
    ctx->retry_after_seconds = seconds;
  }
  return ESP_OK;
}

static bool
ContainsWhitespace(const char* text)
{
  if (text == NULL) {
    return false;
  }
  for (size_t index = 0; text[index] != '\0'; ++index) {
    if (isspace((unsigned char)text[index])) {
      return true;
    }
  }
  return false;
}

static bool
ExtractHostnameFromUrl(const char* url, char* out_host, size_t out_host_size)
{
  if (url == NULL || out_host == NULL || out_host_size == 0) {
    return false;
  }
  out_host[0] = '\0';

  const char* scheme_sep = strstr(url, "://");
  if (scheme_sep == NULL) {
    return false;
  }
  const char* host_start = scheme_sep + 3;
  if (host_start[0] == '\0') {
    return false;
  }

  const char* host_end = host_start;
  while (host_end[0] != '\0' && host_end[0] != '/') {
    ++host_end;
  }
  size_t host_len = (size_t)(host_end - host_start);
  if (host_len == 0) {
    return false;
  }

  const char* parsed_start = host_start;
  size_t parsed_len = host_len;
  if (host_start[0] == '[') {
    const char* closing_bracket = memchr(host_start, ']', host_len);
    if (closing_bracket == NULL) {
      return false;
    }
    parsed_start = host_start + 1;
    parsed_len = (size_t)(closing_bracket - parsed_start);
    if (parsed_len == 0) {
      return false;
    }
  } else {
    const char* colon = memchr(host_start, ':', host_len);
    if (colon != NULL) {
      parsed_len = (size_t)(colon - host_start);
    }
  }

  if (parsed_len == 0 || parsed_len >= out_host_size) {
    return false;
  }
  memcpy(out_host, parsed_start, parsed_len);
  out_host[parsed_len] = '\0';
  return true;
}

static void
LogDnsAddresses(const struct addrinfo* result,
                char* out_first_ip,
                size_t out_first_ip_size)
{
  if (out_first_ip != NULL && out_first_ip_size > 0) {
    out_first_ip[0] = '\0';
  }

  uint32_t logged_count = 0;
  for (const struct addrinfo* node = result; node != NULL; node = node->ai_next) {
    char ip_text[INET6_ADDRSTRLEN] = { 0 };
    const char* family = NULL;
    const void* address_ptr = NULL;

    if (node->ai_family == AF_INET) {
      const struct sockaddr_in* sockaddr_v4 =
        (const struct sockaddr_in*)node->ai_addr;
      address_ptr = &sockaddr_v4->sin_addr;
      family = "IPv4";
    } else if (node->ai_family == AF_INET6) {
      const struct sockaddr_in6* sockaddr_v6 =
        (const struct sockaddr_in6*)node->ai_addr;
      address_ptr = &sockaddr_v6->sin6_addr;
      family = "IPv6";
    } else {
      continue;
    }

    if (inet_ntop(node->ai_family, address_ptr, ip_text, sizeof(ip_text)) == NULL) {
      continue;
    }

    if (logged_count == 0 && out_first_ip != NULL && out_first_ip_size > 0) {
      strlcpy(out_first_ip, ip_text, out_first_ip_size);
    }

    if (logged_count < 3) {
      ESP_LOGI(kTag, "ntfy dns resolve[%" PRIu32 "] %s %s", logged_count + 1, family, ip_text);
    }
    ++logged_count;
  }
}

static void
LogNtfyFailure(alert_ntfy_t* ntfy,
               const char* base_url,
               const char* hostname,
               esp_err_t err,
               int status,
               uint32_t timeout_ms,
               uint32_t attempt)
{
  if (ntfy == NULL) {
    return;
  }
  if (!LogRateLimitAllow(&ntfy->last_send_fail_log_ms, 5000)) {
    return;
  }

  ESP_LOGW(kTag,
           "ntfy failure: base=%s host=%s dns=%d err=%s status=%d timeout_ms=%" PRIu32
           " attempt=%" PRIu32,
           (base_url != NULL && base_url[0] != '\0') ? base_url : "<unset>",
           (hostname != NULL && hostname[0] != '\0') ? hostname : "<none>",
           ntfy->last_dns_result,
           esp_err_to_name(err),
           status,
           timeout_ms,
           attempt);
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
    snprintf(
      body + strlen(body), body_size - strlen(body), "time: %s\n", time_str);
  } else {
    snprintf(body + strlen(body),
             body_size - strlen(body),
             "time: uptime=%" PRIu32 "ms (time_invalid)\n",
             (uint32_t)payload->event_uptime_ms);
  }
}

static uint8_t
AlertNtfyComputeFragPercent(const multi_heap_info_t* info)
{
  if (info == NULL || info->total_free_bytes == 0u) {
    return 0u;
  }
  const uint32_t free_bytes = info->total_free_bytes;
  const uint32_t largest = info->largest_free_block;
  if (largest >= free_bytes) {
    return 0u;
  }
  const uint32_t frag = ((free_bytes - largest) * 100u) / free_bytes;
  return (frag > 100u) ? 100u : (uint8_t)frag;
}

static void
AlertNtfySnapshotResources(uint32_t* out_internal_free,
                           uint32_t* out_internal_largest,
                           uint8_t* out_internal_frag,
                           uint32_t* out_psram_free,
                           uint32_t* out_psram_largest,
                           uint8_t* out_psram_frag)
{
  multi_heap_info_t internal_info = { 0 };
  heap_caps_get_info(&internal_info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (out_internal_free != NULL) {
    *out_internal_free = internal_info.total_free_bytes;
  }
  if (out_internal_largest != NULL) {
    *out_internal_largest = internal_info.largest_free_block;
  }
  if (out_internal_frag != NULL) {
    *out_internal_frag = AlertNtfyComputeFragPercent(&internal_info);
  }

  multi_heap_info_t psram_info = { 0 };
  heap_caps_get_info(&psram_info, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (out_psram_free != NULL) {
    *out_psram_free = psram_info.total_free_bytes;
  }
  if (out_psram_largest != NULL) {
    *out_psram_largest = psram_info.largest_free_block;
  }
  if (out_psram_frag != NULL) {
    *out_psram_frag = AlertNtfyComputeFragPercent(&psram_info);
  }
}

static void
AlertNtfyLogAttemptResources(const char* phase,
                             const alert_ntfy_config_t* cfg,
                             esp_err_t err,
                             int status,
                             bool cleanup_performed,
                             bool will_retry)
{
  uint32_t internal_free = 0;
  uint32_t internal_largest = 0;
  uint8_t internal_frag = 0;
  uint32_t psram_free = 0;
  uint32_t psram_largest = 0;
  uint8_t psram_frag = 0;
  AlertNtfySnapshotResources(&internal_free,
                             &internal_largest,
                             &internal_frag,
                             &psram_free,
                             &psram_largest,
                             &psram_frag);
  ESP_LOGI(kTag,
           "ntfy diag[%s] attempt=%" PRIu32 " retry=%u qdepth=%" PRIu32
           " task=%s stack_free=%" PRIu32
           " int_free=%" PRIu32 " int_largest=%" PRIu32 " int_frag=%u%%"
           " psram_free=%" PRIu32 " psram_largest=%" PRIu32 " psram_frag=%u%%"
           " status=%d err=%s (%d) cleanup=%u will_retry=%u",
           (phase != NULL) ? phase : "unknown",
           (cfg != NULL) ? cfg->attempt : 0u,
           (cfg != NULL && cfg->is_retry) ? 1u : 0u,
           (cfg != NULL) ? cfg->queue_depth : 0u,
           (cfg != NULL && cfg->task_name != NULL) ? cfg->task_name : "unknown",
           (cfg != NULL) ? cfg->task_stack_free_bytes : 0u,
           internal_free,
           internal_largest,
           (unsigned)internal_frag,
           psram_free,
           psram_largest,
           (unsigned)psram_frag,
           status,
           esp_err_to_name(err),
           (int)err,
           cleanup_performed ? 1u : 0u,
           will_retry ? 1u : 0u);
}

static alert_ntfy_result_t
AlertNtfySendHttp(alert_ntfy_t* ntfy,
                  const alert_ntfy_config_t* cfg,
                  const char* title,
                  const char* body,
                  const char* priority,
                  const char* sequence_id,
                  int* out_retry_after_seconds,
                  int* out_status,
                  esp_err_t* out_err)
{
  if (out_status != NULL) {
    *out_status = 0;
  }
  if (out_retry_after_seconds != NULL) {
    *out_retry_after_seconds = -1;
  }
  if (out_err != NULL) {
    *out_err = ESP_OK;
  }
  AlertNtfyLogAttemptResources("pre", cfg, ESP_OK, 0, false, false);
  if (cfg == NULL || ntfy == NULL || title == NULL || body == NULL) {
    if (out_err != NULL) {
      *out_err = ESP_ERR_INVALID_ARG;
    }
    AlertNtfyLogAttemptResources(
      "post", cfg, ESP_ERR_INVALID_ARG, 0, false, true);
    return ALERT_NTFY_FAILED;
  }
  if (cfg->url == NULL || cfg->url[0] == '\0' || cfg->topic == NULL ||
      cfg->topic[0] == '\0') {
    if (out_err != NULL) {
      *out_err = ESP_ERR_INVALID_STATE;
    }
    AlertNtfyLogAttemptResources(
      "post", cfg, ESP_ERR_INVALID_STATE, 0, false, false);
    return ALERT_NTFY_SKIPPED;
  }

  alert_ntfy_http_context_t http_ctx = {
    .retry_after_seconds = -1,
  };

  ntfy->last_dns_result = 0;
  ntfy->last_resolved_ip[0] = '\0';

  char hostname[96] = { 0 };
  if (!ExtractHostnameFromUrl(cfg->url, hostname, sizeof(hostname))) {
    if (out_err != NULL) {
      *out_err = ESP_ERR_INVALID_ARG;
    }
    AlertNtfyLogAttemptResources(
      "post", cfg, ESP_ERR_INVALID_ARG, 0, false, true);
    return ALERT_NTFY_FAILED;
  }

  const uint32_t timeout_ms =
    (cfg->http_timeout_ms > 0) ? cfg->http_timeout_ms : kNtfyHttpDefaultTimeoutMs;

  struct addrinfo hints = {
    .ai_family = AF_UNSPEC,
    .ai_socktype = SOCK_STREAM,
  };
  struct addrinfo* result = NULL;
  const int dns_result = getaddrinfo(hostname, NULL, &hints, &result);
  ntfy->last_dns_result = dns_result;
  ESP_LOGI(kTag, "ntfy dns preflight host=%s result=%d", hostname, dns_result);
  if (dns_result == 0 && result != NULL) {
    LogDnsAddresses(result, ntfy->last_resolved_ip, sizeof(ntfy->last_resolved_ip));
  }
  if (dns_result != 0) {
    if (result != NULL) {
      freeaddrinfo(result);
    }
    if (out_err != NULL) {
      *out_err = ESP_ERR_HTTP_CONNECT;
    }
    AlertNtfyLogAttemptResources(
      "post", cfg, ESP_ERR_HTTP_CONNECT, 0, false, true);
    LogNtfyFailure(ntfy, cfg->url, hostname, ESP_ERR_HTTP_CONNECT, 0, timeout_ms,
                   ntfy->send_fail + 1);
    return ALERT_NTFY_FAILED;
  }
  freeaddrinfo(result);

  char url[256];
  if (cfg->url[strlen(cfg->url) - 1] == '/') {
    snprintf(url, sizeof(url), "%s%s", cfg->url, cfg->topic);
  } else {
    snprintf(url, sizeof(url), "%s/%s", cfg->url, cfg->topic);
  }

  esp_http_client_config_t config = {
    .url = url,
    .method = HTTP_METHOD_POST,
    .timeout_ms = (int)timeout_ms,
    .event_handler = AlertNtfyHttpEventHandler,
    .user_data = &http_ctx,
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    .crt_bundle_attach = esp_crt_bundle_attach,
#endif
  };

  ESP_LOGI(kTag,
           "ntfy http init: url=%s host=%s timeout_ms=%" PRIu32
           " transport=https cert_bundle=%u",
           url,
           hostname,
           timeout_ms,
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
           1u
#else
           0u
#endif
  );

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    if (out_err != NULL) {
      *out_err = ESP_ERR_NO_MEM;
    }
    AlertNtfyLogAttemptResources("post", cfg, ESP_ERR_NO_MEM, 0, false, true);
    return ALERT_NTFY_FAILED;
  }

  (void)esp_http_client_set_header(client, "Title", title);
  if (priority != NULL && priority[0] != '\0') {
    (void)esp_http_client_set_header(client, "Priority", priority);
  }
  (void)esp_http_client_set_header(client, "Tags", "pt100,mesh,alarm");
  if (sequence_id != NULL && sequence_id[0] != '\0') {
    (void)esp_http_client_set_header(client, "X-Sequence-ID", sequence_id);
  }
  if (cfg->token != NULL && cfg->token[0] != '\0') {
    char auth[160];
    snprintf(auth, sizeof(auth), "Bearer %s", cfg->token);
    (void)esp_http_client_set_header(client, "Authorization", auth);
  }

  esp_http_client_set_post_field(client, body, (int)strlen(body));

  bool cleanup_performed = false;
  esp_err_t err = esp_http_client_perform(client);
  int status = esp_http_client_get_status_code(client);
  esp_http_client_cleanup(client);
  cleanup_performed = true;
  if (err != ESP_OK) {
    ESP_LOGW(kTag,
             "ntfy perform failed: url=%s host=%s timeout_ms=%" PRIu32
             " transport=https cert_bundle=%u err=%s (%d)",
             url,
             hostname,
             timeout_ms,
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
             1u,
#else
             0u,
#endif
             esp_err_to_name(err),
             (int)err);
  }

  if (out_status != NULL) {
    *out_status = status;
  }
  if (out_retry_after_seconds != NULL) {
    *out_retry_after_seconds = http_ctx.retry_after_seconds;
  }
  if (out_err != NULL) {
    *out_err = err;
  }

  if (err == ESP_OK && status >= 200 && status < 300) {
    AlertNtfyLogAttemptResources("post", cfg, err, status, cleanup_performed, false);
    return ALERT_NTFY_OK;
  }

  AlertNtfyLogAttemptResources("post", cfg, err, status, cleanup_performed, true);
  LogNtfyFailure(
    ntfy, cfg->url, hostname, err, status, timeout_ms, ntfy->send_fail + 1);
  return ALERT_NTFY_FAILED;
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
  const size_t queue_storage_bytes =
    sizeof(alert_notification_t) * ALERT_NTFY_QUEUE_LEN;
  ntfy->queue_storage = heap_caps_calloc(
    1, queue_storage_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  ntfy->queue_buffer = heap_caps_calloc(
    1, sizeof(StaticQueue_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ntfy->queue_storage != NULL && ntfy->queue_buffer != NULL) {
    ntfy->queue = xQueueCreateStatic(ALERT_NTFY_QUEUE_LEN,
                                     sizeof(alert_notification_t),
                                     ntfy->queue_storage,
                                     ntfy->queue_buffer);
  } else {
    ESP_LOGE(kTag, "Failed to allocate ntfy queue storage in PSRAM");
    heap_caps_free(ntfy->queue_storage);
    heap_caps_free(ntfy->queue_buffer);
    ntfy->queue_storage = NULL;
    ntfy->queue_buffer = NULL;
  }

  const size_t job_queue_storage_bytes =
    sizeof(alert_ntfy_job_t) * ALERT_NTFY_JOB_QUEUE_LEN;
  ntfy->job_queue_storage = heap_caps_calloc(
    1, job_queue_storage_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  ntfy->job_queue_buffer = heap_caps_calloc(
    1, sizeof(StaticQueue_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ntfy->job_queue_storage != NULL && ntfy->job_queue_buffer != NULL) {
    ntfy->job_queue = xQueueCreateStatic(ALERT_NTFY_JOB_QUEUE_LEN,
                                         sizeof(alert_ntfy_job_t),
                                         ntfy->job_queue_storage,
                                         ntfy->job_queue_buffer);
  } else {
    ESP_LOGE(kTag, "Failed to allocate ntfy job queue storage in PSRAM");
    heap_caps_free(ntfy->job_queue_storage);
    heap_caps_free(ntfy->job_queue_buffer);
    ntfy->job_queue_storage = NULL;
    ntfy->job_queue_buffer = NULL;
  }
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
 * @brief Execute AlertNtfyEnqueueJob.
 * @param ntfy Parameter ntfy.
 * @param job Parameter job.
 * @return Return the function result.
 */
bool
AlertNtfyEnqueueJob(alert_ntfy_t* ntfy, const alert_ntfy_job_t* job)
{
  if (ntfy == NULL || job == NULL || ntfy->job_queue == NULL) {
    return false;
  }
  if (xQueueSend(ntfy->job_queue, job, 0) == pdTRUE) {
    return true;
  }
  alert_ntfy_job_t dropped_job;
  if (xQueueReceive(ntfy->job_queue, &dropped_job, 0) == pdTRUE) {
    ntfy->job_dropped++;
  }
  if (xQueueSend(ntfy->job_queue, job, 0) == pdTRUE) {
    return true;
  }
  ntfy->job_dropped++;
  if (LogRateLimitAllow(&ntfy->job_last_drop_log_ms,
                        kNtfyJobDropLogRateLimitMs)) {
    ESP_LOGW(kTag, "ntfy job queue full; dropping newest");
  }
  return false;
}

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
alert_ntfy_result_t
AlertNtfySend(alert_ntfy_t* ntfy,
              const alert_ntfy_config_t* cfg,
              const alert_notification_t* note,
              int* out_retry_after_seconds,
              int* out_status,
              esp_err_t* out_err)
{
  if (note == NULL) {
    if (out_err != NULL) {
      *out_err = ESP_ERR_INVALID_ARG;
    }
    return ALERT_NTFY_FAILED;
  }
  char leaf_id[32] = "";
  FormatLeafId(note->leaf_id, leaf_id, sizeof(leaf_id));

  char title[64];
  snprintf(
    title, sizeof(title), "PT100 %s", note->resolved ? "RESOLVED" : "ALERT");

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
      FormatMilliC(
        note->payload.current_temp_milli_c, temp_str, sizeof(temp_str));
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
      FormatMilliC(
        note->payload.current_temp_milli_c, temp_str, sizeof(temp_str));
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
               "type: leaf_sequence_reset\nlast_seq: %" PRIu32 "\n",
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
        case ALERT_SYSTEM_CODE_ERROR_NTFY_RATE_LIMIT:
          error = "ntfy_rate_limit";
          break;
        case ALERT_SYSTEM_CODE_ERROR_FRAM_ERRLOG:
          error = "fram_errlog";
          break;
        case ALERT_SYSTEM_CODE_ERROR_NTFY_QUEUE:
          error = "ntfy_queue_full";
          break;
        case ALERT_SYSTEM_CODE_ERROR_I2C_HANG:
          error = "i2c_hang";
          break;
        case ALERT_SYSTEM_CODE_ERROR_SD_OOS:
          error = "sd_oos";
          break;
        default:
          known = false;
          break;
      }
      snprintf(body + strlen(body),
               sizeof(body) - strlen(body),
               "type: system_error\nerror: %s\n",
               error);
      if (note->payload.event_code == ALERT_SYSTEM_CODE_ERROR_NTFY_RATE_LIMIT &&
          note->payload.duration_ms > 0) {
        snprintf(body + strlen(body),
                 sizeof(body) - strlen(body),
                 "suppressed: %" PRIu32 "\n",
                 note->payload.duration_ms);
      }
      if (!known) {
        snprintf(body + strlen(body),
                 sizeof(body) - strlen(body),
                 "error_code: %" PRIu32 "\n",
                 note->payload.event_code);
      }
      break;
    }
    default:
      snprintf(
        body + strlen(body), sizeof(body) - strlen(body), "type: unknown\n");
      break;
  }

  AppendTimeLine(&note->payload, body, sizeof(body));

  return AlertNtfySendHttp(ntfy,
                           cfg,
                           title,
                           body,
                           SeverityToPriority(note->severity),
                           NULL,
                           out_retry_after_seconds,
                           out_status,
                           out_err);
}

/**
 * @brief Execute AlertNtfySendText.
 * @param ntfy Parameter ntfy.
 * @param cfg Parameter cfg.
 * @param title Parameter title.
 * @param body Parameter body.
 * @param sequence_id Parameter sequence_id.
 * @param out_retry_after_seconds Parameter out_retry_after_seconds.
 * @param out_status Parameter out_status.
 * @param out_err Parameter out_err.
 * @return Return the function result.
 */
alert_ntfy_result_t
AlertNtfySendText(alert_ntfy_t* ntfy,
                  const alert_ntfy_config_t* cfg,
                  const char* title,
                  const char* body,
                  const char* sequence_id,
                  int* out_retry_after_seconds,
                  int* out_status,
                  esp_err_t* out_err)
{
  return AlertNtfySendHttp(ntfy,
                           cfg,
                           title,
                           body,
                           "default",
                           sequence_id,
                           out_retry_after_seconds,
                           out_status,
                           out_err);
}
