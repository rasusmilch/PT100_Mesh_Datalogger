#include "console_alerts.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alerts/alert_manager.h"
#include "alerts/alert_ntfy.h"
#include "console_help.h"
#include "console_registry.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "runtime_manager.h"

static void PrintAlertTypes(void);
static const alert_leaf_config_t* FindLeafOverride(
  const alert_manager_t* manager, uint64_t leaf_id);
static int ParseLeafId(const char* text, uint64_t* out);
static bool ParseTempAbsolute(const char* text, int32_t* out_milli_c);
static bool ParseTempDelta(const char* text, int32_t* out_delta_milli_c);
static bool ParseAlertType(const char* text, alert_type_t* out_type);
static bool IsHystSubcommand(const char* text);
static const char* AlertTypeToName(alert_type_t type);
static void PrintStatus(const alert_manager_t* manager);
static void PrintLeafList(const alert_manager_t* manager);
static int CommandAlert(int argc, char** argv);
static int CommandRebootLatch(int argc, char** argv);
static int AlertTopicHelp(const char* topic);
static void PrintAlertHelpBody(void);
static void PrintRebootLatchHelpBody(void);

static const char* kTag = "console_alerts";
static app_runtime_t* g_runtime = NULL;
static const char* const kAlertTypeNames[] = { "high",   "low",   "missing", "offline",
                                               "restart", "root", "boot",    "mode",
                                               "error",  "all" };
static const size_t kAlertTypeNameCount =
  sizeof(kAlertTypeNames) / sizeof(kAlertTypeNames[0]);

static const console_help_topic_t kAlertTopics[] = {
  {
    .name = "status",
    .summary = "Show global alert status and current thresholds",
    .synopsis = "alert status",
    .details = "Prints enabled alert types, notification/rate-limit settings, "
               "and active global plus per-leaf thresholds.",
    .options = NULL,
    .examples = "  alert status",
  },
  {
    .name = "list",
    .summary = "List leaf-level alert state (root only)",
    .synopsis = "alert list",
    .details = "Available only on the root node. Displays per-leaf alert "
               "state and effective overrides.",
    .options = NULL,
    .examples = "  alert list",
  },
  {
    .name = "types",
    .summary = "Print valid alert type names",
    .synopsis = "alert types",
    .details = "Prints the accepted type strings used by enable/clear "
               "operations.",
    .options = NULL,
    .examples = "  alert types",
  },
  {
    .name = "enable",
    .summary = "Enable or disable one alert type globally or per leaf",
    .synopsis =
      "alert enable <high|low|missing|offline|restart|root|boot|mode|error|all> <on|off> [leaf_id]",
    .details = "Sets the enable mask for an alert type. Optional leaf_id "
               "targeting is root-only and updates a leaf override.",
    .options = "  <type>       Alert type name or all.\n"
               "  <on|off>     Enable or disable alerts for that type.\n"
               "  [leaf_id]    Optional leaf target; root node only.",
    .examples = "  alert enable high on\n"
                "  alert enable missing off 98:A3:16:12:57:B0",
  },
  {
    .name = "set",
    .summary = "Configure alert limits, timers, and hysteresis",
    .synopsis = "alert set limit <leaf_id|default> <high|low> <value><C|F>\n"
                "alert set missing_ms <ms>\n"
                "alert set offline_ms <ms>\n"
                "alert set hold_ms <ms>\n"
                "alert set hyst <delta><C|F>",
    .details = "Updates temperature limits and alert timing behavior. Limit "
               "targets can be default (global) or a specific leaf override "
               "(root only).",
    .options = "  <leaf_id|default>  Limit target scope.\n"
               "  <high|low>         Select high or low threshold.\n"
               "  <value><C|F>       Threshold absolute temperature.\n"
               "  <ms>               Millisecond duration value.\n"
               "  <delta><C|F>       Hysteresis delta.",
    .examples = "  alert set limit default high 80C\n"
                "  alert set limit 98:A3:16:12:57:B0 low 30F\n"
                "  alert set missing_ms 30000\n"
                "  alert set hyst 0.5C",
  },
  {
    .name = "ntfy",
    .summary = "Configure ntfy endpoint fields and run a test",
    .synopsis = "alert ntfy set url <value>\n"
                "alert ntfy set topic <value>\n"
                "alert ntfy set token <value|clear>\n"
                "alert ntfy test",
    .details = "Configures ntfy delivery endpoint metadata used by alerts. "
               "Token is optional and may be cleared. Test sends a direct "
               "lightweight transport probe payload.",
    .options = "  url <value>          Ntfy server URL.\n"
               "  topic <value>        Ntfy topic name.\n"
               "  token <value|clear>  Optional bearer token.\n"
               "  test                 Send a test notification.",
    .examples = "  alert ntfy set url https://ntfy.sh\n"
                "  alert ntfy set topic PT100_Mesh_Datalogger\n"
                "  alert ntfy set token clear\n"
                "  alert ntfy test",
  },
  {
    .name = "notify",
    .summary = "Alias for ntfy",
    .synopsis = "alert notify ...",
    .details = "Alias for the ntfy subcommand. Use alert ntfy for canonical "
               "documentation and syntax.",
    .options = NULL,
    .examples = "  alert notify set topic PT100_Mesh_Datalogger",
  },
  {
    .name = "ratelimit",
    .summary = "Tune alert delivery rate-limit controls",
    .synopsis = "alert ratelimit set per_key_ms <ms>\n"
                "alert ratelimit set per_minute <n>\n"
                "alert ratelimit set min_interval_ms <ms>",
    .details = "Adjusts global rate limiting applied before notification "
               "delivery.",
    .options = "  per_key_ms <ms>       Per-alert-key cooldown.\n"
               "  per_minute <n>        Global send cap per minute.\n"
               "  min_interval_ms <ms>  Minimum spacing for ntfy sends.",
    .examples = "  alert ratelimit set per_key_ms 60000\n"
                "  alert ratelimit set per_minute 20\n"
                "  alert ratelimit set min_interval_ms 300000",
  },
  {
    .name = "clear",
    .summary = "Clear active alerts by type",
    .synopsis =
      "alert clear <high|low|missing|offline|restart|root|boot|mode|error|all> [leaf_id]",
    .details = "Clears active alert state globally or for one leaf. Optional "
               "leaf selection is root-only.",
    .options = "  <type|all>  Alert type to clear, or all.\n"
               "  [leaf_id]   Optional leaf target; root node only.",
    .examples = "  alert clear high\n"
                "  alert clear all\n"
                "  alert clear missing 98:A3:16:12:57:B0",
  },
  { 0 },
};

static const char*
RebootLatchGateReasonToString(runtime_reboot_alert_gate_reason_t reason)
{
  switch (reason) {
    case RUNTIME_REBOOT_ALERT_GATE_NOT_CONFIGURED:
      return "not_configured";
    case RUNTIME_REBOOT_ALERT_GATE_DISABLED_BY_MASK:
      return "disabled_by_mask";
    case RUNTIME_REBOOT_ALERT_GATE_NOT_ELIGIBLE_NET_MODE:
      return "not_eligible_net_mode";
    case RUNTIME_REBOOT_ALERT_GATE_NOT_ELIGIBLE_ROLE:
      return "not_eligible_role";
    case RUNTIME_REBOOT_ALERT_GATE_WIFI_DISCONNECTED:
      return "wifi_disconnected";
    case RUNTIME_REBOOT_ALERT_GATE_MESH_CONNECTED_BLOCKING_DIRECT:
      return "mesh_connected";
    case RUNTIME_REBOOT_ALERT_GATE_COOLDOWN_ACTIVE:
      return "cooldown_active";
    case RUNTIME_REBOOT_ALERT_GATE_QUEUE_FULL:
      return "queue_full";
    case RUNTIME_REBOOT_ALERT_GATE_UNKNOWN:
    default:
      return "unknown";
  }
}

static const char*
RebootLatchSendResultToString(runtime_reboot_alert_send_result_t result)
{
  switch (result) {
    case RUNTIME_REBOOT_ALERT_SEND_OK:
      return "ok";
    case RUNTIME_REBOOT_ALERT_SEND_FAIL:
      return "fail";
    case RUNTIME_REBOOT_ALERT_SEND_SKIPPED:
      return "skipped";
    case RUNTIME_REBOOT_ALERT_SEND_NONE:
    default:
      return "none";
  }
}

/**
 * @brief Print valid alert type strings to stdout.
 */
static void
PrintAlertTypes(void)
{
  for (size_t i = 0; i < kAlertTypeNameCount; ++i) {
    if (i > 0) {
      printf(" ");
    }
    printf("%s", kAlertTypeNames[i]);
  }
  printf("\n");
}

/**
 * @brief Execute FindLeafOverride.
 * @param manager Parameter manager.
 * @param leaf_id Parameter leaf_id.
 * @return Return the function result.
 */
static const alert_leaf_config_t*
FindLeafOverride(const alert_manager_t* manager, uint64_t leaf_id)
{
  if (manager == NULL) {
    return NULL;
  }
  for (size_t i = 0; i < manager->config.leaf_override_count; ++i) {
    if (manager->config.leaf_overrides[i].leaf_id == leaf_id) {
      return &manager->config.leaf_overrides[i];
    }
  }
  return NULL;
}

/**
 * @brief Execute ParseLeafId.
 * @param text Parameter text.
 * @param out Parameter out.
 * @return Return the function result.
 */
static int
ParseLeafId(const char* text, uint64_t* out)
{
  if (text == NULL || out == NULL) {
    return 0;
  }
  int values[6] = { 0 };
  if (sscanf(text,
             "%x:%x:%x:%x:%x:%x",
             &values[0],
             &values[1],
             &values[2],
             &values[3],
             &values[4],
             &values[5]) != 6) {
    return 0;
  }
  uint64_t id = 0;
  for (int i = 0; i < 6; ++i) {
    id = (id << 8) | (uint8_t)values[i];
  }
  *out = id;
  return 1;
}

/**
 * @brief Parse an absolute temperature value (e.g. limits).
 * @param text Input string formatted as <value><C|F>.
 * @param out_milli_c Output in milli-degrees C.
 * @return true on success, false otherwise.
 */
static bool
ParseTempAbsolute(const char* text, int32_t* out_milli_c)
{
  if (text == NULL || out_milli_c == NULL) {
    return false;
  }
  char* end = NULL;
  double value = strtod(text, &end);
  if (end == text) {
    return false;
  }
  while (*end != '\0' && isspace((unsigned char)*end)) {
    ++end;
  }
  if (*end == '\0') {
    return false;
  }
  char unit = (char)toupper((unsigned char)*end);
  if (unit != 'C' && unit != 'F') {
    return false;
  }
  ++end;
  while (*end != '\0' && isspace((unsigned char)*end)) {
    ++end;
  }
  if (*end != '\0') {
    return false;
  }
  double value_c = (unit == 'F') ? ((value - 32.0) * 5.0 / 9.0) : value;
  *out_milli_c = (int32_t)llround(value_c * 1000.0);
  return true;
}

/**
 * @brief Parse a temperature delta value (e.g. hysteresis).
 * @param text Input string formatted as <delta><C|F>.
 * @param out_milli_c Output delta in milli-degrees C.
 * @return true on success, false otherwise.
 */
static bool
ParseTempDelta(const char* text, int32_t* out_milli_c)
{
  if (text == NULL || out_milli_c == NULL) {
    return false;
  }
  char* end = NULL;
  double value = strtod(text, &end);
  if (end == text) {
    return false;
  }
  while (*end != '\0' && isspace((unsigned char)*end)) {
    ++end;
  }
  if (*end == '\0') {
    return false;
  }
  char unit = (char)toupper((unsigned char)*end);
  if (unit != 'C' && unit != 'F') {
    return false;
  }
  ++end;
  while (*end != '\0' && isspace((unsigned char)*end)) {
    ++end;
  }
  if (*end != '\0') {
    return false;
  }
  double value_c = (unit == 'F') ? (value * 5.0 / 9.0) : value;
  *out_milli_c = (int32_t)llround(value_c * 1000.0);
  return true;
}

/**
 * @brief Execute ParseAlertType.
 * @param text Parameter text.
 * @param out Parameter out.
 * @return Return the function result.
 */
static bool
ParseAlertType(const char* text, alert_type_t* out)
{
  if (text == NULL || out == NULL) {
    return false;
  }
  if (strcmp(text, "high") == 0) {
    *out = ALERT_TEMP_HIGH;
    return true;
  }
  if (strcmp(text, "low") == 0) {
    *out = ALERT_TEMP_LOW;
    return true;
  }
  if (strcmp(text, "missing") == 0) {
    *out = ALERT_MISSING_RECORDS;
    return true;
  }
  if (strcmp(text, "offline") == 0) {
    *out = ALERT_LEAF_OFFLINE;
    return true;
  }
  if (strcmp(text, "restart") == 0) {
    *out = ALERT_LEAF_RESTART;
    return true;
  }
  if (strcmp(text, "root") == 0) {
    *out = ALERT_ROOT_RESTART;
    return true;
  }
  if (strcmp(text, "boot") == 0) {
    *out = ALERT_SYSTEM_BOOT;
    return true;
  }
  if (strcmp(text, "mode") == 0) {
    *out = ALERT_SYSTEM_MODE;
    return true;
  }
  if (strcmp(text, "error") == 0) {
    *out = ALERT_SYSTEM_ERROR;
    return true;
  }
  return false;
}

/**
 * @brief Check whether subcommand refers to hysteresis.
 * @param text Subcommand string.
 * @return true if matches hyst/hyster/hysteresis.
 */
static bool
IsHystSubcommand(const char* text)
{
  if (text == NULL) {
    return false;
  }
  return (strcmp(text, "hyst") == 0 || strcmp(text, "hyster") == 0
          || strcmp(text, "hysteresis") == 0);
}

/**
 * @brief Execute AlertTypeToName.
 * @param type Parameter type.
 * @return Return the function result.
 */
static const char*
AlertTypeToName(alert_type_t type)
{
  switch (type) {
    case ALERT_TEMP_HIGH:
      return "high";
    case ALERT_TEMP_LOW:
      return "low";
    case ALERT_MISSING_RECORDS:
      return "missing";
    case ALERT_LEAF_OFFLINE:
      return "offline";
    case ALERT_LEAF_RESTART:
      return "restart";
    case ALERT_ROOT_RESTART:
      return "root";
    case ALERT_SYSTEM_BOOT:
      return "boot";
    case ALERT_SYSTEM_MODE:
      return "mode";
    case ALERT_SYSTEM_ERROR:
      return "error";
    default:
      return "unknown";
  }
}

/**
 * @brief Execute PrintStatus.
 * @param manager Parameter manager.
 */
static void
PrintStatus(const alert_manager_t* manager)
{
  if (manager == NULL) {
    return;
  }
  const alert_config_t* cfg = &manager->config;
  int32_t hyst_whole = cfg->hysteresis_milli_c / 1000;
  int32_t hyst_frac = cfg->hysteresis_milli_c % 1000;
  if (hyst_frac < 0) {
    hyst_frac = -hyst_frac;
  }
  int32_t high_whole = cfg->default_high_milli_c / 1000;
  int32_t high_frac = cfg->default_high_milli_c % 1000;
  if (high_frac < 0) {
    high_frac = -high_frac;
  }
  int32_t low_whole = cfg->default_low_milli_c / 1000;
  int32_t low_frac = cfg->default_low_milli_c % 1000;
  if (low_frac < 0) {
    low_frac = -low_frac;
  }
  printf("ntfy: url=%s topic=%s token=%s\n",
         cfg->ntfy_url[0] ? cfg->ntfy_url : "<unset>",
         cfg->ntfy_topic[0] ? cfg->ntfy_topic : "<unset>",
         cfg->ntfy_token[0] ? "<set>" : "<empty>");
  printf("enable_mask: 0x%08" PRIX32 "\n", cfg->enable_mask);
  printf("rate_limit: per_key_ms=%" PRIu32 " per_minute=%" PRIu32
         " min_interval_ms=%" PRIu32 "\n",
         cfg->per_key_cooldown_ms,
         cfg->global_max_per_minute,
         cfg->ntfy_min_send_interval_ms);
  printf("missing_gap_ms=%" PRIu32 " offline_ms=%" PRIu32
         " hold_ms=%" PRIu32 " hyst=%" PRIi32 ".%03" PRIi32 "C\n",
         cfg->missing_gap_ms,
         cfg->offline_ms,
         cfg->hold_ms,
         hyst_whole,
         hyst_frac);
  printf("default_limits: high=%" PRIi32 ".%03" PRIi32 "C low=%" PRIi32 ".%03" PRIi32 "C\n",
         high_whole,
         high_frac,
         low_whole,
         low_frac);
  printf("queue: depth=%" PRIu32 " dropped=%" PRIu32
         " send_ok=%" PRIu32 " send_fail=%" PRIu32
         " last_http_status=%d last_err=%s last_dns=%d last_ip=%s\n",
         (uint32_t)uxQueueMessagesWaiting(manager->ntfy.queue),
         manager->ntfy.dropped,
         manager->ntfy.send_success,
         manager->ntfy.send_fail,
         manager->ntfy.last_http_status,
         esp_err_to_name(manager->ntfy.last_err),
         manager->ntfy.last_dns_result,
         manager->ntfy.last_resolved_ip[0] ? manager->ntfy.last_resolved_ip : "<none>");
  printf("http_queue: depth=%" PRIu32 " dropped=%" PRIu32 " backoff_ms=%" PRIu32
         " cooldown_until_ms=%" PRId64 "\n",
         manager->ntfy.job_queue != NULL
           ? (uint32_t)uxQueueMessagesWaiting(manager->ntfy.job_queue)
           : 0u,
         manager->ntfy.job_dropped,
         manager->ntfy.backoff_ms,
         manager->ntfy.cooldown_until_ms);

  size_t active_count = 0;
  for (size_t leaf_index = 0; leaf_index < ALERT_MAX_LEAVES; ++leaf_index) {
    if (!manager->leaves[leaf_index].in_use) {
      continue;
    }
    for (size_t type_index = 0; type_index < ALERT_TYPE_COUNT; ++type_index) {
      if (manager->states[leaf_index][type_index].active) {
        ++active_count;
      }
    }
  }
  printf("active alerts: %u\n", (unsigned)active_count);

  const size_t print_limit = 16;
  size_t printed = 0;
  for (size_t leaf_index = 0; leaf_index < ALERT_MAX_LEAVES; ++leaf_index) {
    if (!manager->leaves[leaf_index].in_use) {
      continue;
    }
    for (size_t type_index = 0; type_index < ALERT_TYPE_COUNT; ++type_index) {
      const alert_state_t* state = &manager->states[leaf_index][type_index];
      if (!state->active) {
        continue;
      }
      if (printed < print_limit) {
        char leaf_str[24];
        AlertManagerFormatLeafId(manager->leaves[leaf_index].leaf_id,
                                 leaf_str,
                                 sizeof(leaf_str));
        printf("  %s %s transitions=%" PRIu32 " last_change_ms=%" PRIi64 "\n",
               leaf_str,
               AlertTypeToName((alert_type_t)type_index),
               state->transitions,
               state->last_change_ms);
        ++printed;
      }
    }
  }

  if (active_count > printed) {
    printf("  ... (%u more)\n", (unsigned)(active_count - printed));
  }
}

/**
 * @brief Execute PrintLeafList.
 * @param manager Parameter manager.
 */
static void
PrintLeafList(const alert_manager_t* manager)
{
  if (manager == NULL) {
    return;
  }
  alert_leaf_state_t leaves[ALERT_MAX_LEAVES];
  size_t count = AlertManagerCopyLeaves(manager, leaves, ALERT_MAX_LEAVES);
  printf("leaves: %u\n", (unsigned)count);
  for (size_t i = 0; i < count; ++i) {
    char leaf_str[24];
    AlertManagerFormatLeafId(leaves[i].leaf_id, leaf_str, sizeof(leaf_str));
    int32_t high_limit = manager->config.default_high_milli_c;
    int32_t low_limit = manager->config.default_low_milli_c;
    const alert_leaf_config_t* override = FindLeafOverride(manager, leaves[i].leaf_id);
    if (override != NULL && override->has_limits) {
      high_limit = override->high_limit_milli_c;
      low_limit = override->low_limit_milli_c;
    }
    const int32_t temp_whole = leaves[i].last_temp_milli_c / 1000;
    int32_t temp_frac = leaves[i].last_temp_milli_c % 1000;
    if (temp_frac < 0) {
      temp_frac = -temp_frac;
    }
    const int32_t high_whole = high_limit / 1000;
    int32_t high_frac = high_limit % 1000;
    if (high_frac < 0) {
      high_frac = -high_frac;
    }
    const int32_t low_whole = low_limit / 1000;
    int32_t low_frac = low_limit % 1000;
    if (low_frac < 0) {
      low_frac = -low_frac;
    }
    printf("  %s online=%u temp=%" PRIi32 ".%03" PRIi32 "C last_seq=%" PRIu32
           " last_rx_ms=%" PRIi64 " limits=[%" PRIi32 ".%03" PRIi32 "C/%" PRIi32 ".%03" PRIi32 "C]\n",
           leaf_str,
           leaves[i].online ? 1u : 0u,
           temp_whole,
           temp_frac,
           leaves[i].last_seq,
           leaves[i].last_rx_uptime_ms,
           high_whole,
           high_frac,
           low_whole,
           low_frac);
  }
}

/**
 * @brief Execute CommandAlert.
 * @param argc Parameter argc.
 * @param argv Parameter argv.
 * @return Return the function result.
 */
static int
CommandAlert(int argc, char** argv)
{
  if (g_runtime == NULL || g_runtime->alert_manager == NULL) {
    return 1;
  }
  const bool is_root = (g_runtime->settings->node_role == APP_NODE_ROLE_ROOT);

  if (argc < 2) {
    if (is_root) {
      printf("usage: alert status | alert list | alert types | alert enable <high|low|missing|offline|restart|root|boot|mode|error|all> <on|off> [leaf]\n"
             "       alert set limit <leaf|default> <high|low> <value><C|F>\n"
             "       alert set missing_ms <ms> | alert set offline_ms <ms> | alert set hold_ms <ms> | alert set hyst <delta><C|F>\n"
             "       alert ntfy set url|topic|token <value>|clear | alert ntfy test\n"
             "       alert ratelimit set per_key_ms <ms> | alert ratelimit set per_minute <n> | alert ratelimit set min_interval_ms <ms>\n"
             "       alert clear <high|low|missing|offline|restart|root|boot|mode|error|all> [leaf]\n"
             "note: ntfy token is optional (only needed for protected topics)\n");
    } else {
      printf("usage: alert status | alert types | alert enable <high|low|missing|offline|restart|root|boot|mode|error|all> <on|off>\n"
             "       alert set limit default <high|low> <value><C|F>\n"
             "       alert set missing_ms <ms> | alert set offline_ms <ms> | alert set hold_ms <ms> | alert set hyst <delta><C|F>\n"
             "       alert ntfy set url|topic|token <value>|clear | alert ntfy test\n"
             "       alert ratelimit set per_key_ms <ms> | alert ratelimit set per_minute <n> | alert ratelimit set min_interval_ms <ms>\n"
             "       alert clear <high|low|missing|offline|restart|root|boot|mode|error|all>\n"
             "note: leaf overrides and 'alert list' require node role root\n"
             "note: ntfy token is optional (only needed for protected topics)\n");
    }
    printf("valid types: ");
    PrintAlertTypes();
    return 1;
  }

  alert_manager_t* manager = g_runtime->alert_manager;
  const char* action = argv[1];
  if (strcmp(action, "notify") == 0) {
    action = "ntfy";
  }

  if (strcmp(action, "status") == 0) {
    PrintStatus(manager);
    return 0;
  }
  if (strcmp(action, "list") == 0) {
    if (!is_root) {
      printf("alert list is only available on the root node\n");
      return 1;
    }
    PrintLeafList(manager);
    return 0;
  }
  if (strcmp(action, "types") == 0) {
    PrintAlertTypes();
    return 0;
  }
  if (strcmp(action, "enable") == 0) {
    if (argc < 4) {
      printf("usage: alert enable <high|low|missing|offline|restart|root|boot|mode|error|all> <on|off> [leaf]\n");
      printf("valid types: ");
      PrintAlertTypes();
      printf("note: high/low are temperature thresholds; others are system/health events\n");
      return 1;
    }
    const char* type_str = argv[2];
    const char* state_str = argv[3];
    bool enable = (strcmp(state_str, "on") == 0);
    bool disable = (strcmp(state_str, "off") == 0);
    if (!enable && !disable) {
      printf("usage: alert enable <high|low|missing|offline|restart|root|boot|mode|error|all> <on|off> [leaf]\n");
      printf("valid types: ");
      PrintAlertTypes();
      printf("note: high/low are temperature thresholds; others are system/health events\n");
      return 1;
    }
    bool per_leaf = false;
    uint64_t leaf_id = 0;
    if (argc > 4) {
      if (!is_root) {
        printf("leaf overrides are only available on the root node\n");
        return 1;
      }
      per_leaf = ParseLeafId(argv[4], &leaf_id);
      if (!per_leaf) {
        printf("invalid leaf id\n");
        return 1;
      }
    }
    if (strcmp(type_str, "all") == 0) {
      bool ok = true;
      for (int t = 0; t < ALERT_TYPE_COUNT; ++t) {
        ok &= AlertManagerEnableType(manager,
                                     (alert_type_t)t,
                                     enable,
                                     leaf_id,
                                     per_leaf);
      }
      printf("alert enable all %s\n", ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    alert_type_t type;
    if (!ParseAlertType(type_str, &type)) {
      printf("invalid type\n");
      printf("valid types: ");
      PrintAlertTypes();
      return 1;
    }
    bool ok = AlertManagerEnableType(manager, type, enable, leaf_id, per_leaf);
    printf("alert enable %s %s\n", type_str, ok ? "ok" : "failed");
    return ok ? 0 : 1;
  }
  if (strcmp(action, "set") == 0) {
    if (argc < 3) {
      printf("usage: alert set limit|missing_ms|offline_ms|hold_ms|hyst ...\n");
      return 1;
    }
    const char* sub = argv[2];
    if (strcmp(sub, "limit") == 0) {
      if (argc != 6) {
        printf("usage: alert set limit <leaf|default> <high|low> <value><C|F>\n");
        return 1;
      }
      const char* target = argv[3];
      const char* which = argv[4];
      const char* value = argv[5];
      int32_t milli_c = 0;
      if (!ParseTempAbsolute(value, &milli_c)) {
        printf("invalid temp\n");
        return 1;
      }
      bool is_high = (strcmp(which, "high") == 0);
      bool is_low = (strcmp(which, "low") == 0);
      if (!is_high && !is_low) {
        printf("invalid limit type\n");
        return 1;
      }
      bool ok = false;
      if (strcmp(target, "default") == 0) {
        ok = AlertManagerSetDefaultLimit(manager, is_high, milli_c);
      } else {
        if (!is_root) {
          printf("leaf overrides are only available on the root node\n");
          return 1;
        }
        uint64_t leaf_id = 0;
        if (!ParseLeafId(target, &leaf_id)) {
          printf("invalid leaf id\n");
          return 1;
        }
        ok = AlertManagerSetLeafLimit(manager, leaf_id, is_high, milli_c);
      }
      printf("alert set limit %s\n", ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    if (strcmp(sub, "missing_ms") == 0) {
      if (argc != 4) {
        printf("usage: alert set missing_ms <ms>\n");
        return 1;
      }
      uint32_t value = (uint32_t)strtoul(argv[3], NULL, 10);
      bool ok = AlertManagerSetMissingGap(manager, value);
      printf("alert set missing_ms %s\n", ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    if (strcmp(sub, "offline_ms") == 0) {
      if (argc != 4) {
        printf("usage: alert set offline_ms <ms>\n");
        return 1;
      }
      uint32_t value = (uint32_t)strtoul(argv[3], NULL, 10);
      bool ok = AlertManagerSetOfflineMs(manager, value);
      printf("alert set offline_ms %s\n", ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    if (strcmp(sub, "hold_ms") == 0) {
      if (argc != 4) {
        printf("usage: alert set hold_ms <ms>\n");
        return 1;
      }
      uint32_t value = (uint32_t)strtoul(argv[3], NULL, 10);
      bool ok = AlertManagerSetHoldMs(manager, value);
      printf("alert set hold_ms %s\n", ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    if (IsHystSubcommand(sub)) {
      if (argc != 4) {
        printf("usage: alert set hyst <delta><C|F>\n");
        return 1;
      }
      int32_t milli_c = 0;
      if (!ParseTempDelta(argv[3], &milli_c)) {
        printf("invalid hysteresis (expected <value><C|F>, e.g. 0.2C or 0.2F)\n");
        return 1;
      }
      bool ok = AlertManagerSetHysteresis(manager, milli_c);
      printf("alert set hyst %s\n", ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    printf("unknown alert set target\n");
    return 1;
  }
  if (strcmp(action, "ntfy") == 0) {
    if (argc < 3) {
      printf("usage: alert ntfy set url|topic|token <value>|clear | alert ntfy test\n");
      printf("note: ntfy token is optional (only needed for protected topics)\n");
      return 1;
    }
    const char* sub = argv[2];
    if (strcmp(sub, "set") == 0) {
      if (argc < 5) {
        printf("usage: alert ntfy set url|topic|token <value>|clear\n");
        printf("note: ntfy token is optional (only needed for protected topics)\n");
        return 1;
      }
      const char* field = argv[3];
      const char* value = argv[4];
      bool ok = false;
      if (strcmp(field, "url") == 0) {
        char normalized_url[128] = { 0 };
        alert_ntfy_url_sanitize_result_t sanitize_reason =
          ALERT_NTFY_URL_SANITIZE_OK;
        if (!AlertNtfySanitizeBaseUrl(value,
                                      normalized_url,
                                      sizeof(normalized_url),
                                      &sanitize_reason)) {
          printf("invalid ntfy url: reason=%s\n",
                 AlertNtfyUrlSanitizeResultToString(sanitize_reason));
          ok = false;
        } else {
          ok = AlertManagerSetNtfyUrl(manager, normalized_url);
          if (!ok) {
            printf("failed to persist ntfy url\n");
          }
        }
      } else if (strcmp(field, "topic") == 0) {
        ok = AlertManagerSetNtfyTopic(manager, value);
      } else if (strcmp(field, "token") == 0) {
        if (strcmp(value, "clear") == 0) {
          ok = AlertManagerSetNtfyToken(manager, "");
        } else {
          ok = AlertManagerSetNtfyToken(manager, value);
        }
      } else {
        printf("unknown ntfy field\n");
        return 1;
      }
      printf("alert ntfy set %s %s\n", field, ok ? "ok" : "failed");
      return ok ? 0 : 1;
    }
    if (strcmp(sub, "test") == 0) {
      const uint32_t internal_free =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      const uint32_t internal_largest = heap_caps_get_largest_free_block(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      printf("alert ntfy test path: direct transport probe "
             "(title='PT100 ntfy test' body='ntfy test')\n");
      printf("alert ntfy test mem[enqueue]: internal_free=%" PRIu32
             " internal_largest=%" PRIu32 "\n",
             internal_free,
             internal_largest);
      const bool queued =
        AlertManagerSendDirectNtfyTest(manager, esp_timer_get_time() / 1000);
      printf("alert ntfy test %s\n", queued ? "queued" : "queue_failed");
      return queued ? 0 : 1;
    }
    printf("unknown ntfy command\n");
    return 1;
  }
  if (strcmp(action, "ratelimit") == 0) {
    if (argc < 5 || strcmp(argv[2], "set") != 0) {
      printf("usage: alert ratelimit set per_key_ms <ms> | per_minute <n> | min_interval_ms <ms>\n");
      return 1;
    }
    const char* field = argv[3];
    uint32_t value = (uint32_t)strtoul(argv[4], NULL, 10);
    bool ok = false;
    if (strcmp(field, "per_key_ms") == 0) {
      ok = AlertManagerSetRateLimit(manager,
                                    value,
                                    manager->config.global_max_per_minute);
    } else if (strcmp(field, "per_minute") == 0) {
      ok = AlertManagerSetRateLimit(manager,
                                    manager->config.per_key_cooldown_ms,
                                    value);
    } else if (strcmp(field, "min_interval_ms") == 0) {
      ok = AlertManagerSetNtfyMinIntervalMs(manager, value);
    } else {
      printf("unknown ratelimit field\n");
      return 1;
    }
    printf("alert ratelimit set %s %s\n", field, ok ? "ok" : "failed");
    return ok ? 0 : 1;
  }
  if (strcmp(action, "clear") == 0) {
    if (argc < 3) {
      printf("usage: alert clear <high|low|missing|offline|restart|root|boot|mode|error|all> [leaf]\n");
      printf("valid types: ");
      PrintAlertTypes();
      return 1;
    }
    const char* type_str = argv[2];
    alert_type_t type = ALERT_TYPE_COUNT;
    if (strcmp(type_str, "all") != 0) {
      if (!ParseAlertType(type_str, &type)) {
        printf("invalid type\n");
        printf("valid types: ");
        PrintAlertTypes();
        return 1;
      }
    }
    bool all_leaves = true;
    uint64_t leaf_id = 0;
    if (argc > 3) {
      if (!is_root) {
        printf("leaf selection is only available on the root node\n");
        return 1;
      }
      all_leaves = false;
      if (!ParseLeafId(argv[3], &leaf_id)) {
        printf("invalid leaf id\n");
        return 1;
      }
    }
    AlertManagerClear(manager, type, leaf_id, all_leaves);
    printf("alert clear ok\n");
    return 0;
  }

  printf("unknown alert command\n");
  return 1;
}

static int
CommandRebootLatch(int argc, char** argv)
{
  if (argc < 2) {
    printf("usage: reboot_latch <status|clear>\n");
    return 1;
  }
  const char* action = argv[1];
  if (strcmp(action, "status") == 0) {
    runtime_reboot_alert_latch_t latch = { 0 };
    RuntimeRebootAlertLatchCopy(&latch);
    if (latch.magic == 0) {
      printf("reboot latch invalid\n");
      return 0;
    }
    printf("pending=%s code=%" PRIu32 "\n",
           latch.pending_is_active ? "true" : "false",
           latch.pending_system_code);
    printf("pending_epoch=%" PRIi64 " pending_uptime_ms=%" PRIu32 "\n",
           latch.pending_epoch,
           latch.pending_uptime_ms);
    printf("attempts=%" PRIu32 " sent_success=%s\n",
           latch.send_attempt_count,
           latch.sent_successfully ? "true" : "false");
    printf("last_attempt_epoch=%" PRIi64 " last_attempt_uptime_ms=%" PRIu32
           "\n",
           latch.last_attempt_epoch,
           latch.last_attempt_uptime_ms);
    printf("last_gate=%s last_send=%s\n",
           RebootLatchGateReasonToString(
             (runtime_reboot_alert_gate_reason_t)latch.last_gate_reason),
           RebootLatchSendResultToString(
             (runtime_reboot_alert_send_result_t)latch.last_send_result));
    printf("last_http_status=%" PRIi32 " last_ntfy_err=%s\n",
           latch.last_http_status,
           esp_err_to_name((esp_err_t)latch.last_ntfy_err));
    printf("last_retry_after_seconds=%" PRIi32 "\n",
           latch.last_retry_after_seconds);
    return 0;
  }
  if (strcmp(action, "clear") == 0) {
    RuntimeRebootAlertLatchClearSticky();
    printf("reboot latch cleared\n");
    return 0;
  }

  printf("unknown reboot_latch command\n");
  return 1;
}

/**
 * @brief Execute ConsoleAlertsRegister.
 * @param runtime Parameter runtime.
 */
void
ConsoleAlertsRegister(app_runtime_t* runtime)
{
  g_runtime = runtime;
  static const console_registry_entry_t alert_cmd = {
    .command = "alert",
    .summary = "Manage alerting rules and status",
    .synopsis = "alert <status|list|types|enable|set|ntfy|ratelimit|clear> ...",
    .description = "Inspect and configure the device alert manager.",
    .print_body = PrintAlertHelpBody,
    .topic_help = &AlertTopicHelp,
    .func = &CommandAlert,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&alert_cmd));
  static const console_registry_entry_t reboot_latch_cmd = {
    .command = "reboot_latch",
    .summary = "Inspect or clear reboot alert latch",
    .synopsis = "reboot_latch <status|clear>",
    .description = "Shows persisted reboot alert delivery state.",
    .print_body = PrintRebootLatchHelpBody,
    .func = &CommandRebootLatch,
  };
  ESP_ERROR_CHECK(ConsoleRegistryRegister(&reboot_latch_cmd));
  ESP_LOGD(kTag, "alert commands registered");
}

static void
PrintAlertHelpBody(void)
{
  ConsoleHelpPrintTopicList(kAlertTopics);
  printf("Use: help alert <subcommand>\n\n");
  printf("EXAMPLES\n");
  printf("  alert status\n");
  printf("  alert enable high on\n");
  printf("  alert set limit default high 80C\n");
  printf("  alert set hyst 0.5C\n");
  printf("  alert ntfy set url https://ntfy.sh\n");
  printf("  alert ntfy set topic PT100_Mesh_Datalogger\n");
  printf("  alert ratelimit set min_interval_ms 300000\n");
  printf("  alert clear all\n");
}

/**
 * @brief Print detailed alert subtopic help for a specific subcommand.
 * @param topic Alert subtopic name.
 * @return 0 if the topic was found and printed, otherwise 1.
 */
static int
AlertTopicHelp(const char* topic)
{
  if (topic == NULL || topic[0] == '\0') {
    return 1;
  }

  for (size_t i = 0; kAlertTopics[i].name != NULL; ++i) {
    if (strcmp(kAlertTopics[i].name, topic) == 0) {
      ConsoleHelpPrintTopicManpage("alert", &kAlertTopics[i]);
      return 0;
    }
  }

  return 1;
}

static void
PrintRebootLatchHelpBody(void)
{
  printf("EXAMPLES\n");
  printf("  reboot_latch status\n");
  printf("  reboot_latch clear\n");
}
