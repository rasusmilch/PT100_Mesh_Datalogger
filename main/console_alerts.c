#include "console_alerts.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alerts/alert_manager.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* kTag = "console_alerts";
static app_runtime_t* g_runtime = NULL;

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
 * @brief Execute ParseTemp.
 * @param text Parameter text.
 * @param out_milli_c Parameter out_milli_c.
 * @return Return the function result.
 */
static bool
ParseTemp(const char* text, int32_t* out_milli_c)
{
  if (text == NULL || out_milli_c == NULL) {
    return false;
  }
  char* end = NULL;
  double value = strtod(text, &end);
  if (end == text || *end == '\0') {
    return false;
  }
  char unit = (char)toupper((unsigned char)*end);
  if (unit != 'C' && unit != 'F') {
    return false;
  }
  double value_c = (unit == 'F') ? ((value - 32.0) * 5.0 / 9.0) : value;
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
  printf("rate_limit: per_key_ms=%" PRIu32 " per_minute=%" PRIu32 "\n",
         cfg->per_key_cooldown_ms,
         cfg->global_max_per_minute);
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
         " send_ok=%" PRIu32 " send_fail=%" PRIu32 " last_status=%d\n",
         (uint32_t)uxQueueMessagesWaiting(manager->ntfy.queue),
         manager->ntfy.dropped,
         manager->ntfy.send_success,
         manager->ntfy.send_fail,
         manager->ntfy.last_http_status);

  alert_state_t states[ALERT_MAX_LEAVES * ALERT_TYPE_COUNT];
  alert_type_t types[ALERT_MAX_LEAVES * ALERT_TYPE_COUNT];
  uint64_t leaf_ids[ALERT_MAX_LEAVES * ALERT_TYPE_COUNT];
  const size_t count = AlertManagerCopyActiveAlerts(
    manager, states, types, leaf_ids, ALERT_MAX_LEAVES * ALERT_TYPE_COUNT);
  printf("active alerts: %u\n", (unsigned)count);
  for (size_t i = 0; i < count; ++i) {
    char leaf_str[24];
    AlertManagerFormatLeafId(leaf_ids[i], leaf_str, sizeof(leaf_str));
    printf("  %s %s transitions=%" PRIu32 " last_change_ms=%" PRIi64 "\n",
           leaf_str,
           AlertTypeToName(types[i]),
           states[i].transitions,
           states[i].last_change_ms);
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
      printf("usage: alert status | alert list | alert enable <type|all> <on|off> [leaf]\n"
             "       alert set limit <leaf|default> <high|low> <value><C|F>\n"
             "       alert set missing_ms <ms> | alert set offline_ms <ms> | alert set hold_ms <ms> | alert set hyst <value><C|F>\n"
             "       alert ntfy set url|topic|token <value>|clear | alert ntfy test\n"
             "       alert ratelimit set per_key_ms <ms> | alert ratelimit set per_minute <n>\n"
             "       alert clear <type|all> [leaf]\n");
    } else {
      printf("usage: alert status | alert enable <type|all> <on|off>\n"
             "       alert set limit default <high|low> <value><C|F>\n"
             "       alert set missing_ms <ms> | alert set offline_ms <ms> | alert set hold_ms <ms> | alert set hyst <value><C|F>\n"
             "       alert ntfy set url|topic|token <value>|clear | alert ntfy test\n"
             "       alert ratelimit set per_key_ms <ms> | alert ratelimit set per_minute <n>\n"
             "       alert clear <type|all>\n"
             "note: leaf overrides and 'alert list' require node role root\n");
    }
    return 1;
  }

  alert_manager_t* manager = g_runtime->alert_manager;
  const char* action = argv[1];

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
  if (strcmp(action, "enable") == 0) {
    if (argc < 4) {
      printf("usage: alert enable <type|all> <on|off> [leaf]\n");
      return 1;
    }
    const char* type_str = argv[2];
    const char* state_str = argv[3];
    bool enable = (strcmp(state_str, "on") == 0);
    bool disable = (strcmp(state_str, "off") == 0);
    if (!enable && !disable) {
      printf("invalid enable state\n");
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
      if (!ParseTemp(value, &milli_c)) {
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
    if (strcmp(sub, "hyst") == 0) {
      if (argc != 4) {
        printf("usage: alert set hyst <value><C|F>\n");
        return 1;
      }
      int32_t milli_c = 0;
      if (!ParseTemp(argv[3], &milli_c)) {
        printf("invalid hysteresis\n");
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
      return 1;
    }
    const char* sub = argv[2];
    if (strcmp(sub, "set") == 0) {
      if (argc < 5) {
        printf("usage: alert ntfy set url|topic|token <value>|clear\n");
        return 1;
      }
      const char* field = argv[3];
      const char* value = argv[4];
      bool ok = false;
      if (strcmp(field, "url") == 0) {
        ok = AlertManagerSetNtfyUrl(manager, value);
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
      AlertManagerSendTest(manager, esp_timer_get_time() / 1000);
      printf("alert ntfy test queued\n");
      return 0;
    }
    printf("unknown ntfy command\n");
    return 1;
  }
  if (strcmp(action, "ratelimit") == 0) {
    if (argc < 5 || strcmp(argv[2], "set") != 0) {
      printf("usage: alert ratelimit set per_key_ms <ms> | per_minute <n>\n");
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
    } else {
      printf("unknown ratelimit field\n");
      return 1;
    }
    printf("alert ratelimit set %s %s\n", field, ok ? "ok" : "failed");
    return ok ? 0 : 1;
  }
  if (strcmp(action, "clear") == 0) {
    if (argc < 3) {
      printf("usage: alert clear <type|all> [leaf]\n");
      return 1;
    }
    const char* type_str = argv[2];
    alert_type_t type = ALERT_TYPE_COUNT;
    if (strcmp(type_str, "all") != 0) {
      if (!ParseAlertType(type_str, &type)) {
        printf("invalid type\n");
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

/**
 * @brief Execute ConsoleAlertsRegister.
 * @param runtime Parameter runtime.
 */
void
ConsoleAlertsRegister(app_runtime_t* runtime)
{
  g_runtime = runtime;
  const esp_console_cmd_t alert_cmd = {
    .command = "alert",
    .help = "Alerting commands: alert status | alert list | alert enable ...",
    .hint = NULL,
    .func = &CommandAlert,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&alert_cmd));
  ESP_LOGD(kTag, "alert commands registered");
}
