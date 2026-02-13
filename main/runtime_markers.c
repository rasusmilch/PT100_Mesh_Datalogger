#include "runtime_markers.h"

#include <inttypes.h>
#include <stdio.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime_manager.h"
#include "time_sync.h"

static const char* kTag = "marker";

RTC_NOINIT_ATTR static uint32_t g_last_storage_marker;
RTC_NOINIT_ATTR static uint32_t g_last_sd_flush_marker;
RTC_NOINIT_ATTR static uint32_t g_last_marker_uptime_ms;
RTC_NOINIT_ATTR static uint32_t g_last_marker_epoch_sec;
RTC_NOINIT_ATTR static uint32_t g_last_marker_reset_reason;

static const char*
RuntimeMarkerToString(runtime_marker_id_t marker)
{
  switch (marker) {
    case STORAGE_010_BEFORE_QUEUE_RECV:
      return "STORAGE_010_BEFORE_QUEUE_RECV";
    case STORAGE_020_AFTER_QUEUE_RECV:
      return "STORAGE_020_AFTER_QUEUE_RECV";
    case STORAGE_030_ASSIGN_IDS:
      return "STORAGE_030_ASSIGN_IDS";
    case STORAGE_040_ALERT_MANAGER:
      return "STORAGE_040_ALERT_MANAGER";
    case STORAGE_050_FRAM_APPEND:
      return "STORAGE_050_FRAM_APPEND";
    case STORAGE_060_EXPORT_UART_ENQUEUE:
      return "STORAGE_060_EXPORT_UART_ENQUEUE";
    case STORAGE_070_MQTT_ENQUEUE:
      return "STORAGE_070_MQTT_ENQUEUE";
    case SD_010_DETECT_POLL:
      return "SD_010_DETECT_POLL";
    case SD_020_MAINTENANCE:
      return "SD_020_MAINTENANCE";
    case SD_030_SCHEDULER:
      return "SD_030_SCHEDULER";
    case SD_040_FLUSH_WORKER_ENTER:
      return "SD_040_FLUSH_WORKER_ENTER";
    case SD_050_FLUSH_WORKER_EXIT:
      return "SD_050_FLUSH_WORKER_EXIT";
    default:
      return "NONE";
  }
}

const char*
RuntimeMarkersResetReasonToString(esp_reset_reason_t reason)
{
  switch (reason) {
    case ESP_RST_POWERON:
      return "poweron";
    case ESP_RST_EXT:
      return "external";
    case ESP_RST_SW:
      return "software";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
      return "int_wdt";
    case ESP_RST_TASK_WDT:
      return "task_wdt";
    case ESP_RST_WDT:
      return "other_wdt";
    case ESP_RST_DEEPSLEEP:
      return "deepsleep";
    case ESP_RST_BROWNOUT:
      return "brownout";
    case ESP_RST_SDIO:
      return "sdio";
#if defined(ESP_RST_USB)
    case ESP_RST_USB:
      return "usb";
#endif
#if !defined(ESP_RST_USB)
    case (esp_reset_reason_t)11:
      return "usb";
#endif
#if defined(ESP_RST_JTAG)
    case ESP_RST_JTAG:
      return "jtag";
#endif
#if defined(ESP_RST_EFUSE)
    case ESP_RST_EFUSE:
      return "efuse";
#endif
#if defined(ESP_RST_PWR_GLITCH)
    case ESP_RST_PWR_GLITCH:
      return "pwr_glitch";
#endif
#if defined(ESP_RST_CPU_LOCKUP)
    case ESP_RST_CPU_LOCKUP:
      return "cpu_lockup";
#endif
    default:
      return "unknown";
  }
}

static void
RuntimeMarkersUpdateMetadata(void)
{
  g_last_marker_uptime_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
  g_last_marker_epoch_sec =
    TimeSyncIsSystemTimeValid() ? (uint32_t)time(NULL) : 0u;
  g_last_marker_reset_reason = (uint32_t)esp_reset_reason();
}

void
RuntimeMarkersSetStorage(runtime_state_t* state, runtime_marker_id_t marker)
{
  if (state == NULL) {
    return;
  }
  state->storage_marker = (uint32_t)marker;
  g_last_storage_marker = (uint32_t)marker;
  RuntimeMarkersUpdateMetadata();
}

void
RuntimeMarkersSetSdFlush(runtime_state_t* state, runtime_marker_id_t marker)
{
  if (state == NULL) {
    return;
  }
  state->sd_flush_marker = (uint32_t)marker;
  g_last_sd_flush_marker = (uint32_t)marker;
  RuntimeMarkersUpdateMetadata();
}

void
RuntimeMarkersDumpOnBoot(void)
{
  const runtime_marker_id_t storage_marker =
    (runtime_marker_id_t)g_last_storage_marker;
  const runtime_marker_id_t sd_marker =
    (runtime_marker_id_t)g_last_sd_flush_marker;
  const esp_reset_reason_t reset_reason =
    (esp_reset_reason_t)g_last_marker_reset_reason;
  ESP_LOGI(kTag,
           "Last markers: storage=%s(%" PRIu32 ") sd=%s(%" PRIu32 ")",
           RuntimeMarkerToString(storage_marker),
           g_last_storage_marker,
           RuntimeMarkerToString(sd_marker),
           g_last_sd_flush_marker);
  ESP_LOGI(kTag,
           "Last marker meta: uptime_ms=%" PRIu32 " epoch_sec=%" PRIu32
           " reset_reason=%s(%" PRIu32 ")",
           g_last_marker_uptime_ms,
           g_last_marker_epoch_sec,
           RuntimeMarkersResetReasonToString(reset_reason),
           g_last_marker_reset_reason);
}

void
RuntimeMarkersDump(const runtime_state_t* state)
{
  const runtime_marker_id_t storage_marker =
    (runtime_marker_id_t)g_last_storage_marker;
  const runtime_marker_id_t sd_marker =
    (runtime_marker_id_t)g_last_sd_flush_marker;
  const esp_reset_reason_t reset_reason =
    (esp_reset_reason_t)g_last_marker_reset_reason;
  printf("Persistent markers:\n");
  printf("  storage=%s (%" PRIu32 ")\n",
         RuntimeMarkerToString(storage_marker),
         g_last_storage_marker);
  printf("  sd=%s (%" PRIu32 ")\n",
         RuntimeMarkerToString(sd_marker),
         g_last_sd_flush_marker);
  printf("  uptime_ms=%" PRIu32 " epoch_sec=%" PRIu32
         " reset_reason=%s (%" PRIu32 ")\n",
         g_last_marker_uptime_ms,
         g_last_marker_epoch_sec,
         RuntimeMarkersResetReasonToString(reset_reason),
         g_last_marker_reset_reason);

  if (state == NULL) {
    printf("Live markers: unavailable (runtime not initialized)\n");
    return;
  }
  printf("Live markers:\n");
  printf("  storage=%s (%" PRIu32 ")\n",
         RuntimeMarkerToString((runtime_marker_id_t)state->storage_marker),
         state->storage_marker);
  printf("  sd=%s (%" PRIu32 ")\n",
         RuntimeMarkerToString((runtime_marker_id_t)state->sd_flush_marker),
         state->sd_flush_marker);
}

void
RuntimeMarkersClear(void)
{
  g_last_storage_marker = 0;
  g_last_sd_flush_marker = 0;
  g_last_marker_uptime_ms = 0;
  g_last_marker_epoch_sec = 0;
  g_last_marker_reset_reason = 0;

  runtime_state_t* state = RuntimeGetState();
  if (state == NULL) {
    return;
  }
  state->storage_marker = 0;
  state->sd_flush_marker = 0;
}
