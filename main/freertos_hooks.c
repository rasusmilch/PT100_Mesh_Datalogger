#include <inttypes.h>
#include <stdlib.h>

#include "esp_rom_sys.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "runtime_manager.h"
#include "runtime_state.h"

static void
PrintHookHeader(const char* hook, TaskHandle_t task, const char* task_name);

static void
PrintRuntimeSnapshot(const runtime_state_t* state);

static void
PrintHookHeader(const char* hook, TaskHandle_t task, const char* task_name)
{
  const char* name = (task_name != NULL) ? task_name : "unknown";
  uint32_t hwm_bytes = 0;
  if (task != NULL) {
    hwm_bytes = (uint32_t)uxTaskGetStackHighWaterMark(task);
  }
  esp_rom_printf("\nRTOS hook=%s task=%s handle=%p hwm=%" PRIu32 " bytes\n",
                 (hook != NULL) ? hook : "unknown",
                 name,
                 task,
                 hwm_bytes);
}

static void
PrintRuntimeSnapshot(const runtime_state_t* state)
{
  if (state == NULL) {
    esp_rom_printf("runtime: unavailable\n");
    return;
  }

  const runtime_cached_status_t* cached = &state->cached_status;
  const uint32_t uptime_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
  const esp_reset_reason_t reset_reason = esp_reset_reason();

  esp_rom_printf("markers: storage=%" PRIu32 " sd=%" PRIu32
                 " uptime_ms=%" PRIu32 " reset_reason=%u\n",
                 state->storage_marker,
                 state->sd_flush_marker,
                 uptime_ms,
                 (unsigned)reset_reason);
  esp_rom_printf("fram: count=%" PRIu32 "/%" PRIu32 " wm=%" PRIu32 "\n",
                 cached->fram_count,
                 cached->fram_capacity,
                 cached->fram_flush_watermark_records);
  esp_rom_printf("heap: int_free=%" PRIu32 " int_largest=%" PRIu32
                 " psram_free=%" PRIu32 " psram_largest=%" PRIu32 "\n",
                 cached->heap_internal_free_bytes,
                 cached->heap_internal_largest_free_block_bytes,
                 cached->heap_psram_free_bytes,
                 cached->heap_psram_largest_free_block_bytes);
}

void
vApplicationStackOverflowHook(TaskHandle_t task, char* task_name)
{
  PrintHookHeader("stack_overflow", task, task_name);
  PrintRuntimeSnapshot(RuntimeGetState());
  abort();
}

void
vApplicationMallocFailedHook(void)
{
  PrintHookHeader("malloc_failed", xTaskGetCurrentTaskHandle(), NULL);
  PrintRuntimeSnapshot(RuntimeGetState());
  abort();
}
