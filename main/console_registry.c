#include "console_registry.h"

#include "esp_console.h"

#include "console_help.h"

esp_err_t
ConsoleRegistryRegister(const console_registry_entry_t* entry)
{
  if (entry == NULL || entry->command == NULL || entry->summary == NULL ||
      entry->func == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const esp_console_cmd_t command = {
    .command = entry->command,
    .help = entry->summary,
    .hint = NULL,
    .func = entry->func,
    .argtable = entry->argtable,
  };

  esp_err_t result = esp_console_cmd_register(&command);
  if (result != ESP_OK) {
    return result;
  }

  return ConsoleHelpRegisterEntry(entry);
}
