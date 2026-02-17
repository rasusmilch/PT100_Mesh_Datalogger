#ifndef CONSOLE_REGISTRY_H
#define CONSOLE_REGISTRY_H

#include "esp_err.h"

typedef void (*console_help_fn_t)(void);

typedef struct console_registry_entry_s
{
  const char* command;
  const char* summary;
  const char* synopsis;
  const char* description;
  console_help_fn_t print_body;
  int (*topic_help)(const char* topic);
  int (*func)(int argc, char** argv);
  void** argtable;
} console_registry_entry_t;

esp_err_t ConsoleRegistryRegister(const console_registry_entry_t* entry);

#endif
