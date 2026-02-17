#include "console_help.h"

#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"

typedef struct
{
  const console_registry_entry_t* entries[CONSOLE_HELP_MAX_COMMANDS];
  size_t count;
} console_help_registry_t;

static console_help_registry_t g_help_registry = { 0 };

static const console_registry_entry_t* FindEntry(const char* command);
static size_t CollectPrefixMatches(const char* prefix,
                                   const console_registry_entry_t** matches,
                                   size_t max_matches);

void
ConsoleHelpInit(void)
{
  memset(&g_help_registry, 0, sizeof(g_help_registry));
}

esp_err_t
ConsoleHelpRegisterEntry(const console_registry_entry_t* entry)
{
  if (entry == NULL || entry->command == NULL || entry->summary == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (FindEntry(entry->command) != NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (g_help_registry.count >= CONSOLE_HELP_MAX_COMMANDS) {
    return ESP_ERR_NO_MEM;
  }

  g_help_registry.entries[g_help_registry.count++] = entry;
  return ESP_OK;
}

void
ConsoleHelpPrintIndex(void)
{
  const console_registry_entry_t* sorted[CONSOLE_HELP_MAX_COMMANDS] = { 0 };
  const size_t count = g_help_registry.count;
  size_t max_cmd_width = 0;

  for (size_t i = 0; i < count; ++i) {
    sorted[i] = g_help_registry.entries[i];
    const size_t cmd_len = strlen(sorted[i]->command);
    if (cmd_len > max_cmd_width) {
      max_cmd_width = cmd_len;
    }
  }

  for (size_t i = 1; i < count; ++i) {
    const console_registry_entry_t* key = sorted[i];
    size_t j = i;
    while (j > 0 && strcmp(sorted[j - 1]->command, key->command) > 0) {
      sorted[j] = sorted[j - 1];
      --j;
    }
    sorted[j] = key;
  }

  printf("Commands:\n");
  for (size_t i = 0; i < count; ++i) {
    printf("  %-*s  %s\n",
           (int)max_cmd_width,
           sorted[i]->command,
           sorted[i]->summary);
  }
  printf("\nTry: help <command>\n");
}

void
ConsoleHelpPrintManpage(const char* command)
{
  const console_registry_entry_t* entry = FindEntry(command);
  if (entry == NULL) {
    const console_registry_entry_t* matches[8] = { 0 };
    const size_t match_count = CollectPrefixMatches(command, matches, 8);
    printf("Unknown command: %s\n", command);
    printf("Try: help\n");
    if (match_count > 0) {
      printf("Did you mean:\n");
      for (size_t i = 0; i < match_count; ++i) {
        printf("  %s\n", matches[i]->command);
      }
    }
    return;
  }

  printf("NAME\n");
  printf("  %s - %s\n\n", entry->command, entry->summary);

  printf("SYNOPSIS\n");
  if (entry->synopsis != NULL && entry->synopsis[0] != '\0') {
    printf("  %s\n\n", entry->synopsis);
  } else {
    printf("  %s ...\n\n", entry->command);
  }

  printf("DESCRIPTION\n");
  printf("  %s\n\n",
         (entry->description != NULL && entry->description[0] != '\0')
           ? entry->description
           : entry->summary);

  if (entry->argtable != NULL) {
    printf("OPTIONS\n");
    arg_print_syntax(stdout, entry->argtable, "\n");
    arg_print_glossary(stdout, entry->argtable, "  %-20s %s\n");
    printf("\n");
  }

  if (entry->print_body != NULL) {
    entry->print_body();
  }
}

int
ConsoleHelpCommand(int argc, char** argv)
{
  if (argc == 1) {
    ConsoleHelpPrintIndex();
    return 0;
  }

  if (argc == 2) {
    ConsoleHelpPrintManpage(argv[1]);
    return 0;
  }

  printf("usage: help [command]\n");
  return 1;
}

static const console_registry_entry_t*
FindEntry(const char* command)
{
  if (command == NULL) {
    return NULL;
  }

  for (size_t i = 0; i < g_help_registry.count; ++i) {
    const console_registry_entry_t* entry = g_help_registry.entries[i];
    if (strcmp(entry->command, command) == 0) {
      return entry;
    }
  }
  return NULL;
}

static size_t
CollectPrefixMatches(const char* prefix,
                     const console_registry_entry_t** matches,
                     size_t max_matches)
{
  if (prefix == NULL || matches == NULL || max_matches == 0 || prefix[0] == '\0') {
    return 0;
  }

  const size_t prefix_len = strlen(prefix);
  size_t count = 0;
  for (size_t i = 0; i < g_help_registry.count && count < max_matches; ++i) {
    const console_registry_entry_t* entry = g_help_registry.entries[i];
    if (strncmp(entry->command, prefix, prefix_len) == 0) {
      matches[count++] = entry;
    }
  }

  for (size_t i = 1; i < count; ++i) {
    const console_registry_entry_t* key = matches[i];
    size_t j = i;
    while (j > 0 && strcmp(matches[j - 1]->command, key->command) > 0) {
      matches[j] = matches[j - 1];
      --j;
    }
    matches[j] = key;
  }

  return count;
}
