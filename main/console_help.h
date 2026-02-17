#ifndef CONSOLE_HELP_H
#define CONSOLE_HELP_H

#include "esp_err.h"

#include "console_registry.h"

#define CONSOLE_HELP_MAX_COMMANDS 64

typedef struct
{
  const char* name;
  const char* summary;
  const char* synopsis;
  const char* details;
  const char* options;
  const char* examples;
} console_help_topic_t;

void ConsoleHelpInit(void);
esp_err_t ConsoleHelpRegisterEntry(const console_registry_entry_t* entry);
void ConsoleHelpPrintIndex(void);
void ConsoleHelpPrintManpage(const char* command);
void ConsoleHelpPrintTopicList(const console_help_topic_t* topics);
void ConsoleHelpPrintTopicManpage(const char* command,
                                  const console_help_topic_t* topic);
int ConsoleHelpCommand(int argc, char** argv);

#endif
