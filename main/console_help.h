#ifndef CONSOLE_HELP_H
#define CONSOLE_HELP_H

#include "esp_err.h"

#include "console_registry.h"

#define CONSOLE_HELP_MAX_COMMANDS 64

void ConsoleHelpInit(void);
esp_err_t ConsoleHelpRegisterEntry(const console_registry_entry_t* entry);
void ConsoleHelpPrintIndex(void);
void ConsoleHelpPrintManpage(const char* command);
int ConsoleHelpCommand(int argc, char** argv);

#endif
