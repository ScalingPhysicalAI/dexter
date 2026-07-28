#ifndef GCODE_H
#define GCODE_H

#include "command_io.h"
#include <stdbool.h>

void GCode_Init(void);
void GCode_PrintBootHelp(void);
void GCode_PutCharFrom(CommandSource source, char character);
void GCode_Poll(void);
bool GCode_ExecuteScriptLine(CommandSource source, const char *line);
void GCode_Send(const char *text);
void GCode_SendTo(CommandSource source, const char *text);
CommandSource GCode_GetResponseSource(void);

#endif
