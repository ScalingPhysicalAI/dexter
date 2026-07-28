#ifndef CYCLE_ENGINE_H
#define CYCLE_ENGINE_H

#include "command_io.h"
#include <stdbool.h>
#include <stdint.h>

#define CE_MACRO_COUNT 8U

typedef enum {
    CE_IDLE = 0,
    CE_RUNNING,
    CE_WAIT_MOTION,
    CE_WAIT_DWELL,
    CE_DONE
} CycleEngineState;

void CycleEngine_Init(void);
void CycleEngine_Poll(void);
bool CycleEngine_Run(const char *name, CommandSource source);
void CycleEngine_Stop(void);
void CycleEngine_List(CommandSource source);
void CycleEngine_PrintMacro(const char *name, CommandSource source);
bool CycleEngine_IsBusy(void);
const char *CycleEngine_GetCurrentName(void);
uint8_t CycleEngine_GetStep(void);
uint8_t CycleEngine_GetTotalSteps(void);

#endif
