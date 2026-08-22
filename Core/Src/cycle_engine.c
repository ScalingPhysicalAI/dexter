#include "cycle_engine.h"
#include "gcode.h"
#include "linear_actuator.h"
#include "stepper.h"
#include <stddef.h>

static const char * const macro_init[] = {
    "$14=1", "$0=800", "$1=800", "$6=400", "$2=5000", "$3=5000", "$7=2000",
    "G90", "G92 X0 Y0 Z0", NULL
};
static const char * const macro_kitchen[] = {
    "G0 Z0", "G4 P300", "G0 X0 Y0", "G0 X1500 Y0", "G0 X1500 Y-800",
    "G0 X2200 Y-800", "G4 P500", "G0 Z50", NULL
};
static const char * const macro_living[] = {
    "G0 Z0", "G4 P300", "G0 X0 Y0", "G0 X800 Y500", "G0 X800 Y1200",
    "G4 P500", "G0 Z30", NULL
};
static const char * const macro_bedroom[] = {
    "G0 Z0", "G4 P300", "G0 X0 Y0", "G0 X-200 Y0", "G0 X-200 Y-1500",
    "G0 X400 Y-1500", "G4 P500", "G0 Z20", NULL
};
static const char * const macro_door[] = {
    "G0 Z0", "G4 P300", "G0 X0 Y0", "G0 X0 Y900", "G0 X-600 Y900", "G4 P500", NULL
};
static const char * const macro_charge[] = {
    "G0 Z0", "G4 P300", "G0 X0 Y0", "G4 P200", "G92 X0 Y0 Z0", NULL
};
static const char * const macro_patrol[] = {
    "G0 Z0", "G0 X0 Y0", "G0 X800 Y0", "G4 P2000", "G0 X800 Y800", "G4 P2000",
    "G0 X0 Y800", "G4 P2000", "G0 X0 Y0", NULL
};
static const char * const macro_height_cal[] = {
    "G0 Z0", "G4 P500", "G0 Z100", "G4 P500", "G0 Z50", "G4 P500", "G0 Z0", NULL
};

static const char * const macro_names[CE_MACRO_COUNT] = {
    "INIT", "KITCHEN", "LIVING_ROOM", "BEDROOM", "DOOR", "CHARGE", "PATROL", "HEIGHT_CAL"
};
static const char * const * const macros[CE_MACRO_COUNT] = {
    macro_init, macro_kitchen, macro_living, macro_bedroom,
    macro_door, macro_charge, macro_patrol, macro_height_cal
};

static CycleEngineState s_state;
static uint8_t s_macro_index;
static uint8_t s_step;
static uint8_t s_total;
static uint32_t s_dwell_end;
static CommandSource s_source;

static bool equal_ignore_case(const char *left, const char *right)
{
    while (*left != '\0' && *right != '\0') {
        char a = (*left >= 'a' && *left <= 'z') ? (char)(*left - 32) : *left;
        char b = (*right >= 'a' && *right <= 'z') ? (char)(*right - 32) : *right;
        if (a != b) return false;
        ++left;
        ++right;
    }
    return *left == '\0' && *right == '\0';
}

static uint8_t macro_length(uint8_t index)
{
    uint8_t length = 0U;
    while (macros[index][length] != NULL) ++length;
    return length;
}

static void send_to(CommandSource source, const char *text) { GCode_SendTo(source, text); }

static void send_progress(void)
{
    char buffer[64];
    char *out = buffer;
    const char *prefix = "[CE:";
    while (*prefix) *out++ = *prefix++;
    const char *name = macro_names[s_macro_index];
    while (*name) *out++ = *name++;
    *out++ = ' ';
    uint8_t shown_step = (uint8_t)(s_step + 1U);
    if (shown_step >= 10U) *out++ = (char)('0' + shown_step / 10U);
    *out++ = (char)('0' + shown_step % 10U);
    *out++ = '/';
    if (s_total >= 10U) *out++ = (char)('0' + s_total / 10U);
    *out++ = (char)('0' + s_total % 10U);
    *out++ = ']'; *out++ = '\r'; *out++ = '\n'; *out = '\0';
    send_to(s_source, buffer);
}

static bool line_is_dwell(const char *line, uint32_t *duration)
{
    if (!((line[0] == 'G' || line[0] == 'g') && line[1] == '4' &&
          (line[2] == ' ' || line[2] == '\0'))) return false;
    const char *cursor = line;
    while (*cursor && *cursor != 'P' && *cursor != 'p') ++cursor;
    uint32_t value = 0U;
    if (*cursor) {
        ++cursor;
        while (*cursor >= '0' && *cursor <= '9') value = value * 10U + (uint32_t)(*cursor++ - '0');
    }
    *duration = value;
    return true;
}

void CycleEngine_Init(void)
{
    s_state = CE_IDLE;
    s_step = s_total = 0U;
}

bool CycleEngine_Run(const char *name, CommandSource source)
{
    if (CycleEngine_IsBusy()) {
        send_to(source, "[CE:error busy - send STOP first]\r\n");
        return false;
    }
    for (uint8_t index = 0U; index < CE_MACRO_COUNT; ++index) {
        if (!equal_ignore_case(name, macro_names[index])) continue;
        s_macro_index = index;
        s_step = 0U;
        s_total = macro_length(index);
        s_source = source;
        s_state = CE_RUNNING;
        send_to(source, "[CE:starting ");
        send_to(source, macro_names[index]);
        send_to(source, "]\r\n");
        return true;
    }
    send_to(source, "[CE:error unknown macro]\r\n");
    return false;
}

void CycleEngine_Stop(void)
{
    Stepper_StopAll();
    LinearActuator_Stop();
    if (s_state == CE_IDLE) return;
    s_state = CE_IDLE;
    send_to(s_source, "[CE:stopped]\r\n");
}

void CycleEngine_List(CommandSource source)
{
    send_to(source, "[CE:macros]\r\n");
    for (uint8_t index = 0U; index < CE_MACRO_COUNT; ++index) {
        uint8_t length = macro_length(index);
        char count[3];
        uint8_t pos = 0U;
        if (length >= 10U) count[pos++] = (char)('0' + length / 10U);
        count[pos++] = (char)('0' + length % 10U);
        count[pos] = '\0';
        send_to(source, "  ");
        send_to(source, macro_names[index]);
        send_to(source, " (");
        send_to(source, count);
        send_to(source, " lines)\r\n");
    }
}

void CycleEngine_PrintMacro(const char *name, CommandSource source)
{
    for (uint8_t index = 0U; index < CE_MACRO_COUNT; ++index) {
        if (!equal_ignore_case(name, macro_names[index])) continue;
        send_to(source, "[CE:macro ");
        send_to(source, macro_names[index]);
        send_to(source, "]\r\n");
        for (uint8_t line = 0U; macros[index][line] != NULL; ++line) {
            send_to(source, "  ");
            send_to(source, macros[index][line]);
            send_to(source, "\r\n");
        }
        return;
    }
    send_to(source, "[CE:error unknown macro]\r\n");
}

bool CycleEngine_IsBusy(void) { return s_state != CE_IDLE && s_state != CE_DONE; }
const char *CycleEngine_GetCurrentName(void) { return s_state == CE_IDLE ? "" : macro_names[s_macro_index]; }
uint8_t CycleEngine_GetStep(void) { return s_step; }
uint8_t CycleEngine_GetTotalSteps(void) { return s_total; }

void CycleEngine_Poll(void)
{
    switch (s_state) {
        case CE_IDLE:
            return;
        case CE_DONE:
            send_to(s_source, "[CE:done ");
            send_to(s_source, macro_names[s_macro_index]);
            send_to(s_source, "]\r\n");
            s_state = CE_IDLE;
            return;
        case CE_WAIT_MOTION:
            if (!Stepper_IsBusy()) {
                if (Stepper_LimitStopped()) {
                    send_to(s_source, "[CE:error Z limit]\r\n");
                    s_state = CE_IDLE;
                } else {
                    ++s_step;
                    s_state = CE_RUNNING;
                }
            }
            return;
        case CE_WAIT_DWELL:
            if ((int32_t)(HAL_GetTick() - s_dwell_end) >= 0) {
                ++s_step;
                s_state = CE_RUNNING;
            }
            return;
        case CE_RUNNING: {
            if (s_step >= s_total) { s_state = CE_DONE; return; }
            const char *line = macros[s_macro_index][s_step];
            send_progress();
            uint32_t dwell_ms = 0U;
            bool dwell = line_is_dwell(line, &dwell_ms);
            if (!GCode_ExecuteScriptLine(s_source, line)) {
                send_to(s_source, "[CE:error command failed]\r\n");
                s_state = CE_IDLE;
                return;
            }
            if (dwell) {
                s_dwell_end = HAL_GetTick() + dwell_ms;
                s_state = CE_WAIT_DWELL;
            } else if (Stepper_IsBusy()) {
                s_state = CE_WAIT_MOTION;
            } else {
                ++s_step;
            }
            return;
        }
        default:
            s_state = CE_IDLE;
            return;
    }
}
