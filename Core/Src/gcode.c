#include "gcode.h"
#include "can_interface.h"
#include "cycle_engine.h"
#include "limit_switches.h"
#include "linear_actuator.h"
#include "main.h"
#include "stepper.h"
#include <stdint.h>
#include <string.h>

#define GCODE_LINE_MAX 96U
#define DEFAULT_FEED_SPS 500U
#define DEFAULT_RAPID_SPS 1000U
#define DEFAULT_STEPS_PER_REV 3200U
#define DEFAULT_UM_PER_REV 8000U
#define COMMAND_QUEUE_DEPTH 32U

typedef enum { DIST_ABSOLUTE = 0, DIST_RELATIVE } DistanceMode;

typedef struct {
    char line[GCODE_LINE_MAX];
    uint8_t length;
} InputBuffer;

typedef struct {
    char line[GCODE_LINE_MAX];
    CommandSource source;
} QueuedCommand;

static struct {
    InputBuffer input[COMMAND_SOURCE_COUNT];
    DistanceMode distance_mode;
    uint32_t feed_sps;
    uint32_t rapid_sps;
    uint32_t accel[NUM_AXES];
    uint32_t max_sps[NUM_AXES];
    uint32_t steps_per_rev[NUM_AXES];
    uint32_t um_per_rev[NUM_AXES];
    int32_t work_position[NUM_AXES];
    int32_t offset[NUM_AXES];
    uint32_t dwell_end_ms;
    bool dwelling;
    bool paused;
    bool alarm;
    bool unit_mm;
    QueuedCommand queue[COMMAND_QUEUE_DEPTH];
    uint8_t queue_head;
    uint8_t queue_tail;
} gc;

static CommandSource s_response_source = COMMAND_SOURCE_USB;
static volatile bool s_estop_latched;
static volatile bool s_estop_report_pending;

static const char s_command_help[] =
    "\r\n=== Dexter STM32L552ZET6 motion controller ===\r\n"
    "Command ports: USB CDC, LPUART1 IRQ, or CAN (CANID? to query)\r\n"
    "Motion: G0/G1 X.. Y.. Z.. [F..] (XYZ synchronized)\r\n"
    "Modes: G90 absolute, G91 relative, G92 set position, G4 P.. dwell\r\n"
    "Linear: M3 S<ms> extend, M4 S<ms> retract, M5 stop\r\n"
    "Control: ? status, ! hold, ~ resume, ESTOP/M112, CLEAR ALARM\r\n"
#if ESTOP_BUTTON_ENABLE
    "E-stop button: PC2 active-low; ESTOP RESET only after release\r\n"
#else
    "E-stop button: disabled by ESTOP_BUTTON_ENABLE=0\r\n"
#endif
    "Direction: DIR X|Y|Z NORMAL|REVERSE (or $20/$21/$22=0|1)\r\n"
    "Limits: LIMIT ON|OFF, $23=polarity, $24=enable (default OFF)\r\n"
    "CAN 500k: CAN STATUS, CAN TEST; IDs: CANID <RX> <TX>, CANID?\r\n"
    "Cycles: LIST, RUN <name>, MACRO <name>, STOP\r\n"
    "Type HELP to print this guide again.\r\n"
    "ready\r\n";

static char *append_text(char *out, const char *text)
{
    while (*text != '\0') *out++ = *text++;
    return out;
}

static char *append_u32(char *out, uint32_t value)
{
    char reverse[10];
    uint8_t count = 0U;
    if (value == 0U) {
        *out++ = '0';
        return out;
    }
    while (value != 0U) {
        reverse[count++] = (char)('0' + value % 10U);
        value /= 10U;
    }
    while (count != 0U) *out++ = reverse[--count];
    return out;
}

static char *append_i32(char *out, int32_t value)
{
    if (value < 0) {
        *out++ = '-';
        return append_u32(out, (uint32_t)(-(int64_t)value));
    }
    return append_u32(out, (uint32_t)value);
}

static char *append_can_id(char *out, uint16_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    *out++ = '0';
    *out++ = 'x';
    *out++ = hex[(value >> 8U) & 0x0FU];
    *out++ = hex[(value >> 4U) & 0x0FU];
    *out++ = hex[value & 0x0FU];
    return out;
}

static void send_value(const char *prefix, int32_t value, const char *suffix)
{
    char buffer[72];
    char *out = append_text(buffer, prefix);
    out = append_i32(out, value);
    out = append_text(out, suffix);
    *out = '\0';
    GCode_Send(buffer);
}

void GCode_SendTo(CommandSource source, const char *text)
{
    CommandIO_Send(source, text, (uint16_t)strlen(text));
}

void GCode_Send(const char *text) { GCode_SendTo(s_response_source, text); }
CommandSource GCode_GetResponseSource(void) { return s_response_source; }
static void send_ok(void) { GCode_Send("ok\r\n"); }

void GCode_PrintBootHelp(void)
{
    GCode_SendTo(COMMAND_SOURCE_UART, s_command_help);
    GCode_SendTo(COMMAND_SOURCE_USB, s_command_help);
}

static void send_error(const char *message)
{
    GCode_Send("error:");
    GCode_Send(message);
    GCode_Send("\r\n");
}

static bool estop_button_active(void)
{
#if ESTOP_BUTTON_ENABLE
    bool pin_high = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET;
    return ESTOP_BUTTON_ACTIVE_LOW ? !pin_high : pin_high;
#else
    return false;
#endif
}

static bool take_estop_report_pending(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool pending = s_estop_report_pending;
    s_estop_report_pending = false;
    if (primask == 0U) __enable_irq();
    return pending;
}

static void enter_estop_alarm(void)
{
    LinearActuator_Stop();
    CycleEngine_Stop();
    gc.dwelling = false;
    gc.queue_head = gc.queue_tail = 0U;
    gc.alarm = true;
    GCode_SendTo(COMMAND_SOURCE_USB, "ALARM:E-STOP\r\n");
    GCode_SendTo(COMMAND_SOURCE_UART, "ALARM:E-STOP\r\n");
    GCode_SendTo(COMMAND_SOURCE_CAN, "ALARM:E-STOP\r\n");
}

static void trigger_estop_command(void)
{
    Stepper_StopAll();
    LinearActuator_Stop();
    s_estop_latched = true;
    s_estop_report_pending = true;
    (void)take_estop_report_pending();
    enter_estop_alarm();
}

void GCode_EStopFromISR(void)
{
#if ESTOP_BUTTON_ENABLE
    if (!estop_button_active()) return;
    Stepper_EmergencyStopFromISR();
    LinearActuator_EmergencyStopFromISR();
    s_estop_latched = true;
    s_estop_report_pending = true;
#endif
}

static bool clear_estop_latch(void)
{
    if (estop_button_active()) return false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_estop_latched = false;
    s_estop_report_pending = false;
    if (primask == 0U) __enable_irq();
    return true;
}

static void unlock_alarms(void)
{
    if (s_estop_latched && !clear_estop_latch()) {
        send_error("E-stop button active");
        return;
    }
    gc.alarm = false;
    Stepper_ClearLimitStopped();
    send_ok();
}

static void uppercase(char *text)
{
    while (*text != '\0') {
        if (*text >= 'a' && *text <= 'z') *text = (char)(*text - ('a' - 'A'));
        ++text;
    }
}

static void strip_comments(char *line)
{
    char *read = line;
    char *write = line;
    bool parenthesis = false;
    while (*read != '\0') {
        if (*read == ';' && !parenthesis) break;
        if (*read == '(') parenthesis = true;
        else if (*read == ')') parenthesis = false;
        else if (!parenthesis) *write++ = *read;
        ++read;
    }
    *write = '\0';
}

static int32_t parse_word_tenths(const char *line, char word, bool *found)
{
    *found = false;
    for (const char *p = line; *p != '\0'; ++p) {
        if (*p != word) continue;
        *found = true;
        ++p;
        int32_t sign = 1;
        if (*p == '-') { sign = -1; ++p; }
        else if (*p == '+') ++p;
        int32_t integer = 0;
        bool digit = false;
        while (*p >= '0' && *p <= '9') {
            digit = true;
            integer = integer * 10 + (*p++ - '0');
        }
        int32_t fraction = 0;
        if (*p == '.' && p[1] >= '0' && p[1] <= '9') fraction = p[1] - '0';
        if (!digit) *found = false;
        return sign * (integer * 10 + fraction);
    }
    return 0;
}

static int32_t tenths_to_integer(int32_t value)
{
    return value >= 0 ? (value + 5) / 10 : -((-value + 5) / 10);
}

static int32_t tenths_mm_to_steps(int32_t value, uint8_t axis)
{
    if (gc.um_per_rev[axis] == 0U) return 0;
    int32_t sign = value < 0 ? -1 : 1;
    uint64_t magnitude = (uint64_t)(value < 0 ? -(int64_t)value : value);
    uint64_t steps = magnitude * gc.steps_per_rev[axis] * 100ULL / gc.um_per_rev[axis];
    return sign * (int32_t)steps;
}

static int32_t steps_to_tenths_mm(int32_t steps, uint8_t axis)
{
    if (gc.steps_per_rev[axis] == 0U) return 0;
    int32_t sign = steps < 0 ? -1 : 1;
    uint64_t magnitude = (uint64_t)(steps < 0 ? -(int64_t)steps : steps);
    uint64_t value = magnitude * gc.um_per_rev[axis] /
                     ((uint64_t)gc.steps_per_rev[axis] * 100ULL);
    return sign * (int32_t)value;
}

static uint32_t feed_mm_per_min_to_sps(uint32_t feed, uint8_t axis)
{
    if (gc.um_per_rev[axis] == 0U || gc.steps_per_rev[axis] == 0U) return gc.feed_sps;
    uint64_t numerator = (uint64_t)feed * gc.steps_per_rev[axis] * 1000ULL;
    uint32_t speed = (uint32_t)(numerator / ((uint64_t)gc.um_per_rev[axis] * 60ULL));
    if (speed < STEPPER_MIN_SPEED_SPS) speed = STEPPER_MIN_SPEED_SPS;
    if (speed > STEPPER_MAX_SPEED_SPS) speed = STEPPER_MAX_SPEED_SPS;
    return speed;
}

static char *append_position(char *out, int32_t steps, uint8_t axis)
{
    if (!gc.unit_mm) return append_i32(out, steps);
    int32_t tenths = steps_to_tenths_mm(steps, axis);
    if (tenths < 0) {
        *out++ = '-';
        tenths = -tenths;
    }
    out = append_u32(out, (uint32_t)(tenths / 10));
    *out++ = '.';
    *out++ = (char)('0' + tenths % 10);
    return out;
}

static void send_status(void)
{
    char buffer[224];
    char *out = buffer;
    const char *state = gc.alarm ? "Alarm" : gc.paused ? "Hold" :
                        (Stepper_IsBusy() || LinearActuator_IsBusy()) ? "Run" : "Idle";
    out = append_text(out, "<");
    out = append_text(out, state);
    out = append_text(out, "|MPos:");
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        out = append_position(out, Stepper_GetPos(axis), axis);
        if (axis != AXIS_Z) *out++ = ',';
    }
    out = append_text(out, "|WPos:");
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        out = append_position(out, Stepper_GetPos(axis) + gc.offset[axis], axis);
        if (axis != AXIS_Z) *out++ = ',';
    }
    out = append_text(out, "|F:");
    out = append_u32(out, gc.feed_sps);
    out = append_text(out, "|LimZ:");
    *out++ = Limit_ZMinActive() ? '1' : '0';
    *out++ = ',';
    *out++ = Limit_ZMaxActive() ? '1' : '0';
    out = append_text(out, "|LimEn:");
    *out++ = Limit_GetEnabled() ? '1' : '0';
    if (Stepper_LimitStopped()) out = append_text(out, "|Limit:Z");
    out = append_text(out, "|EStop:");
    *out++ = s_estop_latched ? '1' : '0';
    out = append_text(out, "|Act:");
    LinearActuatorDirection actuator_direction = LinearActuator_GetDirection();
    if (actuator_direction == LINEAR_ACTUATOR_EXTEND) out = append_text(out, "EXT");
    else if (actuator_direction == LINEAR_ACTUATOR_RETRACT) out = append_text(out, "RET");
    else out = append_text(out, "STOP");
    const char *macro = CycleEngine_GetCurrentName();
    if (*macro != '\0') {
        out = append_text(out, "|CE:");
        out = append_text(out, macro);
        *out++ = ' ';
        out = append_u32(out, CycleEngine_GetStep() + 1U);
        *out++ = '/';
        out = append_u32(out, CycleEngine_GetTotalSteps());
    }
    out = append_text(out, ">\r\n");
    *out = '\0';
    GCode_Send(buffer);
}

static void print_settings(void)
{
    send_value("$0=", (int32_t)gc.max_sps[AXIS_X], " ;X max sps\r\n");
    send_value("$1=", (int32_t)gc.max_sps[AXIS_Y], " ;Y max sps\r\n");
    send_value("$2=", (int32_t)gc.accel[AXIS_X], " ;X accel sps2\r\n");
    send_value("$3=", (int32_t)gc.accel[AXIS_Y], " ;Y accel sps2\r\n");
    send_value("$4=", (int32_t)gc.feed_sps, " ;feed sps\r\n");
    send_value("$5=", (int32_t)gc.rapid_sps, " ;rapid sps\r\n");
    send_value("$6=", (int32_t)gc.max_sps[AXIS_Z], " ;Z max sps\r\n");
    send_value("$7=", (int32_t)gc.accel[AXIS_Z], " ;Z accel sps2\r\n");
    send_value("$10=", (int32_t)gc.steps_per_rev[AXIS_X], " ;X pulses/rev\r\n");
    send_value("$11=", (int32_t)gc.um_per_rev[AXIS_X], " ;X um/rev\r\n");
    send_value("$12=", (int32_t)gc.steps_per_rev[AXIS_Y], " ;Y pulses/rev\r\n");
    send_value("$13=", (int32_t)gc.um_per_rev[AXIS_Y], " ;Y um/rev\r\n");
    send_value("$14=", gc.unit_mm ? 1 : 0, " ;units 0=steps 1=mm\r\n");
    send_value("$15=", (int32_t)gc.steps_per_rev[AXIS_Z], " ;Z pulses/rev\r\n");
    send_value("$16=", (int32_t)gc.um_per_rev[AXIS_Z], " ;Z um/rev\r\n");
    send_value("$20=", Stepper_GetDirectionInverted(AXIS_X) ? 1 : 0, " ;X direction invert\r\n");
    send_value("$21=", Stepper_GetDirectionInverted(AXIS_Y) ? 1 : 0, " ;Y direction invert\r\n");
    send_value("$22=", Stepper_GetDirectionInverted(AXIS_Z) ? 1 : 0, " ;Z direction invert\r\n");
    send_value("$23=", Limit_GetActiveHigh() ? 1 : 0, " ;limit active-high\r\n");
    send_value("$24=", Limit_GetEnabled() ? 1 : 0, " ;Z limits enabled\r\n");
    send_ok();
}

static bool parse_unsigned(const char *text, uint32_t *value)
{
    if (*text < '0' || *text > '9') return false;
    uint32_t result = 0U;
    while (*text >= '0' && *text <= '9') result = result * 10U + (uint32_t)(*text++ - '0');
    while (*text == ' ' || *text == '\t') ++text;
    if (*text != '\0') return false;
    *value = result;
    return true;
}

static bool parse_can_id_token(const char **text, uint16_t *value)
{
    const char *cursor = *text;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    uint32_t base = 10U;
    if (cursor[0] == '0' && cursor[1] == 'X') {
        base = 16U;
        cursor += 2;
    }

    uint32_t parsed = 0U;
    bool found = false;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') {
        uint8_t digit;
        if (*cursor >= '0' && *cursor <= '9') digit = (uint8_t)(*cursor - '0');
        else if (base == 16U && *cursor >= 'A' && *cursor <= 'F') {
            digit = (uint8_t)(*cursor - 'A' + 10);
        } else {
            return false;
        }
        if (digit >= base) return false;
        parsed = parsed * base + digit;
        if (parsed > 0x7FFU) return false;
        found = true;
        ++cursor;
    }
    if (!found) return false;
    *value = (uint16_t)parsed;
    *text = cursor;
    return true;
}

static void send_can_ids(void)
{
    uint16_t receive_id;
    uint16_t transmit_id;
    char buffer[40];
    char *out = buffer;
    CommandIO_GetCanIds(&receive_id, &transmit_id);
    out = append_text(out, "CANID RX=");
    out = append_can_id(out, receive_id);
    out = append_text(out, " TX=");
    out = append_can_id(out, transmit_id);
    out = append_text(out, "\r\n");
    *out = '\0';
    GCode_Send(buffer);
}

static void parse_can_id_command(const char *line)
{
    const char *cursor = line + 5;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0' || strcmp(cursor, "?") == 0) {
        send_can_ids();
        send_ok();
        return;
    }

    uint16_t receive_id;
    uint16_t transmit_id;
    if (strcmp(cursor, "DEFAULT") == 0) {
        receive_id = CAN_COMMAND_RX_ID;
        transmit_id = CAN_COMMAND_TX_ID;
    } else {
        if (!parse_can_id_token(&cursor, &receive_id) ||
            !parse_can_id_token(&cursor, &transmit_id)) {
            send_error("CANID needs RX TX (0x000..0x7FF)");
            return;
        }
        while (*cursor == ' ' || *cursor == '\t') ++cursor;
        if (*cursor != '\0') {
            send_error("CANID needs exactly two IDs");
            return;
        }
    }
    if (receive_id == transmit_id) {
        send_error("CAN RX and TX IDs must differ");
        return;
    }

    /* Queue the reply on the old TX ID, then switch after the CAN queue drains. */
    char buffer[40];
    char *out = buffer;
    out = append_text(out, "CANID RX=");
    out = append_can_id(out, receive_id);
    out = append_text(out, " TX=");
    out = append_can_id(out, transmit_id);
    out = append_text(out, "\r\n");
    *out = '\0';
    GCode_Send(buffer);
    send_ok();
    (void)CommandIO_SetCanIds(receive_id, transmit_id);
}

static void send_can_status(void)
{
    CAN_InterfaceStatus status;
    char buffer[160];
    char *out = buffer;

    if (!CAN_GetStatus(&status)) {
        send_error("CAN status unavailable");
        return;
    }

    out = append_text(out, "CAN CLOCK=");
    out = append_u32(out, status.kernel_clock_hz);
    out = append_text(out, " BITRATE=");
    out = append_u32(out, status.nominal_bitrate);
    out = append_text(out, " TEC=");
    out = append_u32(out, status.tx_error_count);
    out = append_text(out, " REC=");
    out = append_u32(out, status.rx_error_count);
    out = append_text(out, " LEC=");
    out = append_u32(out, status.last_error_code);
    out = append_text(out, " PASSIVE=");
    out = append_u32(out, status.error_passive ? 1U : 0U);
    out = append_text(out, " WARNING=");
    out = append_u32(out, status.warning ? 1U : 0U);
    out = append_text(out, " BUSOFF=");
    out = append_u32(out, status.bus_off ? 1U : 0U);
    out = append_text(out, "\r\n");
    *out = '\0';
    GCode_Send(buffer);
    send_ok();
}

static void run_can_test(void)
{
    if (Stepper_IsBusy() || CycleEngine_IsBusy()) {
        send_error("stop motion before CAN TEST");
        return;
    }
    if (CAN_RunInternalLoopbackTest()) {
        GCode_Send("CAN LOOPBACK=PASS; NORMAL MODE RESTORED\r\n");
        send_ok();
    } else {
        send_error("CAN loopback failed or normal-mode restore failed");
    }
}

static void parse_setting(const char *line)
{
    if (line[1] == '\0') { print_settings(); return; }
    if (strcmp(line, "$X") == 0) {
        unlock_alarms();
        return;
    }
    if (strcmp(line, "$H") == 0) {
        int32_t target[NUM_AXES] = {0, 0, 0};
        if (!Stepper_MoveCoordinated(target, gc.rapid_sps)) send_error("busy");
        else send_ok();
        return;
    }
    if (strcmp(line, "$20") == 0) { send_value("$20=", Stepper_GetDirectionInverted(AXIS_X) ? 1 : 0, " ;X direction invert\r\n"); send_ok(); return; }
    if (strcmp(line, "$21") == 0) { send_value("$21=", Stepper_GetDirectionInverted(AXIS_Y) ? 1 : 0, " ;Y direction invert\r\n"); send_ok(); return; }
    if (strcmp(line, "$22") == 0) { send_value("$22=", Stepper_GetDirectionInverted(AXIS_Z) ? 1 : 0, " ;Z direction invert\r\n"); send_ok(); return; }
    if (strcmp(line, "$23") == 0) { send_value("$23=", Limit_GetActiveHigh() ? 1 : 0, " ;limit active-high\r\n"); send_ok(); return; }
    if (strcmp(line, "$24") == 0) { send_value("$24=", Limit_GetEnabled() ? 1 : 0, " ;Z limits enabled\r\n"); send_ok(); return; }

    const char *cursor = line + 1;
    uint32_t number = 0U;
    if (*cursor < '0' || *cursor > '9') { send_error("bad $ syntax"); return; }
    while (*cursor >= '0' && *cursor <= '9') number = number * 10U + (uint32_t)(*cursor++ - '0');
    if (*cursor++ != '=') { send_error("bad $ syntax"); return; }
    uint32_t value;
    if (!parse_unsigned(cursor, &value)) { send_error("bad value"); return; }

    if (Stepper_IsBusy() && number >= 20U && number <= 24U) { send_error("busy"); return; }
    if ((number == 0U || number == 1U || number == 4U || number == 5U || number == 6U) &&
        (value < STEPPER_MIN_SPEED_SPS || value > STEPPER_MAX_SPEED_SPS)) {
        send_error("speed range 20..10000"); return;
    }
    if ((number == 2U || number == 3U || number == 7U) && (value == 0U || value > 1000000U)) {
        send_error("accel range 1..1000000"); return;
    }
    if ((number == 10U || number == 11U || number == 12U || number == 13U ||
         number == 15U || number == 16U) && value == 0U) {
        send_error("calibration must be >0"); return;
    }
    if ((number == 14U || (number >= 20U && number <= 24U)) && value > 1U) {
        send_error("boolean must be 0 or 1"); return;
    }
    switch (number) {
        case 0: gc.max_sps[AXIS_X] = value; Stepper_SetSpeed(AXIS_X, value); break;
        case 1: gc.max_sps[AXIS_Y] = value; Stepper_SetSpeed(AXIS_Y, value); break;
        case 2: gc.accel[AXIS_X] = value; Stepper_SetAccel(AXIS_X, value); break;
        case 3: gc.accel[AXIS_Y] = value; Stepper_SetAccel(AXIS_Y, value); break;
        case 4: gc.feed_sps = value; break;
        case 5: gc.rapid_sps = value; break;
        case 6: gc.max_sps[AXIS_Z] = value; Stepper_SetSpeed(AXIS_Z, value); break;
        case 7: gc.accel[AXIS_Z] = value; Stepper_SetAccel(AXIS_Z, value); break;
        case 10: gc.steps_per_rev[AXIS_X] = value; break;
        case 11: gc.um_per_rev[AXIS_X] = value; break;
        case 12: gc.steps_per_rev[AXIS_Y] = value; break;
        case 13: gc.um_per_rev[AXIS_Y] = value; break;
        case 14: gc.unit_mm = value != 0U; break;
        case 15: gc.steps_per_rev[AXIS_Z] = value; break;
        case 16: gc.um_per_rev[AXIS_Z] = value; break;
        case 20: Stepper_SetDirectionInverted(AXIS_X, value != 0U); break;
        case 21: Stepper_SetDirectionInverted(AXIS_Y, value != 0U); break;
        case 22: Stepper_SetDirectionInverted(AXIS_Z, value != 0U); break;
        case 23: Limit_SetActiveHigh(value != 0U); break;
        case 24: Limit_SetEnabled(value != 0U); break;
        default: send_error("unknown setting"); return;
    }
    send_ok();
}

static bool start_move(int32_t target[NUM_AXES], uint32_t speed)
{
    if (s_estop_latched || gc.alarm || gc.paused) { send_error("hold/alarm"); return false; }
    if (Stepper_IsBusy()) { send_error("busy"); return false; }
    int32_t machine_target[NUM_AXES];
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) machine_target[axis] = target[axis] - gc.offset[axis];
    if (!Stepper_MoveCoordinated(machine_target, speed)) { send_error("move rejected"); return false; }
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) gc.work_position[axis] = target[axis] + gc.offset[axis];
    send_ok();
    return true;
}

static void print_direction_settings(void)
{
    send_value("X=", Stepper_GetDirectionInverted(AXIS_X) ? 1 : 0, " ;0=normal 1=reverse\r\n");
    send_value("Y=", Stepper_GetDirectionInverted(AXIS_Y) ? 1 : 0, " ;0=normal 1=reverse\r\n");
    send_value("Z=", Stepper_GetDirectionInverted(AXIS_Z) ? 1 : 0, " ;0=normal 1=reverse\r\n");
    send_ok();
}

static void parse_direction_command(const char *line)
{
    const char *cursor = line + 4;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;

    uint8_t axis;
    if (*cursor == 'X') axis = AXIS_X;
    else if (*cursor == 'Y') axis = AXIS_Y;
    else if (*cursor == 'Z') axis = AXIS_Z;
    else { send_error("DIR needs X, Y, or Z"); return; }

    ++cursor;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0' || strcmp(cursor, "?") == 0) {
        send_value("DIR=", Stepper_GetDirectionInverted(axis) ? 1 : 0,
                   " ;0=normal 1=reverse\r\n");
        send_ok();
        return;
    }
    if (Stepper_IsBusy()) { send_error("busy"); return; }

    bool inverted;
    if (strcmp(cursor, "NORMAL") == 0 || strcmp(cursor, "0") == 0) inverted = false;
    else if (strcmp(cursor, "REVERSE") == 0 || strcmp(cursor, "1") == 0) inverted = true;
    else if (strcmp(cursor, "TOGGLE") == 0) inverted = !Stepper_GetDirectionInverted(axis);
    else { send_error("DIR value NORMAL, REVERSE, or TOGGLE"); return; }

    Stepper_SetDirectionInverted(axis, inverted);
    send_ok();
}

static void parse_limit_command(const char *line)
{
    const char *cursor = line + 5;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0' || strcmp(cursor, "?") == 0) {
        send_value("LIMIT=", Limit_GetEnabled() ? 1 : 0, " ;0=off 1=on\r\n");
        send_ok();
        return;
    }
    if (Stepper_IsBusy()) { send_error("busy"); return; }

    bool enabled;
    if (strcmp(cursor, "ON") == 0 || strcmp(cursor, "1") == 0) enabled = true;
    else if (strcmp(cursor, "OFF") == 0 || strcmp(cursor, "0") == 0) enabled = false;
    else { send_error("LIMIT value ON or OFF"); return; }

    Limit_SetEnabled(enabled);
    if (!enabled) Stepper_ClearLimitStopped();
    send_ok();
}

static bool execute_line(CommandSource source, char *line, bool script)
{
    s_response_source = source;
    strip_comments(line);
    uppercase(line);
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') { if (!script) send_ok(); return true; }

    if (strcmp(line, "HELP") == 0) { GCode_Send(s_command_help); return true; }
    if (strcmp(line, "CANID") == 0 || strcmp(line, "CANID?") == 0 ||
        strncmp(line, "CANID ", 6U) == 0) { parse_can_id_command(line); return true; }
    if (strcmp(line, "CAN STATUS") == 0) { send_can_status(); return true; }
    if (strcmp(line, "CAN TEST") == 0) { run_can_test(); return true; }
    if (strcmp(line, "ESTOP") == 0) { trigger_estop_command(); return true; }
    if (strcmp(line, "ESTOP?") == 0) {
        send_value("ESTOP=", s_estop_latched ? 1 : 0, " ;latched\r\n");
        send_value("BUTTON=", estop_button_active() ? 1 : 0, " ;physical input\r\n");
        send_ok();
        return true;
    }
    if (strcmp(line, "ESTOP RESET") == 0) { unlock_alarms(); return true; }
    if (strcmp(line, "CLEAR ALARM") == 0 || strcmp(line, "ALARM CLEAR") == 0) {
        unlock_alarms();
        return true;
    }
    if (strcmp(line, "DIR?") == 0) { print_direction_settings(); return true; }
    if (strncmp(line, "DIR ", 4U) == 0) { parse_direction_command(line); return true; }
    if (strcmp(line, "LIMIT") == 0 || strcmp(line, "LIMIT?") == 0 ||
        strncmp(line, "LIMIT ", 6U) == 0) { parse_limit_command(line); return true; }
    if (strcmp(line, "?") == 0) { send_status(); return true; }
    if (strcmp(line, "!") == 0) {
        Stepper_StopAll();
        LinearActuator_Stop();
        gc.paused = true;
        CycleEngine_Stop();
        GCode_Send("HOLD\r\n");
        return true;
    }
    if (strcmp(line, "~") == 0) { gc.paused = false; send_ok(); return true; }
    if (strncmp(line, "RUN ", 4U) == 0) return CycleEngine_Run(line + 4, source);
    if (strcmp(line, "LIST") == 0) { CycleEngine_List(source); send_ok(); return true; }
    if (strcmp(line, "STOP") == 0) {
        Stepper_StopAll();
        LinearActuator_Stop();
        CycleEngine_Stop();
        send_ok();
        return true;
    }
    if (strncmp(line, "MACRO ", 6U) == 0) { CycleEngine_PrintMacro(line + 6, source); send_ok(); return true; }
    if (*line == '$') { parse_setting(line); return true; }

    bool has_g, has_m, has_x, has_y, has_z, has_f, has_p;
    int32_t g10 = parse_word_tenths(line, 'G', &has_g);
    int32_t m10 = parse_word_tenths(line, 'M', &has_m);
    int32_t x10 = parse_word_tenths(line, 'X', &has_x);
    int32_t y10 = parse_word_tenths(line, 'Y', &has_y);
    int32_t z10 = parse_word_tenths(line, 'Z', &has_z);
    int32_t f10 = parse_word_tenths(line, 'F', &has_f);
    int32_t p10 = parse_word_tenths(line, 'P', &has_p);
    int32_t g = tenths_to_integer(g10);
    int32_t m = tenths_to_integer(m10);

    if (has_m) {
        if (m == 0 || m == 1) { gc.paused = true; send_ok(); return true; }
        if (m == 2 || m == 18 || m == 30 || m == 84) {
            Stepper_StopAll();
            LinearActuator_Stop();
            send_ok();
            return true;
        }
        if (m == 3 || m == 4) {
            if (s_estop_latched || gc.alarm || gc.paused) {
                send_error("hold/alarm");
                return false;
            }
            bool has_s;
            int32_t s10 = parse_word_tenths(line, 'S', &has_s);
            if (has_p || (has_s && s10 < 0)) {
                send_error("M3/M4 needs S>=0; P unsupported");
                return false;
            }
            uint32_t duration_ms = has_s ? (uint32_t)tenths_to_integer(s10) : 0U;
            LinearActuatorDirection direction = m == 3 ? LINEAR_ACTUATOR_EXTEND :
                                                         LINEAR_ACTUATOR_RETRACT;
            LinearActuator_Run(direction, duration_ms);
            if (direction == LINEAR_ACTUATOR_EXTEND) {
                GCode_Send("[MSG:Linear actuator extending]\r\n");
            } else {
                GCode_Send("[MSG:Linear actuator retracting]\r\n");
            }
            send_ok();
            return true;
        }
        if (m == 5) {
            LinearActuator_Stop();
            GCode_Send("[MSG:Linear actuator stopped]\r\n");
            send_ok();
            return true;
        }
        if (m == 17) { send_ok(); return true; }
        if (m == 112) { trigger_estop_command(); return true; }
        send_error("M?"); return false;
    }

    if (!has_g && (has_x || has_y || has_z)) { has_g = true; g = 1; }
    if (!has_g) { send_error("command?"); return false; }

    switch (g) {
        case 0:
        case 1: {
            int32_t target[NUM_AXES] = {gc.work_position[AXIS_X], gc.work_position[AXIS_Y], gc.work_position[AXIS_Z]};
            int32_t words[NUM_AXES] = {x10, y10, z10};
            bool present[NUM_AXES] = {has_x, has_y, has_z};
            for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
                if (!present[axis]) continue;
                int32_t coordinate = gc.unit_mm ? tenths_mm_to_steps(words[axis], axis) : tenths_to_integer(words[axis]);
                if (gc.distance_mode == DIST_RELATIVE) target[axis] += coordinate;
                else target[axis] = coordinate;
            }
            uint32_t speed = g == 0 ? gc.rapid_sps : gc.feed_sps;
            if (g == 1 && has_f && f10 > 0) {
                uint32_t feed = (uint32_t)tenths_to_integer(f10);
                if (gc.unit_mm) {
                    uint8_t dominant = AXIS_X;
                    uint32_t longest = 0U;
                    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
                        int64_t delta = (int64_t)target[axis] - gc.work_position[axis];
                        uint32_t distance = (uint32_t)(delta < 0 ? -delta : delta);
                        if (distance > longest) { longest = distance; dominant = axis; }
                    }
                    speed = feed_mm_per_min_to_sps(feed, dominant);
                } else {
                    speed = feed;
                }
                gc.feed_sps = speed;
            }
            return start_move(target, speed);
        }
        case 4:
            if (!has_p) { send_error("G4 needs P"); return false; }
            gc.dwelling = true;
            gc.dwell_end_ms = HAL_GetTick() + (uint32_t)tenths_to_integer(p10);
            send_ok();
            return true;
        case 28: {
            int32_t target[NUM_AXES] = {0, 0, 0};
            return start_move(target, gc.rapid_sps);
        }
        case 90: gc.distance_mode = DIST_ABSOLUTE; send_ok(); return true;
        case 91: gc.distance_mode = DIST_RELATIVE; send_ok(); return true;
        case 92:
            if (Stepper_IsBusy()) { send_error("busy"); return false; }
            if (has_x) { int32_t v = gc.unit_mm ? tenths_mm_to_steps(x10, AXIS_X) : tenths_to_integer(x10); gc.offset[AXIS_X] = v - Stepper_GetPos(AXIS_X); gc.work_position[AXIS_X] = v; }
            if (has_y) { int32_t v = gc.unit_mm ? tenths_mm_to_steps(y10, AXIS_Y) : tenths_to_integer(y10); gc.offset[AXIS_Y] = v - Stepper_GetPos(AXIS_Y); gc.work_position[AXIS_Y] = v; }
            if (has_z) { int32_t v = gc.unit_mm ? tenths_mm_to_steps(z10, AXIS_Z) : tenths_to_integer(z10); gc.offset[AXIS_Z] = v - Stepper_GetPos(AXIS_Z); gc.work_position[AXIS_Z] = v; }
            send_ok(); return true;
        default: send_error("G?"); return false;
    }
}

static bool command_is_realtime(const char *line)
{
    while (*line == ' ' || *line == '\t') ++line;
    if ((line[0] == '?' || line[0] == '!' || line[0] == '~') && line[1] == '\0') return true;
    char upper[8];
    uint8_t length = 0U;
    while (line[length] != '\0' && length < sizeof(upper) - 1U) {
        char value = line[length];
        upper[length] = (value >= 'a' && value <= 'z') ? (char)(value - 32) : value;
        ++length;
    }
    upper[length] = '\0';
    return strcmp(upper, "STOP") == 0 || strcmp(upper, "M5") == 0 ||
           strcmp(upper, "M112") == 0 ||
           strcmp(upper, "ESTOP") == 0;
}

static bool command_processing_busy(void)
{
    return Stepper_IsBusy() || gc.dwelling || CycleEngine_IsBusy();
}

static void submit_line(CommandSource source, char *line)
{
    if (!command_processing_busy() || command_is_realtime(line)) {
        (void)execute_line(source, line, false);
        return;
    }
    uint8_t next = (uint8_t)((gc.queue_head + 1U) % COMMAND_QUEUE_DEPTH);
    if (next == gc.queue_tail) {
        GCode_SendTo(source, "error:command queue full\r\n");
        return;
    }
    size_t length = strlen(line);
    memcpy(gc.queue[gc.queue_head].line, line, length + 1U);
    gc.queue[gc.queue_head].source = source;
    gc.queue_head = next;
}

void GCode_Init(void)
{
    memset(&gc, 0, sizeof(gc));
    gc.distance_mode = DIST_ABSOLUTE;
    gc.feed_sps = DEFAULT_FEED_SPS;
    gc.rapid_sps = DEFAULT_RAPID_SPS;
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        gc.accel[axis] = STEPPER_DEFAULT_ACCEL_SPS2;
        gc.max_sps[axis] = STEPPER_DEFAULT_MAX_SPEED_SPS;
        gc.steps_per_rev[axis] = DEFAULT_STEPS_PER_REV;
        gc.um_per_rev[axis] = DEFAULT_UM_PER_REV;
    }
    if (estop_button_active()) {
        Stepper_StopAll();
        s_estop_latched = true;
        s_estop_report_pending = true;
    }
}

void GCode_PutCharFrom(CommandSource source, char character)
{
    if ((uint8_t)source >= (uint8_t)COMMAND_SOURCE_COUNT) return;
    InputBuffer *input = &gc.input[(uint8_t)source];
    if (character == '\r' || character == '\n') {
        if (input->length != 0U) {
            input->line[input->length] = '\0';
            submit_line(source, input->line);
            input->length = 0U;
        }
        return;
    }
    if (input->length < GCODE_LINE_MAX - 1U) input->line[input->length++] = character;
    else input->length = 0U;
}

bool GCode_ExecuteScriptLine(CommandSource source, const char *line)
{
    char copy[GCODE_LINE_MAX];
    size_t length = strlen(line);
    if (length >= sizeof(copy)) return false;
    memcpy(copy, line, length + 1U);
    return execute_line(source, copy, true);
}

void GCode_Poll(void)
{
    LinearActuator_Poll();
    if (take_estop_report_pending()) enter_estop_alarm();
    if (gc.dwelling && (int32_t)(HAL_GetTick() - gc.dwell_end_ms) >= 0) gc.dwelling = false;
    if (Stepper_LimitStopped() && !gc.alarm) {
        gc.alarm = true;
        GCode_SendTo(COMMAND_SOURCE_USB, "ALARM:Z limit\r\n");
        GCode_SendTo(COMMAND_SOURCE_UART, "ALARM:Z limit\r\n");
        GCode_SendTo(COMMAND_SOURCE_CAN, "ALARM:Z limit\r\n");
    }
    CycleEngine_Poll();
    if (!command_processing_busy() && gc.queue_tail != gc.queue_head) {
        QueuedCommand *command = &gc.queue[gc.queue_tail];
        gc.queue_tail = (uint8_t)((gc.queue_tail + 1U) % COMMAND_QUEUE_DEPTH);
        (void)execute_line(command->source, command->line, false);
    }
}
