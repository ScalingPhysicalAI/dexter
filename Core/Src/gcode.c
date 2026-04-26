/* gcode.c — GRBL-compatible G-code parser, flash-optimised for STM32F103C6 (32 KB)
 *
 * No float, no snprintf, no math.h, no stdlib — safe for 32 KB flash.
 *
 * Architecture (ISR-safe):
 *   - CDC_Receive_FS() (USB ISR) → writes raw bytes to ring buffer only
 *   - USB_CDC_DrainRX() called from GCode_Poll() in main loop → GCode_PutChar()
 *   - GCode_Send() copies into static g_tx_buf before passing to USB DMA
 *     (stack-local buffers must NEVER be passed to CDC_Transmit_FS)
 */

#include "gcode.h"
#include "stepper.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

/* ── config ─────────────────────────────────────────────────────────────── */
#define GCODE_LINE_MAX   80
#define DEFAULT_FEED_SPS    500UL
#define DEFAULT_RAPID_SPS   STEPPER_DEFAULT_MAX_SPEED_SPS
#define DEFAULT_ACCEL_SPS2  STEPPER_DEFAULT_ACCEL_SPS2

/* ══════════════════════════════════════════════════════════════════════════
 *  TX — static buffer so USB DMA always has a valid persistent pointer
 * ══════════════════════════════════════════════════════════════════════════ */

void GCode_Send(const char *str)
{
    uint16_t len = 0;
    while (str[len]) len++;
    /* Push into TX ring buffer — USB_CDC_TxPoll() drains it from main loop.
     * No DMA pointer concerns: TxWrite copies bytes into the ring,
     * TxPoll copies ring→UserTxBufferFS (static) before firing DMA. */
    USB_CDC_TxWrite(str, len);
}

/* Append unsigned decimal to buffer, return pointer past last char written */
static char *fmt_u32(char *p, uint32_t v)
{
    char tmp[10]; int n = 0;
    if (v == 0) { *p++ = '0'; return p; }
    while (v > 0) { tmp[n++] = (char)('0' + v % 10); v /= 10; }
    while (n > 0) { *p++ = tmp[--n]; }   /* reverse in-place */
    return p;
}

static char *fmt_i32(char *p, int32_t v)
{
    if (v < 0) { *p++ = '-'; v = -v; }
    return fmt_u32(p, (uint32_t)v);
}

static char *fmt_str(char *p, const char *s)
{
    while (*s) *p++ = *s++;
    return p;
}

/* Send:  key + int32 + suffix   e.g. "$0=" + 2000 + " ;X max sps\r\n"
 * buf must hold: len(key) + 11 (max int32 digits+sign) + len(sfx) + 1
 * Longest suffix here is " ;X accel sps2\r\n" = 17 chars
 * key max = 3, number max = 11 → 3+11+17+1 = 32 — use 48 for safety */
static void send_kv(const char *key, int32_t val, const char *sfx)
{
    char buf[48]; char *p = buf;
    p = fmt_str(p, key);
    p = fmt_i32(p, val);
    p = fmt_str(p, sfx);
    *p = '\0';
    GCode_Send(buf);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Parser state
 * ══════════════════════════════════════════════════════════════════════════ */
typedef enum { DIST_ABS = 0, DIST_REL } DistMode;

static struct {
    char     line_buf[GCODE_LINE_MAX];
    uint8_t  line_len;

    DistMode  dist_mode;
    uint32_t  feed_sps;
    uint32_t  rapid_sps;
    uint32_t  accel[NUM_AXES];
    uint32_t  max_sps[NUM_AXES];

    int32_t   wpos[NUM_AXES];
    int32_t   offset[NUM_AXES];

    bool      dwelling;
    uint32_t  dwell_end_ms;
    bool      paused;
    bool      alarm;
} gc;

/* ══════════════════════════════════════════════════════════════════════════
 *  Word parser — integer only, no float
 * ══════════════════════════════════════════════════════════════════════════
 * Returns value in TENTHS (×10).  e.g. "X10.5" → 105, "X-3" → -30
 * Caller converts: steps = (tenths + 5) / 10  for positive values.
 */
static int32_t parse_word_tenths(const char *line, char word, bool *found)
{
    *found = false;
    char w = (word >= 'a' && word <= 'z') ? (char)(word - 32) : word;
    for (const char *p = line; *p; p++) {
        char c = (*p >= 'a' && *p <= 'z') ? (char)(*p - 32) : *p;
        if (c != w) continue;
        /* skip if preceded by another letter (substring guard) */
        if (p > line) {
            char prev = *(p - 1);
            if ((prev >= 'A' && prev <= 'Z') || (prev >= 'a' && prev <= 'z')) continue;
        }
        *found = true;
        p++;
        int32_t sign = 1;
        if (*p == '-') { sign = -1; p++; }
        else if (*p == '+') { p++; }
        int32_t intpart = 0;
        while (*p >= '0' && *p <= '9') { intpart = intpart * 10 + (*p - '0'); p++; }
        int32_t frac = 0;
        if (*p == '.') { p++; if (*p >= '0' && *p <= '9') frac = *p - '0'; }
        return sign * (intpart * 10 + frac);
    }
    return 0;
}

static inline int32_t tenths_to_steps(int32_t t)
{
    if (t >= 0) return (t + 5) / 10;
    return -((-t + 5) / 10);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  String helpers
 * ══════════════════════════════════════════════════════════════════════════ */
static void str_upper(char *s)
{
    for (; *s; s++) if (*s >= 'a' && *s <= 'z') *s = (char)(*s - 32);
}

static void strip_comments(char *line)
{
    char *p = line;
    while (*p) {
        if (*p == ';') { *p = '\0'; return; }
        if (*p == '(') {
            char *q = p + 1;
            while (*q && *q != ')') q++;
            if (*q == ')') {
                /* shift everything after ')' left to overwrite comment */
                char *dst = p, *src = q + 1;
                while (*src) *dst++ = *src++;
                *dst = '\0';
                /* don't advance p — recheck same position */
            } else { *p = '\0'; return; }
        } else { p++; }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Responses
 * ══════════════════════════════════════════════════════════════════════════ */
static void send_ok(void)  { GCode_Send("ok\r\n"); }
static void send_err(const char *m)
{
    GCode_Send("error:"); GCode_Send(m); GCode_Send("\r\n");
}

static void send_status(void)
{
    /* Worst case: "<Alarm|MPos:-32768,-32768|WPos:-32768,-32768|F:65535>\r\n" = ~55 chars */
    char buf[80]; char *p = buf;
    const char *st;
    if      (gc.alarm)          st = "Alarm";
    else if (gc.paused)         st = "Hold";
    else if (Stepper_IsBusy())  st = "Run";
    else                        st = "Idle";

    p = fmt_str(p, "<");
    p = fmt_str(p, st);
    p = fmt_str(p, "|MPos:");
    p = fmt_i32(p, Stepper_GetPos(AXIS_X));
    *p++ = ',';
    p = fmt_i32(p, Stepper_GetPos(AXIS_Y));
    p = fmt_str(p, "|WPos:");
    p = fmt_i32(p, Stepper_GetPos(AXIS_X) + gc.offset[AXIS_X]);
    *p++ = ',';
    p = fmt_i32(p, Stepper_GetPos(AXIS_Y) + gc.offset[AXIS_Y]);
    p = fmt_str(p, "|F:");
    p = fmt_u32(p, gc.feed_sps);
    p = fmt_str(p, ">\r\n");
    *p = '\0';
    GCode_Send(buf);
}

static void print_settings(void)
{
    send_kv("$0=", (int32_t)gc.max_sps[AXIS_X], " ;X max sps\r\n");
    send_kv("$1=", (int32_t)gc.max_sps[AXIS_Y], " ;Y max sps\r\n");
    send_kv("$2=", (int32_t)gc.accel[AXIS_X],   " ;X accel sps2\r\n");
    send_kv("$3=", (int32_t)gc.accel[AXIS_Y],   " ;Y accel sps2\r\n");
    send_kv("$4=", (int32_t)gc.feed_sps,         " ;feed sps\r\n");
    send_kv("$5=", (int32_t)gc.rapid_sps,        " ;rapid sps\r\n");
    send_ok();
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Motion
 * ══════════════════════════════════════════════════════════════════════════ */
/* Wait for BOTH axes simultaneously, servicing USB TX while waiting.
 * TIM2 IRQ (priority 0) preempts freely — no deadlock. */
static void wait_all(void)
{
    while (Stepper_IsAxisBusy(AXIS_X) || Stepper_IsAxisBusy(AXIS_Y)) {
        USB_CDC_TxPoll();
    }
}

/* Wait for a single axis (used after G28 / single-axis moves) */
static void wait_axis(uint8_t axis)
{
    while (Stepper_IsAxisBusy(axis)) {
        USB_CDC_TxPoll();
    }
}

/* Simultaneous 2-axis move with linear interpolation (Bresenham).
 *
 * G0 X1000 Y1000  — both axes start together, each at its own speed
 *   scaled so they finish at the same time (vector feed-rate).
 *
 * G1 X1000 Y500 F500 — resultant feed rate = 500 steps/s along the
 *   vector; X runs at 500*cos(θ), Y at 500*sin(θ) where
 *   θ = atan2(dy, dx). Integer approximation: scale each axis speed
 *   proportionally to its step count vs the dominant axis.
 *
 * For G0 (rapid) both axes run at max speed independently — they start
 * together but the shorter axis finishes first (true rapid behaviour).
 */
static void do_move(int32_t wx, int32_t wy, uint32_t spd)
{
    if (gc.alarm || gc.paused) return;

    /* Wait for any previous move to finish */
    wait_all();

    int32_t mx = wx - gc.offset[AXIS_X];
    int32_t my = wy - gc.offset[AXIS_Y];

    int32_t dx = mx - Stepper_GetPos(AXIS_X);
    int32_t dy = my - Stepper_GetPos(AXIS_Y);

    if (dx == 0 && dy == 0) return;

    /* Compute per-axis speeds for coordinated motion.
     * Dominant axis runs at 'spd'; subordinate axis is scaled:
     *   spd_minor = spd * |d_minor| / |d_major|
     * This keeps the vector resultant speed constant (linear interp). */
    uint32_t steps_x = (uint32_t)(dx < 0 ? -dx : dx);
    uint32_t steps_y = (uint32_t)(dy < 0 ? -dy : dy);

    uint32_t spd_x, spd_y;

    if (steps_x == 0) {
        spd_x = STEPPER_MIN_SPEED_SPS;
        spd_y = spd;
    } else if (steps_y == 0) {
        spd_x = spd;
        spd_y = STEPPER_MIN_SPEED_SPS;
    } else if (steps_x >= steps_y) {
        /* X is dominant */
        spd_x = spd;
        spd_y = (uint32_t)((uint64_t)spd * steps_y / steps_x);
        if (spd_y < STEPPER_MIN_SPEED_SPS) spd_y = STEPPER_MIN_SPEED_SPS;
    } else {
        /* Y is dominant */
        spd_y = spd;
        spd_x = (uint32_t)((uint64_t)spd * steps_x / steps_y);
        if (spd_x < STEPPER_MIN_SPEED_SPS) spd_x = STEPPER_MIN_SPEED_SPS;
    }

    /* Set per-axis speed and accel */
    Stepper_SetSpeed(AXIS_X, spd_x);
    Stepper_SetSpeed(AXIS_Y, spd_y);

    /* Start both axes simultaneously */
    if (dx != 0) Stepper_MoveTo(AXIS_X, mx);
    if (dy != 0) Stepper_MoveTo(AXIS_Y, my);

    /* Wait for both to finish */
    wait_all();

    gc.wpos[AXIS_X] = wx;
    gc.wpos[AXIS_Y] = wy;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  $ settings
 * ══════════════════════════════════════════════════════════════════════════ */
static void parse_setting(const char *line)
{
    char c1 = line[1];

    /* bare $ — print settings */
    if (c1 == '\0' || c1 == ' ' || c1 == '\r' || c1 == '\n') {
        print_settings();
        return;
    }

    /* $H — home */
    if (c1 == 'H') {
        do_move(gc.offset[AXIS_X], gc.offset[AXIS_Y], gc.rapid_sps);
        Stepper_SetZero(AXIS_X);
        Stepper_SetZero(AXIS_Y);
        gc.wpos[AXIS_X] = gc.wpos[AXIS_Y] = 0;
        GCode_Send("[MSG:Homed]\r\n");
        send_ok();
        return;
    }

    /* $X — kill alarm */
    if (c1 == 'X') {
        gc.alarm = false;
        GCode_Send("[MSG:Alarm cleared]\r\n");
        send_ok();
        return;
    }

    /* $n=v */
    int n = 0;
    const char *p = &line[1];
    while (*p >= '0' && *p <= '9') { n = n * 10 + (*p - '0'); p++; }
    if (*p != '=') { send_err("bad $ syntax"); return; }
    p++;
    uint32_t v = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10 + (uint32_t)(*p - '0'); p++; }
    if (v == 0) { send_err("value must be >0"); return; }

    switch (n) {
        case 0: gc.max_sps[AXIS_X] = v; Stepper_SetSpeed(AXIS_X, v); break;
        case 1: gc.max_sps[AXIS_Y] = v; Stepper_SetSpeed(AXIS_Y, v); break;
        case 2: gc.accel[AXIS_X]   = v; Stepper_SetAccel(AXIS_X, v); break;
        case 3: gc.accel[AXIS_Y]   = v; Stepper_SetAccel(AXIS_Y, v); break;
        case 4: gc.feed_sps  = v; break;
        case 5: gc.rapid_sps = v; break;
        default: send_err("unknown $n"); return;
    }
    send_ok();
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Main line executor
 * ══════════════════════════════════════════════════════════════════════════ */
static void execute_line(char *line)
{
    strip_comments(line);
    str_upper(line);
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') { send_ok(); return; }

    /* single-char real-time commands */
    if (line[1] == '\0') {
        if (line[0] == '?') { send_status(); return; }
        if (line[0] == '!') { Stepper_StopAll(); GCode_Send("HOLD\r\n"); return; }
        if (line[0] == '~') { gc.paused = false; GCode_Send("RESUME\r\n"); send_ok(); return; }
    }

    /* $ commands — pass original line (already upper-cased) */
    if (line[0] == '$') { parse_setting(line); return; }

    /* skip optional line number: Nnnn */
    if (line[0] == 'N') {
        while (*line && *line != ' ' && !((*line >= 'G' && *line <= 'M') || *line == '$')) line++;
        while (*line == ' ') line++;
    }

    /* parse word values — all returned in tenths */
    bool has_g, has_m, has_x, has_y, has_f, has_p;
    int32_t g10 = parse_word_tenths(line, 'G', &has_g);
    int32_t m10 = parse_word_tenths(line, 'M', &has_m);
    int32_t x10 = parse_word_tenths(line, 'X', &has_x);
    int32_t y10 = parse_word_tenths(line, 'Y', &has_y);
    int32_t f10 = parse_word_tenths(line, 'F', &has_f);
    int32_t p10 = parse_word_tenths(line, 'P', &has_p);

    int g_code = (int)tenths_to_steps(g10);
    int m_code = (int)tenths_to_steps(m10);

    if (has_f && f10 > 0) gc.feed_sps = (uint32_t)tenths_to_steps(f10);

    /* ── M codes ── */
    if (has_m) {
        switch (m_code) {
            case 0: case 1:
                gc.paused = true;
                GCode_Send("[MSG:Paused-send ~ to resume]\r\n");
                send_ok(); return;
            case 2: case 30:
                Stepper_StopAll();
                GCode_Send("[MSG:Program end]\r\n");
                send_ok(); return;
            case 17:
                send_ok(); return;
            case 18: case 84:
                Stepper_StopAll(); send_ok(); return;
            case 112:
                Stepper_StopAll(); gc.alarm = true;
                GCode_Send("ALARM\r\n"); return;
            default:
                send_err("M?"); return;
        }
    }

    /* implicit G1 if X/Y present with no G word */
    if (!has_g) {
        if (has_x || has_y) { has_g = true; g_code = 1; }
        else { send_ok(); return; }
    }

    /* ── G codes ── */
    switch (g_code) {
        case 0:
        case 1: {
            int32_t tx = gc.wpos[AXIS_X];
            int32_t ty = gc.wpos[AXIS_Y];
            if (gc.dist_mode == DIST_ABS) {
                if (has_x) tx = tenths_to_steps(x10);
                if (has_y) ty = tenths_to_steps(y10);
            } else {
                if (has_x) tx += tenths_to_steps(x10);
                if (has_y) ty += tenths_to_steps(y10);
            }
            uint32_t spd = (g_code == 0) ? gc.rapid_sps : gc.feed_sps;
            if (spd > gc.max_sps[AXIS_X]) spd = gc.max_sps[AXIS_X];
            do_move(tx, ty, spd);
            send_ok();
            break;
        }
        case 4: {
            uint32_t ms = has_p ? (uint32_t)tenths_to_steps(p10) : 0;
            wait_axis(AXIS_X); wait_axis(AXIS_Y);
            gc.dwelling     = true;
            gc.dwell_end_ms = HAL_GetTick() + ms;
            /* ok sent by GCode_Poll() after dwell completes */
            break;
        }
        case 28:
            do_move(gc.offset[AXIS_X], gc.offset[AXIS_Y], gc.rapid_sps);
            Stepper_SetZero(AXIS_X); Stepper_SetZero(AXIS_Y);
            gc.wpos[AXIS_X] = gc.wpos[AXIS_Y] = 0;
            send_ok(); break;
        case 90:
            gc.dist_mode = DIST_ABS; send_ok(); break;
        case 91:
            gc.dist_mode = DIST_REL; send_ok(); break;
        case 92:
            if (has_x) gc.offset[AXIS_X] = tenths_to_steps(x10) - Stepper_GetPos(AXIS_X);
            if (has_y) gc.offset[AXIS_Y] = tenths_to_steps(y10) - Stepper_GetPos(AXIS_Y);
            gc.wpos[AXIS_X] = Stepper_GetPos(AXIS_X) + gc.offset[AXIS_X];
            gc.wpos[AXIS_Y] = Stepper_GetPos(AXIS_Y) + gc.offset[AXIS_Y];
            send_ok(); break;
        default:
            send_err("G?"); break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════════════════════════ */

void GCode_Init(void)
{
    memset(&gc, 0, sizeof(gc));
    gc.dist_mode = DIST_ABS;
    gc.feed_sps  = DEFAULT_FEED_SPS;
    gc.rapid_sps = DEFAULT_RAPID_SPS;
    for (int i = 0; i < NUM_AXES; i++) {
        gc.max_sps[i] = DEFAULT_RAPID_SPS;
        gc.accel[i]   = DEFAULT_ACCEL_SPS2;
        Stepper_SetSpeed(i, gc.max_sps[i]);
        Stepper_SetAccel(i, gc.accel[i]);
    }
    GCode_Send("\r\nHumanoidBase GRBL v1.1\r\n");
    GCode_Send("? status  $ settings  ! hold  ~ resume\r\n");
}

void GCode_PutChar(char c)
{
    /* real-time single-char commands — handled immediately, no buffering */
    if (c == '?') { send_status(); return; }
    if (c == '!') { Stepper_StopAll(); GCode_Send("HOLD\r\n"); return; }
    if (c == '~') { gc.paused = false; GCode_Send("RESUME\r\n"); return; }
    if (c == 0x18) { Stepper_StopAll(); GCode_Init(); return; } /* Ctrl-X */

    if (c == '\r') return; /* ignore CR */

    if (c == '\n') {
        gc.line_buf[gc.line_len] = '\0';
        execute_line(gc.line_buf);
        gc.line_len = 0;
        return;
    }

    if ((c == 0x7F || c == '\b') && gc.line_len > 0) { gc.line_len--; return; }

    if (gc.line_len < GCODE_LINE_MAX - 1) gc.line_buf[gc.line_len++] = c;
}

void GCode_Poll(void)
{
    /* Drain USB RX ring — parse bytes in main-loop context (never in ISR) */
    USB_CDC_DrainRX();

    /* Drain USB TX ring — fire next DMA chunk if USB is free */
    USB_CDC_TxPoll();

    /* Handle dwell completion */
    if (gc.dwelling && HAL_GetTick() >= gc.dwell_end_ms) {
        gc.dwelling = false;
        send_ok();
    }
}
