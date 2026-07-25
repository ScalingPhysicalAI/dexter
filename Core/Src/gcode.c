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
#include "actuator.h"
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

/* Unit conversion config
 * steps_per_rev: motor steps per full shaft revolution (e.g. 200 * 16 = 3200)
 * um_per_rev:    distance per revolution in micrometres (e.g. 8 mm = 8000 µm)
 * unit_mm:       0 = all commands in raw steps, 1 = commands in mm
 *
 * steps/mm = steps_per_rev * 1000 / um_per_rev
 * (multiply by 1000 to avoid float: um_per_rev/1000 = mm_per_rev)
 */
#define DEFAULT_STEPS_PER_REV  3200UL   /* 200 steps * 16 microsteps        */
#define DEFAULT_UM_PER_REV     8000UL   /* 8 mm lead screw = 8000 µm/rev    */
#define DEFAULT_Z_STEPS_PER_REV 3200UL  /* Z vertical stepper — same default */
#define DEFAULT_Z_UM_PER_REV    8000UL  /* adjust for your vertical drive    */

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

    /* unit conversion — per axis */
    uint32_t  steps_per_rev[NUM_AXES];   /* pulses per motor revolution      */
    uint32_t  um_per_rev[NUM_AXES];      /* µm travelled per revolution      */
    bool      unit_mm;                   /* false=steps mode, true=mm mode   */
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

/* Convert tenths-of-mm (from parser) to steps using axis calibration.
 * tenths_mm = value × 10 from parse_word_tenths (e.g. "X10.5" → 105)
 * Formula:  steps = tenths_mm × steps_per_rev × 100 / um_per_rev
 *           (×100 because tenths_mm is mm×10, and um_per_rev is µm,
 *            so mm×10 × 1000/10 = mm×100; steps = mm × steps_per_mm)
 * Integer-only, no float, no libm.
 */
static int32_t tenths_mm_to_steps(int32_t t, uint8_t axis)
{
    if (gc.um_per_rev[axis] == 0) return 0;
    /* t is in tenths of mm = µm * 100
     * steps = t * steps_per_rev * 100 / um_per_rev
     * To avoid 32-bit overflow: max t ≈ 100000 (10 m), spr ≈ 6400, upr ≈ 1000
     * 100000 * 6400 * 100 = 6.4e10 → needs 64-bit intermediate */
    int32_t sign = (t < 0) ? -1 : 1;
    uint32_t abs_t = (uint32_t)(t < 0 ? -t : t);
    uint64_t num = (uint64_t)abs_t * gc.steps_per_rev[axis] * 100ULL;
    uint32_t steps = (uint32_t)(num / gc.um_per_rev[axis]);
    return sign * (int32_t)steps;
}

/* Convert steps to tenths-of-mm for display */
static int32_t steps_to_tenths_mm(int32_t steps, uint8_t axis)
{
    if (gc.steps_per_rev[axis] == 0) return 0;
    int32_t sign = (steps < 0) ? -1 : 1;
    uint32_t abs_s = (uint32_t)(steps < 0 ? -steps : steps);
    uint64_t num = (uint64_t)abs_s * gc.um_per_rev[axis];
    uint32_t tenths = (uint32_t)(num / ((uint64_t)gc.steps_per_rev[axis] * 100ULL));
    return sign * (int32_t)tenths;
}

/* Append a fixed-point mm value (tenths_mm / 10) as "nnn.n" */
static char *fmt_mm(char *p, int32_t tenths_mm)
{
    if (tenths_mm < 0) { *p++ = '-'; tenths_mm = -tenths_mm; }
    p = fmt_u32(p, (uint32_t)(tenths_mm / 10));
    *p++ = '.';
    *p++ = (char)('0' + tenths_mm % 10);
    return p;
}

/* Convert F word: mm/min → sps using axis 0 calibration (dominant axis) */
static uint32_t feed_mmpm_to_sps(uint32_t mmpm, uint8_t axis)
{
    if (gc.um_per_rev[axis] == 0 || gc.steps_per_rev[axis] == 0) return gc.feed_sps;
    /* sps = (mmpm * steps_per_rev) / (um_per_rev / 1000 * 60)
     *     = mmpm * steps_per_rev * 1000 / (um_per_rev * 60)  */
    uint64_t num = (uint64_t)mmpm * gc.steps_per_rev[axis] * 1000ULL;
    uint32_t sps = (uint32_t)(num / ((uint64_t)gc.um_per_rev[axis] * 60ULL));
    if (sps < STEPPER_MIN_SPEED_SPS) sps = STEPPER_MIN_SPEED_SPS;
    if (sps > 20000UL)               sps = 20000UL;
    return sps;
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
    char buf[120]; char *p = buf;
    const char *st;
    if      (gc.alarm)          st = "Alarm";
    else if (gc.paused)         st = "Hold";
    else if (Stepper_IsBusy())  st = "Run";
    else                        st = "Idle";

    p = fmt_str(p, "<");
    p = fmt_str(p, st);
    p = fmt_str(p, "|MPos:");
    if (gc.unit_mm) {
        p = fmt_mm(p, steps_to_tenths_mm(Stepper_GetPos(AXIS_X), AXIS_X)); *p++ = ',';
        p = fmt_mm(p, steps_to_tenths_mm(Stepper_GetPos(AXIS_Y), AXIS_Y)); *p++ = ',';
        p = fmt_mm(p, steps_to_tenths_mm(Stepper_GetPos(AXIS_Z), AXIS_Z));
    } else {
        p = fmt_i32(p, Stepper_GetPos(AXIS_X)); *p++ = ',';
        p = fmt_i32(p, Stepper_GetPos(AXIS_Y)); *p++ = ',';
        p = fmt_i32(p, Stepper_GetPos(AXIS_Z));
    }
    p = fmt_str(p, "|WPos:");
    if (gc.unit_mm) {
        p = fmt_mm(p, steps_to_tenths_mm(Stepper_GetPos(AXIS_X)+gc.offset[AXIS_X], AXIS_X)); *p++ = ',';
        p = fmt_mm(p, steps_to_tenths_mm(Stepper_GetPos(AXIS_Y)+gc.offset[AXIS_Y], AXIS_Y)); *p++ = ',';
        p = fmt_mm(p, steps_to_tenths_mm(Stepper_GetPos(AXIS_Z)+gc.offset[AXIS_Z], AXIS_Z));
    } else {
        p = fmt_i32(p, Stepper_GetPos(AXIS_X)+gc.offset[AXIS_X]); *p++ = ',';
        p = fmt_i32(p, Stepper_GetPos(AXIS_Y)+gc.offset[AXIS_Y]); *p++ = ',';
        p = fmt_i32(p, Stepper_GetPos(AXIS_Z)+gc.offset[AXIS_Z]);
    }
    p = fmt_str(p, "|F:");
    p = fmt_u32(p, gc.feed_sps);
    {
        ActuatorDir _d = Actuator_GetDir();
        p = fmt_str(p, "|Act:");
        p = fmt_str(p, _d==ACT_EXTEND ? "Ext" : _d==ACT_RETRACT ? "Ret" : "Stop");
        if (_d != ACT_STOP) { *p++ = '@'; p = fmt_u32(p, Actuator_GetSpeed()); *p++ = '%'; }
    }
    p = fmt_str(p, ">\r\n");
    *p = '\0';
    GCode_Send(buf);
}

static void print_settings(void)
{
    send_kv("$0=",  (int32_t)gc.max_sps[AXIS_X],       " ;X max sps\r\n");
    send_kv("$1=",  (int32_t)gc.max_sps[AXIS_Y],       " ;Y max sps\r\n");
    send_kv("$2=",  (int32_t)gc.accel[AXIS_X],         " ;X accel sps2\r\n");
    send_kv("$3=",  (int32_t)gc.accel[AXIS_Y],         " ;Y accel sps2\r\n");
    send_kv("$4=",  (int32_t)gc.feed_sps,               " ;feed sps\r\n");
    send_kv("$5=",  (int32_t)gc.rapid_sps,              " ;rapid sps\r\n");
    send_kv("$6=",  (int32_t)gc.max_sps[AXIS_Z],       " ;Z max sps\r\n");
    send_kv("$7=",  (int32_t)gc.accel[AXIS_Z],         " ;Z accel sps2\r\n");
    send_kv("$10=", (int32_t)gc.steps_per_rev[AXIS_X],  " ;X pulses/rev\r\n");
    send_kv("$11=", (int32_t)gc.um_per_rev[AXIS_X],     " ;X um/rev\r\n");
    send_kv("$12=", (int32_t)gc.steps_per_rev[AXIS_Y],  " ;Y pulses/rev\r\n");
    send_kv("$13=", (int32_t)gc.um_per_rev[AXIS_Y],     " ;Y um/rev\r\n");
    send_kv("$14=", gc.unit_mm ? 1 : 0,                 " ;unit 0=steps 1=mm\r\n");
    send_kv("$15=", (int32_t)gc.steps_per_rev[AXIS_Z],  " ;Z pulses/rev\r\n");
    send_kv("$16=", (int32_t)gc.um_per_rev[AXIS_Z],     " ;Z um/rev\r\n");
    /* Derived steps/mm for each axis */
    for (int _i = 0; _i < NUM_AXES; _i++) {
        if (gc.um_per_rev[_i] > 0) {
            uint32_t spmm = gc.steps_per_rev[_i] * 1000UL / gc.um_per_rev[_i];
            const char *lbl = (_i==AXIS_X)?"[X steps/mm=":(_i==AXIS_Y)?"[Y steps/mm=":"[Z steps/mm=";
            send_kv(lbl, (int32_t)spmm, "]\r\n");
        }
    }
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
static void do_move(int32_t wx, int32_t wy, int32_t wz, uint32_t spd)
{
    if (gc.alarm || gc.paused) return;

    wait_all();

    int32_t mx = wx - gc.offset[AXIS_X];
    int32_t my = wy - gc.offset[AXIS_Y];
    int32_t mz = wz - gc.offset[AXIS_Z];

    int32_t dx = mx - Stepper_GetPos(AXIS_X);
    int32_t dy = my - Stepper_GetPos(AXIS_Y);
    int32_t dz = mz - Stepper_GetPos(AXIS_Z);

    if (dx == 0 && dy == 0 && dz == 0) return;

    uint32_t sx = (uint32_t)(dx < 0 ? -dx : dx);
    uint32_t sy = (uint32_t)(dy < 0 ? -dy : dy);
    uint32_t sz = (uint32_t)(dz < 0 ? -dz : dz);

    /* Find dominant axis (most steps) — runs at full spd.
     * Others scaled proportionally so all finish simultaneously. */
    uint32_t smax = sx;
    if (sy > smax) smax = sy;
    if (sz > smax) smax = sz;

    uint32_t spd_x = (smax > 0 && sx > 0) ? (uint32_t)((uint64_t)spd * sx / smax) : STEPPER_MIN_SPEED_SPS;
    uint32_t spd_y = (smax > 0 && sy > 0) ? (uint32_t)((uint64_t)spd * sy / smax) : STEPPER_MIN_SPEED_SPS;
    uint32_t spd_z = (smax > 0 && sz > 0) ? (uint32_t)((uint64_t)spd * sz / smax) : STEPPER_MIN_SPEED_SPS;

    if (spd_x < STEPPER_MIN_SPEED_SPS) spd_x = STEPPER_MIN_SPEED_SPS;
    if (spd_y < STEPPER_MIN_SPEED_SPS) spd_y = STEPPER_MIN_SPEED_SPS;
    if (spd_z < STEPPER_MIN_SPEED_SPS) spd_z = STEPPER_MIN_SPEED_SPS;

    Stepper_SetSpeed(AXIS_X, spd_x);
    Stepper_SetSpeed(AXIS_Y, spd_y);
    Stepper_SetSpeed(AXIS_Z, spd_z);

    /* Start all three simultaneously */
    if (dx != 0) Stepper_MoveTo(AXIS_X, mx);
    if (dy != 0) Stepper_MoveTo(AXIS_Y, my);
    if (dz != 0) Stepper_MoveTo(AXIS_Z, mz);

    wait_all();

    gc.wpos[AXIS_X] = wx;
    gc.wpos[AXIS_Y] = wy;
    gc.wpos[AXIS_Z] = wz;
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
        do_move(gc.offset[AXIS_X], gc.offset[AXIS_Y], gc.offset[AXIS_Z], gc.rapid_sps);
        Stepper_SetZero(AXIS_X); Stepper_SetZero(AXIS_Y); Stepper_SetZero(AXIS_Z);
        gc.wpos[AXIS_X] = gc.wpos[AXIS_Y] = gc.wpos[AXIS_Z] = 0;
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
        case 0:  gc.max_sps[AXIS_X] = v; Stepper_SetSpeed(AXIS_X, v); break;
        case 1:  gc.max_sps[AXIS_Y] = v; Stepper_SetSpeed(AXIS_Y, v); break;
        case 2:  gc.accel[AXIS_X]   = v; Stepper_SetAccel(AXIS_X, v); break;
        case 3:  gc.accel[AXIS_Y]   = v; Stepper_SetAccel(AXIS_Y, v); break;
        case 4:  gc.feed_sps  = v; break;
        case 5:  gc.rapid_sps = v; break;
        case 6:  gc.max_sps[AXIS_Z] = v; Stepper_SetSpeed(AXIS_Z, v); break;
        case 7:  gc.accel[AXIS_Z]   = v; Stepper_SetAccel(AXIS_Z, v); break;
        case 10: gc.steps_per_rev[AXIS_X] = v; break;
        case 11: gc.um_per_rev[AXIS_X]    = v; break;
        case 12: gc.steps_per_rev[AXIS_Y] = v; break;
        case 13: gc.um_per_rev[AXIS_Y]    = v; break;
        case 14: gc.unit_mm = (v != 0);        break;
        case 15: gc.steps_per_rev[AXIS_Z] = v; break;
        case 16: gc.um_per_rev[AXIS_Z]    = v; break;
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
                Stepper_StopAll(); Actuator_Stop();
                GCode_Send("[MSG:Program end]\r\n");
                send_ok(); return;
            case 3: {
                /* M3 S<ms> P<pct> — extend, duration ms, speed 0-100% */
                bool has_s; int32_t s10 = parse_word_tenths(line, 'S', &has_s);
                bool has_pw; int32_t pw10 = parse_word_tenths(line, 'P', &has_pw);
                uint32_t ms  = has_s  ? (uint32_t)tenths_to_steps(s10)  : 0;
                uint8_t  pct = has_pw ? (uint8_t) tenths_to_steps(pw10) : Actuator_GetSpeed();
                if (pct > 100) pct = 100;
                Actuator_Run(ACT_EXTEND, ms, pct);
                GCode_Send("[MSG:Actuator extending]\r\n");
                send_ok(); return;
            }
            case 4: {
                /* M4 S<ms> P<pct> — retract, duration ms, speed 0-100% */
                bool has_s; int32_t s10 = parse_word_tenths(line, 'S', &has_s);
                bool has_pw; int32_t pw10 = parse_word_tenths(line, 'P', &has_pw);
                uint32_t ms  = has_s  ? (uint32_t)tenths_to_steps(s10)  : 0;
                uint8_t  pct = has_pw ? (uint8_t) tenths_to_steps(pw10) : Actuator_GetSpeed();
                if (pct > 100) pct = 100;
                Actuator_Run(ACT_RETRACT, ms, pct);
                GCode_Send("[MSG:Actuator retracting]\r\n");
                send_ok(); return;
            }
            case 5:
                /* M5 — stop actuator immediately */
                Actuator_Stop();
                GCode_Send("[MSG:Actuator stopped]\r\n");
                send_ok(); return;
            case 17:
                send_ok(); return;
            case 18: case 84:
                Stepper_StopAll(); Actuator_Stop(); send_ok(); return;
            case 112:
                Stepper_StopAll(); Actuator_Stop(); gc.alarm = true;
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
            bool has_z; int32_t z10 = parse_word_tenths(line, 'Z', &has_z);
            int32_t tx = gc.wpos[AXIS_X];
            int32_t ty = gc.wpos[AXIS_Y];
            int32_t tz = gc.wpos[AXIS_Z];
            if (gc.dist_mode == DIST_ABS) {
                if (has_x) tx = gc.unit_mm ? tenths_mm_to_steps(x10, AXIS_X) : tenths_to_steps(x10);
                if (has_y) ty = gc.unit_mm ? tenths_mm_to_steps(y10, AXIS_Y) : tenths_to_steps(y10);
                if (has_z) tz = gc.unit_mm ? tenths_mm_to_steps(z10, AXIS_Z) : tenths_to_steps(z10);
            } else {
                if (has_x) tx += gc.unit_mm ? tenths_mm_to_steps(x10, AXIS_X) : tenths_to_steps(x10);
                if (has_y) ty += gc.unit_mm ? tenths_mm_to_steps(y10, AXIS_Y) : tenths_to_steps(y10);
                if (has_z) tz += gc.unit_mm ? tenths_mm_to_steps(z10, AXIS_Z) : tenths_to_steps(z10);
            }
            uint32_t spd;
            if (g_code == 0) {
                spd = gc.rapid_sps;
            } else {
                spd = gc.feed_sps;
                if (has_f && f10 > 0) {
                    if (gc.unit_mm)
                        spd = feed_mmpm_to_sps((uint32_t)tenths_to_steps(f10), AXIS_X);
                    else
                        spd = (uint32_t)tenths_to_steps(f10);
                    gc.feed_sps = spd;
                }
            }
            if (spd > gc.max_sps[AXIS_X]) spd = gc.max_sps[AXIS_X];
            do_move(tx, ty, tz, spd);
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
            do_move(gc.offset[AXIS_X], gc.offset[AXIS_Y], gc.offset[AXIS_Z], gc.rapid_sps);
            Stepper_SetZero(AXIS_X); Stepper_SetZero(AXIS_Y); Stepper_SetZero(AXIS_Z);
            gc.wpos[AXIS_X] = gc.wpos[AXIS_Y] = gc.wpos[AXIS_Z] = 0;
            send_ok(); break;
        case 90:
            gc.dist_mode = DIST_ABS; send_ok(); break;
        case 91:
            gc.dist_mode = DIST_REL; send_ok(); break;
        case 92: {
            bool has_z92; int32_t z10_92 = parse_word_tenths(line, 'Z', &has_z92);
            if (has_x) gc.offset[AXIS_X] = (gc.unit_mm ? tenths_mm_to_steps(x10,    AXIS_X) : tenths_to_steps(x10))    - Stepper_GetPos(AXIS_X);
            if (has_y) gc.offset[AXIS_Y] = (gc.unit_mm ? tenths_mm_to_steps(y10,    AXIS_Y) : tenths_to_steps(y10))    - Stepper_GetPos(AXIS_Y);
            if (has_z92) gc.offset[AXIS_Z] = (gc.unit_mm ? tenths_mm_to_steps(z10_92, AXIS_Z) : tenths_to_steps(z10_92)) - Stepper_GetPos(AXIS_Z);
            gc.wpos[AXIS_X] = Stepper_GetPos(AXIS_X) + gc.offset[AXIS_X];
            gc.wpos[AXIS_Y] = Stepper_GetPos(AXIS_Y) + gc.offset[AXIS_Y];
            gc.wpos[AXIS_Z] = Stepper_GetPos(AXIS_Z) + gc.offset[AXIS_Z];
            send_ok(); break;
        }
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
    gc.unit_mm   = false;
    for (int i = 0; i < NUM_AXES; i++) {
        gc.max_sps[i]       = DEFAULT_RAPID_SPS;
        gc.accel[i]         = DEFAULT_ACCEL_SPS2;
        gc.steps_per_rev[i] = (i == AXIS_Z) ? DEFAULT_Z_STEPS_PER_REV : DEFAULT_STEPS_PER_REV;
        gc.um_per_rev[i]    = (i == AXIS_Z) ? DEFAULT_Z_UM_PER_REV    : DEFAULT_UM_PER_REV;
        Stepper_SetSpeed(i, gc.max_sps[i]);
        Stepper_SetAccel(i, gc.accel[i]);
    }
    gc.wpos[AXIS_Z]   = 0;
    gc.offset[AXIS_Z] = 0;
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

    /* Service linear actuator timed stop */
    Actuator_Poll();

    /* Handle dwell completion */
    if (gc.dwelling && HAL_GetTick() >= gc.dwell_end_ms) {
        gc.dwelling = false;
        send_ok();
    }
}
