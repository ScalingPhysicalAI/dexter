/* gcode.h — GRBL-compatible G-code parser for HumanoidBase
 *
 * Supported commands
 * ──────────────────
 *  G0  Xnnn Ynnn          Rapid move (max speed)
 *  G1  Xnnn Ynnn Fnnn     Linear move with feed-rate F (steps/s)
 *  G4  Pnnn               Dwell P milliseconds
 *  G28                    Home all axes (go to zero)
 *  G90                    Absolute positioning (default)
 *  G91                    Relative (incremental) positioning
 *  G92 Xnnn Ynnn          Set current position (offset)
 *
 *  M0  / M1               Program pause (wait for resume)
 *  M2  / M30              Program end
 *  M17                    Enable steppers (no-op, always on)
 *  M18 / M84              Disable steppers (stop)
 *  M112                   Emergency stop
 *
 *  $  (bare)              Print settings
 *  $H                     Home
 *  $X                     Kill alarm
 *  ?                      Status query  → "<Idle|MPos:x,y>"
 *  !                      Feed hold (stop with decel)
 *  ~                      Cycle start / resume
 *
 * Comments: anything after ';' or inside '(' ... ')' is ignored.
 * Line numbers (Nnnn) are accepted and ignored.
 * All word values may be integers or decimals (e.g. X10.5).
 * Units are STEPS throughout (no mm conversion — add scale if needed).
 */

#ifndef GCODE_H
#define GCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Feed-rate unit in steps/s.  Host sends F in steps/s directly. */

/**
 * Initialise the G-code layer.  Must be called after Stepper_Init().
 */
void GCode_Init(void);

/**
 * Feed one character into the line-buffer.
 * When '\n' or '\r' is received the accumulated line is parsed and
 * executed.  Call this from your USB-CDC receive callback.
 */
void GCode_PutChar(char c);

/**
 * Poll — call from main loop.
 * Handles dwell timing, async responses, etc.
 */
void GCode_Poll(void);

/**
 * Send a NUL-terminated string over USB-CDC.
 * Used internally; exposed for debug convenience.
 */
void GCode_Send(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* GCODE_H */
