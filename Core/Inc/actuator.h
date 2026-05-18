/* actuator.h — Linear DC actuator driver via L298 on PB10/PB11
 *
 * L298 wiring:
 *   PB10 → IN1  (HIGH = extend direction)
 *   PB11 → IN2  (HIGH = retract direction)
 *
 * L298 truth table:
 *   IN1=1 IN2=0 → Motor forward  (EXTEND)
 *   IN1=0 IN2=1 → Motor reverse  (RETRACT)
 *   IN1=0 IN2=0 → Coast / stop
 *   IN1=1 IN2=1 → Brake (avoid — may damage L298)
 *
 * G-code commands:
 *   M3 S<ms>  — extend   for S milliseconds  (0 = run indefinitely)
 *   M4 S<ms>  — retract  for S milliseconds  (0 = run indefinitely)
 *   M5        — stop actuator immediately
 *
 * Actuator_Poll() must be called from the main loop (GCode_Poll).
 */

#ifndef ACTUATOR_H
#define ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ACT_STOP    = 0,
    ACT_EXTEND  = 1,
    ACT_RETRACT = 2
} ActuatorDir;

/* Initialise GPIO pins PB10 and PB11 */
void Actuator_Init(void);

/* Start actuator in given direction for duration_ms.
 * Pass duration_ms = 0 to run until Actuator_Stop() is called. */
void Actuator_Run(ActuatorDir dir, uint32_t duration_ms);

/* Stop actuator immediately */
void Actuator_Stop(void);

/* Returns true if a timed run is still active */
bool Actuator_IsBusy(void);

/* Get current direction */
ActuatorDir Actuator_GetDir(void);

/* Call from main loop — handles timed stop */
void Actuator_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATOR_H */
