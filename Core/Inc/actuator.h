/* actuator.h — Linear DC actuator via L298 with PWM speed control
 *
 * Hardware connections:
 *   PB10 → L298 IN1   (direction: HIGH = extend)
 *   PB11 → L298 IN2   (direction: HIGH = retract)
 *   PB0  → L298 ENA   (TIM3 CH3 PWM — speed 0-100%)
 *
 * L298 truth table:
 *   IN1=1 IN2=0 ENA=PWM → Extend  at speed%
 *   IN1=0 IN2=1 ENA=PWM → Retract at speed%
 *   IN1=x IN2=x ENA=0   → Stop (coast)
 *
 * G-code commands:
 *   M3 S<ms> P<pct>  — extend   for S ms at P% speed (S=0 = forever)
 *   M4 S<ms> P<pct>  — retract  for S ms at P% speed (S=0 = forever)
 *   M5               — stop immediately
 *
 *   P word is optional — defaults to last set speed (default 100%)
 *   Speed can also be changed while running: M3 P50 (no S = keep running)
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

/* Initialise GPIO (PB10/PB11) and TIM3 CH3 PWM (PB0) */
void Actuator_Init(void);

/* Run in given direction, duration_ms (0=forever), speed_pct 0-100 */
void Actuator_Run(ActuatorDir dir, uint32_t duration_ms, uint8_t speed_pct);

/* Change speed while running (0-100%) */
void Actuator_SetSpeed(uint8_t speed_pct);

/* Stop immediately */
void Actuator_Stop(void);

/* Returns true if actuator is running */
bool Actuator_IsBusy(void);

/* Get current direction */
ActuatorDir Actuator_GetDir(void);

/* Get current speed 0-100 */
uint8_t Actuator_GetSpeed(void);

/* Call from main loop — handles timed auto-stop */
void Actuator_Poll(void);

#ifdef __cplusplus
}
#endif
#endif /* ACTUATOR_H */
