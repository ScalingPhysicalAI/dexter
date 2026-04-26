/* stepper.h — 2-axis stepper driver, STM32F103C6 (32 KB flash)
 *
 *  PA0 TIM2-CH1 STEP_X   PA4 DIR_X
 *  PA1 TIM2-CH2 STEP_Y   PA5 DIR_Y
 */
#ifndef STEPPER_H
#define STEPPER_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── defaults (override with Stepper_SetSpeed / Stepper_SetAccel) ──────── */
#define STEPPER_DEFAULT_MAX_SPEED_SPS   1000UL
#define STEPPER_DEFAULT_ACCEL_SPS2      5000UL
#define STEPPER_MIN_SPEED_SPS           50UL
#define STEPPER_TIMER_CLOCK_HZ          48000000UL

#define AXIS_X   0
#define AXIS_Y   1
#define NUM_AXES 2

typedef enum { MOTOR_IDLE=0, MOTOR_ACCEL, MOTOR_CRUISE, MOTOR_DECEL } MotorState;

typedef struct {
    uint32_t max_speed_sps;
    uint32_t accel_sps2;

    volatile int32_t  pos;
    volatile int32_t  target;
    volatile int32_t  steps_total;
    volatile int32_t  steps_done;

    volatile uint32_t cur_speed_sps;
    volatile uint32_t cruise_speed_sps;

    volatile MotorState state;
    volatile bool       dir;
    volatile bool       busy;
} AxisCtrl;

extern AxisCtrl g_axis[NUM_AXES];

void    Stepper_Init       (TIM_HandleTypeDef *htim);
void    Stepper_MoveTo     (uint8_t axis, int32_t abs_pos);
void    Stepper_MoveRel    (uint8_t axis, int32_t delta);
void    Stepper_SetSpeed   (uint8_t axis, uint32_t sps);
void    Stepper_SetAccel   (uint8_t axis, uint32_t sps2);
bool    Stepper_IsBusy     (void);
bool    Stepper_IsAxisBusy (uint8_t axis);
void    Stepper_StopAll    (void);
int32_t Stepper_GetPos     (uint8_t axis);
void    Stepper_SetZero    (uint8_t axis);
void    Stepper_TIM_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif /* STEPPER_H */
