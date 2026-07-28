#ifndef STEPPER_H
#define STEPPER_H

#include "stm32l5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define NUM_AXES 3U
#define AXIS_X 0U
#define AXIS_Y 1U
#define AXIS_Z 2U
#define STEPPER_MIN_SPEED_SPS 20U
#define STEPPER_MAX_SPEED_SPS 10000U
#define STEPPER_DEFAULT_MAX_SPEED_SPS 1000U
#define STEPPER_DEFAULT_ACCEL_SPS2 5000U

typedef struct {
    volatile int32_t pos;
    volatile int32_t target;
    volatile bool busy;
    uint32_t max_speed_sps;
    uint32_t accel_sps2;
} AxisCtrl;

extern AxisCtrl g_axis[NUM_AXES];

void Stepper_Init(TIM_HandleTypeDef *htim);
bool Stepper_MoveCoordinated(const int32_t target[NUM_AXES], uint32_t speed_sps);
bool Stepper_MoveTo(uint8_t axis, int32_t target);
bool Stepper_MoveRel(uint8_t axis, int32_t delta);
void Stepper_SetSpeed(uint8_t axis, uint32_t speed_sps);
void Stepper_SetAccel(uint8_t axis, uint32_t accel_sps2);
void Stepper_SetDirectionInverted(uint8_t axis, bool inverted);
bool Stepper_GetDirectionInverted(uint8_t axis);
bool Stepper_IsBusy(void);
bool Stepper_IsAxisBusy(uint8_t axis);
void Stepper_StopAll(void);
int32_t Stepper_GetPos(uint8_t axis);
void Stepper_SetPosition(uint8_t axis, int32_t position);
bool Stepper_LimitStopped(void);
void Stepper_ClearLimitStopped(void);
void Stepper_TIM_IRQHandler(void);

#endif
