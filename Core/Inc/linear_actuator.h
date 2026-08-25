#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include "stm32l5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Linear DC actuator through an H-bridge:
 *   PC6            -> ENA (digital HIGH = enabled, LOW = disabled)
 *   PC7            -> IN1 (extend)
 *   PC8            -> IN2 (retract)
 */
typedef enum {
    LINEAR_ACTUATOR_STOP = 0,
    LINEAR_ACTUATOR_EXTEND,
    LINEAR_ACTUATOR_RETRACT
} LinearActuatorDirection;

void LinearActuator_Init(void);
void LinearActuator_Run(LinearActuatorDirection direction,
                        uint32_t duration_ms);
void LinearActuator_Stop(void);
void LinearActuator_EmergencyStopFromISR(void);
void LinearActuator_Poll(void);
bool LinearActuator_IsBusy(void);
LinearActuatorDirection LinearActuator_GetDirection(void);

#endif
