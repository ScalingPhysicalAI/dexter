#include "linear_actuator.h"
#include "main.h"

static volatile LinearActuatorDirection s_direction = LINEAR_ACTUATOR_STOP;
static volatile uint32_t s_stop_at_ms;
static volatile bool s_timed;

static void set_direction(LinearActuatorDirection direction)
{
    /* Remove drive before changing direction to avoid an H-bridge shoot-through. */
    LINEAR_ACT_EN_GPIO_Port->BRR = LINEAR_ACT_EN_Pin;
    LINEAR_ACT_IN1_GPIO_Port->BRR = LINEAR_ACT_IN1_Pin | LINEAR_ACT_IN2_Pin;

    if (direction == LINEAR_ACTUATOR_EXTEND) {
        LINEAR_ACT_IN1_GPIO_Port->BSRR = LINEAR_ACT_IN1_Pin;
    } else if (direction == LINEAR_ACTUATOR_RETRACT) {
        LINEAR_ACT_IN2_GPIO_Port->BSRR = LINEAR_ACT_IN2_Pin;
    }

    s_direction = direction;
    if (direction != LINEAR_ACTUATOR_STOP) {
        LINEAR_ACT_EN_GPIO_Port->BSRR = LINEAR_ACT_EN_Pin;
    }
}

void LinearActuator_Init(void)
{
    s_direction = LINEAR_ACTUATOR_STOP;
    s_timed = false;
    LINEAR_ACT_EN_GPIO_Port->BRR = LINEAR_ACT_EN_Pin;
    LINEAR_ACT_IN1_GPIO_Port->BRR = LINEAR_ACT_IN1_Pin | LINEAR_ACT_IN2_Pin;
}

void LinearActuator_Run(LinearActuatorDirection direction,
                        uint32_t duration_ms)
{
    if (direction == LINEAR_ACTUATOR_STOP) {
        LinearActuator_Stop();
        return;
    }
    if (duration_ms == 0U) {
        s_timed = false;
    } else {
        s_stop_at_ms = HAL_GetTick() + duration_ms;
        s_timed = true;
    }
    set_direction(direction);
}

void LinearActuator_Stop(void)
{
    s_timed = false;
    set_direction(LINEAR_ACTUATOR_STOP);
}

void LinearActuator_EmergencyStopFromISR(void)
{
    s_timed = false;
    s_direction = LINEAR_ACTUATOR_STOP;
    LINEAR_ACT_EN_GPIO_Port->BRR = LINEAR_ACT_EN_Pin;
    LINEAR_ACT_IN1_GPIO_Port->BRR = LINEAR_ACT_IN1_Pin | LINEAR_ACT_IN2_Pin;
}

void LinearActuator_Poll(void)
{
    if (s_timed && s_direction != LINEAR_ACTUATOR_STOP &&
        (int32_t)(HAL_GetTick() - s_stop_at_ms) >= 0) {
        LinearActuator_Stop();
    }
}

bool LinearActuator_IsBusy(void)
{
    return s_direction != LINEAR_ACTUATOR_STOP;
}

LinearActuatorDirection LinearActuator_GetDirection(void)
{
    return s_direction;
}
