#include "linear_actuator.h"
#include "main.h"

#define LINEAR_ACTUATOR_PWM_CHANNEL TIM_CHANNEL_1

static TIM_HandleTypeDef *s_pwm_timer;
static volatile LinearActuatorDirection s_direction = LINEAR_ACTUATOR_STOP;
static volatile uint32_t s_stop_at_ms;
static volatile bool s_timed;
static uint8_t s_speed_pct = 100U;

static void set_pwm(uint8_t speed_pct)
{
    if (s_pwm_timer == NULL) return;
    uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(s_pwm_timer) + 1U;
    uint32_t compare = ((uint32_t)speed_pct * period_counts) / 100U;
    __HAL_TIM_SET_COMPARE(s_pwm_timer, LINEAR_ACTUATOR_PWM_CHANNEL, compare);
}

static void set_direction(LinearActuatorDirection direction)
{
    /* Remove drive before changing direction to avoid an H-bridge shoot-through. */
    set_pwm(0U);
    LINEAR_ACT_IN1_GPIO_Port->BRR = LINEAR_ACT_IN1_Pin | LINEAR_ACT_IN2_Pin;

    if (direction == LINEAR_ACTUATOR_EXTEND) {
        LINEAR_ACT_IN1_GPIO_Port->BSRR = LINEAR_ACT_IN1_Pin;
    } else if (direction == LINEAR_ACTUATOR_RETRACT) {
        LINEAR_ACT_IN2_GPIO_Port->BSRR = LINEAR_ACT_IN2_Pin;
    }

    s_direction = direction;
    if (direction != LINEAR_ACTUATOR_STOP) set_pwm(s_speed_pct);
}

void LinearActuator_Init(TIM_HandleTypeDef *pwm_timer)
{
    s_pwm_timer = pwm_timer;
    s_direction = LINEAR_ACTUATOR_STOP;
    s_timed = false;
    s_speed_pct = 100U;
    LINEAR_ACT_IN1_GPIO_Port->BRR = LINEAR_ACT_IN1_Pin | LINEAR_ACT_IN2_Pin;
    set_pwm(0U);
    if (HAL_TIM_PWM_Start(s_pwm_timer, LINEAR_ACTUATOR_PWM_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
}

void LinearActuator_Run(LinearActuatorDirection direction,
                        uint32_t duration_ms,
                        uint8_t speed_pct)
{
    if (direction == LINEAR_ACTUATOR_STOP || speed_pct == 0U) {
        s_speed_pct = speed_pct;
        LinearActuator_Stop();
        return;
    }
    if (speed_pct > 100U) speed_pct = 100U;

    s_speed_pct = speed_pct;
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
    if (s_pwm_timer != NULL) {
        __HAL_TIM_SET_COMPARE(s_pwm_timer, LINEAR_ACTUATOR_PWM_CHANNEL, 0U);
    }
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

uint8_t LinearActuator_GetSpeed(void)
{
    return s_speed_pct;
}
