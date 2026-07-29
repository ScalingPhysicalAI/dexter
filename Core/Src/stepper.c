#include "stepper.h"
#include "limit_switches.h"
#include "main.h"

#define SCHEDULER_TICK_US 20U
#define TIMER_RATE_HZ 1000000U

typedef struct {
    volatile bool active;
    volatile uint32_t tick_index;
    uint32_t total_ticks;
    uint32_t axis_steps[NUM_AXES];
    uint32_t error[NUM_AXES];
    bool logical_positive[NUM_AXES];
    uint32_t cruise_sps;
    uint32_t current_sps;
    uint32_t accel_sps2;
    volatile uint32_t next_step_us;
} CoordinatedMove;

AxisCtrl g_axis[NUM_AXES];
static TIM_HandleTypeDef *s_timer;
static CoordinatedMove s_move;
static volatile uint32_t s_time_us;
static volatile bool s_limit_stopped;
static bool s_direction_inverted[NUM_AXES];

static const uint16_t s_step_pin[NUM_AXES] = {X_STEP_Pin, Y_STEP_Pin, Z_STEP_Pin};
static GPIO_TypeDef * const s_step_port[NUM_AXES] = {X_STEP_GPIO_Port, Y_STEP_GPIO_Port, Z_STEP_GPIO_Port};
static const uint16_t s_dir_pin[NUM_AXES] = {X_DIR_Pin, Y_DIR_Pin, Z_DIR_Pin};
static GPIO_TypeDef * const s_dir_port[NUM_AXES] = {X_DIR_GPIO_Port, Y_DIR_GPIO_Port, Z_DIR_GPIO_Port};

static uint32_t isqrt32(uint32_t value)
{
    if (value == 0U) return 0U;
    uint32_t x = value;
    uint32_t next = (value >> 1U) + 1U;
    while (next < x) {
        x = next;
        next = (x + value / x) >> 1U;
    }
    return x;
}

static uint32_t clamp_speed(uint32_t speed)
{
    if (speed < STEPPER_MIN_SPEED_SPS) return STEPPER_MIN_SPEED_SPS;
    if (speed > STEPPER_MAX_SPEED_SPS) return STEPPER_MAX_SPEED_SPS;
    return speed;
}

static uint32_t interval_us(uint32_t speed)
{
    return TIMER_RATE_HZ / clamp_speed(speed);
}

static uint32_t decel_ticks(uint32_t speed, uint32_t accel)
{
    if (accel == 0U || speed <= STEPPER_MIN_SPEED_SPS) return 0U;
    uint32_t min_sq = STEPPER_MIN_SPEED_SPS * STEPPER_MIN_SPEED_SPS;
    uint32_t speed_sq = speed * speed;
    return (speed_sq - min_sq) / (2U * accel);
}

static void set_direction(uint8_t axis, bool logical_positive)
{
    bool pin_high = logical_positive ^ s_direction_inverted[axis];
    HAL_GPIO_WritePin(s_dir_port[axis], s_dir_pin[axis], pin_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void all_steps_low(void)
{
    X_STEP_GPIO_Port->BRR = X_STEP_Pin | Y_STEP_Pin;
    Z_STEP_GPIO_Port->BRR = Z_STEP_Pin;
}

static void finish_move(void)
{
    s_move.active = false;
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        g_axis[axis].busy = false;
        g_axis[axis].target = g_axis[axis].pos;
    }
}

void Stepper_Init(TIM_HandleTypeDef *htim)
{
    s_timer = htim;
    s_time_us = 0U;
    s_limit_stopped = false;
    s_move.active = false;
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        g_axis[axis].pos = 0;
        g_axis[axis].target = 0;
        g_axis[axis].busy = false;
        g_axis[axis].max_speed_sps = STEPPER_DEFAULT_MAX_SPEED_SPS;
        g_axis[axis].accel_sps2 = STEPPER_DEFAULT_ACCEL_SPS2;
        s_direction_inverted[axis] = false;
    }
    all_steps_low();
    if (HAL_TIM_Base_Start_IT(s_timer) != HAL_OK) Error_Handler();
}

bool Stepper_MoveCoordinated(const int32_t target[NUM_AXES], uint32_t speed_sps)
{
    if (target == NULL || s_move.active) return false;

    uint32_t dominant = 0U;
    uint32_t slowest_accel = 0xFFFFFFFFU;
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        int64_t delta = (int64_t)target[axis] - (int64_t)g_axis[axis].pos;
        uint32_t steps = (uint32_t)(delta < 0 ? -delta : delta);
        s_move.axis_steps[axis] = steps;
        s_move.logical_positive[axis] = (delta >= 0);
        if (steps > dominant) dominant = steps;
        if (steps != 0U && g_axis[axis].accel_sps2 < slowest_accel) slowest_accel = g_axis[axis].accel_sps2;
    }
    if (dominant == 0U) return true;

    speed_sps = clamp_speed(speed_sps);
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        if (s_move.axis_steps[axis] != 0U) {
            uint32_t axis_limited_master = (uint32_t)(((uint64_t)g_axis[axis].max_speed_sps * dominant) /
                                                       s_move.axis_steps[axis]);
            if (axis_limited_master < speed_sps) speed_sps = axis_limited_master;
        }
    }
    speed_sps = clamp_speed(speed_sps);

    __disable_irq();
    s_move.total_ticks = dominant;
    s_move.tick_index = 0U;
    s_move.cruise_sps = speed_sps;
    s_move.current_sps = STEPPER_MIN_SPEED_SPS;
    s_move.accel_sps2 = (slowest_accel == 0xFFFFFFFFU) ? STEPPER_DEFAULT_ACCEL_SPS2 : slowest_accel;
    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        s_move.error[axis] = 0U;
        g_axis[axis].target = target[axis];
        g_axis[axis].busy = (s_move.axis_steps[axis] != 0U);
        if (g_axis[axis].busy) set_direction(axis, s_move.logical_positive[axis]);
    }
    s_limit_stopped = false;
    s_move.next_step_us = s_time_us + interval_us(s_move.current_sps);
    s_move.active = true;
    __enable_irq();
    return true;
}

bool Stepper_MoveTo(uint8_t axis, int32_t target)
{
    if (axis >= NUM_AXES) return false;
    int32_t targets[NUM_AXES] = {g_axis[AXIS_X].pos, g_axis[AXIS_Y].pos, g_axis[AXIS_Z].pos};
    targets[axis] = target;
    return Stepper_MoveCoordinated(targets, g_axis[axis].max_speed_sps);
}

bool Stepper_MoveRel(uint8_t axis, int32_t delta)
{
    if (axis >= NUM_AXES) return false;
    return Stepper_MoveTo(axis, g_axis[axis].pos + delta);
}

void Stepper_SetSpeed(uint8_t axis, uint32_t speed_sps)
{
    if (axis < NUM_AXES) g_axis[axis].max_speed_sps = clamp_speed(speed_sps);
}

void Stepper_SetAccel(uint8_t axis, uint32_t accel_sps2)
{
    if (axis < NUM_AXES) g_axis[axis].accel_sps2 = accel_sps2 == 0U ? 1U : accel_sps2;
}

void Stepper_SetDirectionInverted(uint8_t axis, bool inverted)
{
    if (axis < NUM_AXES && !s_move.active) s_direction_inverted[axis] = inverted;
}

bool Stepper_GetDirectionInverted(uint8_t axis)
{
    return axis < NUM_AXES ? s_direction_inverted[axis] : false;
}

bool Stepper_IsBusy(void) { return s_move.active; }
bool Stepper_IsAxisBusy(uint8_t axis) { return axis < NUM_AXES && g_axis[axis].busy; }

void Stepper_StopAll(void)
{
    __disable_irq();
    finish_move();
    all_steps_low();
    __enable_irq();
}

void Stepper_EmergencyStopFromISR(void)
{
    /* EXTI priority is higher than TIM2, so the motion state is stable here. */
    finish_move();
    all_steps_low();
}

int32_t Stepper_GetPos(uint8_t axis) { return axis < NUM_AXES ? g_axis[axis].pos : 0; }

void Stepper_SetPosition(uint8_t axis, int32_t position)
{
    if (axis >= NUM_AXES || s_move.active) return;
    g_axis[axis].pos = position;
    g_axis[axis].target = position;
}

bool Stepper_LimitStopped(void) { return s_limit_stopped; }
void Stepper_ClearLimitStopped(void) { s_limit_stopped = false; }

void Stepper_TIM_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_UIF) == 0U) return;
    TIM2->SR &= ~TIM_SR_UIF;
    s_time_us += SCHEDULER_TICK_US;
    all_steps_low();

    if (!s_move.active || (int32_t)(s_time_us - s_move.next_step_us) < 0) return;

    uint32_t remaining = s_move.total_ticks - s_move.tick_index;
    if (remaining <= decel_ticks(s_move.current_sps, s_move.accel_sps2)) {
        uint32_t speed_sq = s_move.current_sps * s_move.current_sps;
        uint32_t dec = 2U * s_move.accel_sps2;
        s_move.current_sps = (dec >= speed_sq) ? STEPPER_MIN_SPEED_SPS : isqrt32(speed_sq - dec);
        if (s_move.current_sps < STEPPER_MIN_SPEED_SPS) s_move.current_sps = STEPPER_MIN_SPEED_SPS;
    } else if (s_move.current_sps < s_move.cruise_sps) {
        uint32_t speed_sq = s_move.current_sps * s_move.current_sps;
        s_move.current_sps = isqrt32(speed_sq + 2U * s_move.accel_sps2);
        if (s_move.current_sps > s_move.cruise_sps) s_move.current_sps = s_move.cruise_sps;
    }

    if (s_move.axis_steps[AXIS_Z] != 0U &&
        !Limit_ZMotionAllowed(s_move.logical_positive[AXIS_Z])) {
        s_limit_stopped = true;
        finish_move();
        return;
    }

    for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) {
        s_move.error[axis] += s_move.axis_steps[axis];
        if (s_move.error[axis] < s_move.total_ticks) continue;
        s_move.error[axis] -= s_move.total_ticks;
        s_step_port[axis]->BSRR = s_step_pin[axis];
        g_axis[axis].pos += s_move.logical_positive[axis] ? 1 : -1;
    }

    ++s_move.tick_index;
    if (s_move.tick_index >= s_move.total_ticks) {
        for (uint8_t axis = 0U; axis < NUM_AXES; ++axis) g_axis[axis].pos = g_axis[axis].target;
        finish_move();
        return;
    }
    s_move.next_step_us += interval_us(s_move.current_sps);
}
