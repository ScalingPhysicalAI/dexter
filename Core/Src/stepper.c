/* stepper.c — 3-axis stepper, absolute tick scheduler, Bresenham ramp
 *
 * Axes:
 *   X: PA0 STEP  PA4 DIR   (horizontal)
 *   Y: PA1 STEP  PA5 DIR   (horizontal)
 *   Z: PB10 STEP PB11 DIR  (vertical — replaces DC actuator)
 *
 * TIM2: PSC=47 → 1 MHz tick. IRQ every 100 µs (ARR=99).
 * Each axis has s_next_step[axis] — fires independently.
 */

#include "stepper.h"

/* ── GPIO definitions ───────────────────────────────────────────────────── */
/* X and Y on GPIOA */
#define STEP_X_PIN   GPIO_PIN_0    /* PA0 */
#define STEP_Y_PIN   GPIO_PIN_1    /* PA1 */
#define DIR_X_PIN    GPIO_PIN_4    /* PA4 */
#define DIR_Y_PIN    GPIO_PIN_5    /* PA5 */

/* Z on GPIOB — replaces L298 actuator */
#define STEP_Z_PIN   GPIO_PIN_10   /* PB10 */
#define DIR_Z_PIN    GPIO_PIN_11   /* PB11 */

/* ── Timer ──────────────────────────────────────────────────────────────── */
#define TIM_PSC        47UL
#define TIMER_CLK_HZ   1000000UL
#define PULSE_TICKS    10UL         /* 10 µs STEP pulse width               */

static inline uint32_t sps_to_ticks(uint32_t sps)
{
    if (sps < STEPPER_MIN_SPEED_SPS) sps = STEPPER_MIN_SPEED_SPS;
    if (sps > 20000UL)               sps = 20000UL;
    return TIMER_CLK_HZ / sps;
}

/* ── State ──────────────────────────────────────────────────────────────── */
static TIM_HandleTypeDef *s_htim;

AxisCtrl g_axis[NUM_AXES];

static volatile uint32_t s_next_step[NUM_AXES];  /* next step time (µs)    */
static volatile uint32_t s_pulse_end[NUM_AXES];  /* pulse LOW time (µs)    */
static volatile uint8_t  s_dir_hold[NUM_AXES];   /* DIR setup hold count   */
static volatile uint32_t s_tick_base;             /* 32-bit µs counter base */

/* ── GPIO helpers — step and dir per axis ───────────────────────────────── */
static inline void step_high(uint8_t idx)
{
    if      (idx == AXIS_X) GPIOA->BSRR = STEP_X_PIN;
    else if (idx == AXIS_Y) GPIOA->BSRR = STEP_Y_PIN;
    else                    GPIOB->BSRR = STEP_Z_PIN;
}

static inline void step_low(uint8_t idx)
{
    if      (idx == AXIS_X) GPIOA->BRR = STEP_X_PIN;
    else if (idx == AXIS_Y) GPIOA->BRR = STEP_Y_PIN;
    else                    GPIOB->BRR = STEP_Z_PIN;
}

static inline void dir_set(uint8_t idx, bool positive)
{
    uint32_t pin;
    GPIO_TypeDef *port;
    if      (idx == AXIS_X) { port = GPIOA; pin = DIR_X_PIN; }
    else if (idx == AXIS_Y) { port = GPIOA; pin = DIR_Y_PIN; }
    else                    { port = GPIOB; pin = DIR_Z_PIN; }
    if (positive) port->BSRR = pin;
    else          port->BRR  = pin;
}

/* ── Math helpers ───────────────────────────────────────────────────────── */
static uint32_t isqrt32(uint32_t n)
{
    if (n == 0) return 0;
    uint32_t x = n, x1 = (n >> 1) + 1;
    while (x1 < x) { x = x1; x1 = (x + n / x) >> 1; }
    return x;
}

static int32_t decel_steps(uint32_t v, uint32_t a)
{
    if (a == 0 || v <= STEPPER_MIN_SPEED_SPS) return 0;
    uint32_t v2     = v * v;
    uint32_t v2_min = (uint32_t)STEPPER_MIN_SPEED_SPS * STEPPER_MIN_SPEED_SPS;
    if (v2 <= v2_min) return 0;
    return (int32_t)((v2 - v2_min) / (2UL * a));
}

static uint32_t ramp_up(uint32_t v, uint32_t a)
{
    uint32_t vn = isqrt32(v * v + 2UL * a);
    return (vn > 20000UL) ? 20000UL : vn;
}

static uint32_t ramp_down(uint32_t v, uint32_t a)
{
    uint32_t v2  = v * v;
    uint32_t sub = 2UL * a;
    if (sub >= v2) return STEPPER_MIN_SPEED_SPS;
    uint32_t vn = isqrt32(v2 - sub);
    return (vn < STEPPER_MIN_SPEED_SPS) ? STEPPER_MIN_SPEED_SPS : vn;
}

static inline uint32_t now_ticks(void)
{
    return s_tick_base + TIM2->CNT;
}

/* ── Per-axis step handler ───────────────────────────────────────────────── */
static void axis_do_step(uint8_t idx)
{
    AxisCtrl *a = &g_axis[idx];

    /* DIR hold guard — wait N ticks after direction change */
    if (s_dir_hold[idx] > 0) {
        s_dir_hold[idx]--;
        s_next_step[idx] = now_ticks() + sps_to_ticks(a->cur_speed_sps);
        return;
    }

    /* Raise STEP pin */
    step_high(idx);
    s_pulse_end[idx] = now_ticks() + PULSE_TICKS;

    /* Count step */
    a->steps_done++;
    a->pos += a->dir ? 1 : -1;

    /* Check completion */
    if (a->steps_done >= a->steps_total) {
        a->state         = MOTOR_IDLE;
        a->busy          = false;
        a->cur_speed_sps = STEPPER_MIN_SPEED_SPS;
        s_next_step[idx] = 0xFFFFFFFFUL;
        step_low(idx);
        return;
    }

    int32_t remaining = a->steps_total - a->steps_done;

    /* Ramp */
    switch (a->state) {
        case MOTOR_ACCEL:
            a->cur_speed_sps = ramp_up(a->cur_speed_sps, a->accel_sps2);
            if (a->cur_speed_sps >= a->cruise_speed_sps) {
                a->cur_speed_sps = a->cruise_speed_sps;
                a->state = MOTOR_CRUISE;
            }
            if (remaining <= decel_steps(a->cur_speed_sps, a->accel_sps2))
                a->state = MOTOR_DECEL;
            break;
        case MOTOR_CRUISE:
            if (remaining <= decel_steps(a->cruise_speed_sps, a->accel_sps2))
                a->state = MOTOR_DECEL;
            break;
        case MOTOR_DECEL:
            a->cur_speed_sps = ramp_down(a->cur_speed_sps, a->accel_sps2);
            break;
        default: break;
    }

    /* Schedule next step */
    s_next_step[idx] = now_ticks() + sps_to_ticks(a->cur_speed_sps);
}

/* ════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════ */

void Stepper_Init(TIM_HandleTypeDef *htim)
{
    s_htim      = htim;
    s_tick_base = 0;

    for (int i = 0; i < NUM_AXES; i++) {
        g_axis[i].max_speed_sps  = STEPPER_DEFAULT_MAX_SPEED_SPS;
        g_axis[i].accel_sps2     = STEPPER_DEFAULT_ACCEL_SPS2;
        g_axis[i].cur_speed_sps  = STEPPER_MIN_SPEED_SPS;
        g_axis[i].state          = MOTOR_IDLE;
        g_axis[i].busy           = false;
        g_axis[i].pos            = 0;
        g_axis[i].steps_done     = 0;
        g_axis[i].steps_total    = 0;
        s_next_step[i] = 0xFFFFFFFFUL;
        s_pulse_end[i] = 0;
        s_dir_hold[i]  = 0;
    }

    /* GPIOA: PA0 PA1 STEP_X/Y   PA4 PA5 DIR_X/Y */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = STEP_X_PIN | STEP_Y_PIN | DIR_X_PIN | DIR_Y_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
    GPIOA->BRR = STEP_X_PIN | STEP_Y_PIN | DIR_X_PIN | DIR_Y_PIN;

    /* GPIOB: PB10 STEP_Z   PB11 DIR_Z */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin   = STEP_Z_PIN | DIR_Z_PIN;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    GPIOB->BRR = STEP_Z_PIN | DIR_Z_PIN;

    /* TIM2: 1 MHz tick, IRQ every 100 µs */
    TIM2->CR1   = 0;
    TIM2->CR2   = 0;
    TIM2->SMCR  = 0;
    TIM2->DIER  = 0;
    TIM2->SR    = 0;
    TIM2->CCMR1 = 0;
    TIM2->CCMR2 = 0;
    TIM2->CCER  = 0;
    TIM2->PSC   = TIM_PSC;
    TIM2->ARR   = 99;
    TIM2->CNT   = 0;
    TIM2->EGR   = TIM_EGR_UG;
    TIM2->SR    = 0;
    TIM2->DIER  = TIM_DIER_UIE;
    TIM2->CR1   = TIM_CR1_CEN;
}

void Stepper_SetSpeed(uint8_t axis, uint32_t sps)
{
    if (axis >= NUM_AXES) return;
    if (sps < STEPPER_MIN_SPEED_SPS) sps = STEPPER_MIN_SPEED_SPS;
    if (sps > 20000UL)               sps = 20000UL;
    g_axis[axis].max_speed_sps = sps;
}

void Stepper_SetAccel(uint8_t axis, uint32_t sps2)
{
    if (axis >= NUM_AXES) return;
    if (sps2 < 1) sps2 = 1;
    g_axis[axis].accel_sps2 = sps2;
}

void Stepper_MoveTo(uint8_t axis, int32_t abs_pos)
{
    if (axis >= NUM_AXES) return;
    AxisCtrl *a = &g_axis[axis];

    int32_t delta = abs_pos - a->pos;
    if (delta == 0) return;

    bool dir_pos = (delta > 0);
    a->dir = dir_pos;
    dir_set(axis, dir_pos);

    int32_t  steps  = (delta > 0) ? delta : -delta;
    int32_t  dsteps = decel_steps(a->max_speed_sps, a->accel_sps2);
    uint32_t cruise;
    if (steps < 2 * dsteps) {
        cruise = isqrt32(a->accel_sps2 * (uint32_t)steps);
        if (cruise < STEPPER_MIN_SPEED_SPS) cruise = STEPPER_MIN_SPEED_SPS;
        if (cruise > a->max_speed_sps)      cruise = a->max_speed_sps;
    } else {
        cruise = a->max_speed_sps;
    }

    __disable_irq();
    a->target           = abs_pos;
    a->steps_total      = steps;
    a->steps_done       = 0;
    a->cruise_speed_sps = cruise;
    a->cur_speed_sps    = STEPPER_MIN_SPEED_SPS;
    a->state            = MOTOR_ACCEL;
    a->busy             = true;
    s_dir_hold[axis]    = 4;
    s_next_step[axis]   = now_ticks() + sps_to_ticks(STEPPER_MIN_SPEED_SPS);
    __enable_irq();
}

void Stepper_MoveRel(uint8_t axis, int32_t delta)
{
    if (axis >= NUM_AXES) return;
    Stepper_MoveTo(axis, g_axis[axis].pos + delta);
}

bool Stepper_IsBusy(void)
{
    for (int i = 0; i < NUM_AXES; i++)
        if (g_axis[i].busy) return true;
    return false;
}

bool Stepper_IsAxisBusy(uint8_t axis)
{
    return (axis < NUM_AXES) && g_axis[axis].busy;
}

void Stepper_StopAll(void)
{
    __disable_irq();
    for (int i = 0; i < NUM_AXES; i++) {
        g_axis[i].state         = MOTOR_IDLE;
        g_axis[i].busy          = false;
        g_axis[i].cur_speed_sps = STEPPER_MIN_SPEED_SPS;
        s_next_step[i]          = 0xFFFFFFFFUL;
        s_pulse_end[i]          = 0;
        s_dir_hold[i]           = 0;
    }
    GPIOA->BRR = STEP_X_PIN | STEP_Y_PIN;
    GPIOB->BRR = STEP_Z_PIN;
    __enable_irq();
}

int32_t Stepper_GetPos(uint8_t axis)
{
    return (axis < NUM_AXES) ? g_axis[axis].pos : 0;
}

void Stepper_SetZero(uint8_t axis)
{
    if (axis >= NUM_AXES) return;
    __disable_irq();
    g_axis[axis].pos = 0;
    __enable_irq();
}

/* ── TIM2 Update ISR — fires every 100 µs ───────────────────────────────── */
void Stepper_TIM_IRQHandler(void)
{
    if (!(TIM2->SR & TIM_SR_UIF)) return;
    TIM2->SR = ~TIM_SR_UIF;

    s_tick_base += 100UL;
    uint32_t now = s_tick_base + TIM2->CNT;

    /* Phase 1: lower STEP pins whose pulse time has elapsed */
    for (int i = 0; i < NUM_AXES; i++) {
        if (s_pulse_end[i] != 0 && now >= s_pulse_end[i]) {
            step_low(i);
            s_pulse_end[i] = 0;
        }
    }

    /* Phase 2: fire steps whose scheduled time has arrived */
    for (int i = 0; i < NUM_AXES; i++) {
        if (g_axis[i].busy && now >= s_next_step[i])
            axis_do_step(i);
    }
}
