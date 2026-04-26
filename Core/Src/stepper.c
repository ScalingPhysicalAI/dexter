/* stepper.c — 2-axis simultaneous stepper, Bresenham trapezoidal ramp
 *
 * Architecture: TIM2 runs as a fixed 1 MHz timebase (PSC=47).
 * Each axis tracks its own "next_step_tick" — the timer count value at
 * which the next step should fire. The ISR compares current CNT against
 * each axis's next_step_tick independently. This gives truly independent
 * step rates on one timer with no divider approximation errors.
 *
 * Step pulse: STEP pin goes HIGH when the tick fires, LOW after
 * PULSE_WIDTH_TICKS (10 µs). A separate falling-edge timer per axis
 * handles the low transition without a second ISR.
 *
 * Pins: PA0 STEP_X  PA1 STEP_Y  PA4 DIR_X  PA5 DIR_Y  (plain GPIO)
 */

#include "stepper.h"

/* ── GPIO ───────────────────────────────────────────────────────────────── */
#define STEP_X_PIN  GPIO_PIN_0
#define STEP_Y_PIN  GPIO_PIN_1
#define DIR_X_PIN   GPIO_PIN_4
#define DIR_Y_PIN   GPIO_PIN_5

/* ── Timer ──────────────────────────────────────────────────────────────── */
/* PSC=47 → 48 MHz / 48 = 1 MHz (1 µs per tick), 16-bit counter 0..65535  */
#define TIM_PSC         47UL
#define TIMER_CLK_HZ    1000000UL
#define TIM_MAX         65535UL
#define PULSE_TICKS     10UL    /* STEP pulse width: 10 µs                  */

/* ticks per step at a given speed (period = 1/sps seconds = 1e6/sps µs)  */
static inline uint32_t sps_to_ticks(uint32_t sps)
{
    if (sps < STEPPER_MIN_SPEED_SPS) sps = STEPPER_MIN_SPEED_SPS;
    if (sps > 20000UL)               sps = 20000UL;
    return TIMER_CLK_HZ / sps;   /* ticks per full step period              */
}

#define TICKS_IDLE  sps_to_ticks(10UL)  /* slow idle period                */

/* ── axis state ─────────────────────────────────────────────────────────── */
AxisCtrl g_axis[NUM_AXES];
static TIM_HandleTypeDef *s_htim;

/* Next step fire time (in timer ticks from last overflow anchor).
 * Compared against TIM2->CNT each ISR. */
static volatile uint32_t s_next_step[NUM_AXES];

/* Falling-edge countdown: ticks remaining until STEP goes LOW */
static volatile uint32_t s_pulse_end[NUM_AXES];

/* DIR hold: skip N step opportunities after direction change */
static volatile uint8_t  s_dir_hold[NUM_AXES];

/* Running tick anchor — add to handle 16-bit counter wrap */
static volatile uint32_t s_tick_base;

/* ── math helpers ───────────────────────────────────────────────────────── */
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
    if (vn > 20000UL) vn = 20000UL;
    return vn;
}

static uint32_t ramp_down(uint32_t v, uint32_t a)
{
    uint32_t v2  = v * v;
    uint32_t sub = 2UL * a;
    if (sub >= v2) return STEPPER_MIN_SPEED_SPS;
    uint32_t vn = isqrt32(v2 - sub);
    return (vn < STEPPER_MIN_SPEED_SPS) ? STEPPER_MIN_SPEED_SPS : vn;
}

/* ── current absolute tick (32-bit, wraps handled) ──────────────────────── */
static inline uint32_t now_ticks(void)
{
    return s_tick_base + TIM2->CNT;
}

/* ── per-axis step handler ───────────────────────────────────────────────── */
static void axis_do_step(uint8_t idx)
{
    AxisCtrl *a = &g_axis[idx];

    /* DIR hold guard */
    if (s_dir_hold[idx] > 0) {
        s_dir_hold[idx]--;
        s_next_step[idx] = now_ticks() + sps_to_ticks(a->cur_speed_sps);
        return;
    }

    /* Raise STEP pin */
    GPIOA->BSRR = (idx == AXIS_X) ? STEP_X_PIN : STEP_Y_PIN;
    s_pulse_end[idx] = now_ticks() + PULSE_TICKS;

    /* Count step */
    a->steps_done++;
    a->pos += a->dir ? 1 : -1;

    /* Check completion */
    if (a->steps_done >= a->steps_total) {
        a->state         = MOTOR_IDLE;
        a->busy          = false;
        a->cur_speed_sps = STEPPER_MIN_SPEED_SPS;
        s_next_step[idx] = 0xFFFFFFFFUL;  /* don't fire again */
        return;
    }

    int32_t remaining = a->steps_total - a->steps_done;

    /* Ramp: v² = v0² ± 2a */
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
    s_htim     = htim;
    s_tick_base = 0;

    for (int i = 0; i < NUM_AXES; i++) {
        g_axis[i].max_speed_sps = STEPPER_DEFAULT_MAX_SPEED_SPS;
        g_axis[i].accel_sps2    = STEPPER_DEFAULT_ACCEL_SPS2;
        g_axis[i].cur_speed_sps = STEPPER_MIN_SPEED_SPS;
        g_axis[i].state         = MOTOR_IDLE;
        g_axis[i].busy          = false;
        g_axis[i].pos           = 0;
        g_axis[i].steps_done    = 0;
        g_axis[i].steps_total   = 0;
        s_next_step[i] = 0xFFFFFFFFUL;
        s_pulse_end[i] = 0;
        s_dir_hold[i]  = 0;
    }

    /* PA0/PA1 STEP, PA4/PA5 DIR — plain GPIO outputs */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = STEP_X_PIN | STEP_Y_PIN | DIR_X_PIN | DIR_Y_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
    GPIOA->BRR = STEP_X_PIN | STEP_Y_PIN | DIR_X_PIN | DIR_Y_PIN;

    /* TIM2: plain 1 MHz up-counter, Update IRQ every 100 µs (ARR=99)
     * Fine enough to fire steps accurately; ISR overhead ~1 µs         */
    TIM2->CR1   = 0;
    TIM2->CR2   = 0;
    TIM2->SMCR  = 0;
    TIM2->DIER  = 0;
    TIM2->SR    = 0;
    TIM2->CCMR1 = 0;
    TIM2->CCMR2 = 0;
    TIM2->CCER  = 0;
    TIM2->PSC   = TIM_PSC;
    TIM2->ARR   = 99;           /* IRQ every 100 µs = 10 kHz poll rate     */
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

    /* Set DIR pin */
    if (axis == AXIS_X) {
        if (dir_pos) GPIOA->BSRR = DIR_X_PIN;
        else         GPIOA->BRR  = DIR_X_PIN;
    } else {
        if (dir_pos) GPIOA->BSRR = DIR_Y_PIN;
        else         GPIOA->BRR  = DIR_Y_PIN;
    }

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
    s_dir_hold[axis]    = 4;    /* 4 ISR polls = 400 µs DIR setup           */
    /* Schedule first step immediately */
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
/*
 * Each ISR tick:
 * 1. Advance the 32-bit tick counter (handles 16-bit CNT wrap).
 * 2. For each axis: pull STEP low if pulse time expired.
 * 3. For each axis: fire a step if now >= s_next_step[axis].
 *
 * No shared divider — each axis manages its own schedule independently.
 * Both axes can fire in the same ISR if their times coincide.
 */
void Stepper_TIM_IRQHandler(void)
{
    if (!(TIM2->SR & TIM_SR_UIF)) return;
    TIM2->SR = ~TIM_SR_UIF;

    /* Advance 32-bit tick base on every counter overflow (ARR=99 → every 100 µs) */
    s_tick_base += 100UL;   /* ARR+1 ticks per IRQ */
    uint32_t now = s_tick_base + TIM2->CNT;

    /* ── Phase 1: end STEP pulses whose time has elapsed ── */
    for (int i = 0; i < NUM_AXES; i++) {
        if (s_pulse_end[i] != 0 && now >= s_pulse_end[i]) {
            GPIOA->BRR  = (i == AXIS_X) ? STEP_X_PIN : STEP_Y_PIN;
            s_pulse_end[i] = 0;
        }
    }

    /* ── Phase 2: fire steps whose scheduled time has arrived ── */
    for (int i = 0; i < NUM_AXES; i++) {
        if (!g_axis[i].busy) continue;
        if (now >= s_next_step[i]) {
            axis_do_step(i);
        }
    }
}
