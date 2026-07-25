/* actuator.c — Linear DC actuator via L298 with PWM speed control
 *
 * PB10 → IN1  PB11 → IN2  PB0 → ENA (TIM3 CH3 PWM)
 *
 * TIM3 config:
 *   Clock:      48 MHz / PSC=47 = 1 MHz
 *   Period:     ARR = 99  → 10 kHz PWM (smooth for DC motor)
 *   Duty cycle: CCR3 = 0..99  (0% .. 99% ≈ 100%)
 *   speed_pct:  0-100 → CCR3 = speed_pct * 99 / 100
 */

#include "actuator.h"

/* ── pin definitions ────────────────────────────────────────────────────── */
#define ACT_DIR_PORT  GPIOB
#define ACT_IN1_PIN   GPIO_PIN_10   /* PB10 — extend  */
#define ACT_IN2_PIN   GPIO_PIN_11   /* PB11 — retract */
#define ACT_ENA_PORT  GPIOB
#define ACT_ENA_PIN   GPIO_PIN_0    /* PB0  — TIM3 CH3 PWM speed */

/* ── PWM constants ──────────────────────────────────────────────────────── */
#define PWM_ARR       99U           /* 10 kHz at 1 MHz timer clock */
#define PCT_TO_CCR(p) ((uint32_t)(p) * PWM_ARR / 100U)

/* ── state ──────────────────────────────────────────────────────────────── */
static volatile ActuatorDir s_dir       = ACT_STOP;
static volatile uint8_t     s_speed_pct = 100;
static volatile uint32_t    s_stop_at   = 0;
static volatile bool        s_timed     = false;

/* ── internal helpers ───────────────────────────────────────────────────── */
static void set_pwm(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_speed_pct = pct;
    TIM3->CCR3  = PCT_TO_CCR(pct);
}

static void drive(ActuatorDir dir)
{
    s_dir = dir;
    switch (dir) {
        case ACT_EXTEND:
            ACT_DIR_PORT->BSRR = ACT_IN1_PIN;           /* IN1 HIGH */
            ACT_DIR_PORT->BRR  = ACT_IN2_PIN;           /* IN2 LOW  */
            break;
        case ACT_RETRACT:
            ACT_DIR_PORT->BRR  = ACT_IN1_PIN;           /* IN1 LOW  */
            ACT_DIR_PORT->BSRR = ACT_IN2_PIN;           /* IN2 HIGH */
            break;
        case ACT_STOP:
        default:
            ACT_DIR_PORT->BRR = ACT_IN1_PIN | ACT_IN2_PIN;
            set_pwm(0);                                  /* ENA=0 kills drive */
            break;
    }
}

/* ════════════════════════════════════════════════════════════════════════ */

void Actuator_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* PB10, PB11 — direction GPIO outputs */
    GPIO_InitTypeDef g = {0};
    g.Pin   = ACT_IN1_PIN | ACT_IN2_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ACT_DIR_PORT, &g);
    ACT_DIR_PORT->BRR = ACT_IN1_PIN | ACT_IN2_PIN;

    /* PB0 — TIM3 CH3 alternate function PWM output */
    g.Pin   = ACT_ENA_PIN;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ACT_ENA_PORT, &g);

    /* TIM3 — 10 kHz PWM on CH3
     * PSC=47 → 1 MHz tick  ARR=99 → 10 kHz period */
    TIM3->CR1   = 0;
    TIM3->PSC   = 47;
    TIM3->ARR   = PWM_ARR;
    TIM3->CNT   = 0;
    /* CH3 in PWM mode 1 */
    TIM3->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM3->CCER  = TIM_CCER_CC3E;           /* enable CH3 output */
    TIM3->CCR3  = 0;                        /* 0% duty = off     */
    TIM3->EGR   = TIM_EGR_UG;              /* load registers    */
    TIM3->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

    s_dir       = ACT_STOP;
    s_speed_pct = 100;
    s_timed     = false;
}

void Actuator_Run(ActuatorDir dir, uint32_t duration_ms, uint8_t speed_pct)
{
    if (dir == ACT_STOP) { Actuator_Stop(); return; }
    set_pwm(speed_pct);
    if (duration_ms > 0) {
        s_stop_at = HAL_GetTick() + duration_ms;
        s_timed   = true;
    } else {
        s_timed = false;
    }
    drive(dir);
}

void Actuator_SetSpeed(uint8_t speed_pct)
{
    set_pwm(speed_pct);
    /* If stopped, don't restart — speed takes effect on next Run() */
}

void Actuator_Stop(void)
{
    s_timed = false;
    drive(ACT_STOP);
}

bool Actuator_IsBusy(void)
{
    return (s_dir != ACT_STOP);
}

ActuatorDir Actuator_GetDir(void)
{
    return s_dir;
}

uint8_t Actuator_GetSpeed(void)
{
    return s_speed_pct;
}

void Actuator_Poll(void)
{
    if (s_timed && s_dir != ACT_STOP && HAL_GetTick() >= s_stop_at) {
        s_timed = false;
        drive(ACT_STOP);
    }
}
