/* actuator.c — Linear DC actuator via L298 on PB10/PB11
 *
 *   PB10 → L298 IN1   PB11 → L298 IN2
 *
 *   EXTEND:   IN1=1 IN2=0
 *   RETRACT:  IN1=0 IN2=1
 *   STOP:     IN1=0 IN2=0
 */

#include "actuator.h"

#define ACT_PORT    GPIOB
#define ACT_IN1_PIN GPIO_PIN_10   /* extend  */
#define ACT_IN2_PIN GPIO_PIN_11   /* retract */

static volatile ActuatorDir s_dir       = ACT_STOP;
static volatile uint32_t    s_stop_at   = 0;        /* HAL_GetTick() target */
static volatile bool        s_timed     = false;    /* false = run forever  */

/* ── GPIO drive ─────────────────────────────────────────────────────────── */
static void drive(ActuatorDir dir)
{
    s_dir = dir;
    switch (dir) {
        case ACT_EXTEND:
            ACT_PORT->BSRR = ACT_IN1_PIN;           /* IN1 HIGH */
            ACT_PORT->BRR  = ACT_IN2_PIN;           /* IN2 LOW  */
            break;
        case ACT_RETRACT:
            ACT_PORT->BRR  = ACT_IN1_PIN;           /* IN1 LOW  */
            ACT_PORT->BSRR = ACT_IN2_PIN;           /* IN2 HIGH */
            break;
        case ACT_STOP:
        default:
            ACT_PORT->BRR  = ACT_IN1_PIN | ACT_IN2_PIN;  /* both LOW */
            break;
    }
}

/* ════════════════════════════════════════════════════════════════════════ */

void Actuator_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = ACT_IN1_PIN | ACT_IN2_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;   /* DC motor — no need for high speed */
    HAL_GPIO_Init(ACT_PORT, &g);

    /* Start stopped */
    ACT_PORT->BRR = ACT_IN1_PIN | ACT_IN2_PIN;
    s_dir   = ACT_STOP;
    s_timed = false;
}

void Actuator_Run(ActuatorDir dir, uint32_t duration_ms)
{
    if (dir == ACT_STOP) { Actuator_Stop(); return; }
    if (duration_ms > 0) {
        s_stop_at = HAL_GetTick() + duration_ms;
        s_timed   = true;
    } else {
        s_timed = false;      /* run until explicit stop */
    }
    drive(dir);
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

void Actuator_Poll(void)
{
    /* Auto-stop when timed run expires */
    if (s_timed && s_dir != ACT_STOP) {
        if (HAL_GetTick() >= s_stop_at) {
            s_timed = false;
            drive(ACT_STOP);
        }
    }
}
