/* stm32f1xx_hal_msp.c */
#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    /* Disable JTAG, keep SWD */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

/* HAL_TIM_Base_Init calls this — we enable TIM2 clock + NVIC here */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* TIM2 must be HIGHER priority than USB so step ISR always fires.
         * Cortex-M3: lower number = higher priority.
         * TIM2=0 (highest), USB=1, SysTick=15 (lowest preempt) */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
    }
}
