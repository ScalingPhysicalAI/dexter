/* main.c — HumanoidBase stepper controller
 *
 * TIM2 is initialised as a plain up-counter (no PWM).
 * stepper.c reconfigures it at runtime and drives STEP via GPIO.
 *
 * Pin map:
 *   PA0  STEP_X  (GPIO output — driven by stepper ISR)
 *   PA1  STEP_Y  (GPIO output — driven by stepper ISR)
 *   PA4  DIR_X   (GPIO output)
 *   PA5  DIR_Y   (GPIO output)
 *   PA11/PA12  USB D-/D+
 */
#include "main.h"
#include "usb_device.h"
#include "stepper.h"
#include "gcode.h"

TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USB_DEVICE_Init();

    HAL_Delay(500);         /* USB enumeration settle */
    Stepper_Init(&htim2);   /* reconfigures TIM2 as plain counter + GPIO */
    GCode_Init();

    /* ── Startup auto-move: X axis runs 200 steps forward then back ── */
    Stepper_SetSpeed(AXIS_X, 500);          /* 500 steps/s              */
    Stepper_SetAccel(AXIS_X, 800);          /* 800 steps/s²             */
    Stepper_MoveTo(AXIS_X, 200);            /* move to position 200     */
    while (Stepper_IsAxisBusy(AXIS_X)) {}  /* wait for move to finish  */
    Stepper_MoveTo(AXIS_X, 0);              /* return to home           */
    while (Stepper_IsAxisBusy(AXIS_X)) {}  /* wait                     */
    GCode_Send("ok X homing done\r\n");    /* notify host              */

    while (1) {
        GCode_Poll();
        /* No HAL_Delay here — GCode_Poll must run as fast as possible
         * to drain USB RX/TX rings without latency */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL         = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

/* TIM2 minimal init — stepper.c takes over in Stepper_Init() */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFF;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    /* Just init the handle structure — Stepper_Init() configures registers */
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
}

/* GPIO: DIR pins only. STEP pins configured by stepper.c as plain outputs */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* HAL_TIM_MspPostInit — called by HAL_TIM_PWM_Init, not used anymore.
 * Keep stub to satisfy linker (main.h declares it). */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    (void)htim;
    /* STEP pins now configured as plain GPIO in Stepper_Init() */
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
