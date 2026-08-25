/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Core/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_TIM2_Init(void);
void MX_FDCAN1_Init(void);
void MX_LPUART1_UART_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
/* Set to 0 to compile out physical E-stop button handling. */
#ifndef ESTOP_BUTTON_ENABLE
#define ESTOP_BUTTON_ENABLE 1U
#endif

/* 1: button shorts PC2 to GND when pressed; 0: active-high input. */
#ifndef ESTOP_BUTTON_ACTIVE_LOW
#define ESTOP_BUTTON_ACTIVE_LOW 1U
#endif

#define X_STEP_Pin GPIO_PIN_0
#define X_STEP_GPIO_Port GPIOA
#define Y_STEP_Pin GPIO_PIN_1
#define Y_STEP_GPIO_Port GPIOA
#define X_DIR_Pin GPIO_PIN_4
#define X_DIR_GPIO_Port GPIOA
#define Y_DIR_Pin GPIO_PIN_5
#define Y_DIR_GPIO_Port GPIOA
#define Z_STEP_Pin GPIO_PIN_10
#define Z_STEP_GPIO_Port GPIOB
#define Z_DIR_Pin GPIO_PIN_11
#define Z_DIR_GPIO_Port GPIOB
#define Z_LIMIT_MIN_Pin GPIO_PIN_0
#define Z_LIMIT_MIN_GPIO_Port GPIOC
#define Z_LIMIT_MAX_Pin GPIO_PIN_1
#define Z_LIMIT_MAX_GPIO_Port GPIOC
#define ESTOP_Pin GPIO_PIN_2
#define ESTOP_GPIO_Port GPIOC
#define LINEAR_ACT_EN_Pin GPIO_PIN_6
#define LINEAR_ACT_EN_GPIO_Port GPIOC
#define LINEAR_ACT_IN1_Pin GPIO_PIN_7
#define LINEAR_ACT_IN1_GPIO_Port GPIOC
#define LINEAR_ACT_IN2_Pin GPIO_PIN_8
#define LINEAR_ACT_IN2_GPIO_Port GPIOC

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
