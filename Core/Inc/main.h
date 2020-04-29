/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPINDLE_ENABLE_BIT_Pin GPIO_PIN_0
#define SPINDLE_ENABLE_BIT_GPIO_Port GPIOF
#define SPINDLE_DIRECTION_BIT_Pin GPIO_PIN_1
#define SPINDLE_DIRECTION_BIT_GPIO_Port GPIOF
#define PROBE_BIT_Pin GPIO_PIN_10
#define PROBE_BIT_GPIO_Port GPIOG
#define STEPPERS_DISABLE_BIT_Pin GPIO_PIN_0
#define STEPPERS_DISABLE_BIT_GPIO_Port GPIOA
#define SPINDLE_PWM_BIT_Pin GPIO_PIN_1
#define SPINDLE_PWM_BIT_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define X_STEP_BIT_Pin GPIO_PIN_4
#define X_STEP_BIT_GPIO_Port GPIOA
#define Y_STEP_BIT_Pin GPIO_PIN_5
#define Y_STEP_BIT_GPIO_Port GPIOA
#define Z_STEP_BIT_Pin GPIO_PIN_6
#define Z_STEP_BIT_GPIO_Port GPIOA
#define X_DIRECTION_BIT_Pin GPIO_PIN_7
#define X_DIRECTION_BIT_GPIO_Port GPIOA
#define Y_DIRECTION_BIT_Pin GPIO_PIN_8
#define Y_DIRECTION_BIT_GPIO_Port GPIOA
#define Z_DIRECTION_BIT_Pin GPIO_PIN_9
#define Z_DIRECTION_BIT_GPIO_Port GPIOA
#define CONTROL_RESET_BIT_Pin GPIO_PIN_10
#define CONTROL_RESET_BIT_GPIO_Port GPIOA
#define CONTROL_RESET_BIT_EXTI_IRQn EXTI15_10_IRQn
#define CONTROL_FEED_HOLD_BIT_Pin GPIO_PIN_11
#define CONTROL_FEED_HOLD_BIT_GPIO_Port GPIOA
#define CONTROL_FEED_HOLD_BIT_EXTI_IRQn EXTI15_10_IRQn
#define CONTROL_CYCLE_START_BIT_Pin GPIO_PIN_12
#define CONTROL_CYCLE_START_BIT_GPIO_Port GPIOA
#define CONTROL_CYCLE_START_BIT_EXTI_IRQn EXTI15_10_IRQn
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define CONTROL_SAFETY_DOOR_BIT_Pin GPIO_PIN_15
#define CONTROL_SAFETY_DOOR_BIT_GPIO_Port GPIOA
#define CONTROL_SAFETY_DOOR_BIT_EXTI_IRQn EXTI15_10_IRQn
#define COOLANT_MIST_BIT_Pin GPIO_PIN_3
#define COOLANT_MIST_BIT_GPIO_Port GPIOB
#define Z_LIMIT_BIT_Pin GPIO_PIN_4
#define Z_LIMIT_BIT_GPIO_Port GPIOB
#define X_LIMIT_BIT_Pin GPIO_PIN_5
#define X_LIMIT_BIT_GPIO_Port GPIOB
#define Y_LIMIT_BIT_Pin GPIO_PIN_6
#define Y_LIMIT_BIT_GPIO_Port GPIOB
#define COOLANT_FLOOD_BIT_Pin GPIO_PIN_7
#define COOLANT_FLOOD_BIT_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
typedef uint8_t bool;
extern	TIM_HandleTypeDef htim2;
extern	TIM_HandleTypeDef htim6;
extern	TIM_HandleTypeDef htim7;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
