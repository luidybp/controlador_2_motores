/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define debug_Pin GPIO_PIN_13
#define debug_GPIO_Port GPIOC
#define shunt_Pin GPIO_PIN_4
#define shunt_GPIO_Port GPIOA
#define pot1_Pin GPIO_PIN_5
#define pot1_GPIO_Port GPIOA
#define pot2_Pin GPIO_PIN_6
#define pot2_GPIO_Port GPIOA
#define emf1_Pin GPIO_PIN_7
#define emf1_GPIO_Port GPIOA
#define emf2_Pin GPIO_PIN_0
#define emf2_GPIO_Port GPIOB
#define bot_Pin GPIO_PIN_15
#define bot_GPIO_Port GPIOB
#define pwm1_Pin GPIO_PIN_8
#define pwm1_GPIO_Port GPIOA
#define pwm2_Pin GPIO_PIN_9
#define pwm2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
