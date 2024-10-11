/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define shijue_TX_Pin GPIO_PIN_1
#define shijue_TX_GPIO_Port GPIOE
#define shijue_RX_Pin GPIO_PIN_0
#define shijue_RX_GPIO_Port GPIOE
#define unitree_TX_Pin GPIO_PIN_14
#define unitree_TX_GPIO_Port GPIOG
#define gray_D1_Pin GPIO_PIN_5
#define gray_D1_GPIO_Port GPIOE
#define A1Motor_RE_Pin GPIO_PIN_7
#define A1Motor_RE_GPIO_Port GPIOI
#define unitree_RX_Pin GPIO_PIN_9
#define unitree_RX_GPIO_Port GPIOG
#define huadao_Pin GPIO_PIN_0
#define huadao_GPIO_Port GPIOI
#define gray_D2_Pin GPIO_PIN_1
#define gray_D2_GPIO_Port GPIOF
#define tulun_Pin GPIO_PIN_12
#define tulun_GPIO_Port GPIOH
#define bogan_Pin GPIO_PIN_11
#define bogan_GPIO_Port GPIOH
#define fuyong_TX_Pin GPIO_PIN_2
#define fuyong_TX_GPIO_Port GPIOA
#define servo_TX_Pin GPIO_PIN_8
#define servo_TX_GPIO_Port GPIOE
#define IC_RX_Pin GPIO_PIN_9
#define IC_RX_GPIO_Port GPIOD
#define IC_TX_Pin GPIO_PIN_8
#define IC_TX_GPIO_Port GPIOD
#define fuyong_RX_Pin GPIO_PIN_3
#define fuyong_RX_GPIO_Port GPIOA
#define servo_RX_Pin GPIO_PIN_7
#define servo_RX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
