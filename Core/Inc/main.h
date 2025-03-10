/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define I_D0_Pin GPIO_PIN_0
#define I_D0_GPIO_Port GPIOA
#define I_D1_Pin GPIO_PIN_1
#define I_D1_GPIO_Port GPIOA
#define I_D2_Pin GPIO_PIN_2
#define I_D2_GPIO_Port GPIOA
#define I_D3_Pin GPIO_PIN_3
#define I_D3_GPIO_Port GPIOA
#define I_D4_Pin GPIO_PIN_4
#define I_D4_GPIO_Port GPIOA
#define I_D5_Pin GPIO_PIN_5
#define I_D5_GPIO_Port GPIOA
#define I_D6_Pin GPIO_PIN_6
#define I_D6_GPIO_Port GPIOA
#define I_D7_Pin GPIO_PIN_7
#define I_D7_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

void user_code();

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
