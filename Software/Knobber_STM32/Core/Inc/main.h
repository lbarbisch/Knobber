/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define ADC_I_U_Pin GPIO_PIN_0
#define ADC_I_U_GPIO_Port GPIOA
#define ADC_I_V_Pin GPIO_PIN_1
#define ADC_I_V_GPIO_Port GPIOA
#define ADC_I_W_Pin GPIO_PIN_2
#define ADC_I_W_GPIO_Port GPIOA
#define ADC_VIN_Pin GPIO_PIN_3
#define ADC_VIN_GPIO_Port GPIOA
#define ADC_Angle_Pin GPIO_PIN_4
#define ADC_Angle_GPIO_Port GPIOA
#define nSLEEP_Pin GPIO_PIN_0
#define nSLEEP_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_1
#define ENABLE_GPIO_Port GPIOB
#define Motor_W_Pin GPIO_PIN_8
#define Motor_W_GPIO_Port GPIOA
#define Motor_V_Pin GPIO_PIN_9
#define Motor_V_GPIO_Port GPIOA
#define Motor_U_Pin GPIO_PIN_10
#define Motor_U_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_15
#define nFAULT_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB
#define Button_Pin GPIO_PIN_3
#define Button_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
