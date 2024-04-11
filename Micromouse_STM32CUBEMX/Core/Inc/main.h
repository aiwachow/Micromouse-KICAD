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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT_LVL_Pin GPIO_PIN_0
#define BATT_LVL_GPIO_Port GPIOC
#define PhotoT_1_Pin GPIO_PIN_1
#define PhotoT_1_GPIO_Port GPIOC
#define PhotoT_2_Pin GPIO_PIN_2
#define PhotoT_2_GPIO_Port GPIOC
#define PhotoT_3_Pin GPIO_PIN_3
#define PhotoT_3_GPIO_Port GPIOC
#define PhotoT_6_Pin GPIO_PIN_0
#define PhotoT_6_GPIO_Port GPIOA
#define PhotoT_5_Pin GPIO_PIN_1
#define PhotoT_5_GPIO_Port GPIOA
#define PhotoT_4_Pin GPIO_PIN_2
#define PhotoT_4_GPIO_Port GPIOA
#define MotorR_PWM_Pin GPIO_PIN_3
#define MotorR_PWM_GPIO_Port GPIOA
#define MotorR_AIN2_Pin GPIO_PIN_4
#define MotorR_AIN2_GPIO_Port GPIOA
#define MotorR_AIN1_Pin GPIO_PIN_5
#define MotorR_AIN1_GPIO_Port GPIOA
#define EMITTER_6_Pin GPIO_PIN_6
#define EMITTER_6_GPIO_Port GPIOA
#define EMITTER_5_Pin GPIO_PIN_7
#define EMITTER_5_GPIO_Port GPIOA
#define MotorL_BIN1_Pin GPIO_PIN_4
#define MotorL_BIN1_GPIO_Port GPIOC
#define MotorL_BIN2_Pin GPIO_PIN_5
#define MotorL_BIN2_GPIO_Port GPIOC
#define MotorL_PWM_Pin GPIO_PIN_0
#define MotorL_PWM_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_1
#define IMU_INT_GPIO_Port GPIOB
#define DRDY_Pin GPIO_PIN_2
#define DRDY_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define USER_BUTT1_Pin GPIO_PIN_15
#define USER_BUTT1_GPIO_Port GPIOB
#define USER_BUTT2_Pin GPIO_PIN_6
#define USER_BUTT2_GPIO_Port GPIOC
#define EMITTER_1_Pin GPIO_PIN_8
#define EMITTER_1_GPIO_Port GPIOC
#define EMITTER_2_Pin GPIO_PIN_9
#define EMITTER_2_GPIO_Port GPIOC
#define EMITTER_3_Pin GPIO_PIN_8
#define EMITTER_3_GPIO_Port GPIOA
#define EMITTER_4_Pin GPIO_PIN_9
#define EMITTER_4_GPIO_Port GPIOA
#define esp_spi_cs_Pin GPIO_PIN_12
#define esp_spi_cs_GPIO_Port GPIOA
#define TIM_ENCODER_B1_Pin GPIO_PIN_6
#define TIM_ENCODER_B1_GPIO_Port GPIOB
#define TIM_ENCODER_A1_Pin GPIO_PIN_7
#define TIM_ENCODER_A1_GPIO_Port GPIOB
#define TIM_ENCODER_B2_Pin GPIO_PIN_8
#define TIM_ENCODER_B2_GPIO_Port GPIOB
#define TIM_ENCODER_A2_Pin GPIO_PIN_9
#define TIM_ENCODER_A2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
