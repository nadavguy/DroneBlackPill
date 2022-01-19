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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "ControlAgent.h"
#include "AHRS.h"
#include "IMUAgent.h"
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
#define TBS_TX2_Pin GPIO_PIN_2
#define TBS_TX2_GPIO_Port GPIOA
#define TBS_RX2_Pin GPIO_PIN_3
#define TBS_RX2_GPIO_Port GPIOA
#define IMU_SCK1_Pin GPIO_PIN_5
#define IMU_SCK1_GPIO_Port GPIOA
#define IMU_MISO1_Pin GPIO_PIN_6
#define IMU_MISO1_GPIO_Port GPIOA
#define IMU_MOSI1_Pin GPIO_PIN_7
#define IMU_MOSI1_GPIO_Port GPIOA
#define LF_Motor_CH1_Pin GPIO_PIN_8
#define LF_Motor_CH1_GPIO_Port GPIOA
#define RF_Motor_CH2_Pin GPIO_PIN_9
#define RF_Motor_CH2_GPIO_Port GPIOA
#define LB_Motor_CH3_Pin GPIO_PIN_10
#define LB_Motor_CH3_GPIO_Port GPIOA
#define RB_Motor_CH4_Pin GPIO_PIN_11
#define RB_Motor_CH4_GPIO_Port GPIOA
#define GPS_TX1_Pin GPIO_PIN_15
#define GPS_TX1_GPIO_Port GPIOA
#define GPS_RX1_Pin GPIO_PIN_3
#define GPS_RX1_GPIO_Port GPIOB
#define BARO_SCL1_Pin GPIO_PIN_6
#define BARO_SCL1_GPIO_Port GPIOB
#define BARO_SDA1_Pin GPIO_PIN_7
#define BARO_SDA1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
