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
#include "stm32h7xx_hal.h"

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
#define Barometer_MISO_Pin GPIO_PIN_9
#define Barometer_MISO_GPIO_Port GPIOG
#define RF_Rx_Pin GPIO_PIN_5
#define RF_Rx_GPIO_Port GPIOD
#define Barometer_SCK_Pin GPIO_PIN_11
#define Barometer_SCK_GPIO_Port GPIOG
#define RF_Tx_Pin GPIO_PIN_6
#define RF_Tx_GPIO_Port GPIOD
#define BNO085_SCK_Pin GPIO_PIN_3
#define BNO085_SCK_GPIO_Port GPIOD
#define GDK101_Tx_Pin GPIO_PIN_7
#define GDK101_Tx_GPIO_Port GPIOB
#define Barometer_MOSI_Pin GPIO_PIN_7
#define Barometer_MOSI_GPIO_Port GPIOD
#define RTC_SDA_Pin GPIO_PIN_9
#define RTC_SDA_GPIO_Port GPIOB
#define RTC_SCL_Pin GPIO_PIN_8
#define RTC_SCL_GPIO_Port GPIOB
#define GDK101_Rx_Pin GPIO_PIN_9
#define GDK101_Rx_GPIO_Port GPIOA
#define BNO085_MOSI_Pin GPIO_PIN_1
#define BNO085_MOSI_GPIO_Port GPIOC
#define BNO085_MISO_Pin GPIO_PIN_2
#define BNO085_MISO_GPIO_Port GPIOC
#define EN_Pin GPIO_PIN_2
#define EN_GPIO_Port GPIOA
#define Debug_LD1_Pin GPIO_PIN_1
#define Debug_LD1_GPIO_Port GPIOA
#define Debug_LD0_Pin GPIO_PIN_0
#define Debug_LD0_GPIO_Port GPIOA
#define RF_RST_Pin GPIO_PIN_0
#define RF_RST_GPIO_Port GPIOJ
#define AS6221_SCL_Pin GPIO_PIN_4
#define AS6221_SCL_GPIO_Port GPIOH
#define AS6221_SDA_Pin GPIO_PIN_5
#define AS6221_SDA_GPIO_Port GPIOH
#define RF_M0_Pin GPIO_PIN_1
#define RF_M0_GPIO_Port GPIOJ
#define SBD_Rx_Pin GPIO_PIN_10
#define SBD_Rx_GPIO_Port GPIOB
#define SBD_Tx_Pin GPIO_PIN_11
#define SBD_Tx_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_6
#define D6_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOA
#define SDRAM_CS_A0_Pin GPIO_PIN_12
#define SDRAM_CS_A0_GPIO_Port GPIOD
#define SDRAM_CS_A1_Pin GPIO_PIN_13
#define SDRAM_CS_A1_GPIO_Port GPIOD
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOA
#define RF_M1_Pin GPIO_PIN_2
#define RF_M1_GPIO_Port GPIOJ
#define Buzzer_Pin GPIO_PIN_6
#define Buzzer_GPIO_Port GPIOH
#define GPS_Tx_Pin GPIO_PIN_12
#define GPS_Tx_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_3
#define RS_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOA
#define GPS_Rx_Pin GPIO_PIN_13
#define GPS_Rx_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
