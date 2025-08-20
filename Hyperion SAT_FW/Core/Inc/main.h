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
#define MMC5983MA_INT_Pin GPIO_PIN_3
#define MMC5983MA_INT_GPIO_Port GPIOE
#define SPI4_CS_Pin GPIO_PIN_4
#define SPI4_CS_GPIO_Port GPIOE
#define PMIC_PG_Pin GPIO_PIN_13
#define PMIC_PG_GPIO_Port GPIOC
#define PMIC_PGC14_Pin GPIO_PIN_14
#define PMIC_PGC14_GPIO_Port GPIOC
#define IGN_BAT_ADC_Pin GPIO_PIN_1
#define IGN_BAT_ADC_GPIO_Port GPIOC
#define Board_TEMP_INT_Pin GPIO_PIN_2
#define Board_TEMP_INT_GPIO_Port GPIOC
#define PMIC_TEMP_INT_Pin GPIO_PIN_3
#define PMIC_TEMP_INT_GPIO_Port GPIOC
#define CDS_Top_Pin GPIO_PIN_0
#define CDS_Top_GPIO_Port GPIOA
#define CDS_Bottom_Pin GPIO_PIN_1
#define CDS_Bottom_GPIO_Port GPIOA
#define CR2032_ADC_Pin GPIO_PIN_2
#define CR2032_ADC_GPIO_Port GPIOA
#define ICM_45686_INT_Pin GPIO_PIN_3
#define ICM_45686_INT_GPIO_Port GPIOA
#define ICM_45686_INT_EXTI_IRQn EXTI3_IRQn
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define RF_TEMP_INT_Pin GPIO_PIN_4
#define RF_TEMP_INT_GPIO_Port GPIOC
#define IGN_FET_TEMP_INT_Pin GPIO_PIN_5
#define IGN_FET_TEMP_INT_GPIO_Port GPIOC
#define Main_Cell2_ADC_Pin GPIO_PIN_0
#define Main_Cell2_ADC_GPIO_Port GPIOB
#define Main_Cell1_ADC_Pin GPIO_PIN_1
#define Main_Cell1_ADC_GPIO_Port GPIOB
#define Test_Button_Pin GPIO_PIN_2
#define Test_Button_GPIO_Port GPIOB
#define IGN1_SIG_Pin GPIO_PIN_7
#define IGN1_SIG_GPIO_Port GPIOE
#define IGN2_SIG_Pin GPIO_PIN_8
#define IGN2_SIG_GPIO_Port GPIOE
#define IGN3_SIG_Pin GPIO_PIN_9
#define IGN3_SIG_GPIO_Port GPIOE
#define Solenoid_SIG_Pin GPIO_PIN_10
#define Solenoid_SIG_GPIO_Port GPIOE
#define DRV8871_A_Pin GPIO_PIN_11
#define DRV8871_A_GPIO_Port GPIOE
#define DRV8871_B_Pin GPIO_PIN_12
#define DRV8871_B_GPIO_Port GPIOE
#define LD1_Pin GPIO_PIN_13
#define LD1_GPIO_Port GPIOE
#define LD2_Pin GPIO_PIN_14
#define LD2_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define ToF_xSHUT_Pin GPIO_PIN_11
#define ToF_xSHUT_GPIO_Port GPIOD
#define Buzzer_Pin GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOD
#define Servo_Y_Pin GPIO_PIN_6
#define Servo_Y_GPIO_Port GPIOC
#define Servo_X_Pin GPIO_PIN_7
#define Servo_X_GPIO_Port GPIOC
#define RF_RST_Pin GPIO_PIN_8
#define RF_RST_GPIO_Port GPIOA
#define RF_M0_Pin GPIO_PIN_11
#define RF_M0_GPIO_Port GPIOA
#define RF_M1_Pin GPIO_PIN_12
#define RF_M1_GPIO_Port GPIOA
#define SDMMC1_DET_Pin GPIO_PIN_15
#define SDMMC1_DET_GPIO_Port GPIOA
#define STX_3_EN_Pin GPIO_PIN_0
#define STX_3_EN_GPIO_Port GPIOD
#define STX_3_Reset_Pin GPIO_PIN_1
#define STX_3_Reset_GPIO_Port GPIOD
#define SPI3_CS_Pin GPIO_PIN_7
#define SPI3_CS_GPIO_Port GPIOD
#define ICM_20948_INT_Pin GPIO_PIN_6
#define ICM_20948_INT_GPIO_Port GPIOB
#define EEPROM_WP_Pin GPIO_PIN_7
#define EEPROM_WP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
