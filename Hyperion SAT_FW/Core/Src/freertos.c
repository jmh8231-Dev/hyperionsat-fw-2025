/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_ms5611_basic.h"
#include <stdio.h>
#include <math.h>
#include "AS6221.h"
#include "DS3231.h"

#include "ICM45686.h"

#include "Data.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "LoRa.h"
#include "INA219.h"
#include "i2c.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include <stdint.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS6221_Board_ADDR 0x48 << 1
#define AS6221_PMIC_ADDR 0x49 << 1
#define AS6221_LoRa_ADDR 0x4A << 1
#define AS6221_IGNFET_ADDR 0x4B << 1
#define NUM_SENSORS 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int _write(int file, char* p, int len){
	HAL_UART_Transmit(&huart1, p, len, HAL_MAX_DELAY);
	return len;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
adcData adcdata;
uint16_t adc1_val[6];
uint16_t adc3_val[2];
icm45686Data icm45686;
ms5611Data ms5611;
ina219Data ina219_main;
ina219Data ina219_ign;
AS6221 as6221[NUM_SENSORS];
float CPU_Temp = 0;

uint8_t m10s_rx_buf[36];
volatile uint8_t m10s_rx_cplt_flag = 0;
volatile uint8_t rx3_data;
volatile uint8_t rx1_data;

volatile uint8_t Decoupling_flag = false;
volatile uint8_t forceING = false;

gpsData gps;

// 임무 상태 변수들
bool cansat_ejected = false;
bool hyperion_separated = false;
bool second_propellant_ignited = false;
bool evasive_maneuver_done = false;
bool max_altitude_reached = false;
bool parachute_ejected = false;
static float prev_altitude_for_parachute = 0.0f;

/* USER CODE END Variables */
/* Definitions for BootTask */
osThreadId_t BootTaskHandle;
const osThreadAttr_t BootTask_attributes = {
  .name = "BootTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendDataTask */
osThreadId_t sendDataTaskHandle;
const osThreadAttr_t sendDataTask_attributes = {
  .name = "sendDataTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADC_Task */
osThreadId_t ADC_TaskHandle;
const osThreadAttr_t ADC_Task_attributes = {
  .name = "ADC_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GPS_Task */
osThreadId_t GPS_TaskHandle;
const osThreadAttr_t GPS_Task_attributes = {
  .name = "GPS_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MS5611_Task */
osThreadId_t MS5611_TaskHandle;
const osThreadAttr_t MS5611_Task_attributes = {
  .name = "MS5611_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for INA219_Task */
osThreadId_t INA219_TaskHandle;
const osThreadAttr_t INA219_Task_attributes = {
  .name = "INA219_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ReadTemp_Task */
osThreadId_t ReadTemp_TaskHandle;
const osThreadAttr_t ReadTemp_Task_attributes = {
  .name = "ReadTemp_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t ubx_checksum(uint8_t *payload, uint16_t length);
void M10S_UBX_POSLLH_Parsing(unsigned char* data, gpsData* gps);
void M10S_TransmitData(unsigned char* data, unsigned len);
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length);
void CRC16_CCITT_Split(const uint8_t *data, uint16_t length,
                       uint8_t *high, uint8_t *low);

/* USER CODE END FunctionPrototypes */

void StartBootTask(void *argument);
void StartsendDataTask(void *argument);
void StartADC_Task(void *argument);
void StartIMU_Task(void *argument);
void StartGPS_Task(void *argument);
void StartMS5611_Task(void *argument);
void StartINA219_Task(void *argument);
void StartReadTemp_Task(void *argument);
void StartMainTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BootTask */
  BootTaskHandle = osThreadNew(StartBootTask, NULL, &BootTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBootTask */
/**
 * @brief  Function implementing the BootTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBootTask */
void StartBootTask(void *argument)
{
  /* USER CODE BEGIN StartBootTask */
	HAL_UART_Receive_IT(&huart1, &rx1_data, 1);
	HAL_UART_Receive_IT(&huart3, &rx3_data, 1);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); // 전자석 On
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET); // 전자석 On

	if (ICM45686_Init(&hspi1) == HAL_OK) {
		IMU_TaskHandle = osThreadNew(StartIMU_Task, NULL, &IMU_Task_attributes);
	}
	if (ms5611_basic_init(MS5611_INTERFACE_SPI, MS5611_ADDRESS_CSB_0) == 0) {
		MS5611_TaskHandle = osThreadNew(StartMS5611_Task, NULL,
				&MS5611_Task_attributes);
	}
	if (HAL_ADC_Start_DMA(&hadc1, &adc1_val[0], 6) == HAL_OK) {
		ADC_TaskHandle = osThreadNew(StartADC_Task, NULL, &ADC_Task_attributes);
	}
	if (INA219_Init(&ina219_main, &hi2c2, 0x40) == HAL_OK && INA219_Init(&ina219_ign, &hi2c2, 0x41) == HAL_OK) {
		INA219_setCalibration_32V_30A_001R(&ina219_ign);
		INA219_TaskHandle = osThreadNew(StartINA219_Task, NULL, &INA219_Task_attributes);
	}

	as6221[0].address = AS6221_Board_ADDR;
	as6221[0].CR = ConvPer125ms;
	as6221[0].CF = Quadruple;
	as6221[0].SM = false;
	as6221[0].IM = false;
	as6221[0].POL = false;
	as6221[0].SS = false;

	as6221[1] = as6221[0];
	as6221[2] = as6221[0];
	as6221[3] = as6221[0];

	as6221[1].address = AS6221_PMIC_ADDR;
	as6221[2].address = AS6221_LoRa_ADDR;
	as6221[3].address = AS6221_IGNFET_ADDR;

	uint8_t count = 0;

	for (int i = 0; i < NUM_SENSORS; i++) {
		if (AS6221_Init(&as6221[i]) != AS6221_ERROR_NONE) {
			count++;
		}
	}

	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	if (count < 4 && HAL_ADC_Start_DMA(&hadc3, &adc3_val[0], 2) == HAL_OK)
		ReadTemp_TaskHandle = osThreadNew(StartReadTemp_Task, NULL,
				&ReadTemp_Task_attributes);

	sendDataTaskHandle = osThreadNew(StartsendDataTask, NULL,
			&sendDataTask_attributes);
	GPS_TaskHandle = osThreadNew(StartGPS_Task, NULL, &GPS_Task_attributes);
	MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	TIM4->PSC = 1199;
	osDelay(100);
	TIM4->PSC = 799;
	osDelay(100);
	TIM4->PSC = 599;
	osDelay(150);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);

	// BootTask 종료
	vTaskDelete(NULL); // Worker Task 삭제
  /* USER CODE END StartBootTask */
}

/* USER CODE BEGIN Header_StartsendDataTask */
/**
 * @brief Function implementing the sendDataTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartsendDataTask */
void StartsendDataTask(void *argument)
{
  /* USER CODE BEGIN StartsendDataTask */

	/* Infinite loop */
	for (;;) {
		uint8_t F_data[8] = {0};

		uint8_t altitude_L = 0;
		uint8_t altitude_H = 0;
		uint8_t ck_L, ck_H = 0;

		altitude_H = (ms5611.altitude >> 8) & 0xFF;  // 상위 8비트
	    altitude_L = ms5611.altitude & 0xFF;          // 하위 8비트

	    F_data[0] = 'H';
	    F_data[1] = 'S';
	    F_data[2] = 10;
	    F_data[3] = 2;
	    F_data[4] = altitude_H;
	    F_data[5] = altitude_L;
	    CRC16_CCITT_Split(&F_data[2], 4, ck_H, ck_L);
	    F_data[6] = ck_H;
	    F_data[7] = ck_L;

	    for(int i = 0; i < sizeof(F_data); i++) {
	    	printf("%d", F_data[i]);

	    	if(i < sizeof(F_data) - 1)
	    		printf(",");
	    		osDelay(10);
	    }
	    printf("\n");

		ck_L = 0;
		ck_H = 0;

		uint8_t A_data[38] = {0};

				// 가속도, 자이로, GPS 데이터를 바이트로 변환
				int16_t i_accel_x = (int16_t)icm45686.accel_x;
				uint16_t f_accel_x = (uint16_t)((icm45686.accel_x - i_accel_x) * 10000);
				int16_t i_accel_y = (int16_t)icm45686.accel_y;
				uint16_t f_accel_y = (uint16_t)((icm45686.accel_y - i_accel_y) * 10000);
				int16_t i_accel_z = (int16_t)icm45686.accel_z;
				uint16_t f_accel_z = (uint16_t)((icm45686.accel_z - i_accel_z) * 10000);

				int16_t i_gyro_x = (int16_t)icm45686.gyro_x;
				uint16_t f_gyro_x = (uint16_t)((icm45686.gyro_x - i_gyro_x) * 10000);
				int16_t i_gyro_y = (int16_t)icm45686.gyro_y;
				uint16_t f_gyro_y = (uint16_t)((icm45686.gyro_y - i_gyro_y) * 10000);
				int16_t i_gyro_z = (int16_t)icm45686.gyro_z;
				uint16_t f_gyro_z = (uint16_t)((icm45686.gyro_z - i_gyro_z) * 10000);

				int16_t i_gps_lat = (int16_t)gps.lat_f64;
				uint16_t f_gps_lat = (uint16_t)((gps.lat_f64 - i_gps_lat) * 1000000);
				int16_t i_gps_lon = (int16_t)gps.lon_f64;
				uint16_t f_gps_lon = (uint16_t)((gps.lon_f64 - i_gps_lon) * 1000000);

				A_data[0] = 'H';
				A_data[1] = 'S';
				A_data[2] = 11;
				A_data[3] = 32;

				A_data[4] = (i_accel_x >> 8) & 0xFF;
				A_data[5] = i_accel_x & 0xFF;
				A_data[6] = (f_accel_x >> 8) & 0xFF;
				A_data[7] = f_accel_x & 0xFF;
				A_data[8] = (i_accel_y >> 8) & 0xFF;
				A_data[9] = i_accel_y & 0xFF;
				A_data[10] = (f_accel_y >> 8) & 0xFF;
				A_data[11] = f_accel_y & 0xFF;
				A_data[12] = (i_accel_z >> 8) & 0xFF;
				A_data[13] = i_accel_z & 0xFF;
				A_data[14] = (f_accel_z >> 8) & 0xFF;
				A_data[15] = f_accel_z & 0xFF;

				A_data[16] = (i_gyro_x >> 8) & 0xFF;
				A_data[17] = i_gyro_x & 0xFF;
				A_data[18] = (f_gyro_x >> 8) & 0xFF;
				A_data[19] = f_gyro_x & 0xFF;
				A_data[20] = (i_gyro_y >> 8) & 0xFF;
				A_data[21] = i_gyro_y & 0xFF;
				A_data[22] = (f_gyro_y >> 8) & 0xFF;
				A_data[23] = f_gyro_y & 0xFF;
				A_data[24] = (i_gyro_z >> 8) & 0xFF;
				A_data[25] = i_gyro_z & 0xFF;
				A_data[26] = (f_gyro_z >> 8) & 0xFF;
				A_data[27] = f_gyro_z & 0xFF;

				A_data[28] = (i_gps_lat >> 8) & 0xFF;
				A_data[29] = i_gps_lat & 0xFF;
				A_data[30] = (f_gps_lat >> 8) & 0xFF;
				A_data[31] = f_gps_lat & 0xFF;
				A_data[32] = (i_gps_lon >> 8) & 0xFF;
				A_data[33] = i_gps_lon & 0xFF;
				A_data[34] = (f_gps_lon >> 8) & 0xFF;
				A_data[35] = f_gps_lon & 0xFF;

				CRC16_CCITT_Split(&A_data[2], 34, ck_H, ck_L);
				A_data[36] = ck_H;
				A_data[37] = ck_L;

				for(int i = 0; i < sizeof(A_data); i++) {
					printf("%d", A_data[i]);
					if(i < sizeof(A_data) - 1)
						printf(",");
						osDelay(10);
				}
				printf("\n");


				uint8_t S_data[33] = {0};

				// 센서 데이터를 바이트로 변환
				uint8_t i_main_cell1 = (uint8_t)adcdata.Main_BAT_Cell1;
				uint8_t f_main_cell1 = (uint8_t)((adcdata.Main_BAT_Cell1 - i_main_cell1) * 100);
				uint8_t i_main_cell2 = (uint8_t)adcdata.Main_BAT_Cell2;
				uint8_t f_main_cell2 = (uint8_t)((adcdata.Main_BAT_Cell2 - i_main_cell2) * 100);

				uint8_t i_ign_v = (uint8_t)adcdata.IGN_BAT;
				uint8_t f_ign_v = (uint8_t)((adcdata.IGN_BAT - i_ign_v) * 100);

				uint8_t i_cpu_temp = (uint8_t)CPU_Temp;
				uint8_t f_cpu_temp = (uint8_t)((CPU_Temp - i_cpu_temp) * 100);

				uint8_t i_board_temp = (uint8_t)as6221[0].Temp;
				uint8_t f_board_temp = (uint8_t)((as6221[0].Temp - i_board_temp) * 100);

				uint8_t i_pmic_temp = (uint8_t)as6221[1].Temp;
				uint8_t f_pmic_temp = (uint8_t)((as6221[1].Temp - i_pmic_temp) * 100);

				uint8_t i_lora_temp = (uint8_t)as6221[2].Temp;
				uint8_t f_lora_temp = (uint8_t)((as6221[2].Temp - i_lora_temp) * 100);

				uint8_t i_ign_fet_temp = (uint8_t)as6221[3].Temp;
				uint8_t f_ign_fet_temp = (uint8_t)((as6221[3].Temp - i_ign_fet_temp) * 100);

				S_data[0] = 'H';
				S_data[1] = 'S';
				S_data[2] = 12;
				S_data[3] = 27;

				S_data[4] = i_main_cell1;
				S_data[5] = f_main_cell1;
				S_data[6] = i_main_cell2;
				S_data[7] = f_main_cell2;
				S_data[8] = (ina219_main.current >> 8) & 0xFF;
				S_data[9] = ina219_main.current & 0xFF;
				S_data[10] = ina219_main.battery_level;
				S_data[11] = i_ign_v;
				S_data[12] = f_ign_v;
				S_data[13] = ina219_ign.current;
				S_data[14] = ina219_ign.battery_level;
				S_data[15] = i_cpu_temp;
				S_data[16] = f_cpu_temp;
				S_data[17] = i_board_temp;
				S_data[18] = f_board_temp;
				S_data[19] = i_pmic_temp;
				S_data[20] = f_pmic_temp;
				S_data[21] = i_lora_temp;
				S_data[22] = f_lora_temp;
				S_data[23] = i_ign_fet_temp;
				S_data[24] = f_ign_fet_temp;
				S_data[25] = (adcdata.CR2032_BAT > 2.5) ? 1 : 0; // 3.3V PG
				S_data[26] = (adcdata.CR2032_BAT > 1.5) ? 1 : 0; // 1.8V PG (임시 값, 실제 1.8V 레귤레이터 PG 데이터 필요)
				S_data[27] = (adcdata.CDS_Top >> 8) & 0xFF;
				S_data[28] = adcdata.CDS_Top & 0xFF;
				S_data[29] = (adcdata.CDS_Bottom >> 8) & 0xFF;
				S_data[30] = adcdata.CDS_Bottom & 0xFF;

				CRC16_CCITT_Split(&S_data[2], 29, ck_H, ck_L);
				S_data[31] = ck_H;
				S_data[32] = ck_L;

				for(int i = 0; i < sizeof(S_data); i++) {
					printf("%d", S_data[i]);
					if(i < sizeof(S_data) - 1)
						printf(",");
						osDelay(10);
				}
				printf("\n");

				osDelay(2000);
	}
  /* USER CODE END StartsendDataTask */
}

/* USER CODE BEGIN Header_StartADC_Task */
/**
 * @brief Function implementing the ADC_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADC_Task */
void StartADC_Task(void *argument)
{
  /* USER CODE BEGIN StartADC_Task */
#define ADC_RESOLUTION 65535.0f

	/* Infinite loop */
	for (;;) {
		adcdata.Main_BAT_Cell1 = (((float) adc1_val[0] / ADC_RESOLUTION) * 3.3)
				* 4.5;
		adcdata.Main_BAT_Cell2 = (((float) adc1_val[1] / ADC_RESOLUTION) * 3.3)
				* 4.5;
		adcdata.IGN_BAT = (((float) adc1_val[2] / ADC_RESOLUTION) * 3.3) * 4.5;
		adcdata.CR2032_BAT = (((float) adc1_val[3] / ADC_RESOLUTION) * 3.3) * 2;
		adcdata.CDS_Top = adc1_val[4];
		adcdata.CDS_Bottom = adc1_val[5];

		osDelay(100);
	}
  /* USER CODE END StartADC_Task */
}

/* USER CODE BEGIN Header_StartIMU_Task */
/**
 * @brief Function implementing the IMU_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIMU_Task */
void StartIMU_Task(void *argument)
{
  /* USER CODE BEGIN StartIMU_Task */
    static uint32_t last_impact_ms = 0;
    // 임계값은 실제 비행 데이터 로그를 분석하여 최적의 값을 찾아야 합니다.
    #define ACCEL_IMPACT_THRESHOLD_G   30.0f   // 충격 감지를 위한 가속도 임계값 (G)
    #define GYRO_IMPACT_THRESHOLD_DPS  2000.0f  // 충격 감지를 위한 각속도 임계값 (degrees per second)
    #define IMPACT_DEBOUNCE_MS         500     // 이벤트 감지 후 안정화 시간 (조금 더 길게 설정)

    /* Infinite loop */
    for (;;) {
        if (ICM45686_DataReady()) {
            ICM45686_ReadScaledData(&icm45686);

            // 1. 가속도 크기 계산
            float accel_mag = sqrtf(icm45686.accel_x * icm45686.accel_x +
                                    icm45686.accel_y * icm45686.accel_y +
                                    icm45686.accel_z * icm45686.accel_z);

            // 2. 각속도 크기 계산
            float gyro_mag = sqrtf(icm45686.gyro_x * icm45686.gyro_x +
                                  icm45686.gyro_y * icm45686.gyro_y +
                                  icm45686.gyro_z * icm45686.gyro_z);

            // 3. 가속도와 각속도가 동시에 임계값을 넘는지 확인
            if (accel_mag >= ACCEL_IMPACT_THRESHOLD_G && gyro_mag >= GYRO_IMPACT_THRESHOLD_DPS) {
                uint32_t now = HAL_GetTick();
                if (now - last_impact_ms >= IMPACT_DEBOUNCE_MS) {
                    last_impact_ms = now;
                    printf("SEPARATION DETECTED! Accel: %.2f G, Gyro: %.2f dps\n\n", accel_mag, gyro_mag);
                }
            }
        }
        else {
            osDelay(1);
        }
    }
  /* USER CODE END StartIMU_Task */
}

/* USER CODE BEGIN Header_StartGPS_Task */
/**
 * @brief Function implementing the GPS_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGPS_Task */
void StartGPS_Task(void *argument)
{
  /* USER CODE BEGIN StartGPS_Task */
	M10S_TransmitData(&UBX_CFG_PRT[0], sizeof(UBX_CFG_PRT));
	osDelay(200);
	M10S_TransmitData(&UBX_CFG_MSG[0], sizeof(UBX_CFG_MSG));
	osDelay(200);
	M10S_TransmitData(&UBX_CFG_RATE[0], sizeof(UBX_CFG_RATE));
	osDelay(200);
	M10S_TransmitData(&UBX_CFG_CFG[0], sizeof(UBX_CFG_CFG));

  /* Infinite loop */
  for(;;)
  {
	  if(m10s_rx_cplt_flag == 1) {
		  m10s_rx_cplt_flag = 0;

		  if(ubx_checksum(&m10s_rx_buf[0], 36) == 1) {
			  M10S_UBX_POSLLH_Parsing(&m10s_rx_buf[0], &gps);
		  }
	  }
    osDelay(100);
  }
  /* USER CODE END StartGPS_Task */
}

/* USER CODE BEGIN Header_StartMS5611_Task */
/**
 * @brief Function implementing the MS5611_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMS5611_Task */
void StartMS5611_Task(void *argument)
{
  /* USER CODE BEGIN StartMS5611_Task */
	float launch_pad_pressure_hpa = 0.0f;

	// 고도 값 보정
	for (int i = 0; i < 20; i++) {
		if (ms5611_basic_read(&ms5611.temp, &ms5611.pressure_mbar) == 0)
			launch_pad_pressure_hpa += ms5611.pressure_mbar;
	}

	launch_pad_pressure_hpa = launch_pad_pressure_hpa / 20;

	/* Infinite loop */
	for (;;) {
		if (ms5611_basic_read(&ms5611.temp, &ms5611.pressure_mbar) == 0) {
			ms5611.altitude = 44330.0f *(1.0f - pow(ms5611.pressure_mbar / launch_pad_pressure_hpa, 1.0f / 5.255f));
		}
		osDelay(1);
	}
  /* USER CODE END StartMS5611_Task */
}

/* USER CODE BEGIN Header_StartINA219_Task */
/**
 * @brief Function implementing the INA219_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartINA219_Task */
void StartINA219_Task(void *argument)
{
  /* USER CODE BEGIN StartINA219_Task */
	/* Infinite loop */
	for (;;) {
		ina219_main.bus_voltage_mv = INA219_ReadBusVoltage(&ina219_main) + 790;
		ina219_main.current = INA219_ReadCurrent(&ina219_main);         // mA 단위
		ina219_main.power = INA219_ReadPower(&ina219_main) / 10;        // mW 단위
		ina219_main.battery_level = INA219_GetBatteryLife(&ina219_main, 8400,
				6000);

		ina219_ign.bus_voltage_mv = INA219_ReadBusVoltage(&ina219_ign);
		ina219_ign.current = INA219_ReadCurrent(&ina219_ign) / 1000;
		ina219_ign.power = INA219_ReadPower(&ina219_ign) / 1000;
		ina219_ign.battery_level = INA219_GetBatteryLife(&ina219_ign, 12600,
				9000);

		osDelay(100);
	}
  /* USER CODE END StartINA219_Task */
}

/* USER CODE BEGIN Header_StartReadTemp_Task */
/**
 * @brief Function implementing the ReadTemp_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartReadTemp_Task */
void StartReadTemp_Task(void *argument)
{
  /* USER CODE BEGIN StartReadTemp_Task */
	/* Infinite loop */
	for (;;) {
		for (int i = 0; i < NUM_SENSORS; i++) {
			AS6221_ReadTemperature(&as6221[i]);
			osDelay(10);
		}

		CPU_Temp = (float) __HAL_ADC_CALC_TEMPERATURE(3300, adc3_val[0],
				ADC_RESOLUTION_12B) / 100.0f;

		osDelay(100);
	}
  /* USER CODE END StartReadTemp_Task */
}

/* USER CODE BEGIN Header_StartMainTask */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
	uint16_t Decoupling_alt = 0;
	uint16_t Max_alt = 0;
  /* Infinite loop */
  for(;;) {
    // 1. 대회용 로켓 발사 (StartMainTask 진입 시 이미 발사된 것으로 가정)

    // 2. CanSAT 발사체로부터 사출 (고도 400m 이상에서 분리)
    if (Decoupling_flag) {
    	Decoupling_flag = false;

    	Decoupling_alt = ms5611.altitude;

    	if(Decoupling_alt > ms5611.altitude + 20) {
    		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET); // 솔레노이드 HIGH (아답터 분리)
    		osDelay(1000); // 솔레노이드 작동 시간
    		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET); // 솔레노이드 LOW

    		 hyperion_separated = true;
    	}
    }

    // 5. 아답터와 일정 거리 떨어진 후 2차 추진제 점화 (고도센서가 200m 이하일 경우 그냥 점화)
    if (hyperion_separated && !second_propellant_ignited) {
      if (ms5611.altitude <= 200 || forceING) { // 고도 200m 이하일 경우
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET); // 고체연료 점화 HIGH
        osDelay(15000); // 최대 15초간 On
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); // 고체연료 점화 LOW
        second_propellant_ignited = true;
      }
    }

    if(Max_alt < ms5611.altitude)
    	Max_alt = ms5611.altitude;

    if(Max_alt > ms5611.altitude + 30 || (ms5611.altitude < 200 && hyperion_separated)) {
    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    osDelay(1);

  }
  /* USER CODE END StartMainTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static unsigned char cnt = 0;

	if (huart->Instance == USART3) {
		HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
//		HAL_UART_Transmit(&huart8, &rx3_data, 1, 100);

		switch(cnt) {
		case 0:
			if(rx3_data == 0xb5) {
				m10s_rx_buf[cnt] = rx3_data;
				cnt++;
			}
			break;
		case 1:
			if(rx3_data == 0x62) {
				m10s_rx_buf[cnt] = rx3_data;
				cnt++;
			}
			else
				cnt = 0;
			break;
		case 35:
			m10s_rx_buf[cnt] = rx3_data;
			cnt = 0;
			m10s_rx_cplt_flag = 1;
			break;
		default:
			m10s_rx_buf[cnt] = rx3_data;
			cnt++;
			break;
		}
	}
	if(huart->Instance == USART3) {
		HAL_UART_Receive_IT(&huart1, &rx1_data, 1);

		if(rx1_data == 'D') {
			Decoupling_flag = true;
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		}
		if(rx1_data == 'I') {
			forceING = true;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}
}

void M10S_TransmitData(unsigned char* data, unsigned len) {
	for(int i = 0; i < len; i++) {
		HAL_UART_Transmit(&huart3, &data[i], 1, 100);
	}
}

uint8_t ubx_checksum(uint8_t *payload, uint16_t length) {
	uint8_t a = 0, b = 0;

	for (uint16_t i = 2; i < length - 2; i++) {
		a = a + payload[i];
		b = b + a;
	}

	return(a == payload[length - 2]) && (b == payload[length - 1]);
}

void M10S_UBX_POSLLH_Parsing(unsigned char* data, gpsData* gps) {
	gps->CLASS = data[2];
	gps->ID = data[3];
	gps->LENGTH = data[4] | data[5] << 8;

	gps->iTOW = data[6] | data[7] << 8 | data[8] << 16 | data[9] << 24;
	gps->lon = data[10] | data[11] << 8 | data[12] << 16 | data[13] <<24;
	gps->lat = data[14] | data[15] << 8 | data[16] << 16 | data[17] <<24;
	gps->height = data[18] | data[19] << 8 | data[20] << 16 | data[21] <<24;
	gps->hMSL = data[22] | data[23] << 8 | data[24] << 16 | data[25] <<24;
	gps->hAcc = data[26] | data[27] << 8 | data[28] << 16 | data[29] <<24;
	gps->vAcc = data[30] | data[31] << 8 | data[32] << 16 | data[33] <<24;

	gps->lon_f64 = gps->lon  / 10000000.;
	gps->lat_f64 = gps->lat  / 10000000.;
}

uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    while (length--)
    {
        crc ^= (uint16_t)(*data++) << 8;  // 상위바이트부터 XOR
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021
                                 : (crc << 1);
    }
    return crc;
}

void CRC16_CCITT_Split(const uint8_t *data, uint16_t length,
                       uint8_t *high, uint8_t *low)
{
    uint16_t crc = CRC16_CCITT(data, length);
    *high = (uint8_t)(crc >> 8);
    *low  = (uint8_t)(crc & 0xFF);
}

/* USER CODE END Application */

