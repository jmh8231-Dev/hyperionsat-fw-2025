/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "memorymap.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "AS6221.h"
#include "CLCD.h"
#include "INA219.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS6221_MCU_ADDR 0x48 << 1
#define AS6221_SDRAM_ADDR 0x49 << 1
#define AS6221_RF1_ADDR 0x4A << 1
#define AS6221_RF2_ADDR 0x4B << 1

#define NUM_SENSORS	4

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int _write(int file, char *ptr, int len){
    CDC_Transmit_FS(ptr, len);
    return (len);
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern SDRAM_HandleTypeDef hsdram1;
#define SDRAM_DEVICE_ADDR  ((uint32_t)0xD0000000)
#define SDRAM_DEVICE_SIZE  (0x02000000)   /* 32 MB */
#define TEST_BUF_WORDS     256

static uint32_t txBuf[TEST_BUF_WORDS];
static uint32_t rxBuf[TEST_BUF_WORDS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void BSP_SDRAM_Init(void)
{
    /* 1) FMC 컨트롤러 레지스터 초기화 */
    MX_FMC_Init();

    /* 2) JEDEC 표준 전원-업 시퀀스 */
    FMC_SDRAM_CommandTypeDef cmd = {0};

    /* CLK enable */
    cmd.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
    cmd.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
    cmd.AutoRefreshNumber      = 1;
    cmd.ModeRegisterDefinition = 0;
    HAL_SDRAM_SendCommand(&hsdram1, &cmd, HAL_MAX_DELAY);
    HAL_Delay(1);                       /* >100 µs */

    /* Precharge all */
    cmd.CommandMode = FMC_SDRAM_CMD_PALL;
    HAL_SDRAM_SendCommand(&hsdram1, &cmd, HAL_MAX_DELAY);

    /* Auto-Refresh 2회 */
    cmd.CommandMode       = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    cmd.AutoRefreshNumber = 2;
    HAL_SDRAM_SendCommand(&hsdram1, &cmd, HAL_MAX_DELAY);

    /* Mode Register 설정 (Burst=4, Sequential, CAS=3, WriteBurst=Single) */
    uint32_t mode =  (0 << 0)  |      /* Burst length = 1 → 0, 2→1, 4→2…*/
                     (0 << 3)  |      /* Sequential */
                     (3 << 4)  |      /* CAS Latency = 3 */
                     (1 << 9);        /* Write burst = single */
    cmd.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
    cmd.ModeRegisterDefinition = mode;
    HAL_SDRAM_SendCommand(&hsdram1, &cmd, HAL_MAX_DELAY);

    /* Refresh Rate 설정 : 64 ms / 8192 rows = 7.81 µs
       → (7.81 µs × 100 MHz) – 20 = 758 */
    HAL_SDRAM_ProgramRefreshRate(&hsdram1, 758);
}

/* ② 간단한 R/W 테스트 함수 */
void SDRAM_Test(void)
{
    /* 패턴 생성 */
	uint32_t testData = 0x77777777;

    /* 32-비트 단위 Write & Read */
	HAL_SDRAM_Write_32b(&hsdram1, SDRAM_DEVICE_ADDR, &testData, SDRAM_DEVICE_SIZE / 4);
//    HAL_SDRAM_Read_32b (&hsdram1, SDRAM_DEVICE_ADDR, rxBuf, TEST_BUF_WORDS);
//
//    /* 검증 */
//    for (uint32_t i=0;i<TEST_BUF_WORDS;i++)
//    {
//        if (rxBuf[i] != txBuf[i])
//        {
//            Error_Handler();          /* 불일치 시 중단 */
//        }
//    }
}

static HAL_StatusTypeDef GDK101_Send(char cmd,
                                     const char *data,
                                     uint8_t *rx,
                                     uint16_t len,
                                     uint32_t tout_ms)
{
    /* ---------- TX ---------- */
    uint8_t tx[8];               /* 0x02 C : ? CR LF  = 6~7B */
    uint8_t i = 0;
    tx[i++] = 0x02;              /* STX '┐' */
    tx[i++] = (uint8_t)cmd;
    tx[i++] = ':';
    while (*data && i < sizeof(tx)-2) tx[i++] = (uint8_t)*data++;
    tx[i++] = 0x0D;              /* CR */
    tx[i++] = 0x0A;              /* LF */

    if (HAL_UART_Transmit(&huart1, tx, i, tout_ms) != HAL_OK)
        return HAL_ERROR;

    /* ---------- RX : < … LF > 까지 가변 길 읽기 ---------- */
    uint32_t t0 = HAL_GetTick();
    uint16_t pos = 0;
    while ((HAL_GetTick() - t0) < tout_ms)
    {
        uint8_t c;
        if (HAL_UART_Receive(&huart1, &c, 1, 10) == HAL_OK)
        {
            if (pos < len-1) rx[pos++] = c;   /* 버퍼 초과 보호 */
            if (c == 0x0A) break;             /* LF → 패킷 끝 */
        }
    }

    if (pos == 0 || rx[pos-1] != 0x0A)        /* 아무 것도 못 받음 */
        return HAL_TIMEOUT;

    rx[pos] = 0;                              /* NULL 종단 */
    return HAL_OK;
}


static void ShowDose(void)
{
    static uint8_t first = 1;
    uint8_t buf[24];
    char    line[20];
    float   dose;

    /* ── Auto-send OFF : U:0 (응답 없음 정상) ── */
    if (first) { GDK101_Send('U', "0", buf, sizeof(buf), 100); HAL_Delay(20); first = 0; }

    /* ── 1 분 평균 ── */
    if (GDK101_Send('M', "?", buf, sizeof(buf), 60) != HAL_OK) {
        lcd_setCurStr(0, 0, "UART Timeout   "); return;
    }
    char *p = (buf[0] == 0x02) ? (char*)&buf[1] : (char*)buf;  /* STX 필터 */
    if (sscanf(p, "M:%f", &dose) != 1) {
        lcd_setCurStr(0, 0, "Parse Error    "); return;
    }
    sprintf(line, "1m:%5.2f uSv/h", dose);
    lcd_setCurStr(0, 0, line);

    /* ── 10 분 평균 ── */
    if (GDK101_Send('D', "?", buf, sizeof(buf), 60) == HAL_OK) {
        p = (buf[0] == 0x02) ? (char*)&buf[1] : (char*)buf;
        if (sscanf(p, "D:%f", &dose) == 1) {
            sprintf(line, "10m:%5.2f uSv/h", dose);
            lcd_setCurStr(0, 1, line);
            return;
        }
    }
    lcd_setCurStr(0, 1, "                ");   /* 실패 시 클리어 */
}

/* ----- 설정 값 ---------------------------------------------------------- */
#define TEST_FILE_NAME   "0:/speed.bin"
#define TEST_TOTAL_BYTES (1024 * 1024)     /* 1 MiB */
#define CHUNK_BYTES      512               /* 섹터 크기(= DMA 전송 단위) */

/* ----- 전역 변수 -------------------------------------------------------- */
static uint8_t ioBuf[CHUNK_BYTES];         /* 512 B 버퍼 */
static FIL      sdFile;
static UINT     xferBytes;

/* LCD 출력 헬퍼 (row, col, printf-style) */
static void LCD_Print(uint8_t row, uint8_t col, const char *fmt, ...)
{
    static char line[32];
    va_list ap; va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    lcd_setCurStr(row, col, line);
}

/* ----- 속도 측정 함수 --------------------------------------------------- */
static float Measure_Write_Speed(void)
{
    /* 1. 새 파일 생성 */
    retSD = f_open(&sdFile, TEST_FILE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (retSD != FR_OK) { LCD_Print(0,0,"openW err %d", retSD); return -1.0f; }

    uint32_t start_ms = HAL_GetTick();
    uint32_t done     = 0;

    /* 2. TEST_TOTAL_BYTES 만큼 반복 기록 */
    while (done < TEST_TOTAL_BYTES)
    {
        retSD = f_write(&sdFile, ioBuf, CHUNK_BYTES, &xferBytes);
        if (retSD != FR_OK || xferBytes != CHUNK_BYTES) break;
        done += CHUNK_BYTES;
    }
    f_close(&sdFile);

    uint32_t elapsed_ms = HAL_GetTick() - start_ms;          /* 경과 시간 */
    return (done / 1024.0f) / (elapsed_ms / 1000.0f);        /* kB/s */
}

static float Measure_Read_Speed(void)
{
    /* 1. 읽기 전용으로 다시 열기 */
    retSD = f_open(&sdFile, TEST_FILE_NAME, FA_READ);
    if (retSD != FR_OK) { LCD_Print(0,0,"openR err %d", retSD); return -1.0f; }

    uint32_t start_ms = HAL_GetTick();
    uint32_t done     = 0;

    /* 2. 파일 끝까지 반복 읽기 */
    while (done < TEST_TOTAL_BYTES)
    {
        retSD = f_read(&sdFile, ioBuf, CHUNK_BYTES, &xferBytes);
        if (retSD != FR_OK || xferBytes == 0) break;
        done += xferBytes;
    }
    f_close(&sdFile);

    uint32_t elapsed_ms = HAL_GetTick() - start_ms;
    return (done / 1024.0f) / (elapsed_ms / 1000.0f);        /* kB/s */
}

/* ----- 메인 루틴 예시 --------------------------------------------------- */
void SD_SpeedTest(void)
{
    /* FatFs 마운트 */
    if ((retSD = f_mount(&SDFatFS, &SDPath[0], 1)) != FR_OK)
    { LCD_Print(0,0,"mount err %d", retSD);
      return;
    }

    /* 버퍼 초기화(패턴 채우기) */
    memset(ioBuf, 0xA5, sizeof(ioBuf));

    float w_kBps = Measure_Write_Speed();   /* 쓰기 속도 */
    float r_kBps = Measure_Read_Speed();    /* 읽기 속도 */

    if (w_kBps < 0 || r_kBps < 0) return;   /* 오류 발생 시 종료 */

    /* LCD에 결과 표시 (row, col) */
    LCD_Print(0, 0, "W:%.1f kB/s", w_kBps);
    LCD_Print(0, 1, "R:%.1f kB/s", r_kBps);
}

void SelectSdram(uint8_t sdram_num)
{
    /* 모든 칩 비활성화(HIGH) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);

    /* 원하는 SDRAM만 LOW로 설정 */
    switch (sdram_num)
    {
        case 1:                     /* SDRAM1 → PD13 */
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
            break;

        case 2:                     /* SDRAM2 → PD12 */
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
            break;

        case 3:                     /* SDRAM3 → PA0  */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,  GPIO_PIN_RESET);
            break;

        case 4:                     /* SDRAM4 → PA1  */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,  GPIO_PIN_RESET);
            break;

        default:                    /* 잘못된 값 → 전부 HIGH 유지 */
            break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
//  timeout = 0xFFFF;
//  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
//  if ( timeout < 0 )
//  {
//  Error_Handler();
//  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
//timeout = 0xFFFF;
//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
//if ( timeout < 0 )
//{
//Error_Handler();
//}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM12_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_RESET);

//  printf("Boot..!\n");

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

  __HAL_TIM_SET_AUTORELOAD(&htim12, 999);
  __HAL_TIM_SET_COMPARE (&htim12, TIM_CHANNEL_1, 499);
  HAL_Delay(150);

  HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);

  char str[16] = "choi hee soo gay";

//  printf("Boot\n");
//  HAL_Delay(2000);
//
//  uint8_t str[30];
//  lcd_Init(20, 4);
//  lcd_clear();
//
//  SelectSdram(1);
//
//  BSP_SDRAM_Init();       /* 위 시퀀스 호출 */
//
//  SDRAM_Test();           /* 메모리 검증 */
//
//  SelectSdram(2);
//
//  BSP_SDRAM_Init();       /* 위 시퀀스 호출 */
//
//  SDRAM_Test();           /* 메모리 검증 */
//
//  SelectSdram(3);
//
//  BSP_SDRAM_Init();       /* 위 시퀀스 호출 */
//
//  SDRAM_Test();           /* 메모리 검증 */
//
//  SelectSdram(4);
//
//  BSP_SDRAM_Init();       /* 위 시퀀스 호출 */
//
//  SDRAM_Test();           /* 메모리 검증 */
//
//  BYTE buf[32] = "Hello world";
//  uint32_t bw, br;
//
//  if((retSD = f_mount(&SDFatFS, &SDPath[0], 1)) == FR_OK) {
//	  sprintf(str, "f_mount OK %d", retSD);
//	  lcd_setCurStr(0, 0, str);
//  }
//  else {
//	  sprintf(str, "f_mount Failed %d", retSD);
//	  lcd_setCurStr(0, 0, str);
//  }
//
//  if((retSD = f_open(&SDFile, "0:/jmh.txt", FA_CREATE_NEW | FA_WRITE)) == FR_OK) {
//	  sprintf(buf, "Hello world");
//	  f_write(&SDFile, buf, sizeof(buf), &bw);
//
//	  sprintf(str, "%d bytes", bw);
//	  lcd_setCurStr(0, 1, str);
//
//	  f_close(&SDFile);
//  }
//  else {
//	  sprintf(str, "error %d", retSD);
//	  lcd_setCurStr(0, 0, str);
//  }


//  INA219_t ina219;
//
//  // INA219 센서 초기화
//  if (INA219_Init(&ina219, &hi2c1, INA219_ADDRESS) != 1) {
//      lcd_setCurStr(0, 0, "INA219 Init FAIL");
//      sprintf(str, "I2C Error: %lu", HAL_I2C_GetError(&hi2c1));
//      lcd_setCurStr(0, 1, str);
//      while(1); // 에러 발생시 멈춤
//  }
//
//  lcd_setCurStr(0, 0, "INA219 Init PASS!");
//  HAL_Delay(2000);
//  lcd_clear();

  lcd_Init(20, 4);

  AS6221_t as6221[NUM_SENSORS];

  as6221[0].address = AS6221_MCU_ADDR;
  as6221[0].CR = ConvPer125ms;
  as6221[0].CF = Quadruple;
  as6221[0].SM = false;
  as6221[0].IM = false;
  as6221[0].POL = false;
  as6221[0].SS = false;

  as6221[1] = as6221[0];
  as6221[2] = as6221[0];

  as6221[1].address = AS6221_SDRAM_ADDR;
  as6221[2].address = AS6221_RF1_ADDR;
  as6221[3].address = AS6221_RF2_ADDR;

  for (int i = 0; i < NUM_SENSORS; i++) {
	  if (AS6221_Init(&as6221[i]) != AS6221_ERROR_NONE) {
		  sprintf(str, "Failed Init %d", i + 1);
		  lcd_setCurStr(0, i, str);
		  sprintf(str, "Error Code: %d", AS6221_GetLastError());
		  lcd_setCurStr(0, 3, str);
	  }
  }


  HAL_Delay(2000);
  lcd_clear();

//  SD_SpeedTest();
//  HAL_Delay(1000);


//  lcd_Init(20, 4);
//  lcd_clear();
//
//  uint8_t str[30];
//
//  BYTE buf[32] = " Hello world";
//  uint32_t bw, br;
//
//  if((retSD = f_mount(&SDFatFS, &SDPath[0], 1)) == FR_OK) {
//	  sprintf(str, "f_mount OK %d", retSD);
//	  lcd_setCurStr(0, 0, str);
//  }
//  else {
//	  sprintf(str, "f_mount Failed %d", retSD);
//	  lcd_setCurStr(0, 0, str);
//  }

//  uint8_t str[20];
//  lcd_Init(20, 4);
//  lcd_clear();
//
//  for(int i = 0; i < 4; i++) {
//	  SelectSdram(i + 1);
//	  lcd_clear();
//	  sprintf(str, "CS: %d", i + 1);
//	  lcd_setCurStr(0, 0, str);
//	  HAL_Delay(3000);
//  }


  char a = "\n";
  HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for(int i = 0; i < 16; i++)
		  HAL_UART_Transmit(&huart2, str[i], 1, 100);

	  HAL_UART_Transmit(&huart2, &a, 1, 100);
//      ShowDose();             /* 1 초마다 센서 값 표시 */
//      HAL_Delay(1000);

//      uint16_t voltage = INA219_ReadBusVoltage(&ina219);
//      int16_t current = INA219_ReadCurrent(&ina219);
//      uint16_t power = INA219_ReadPower(&ina219);
//
//      sprintf(str, "V: %2.3fV  I: %3dmA", (voltage / 1000.0) + 0.2871, current);
//      lcd_setCurStr(0, 0, str);
//      sprintf(str, "P: %4dmW", power);
//      lcd_setCurStr(0, 1, str);
//
//
//      HAL_ADC_Start(&hadc3);
//      HAL_ADC_PollForConversion(&hadc3, 100);
//      uint32_t raw = HAL_ADC_GetValue(&hadc3);
//      double temp = __HAL_ADC_CALC_TEMPERATURE(3300, raw, LL_ADC_RESOLUTION_16B);
//
//      sprintf(str, "CPU Temp: %.2fC", temp);
//	  lcd_setCurStr(0, 2, str);
//
//	  printf("Test Power: %dmW\n", power);
//
//      HAL_Delay(1000); // 1초마다 갱신
//
	  for (int i = 0; i < NUM_SENSORS; i++)
	  {
		  if (!AS6221_ReadTemperature(&as6221[i]))
		  {
			  sprintf(str, "Sensor %d Error", i + 1);
			  lcd_setCurStr(0, 0, str);
			  HAL_Delay(100);
		  }
	  }

	  sprintf(str, "MCU: %.3f", as6221[0].Temp);
	  lcd_setCurStr(0, 0, str);

	  sprintf(str, "SDRAM: %.3f", as6221[1].Temp);
	  lcd_setCurStr(0, 1, str);

	  sprintf(str, "RF Module1: %.3f", as6221[2].Temp);
	  lcd_setCurStr(0, 2, str);

	  sprintf(str, "RF Module2: %.3f", as6221[3].Temp);
	  lcd_setCurStr(0, 3, str);

	  HAL_Delay(1000);

//
//	  HAL_Delay(2500);
//
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//
//	  HAL_Delay(2500);
//
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//
//	  HAL_Delay(2500);
//
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//
//	  HAL_Delay(2500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* SDMMC1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
