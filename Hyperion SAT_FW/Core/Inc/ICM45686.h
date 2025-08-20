#ifndef __ICM45686_H
#define __ICM45686_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "inv_imu_regmap_le.h"   /* 공식 Bank-0 UI 맵 (little-endian) */

// === [ DEBUG LOGGING ] ===
#define DEBUG 0

#if DEBUG
  #include <stdio.h>
  #define ICM_DEBUG_PRINT(fmt, ...)  printf("[ICM45686] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define ICM_DEBUG_PRINT(fmt, ...)
#endif

// === [ 핀맵/포트 정의 - 사용자가 수정해야 하는 부분 ] ===
#define ICM_CS_GPIO_PORT   GPIOA
#define ICM_CS_GPIO_PIN    GPIO_PIN_4

#define ICM_INT_GPIO_PORT  GPIOA
#define ICM_INT_GPIO_PIN   GPIO_PIN_3

// === [ WHO_AM_I 값 ] ===
#define ICM_WHOAMI_ICM45686       0xE9

// === [ 비트 마스크 및 값 정의 ] ===
// PWR_MGMT0 (0x10) 레지스터
// inv_imu_regmap_le.h의 pwr_mgmt0_t 구조체 참고
#define ICM_PWR_MGMT0_SOFT_RESET_BIT    (1u << 5) // Bit 5: 소프트 리셋 비트 (데이터시트 기준)
#define ICM_PWR_MGMT0_GYR_MODE_MASK     (3u << 2)
#define ICM_PWR_MGMT0_ACCEL_MODE_MASK   0x03
#define ICM_PWR_MGMT0_GYR_MODE_OFF      (0u << 2)
#define ICM_PWR_MGMT0_GYR_MODE_STBY     (1u << 2)
#define ICM_PWR_MGMT0_GYR_MODE_LN       (3u << 2)
#define ICM_PWR_MGMT0_ACCEL_MODE_OFF    0x00
#define ICM_PWR_MGMT0_ACCEL_MODE_LP     0x02
#define ICM_PWR_MGMT0_ACCEL_MODE_LN     0x03

// GYRO_CONFIG0 (0x1C) 레지스터
// inv_imu_regmap_le.h의 gyro_config0_t 구조체 참고
#define ICM_GYRO_CONFIG0_FS_SEL_MASK    0xF0 // gyro_ui_fs_sel (4비트)
#define ICM_GYRO_CONFIG0_ODR_MASK       0x0F // gyro_odr (4비트)
#define ICM_GYRO_FS_2000DPS             (0x1 << 4) // 0x10
#define ICM_GYRO_FS_1000DPS             (0x2 << 4) // 0x20
#define ICM_GYRO_FS_500DPS              (0x3 << 4) // 0x30
#define ICM_GYRO_FS_250DPS              (0x4 << 4) // 0x40
#define ICM_GYRO_FS_125DPS              (0x5 << 4) // 0x50
#define ICM_GYRO_FS_62_5DPS             (0x6 << 4) // 0x60
#define ICM_GYRO_FS_31_25DPS            (0x7 << 4) // 0x70
#define ICM_GYRO_FS_15_625DPS           (0x8 << 4) // 0x80
#define ICM_GYRO_FS_4000DPS             (0x0 << 4) // 0x00 (데이터시트 확인 필요)

// ACCEL_CONFIG0 (0x1B) 레지스터
// inv_imu_regmap_le.h의 accel_config0_t 구조체 참고
#define ICM_ACCEL_CONFIG0_FS_SEL_MASK   0x70 // accel_ui_fs_sel (3비트)
#define ICM_ACCEL_CONFIG0_ODR_MASK      0x0F // accel_odr (4비트)
#define ICM_ACCEL_FS_16G                (0x1 << 4) // 0x10
#define ICM_ACCEL_FS_8G                 (0x2 << 4) // 0x20
#define ICM_ACCEL_FS_4G                 (0x3 << 4) // 0x30
#define ICM_ACCEL_FS_2G                 (0x4 << 4) // 0x40
#define ICM_ACCEL_FS_32G                (0x0 << 4) // 0x00 (데이터시트 확인 필요)

// INT1_CONFIG0 (0x16) / INT1_STATUS0 (0x19) 비트
// inv_imu_regmap_le.h의 int1_config0_t, int1_status0_t 구조체 참고
#define ICM_INT1_CONFIG0_UI_DRDY_INT1_EN (1u << 2) // int1_status_en_drdy
#define ICM_INT1_STATUS0_UI_DRDY_FLAG    (1u << 2) // int1_status_drdy

// INTF_CONFIG0 (0x2C) 비트
// inv_imu_regmap_le.h의 intf_config0_t 구조체 참고
#define ICM_INTF_CONFIG0_AP_SPI_MODE_MASK  0x01 // ap_spi_mode
#define ICM_INTF_CONFIG0_AP_SPI_MODE_4WIRE 0x00 // 4-wire SPI mode
#define ICM_INTF_CONFIG0_AP_SPI_MODE_3WIRE 0x01 // 3-wire SPI mode

// === [ 데이터 구조체 및 열거형 ] ===
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temperature;
} icm45686_raw_data_t;

typedef struct {
    float accel_x, accel_y, accel_z;    // g
    float gyro_x, gyro_y, gyro_z;       // dps
    float temperature;                  // Celsius
} icm45686_scaled_data_t;

typedef enum {
    ICM_ACCEL_RANGE_2G = 0, ICM_ACCEL_RANGE_4G, ICM_ACCEL_RANGE_8G, ICM_ACCEL_RANGE_16G, ICM_ACCEL_RANGE_32G
} icm45686_accel_range_t;

typedef enum {
    ICM_GYRO_RANGE_15_625_DPS = 0, ICM_GYRO_RANGE_31_25_DPS, ICM_GYRO_RANGE_62_5_DPS,
    ICM_GYRO_RANGE_125_DPS, ICM_GYRO_RANGE_250_DPS, ICM_GYRO_RANGE_500_DPS,
    ICM_GYRO_RANGE_1000_DPS, ICM_GYRO_RANGE_2000_DPS, ICM_GYRO_RANGE_4000_DPS
} icm45686_gyro_range_t;

typedef enum {
    ICM_ODR_8000_HZ, ICM_ODR_4000_HZ, ICM_ODR_2000_HZ, ICM_ODR_1000_HZ,
    ICM_ODR_500_HZ, ICM_ODR_200_HZ, ICM_ODR_100_HZ, ICM_ODR_50_HZ,
    ICM_ODR_25_HZ, ICM_ODR_12_5_HZ, ICM_ODR_6_25_HZ, ICM_ODR_3_125_HZ,
    ICM_ODR_1_5625_HZ
} icm45686_odr_t;

// === [ 함수 선언 ] ===
HAL_StatusTypeDef ICM45686_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ICM45686_SoftReset(void);
HAL_StatusTypeDef ICM45686_EnableSensors(void);
HAL_StatusTypeDef ICM45686_DisableSensors(void);
HAL_StatusTypeDef ICM45686_SetAccelRange(icm45686_accel_range_t range);
HAL_StatusTypeDef ICM45686_SetGyroRange(icm45686_gyro_range_t range);
HAL_StatusTypeDef ICM45686_SetODR(icm45686_odr_t odr);
HAL_StatusTypeDef ICM45686_ReadRawData(icm45686_raw_data_t *data);
HAL_StatusTypeDef ICM45686_ReadScaledData(icm45686_scaled_data_t *data);
uint8_t ICM45686_WhoAmI(void);
uint8_t ICM45686_DataReady(void);
HAL_StatusTypeDef ICM45686_EnableDataReadyInterrupt(void);

#endif /* __ICM45686_H */
