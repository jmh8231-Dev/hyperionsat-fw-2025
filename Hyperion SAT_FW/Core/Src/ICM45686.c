#include "ICM45686.h"

// === [ 전역 변수 ] ===
static SPI_HandleTypeDef *icm_hspi = NULL;
static icm45686_accel_range_t current_accel_range = ICM_ACCEL_RANGE_16G;
static icm45686_gyro_range_t current_gyro_range = ICM_GYRO_RANGE_2000_DPS;

// === [ 스케일 팩터 ] ===
static const float accel_scale_factors[] = {
    [ICM_ACCEL_RANGE_2G]  = 2.0f / 32768.0f,
    [ICM_ACCEL_RANGE_4G]  = 4.0f / 32768.0f,
    [ICM_ACCEL_RANGE_8G]  = 8.0f / 32768.0f,
    [ICM_ACCEL_RANGE_16G] = 16.0f / 32768.0f,
    [ICM_ACCEL_RANGE_32G] = 32.0f / 32768.0f
};

static const float gyro_scale_factors[] = {
    [ICM_GYRO_RANGE_15_625_DPS] = 15.625f / 32768.0f,
    [ICM_GYRO_RANGE_31_25_DPS]  = 31.25f / 32768.0f,
    [ICM_GYRO_RANGE_62_5_DPS]   = 62.5f / 32768.0f,
    [ICM_GYRO_RANGE_125_DPS]    = 125.0f / 32768.0f,
    [ICM_GYRO_RANGE_250_DPS]    = 250.0f / 32768.0f,
    [ICM_GYRO_RANGE_500_DPS]    = 500.0f / 32768.0f,
    [ICM_GYRO_RANGE_1000_DPS]   = 1000.0f / 32768.0f,
    [ICM_GYRO_RANGE_2000_DPS]   = 2000.0f / 32768.0f,
    [ICM_GYRO_RANGE_4000_DPS]   = 4000.0f / 32768.0f
};

// === [ 내부 유틸리티 함수 ] ===
static inline void ICM_CS_Low(void) {
    HAL_GPIO_WritePin(ICM_CS_GPIO_PORT, ICM_CS_GPIO_PIN, GPIO_PIN_RESET);
}

static inline void ICM_CS_High(void) {
    HAL_GPIO_WritePin(ICM_CS_GPIO_PORT, ICM_CS_GPIO_PIN, GPIO_PIN_SET);
}

// STM32의 DWT를 사용한 정밀 마이크로초 지연 함수
static void ICM_Delay_us(uint32_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks) __NOP();
}

// === [ 레지스터 접근 함수 ] ===
HAL_StatusTypeDef ICM45686_WriteReg(uint8_t reg, uint8_t value) {
    if (icm_hspi == NULL) return HAL_ERROR;

    ICM_CS_Low();
    ICM_Delay_us(1); // tCS_SETUP
    uint8_t tx_data[2] = {reg & 0x7F, value}; // MSB=0 for write
    HAL_StatusTypeDef status = HAL_SPI_Transmit(icm_hspi, tx_data, 2, 100);
    ICM_Delay_us(1); // tCS_HOLD
    ICM_CS_High();
    ICM_Delay_us(1); // tCS_RECOVERY
    return status;
}

HAL_StatusTypeDef ICM45686_ReadReg(uint8_t reg, uint8_t *value) {
    if (icm_hspi == NULL || value == NULL) return HAL_ERROR;

    ICM_CS_Low();
    ICM_Delay_us(1);
    uint8_t tx_buf[2] = {reg | 0x80, 0x00}; // MSB=1 for read, dummy byte
    uint8_t rx_buf[2];
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(icm_hspi, tx_buf, rx_buf, 2, 100);
    ICM_Delay_us(1);
    ICM_CS_High();
    ICM_Delay_us(1);

    if (status == HAL_OK) {
        *value = rx_buf[1];
    }
    return status;
}

HAL_StatusTypeDef ICM45686_ReadMultipleReg(uint8_t reg, uint8_t *buffer, uint16_t length) {
    if (icm_hspi == NULL || buffer == NULL || length == 0) return HAL_ERROR;

    ICM_CS_Low();
    ICM_Delay_us(1);
    uint8_t tx_header = reg | 0x80;
    HAL_StatusTypeDef status = HAL_SPI_Transmit(icm_hspi, &tx_header, 1, 100);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(icm_hspi, buffer, length, 100);
    }
    ICM_Delay_us(1);
    ICM_CS_High();
    ICM_Delay_us(1);

    return status;
}

// === [ 초기화 및 핵심 기능 함수 ] ===
HAL_StatusTypeDef ICM45686_Init(SPI_HandleTypeDef *hspi) {
    if (hspi == NULL) return HAL_ERROR;
    icm_hspi = hspi;

    ICM_CS_High();
    HAL_Delay(10);

    // DWT 활성화 (정밀 지연을 위해)
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    // 소프트 리셋
    if (ICM45686_SoftReset() != HAL_OK) {
        ICM_DEBUG_PRINT("Soft reset failed");
        return HAL_ERROR;
    }
    HAL_Delay(100); // 리셋 후 안정화 시간

    if (ICM45686_EnableDataReadyInterrupt() != HAL_OK) {
        printf("Failed to enable Data Ready Interrupt source!\r\n");
        //while(1); // 디버깅 시 주석 해제
    }

    // WHO_AM_I 확인
    uint8_t who_am_i = 0;
    if (ICM45686_ReadReg(WHO_AM_I, &who_am_i) != HAL_OK) {
        ICM_DEBUG_PRINT("Failed to read WHO_AM_I");
        return HAL_ERROR;
    }

    ICM_DEBUG_PRINT("WHO_AM_I: 0x%02X", who_am_i);
    if (who_am_i != ICM_WHOAMI_ICM45686) {
        ICM_DEBUG_PRINT("Device ID mismatch! Expected 0x%02X", ICM_WHOAMI_ICM45686);
        return HAL_ERROR;
    }

    // 센서 활성화
    if (ICM45686_EnableSensors() != HAL_OK) {
        ICM_DEBUG_PRINT("Enable sensors failed");
        return HAL_ERROR;
    }
    HAL_Delay(50);

    // 기본 레인지 및 ODR 설정
    ICM45686_SetAccelRange(ICM_ACCEL_RANGE_32G);
    ICM45686_SetGyroRange(ICM_GYRO_RANGE_2000_DPS);
    ICM45686_SetODR(ICM_ODR_1000_HZ);

    ICM_DEBUG_PRINT("ICM45686 initialization completed successfully");
    return HAL_OK;
}

HAL_StatusTypeDef ICM45686_SoftReset(void) {
    // PWR_MGMT0 레지스터의 SOFT_RESET 비트(Bit 5)를 설정
    if (ICM45686_WriteReg(PWR_MGMT0, ICM_PWR_MGMT0_SOFT_RESET_BIT) != HAL_OK) {
        ICM_DEBUG_PRINT("Failed to write reset command");
        return HAL_ERROR;
    }
    HAL_Delay(50); // 리셋 완료 대기

    // 리셋 후 비트가 0으로 클리어되었는지 확인
    uint8_t pwr_mgmt_val;
    if (ICM45686_ReadReg(PWR_MGMT0, &pwr_mgmt_val) == HAL_OK) {
        if ((pwr_mgmt_val & ICM_PWR_MGMT0_SOFT_RESET_BIT) == 0) {
            ICM_DEBUG_PRINT("Soft reset successful");
            return HAL_OK;
        }
    }
    ICM_DEBUG_PRINT("Soft reset check failed");
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef ICM45686_EnableSensors(void) {
    // 가속도계와 자이로스코프를 Low Noise 모드로 활성화
    uint8_t pwr_mgmt = ICM_PWR_MGMT0_ACCEL_MODE_LN | ICM_PWR_MGMT0_GYR_MODE_LN;
    return ICM45686_WriteReg(PWR_MGMT0, pwr_mgmt);
}

HAL_StatusTypeDef ICM45686_DisableSensors(void) {
    // 센서들을 Off 모드로 설정
    uint8_t pwr_mgmt = ICM_PWR_MGMT0_ACCEL_MODE_OFF | ICM_PWR_MGMT0_GYR_MODE_OFF;
    return ICM45686_WriteReg(PWR_MGMT0, pwr_mgmt);
}

// === [ 센서 설정 함수 ] ===
HAL_StatusTypeDef ICM45686_SetAccelRange(icm45686_accel_range_t range) {
    uint8_t value;
    ICM45686_ReadReg(ACCEL_CONFIG0, &value);
    value &= ~ICM_ACCEL_CONFIG0_FS_SEL_MASK;

    switch (range) {
        case ICM_ACCEL_RANGE_2G:  value |= (0x4 << 4);  break; // 0x40
        case ICM_ACCEL_RANGE_4G:  value |= (0x3 << 4);  break; // 0x30
        case ICM_ACCEL_RANGE_8G:  value |= (0x2 << 4);  break; // 0x20
        case ICM_ACCEL_RANGE_16G: value |= (0x1 << 4); break; // 0x10
        case ICM_ACCEL_RANGE_32G: value |= (0x0 << 4); break; // 0x00
        default: return HAL_ERROR;
    }

    if (ICM45686_WriteReg(ACCEL_CONFIG0, value) == HAL_OK) {
        current_accel_range = range;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef ICM45686_SetGyroRange(icm45686_gyro_range_t range) {
    uint8_t value;
    ICM45686_ReadReg(GYRO_CONFIG0, &value);
    value &= ~ICM_GYRO_CONFIG0_FS_SEL_MASK;

    switch (range) {
        case ICM_GYRO_RANGE_15_625_DPS: value |= (0x8 << 4); break; // 0x80
        case ICM_GYRO_RANGE_31_25_DPS:  value |= (0x7 << 4);  break; // 0x70
        case ICM_GYRO_RANGE_62_5_DPS:   value |= (0x6 << 4);   break; // 0x60
        case ICM_GYRO_RANGE_125_DPS:    value |= (0x5 << 4);    break; // 0x50
        case ICM_GYRO_RANGE_250_DPS:    value |= (0x4 << 4);    break; // 0x40
        case ICM_GYRO_RANGE_500_DPS:    value |= (0x3 << 4);    break; // 0x30
        case ICM_GYRO_RANGE_1000_DPS:   value |= (0x2 << 4);   break; // 0x20
        case ICM_GYRO_RANGE_2000_DPS:   value |= (0x1 << 4);   break; // 0x10
        case ICM_GYRO_RANGE_4000_DPS:   value |= (0x0 << 4);   break; // 0x00
        default: return HAL_ERROR;
    }

    if (ICM45686_WriteReg(GYRO_CONFIG0, value) == HAL_OK) {
        current_gyro_range = range;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef ICM45686_SetODR(icm45686_odr_t odr) {
    uint8_t odr_val;
    switch (odr) {
        case ICM_ODR_8000_HZ:   odr_val = 0x03; break;
        case ICM_ODR_4000_HZ:   odr_val = 0x04; break;
        case ICM_ODR_2000_HZ:   odr_val = 0x05; break;
        case ICM_ODR_1000_HZ:   odr_val = 0x06; break;
        case ICM_ODR_500_HZ:    odr_val = 0x07; break;
        case ICM_ODR_200_HZ:    odr_val = 0x08; break;
        case ICM_ODR_100_HZ:    odr_val = 0x09; break;
        case ICM_ODR_50_HZ:     odr_val = 0x0A; break;
        case ICM_ODR_25_HZ:     odr_val = 0x0B; break;
        case ICM_ODR_12_5_HZ:   odr_val = 0x0C; break;
        case ICM_ODR_6_25_HZ:   odr_val = 0x0D; break;
        case ICM_ODR_3_125_HZ:  odr_val = 0x0E; break;
        case ICM_ODR_1_5625_HZ: odr_val = 0x0F; break;
        default: return HAL_ERROR;
    }

    uint8_t accel_conf, gyro_conf;
    ICM45686_ReadReg(ACCEL_CONFIG0, &accel_conf);
    ICM45686_ReadReg(GYRO_CONFIG0, &gyro_conf);

    accel_conf = (accel_conf & ~ICM_ACCEL_CONFIG0_ODR_MASK) | odr_val;
    gyro_conf = (gyro_conf & ~ICM_GYRO_CONFIG0_ODR_MASK) | odr_val;

    if (ICM45686_WriteReg(ACCEL_CONFIG0, accel_conf) != HAL_OK) return HAL_ERROR;
    if (ICM45686_WriteReg(GYRO_CONFIG0, gyro_conf) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

// === [ 데이터 읽기 함수 ] ===
HAL_StatusTypeDef ICM45686_ReadRawData(icm45686_raw_data_t *data) {
    if (data == NULL) return HAL_ERROR;
    uint8_t buffer[14];

    // 센서 데이터는 ACCEL_DATA_X1_UI (0x00) 부터 14바이트 연속으로 읽는 것이 맞습니다.
    if (ICM45686_ReadMultipleReg(ACCEL_DATA_X1_UI, buffer, 14) != HAL_OK) {
        return HAL_ERROR;
    }

    // Little-endian 데이터 형식에 맞게 바이트 순서 변경 (LSB, MSB 순서로 가정)
    // buffer[0] = LSB, buffer[1] = MSB
    data->accel_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->accel_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->accel_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    data->gyro_x  = (int16_t)((buffer[7] << 8) | buffer[6]);
    data->gyro_y  = (int16_t)((buffer[9] << 8) | buffer[8]);
    data->gyro_z  = (int16_t)((buffer[11] << 8) | buffer[10]);
    data->temperature = (int16_t)((buffer[13] << 8) | buffer[12]);

    return HAL_OK;
}

HAL_StatusTypeDef ICM45686_ReadScaledData(icm45686_scaled_data_t *data) {
    if (data == NULL) return HAL_ERROR;
    icm45686_raw_data_t raw;

    if (ICM45686_ReadRawData(&raw) != HAL_OK) {
        return HAL_ERROR;
    }

    // 데이터시트 기반 스케일링
    data->temperature = (raw.temperature / 132.48f) + 25.0f;
    data->accel_x = raw.accel_x * accel_scale_factors[current_accel_range];
    data->accel_y = raw.accel_y * accel_scale_factors[current_accel_range];
    data->accel_z = raw.accel_z * accel_scale_factors[current_accel_range];
    data->gyro_x = raw.gyro_x * gyro_scale_factors[current_gyro_range];
    data->gyro_y = raw.gyro_y * gyro_scale_factors[current_gyro_range];
    data->gyro_z = raw.gyro_z * gyro_scale_factors[current_gyro_range];

    return HAL_OK;
}

uint8_t ICM45686_WhoAmI(void) {
    uint8_t who_am_i_val = 0;
    ICM45686_ReadReg(WHO_AM_I, &who_am_i_val);
    return who_am_i_val;
}

/**
  * @brief  데이터 준비 완료(Data Ready) 인터럽트 소스를 활성화합니다.
  * @note   이 함수는 INT 핀의 물리적 동작(Active High)을 설정하고
  * 센서가 내부적으로 DRDY 상태를 INT_STATUS 레지스터에 보고하도록 합니다.
  */
HAL_StatusTypeDef ICM45686_EnableDataReadyInterrupt(void) {
    // INT1 핀을 Push-pull, Active High로 설정
    // INT1_CONFIG2 (0x18) 레지스터: int1_drive (Bit 2), int1_mode (Bit 1), int1_polarity (Bit 0)
    // 0x06 = 0b00000110 -> int1_drive=1, int1_mode=1, int1_polarity=0 (Push-pull, Active High)
    if (ICM45686_WriteReg(INT1_CONFIG2, (1u << 2) | (1u << 1)) != HAL_OK) { // 0x06
        return HAL_ERROR;
    }

    // Data Ready 인터럽트를 INT1에 연결
    // INT1_CONFIG0 (0x16) 레지스터: int1_status_en_drdy (Bit 2)
    if (ICM45686_WriteReg(INT1_CONFIG0, ICM_INT1_CONFIG0_UI_DRDY_INT1_EN) != HAL_OK) {
        return HAL_ERROR;
    }

    ICM_DEBUG_PRINT("Data Ready interrupt source enabled");
    return HAL_OK;
}

/**
  * @brief  새로운 센서 데이터가 준비되었는지 확인합니다.
  * @retval 1: 데이터 준비됨, 0: 데이터 준비되지 않음
  */
uint8_t ICM45686_DataReady(void) {
    uint8_t int_status = 0;
    if (ICM45686_ReadReg(INT1_STATUS0, &int_status) == HAL_OK) {
        // 데이터 준비 완료 인터럽트 플래그(int1_status_drdy) 비트를 확인
        return (int_status & ICM_INT1_STATUS0_UI_DRDY_FLAG) ? 1 : 0;
    }
    return 0;
}
