/*
 * Data.h
 *
 *  Created on: Aug 5, 2025
 *      Author: binbe
 */

#ifndef INC_DATA_H_
#define INC_DATA_H_

extern float CPU_Temp;

typedef struct {
	float Main_BAT_Cell1;
	float Main_BAT_Cell2;
	float IGN_BAT;
	float CR2032_BAT;
	uint16_t CDS_Top;
	uint16_t CDS_Bottom;
} adcData;

typedef struct {
	uint16_t altitude;
	float temp;
	float pressure_mbar;
} ms5611Data;

typedef struct {
    float accel_x, accel_y, accel_z;    // g
    float gyro_x, gyro_y, gyro_z;       // dps
    float temperature;                  // Celsius
} icm45686Data;

typedef struct {
	I2C_HandleTypeDef *ina219_i2c;
	uint8_t	Address;
	uint16_t bus_voltage_mv;
	int16_t	current;
	uint16_t power;
	uint8_t battery_level;
} ina219Data;

typedef struct {
    uint16_t address;              // I2C 주소
    uint16_t TLOW;                 // 낮은 온도 임계값
    uint16_t THIGH;                // 높은 온도 임계값
    double Temp;                   // 현재 온도
    uint8_t CR : 2;                // 변환률 설정 (2비트)
    uint8_t CF : 2;                // 센서 구성 (2비트)
    bool SM : 1;                   // 동작 모드 (1비트)
    bool IM : 1;                   // 인터럽트 모드 (1비트)
    bool POL : 1;                  // 폴라리티 (1비트)
    bool SS : 1;                   // 셧다운 모드 (1비트)
} AS6221;

/* NAV-PVT 파싱 결과 저장용 */
typedef struct {
    unsigned char CLASS;
    unsigned char ID;
    unsigned short LENGTH;

    unsigned int iTOW;
    signed int lon;
	signed int lat;
    signed int height;
    signed int hMSL;
    unsigned int hAcc;
    unsigned int vAcc;

    double lon_f64;
    double lat_f64;
} gpsData;

const unsigned char UBX_CFG_PRT[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xD0, 0x08,
    0x00, 0x00, 0x80, 0x25, 0x00,
    0x00, 0x01, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79
};

const unsigned char UBX_CFG_MSG[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x01, 0x02, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x13, 0xBE
};

const unsigned char UBX_CFG_RATE[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xC8, 0x00, 0x01, 0x00, 0x01, 0x00,
    0xDE, 0x6A
};

const unsigned char UBX_CFG_CFG[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x17, 0x31, 0xBF
};




#endif /* INC_DATA_H_ */
