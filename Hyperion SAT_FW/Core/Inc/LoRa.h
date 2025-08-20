#ifndef LORA_H_
#define LORA_H_

#include <stdint.h>
#include "stdbool.h"
#include "main.h"

// 기본 채널 정의
#define LORA_CHANNEL_MIN     0x00
#define LORA_CHANNEL_DEFAULT 0x17
#define LORA_CHANNEL_MAX     0x1F

// HEAD 필드 (명령 타입)
#define LORA_DEFAULT_HEAD    0xC2  // 휘발성 설정 (전원 꺼지면 초기화됨)
#define LORA_DEFAULT_ADDH    0x00
#define LORA_DEFAULT_ADDL    0x00


extern volatile uint8_t rx_data_flag;

// LoRa 모듈 핀 기본 매핑 구조체 선언 (구조체 방식 사용)
typedef struct {
    GPIO_TypeDef* m0_port;
    uint16_t m0_pin;
    GPIO_TypeDef* m1_port;
    uint16_t m1_pin;
    GPIO_TypeDef* rst_port;
    uint16_t rst_pin;
} LoRaPinMap_t;

// 외부 참조용 기본 핀맵 객체 선언
extern LoRaPinMap_t LoRa_DefaultPinMap;

// LoRa 모듈 설정 구조체 (1바이트 단위로 정렬)
#pragma pack(push, 1)
typedef struct {
    uint8_t HEAD;  // 명령 헤더 (C0, C1, C2)
    uint8_t ADDH;  // 주소 상위
    uint8_t ADDL;  // 주소 하위

    union {
        uint8_t SPED;
        struct {
            uint8_t airDataRate  : 3;
            uint8_t uartBaudRate : 3;
            uint8_t parity       : 2;
        };
    };

    union {
        uint8_t CHAN;
        struct {
            uint8_t channel  : 5;
            uint8_t reserved : 3;
        };
    };

    union {
        uint8_t OPTION;
        struct {
            uint8_t power             : 2;
            uint8_t fec               : 1;
            uint8_t wakeupTime        : 3;
            uint8_t ioDriveMode       : 1;
            uint8_t fixedTransmission : 1;
        };
    };
} LoRaParams_t;
#pragma pack(pop)

// 직렬 통신 패리티
typedef enum {
    LORA_PARITY_8N1 = 0x00,
    LORA_PARITY_8O1 = 0x01,
    LORA_PARITY_8E1 = 0x02
} LoRaSerialParity_t;

// UART 보레이트 설정
typedef enum {
    LORA_UART_BAUD_1200   = 0x00,
    LORA_UART_BAUD_2400   = 0x01,
    LORA_UART_BAUD_4800   = 0x02,
    LORA_UART_BAUD_9600   = 0x03,
    LORA_UART_BAUD_19200  = 0x04,
    LORA_UART_BAUD_38400  = 0x05,
    LORA_UART_BAUD_57600  = 0x06,
    LORA_UART_BAUD_115200 = 0x07
} LoRaSerialBaud_t;

// 무선 전송 속도
typedef enum {
    LORA_AIR_RATE_0_3K  = 0x00,
    LORA_AIR_RATE_1_2K  = 0x01,
    LORA_AIR_RATE_2_4K  = 0x02,
    LORA_AIR_RATE_4_8K  = 0x03,
    LORA_AIR_RATE_9_6K  = 0x04,
    LORA_AIR_RATE_19_2K = 0x05
} LoRaAirDataRate_t;

// 전송 모드
typedef enum {
    LORA_MODE_TRANSPARENT = 0,  // 브로드캐스트
    LORA_MODE_FIXED       = 1   // 지정 주소 전송
} LoRaFixedTransmit_t;

// IO 드라이브 모드
typedef enum {
    LORA_IO_OPEN_DRAIN = 0,
    LORA_IO_PUSH_PULL  = 1
} LoRaIODrive_t;

// 웨이크업 시간
typedef enum {
    LORA_WAKEUP_250MS  = 0x00,
    LORA_WAKEUP_500MS  = 0x01,
    LORA_WAKEUP_750MS  = 0x02,
    LORA_WAKEUP_1000MS = 0x03,
    LORA_WAKEUP_1250MS = 0x04,
    LORA_WAKEUP_1500MS = 0x05,
    LORA_WAKEUP_1750MS = 0x06,
    LORA_WAKEUP_2000MS = 0x07
} LoRaWakeupTime_t;

// FEC 사용 여부
typedef enum {
    LORA_FEC_DISABLED = 0,
    LORA_FEC_ENABLED  = 1
} LoRaFEC_t;


// 하드웨어 모드
typedef enum {
    LORA_HW_MODE_NORMAL = 0,
    LORA_HW_MODE_WAKEUP,
    LORA_HW_MODE_POWER_SAVING,
    LORA_HW_MODE_SLEEP
} LoRaHardwareMode_t;

// 함수 정의
void LoRa_InitDefaultParams(LoRaParams_t *param);                        // 기본값 설정
void LoRa_SetHardwareMode(LoRaHardwareMode_t mode);
void LoRa_SendParams(const LoRaParams_t *param); // C2 명령으로 파라미터 전송
void LoRa_FactoryReset();                       // 공장 초기화 (C0 명령)
HAL_StatusTypeDef LoRa_ReceiveParams(UART_HandleTypeDef *huart, LoRaParams_t *param);
void LoRa_ReadParams();
void LoRa_Init(UART_HandleTypeDef *huart, LoRaParams_t *param);
uint16_t crc16_calculate(uint8_t *msg, uint16_t len);
uint8_t LoRa_SendPacket(uint8_t *data, uint8_t len);
void LoRa_Receive_Process(uint8_t rx3_Data);
uint16_t crc16_calculate(uint8_t *msg, uint16_t len);
bool LoRa_Receive_CheckCRC(uint8_t *rx3_buf);

#endif /* LORA_H_ */
