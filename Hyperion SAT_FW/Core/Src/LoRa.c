#include "LoRa.h"
#include "usart.h"
#include <stdbool.h>

static UART_HandleTypeDef *lora_huart = NULL;
#define RX_BUF_LEN  6
#define LORA_UART_TIMEOUT 1000  // UART 타임아웃 (ms)
volatile uint8_t rx_data_flag = 0;


// 기본 파라미터 설정 함수
void LoRa_InitDefaultParams(LoRaParams_t *param) {
    if (param == NULL) return;

    param->HEAD = LORA_DEFAULT_HEAD;
    param->ADDH = LORA_DEFAULT_ADDH;
    param->ADDL = LORA_DEFAULT_ADDL;

    param->parity       = LORA_PARITY_8N1;
    param->uartBaudRate = LORA_UART_BAUD_9600;
    param->airDataRate  = LORA_AIR_RATE_2_4K;

    param->channel = LORA_CHANNEL_DEFAULT;

    param->fixedTransmission = LORA_MODE_TRANSPARENT;
    param->ioDriveMode       = LORA_IO_PUSH_PULL;
    param->wakeupTime        = LORA_WAKEUP_250MS;
    param->fec               = LORA_FEC_ENABLED;
    param->power             = 0x00;
}

// 하드웨어 모드 설정 함수
void LoRa_SetHardwareMode(LoRaHardwareMode_t mode) {
    switch (mode) {
        case LORA_HW_MODE_NORMAL:
            HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_RESET);
            break;
        case LORA_HW_MODE_WAKEUP:
            HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_RESET);
            break;
        case LORA_HW_MODE_POWER_SAVING:
            HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_SET);
            break;
        case LORA_HW_MODE_SLEEP:
            HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_SET);
            break;
    }

    // 리셋 시퀀스
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_RESET);
    osDelay(10);
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);
    osDelay(10);
}
//파라미터를 변경하는 함수

void LoRa_Init(UART_HandleTypeDef *huart, LoRaParams_t *param)
{

    if (huart == NULL) return HAL_ERROR;
    	lora_huart = huart;
    if (param == NULL) return;

    // 기본값 고정
    param->HEAD  = LORA_DEFAULT_HEAD;
    param->ADDH  = LORA_DEFAULT_ADDH;
    param->ADDL  = LORA_DEFAULT_ADDL;
    param->parity       = LORA_PARITY_8N1;
    param->ioDriveMode  = LORA_IO_PUSH_PULL;
    param->wakeupTime   = LORA_WAKEUP_250MS;
    param->power = 0X00;

    LoRa_SendParams(param);

}

// 파라미터 전송 함수 (C0 + 5바이트)
void LoRa_SendParams(const LoRaParams_t *param) {
	LoRa_SetHardwareMode(LORA_HW_MODE_SLEEP);
	osDelay(500);
	   //1. 파라미터 전송을 위해 9600으로 설정
		HAL_UART_DeInit(lora_huart);
	    lora_huart->Init.BaudRate = 9600;
	    if (HAL_UART_Init(lora_huart) != HAL_OK) {  // ← 여기서 초기화!
	        Error_Handler();
	    }
	    osDelay(100); // UART 초기화 안정화

    uint8_t sped = (param->parity << 6) | (param->uartBaudRate << 3) | param->airDataRate;
    uint8_t option = (param->fixedTransmission << 7) | (param->ioDriveMode << 6) |
                     (param->wakeupTime << 3) | (param->fec << 2) | param->power;

    uint8_t frame[6] = {
    	0xC0,
        param->ADDH,
        param->ADDL,
        sped,
        param->channel,
        option
    };

    HAL_UART_Transmit(lora_huart, frame, sizeof(frame), LORA_UART_TIMEOUT);
    osDelay(200);  // AUX 사용 안하므로 충분한 지연 필요

    LoRa_SetHardwareMode(LORA_HW_MODE_NORMAL);
    osDelay(500);

    // ✅ 6. MCU UART 재설정 (이제 모듈도 바뀌었기 때문에 MCU도 맞춰줘야 함)
       HAL_UART_DeInit(lora_huart);

    switch (param->uartBaudRate) {
        case LORA_UART_BAUD_1200:
            lora_huart->Init.BaudRate = 1200;
            break;

        case LORA_UART_BAUD_2400:
            lora_huart->Init.BaudRate = 2400;
            break;

        case LORA_UART_BAUD_4800:
            lora_huart->Init.BaudRate = 4800;
            break;

        case LORA_UART_BAUD_9600:
            lora_huart->Init.BaudRate = 9600;
            break;

        case LORA_UART_BAUD_19200:
            lora_huart->Init.BaudRate = 19200;
            break;

        case LORA_UART_BAUD_38400:
            lora_huart->Init.BaudRate = 38400;
            break;

        case LORA_UART_BAUD_57600:
            lora_huart->Init.BaudRate = 57600;
            break;

        case LORA_UART_BAUD_115200:
            lora_huart->Init.BaudRate = 115200;
            break;
}
    if (HAL_UART_Init(lora_huart) != HAL_OK)
    {
      Error_Handler();
    }
    osDelay(100);
}

// LoRa 명령 전송 (파라미터 요청: C1 C1 C1)
void LoRa_ReadParams()
{
    uint8_t cmd[3] = {0xC1, 0xC1, 0xC1};
    HAL_UART_Transmit(lora_huart, cmd, sizeof(cmd), LORA_UART_TIMEOUT);
}

// 공장 초기화 함수 (C0 + 0x00 ~ 0xFF)
void LoRa_FactoryReset() {
    uint8_t reset_cmd[6] = {0xC0, 0x00, 0x00, 0x1A, 0x17, 0x44}; // 일반적인 초기값
    HAL_UART_Transmit(lora_huart, reset_cmd, sizeof(reset_cmd), LORA_UART_TIMEOUT);
    osDelay(200);
}

//fc에서 GCC로 데이터를 송신하는 함수
uint8_t LoRa_SendPacket(uint8_t *data, uint8_t len)
{
    uint8_t packet[128];
    uint8_t ck1, ck2 = 0;

    // 1. 헤더 추가
    packet[0] = 0x48;  // 'H'
    packet[1] = 0x53;  // 'S'


    for (int i = 0; i < len; i++)
        packet[i + 2] = data[i];


    uint16_t crc = crc16_calculate(&packet[4], len -2);

    ck1 = (uint8_t)(crc & 0xFF);        // LSB (하위 바이트)
    ck2 = (uint8_t)((crc >> 8) & 0xFF); // MSB (상위 바이트)

    packet[len + 2] = ck1;
    packet[len + 3] = ck2;

    char comma = ',';

    // 4. 데이터 송신 (배열 요소마다 쉼표와 함께)
    for (uint8_t i = 0; i < len + 4; i++)
    {
        if(HAL_UART_Transmit(lora_huart, &packet[i], 1, LORA_UART_TIMEOUT) != HAL_OK)
        	return HAL_ERROR;

        if(i < len + 3)
        	 if(HAL_UART_Transmit(lora_huart, &comma, 1, LORA_UART_TIMEOUT) != HAL_OK)
        		 return HAL_ERROR;
    }

	return 1;
}
//crc16 함수
uint16_t crc16_calculate(uint8_t *msg, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= msg[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}
//uint16_t crc16_calculate(uint8_t *msg, uint16_t len)
//{
//    uint16_t crc_val = 0;
//    uint8_t ck1 = 0;
//    uint8_t ck2 = 0;
//
//    for (uint16_t i = 0; i < len; i++)
//    {
//        if (i == 0)
//            crc_val = (*msg) ^ 0xFFFF;
//        else
//        {
//            msg++;
//            crc_val = crc_val ^ (*msg);
//        }
//
//        for (uint8_t j = 0; j < 8; j++)
//        {
//            if (crc_val & 0x0001)
//            {
//                crc_val = (crc_val >> 1) ^ 0xA001;
//            }
//            else
//            {
//                crc_val = crc_val >> 1;
//            }
//        }
//    }
//
//    ck1 = crc_val;
//    ck2 = crc_val >> 8;
//    crc_val = (ck2 << 8) + ck1;
//
//    return crc_val;
//}

//FC에서 Check sum 하는 함수

bool LoRa_Receive_CheckCRC(uint8_t *rx3_buf)
{
    // ID와 DATA만으로 CRC 계산
    uint16_t calc_crc = crc16_calculate(&rx3_buf[2], 2);

    // 수신된 CRC 추출 (CK2 << 8 | CK1)
    uint16_t recv_crc = ((uint16_t)rx3_buf[5] << 8) | rx3_buf[4];

    return (calc_crc == recv_crc);
}


//FC에서 수신 인터럽트로 받으며 header 및 check sum 확인
void LoRa_Receive_Process(uint8_t rx3_Data)
{
	static uint8_t cnt = 0;
	static uint8_t rx3_buf[RX_BUF_LEN];

	if(rx_data_flag == 0) {
		switch(cnt)
		{
			case 0:
				if(rx3_Data ==  'G')//아스키 코드로 G임
				{
					rx3_buf[cnt] = rx3_Data;
					cnt ++;
				}
				 break;
			case 1:
				if(rx3_Data == 'S')//아스키 코드로 S임
				{
					rx3_buf[cnt] = rx3_Data;
					cnt++;
				}
				else
					cnt = 0;
				break;
			case 5:
				rx3_buf[cnt] = rx3_Data;
				cnt = 0;
				if (LoRa_Receive_CheckCRC(rx3_buf)) rx_data_flag = 1;
				cnt = 0;
				break;
			default:
				rx3_buf[cnt] = rx3_Data;
				cnt ++;
				break;
		}
	}
}

//HAL_StatusTypeDef LoRa_ReceiveParams(UART_HandleTypeDef *huart, LoRaParams_t *param) {
//    if (HAL_UART_Receive(huart, (uint8_t*)param, 6, 1000) == HAL_OK) {
//        // UART1 (PC 출력용)으로 출력
//        extern UART_HandleTypeDef huart1;  // 반드시 extern 필요
//
//        const char *msg = ">> LoRa Params: ";
//        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
//
//        uint8_t *p = (uint8_t*)param;
//        for (int i = 0; i < 6; i++) {
//            char buf[5];
//            int len = snprintf(buf, sizeof(buf), "%02X ", p[i]);
//            HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
//        }
//        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
//
//        return HAL_OK;
//    }
//    return HAL_ERROR;
//}
