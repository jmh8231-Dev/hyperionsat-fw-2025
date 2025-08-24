# HyperionSAT Flight-Computer – **FW README** (2025)

> **Target MCU**: STM32H723VGT6 (550 MHz) · **RTOS**: FreeRTOS/CMSIS‑OS v2 · **툴체인**: STM32CubeIDE  
> **보드**: HyperionSAT 비행 컴퓨터 (Rev 1.0) · **본 문서 기준 소스**: `main.c`, `freertos.c`(2025‑08‑25)

---

## 0) 임무 개요(펌웨어 관점 요약)

**비행 시퀀스**
1. 대회용 로켓 발사
2. CanSAT 발사체에서 사출(고도≈400 m)
3. 노즈 어댑터 낙하산으로 자세 정렬
4. 약 100 m 낙하 후 자세 안정 → **리니어 솔레노이드**로 Hyperion SAT 분리
5. 어댑터와 **거리 분리 확인(TOF)** 후 **2차 추진(점화 1채널)** 시작  
   · 예외: 거리센서 값 이상 시 **고도 ≤ 200 m** 조건만으로 점화
6. **TVC 서보**로 회피 기동 후 90° 상승
7. 최대 고도 도달
8. 고도 하강 시작 또는 특정 각도 이상 기울어짐 → **전자석 Off → 낙하산 전개**

**FW에서의 연결**
- 분리/점화/전자석 제어는 `StartMainTask()` 상태 로직으로 처리 (GPIO 제어).
- 분리 트리거·예외 트리거는 UART 명령(`'D'`,`'I'`) 및 센서 조건으로 입력.

---

## 1) 펌웨어 아키텍처

### FreeRTOS 태스크(이름·스택·우선순위·역할)
- **BootTask** (2 KB, Normal): UART1/3 RX‑IT 시작, 전자석 **On** 초기화, 센서/드라이버 초기화 후 하위 태스크 생성 → 종료.  
- **IMU_Task** (1 KB, Normal): `ICM45686` 데이터 준비 시 스케일 변환·충격량 계산(가속/자이로 크기) 및 **사출 이벤트 감지**(디바운스 500 ms).  
- **MS5611_Task** (1 KB, BelowNormal): 초기 20샘플로 **발사장 기준압** 평균 → **표준식**으로 고도 계산(연속).  
- **ADC_Task** (1 KB, BelowNormal): `ADC1` DMA(6ch) → **Main Cell1/2**, **IGN_BAT**, **CR2032_BAT**, **CDS_Top/Bottom** 스케일링.  
- **INA219_Task** (0.5 KB, Low): 메인/점화 **전압·전류·전력** 읽기 및 **잔량(%)** 평가(메인: 8.4–6.0 V, 점화: 12.6–9.0 V 범위 기준).  
- **ReadTemp_Task** (1 KB, Low): `AS6221` 4채널(보드/PMIC/LoRa/IGN‑FET) 온도 및 **CPU 온도**(내장 센서) 수집.  
- **GPS_Task** (1 KB, Normal): u‑blox MAX‑M10S 초기 UBX 설정(PRT/MSG/RATE/CFG) → `UBX‑NAV‑POSLLH` 수신·파싱.  
- **sendDataTask** (2 KB, Low): **텔레메트리 프레이밍**(ID 10/11/12) 및 출력(현재 `printf` CSV 로그).  
- **MainTask** (4 KB, High): **임무 시퀀스 상태기계**(분리/점화/전자석) 실행.

> 스택/우선순위는 `freertos.c`의 Thread Attribute를 기준으로 작성했습니다.

### 사용 드라이버/미들웨어
- 센서: `ICM45686`, `MS5611`, `AS6221`(4ch), `INA219`(메인/점화), `DS3231`  
- 통신: u‑blox M10S(UBX), (LoRa 헤더 포함되어 있으나 현재 전송은 `printf` 기반)  
- 저장매체: SDMMC 4‑bit + FatFs (프로젝트 구성상 포함)  
- CRC: **CRC‑16‑CCITT (poly 0x1021, init 0xFFFF)**

### 인터럽트 & 입출력
- **UART3**: M10S GPS 수신(UBX)  
- **UART1**: 디버그/명령 수신(문자 `'D'`=사출, `'I'`=점화 강제)  
- **TIM4 CH4**: 부트 비프/테스트 PWM (BootTask에서 Start→PSC 스텝→Stop)  
- **ADC1(6ch)**: Cell1/Cell2/IGN/CR2032/`CDS_Top`/`CDS_Bottom`  
- **ADC3(내장)**: CPU 온도

> 주의: 현재 `HAL_UART_RxCpltCallback()`의 두 번째 분기 조건이 **USART3**로 중복되어 있어, UART1 디버그 명령 처리부가 의도와 다르게 동작할 수 있습니다. `(huart->Instance == USART1)`로 수정 권장.

---

## 2) 임무 상태기계(`StartMainTask`) — GPIO 액션

- **분리(어댑터)**: `Decoupling_flag`(UART `'D'`) 트리거 시 고도 스냅샷 후 **솔레노이드(GPIOE.10)** 1 s 구동 → `hyperion_separated = true`  
- **2차 점화(1채널)**: `hyperion_separated && !second_propellant_ignited` 이고  
  · (`고도 ≤ 200 m` **또는** `forceING`(UART `'I'`)) → **IGN MOSFET(GPIOE.7)** High 15 s → Low  
- **전자석 Off(낙하산 전개)**: 최대 고도 이후 **30 m 하강** 감지 **또는** (`고도 < 200 m` & 분리 완료) → **전자석(GPIOE.11/12)** Off

---

## 3) 텔레메트리 프로토콜

### 공통
- **헤더**: `'H' (0x48)`, `'S' (0x53)`  
- **ID**: `10=비행`, `11=자세`, `12=센서`  
- **LEN**: 페이로드 길이(바이트)  
- **CK_1/CK_2**: **CRC‑16‑CCITT**(poly 0x1021, init 0xFFFF), 계산 구간=`ID`부터 `LEN+payload` 바이트

### ID=10 비행 데이터(고도)
- **구조**: `H S | 10 | 2 | altitude_H | altitude_L | CK_1 | CK_2`  
- **단위**: cm 또는 정수 고도(코드에서는 `ms5611.altitude` 16‑bit 분리; 실제 단위는 상위 애플리케이션과 합의)

### ID=11 자세 데이터(가속/자이로/GPS)
- **구조**: `H S | 11 | 32 | … 32바이트 … | CK_1 | CK_2`  
- **필드(순서/바이트)**  
  4–7  : `accel_x` 정수부(2B) + 소수부(2B, ×1e‑4)  
  8–11 : `accel_y` 정수부 + 소수부  
  12–15: `accel_z` 정수부 + 소수부  
  16–19: `gyro_x`  정수부 + 소수부  
  20–23: `gyro_y`  정수부 + 소수부  
  24–27: `gyro_z`  정수부 + 소수부  
  28–31: `GPS 위도` 정수부(2B) + 소수부(2B) — 코드상은 `lat_f64`에서 정수/소수 분리  
  32–35: `GPS 경도` 정수부(2B) + 소수부(2B)

> 코드 구현: 부동소수값을 `정수부` + `(소수부×10000)`로 쪼개 **상위/하위 1바이트**씩 저장 후 전송.

### ID=12 센서 데이터
- **구조**: `H S | 12 | 27 | … 27바이트 … | CK_1 | CK_2`  
- **필드(순서/바이트)**  
  4–5  : Main Cell1 전압 `i`,`f` (V)  
  6–7  : Main Cell2 전압 `i`,`f` (V)  
  8–9  : 위성 소비 전류 **mA** 상/하위 바이트 (`INA219_main.current`)  
  10   : Main 배터리 잔량(%)  
  11–12: IGN 전압 `i`,`f` (V)  
  13   : 점화 소비 전류 **A** (`INA219_ign.current`는 코드에서 A 단위)  
  14   : IGN 배터리 잔량(%)  
  15–16: CPU 온도 `i`,`f` (°C)  
  17–24: Board/PMIC/LoRa/IGN‑FET 온도 `i`,`f` (°C) ×4  
  25   : **3.3 V PG** (현재 임시: `CR2032_BAT > 2.5 V` → 1)  
  26   : **1.8 V PG** (현재 임시: `CR2032_BAT > 1.5 V` → 1, 실제 PG 신호로 교체 예정)  
  27–28: `CDS_Top` 상/하위(ADC 원시)  
  29–30: 거리센서(ToF) cm 상/하위(필요 시 채움)

**지상국 파생값(권장 계산식)**
- **위성 소비 전력 [W]** = `(Cell1[V] + Cell2[V]) × (main_current[mA] / 1000)`  
- **점화 전력 [W]** = `IGN_V[V] × ign_current[A]`

### CRC‑16 샘플 코드(C)
```c
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
```

---

## 4) 센서/스케일링 상세

- **ADC 스케일** (`StartADC_Task`)  
  - `Main_Cell1/2 [V]` = `(adc1/ADC_MAX × 3.3) × 4.5`  
  - `IGN_BAT [V]`     = `(adc1/ADC_MAX × 3.3) × 4.5`  
  - `CR2032_BAT [V]`  = `(adc1/ADC_MAX × 3.3) × 2`  
  - `CDS_Top/Bottom`  = 원시값(0–65535), Bottom은 아날로그 버퍼 후 입력
- **MS5611 고도** (`StartMS5611_Task`)  
  - 초기 20회 평균으로 **기준압 hPa** 산출 → `h = 44330*(1 - (p/p0)^(1/5.255))`
- **온도계열** (`StartReadTemp_Task`)  
  - `AS6221` 4개: Board / PMIC / LoRa / IGN‑FET  
  - `CPU_Temp` = HAL 내장 보정 함수 사용(°C)
- **배터리 잔량(%)** (`StartINA219_Task`)  
  - 메인: `INA219_GetBatteryLife(8400, 6000)`  
  - 점화: `INA219_GetBatteryLife(12600, 9000)`

---

## 5) 통신 & 명령

- **GPS (UART3)**: 부팅 시 `UBX‑CFG‑PRT`, `‑MSG`, `‑RATE`, `‑CFG` 전송 후 `NAV‑POSLLH` 수신(체크섬 검증).  
- **디버그/명령 (UART1)**:  
  - `'D'` → `Decoupling_flag = true` (어댑터 분리 루틴 진입)  
  - `'I'` → `forceING = true` (강제 점화 조건 만족)

> 현재 콜백 내 분기 조건 중복(USART3)으로 인해 UART1 명령 처리 버그 가능성 있음 — 분기 조건을 USART1로 수정 권장.

---

## 6) 전송 주기(권장/현행)

- **권장**: 비행 100 Hz, 자세 50 Hz, 센서 10 Hz (링크 여유 고려해 조정)  
- **현행(디버그 출력)**: 각 바이트마다 `osDelay(10)`이 포함되어 약 **1 Hz 내외** 프레임률 — 실제 무선 전송 시 바이트 간 지연 제거 필요.

---

## 7) 빌드 & 포팅

1. **툴**: STM32CubeIDE / GCC, **FPU** 활성화(HW FP, DP)  
2. **Clock**: HSE/PLL 설정(보드 설정과 동일), SDMMC 4‑bit, DMA on  
3. **Peripheral Bindings**(소스 기준)  
   - `SPI1`: ICM45686 / MS5611  
   - `I2C2`: INA219(main/ign), AS6221×4  
   - `UART3`: GPS (MAX‑M10S), `UART1`: 디버그/명령  
   - `ADC1`: 6ch 외부, `ADC3`: 내부 온도  
   - `TIM4 CH4`: PWM(Buzzer/테스트)  
4. **중요 매크로/주소**:  
   - AS6221 주소: `0x48/0x49/0x4A/0x4B`(7‑bit), 코드에선 **왼시프트(<<1)** 적용 주의  
   - CRC 구현: 위 섹션 참고

---

## 8) 로깅/지상국 파서 힌트

- **CSV 디버그 출력**: 현재 `printf`로 바이트를 `,`로 구분해 한 줄 전송.  
- **권장 파서 로직**:  
  1) 스트림에서 `'H''S'` 동기 → 2) `ID`/`LEN` 확인 → 3) `LEN`만큼 수집 → 4) CRC 검증 → 5) 패킷별 디코딩  
- **전력 계산**: 섹션 3의 파생값 공식 활용.

---

## 9) 알려진 이슈 / TODO

- [ ] `HAL_UART_RxCpltCallback()`에서 **UART1** 분기 수정(현재 USART3로 중복)  
- [ ] 센서 패킷의 **PG 비트** 실제 레귤레이터 PG 핀과 매핑(임시값 제거)  
- [ ] `sendDataTask`의 **바이트 간 `osDelay(10)` 제거** 및 LoRa/STX3 전송 함수로 교체  
- [ ] IMU 임계값(가속/자이로) **실비행 로그 기반 튜닝**  
- [ ] SD/FatFs 텔레메트리 로깅 옵션(비행 후 복구용)

---

## 10) 라이선스 / 크레딧

- 라이선스: 프로젝트 루트의 `LICENSE` 참조(없을 경우 AS‑IS).  
- Contributors: HyperionSAT 팀 일동.

_문서 버전: 2025-08-24_
