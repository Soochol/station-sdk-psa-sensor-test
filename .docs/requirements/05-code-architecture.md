# Station SDK - PSA Sensor Test - Code Architecture Design

## 1. Overview

### 1.1 시스템 개요

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PSA Sensor Test Firmware                             │
│                         STM32H723VGT6 @ 384MHz                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │   Protocol  │    │    Test     │    │   Sensor    │    │     HAL     │  │
│  │    Layer    │───▶│   Runner    │───▶│   Manager   │───▶│   Wrapper   │  │
│  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘  │
│        │                                      │                   │         │
│        │                               ┌──────┴──────┐            │         │
│        ▼                               ▼             ▼            ▼         │
│  ┌───────────┐                  ┌──────────┐ ┌──────────┐  ┌───────────┐   │
│  │   Frame   │                  │ MLX90640 │ │ VL53L0X  │  │  CubeMX   │   │
│  │  Handler  │                  │  Driver  │ │  Driver  │  │    HAL    │   │
│  └───────────┘                  └──────────┘ └──────────┘  └───────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 설계 원칙

| 원칙 | 설명 |
|------|------|
| **계층적 구조** | Protocol → Test → Sensor → HAL 순서의 명확한 의존성 |
| **센서 추상화** | 통일된 드라이버 인터페이스로 센서 확장 용이 |
| **관심사 분리** | 통신, 테스트 로직, 센서 제어를 독립적인 모듈로 분리 |
| **테스트 용이성** | HAL 래퍼를 통한 하드웨어 의존성 분리 |

---

## 2. Layer Architecture

### 2.1 Layer Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            Application Layer                                 │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                           main.c                                    │    │
│  │  - 시스템 초기화 및 메인 루프                                        │    │
│  │  - 모듈 초기화 조율                                                  │    │
│  └────────────────────────────────────────────────────────────────────┘    │
├─────────────────────────────────────────────────────────────────────────────┤
│                            Protocol Layer                                    │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────────┐      │
│  │   frame.c/h      │  │  commands.c/h    │  │   protocol.c/h       │      │
│  │  - 프레임 파싱    │  │  - 명령 디스패치  │  │  - 프로토콜 상수     │      │
│  │  - 프레임 빌드    │  │  - 응답 생성     │  │  - 초기화            │      │
│  │  - CRC 계산      │  │  - 에러 처리     │  │                      │      │
│  └──────────────────┘  └──────────────────┘  └──────────────────────┘      │
├─────────────────────────────────────────────────────────────────────────────┤
│                            Test Layer                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                       test_runner.c/h                                │   │
│  │  - 테스트 시퀀스 실행 (TEST_ALL, TEST_SINGLE)                        │   │
│  │  - 결과 수집 및 리포트 생성                                          │   │
│  │  - Fail-fast 로직 (첫 번째 실패 시 중단)                             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────────┤
│                            Sensor Layer                                      │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────────┐      │
│  │ sensor_manager   │  │   mlx90640.c/h   │  │    vl53l0x.c/h       │      │
│  │  - 센서 등록     │  │  - Init/Deinit   │  │  - Init/Deinit       │      │
│  │  - ID로 조회     │  │  - Read Temp     │  │  - Read Distance     │      │
│  │  - 순회 지원     │  │  - Validate      │  │  - Validate          │      │
│  └──────────────────┘  └──────────────────┘  └──────────────────────┘      │
├─────────────────────────────────────────────────────────────────────────────┤
│                            HAL Wrapper Layer                                 │
│  ┌──────────────────────────────┐  ┌──────────────────────────────┐        │
│  │       uart_handler.c/h       │  │       i2c_handler.c/h        │        │
│  │  - UART 송수신               │  │  - I2C Read/Write            │        │
│  │  - 인터럽트 기반 RX          │  │  - Device Ready 확인         │        │
│  │  - 링 버퍼 관리              │  │  - I2C1/I2C4 추상화          │        │
│  └──────────────────────────────┘  └──────────────────────────────┘        │
├─────────────────────────────────────────────────────────────────────────────┤
│                            Hardware Abstraction Layer                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    STM32H7xx HAL Driver (CubeMX Generated)           │   │
│  │  - HAL_I2C, HAL_UART, HAL_GPIO, SystemClock, MPU                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 의존성 규칙

```
Application
    │
    ├──▶ Protocol
    │        │
    │        └──▶ Test Runner
    │                  │
    │                  └──▶ Sensor Manager
    │                            │
    │                            ├──▶ MLX90640 Driver
    │                            │         │
    │                            │         └──▶ I2C Handler (I2C4)
    │                            │
    │                            └──▶ VL53L0X Driver
    │                                      │
    │                                      └──▶ I2C Handler (I2C1)
    │
    └──▶ UART Handler
              │
              └──▶ HAL_UART
```

**규칙:**
- 상위 레이어는 하위 레이어에만 의존
- 동일 레이어 간 의존 금지
- HAL 직접 사용 금지 (Wrapper를 통해 접근)

---

## 3. Module Specifications

### 3.1 Protocol Module

#### 3.1.1 protocol.h

```c
/**
 * @file protocol.h
 * @brief UART 통신 프로토콜 상수 및 타입 정의
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Protocol Constants                                                         */
/*============================================================================*/

#define PROTOCOL_STX                0x02
#define PROTOCOL_ETX                0x03
#define PROTOCOL_MAX_PAYLOAD        64
#define PROTOCOL_RX_BUFFER_SIZE     128

/*============================================================================*/
/* Command Codes                                                              */
/*============================================================================*/

typedef enum {
    /* Host → MCU (Request) */
    CMD_PING                = 0x01,
    CMD_TEST_ALL            = 0x10,
    CMD_TEST_SINGLE         = 0x11,
    CMD_GET_SENSOR_LIST     = 0x12,
    CMD_SET_SPEC            = 0x20,
    CMD_GET_SPEC            = 0x21,

    /* MCU → Host (Response) */
    CMD_PONG                = 0x01,  /* Same as PING */
    CMD_TEST_RESULT         = 0x80,
    CMD_SENSOR_LIST         = 0x81,
    CMD_SPEC_ACK            = 0x82,
    CMD_SPEC_DATA           = 0x83,
    CMD_NAK                 = 0xFE,
} CommandCode_t;

/*============================================================================*/
/* Error Codes                                                                */
/*============================================================================*/

typedef enum {
    ERR_UNKNOWN_CMD         = 0x01,
    ERR_INVALID_SENSOR_ID   = 0x02,
    ERR_INVALID_PAYLOAD     = 0x03,
    ERR_BUSY                = 0x04,
    ERR_CRC_FAIL            = 0x05,
} ErrorCode_t;

/*============================================================================*/
/* Functions                                                                  */
/*============================================================================*/

void Protocol_Init(void);
void Protocol_Process(void);

#endif /* PROTOCOL_H */
```

#### 3.1.2 frame.h

```c
/**
 * @file frame.h
 * @brief 프레임 파싱/빌드 유틸리티
 */

#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol/protocol.h"

/*============================================================================*/
/* Types                                                                      */
/*============================================================================*/

typedef struct {
    uint8_t     cmd;
    uint8_t     payload[PROTOCOL_MAX_PAYLOAD];
    uint8_t     payload_len;
} Frame_t;

typedef enum {
    FRAME_PARSE_OK          = 0,
    FRAME_PARSE_INCOMPLETE  = 1,
    FRAME_PARSE_CRC_ERROR   = 2,
    FRAME_PARSE_FORMAT_ERR  = 3,
} FrameParseResult_t;

/*============================================================================*/
/* Functions                                                                  */
/*============================================================================*/

/**
 * @brief 바이트 버퍼에서 프레임 파싱
 * @param buffer 입력 버퍼
 * @param len 버퍼 길이
 * @param frame 출력 프레임 구조체
 * @param consumed 소비된 바이트 수 (출력)
 * @return 파싱 결과
 */
FrameParseResult_t Frame_Parse(const uint8_t* buffer, uint16_t len,
                                Frame_t* frame, uint16_t* consumed);

/**
 * @brief 프레임을 바이트 버퍼로 직렬화
 * @param frame 입력 프레임
 * @param buffer 출력 버퍼
 * @return 빌드된 바이트 수
 */
uint16_t Frame_Build(const Frame_t* frame, uint8_t* buffer);

/**
 * @brief CRC 계산 (XOR)
 * @param data 데이터 버퍼
 * @param len 데이터 길이
 * @return CRC 값
 */
uint8_t Frame_CalculateCRC(const uint8_t* data, uint8_t len);

#endif /* FRAME_H */
```

#### 3.1.3 commands.h

```c
/**
 * @file commands.h
 * @brief 명령 핸들러 및 디스패처
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include "protocol/frame.h"

/*============================================================================*/
/* Types                                                                      */
/*============================================================================*/

typedef void (*CommandHandler_t)(const Frame_t* request, Frame_t* response);

/*============================================================================*/
/* Functions                                                                  */
/*============================================================================*/

/**
 * @brief 명령 핸들러 초기화
 */
void Commands_Init(void);

/**
 * @brief 수신 프레임 처리 및 응답 생성
 * @param request 수신 프레임
 * @param response 응답 프레임 (출력)
 * @return true: 응답 필요, false: 응답 불필요
 */
bool Commands_Process(const Frame_t* request, Frame_t* response);

#endif /* COMMANDS_H */
```

---

### 3.2 Sensor Module

#### 3.2.1 sensor_types.h

```c
/**
 * @file sensor_types.h
 * @brief 센서 공통 타입 정의
 */

#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/*============================================================================*/
/* Sensor IDs                                                                 */
/*============================================================================*/

typedef enum {
    SENSOR_ID_NONE          = 0x00,
    SENSOR_ID_MLX90640      = 0x01,
    SENSOR_ID_VL53L0X       = 0x02,
    /* Future sensors */
    /* SENSOR_ID_BME280     = 0x03, */
    SENSOR_ID_MAX           = 0x03,
} SensorID_t;

/*============================================================================*/
/* Test Status                                                                */
/*============================================================================*/

typedef enum {
    STATUS_PASS             = 0x00,
    STATUS_FAIL_NO_ACK      = 0x01,
    STATUS_FAIL_TIMEOUT     = 0x02,
    STATUS_FAIL_INVALID     = 0x03,
    STATUS_FAIL_INIT        = 0x04,
    STATUS_FAIL_NO_SPEC     = 0x05,
    STATUS_NOT_TESTED       = 0xFF,
} TestStatus_t;

/*============================================================================*/
/* Sensor Specification (Union for different sensor types)                    */
/*============================================================================*/

typedef union {
    /* MLX90640 Spec */
    struct {
        int16_t     target_temp;    /* ×100 °C (-40.00 ~ 300.00) */
        uint16_t    tolerance;      /* ×100 °C */
    } mlx90640;

    /* VL53L0X Spec */
    struct {
        uint16_t    target_dist;    /* mm (30 ~ 2000) */
        uint16_t    tolerance;      /* mm */
    } vl53l0x;

    /* Raw bytes for serialization */
    uint8_t raw[4];
} SensorSpec_t;

/*============================================================================*/
/* Sensor Result (Union for different sensor types)                           */
/*============================================================================*/

typedef union {
    /* MLX90640 Result */
    struct {
        int16_t     max_temp;       /* ×100 °C - 측정된 최대 픽셀 온도 */
        int16_t     target;         /* ×100 °C - 스펙 목표 온도 */
        uint16_t    tolerance;      /* ×100 °C - 스펙 허용 오차 */
        uint16_t    diff;           /* ×100 °C - |max_temp - target| */
    } mlx90640;

    /* VL53L0X Result */
    struct {
        uint16_t    measured;       /* mm - 측정된 거리 */
        uint16_t    target;         /* mm - 스펙 목표 거리 */
        uint16_t    tolerance;      /* mm - 스펙 허용 오차 */
        uint16_t    diff;           /* mm - |measured - target| */
    } vl53l0x;

    /* Raw bytes for serialization */
    uint8_t raw[8];
} SensorResult_t;

#endif /* SENSOR_TYPES_H */
```

#### 3.2.2 sensor_manager.h

```c
/**
 * @file sensor_manager.h
 * @brief 센서 드라이버 인터페이스 및 관리자
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "sensors/sensor_types.h"

/*============================================================================*/
/* Configuration                                                              */
/*============================================================================*/

#define MAX_SENSORS             8

/*============================================================================*/
/* Sensor Driver Interface                                                    */
/*============================================================================*/

typedef struct SensorDriver {
    /* Identification */
    SensorID_t      id;
    const char*     name;

    /* Lifecycle */
    HAL_StatusTypeDef   (*init)(void);
    void                (*deinit)(void);

    /* Specification */
    void                (*set_spec)(const SensorSpec_t* spec);
    void                (*get_spec)(SensorSpec_t* spec);
    bool                (*has_spec)(void);

    /* Test Execution */
    TestStatus_t        (*run_test)(SensorResult_t* result);

    /* Serialization */
    uint8_t             (*serialize_spec)(const SensorSpec_t* spec, uint8_t* buffer);
    uint8_t             (*parse_spec)(const uint8_t* buffer, SensorSpec_t* spec);
    uint8_t             (*serialize_result)(const SensorResult_t* result, uint8_t* buffer);
} SensorDriver_t;

/*============================================================================*/
/* Sensor Manager API                                                         */
/*============================================================================*/

/**
 * @brief 센서 관리자 초기화 (모든 드라이버 등록)
 */
void SensorManager_Init(void);

/**
 * @brief 센서 드라이버 등록
 * @param driver 드라이버 포인터
 * @return HAL_OK: 성공, HAL_ERROR: 실패 (가득 참)
 */
HAL_StatusTypeDef SensorManager_Register(const SensorDriver_t* driver);

/**
 * @brief ID로 센서 드라이버 조회
 * @param id 센서 ID
 * @return 드라이버 포인터 또는 NULL
 */
const SensorDriver_t* SensorManager_GetByID(SensorID_t id);

/**
 * @brief 인덱스로 센서 드라이버 조회
 * @param index 센서 인덱스 (0-based)
 * @return 드라이버 포인터 또는 NULL
 */
const SensorDriver_t* SensorManager_GetByIndex(uint8_t index);

/**
 * @brief 등록된 센서 수 조회
 * @return 센서 수
 */
uint8_t SensorManager_GetCount(void);

/**
 * @brief 센서 ID 유효성 검사
 * @param id 센서 ID
 * @return true: 유효, false: 무효
 */
bool SensorManager_IsValidID(SensorID_t id);

#endif /* SENSOR_MANAGER_H */
```

#### 3.2.3 mlx90640.h

```c
/**
 * @file mlx90640.h
 * @brief MLX90640 IR 열화상 센서 드라이버
 *
 * 하드웨어 연결:
 *   - I2C4 (PB8: SCL, PB9: SDA)
 *   - I2C Address: 0x33 (7-bit)
 */

#ifndef MLX90640_H
#define MLX90640_H

#include "sensors/sensor_manager.h"

/*============================================================================*/
/* Hardware Configuration                                                     */
/*============================================================================*/

#define MLX90640_I2C_ADDR           0x33
#define MLX90640_RESOLUTION_X       32
#define MLX90640_RESOLUTION_Y       24
#define MLX90640_PIXEL_COUNT        (MLX90640_RESOLUTION_X * MLX90640_RESOLUTION_Y)

/*============================================================================*/
/* Exported Driver Instance                                                   */
/*============================================================================*/

extern const SensorDriver_t MLX90640_Driver;

#endif /* MLX90640_H */
```

#### 3.2.4 vl53l0x.h

```c
/**
 * @file vl53l0x.h
 * @brief VL53L0X ToF 거리 센서 드라이버
 *
 * 하드웨어 연결:
 *   - I2C1 (PB6: SCL, PB7: SDA)
 *   - I2C Address: 0x29 (7-bit)
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include "sensors/sensor_manager.h"

/*============================================================================*/
/* Hardware Configuration                                                     */
/*============================================================================*/

#define VL53L0X_I2C_ADDR            0x29
#define VL53L0X_RANGE_MIN_MM        30
#define VL53L0X_RANGE_MAX_MM        2000

/*============================================================================*/
/* Exported Driver Instance                                                   */
/*============================================================================*/

extern const SensorDriver_t VL53L0X_Driver;

#endif /* VL53L0X_H */
```

---

### 3.3 Test Module

#### 3.3.1 test_runner.h

```c
/**
 * @file test_runner.h
 * @brief 테스트 시퀀스 실행 관리자
 */

#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include "sensors/sensor_types.h"
#include "sensors/sensor_manager.h"

/*============================================================================*/
/* Types                                                                      */
/*============================================================================*/

/**
 * @brief 개별 센서 테스트 결과
 */
typedef struct {
    SensorID_t      sensor_id;
    TestStatus_t    status;
    uint8_t         data_len;
    SensorResult_t  result;
} TestResult_t;

/**
 * @brief 전체 테스트 리포트
 */
typedef struct {
    uint8_t         count;
    TestResult_t    results[MAX_SENSORS];
} TestReport_t;

/*============================================================================*/
/* Functions                                                                  */
/*============================================================================*/

/**
 * @brief 테스트 러너 초기화
 */
void TestRunner_Init(void);

/**
 * @brief 모든 등록된 센서 테스트 실행
 * @param report 테스트 리포트 (출력)
 * @note 첫 번째 실패 시 나머지 센서는 NOT_TESTED로 표시
 */
void TestRunner_RunAll(TestReport_t* report);

/**
 * @brief 특정 센서만 테스트 실행
 * @param id 센서 ID
 * @param report 테스트 리포트 (출력, count=1)
 */
void TestRunner_RunSingle(SensorID_t id, TestReport_t* report);

/**
 * @brief 테스트 리포트를 바이트 버퍼로 직렬화
 * @param report 테스트 리포트
 * @param buffer 출력 버퍼
 * @return 직렬화된 바이트 수
 */
uint16_t TestRunner_SerializeReport(const TestReport_t* report, uint8_t* buffer);

#endif /* TEST_RUNNER_H */
```

---

### 3.4 HAL Wrapper Module

#### 3.4.1 uart_handler.h

```c
/**
 * @file uart_handler.h
 * @brief UART 통신 핸들러
 *
 * 하드웨어: UART4 (PA11: RX, PA12: TX)
 * 설정: 115200-8-N-1
 */

#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

/*============================================================================*/
/* Configuration                                                              */
/*============================================================================*/

#define UART_RX_BUFFER_SIZE     256
#define UART_TX_BUFFER_SIZE     256

/*============================================================================*/
/* Types                                                                      */
/*============================================================================*/

typedef void (*UART_RxCallback_t)(const uint8_t* data, uint16_t len);

/*============================================================================*/
/* Functions                                                                  */
/*============================================================================*/

/**
 * @brief UART 핸들러 초기화
 * @param huart HAL UART 핸들 포인터
 * @return HAL_OK: 성공
 */
HAL_StatusTypeDef UART_Handler_Init(UART_HandleTypeDef* huart);

/**
 * @brief 수신 콜백 설정
 * @param callback 콜백 함수 포인터
 */
void UART_Handler_SetRxCallback(UART_RxCallback_t callback);

/**
 * @brief 데이터 송신 (블로킹)
 * @param data 송신 데이터
 * @param len 데이터 길이
 * @param timeout_ms 타임아웃 (ms)
 * @return HAL_OK: 성공
 */
HAL_StatusTypeDef UART_Handler_Send(const uint8_t* data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief 수신 버퍼에서 데이터 읽기
 * @param buffer 출력 버퍼
 * @param max_len 최대 읽기 크기
 * @return 읽은 바이트 수
 */
uint16_t UART_Handler_Read(uint8_t* buffer, uint16_t max_len);

/**
 * @brief 수신 데이터 처리 (메인 루프에서 호출)
 */
void UART_Handler_Process(void);

/**
 * @brief UART 인터럽트 핸들러 (HAL 콜백에서 호출)
 */
void UART_Handler_IRQHandler(void);

#endif /* UART_HANDLER_H */
```

#### 3.4.2 i2c_handler.h

```c
/**
 * @file i2c_handler.h
 * @brief I2C 통신 핸들러
 *
 * 하드웨어:
 *   - I2C1: VL53L0X (PB6: SCL, PB7: SDA)
 *   - I2C4: MLX90640 (PB8: SCL, PB9: SDA)
 */

#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

/*============================================================================*/
/* Types                                                                      */
/*============================================================================*/

typedef enum {
    I2C_BUS_1 = 0,      /* I2C1 - VL53L0X */
    I2C_BUS_4 = 1,      /* I2C4 - MLX90640 */
    I2C_BUS_COUNT
} I2C_BusID_t;

/*============================================================================*/
/* Functions                                                                  */
/*============================================================================*/

/**
 * @brief I2C 핸들러 초기화
 * @param bus_id 버스 ID
 * @param hi2c HAL I2C 핸들 포인터
 * @return HAL_OK: 성공
 */
HAL_StatusTypeDef I2C_Handler_Init(I2C_BusID_t bus_id, I2C_HandleTypeDef* hi2c);

/**
 * @brief 디바이스 응답 확인
 * @param bus_id 버스 ID
 * @param dev_addr 7-bit 디바이스 주소
 * @param timeout_ms 타임아웃 (ms)
 * @return HAL_OK: 응답 있음, HAL_ERROR/HAL_TIMEOUT: 없음
 */
HAL_StatusTypeDef I2C_Handler_IsDeviceReady(I2C_BusID_t bus_id, uint8_t dev_addr,
                                             uint32_t timeout_ms);

/**
 * @brief 레지스터 읽기 (16-bit 주소)
 * @param bus_id 버스 ID
 * @param dev_addr 7-bit 디바이스 주소
 * @param reg_addr 16-bit 레지스터 주소
 * @param data 출력 버퍼
 * @param len 읽기 길이
 * @param timeout_ms 타임아웃 (ms)
 * @return HAL_OK: 성공
 */
HAL_StatusTypeDef I2C_Handler_Read16(I2C_BusID_t bus_id, uint8_t dev_addr,
                                      uint16_t reg_addr, uint8_t* data,
                                      uint16_t len, uint32_t timeout_ms);

/**
 * @brief 레지스터 쓰기 (16-bit 주소)
 * @param bus_id 버스 ID
 * @param dev_addr 7-bit 디바이스 주소
 * @param reg_addr 16-bit 레지스터 주소
 * @param data 입력 버퍼
 * @param len 쓰기 길이
 * @param timeout_ms 타임아웃 (ms)
 * @return HAL_OK: 성공
 */
HAL_StatusTypeDef I2C_Handler_Write16(I2C_BusID_t bus_id, uint8_t dev_addr,
                                       uint16_t reg_addr, const uint8_t* data,
                                       uint16_t len, uint32_t timeout_ms);

/**
 * @brief 레지스터 읽기 (8-bit 주소)
 * @param bus_id 버스 ID
 * @param dev_addr 7-bit 디바이스 주소
 * @param reg_addr 8-bit 레지스터 주소
 * @param data 출력 버퍼
 * @param len 읽기 길이
 * @param timeout_ms 타임아웃 (ms)
 * @return HAL_OK: 성공
 */
HAL_StatusTypeDef I2C_Handler_Read8(I2C_BusID_t bus_id, uint8_t dev_addr,
                                     uint8_t reg_addr, uint8_t* data,
                                     uint16_t len, uint32_t timeout_ms);

/**
 * @brief 레지스터 쓰기 (8-bit 주소)
 * @param bus_id 버스 ID
 * @param dev_addr 7-bit 디바이스 주소
 * @param reg_addr 8-bit 레지스터 주소
 * @param data 입력 버퍼
 * @param len 쓰기 길이
 * @param timeout_ms 타임아웃 (ms)
 * @return HAL_OK: 성공
 */
HAL_StatusTypeDef I2C_Handler_Write8(I2C_BusID_t bus_id, uint8_t dev_addr,
                                      uint8_t reg_addr, const uint8_t* data,
                                      uint16_t len, uint32_t timeout_ms);

#endif /* I2C_HANDLER_H */
```

---

### 3.5 Configuration Module

#### 3.5.1 config.h

```c
/**
 * @file config.h
 * @brief 전역 설정 및 상수 정의
 */

#ifndef CONFIG_H
#define CONFIG_H

/*============================================================================*/
/* Firmware Version                                                           */
/*============================================================================*/

#define FW_VERSION_MAJOR        1
#define FW_VERSION_MINOR        0
#define FW_VERSION_PATCH        0

/*============================================================================*/
/* Timeout Settings (milliseconds)                                            */
/*============================================================================*/

#define TIMEOUT_SENSOR_TEST_MS      5000    /* 센서당 테스트 타임아웃 */
#define TIMEOUT_UART_TX_MS          1000    /* UART 송신 타임아웃 */
#define TIMEOUT_I2C_MS              100     /* I2C 통신 타임아웃 */

/*============================================================================*/
/* Sensor I2C Configuration                                                   */
/*============================================================================*/

/* MLX90640 - I2C4 */
#define MLX90640_I2C_BUS            I2C_BUS_4
#define MLX90640_I2C_ADDR           0x33

/* VL53L0X - I2C1 */
#define VL53L0X_I2C_BUS             I2C_BUS_1
#define VL53L0X_I2C_ADDR            0x29

/*============================================================================*/
/* Protocol Settings                                                          */
/*============================================================================*/

#define PROTOCOL_MAX_PAYLOAD        64
#define PROTOCOL_RX_BUFFER_SIZE     128

/*============================================================================*/
/* Sensor Manager Settings                                                    */
/*============================================================================*/

#define MAX_SENSORS                 8

/*============================================================================*/
/* Debug Configuration                                                        */
/*============================================================================*/

#define DEBUG_ENABLED               0
#define DEBUG_UART                  huart4

/*============================================================================*/
/* MLX90640 Configuration                                                     */
/*============================================================================*/

#define MLX90640_REFRESH_RATE       4       /* 0.5Hz=0, 1Hz=1, 2Hz=2, 4Hz=3, 8Hz=4, 16Hz=5, 32Hz=6, 64Hz=7 */
#define MLX90640_ADC_RESOLUTION     19      /* 16=16bit, 17=17bit, 18=18bit, 19=19bit */

/*============================================================================*/
/* VL53L0X Configuration                                                      */
/*============================================================================*/

#define VL53L0X_MEASUREMENT_MODE    1       /* 0=Single, 1=Continuous */
#define VL53L0X_TIMING_BUDGET_US    33000   /* Measurement timing budget */

#endif /* CONFIG_H */
```

---

## 4. Data Flow Diagrams

### 4.1 프레임 수신 및 처리 흐름

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Frame Reception Flow                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Host PC                                                                     │
│     │                                                                        │
│     │  [02][LEN][CMD][PAYLOAD][CRC][03]                                     │
│     ▼                                                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                      UART4 RX Interrupt                              │    │
│  │  - 바이트 단위 수신                                                  │    │
│  │  - 링 버퍼에 저장                                                    │    │
│  └───────────────────────────┬─────────────────────────────────────────┘    │
│                              │                                               │
│                              ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     UART_Handler_Process()                           │    │
│  │  - 링 버퍼에서 데이터 읽기                                           │    │
│  │  - 콜백 호출                                                         │    │
│  └───────────────────────────┬─────────────────────────────────────────┘    │
│                              │                                               │
│                              ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                      Frame_Parse()                                   │    │
│  │  - STX 검색                                                          │    │
│  │  - LENGTH 읽기                                                       │    │
│  │  - CMD + PAYLOAD 읽기                                                │    │
│  │  - CRC 검증                                                          │    │
│  │  - ETX 확인                                                          │    │
│  └───────────────────────────┬─────────────────────────────────────────┘    │
│                              │                                               │
│                              ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Commands_Process()                                │    │
│  │  - CMD에 따라 핸들러 디스패치                                        │    │
│  │  - 응답 프레임 생성                                                  │    │
│  └───────────────────────────┬─────────────────────────────────────────┘    │
│                              │                                               │
│                              ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                      Frame_Build()                                   │    │
│  │  - 응답 프레임 직렬화                                                │    │
│  └───────────────────────────┬─────────────────────────────────────────┘    │
│                              │                                               │
│                              ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    UART_Handler_Send()                               │    │
│  │  - 응답 송신                                                         │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 TEST_ALL 시퀀스 흐름

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         TEST_ALL Sequence Flow                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Commands_Process(CMD_TEST_ALL)                                              │
│       │                                                                      │
│       ▼                                                                      │
│  ┌───────────────────────────────────────────────────────────────────┐      │
│  │                    TestRunner_RunAll()                             │      │
│  └───────────────────────────┬───────────────────────────────────────┘      │
│                              │                                               │
│       ┌──────────────────────┼──────────────────────┐                       │
│       │                      │                      │                       │
│       ▼                      ▼                      ▼                       │
│  ┌─────────────┐      ┌─────────────┐      ┌─────────────┐                  │
│  │  Sensor[0]  │      │  Sensor[1]  │      │  Sensor[N]  │                  │
│  │  MLX90640   │      │   VL53L0X   │      │    ...      │                  │
│  └──────┬──────┘      └──────┬──────┘      └──────┬──────┘                  │
│         │                    │                    │                          │
│         ▼                    │                    │                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     MLX90640_Driver.run_test()                       │    │
│  ├─────────────────────────────────────────────────────────────────────┤    │
│  │  1. has_spec() 확인                                                  │    │
│  │     └─ false → return STATUS_FAIL_NO_SPEC                           │    │
│  │                                                                      │    │
│  │  2. init()                                                           │    │
│  │     └─ I2C_Handler_IsDeviceReady() 실패 → return STATUS_FAIL_NO_ACK │    │
│  │     └─ 초기화 실패 → return STATUS_FAIL_INIT                        │    │
│  │                                                                      │    │
│  │  3. Read Temperature                                                 │    │
│  │     └─ 5초 타임아웃 → return STATUS_FAIL_TIMEOUT                    │    │
│  │                                                                      │    │
│  │  4. Validate                                                         │    │
│  │     └─ |max_temp - target| > tolerance → return STATUS_FAIL_INVALID │    │
│  │                                                                      │    │
│  │  5. deinit()                                                         │    │
│  │                                                                      │    │
│  │  6. return STATUS_PASS                                               │    │
│  └──────────────────────────────────┬──────────────────────────────────┘    │
│                                     │                                        │
│                              ┌──────┴──────┐                                 │
│                              │             │                                 │
│                          PASS │             │ FAIL                           │
│                              │             │                                 │
│                              ▼             ▼                                 │
│                      다음 센서 테스트    테스트 중단                         │
│                              │         (나머지: NOT_TESTED)                  │
│                              ▼                                               │
│                    ┌─────────────────┐                                       │
│                    │ TestReport 생성 │                                       │
│                    └─────────────────┘                                       │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. State Machine Diagrams

### 5.1 프레임 파서 상태 머신

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       Frame Parser State Machine                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│                           ┌─────────────┐                                    │
│                           │   IDLE      │◄──────────────────┐               │
│                           │  STX 대기   │                    │               │
│                           └──────┬──────┘                    │               │
│                                  │ STX (0x02)                │               │
│                                  ▼                           │               │
│                           ┌─────────────┐                    │               │
│                           │  GOT_STX    │                    │               │
│                           │ LENGTH 대기 │                    │               │
│                           └──────┬──────┘                    │               │
│                                  │ byte                      │               │
│                                  ▼                           │               │
│                           ┌─────────────┐                    │               │
│                           │ GOT_LENGTH  │                    │               │
│                           │  DATA 수신  │                    │               │
│                           └──────┬──────┘                    │               │
│                                  │ LENGTH bytes received     │               │
│                                  ▼                           │               │
│                           ┌─────────────┐                    │               │
│                           │  GOT_DATA   │                    │               │
│                           │  CRC 대기   │                    │               │
│                           └──────┬──────┘                    │               │
│                                  │ byte                      │               │
│                             ┌────┴────┐                      │               │
│                        CRC OK│         │CRC FAIL             │               │
│                             ▼         ▼                      │               │
│                      ┌───────────┐ ┌───────────┐             │               │
│                      │ GOT_CRC   │ │CRC_ERROR  │─────────────┘               │
│                      │ ETX 대기  │ └───────────┘                             │
│                      └─────┬─────┘                                           │
│                            │ ETX (0x03)                                      │
│                            ▼                                                 │
│                      ┌───────────┐                                           │
│                      │ COMPLETE  │──────────────────────────────┐            │
│                      │ 파싱 완료 │                              │            │
│                      └───────────┘                              ▼            │
│                                                          Return Frame        │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.2 테스트 러너 상태 머신

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Test Runner State Machine                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│                              ┌──────────┐                                    │
│                              │   IDLE   │                                    │
│                              │  대기 중  │                                    │
│                              └────┬─────┘                                    │
│                                   │ TEST_ALL / TEST_SINGLE                   │
│                                   ▼                                          │
│                              ┌──────────┐                                    │
│                              │  CHECK   │                                    │
│                              │스펙 확인 │                                    │
│                              └────┬─────┘                                    │
│                                   │ all sensors have spec                    │
│                    ┌──────────────┴──────────────┐                           │
│                 NO │                             │ YES                       │
│                    ▼                             ▼                           │
│              ┌──────────┐                 ┌──────────┐                       │
│              │  ERROR   │                 │   RUN    │                       │
│              │ NO_SPEC  │                 │  테스트   │                       │
│              └──────────┘                 └────┬─────┘                       │
│                    │                           │                             │
│                    │               ┌───────────┴───────────┐                 │
│                    │               │                       │                 │
│                    │           PASS│                   FAIL│                 │
│                    │               ▼                       ▼                 │
│                    │        ┌───────────┐          ┌───────────┐            │
│                    │        │   NEXT    │          │  ABORT    │            │
│                    │        │ 다음 센서 │          │   중단    │            │
│                    │        └─────┬─────┘          └─────┬─────┘            │
│                    │              │                      │                   │
│                    │              │ more sensors?        │                   │
│                    │         ┌────┴────┐                 │                   │
│                    │      YES│         │NO               │                   │
│                    │         ▼         │                 │                   │
│                    │   ┌──────────┐    │                 │                   │
│                    │   │   RUN    │────┘                 │                   │
│                    │   └──────────┘                      │                   │
│                    │                                     │                   │
│                    │         ┌───────────────────────────┘                   │
│                    │         │                                               │
│                    ▼         ▼                                               │
│              ┌───────────────────┐                                           │
│              │     COMPLETE      │                                           │
│              │   리포트 생성     │                                           │
│              └─────────┬─────────┘                                           │
│                        │                                                     │
│                        ▼                                                     │
│                 Return to IDLE                                               │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. Memory Layout

### 6.1 STM32H723VGT6 메모리 맵

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          STM32H723VGT6 Memory Map                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Address          Size        Name          Usage                            │
│  ─────────────────────────────────────────────────────────────────────────  │
│  0x0800_0000     1 MB        FLASH         Code & Constants                 │
│                                             - .text (code)                   │
│                                             - .rodata (const)                │
│                                             - Vector Table                   │
│  ─────────────────────────────────────────────────────────────────────────  │
│  0x2000_0000     128 KB      DTCMRAM       Fast Data Access                 │
│                                             - Critical variables             │
│                                             - Ring buffers (optional)        │
│  ─────────────────────────────────────────────────────────────────────────  │
│  0x2400_0000     320 KB      AXI SRAM      Main Data Memory                 │
│                                             - .data (initialized)            │
│                                             - .bss (uninitialized)           │
│                                             - Heap                           │
│                                             - Stack                          │
│                                             - MLX90640 Frame Buffer          │
│  ─────────────────────────────────────────────────────────────────────────  │
│  0x3000_0000     32 KB       SRAM1/2       DMA Buffers (if needed)          │
│  ─────────────────────────────────────────────────────────────────────────  │
│  0x3800_0000     16 KB       SRAM4         Backup / Low-power               │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 6.2 주요 데이터 구조 크기

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Data Structure Sizes                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Structure                    Size (bytes)    Description                    │
│  ─────────────────────────────────────────────────────────────────────────  │
│  Frame_t                      66              1 + 64 + 1                     │
│  SensorSpec_t                 4               Union (max: 4 bytes)           │
│  SensorResult_t               8               Union (max: 8 bytes)           │
│  TestResult_t                 12              1 + 1 + 1 + pad + 8            │
│  TestReport_t                 100             4 + 8 × 12                     │
│  SensorDriver_t               44              Pointers × 11                  │
│  ─────────────────────────────────────────────────────────────────────────  │
│  UART_RX_Buffer               256             Ring buffer                    │
│  UART_TX_Buffer               256             Ring buffer                    │
│  Protocol_RX_Buffer           128             Frame assembly                 │
│  ─────────────────────────────────────────────────────────────────────────  │
│  MLX90640_Frame               1544            768 × 2 + 8 (float array)      │
│  MLX90640_EEPROM              1664            832 × 2                        │
│  MLX90640_Params              ~384            Calibration parameters         │
│  ─────────────────────────────────────────────────────────────────────────  │
│                                                                              │
│  Total Static RAM Usage       ~4.5 KB         (excluding MLX90640)           │
│  MLX90640 Buffer              ~3.5 KB                                        │
│  ─────────────────────────────────────────────────────────────────────────  │
│  Total                        ~8 KB                                          │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 7. File Structure

### 7.1 최종 디렉토리 구조

```
PSA-sensor-test/
│
├── docs/                               # 문서
│   ├── 01-protocol-specification.md
│   ├── 02-hardware-configuration.md
│   ├── 03-project-structure.md
│   ├── 04-development-workflow.md
│   └── 05-code-architecture.md         # 본 문서
│
├── Core/                               # CubeMX 생성 파일
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32h7xx_hal_conf.h
│   │   └── stm32h7xx_it.h
│   └── Src/
│       ├── main.c                      # CubeMX 참조용 (직접 사용 안함)
│       ├── stm32h7xx_hal_msp.c
│       ├── stm32h7xx_it.c
│       ├── system_stm32h7xx.c
│       ├── sysmem.c
│       └── syscalls.c
│
├── Drivers/                            # HAL 드라이버
│   ├── CMSIS/
│   └── STM32H7xx_HAL_Driver/
│
├── include/                            # 프로젝트 헤더
│   ├── config.h                        # 전역 설정
│   │
│   ├── protocol/                       # 프로토콜 레이어
│   │   ├── protocol.h
│   │   ├── frame.h
│   │   └── commands.h
│   │
│   ├── sensors/                        # 센서 레이어
│   │   ├── sensor_types.h
│   │   ├── sensor_manager.h
│   │   ├── mlx90640.h
│   │   └── vl53l0x.h
│   │
│   ├── test/                           # 테스트 레이어
│   │   └── test_runner.h
│   │
│   └── hal/                            # HAL 래퍼 레이어
│       ├── uart_handler.h
│       └── i2c_handler.h
│
├── src/                                # 프로젝트 소스
│   ├── main.c                          # 애플리케이션 진입점
│   │
│   ├── protocol/
│   │   ├── protocol.c
│   │   ├── frame.c
│   │   └── commands.c
│   │
│   ├── sensors/
│   │   ├── sensor_manager.c
│   │   ├── mlx90640.c
│   │   └── vl53l0x.c
│   │
│   ├── test/
│   │   └── test_runner.c
│   │
│   └── hal/
│       ├── uart_handler.c
│       └── i2c_handler.c
│
├── lib/                                # 외부 라이브러리
│   ├── MLX90640_API/
│   │   ├── MLX90640_API.h
│   │   ├── MLX90640_API.c
│   │   └── MLX90640_I2C_Driver.h       # 포팅 인터페이스
│   │
│   └── VL53L0X_API/
│       ├── core/
│       │   ├── inc/
│       │   │   └── vl53l0x_api.h
│       │   └── src/
│       │       └── vl53l0x_api.c
│       └── platform/
│           ├── inc/
│           │   └── vl53l0x_platform.h  # 포팅 인터페이스
│           └── src/
│               └── vl53l0x_platform.c
│
├── platformio.ini                      # PlatformIO 설정
├── PSA-sensor-test.ioc                 # CubeMX 프로젝트
├── STM32H723XG_FLASH.ld               # 링커 스크립트
├── startup_stm32h723xx.s              # 스타트업 어셈블리
└── Makefile                           # (CubeMX 생성)
```

---

## 8. Implementation Checklist

### 8.1 Phase 1: 기반 구조

- [ ] **HAL Wrapper**
  - [ ] `src/hal/i2c_handler.c` - I2C1, I2C4 초기화 및 통신
  - [ ] `src/hal/uart_handler.c` - UART4 인터럽트 기반 수신

- [ ] **Protocol Layer**
  - [ ] `src/protocol/frame.c` - 프레임 파싱/빌드
  - [ ] `src/protocol/protocol.c` - 프로토콜 초기화

### 8.2 Phase 2: 센서 드라이버

- [ ] **Sensor Manager**
  - [ ] `src/sensors/sensor_manager.c` - 드라이버 등록/조회

- [ ] **MLX90640 Driver**
  - [ ] `lib/MLX90640_API/` - 공식 API 포팅
  - [ ] `src/sensors/mlx90640.c` - 드라이버 인터페이스 구현

- [ ] **VL53L0X Driver**
  - [ ] `lib/VL53L0X_API/` - 공식 API 포팅
  - [ ] `src/sensors/vl53l0x.c` - 드라이버 인터페이스 구현

### 8.3 Phase 3: 테스트 로직

- [ ] **Test Runner**
  - [ ] `src/test/test_runner.c` - 테스트 시퀀스 실행

- [ ] **Command Handlers**
  - [ ] `src/protocol/commands.c` - 모든 명령 핸들러 구현

### 8.4 Phase 4: 통합 및 테스트

- [ ] **Main Application**
  - [ ] `src/main.c` - 시스템 초기화 및 메인 루프

- [ ] **Integration Test**
  - [ ] PING/PONG 통신 확인
  - [ ] SET_SPEC 명령 확인
  - [ ] TEST_SINGLE 각 센서 확인
  - [ ] TEST_ALL 시퀀스 확인

---

## 9. API Summary

### 9.1 Quick Reference

| Module | Function | Description |
|--------|----------|-------------|
| **Protocol** | `Protocol_Init()` | 프로토콜 모듈 초기화 |
| | `Protocol_Process()` | 메인 루프에서 호출 |
| **Frame** | `Frame_Parse()` | 바이트 → Frame_t |
| | `Frame_Build()` | Frame_t → 바이트 |
| | `Frame_CalculateCRC()` | XOR 체크섬 |
| **Commands** | `Commands_Init()` | 핸들러 등록 |
| | `Commands_Process()` | 명령 디스패치 |
| **Sensor Manager** | `SensorManager_Init()` | 모든 드라이버 등록 |
| | `SensorManager_GetByID()` | ID로 드라이버 조회 |
| | `SensorManager_GetByIndex()` | 인덱스로 드라이버 조회 |
| **Test Runner** | `TestRunner_Init()` | 테스트 러너 초기화 |
| | `TestRunner_RunAll()` | 전체 테스트 실행 |
| | `TestRunner_RunSingle()` | 단일 센서 테스트 |
| **UART Handler** | `UART_Handler_Init()` | UART 초기화 |
| | `UART_Handler_Send()` | 데이터 송신 |
| | `UART_Handler_Read()` | 데이터 수신 |
| **I2C Handler** | `I2C_Handler_Init()` | I2C 버스 초기화 |
| | `I2C_Handler_Read16()` | 16-bit 주소 읽기 |
| | `I2C_Handler_Write16()` | 16-bit 주소 쓰기 |

---

## 10. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0 | 2025-12-29 | - | 초기 아키텍처 설계 |
