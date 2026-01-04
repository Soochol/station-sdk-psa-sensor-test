# Station SDK - PSA Sensor Test - Project Structure

## 1. Directory Layout

```
station-sdk-psa-sensor-test/
│
├── docs/                           # 📚 문서
│   ├── 01-protocol-specification.md
│   ├── 02-hardware-configuration.md
│   ├── 03-project-structure.md
│   └── 04-development-workflow.md
│
├── CubeMX/                         # 🔧 STM32CubeMX 생성 파일
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h
│   │   │   ├── stm32h7xx_hal_conf.h
│   │   │   ├── stm32h7xx_it.h
│   │   │   ├── gpio.h
│   │   │   ├── i2c.h
│   │   │   └── usart.h
│   │   └── Src/
│   │       ├── main.c              # 참조용 (직접 사용 안함)
│   │       ├── stm32h7xx_hal_msp.c
│   │       ├── stm32h7xx_it.c
│   │       ├── system_stm32h7xx.c
│   │       ├── gpio.c
│   │       ├── i2c.c
│   │       └── usart.c
│   ├── Drivers/
│   │   ├── CMSIS/
│   │   └── STM32H7xx_HAL_Driver/
│   ├── Makefile
│   └── PSA-sensor-test.ioc         # CubeMX 프로젝트 파일
│
├── include/                        # 📁 프로젝트 헤더 파일
│   ├── config.h                    # 전역 설정 (타임아웃, 핀맵 등)
│   │
│   ├── protocol/                   # 통신 프로토콜
│   │   ├── protocol.h              # 프로토콜 상수 정의
│   │   ├── frame.h                 # 프레임 파싱/빌드
│   │   └── commands.h              # 명령 핸들러
│   │
│   ├── sensors/                    # 센서 드라이버
│   │   ├── sensor_types.h          # 공통 타입 정의
│   │   ├── sensor_manager.h        # 센서 등록/관리
│   │   ├── mlx90640.h              # MLX90640 드라이버
│   │   └── vl53l0x.h               # VL53L0X 드라이버
│   │
│   ├── test/                       # 테스트 실행
│   │   └── test_runner.h           # 테스트 시퀀스 관리
│   │
│   └── hal/                        # HAL 래퍼
│       ├── uart_handler.h          # UART 송수신
│       └── i2c_handler.h           # I2C 통신
│
├── src/                            # 📁 프로젝트 소스 파일
│   ├── main.c                      # 메인 진입점
│   │
│   ├── protocol/
│   │   ├── protocol.c              # 프로토콜 초기화
│   │   ├── frame.c                 # 프레임 처리
│   │   └── commands.c              # 명령 핸들러 구현
│   │
│   ├── sensors/
│   │   ├── sensor_manager.c        # 센서 관리자
│   │   ├── mlx90640.c              # MLX90640 구현
│   │   └── vl53l0x.c               # VL53L0X 구현
│   │
│   ├── test/
│   │   └── test_runner.c           # 테스트 실행 로직
│   │
│   └── hal/
│       ├── uart_handler.c          # UART 구현
│       └── i2c_handler.c           # I2C 구현
│
├── lib/                            # 📚 외부 라이브러리
│   ├── MLX90640_API/               # Melexis 공식 드라이버
│   │   ├── MLX90640_API.h
│   │   ├── MLX90640_API.c
│   │   └── MLX90640_I2C_Driver.h   # I2C 포팅 레이어
│   │
│   └── VL53L0X_API/                # ST 공식 드라이버
│       ├── Api/
│       ├── Platform/
│       └── vl53l0x_api.h
│
├── test/                           # 🧪 유닛 테스트 (선택)
│   └── ...
│
├── platformio.ini                  # PlatformIO 설정
├── .gitignore
└── README.md
```

---

## 2. Module Descriptions

### 2.1 Protocol Layer (`protocol/`)

UART 통신 프로토콜을 담당합니다.

#### protocol.h / protocol.c
```c
// 프로토콜 상수 정의
#define PROTOCOL_STX        0x02
#define PROTOCOL_ETX        0x03
#define PROTOCOL_MAX_PAYLOAD 64

// 명령 코드
typedef enum {
    CMD_PING            = 0x01,
    CMD_TEST_ALL        = 0x10,
    CMD_TEST_SINGLE     = 0x11,
    CMD_GET_SENSOR_LIST = 0x12,
    CMD_SET_SPEC        = 0x20,
    CMD_GET_SPEC        = 0x21,
    // Response
    CMD_TEST_RESULT     = 0x80,
    CMD_SENSOR_LIST     = 0x81,
    CMD_SPEC_ACK        = 0x82,
    CMD_SPEC_DATA       = 0x83,
    CMD_NAK             = 0xFE,
} CommandCode_t;
```

#### frame.h / frame.c
```c
// 프레임 구조체
typedef struct {
    uint8_t cmd;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD];
    uint8_t payload_len;
} Frame_t;

// 프레임 파싱/빌드 함수
bool Frame_Parse(uint8_t* buffer, uint16_t len, Frame_t* frame);
uint16_t Frame_Build(Frame_t* frame, uint8_t* buffer);
uint8_t Frame_CalculateCRC(uint8_t* data, uint8_t len);
```

#### commands.h / commands.c
```c
// 명령 핸들러 타입
typedef void (*CommandHandler_t)(Frame_t* request, Frame_t* response);

// 명령 처리
void Commands_Init(void);
void Commands_Process(Frame_t* request);
```

---

### 2.2 Sensor Layer (`sensors/`)

센서 추상화 및 드라이버를 담당합니다.

#### sensor_types.h
```c
// 센서 ID
typedef enum {
    SENSOR_ID_MLX90640  = 0x01,
    SENSOR_ID_VL53L0X   = 0x02,
} SensorID_t;

// 테스트 상태
typedef enum {
    STATUS_PASS           = 0x00,
    STATUS_FAIL_NO_ACK    = 0x01,
    STATUS_FAIL_TIMEOUT   = 0x02,
    STATUS_FAIL_INVALID   = 0x03,
    STATUS_FAIL_INIT      = 0x04,
    STATUS_FAIL_NO_SPEC   = 0x05,
    STATUS_NOT_TESTED     = 0xFF,
} TestStatus_t;

// 센서 스펙 (공용체)
typedef union {
    struct {
        int16_t target_temp;    // ×100 °C
        uint16_t tolerance;     // ×100 °C
    } mlx90640;

    struct {
        uint16_t target_dist;   // mm
        uint16_t tolerance;     // mm
    } vl53l0x;
} SensorSpec_t;

// 센서 결과 (공용체)
typedef union {
    struct {
        int16_t max_temp;       // ×100 °C
        int16_t target;
        uint16_t tolerance;
        uint16_t diff;
    } mlx90640;

    struct {
        uint16_t measured;      // mm
        uint16_t target;
        uint16_t tolerance;
        uint16_t diff;
    } vl53l0x;
} SensorResult_t;
```

#### sensor_manager.h
```c
// 센서 드라이버 인터페이스
typedef struct {
    SensorID_t id;
    const char* name;

    // 라이프사이클
    HAL_StatusTypeDef (*init)(void);
    void (*deinit)(void);

    // 스펙 설정
    void (*set_spec)(SensorSpec_t* spec);
    void (*get_spec)(SensorSpec_t* spec);
    bool (*has_spec)(void);

    // 테스트 실행
    TestStatus_t (*run_test)(SensorResult_t* result);

    // 결과 직렬화
    uint8_t (*serialize_result)(SensorResult_t* result, uint8_t* buffer);
} SensorDriver_t;

// 센서 관리자 API
void SensorManager_Init(void);
void SensorManager_Register(const SensorDriver_t* driver);
const SensorDriver_t* SensorManager_GetByID(SensorID_t id);
uint8_t SensorManager_GetCount(void);
const SensorDriver_t* SensorManager_GetByIndex(uint8_t index);
```

---

### 2.3 Test Layer (`test/`)

테스트 시퀀스 실행을 담당합니다.

#### test_runner.h
```c
// 개별 테스트 결과
typedef struct {
    SensorID_t sensor_id;
    TestStatus_t status;
    SensorResult_t result;
} TestResult_t;

// 전체 테스트 결과
typedef struct {
    uint8_t count;
    TestResult_t results[MAX_SENSORS];
} TestReport_t;

// 테스트 API
void TestRunner_Init(void);
void TestRunner_RunAll(TestReport_t* report);
void TestRunner_RunSingle(SensorID_t id, TestReport_t* report);
```

---

### 2.4 HAL Layer (`hal/`)

하드웨어 추상화 래퍼입니다.

#### uart_handler.h
```c
// UART 초기화 및 콜백 설정
void UART_Handler_Init(void);
void UART_Handler_SetRxCallback(void (*callback)(uint8_t* data, uint16_t len));

// 송수신
HAL_StatusTypeDef UART_Handler_Send(uint8_t* data, uint16_t len);
void UART_Handler_Process(void);  // 메인루프에서 호출
```

#### i2c_handler.h
```c
// I2C 통신
HAL_StatusTypeDef I2C_Handler_Init(void);
HAL_StatusTypeDef I2C_Handler_Read(uint8_t addr, uint16_t reg, uint8_t* data, uint16_t len);
HAL_StatusTypeDef I2C_Handler_Write(uint8_t addr, uint16_t reg, uint8_t* data, uint16_t len);
HAL_StatusTypeDef I2C_Handler_IsDeviceReady(uint8_t addr, uint32_t timeout);
```

---

## 3. Configuration (`config.h`)

```c
#ifndef CONFIG_H
#define CONFIG_H

//------------------------------------------------------------
// Timeout Settings
//------------------------------------------------------------
#define SENSOR_TEST_TIMEOUT_MS      5000    // 센서당 테스트 타임아웃
#define UART_RX_TIMEOUT_MS          1000    // UART 수신 타임아웃
#define I2C_TIMEOUT_MS              100     // I2C 통신 타임아웃

//------------------------------------------------------------
// Sensor I2C Addresses (7-bit)
//------------------------------------------------------------
#define MLX90640_I2C_ADDR           0x33
#define VL53L0X_I2C_ADDR            0x29

//------------------------------------------------------------
// Protocol Settings
//------------------------------------------------------------
#define PROTOCOL_MAX_PAYLOAD        64
#define PROTOCOL_RX_BUFFER_SIZE     128

//------------------------------------------------------------
// Sensor Limits
//------------------------------------------------------------
#define MAX_SENSORS                 8       // 최대 등록 가능 센서 수

//------------------------------------------------------------
// Debug Settings
//------------------------------------------------------------
#define DEBUG_UART_ENABLED          0       // 디버그 출력 활성화

#endif // CONFIG_H
```

---

## 4. Build Configuration

### 4.1 platformio.ini

```ini
[env:stm32h723vg]
platform = ststm32
board = genericSTM32H723VG
framework = stm32cube

; 빌드 설정
board_build.mcu = stm32h723vgt6
board_build.f_cpu = 274000000L

; 업로드/디버그
upload_protocol = stlink
debug_tool = stlink
debug_init_break = tbreak main

; 헤더 경로
build_flags =
    -I include
    -I CubeMX/Core/Inc
    -I CubeMX/Drivers/STM32H7xx_HAL_Driver/Inc
    -I CubeMX/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    -I CubeMX/Drivers/CMSIS/Include
    -I lib/MLX90640_API
    -I lib/VL53L0X_API/Api
    -I lib/VL53L0X_API/Platform
    -D USE_HAL_DRIVER
    -D STM32H723xx
    -Wall
    -Wextra

; 소스 필터
build_src_filter =
    +<*>
    +<../CubeMX/Core/Src/stm32h7xx_hal_msp.c>
    +<../CubeMX/Core/Src/stm32h7xx_it.c>
    +<../CubeMX/Core/Src/system_stm32h7xx.c>
    +<../CubeMX/Core/Src/gpio.c>
    +<../CubeMX/Core/Src/i2c.c>
    +<../CubeMX/Core/Src/usart.c>
    +<../lib/MLX90640_API>
    +<../lib/VL53L0X_API>

; 라이브러리 경로
lib_deps =
lib_extra_dirs =
    lib
```

---

## 5. Dependency Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                          main.c                              │
└─────────────────────────────┬───────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│   Protocol    │     │  Test Runner  │     │    Config     │
│   (commands)  │     │               │     │               │
└───────┬───────┘     └───────┬───────┘     └───────────────┘
        │                     │
        ▼                     ▼
┌───────────────┐     ┌───────────────┐
│    Frame      │     │Sensor Manager │
│  (parse/build)│     │               │
└───────┬───────┘     └───────┬───────┘
        │                     │
        │             ┌───────┴───────┐
        ▼             ▼               ▼
┌───────────────┐ ┌─────────┐   ┌─────────┐
│ UART Handler  │ │MLX90640 │   │VL53L0X  │
│               │ │ Driver  │   │ Driver  │
└───────┬───────┘ └────┬────┘   └────┬────┘
        │              │              │
        │              └──────┬───────┘
        ▼                     ▼
┌───────────────┐     ┌───────────────┐
│  HAL_UART     │     │   HAL_I2C     │
│  (CubeMX)     │     │   (CubeMX)    │
└───────────────┘     └───────────────┘
```

---

## 6. Adding New Sensor

새로운 센서를 추가하는 방법:

### Step 1: 헤더 파일 생성
```c
// include/sensors/bme280.h
#ifndef BME280_H
#define BME280_H

#include "sensor_types.h"

extern const SensorDriver_t BME280_Driver;

#endif
```

### Step 2: 소스 파일 구현
```c
// src/sensors/bme280.c
#include "sensors/bme280.h"

static SensorSpec_t s_spec;
static bool s_has_spec = false;

static HAL_StatusTypeDef bme280_init(void) { ... }
static void bme280_deinit(void) { ... }
static void bme280_set_spec(SensorSpec_t* spec) { ... }
static void bme280_get_spec(SensorSpec_t* spec) { ... }
static bool bme280_has_spec(void) { return s_has_spec; }
static TestStatus_t bme280_run_test(SensorResult_t* result) { ... }
static uint8_t bme280_serialize_result(SensorResult_t* result, uint8_t* buf) { ... }

const SensorDriver_t BME280_Driver = {
    .id = SENSOR_ID_BME280,  // 새 ID 추가 필요
    .name = "BME280",
    .init = bme280_init,
    .deinit = bme280_deinit,
    .set_spec = bme280_set_spec,
    .get_spec = bme280_get_spec,
    .has_spec = bme280_has_spec,
    .run_test = bme280_run_test,
    .serialize_result = bme280_serialize_result,
};
```

### Step 3: 센서 등록
```c
// src/sensors/sensor_manager.c
#include "sensors/bme280.h"

void SensorManager_Init(void) {
    SensorManager_Register(&MLX90640_Driver);
    SensorManager_Register(&VL53L0X_Driver);
    SensorManager_Register(&BME280_Driver);  // 추가
}
```

### Step 4: Sensor ID 추가
```c
// include/sensors/sensor_types.h
typedef enum {
    SENSOR_ID_MLX90640  = 0x01,
    SENSOR_ID_VL53L0X   = 0x02,
    SENSOR_ID_BME280    = 0x03,  // 추가
} SensorID_t;
```
