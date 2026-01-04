# Station SDK - PSA Sensor Test - UART Protocol Specification

## 1. Overview

STM32H723VGT MCU 기반 센서 테스트 펌웨어를 위한 UART 통신 프로토콜 명세서.

### 1.1 목적
- MLX90640 (IR 열화상 센서) 및 VL53L0X (ToF 거리 센서) 통신 테스트
- 센서 데이터 읽기 및 스펙 기반 Validation
- Host PC와의 명령/응답 통신

### 1.2 통신 설정

| 항목 | 값 |
|------|-----|
| Baud Rate | 115200 bps |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |

---

## 2. Frame Structure

### 2.1 기본 프레임 포맷

```
┌──────┬────────┬────────┬──────────┬──────┬──────┐
│ STX  │ LENGTH │  CMD   │ PAYLOAD  │ CRC  │ ETX  │
│ 0x02 │ 1 byte │ 1 byte │ N bytes  │ 1 B  │ 0x03 │
└──────┴────────┴────────┴──────────┴──────┴──────┘
```

### 2.2 필드 설명

| 필드 | 크기 | 설명 |
|------|------|------|
| STX | 1 byte | 시작 문자 `0x02` |
| LENGTH | 1 byte | CMD + PAYLOAD 바이트 수 |
| CMD | 1 byte | 명령 코드 |
| PAYLOAD | 0~N bytes | 데이터 (가변 길이) |
| CRC | 1 byte | XOR 체크섬 (LENGTH ~ PAYLOAD) |
| ETX | 1 byte | 종료 문자 `0x03` |

### 2.3 CRC 계산

```c
uint8_t calculate_crc(uint8_t* data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}
```

---

## 3. Command Definitions

### 3.1 Host → MCU (Request Commands)

| CMD | Name | Payload | Description |
|-----|------|---------|-------------|
| `0x01` | PING | - | MCU 연결 확인 |
| `0x10` | TEST_ALL | - | 모든 센서 순차 테스트 |
| `0x11` | TEST_SINGLE | `[SensorID]` | 특정 센서만 테스트 |
| `0x12` | GET_SENSOR_LIST | - | 등록된 센서 목록 요청 |
| `0x20` | SET_SPEC | `[SensorID][SpecData]` | 센서 스펙 설정 |
| `0x21` | GET_SPEC | `[SensorID]` | 현재 스펙 조회 |

### 3.2 MCU → Host (Response Commands)

| CMD | Name | Payload | Description |
|-----|------|---------|-------------|
| `0x01` | PONG | - | PING 응답 |
| `0x80` | TEST_RESULT | 결과 데이터 | 테스트 결과 응답 |
| `0x81` | SENSOR_LIST | 센서 목록 | 등록 센서 정보 |
| `0x82` | SPEC_ACK | `[SensorID]` | 스펙 설정 완료 |
| `0x83` | SPEC_DATA | `[SensorID][Spec]` | 현재 스펙 응답 |
| `0xFE` | NAK | `[ErrorCode]` | 에러 응답 |

---

## 4. Sensor Definitions

### 4.1 Sensor ID

```c
typedef enum {
    SENSOR_ID_MLX90640  = 0x01,
    SENSOR_ID_VL53L0X   = 0x02,
    // 확장 가능
    // SENSOR_ID_BME280 = 0x03,
} SensorID_t;
```

### 4.2 Status Codes

```c
typedef enum {
    STATUS_PASS           = 0x00,  // 테스트 통과
    STATUS_FAIL_NO_ACK    = 0x01,  // I2C ACK 없음 (센서 미연결)
    STATUS_FAIL_TIMEOUT   = 0x02,  // 통신 타임아웃 (5초)
    STATUS_FAIL_INVALID   = 0x03,  // 스펙 범위 초과
    STATUS_FAIL_INIT      = 0x04,  // 초기화 실패
    STATUS_FAIL_NO_SPEC   = 0x05,  // 스펙 미설정
    STATUS_NOT_TESTED     = 0xFF,  // 테스트 미실행 (스킵됨)
} TestStatus_t;
```

### 4.3 Error Codes (NAK Payload)

```c
typedef enum {
    ERR_UNKNOWN_CMD       = 0x01,  // 알 수 없는 명령
    ERR_INVALID_SENSOR_ID = 0x02,  // 잘못된 센서 ID
    ERR_INVALID_PAYLOAD   = 0x03,  // 잘못된 페이로드
    ERR_BUSY              = 0x04,  // 테스트 진행 중
    ERR_CRC_FAIL          = 0x05,  // CRC 오류
} ErrorCode_t;
```

---

## 5. Specification Protocol

### 5.1 MLX90640 Spec Structure (4 bytes)

```
┌─────────────────┬─────────────────┐
│ Target Temp     │ Tolerance       │
│ int16 (×100°C)  │ uint16 (×100°C) │
│ 2 bytes (LE)    │ 2 bytes (LE)    │
└─────────────────┴─────────────────┘
```

| Field | Type | Unit | Example |
|-------|------|------|---------|
| Target Temp | int16 | ×100 °C | 3000 = 30.00°C |
| Tolerance | uint16 | ×100 °C | 100 = 1.00°C |

**Validation Logic:**
```
PASS if: |MaxPixelTemp - TargetTemp| <= Tolerance
```

### 5.2 VL53L0X Spec Structure (4 bytes)

```
┌─────────────────┬─────────────────┐
│ Target Distance │ Tolerance       │
│ uint16 (mm)     │ uint16 (mm)     │
│ 2 bytes (LE)    │ 2 bytes (LE)    │
└─────────────────┴─────────────────┘
```

| Field | Type | Unit | Example |
|-------|------|------|---------|
| Target Distance | uint16 | mm | 500 = 500mm |
| Tolerance | uint16 | mm | 10 = ±10mm |

**Validation Logic:**
```
PASS if: |MeasuredDistance - TargetDistance| <= Tolerance
```

---

## 6. Test Result Protocol

### 6.1 TEST_RESULT Payload Structure

```
┌──────────────┬─────────────────────────────────────────┐
│ Sensor Count │ [SensorID + Status + DataLen + Data]×N │
│ 1 byte       │ Variable                                │
└──────────────┴─────────────────────────────────────────┘
```

### 6.2 Per-Sensor Result Structure

```
┌───────────┬────────┬──────────┬─────────────────┐
│ Sensor ID │ Status │ Data Len │ Result Data     │
│ 1 byte    │ 1 byte │ 1 byte   │ N bytes         │
└───────────┴────────┴──────────┴─────────────────┘
```

### 6.3 MLX90640 Result Data (8 bytes)

```
┌───────────┬───────────┬───────────┬───────────┐
│ Max Temp  │ Target    │ Tolerance │ Diff      │
│ int16     │ int16     │ uint16    │ uint16    │
│ (×100°C)  │ (×100°C)  │ (×100°C)  │ (×100°C)  │
└───────────┴───────────┴───────────┴───────────┘
```

### 6.4 VL53L0X Result Data (8 bytes)

```
┌───────────┬───────────┬───────────┬───────────┐
│ Measured  │ Target    │ Tolerance │ Diff      │
│ uint16    │ uint16    │ uint16    │ uint16    │
│ (mm)      │ (mm)      │ (mm)      │ (mm)      │
└───────────┴───────────┴───────────┴───────────┘
```

---

## 7. Communication Examples

### 7.1 PING / PONG

**Request:**
```
02 01 01 01 03
│  │  │  │  └─ ETX
│  │  │  └──── CRC: 0x01 XOR = 0x01
│  │  └─────── CMD: PING (0x01)
│  └────────── LENGTH: 1
└───────────── STX
```

**Response:**
```
02 01 01 01 03  ← PONG
```

### 7.2 SET_SPEC - MLX90640 (30°C ± 1°C)

**Request:**
```
02 05 20 01 B8 0B 64 00 [CRC] 03
        │  │  └──┴──┴──┴── Spec: Target=3000, Tolerance=100
        │  └─────────────── Sensor ID: 0x01 (MLX90640)
        └──────────────────  CMD: SET_SPEC (0x20)
```

**Response:**
```
02 02 82 01 [CRC] 03  ← SPEC_ACK
```

### 7.3 SET_SPEC - VL53L0X (500mm ± 10mm)

**Request:**
```
02 05 20 02 F4 01 0A 00 [CRC] 03
        │  │  └──┴──┴──┴── Spec: Target=500, Tolerance=10
        │  └─────────────── Sensor ID: 0x02 (VL53L0X)
        └────────────────── CMD: SET_SPEC (0x20)
```

**Response:**
```
02 02 82 02 [CRC] 03  ← SPEC_ACK
```

### 7.4 TEST_ALL - All Pass

**Request:**
```
02 01 10 10 03  ← TEST_ALL
```

**Response:**
```
02 16 80 02                                    ← CMD: TEST_RESULT, Count: 2
   01 00 08 C6 0B B8 0B 64 00 0E 00            ← MLX90640: PASS, 30.14°C, Diff 0.14°C
   02 00 08 F6 01 F4 01 0A 00 02 00            ← VL53L0X: PASS, 502mm, Diff 2mm
   [CRC] 03
```

### 7.5 TEST_ALL - MLX90640 Fail (Sequence Abort)

**Request:**
```
02 01 10 10 03  ← TEST_ALL
```

**Response:**
```
02 0A 80 02                          ← CMD: TEST_RESULT, Count: 2
   01 02 00                          ← MLX90640: TIMEOUT, No data
   02 FF 00                          ← VL53L0X: NOT_TESTED (skipped)
   [CRC] 03
```

### 7.6 TEST_SINGLE - VL53L0X Only

**Request:**
```
02 02 11 02 13 03
        │  └──── Sensor ID: VL53L0X
        └─────── CMD: TEST_SINGLE
```

**Response:**
```
02 0B 80 01                          ← Count: 1
   02 00 08 F6 01 F4 01 0A 00 02 00  ← VL53L0X: PASS
   [CRC] 03
```

---

## 8. Test Sequence Flow

```
┌─────────────────────────────────────────────────────────┐
│                    TEST_ALL 수신                         │
└─────────────────────────┬───────────────────────────────┘
                          ▼
              ┌───────────────────────┐
              │  스펙 설정 확인        │
              │  (SET_SPEC 완료 여부)  │
              └───────────┬───────────┘
                    OK    │    NG → NAK (ERR_NO_SPEC)
                          ▼
              ┌───────────────────────┐
              │   MLX90640 Init       │
              └───────────┬───────────┘
                          ▼
              ┌───────────────────────┐
              │   MLX90640 Read       │──── FAIL ────┐
              │   (Max Pixel Temp)    │    (5s TO)   │
              └───────────┬───────────┘               │
                    PASS  │                           │
                          ▼                           │
              ┌───────────────────────┐               │
              │   MLX90640 Validate   │──── FAIL ────┤
              │   (Spec 범위 확인)    │               │
              └───────────┬───────────┘               │
                    PASS  │                           │
                          ▼                           │
              ┌───────────────────────┐               │
              │   MLX90640 Deinit     │               │
              └───────────┬───────────┘               │
                          ▼                           │
              ┌───────────────────────┐               │
              │   VL53L0X Init        │               │
              └───────────┬───────────┘               │
                          ▼                           │
              ┌───────────────────────┐               │
              │   VL53L0X Read        │──── FAIL ────┤
              │   (Distance)          │               │
              └───────────┬───────────┘               │
                    PASS  │                           │
                          ▼                           │
              ┌───────────────────────┐               │
              │   VL53L0X Validate    │──── FAIL ────┤
              └───────────┬───────────┘               │
                    PASS  │                           │
                          ▼                           ▼
              ┌───────────────────────┐   ┌─────────────────┐
              │   VL53L0X Deinit      │   │ All Handles     │
              └───────────┬───────────┘   │ Deinit          │
                          │               └────────┬────────┘
                          ▼                        ▼
              ┌───────────────────────┐   ┌─────────────────┐
              │  TEST_RESULT 전송     │   │ TEST_RESULT     │
              │  (All PASS)           │   │ (FAIL + SKIP)   │
              └───────────────────────┘   └─────────────────┘
```

---

## 9. Byte Order

모든 멀티바이트 값은 **Little Endian** 형식을 사용합니다.

```
예: int16 값 3000 (0x0BB8)
메모리/전송 순서: [0xB8] [0x0B]
```

---

## 10. Timing Constraints

| 항목 | 값 |
|------|-----|
| 센서당 테스트 타임아웃 | 5초 |
| 명령 응답 타임아웃 | 10초 (전체 테스트 기준) |
| 프레임 수신 타임아웃 | 1초 (바이트 간격) |

---

## Appendix A: Quick Reference

### Command Summary

| Direction | CMD | Name |
|-----------|-----|------|
| → MCU | 0x01 | PING |
| → MCU | 0x10 | TEST_ALL |
| → MCU | 0x11 | TEST_SINGLE |
| → MCU | 0x12 | GET_SENSOR_LIST |
| → MCU | 0x20 | SET_SPEC |
| → MCU | 0x21 | GET_SPEC |
| ← MCU | 0x01 | PONG |
| ← MCU | 0x80 | TEST_RESULT |
| ← MCU | 0x81 | SENSOR_LIST |
| ← MCU | 0x82 | SPEC_ACK |
| ← MCU | 0x83 | SPEC_DATA |
| ← MCU | 0xFE | NAK |

### Sensor ID Summary

| ID | Sensor |
|----|--------|
| 0x01 | MLX90640 |
| 0x02 | VL53L0X |

### Status Code Summary

| Code | Meaning |
|------|---------|
| 0x00 | PASS |
| 0x01 | FAIL_NO_ACK |
| 0x02 | FAIL_TIMEOUT |
| 0x03 | FAIL_INVALID |
| 0x04 | FAIL_INIT |
| 0x05 | FAIL_NO_SPEC |
| 0xFF | NOT_TESTED |
