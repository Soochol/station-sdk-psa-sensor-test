# Commands Reference

PSA 프로토콜의 명령어 레퍼런스입니다.

## 명령 코드 요약

### Host → MCU (Request)

| CMD | 이름 | Payload | 설명 |
|-----|------|---------|------|
| 0x01 | PING | - | 연결 확인 |
| 0x10 | TEST_ALL | - | 전체 센서 테스트 |
| 0x11 | TEST_SINGLE | SensorID | 개별 센서 테스트 |
| 0x12 | GET_SENSOR_LIST | - | 센서 목록 조회 |
| 0x13 | READ_SENSOR | SensorID | 센서 Raw 데이터 읽기 (스펙 비교 없음) |
| 0x20 | SET_SPEC | SensorID + Spec | 테스트 스펙 설정 |
| 0x21 | GET_SPEC | SensorID | 테스트 스펙 조회 |

### MCU → Host (Response)

| CMD | 이름 | Payload | 설명 |
|-----|------|---------|------|
| 0x01 | PONG | Version | PING 응답 |
| 0x80 | TEST_RESULT | Results | 테스트 결과 |
| 0x81 | SENSOR_LIST | Sensors | 센서 목록 |
| 0x82 | SPEC_ACK | SensorID | 스펙 설정 확인 |
| 0x83 | SPEC_DATA | SensorID + Spec | 스펙 데이터 |
| 0x84 | SENSOR_DATA | SensorID + Status + Data | 센서 Raw 데이터 |
| 0xFE | NAK | ErrorCode | 에러 응답 |

---

## PING (0x01)

연결 상태 및 펌웨어 버전을 확인합니다.

### Request

```
┌──────┬──────┬──────┬──────┬──────┐
│ 0x02 │ 0x00 │ 0x01 │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────┴──────┘
```

### Response (PONG)

```
┌──────┬──────┬──────┬───────┬───────┬───────┬──────┬──────┐
│ 0x02 │ 0x03 │ 0x01 │ Major │ Minor │ Patch │ CRC  │ 0x03 │
└──────┴──────┴──────┴───────┴───────┴───────┴──────┴──────┘
                      └────── Firmware Version ──────┘
```

| 필드 | 타입 | 설명 |
|------|------|------|
| Major | uint8 | 메이저 버전 |
| Minor | uint8 | 마이너 버전 |
| Patch | uint8 | 패치 버전 |

### Python 예제

```python
version = client.ping()
print(f"Firmware v{version[0]}.{version[1]}.{version[2]}")
```

---

## GET_SENSOR_LIST (0x12)

등록된 센서 목록을 조회합니다.

### Request

```
┌──────┬──────┬──────┬──────┬──────┐
│ 0x02 │ 0x00 │ 0x12 │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────┴──────┘
```

### Response (SENSOR_LIST - 0x81)

```
┌──────┬──────┬──────┬───────┬─────────────────────────────┬──────┬──────┐
│ 0x02 │ LEN  │ 0x81 │ Count │ [ID][NameLen][Name]... x N  │ CRC  │ 0x03 │
└──────┴──────┴──────┴───────┴─────────────────────────────┴──────┴──────┘
```

| 필드 | 타입 | 설명 |
|------|------|------|
| Count | uint8 | 센서 개수 |
| ID | uint8 | 센서 ID |
| NameLen | uint8 | 이름 길이 |
| Name | char[] | 센서 이름 (ASCII) |

### 응답 예제

```
02 14 81 02 01 07 56 4C 35 33 4C 30 58 02 08 4D 4C 58 39 30 36 34 30 B5 03

해석:
- Count: 02 (2개 센서)
- Sensor 1: ID=01, Len=07, Name="VL53L0X"
- Sensor 2: ID=02, Len=08, Name="MLX90640"
```

---

## READ_SENSOR (0x13)

센서의 Raw 데이터를 읽습니다. 스펙 비교 없이 측정값만 반환합니다.

### Request

```
┌──────┬──────┬──────┬──────────┬──────┬──────┐
│ 0x02 │ 0x01 │ 0x13 │ SensorID │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────────┴──────┴──────┘
```

### Response (SENSOR_DATA - 0x84)

```
┌──────┬──────┬──────┬──────────┬────────┬────────────┬──────┬──────┐
│ 0x02 │ LEN  │ 0x84 │ SensorID │ Status │  RawData   │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────────┴────────┴────────────┴──────┴──────┘
```

| 필드 | 타입 | 설명 |
|------|------|------|
| SensorID | uint8 | 센서 ID |
| Status | uint8 | 측정 상태 (0x00=성공, 기타=에러) |
| RawData | bytes | 센서별 측정 데이터 |

### VL53L0X Raw Data (8 bytes)

```
┌─────────────────┬─────────────────┬─────────────────┬─────────────────┐
│  Measured (mm)  │     Target      │    Tolerance    │      Diff       │
│  uint16 (BE)    │   uint16 (BE)   │   uint16 (BE)   │   uint16 (BE)   │
└─────────────────┴─────────────────┴─────────────────┴─────────────────┘
```

### MLX90640 Raw Data (8 bytes)

```
┌─────────────────┬─────────────────┬─────────────────┬─────────────────┐
│ Measured(x10°C) │  Target(x10°C)  │ Tolerance(x10°C)│   Diff(x10°C)   │
│   int16 (BE)    │   int16 (BE)    │   uint16 (BE)   │   uint16 (BE)   │
└─────────────────┴─────────────────┴─────────────────┴─────────────────┘
```

### Python 예제

```python
# VL53L0X 거리 읽기
data = client.read_sensor(SENSOR_VL53L0X)
distance_mm = data.measured

# MLX90640 온도 읽기
data = client.read_sensor(SENSOR_MLX90640)
temp_c = data.measured / 10.0
```

---

## SET_SPEC (0x20)

센서 테스트 스펙을 설정합니다.

### Request

```
┌──────┬──────┬──────┬──────────┬────────────┬──────┬──────┐
│ 0x02 │ LEN  │ 0x20 │ SensorID │  SpecData  │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────────┴────────────┴──────┴──────┘
```

### VL53L0X Spec (4 bytes)

```
┌─────────────────┬─────────────────┐
│  Target (mm)    │  Tolerance (mm) │
│  uint16 (BE)    │  uint16 (BE)    │
└─────────────────┴─────────────────┘
```

### MLX90640 Spec (6 bytes)

```
┌─────────────────┬─────────────────┬─────────┬─────────┐
│  Target (x10°C) │ Tolerance(x10°C)│ Pixel X │ Pixel Y │
│  int16 (BE)     │  uint16 (BE)    │  uint8  │  uint8  │
└─────────────────┴─────────────────┴─────────┴─────────┘
```

### Response (SPEC_ACK - 0x82)

```
┌──────┬──────┬──────┬──────────┬──────┬──────┐
│ 0x02 │ 0x01 │ 0x82 │ SensorID │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────────┴──────┴──────┘
```

### Python 예제

```python
from psa_protocol import VL53L0XSpec, MLX90640Spec

# VL53L0X: 500mm ± 100mm
client.set_spec_vl53l0x(VL53L0XSpec(target_dist=500, tolerance=100))

# MLX90640: 25.0°C ± 5.0°C
client.set_spec_mlx90640(MLX90640Spec(target_temp=250, tolerance=50))
```

---

## GET_SPEC (0x21)

현재 설정된 테스트 스펙을 조회합니다.

### Request

```
┌──────┬──────┬──────┬──────────┬──────┬──────┐
│ 0x02 │ 0x01 │ 0x21 │ SensorID │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────────┴──────┴──────┘
```

### Response (SPEC_DATA - 0x83)

```
┌──────┬──────┬──────┬──────────┬────────────┬──────┬──────┐
│ 0x02 │ LEN  │ 0x83 │ SensorID │  SpecData  │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────────┴────────────┴──────┴──────┘
```

---

## TEST_SINGLE (0x11)

단일 센서 테스트를 실행합니다.

### Request

```
┌──────┬──────┬──────┬──────────┬──────┬──────┐
│ 0x02 │ 0x01 │ 0x11 │ SensorID │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────────┴──────┴──────┘
```

### Response (TEST_RESULT - 0x80)

```
┌──────┬──────┬──────┬────────────────────────────────────┬──────┬──────┐
│ 0x02 │ LEN  │ 0x80 │           Result Payload           │ CRC  │ 0x03 │
└──────┴──────┴──────┴────────────────────────────────────┴──────┴──────┘
```

### Result Payload 구조

```
┌───────┬───────┬───────┬───────────┬────────────────────────────────────┐
│ Count │ Pass  │ Fail  │ Timestamp │ [ID][Status][ResultData]... x N    │
│ uint8 │ uint8 │ uint8 │ uint32 BE │                                    │
└───────┴───────┴───────┴───────────┴────────────────────────────────────┘
```

### Per-Sensor Result

```
┌──────────┬────────┬──────────────────┐
│ SensorID │ Status │    ResultData    │
│  uint8   │ uint8  │     8 bytes      │
└──────────┴────────┴──────────────────┘
```

---

## TEST_ALL (0x10)

모든 센서를 순차적으로 테스트합니다.

### Request

```
┌──────┬──────┬──────┬──────┬──────┐
│ 0x02 │ 0x00 │ 0x10 │ CRC  │ 0x03 │
└──────┴──────┴──────┴──────┴──────┘
```

### Response

TEST_SINGLE과 동일한 TEST_RESULT 형식으로 응답합니다.

### Fail-Fast 동작

첫 번째 센서가 실패하면:
- 해당 센서: 실패 상태 + 결과 데이터
- 이후 센서: STATUS_NOT_TESTED (0xFF)

---

## NAK (0xFE)

에러 응답입니다.

### Response

```
┌──────┬──────┬──────┬───────────┬──────┬──────┐
│ 0x02 │ 0x01 │ 0xFE │ ErrorCode │ CRC  │ 0x03 │
└──────┴──────┴──────┴───────────┴──────┴──────┘
```

### Error Codes

| 코드 | 이름 | 설명 |
|------|------|------|
| 0x01 | UNKNOWN_CMD | 알 수 없는 명령 |
| 0x02 | INVALID_SENSOR_ID | 잘못된 센서 ID |
| 0x03 | INVALID_PAYLOAD | 잘못된 페이로드 |
| 0x04 | BUSY | 테스트 진행 중 |
| 0x05 | CRC_FAIL | CRC 검증 실패 |
| 0x06 | NO_SPEC | 스펙 미설정 |

---

## Status Codes

테스트 결과 상태 코드입니다.

| 코드 | 이름 | 설명 |
|------|------|------|
| 0x00 | PASS | 테스트 통과 |
| 0x01 | FAIL_NO_ACK | I2C ACK 없음 (센서 미연결) |
| 0x02 | FAIL_TIMEOUT | 통신 타임아웃 |
| 0x03 | FAIL_INVALID | 스펙 범위 초과 |
| 0x04 | FAIL_INIT | 초기화 실패 |
| 0x05 | FAIL_NO_SPEC | 스펙 미설정 |
| 0xFF | NOT_TESTED | 테스트 미실행 (스킵) |

---

## Sensor IDs

| ID | 센서 | 인터페이스 |
|-----|------|----------|
| 0x01 | VL53L0X | I2C1 (0x29) |
| 0x02 | MLX90640 | I2C4 (0x33) |
