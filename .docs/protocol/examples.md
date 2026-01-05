# Communication Examples

실제 UART 통신 예제입니다. 모든 값은 16진수(hex)입니다.

## 1. PING / PONG

연결 상태를 확인합니다.

### Request
```
Host → MCU:
02 00 01 07 03

┌──────┬──────┬──────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │ CRC  │ ETX  │
│ 02   │ 00   │ 01   │ 07   │ 03   │
└──────┴──────┴──────┴──────┴──────┘
         │      └─ PING
         └─ No payload
```

### Response
```
MCU → Host:
02 03 01 01 00 00 DB 03

┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │ v1   │ v2   │ v3   │ CRC  │ ETX  │
│ 02   │ 03   │ 01   │ 01   │ 00   │ 00   │ DB   │ 03   │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
         │      │      └────────────────┘
         │      └─ PONG      Version 1.0.0
         └─ 3 bytes payload
```

---

## 2. GET_SENSOR_LIST

등록된 센서 목록을 조회합니다.

### Request
```
Host → MCU:
02 00 12 7E 03

┌──────┬──────┬──────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │ CRC  │ ETX  │
│ 02   │ 00   │ 12   │ 7E   │ 03   │
└──────┴──────┴──────┴──────┴──────┘
               └─ GET_SENSOR_LIST
```

### Response
```
MCU → Host:
02 14 81 02 01 07 56 4C 35 33 4C 30 58 02 08 4D 4C 58 39 30 36 34 30 B5 03

해석:
┌──────┬──────┬──────┬───────┬─────────────────────────────────────────────┐
│ STX  │ LEN  │ CMD  │ Count │              Sensor Data                    │
│ 02   │ 14   │ 81   │ 02    │ 01 07 "VL53L0X" | 02 08 "MLX90640"          │
└──────┴──────┴──────┴───────┴─────────────────────────────────────────────┘

센서 1:
  ID = 01 (VL53L0X)
  NameLen = 07
  Name = "VL53L0X" (56 4C 35 33 4C 30 58)

센서 2:
  ID = 02 (MLX90640)
  NameLen = 08
  Name = "MLX90640" (4D 4C 58 39 30 36 34 30)
```

---

## 3. SET_SPEC - VL53L0X

VL53L0X 센서 스펙을 설정합니다 (500mm ± 100mm).

### Request
```
Host → MCU:
02 05 20 02 01 F4 00 64 BF 03

┌──────┬──────┬──────┬──────┬─────────────────────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │  ID  │     Spec Data       │ CRC  │ ETX  │
│ 02   │ 05   │ 20   │ 02   │ 01 F4  |  00 64     │ BF   │ 03   │
└──────┴──────┴──────┴──────┴─────────────────────┴──────┴──────┘
               │      │      └──────┴───────────────┘
               │      │       500mm    100mm
               │      └─ VL53L0X (0x02)
               └─ SET_SPEC

Target: 0x01F4 = 500 (mm)
Tolerance: 0x0064 = 100 (mm)
```

### Response
```
MCU → Host:
02 01 82 02 F9 03

┌──────┬──────┬──────┬──────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │  ID  │ CRC  │ ETX  │
│ 02   │ 01   │ 82   │ 02   │ F9   │ 03   │
└──────┴──────┴──────┴──────┴──────┴──────┘
               │      └─ VL53L0X confirmed
               └─ SPEC_ACK
```

---

## 4. TEST_SINGLE - VL53L0X

VL53L0X 센서 테스트를 실행합니다.

### Request
```
Host → MCU:
02 01 11 02 27 03

┌──────┬──────┬──────┬──────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │  ID  │ CRC  │ ETX  │
│ 02   │ 01   │ 11   │ 02   │ 27   │ 03   │
└──────┴──────┴──────┴──────┴──────┴──────┘
               │      └─ VL53L0X
               └─ TEST_SINGLE
```

### Response (PASS)
```
MCU → Host:
02 11 80 01 01 00 00 07 5B 30 02 00 01 1A 01 F4 00 64 00 96 XX 03

해석:
┌──────────────────────────────────────────────────────────────┐
│ Header: 02 11 80                                             │
│   STX=02, LEN=17, CMD=80 (TEST_RESULT)                       │
├──────────────────────────────────────────────────────────────┤
│ Report Header: 01 01 00 00 07 5B 30                          │
│   Count=1, Pass=1, Fail=0, Timestamp=481072ms                │
├──────────────────────────────────────────────────────────────┤
│ Sensor Result: 02 00 01 1A 01 F4 00 64 00 96                 │
│   ID=02 (VL53L0X)                                            │
│   Status=00 (PASS)                                           │
│   Measured=0x011A = 282mm                                    │
│   Target=0x01F4 = 500mm                                      │
│   Tolerance=0x0064 = 100mm                                   │
│   Diff=0x0096 = 150mm                                        │
└──────────────────────────────────────────────────────────────┘

Note: 이 예제에서 Diff(150) > Tolerance(100)이면 FAIL_INVALID가 됨
```

### Response (FAIL - Out of Tolerance)
```
MCU → Host:
Status = 03 (FAIL_INVALID)

측정값이 스펙 범위를 벗어난 경우
```

---

## 5. SET_SPEC - MLX90640

MLX90640 센서 스펙을 설정합니다 (25.0°C ± 5.0°C, 전체 평균).

### Request
```
Host → MCU:
02 07 20 01 00 FA 00 32 FF FF XX 03

┌──────┬──────┬──────┬──────┬───────────────────────────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │  ID  │        Spec Data          │ CRC  │ ETX  │
│ 02   │ 07   │ 20   │ 01   │ 00 FA 00 32 FF FF         │ XX   │ 03   │
└──────┴──────┴──────┴──────┴───────────────────────────┴──────┴──────┘
                      │      │      │      │   │   │
                      │      │      │      │   │   └─ Pixel Y (0xFF=avg)
                      │      │      │      │   └─ Pixel X (0xFF=avg)
                      │      │      │      └─ Tolerance = 50 (5.0°C)
                      │      │      └─ Target = 250 (25.0°C)
                      └─ MLX90640 (0x01)
```

---

## 6. TEST_ALL

모든 센서를 순차 테스트합니다.

### Request
```
Host → MCU:
02 00 10 76 03

┌──────┬──────┬──────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │ CRC  │ ETX  │
│ 02   │ 00   │ 10   │ 76   │ 03   │
└──────┴──────┴──────┴──────┴──────┘
               └─ TEST_ALL
```

### Response (All Pass)
```
MCU → Host:
Count=2, Pass=2, Fail=0

[VL53L0X]  Status=PASS, Result Data...
[MLX90640] Status=PASS, Result Data...
```

### Response (First Fail - Fail-Fast)
```
MCU → Host:
Count=2, Pass=0, Fail=1

[VL53L0X]  Status=FAIL_NO_ACK (0x01)  ← 센서 미연결
[MLX90640] Status=NOT_TESTED (0xFF)   ← 스킵됨
```

---

## 7. READ_SENSOR - VL53L0X

스펙 비교 없이 센서 Raw 데이터만 읽습니다.

### Request
```
Host → MCU:
02 01 13 01 7F 03

┌──────┬──────┬──────┬──────┬──────┬──────┐
│ STX  │ LEN  │ CMD  │  ID  │ CRC  │ ETX  │
│ 02   │ 01   │ 13   │ 01   │ 7F   │ 03   │
└──────┴──────┴──────┴──────┴──────┴──────┘
               │      └─ VL53L0X
               └─ READ_SENSOR
```

### Response (SENSOR_DATA)
```
MCU → Host:
02 0A 84 01 00 00 8A 00 8C 00 64 00 02 XX 03

해석:
┌──────────────────────────────────────────────────────────────┐
│ Header: 02 0A 84                                             │
│   STX=02, LEN=10, CMD=84 (SENSOR_DATA)                       │
├──────────────────────────────────────────────────────────────┤
│ Payload: 01 00 00 8A 00 8C 00 64 00 02                       │
│   SensorID=01 (VL53L0X)                                      │
│   Status=00 (SUCCESS)                                        │
│   Measured=0x008A = 138mm                                    │
│   Target=0x008C = 140mm                                      │
│   Tolerance=0x0064 = 100mm                                   │
│   Diff=0x0002 = 2mm                                          │
└──────────────────────────────────────────────────────────────┘
```

---

## 8. NAK Response

에러 발생 시 NAK 응답입니다.

### Unknown Command
```
Host → MCU:
02 00 FF XX 03  (잘못된 명령 0xFF)

MCU → Host:
02 01 FE 01 XX 03

┌──────┬──────┬──────┬──────┬──────┬──────┐
│ STX  │ LEN  │ NAK  │ ERR  │ CRC  │ ETX  │
│ 02   │ 01   │ FE   │ 01   │ XX   │ 03   │
└──────┴──────┴──────┴──────┴──────┴──────┘
                      └─ UNKNOWN_CMD
```

### CRC Error
```
Host → MCU:
02 00 01 00 03  (잘못된 CRC)

MCU → Host:
02 01 FE 05 XX 03

Error Code = 05 (CRC_FAIL)
```

---

## 9. 타이밍 다이어그램

```
Host                                MCU
  │                                  │
  │──── PING ────────────────────────►│
  │                                  │ Process
  │◄───────────────────── PONG ──────│
  │                                  │
  │──── SET_SPEC (VL53L0X) ──────────►│
  │                                  │ Save spec
  │◄────────────────── SPEC_ACK ─────│
  │                                  │
  │──── TEST_SINGLE (VL53L0X) ───────►│
  │                                  │ Init sensor
  │                                  │ Measure (~100ms)
  │                                  │ Validate
  │◄─────────────── TEST_RESULT ─────│
  │                                  │
  │                                  │
  Time ───────────────────────────────►
       ~10ms     ~10ms      ~1000ms
```

---

## CRC 계산 예제

### PING 프레임

```
프레임: 02 00 01 ?? 03

CRC 범위: 00 01 (LENGTH + CMD)

계산:
  crc = 0x00
  crc = table[0x00 ^ 0x00] = table[0x00] = 0x00
  crc = table[0x00 ^ 0x01] = table[0x01] = 0x07

결과: CRC = 0x07

완성 프레임: 02 00 01 07 03
```

### SET_SPEC 프레임

```
프레임: 02 05 20 02 01 F4 00 64 ?? 03

CRC 범위: 05 20 02 01 F4 00 64

CRC 계산 결과: 0xBF

완성 프레임: 02 05 20 02 01 F4 00 64 BF 03
```
