# Sensor Data Structures

센서별 스펙 및 결과 데이터 구조를 정의합니다.

## VL53L0X (ToF Distance Sensor)

### 개요

| 항목 | 값 |
|------|-----|
| Sensor ID | 0x01 |
| Interface | I2C1 @ 0x29 |
| 측정 범위 | 30mm - 2000mm |
| 정확도 | ±3% (typical) |

### Spec Structure (4 bytes)

```
┌─────────────────────────┬─────────────────────────┐
│     Target Distance     │       Tolerance         │
│      uint16 (BE)        │       uint16 (BE)       │
│        2 bytes          │        2 bytes          │
└─────────────────────────┴─────────────────────────┘
Offset:    0-1                    2-3
```

| 필드 | 타입 | 단위 | 범위 | 예제 |
|------|------|------|------|------|
| Target | uint16 | mm | 30-2000 | 500 = 500mm |
| Tolerance | uint16 | mm | 1-2000 | 100 = ±100mm |

### Result Structure (8 bytes)

```
┌──────────────┬──────────────┬──────────────┬──────────────┐
│   Measured   │    Target    │  Tolerance   │     Diff     │
│ uint16 (BE)  │ uint16 (BE)  │ uint16 (BE)  │ uint16 (BE)  │
│   2 bytes    │   2 bytes    │   2 bytes    │   2 bytes    │
└──────────────┴──────────────┴──────────────┴──────────────┘
Offset:  0-1         2-3           4-5           6-7
```

| 필드 | 타입 | 설명 |
|------|------|------|
| Measured | uint16 | 측정된 거리 (mm) |
| Target | uint16 | 목표 거리 (mm) |
| Tolerance | uint16 | 허용 오차 (mm) |
| Diff | uint16 | |Measured - Target| (mm) |

### Pass/Fail 판정

```
PASS if: |Measured - Target| <= Tolerance
FAIL if: |Measured - Target| > Tolerance
```

### 예제

```
Spec: Target=500mm, Tolerance=100mm
Result: Measured=550mm

Diff = |550 - 500| = 50mm
50 <= 100 → PASS ✓
```

### Python 사용

```python
from psa_protocol import VL53L0XSpec, SensorID

# 스펙 설정
spec = VL53L0XSpec(target_dist=500, tolerance=100)
client.set_spec_vl53l0x(spec)

# 테스트 실행
report = client.test_single(SensorID.VL53L0X)
result = report.results[0]

if result.result:
    print(f"Measured: {result.result.measured}mm")
    print(f"Diff: {result.result.diff}mm")
    print(f"Passed: {result.result.passed}")
```

---

## MLX90640 (IR Thermal Sensor)

### 개요

| 항목 | 값 |
|------|-----|
| Sensor ID | 0x02 |
| Interface | I2C4 @ 0x33 |
| 해상도 | 32 x 24 픽셀 |
| 온도 범위 | -40°C ~ 300°C |
| 정확도 | ±1°C (typical) |

### Spec Structure (6 bytes)

```
┌─────────────────────────┬─────────────────────────┬──────────┬──────────┐
│    Target Temperature   │       Tolerance         │ Pixel X  │ Pixel Y  │
│      int16 (BE)         │      uint16 (BE)        │  uint8   │  uint8   │
│       2 bytes           │       2 bytes           │  1 byte  │  1 byte  │
└─────────────────────────┴─────────────────────────┴──────────┴──────────┘
Offset:     0-1                   2-3                   4          5
```

| 필드 | 타입 | 단위 | 범위 | 예제 |
|------|------|------|------|------|
| Target | int16 | x10 °C | -400 ~ 3000 | 250 = 25.0°C |
| Tolerance | uint16 | x10 °C | 1-5000 | 50 = ±5.0°C |
| Pixel X | uint8 | - | 0-31 or 0xFF | 0xFF = 전체 평균 |
| Pixel Y | uint8 | - | 0-23 or 0xFF | 0xFF = 전체 평균 |

### Pixel 선택

- **특정 픽셀**: X=0-31, Y=0-23 → 해당 픽셀 온도 측정
- **전체 평균**: X=0xFF, Y=0xFF → 모든 픽셀 평균 온도

```
    X →  0  1  2  3 ... 31
  Y ┌────────────────────┐
  ↓ │ [0,0]              │
  0 │                    │
  1 │        [15,12]     │  ← 중앙 픽셀
  : │                    │
 23 │              [31,23]│
    └────────────────────┘
```

### Result Structure (8 bytes)

```
┌──────────────┬──────────────┬──────────────┬──────────────┐
│  Measured    │    Target    │  Tolerance   │     Diff     │
│  int16 (BE)  │  int16 (BE)  │ uint16 (BE)  │ uint16 (BE)  │
│   2 bytes    │   2 bytes    │   2 bytes    │   2 bytes    │
└──────────────┴──────────────┴──────────────┴──────────────┘
Offset:  0-1         2-3           4-5           6-7
```

| 필드 | 타입 | 단위 | 설명 |
|------|------|------|------|
| Measured | int16 | x10 °C | 측정 온도 |
| Target | int16 | x10 °C | 목표 온도 |
| Tolerance | uint16 | x10 °C | 허용 오차 |
| Diff | uint16 | x10 °C | |Measured - Target| |

### Pass/Fail 판정

```
PASS if: |Measured - Target| <= Tolerance
FAIL if: |Measured - Target| > Tolerance
```

### 예제

```
Spec: Target=25.0°C (250), Tolerance=±5.0°C (50)
Result: Measured=27.3°C (273)

Diff = |273 - 250| = 23 (2.3°C)
23 <= 50 → PASS ✓
```

### Python 사용

```python
from psa_protocol import MLX90640Spec, SensorID

# 전체 평균 온도 측정 (25°C ± 10°C)
spec = MLX90640Spec(
    target_temp=250,    # 25.0°C
    tolerance=100,       # ±10.0°C
    pixel_x=0xFF,        # 전체 평균
    pixel_y=0xFF
)
client.set_spec_mlx90640(spec)

# 특정 픽셀 측정 (중앙 픽셀)
spec = MLX90640Spec(
    target_temp=370,    # 37.0°C
    tolerance=20,        # ±2.0°C
    pixel_x=15,          # 중앙 X
    pixel_y=12           # 중앙 Y
)

# 테스트 실행
report = client.test_single(SensorID.MLX90640)
result = report.results[0]

if result.result:
    temp_c = result.result.measured / 10.0
    print(f"Temperature: {temp_c:.1f}°C")
```

---

## 센서 ID 요약

| ID | 센서 | Spec Size | Result Size |
|-----|------|-----------|-------------|
| 0x01 | VL53L0X | 4 bytes | 8 bytes |
| 0x02 | MLX90640 | 6 bytes | 8 bytes |

## 데이터 직렬화

### Big-Endian 예제

```python
import struct

# VL53L0X Spec: target=500, tolerance=100
spec_bytes = struct.pack('>HH', 500, 100)
# → b'\x01\xf4\x00\x64'

# MLX90640 Spec: target=25.0°C, tolerance=5.0°C, average
spec_bytes = struct.pack('>hHBB', 250, 50, 0xFF, 0xFF)
# → b'\x00\xfa\x00\x32\xff\xff'
```

### 결과 파싱

```python
import struct

# VL53L0X Result (8 bytes)
measured, target, tolerance, diff = struct.unpack('>HHHH', result_data)

# MLX90640 Result (8 bytes)
measured, target, tolerance, diff = struct.unpack('>hhHH', result_data)
```
