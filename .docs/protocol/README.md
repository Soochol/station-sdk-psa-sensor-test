# Station SDK - PSA Sensor Test Protocol

PSA(Preconfigured Sensor Array) 센서 테스트 펌웨어를 위한 UART 통신 프로토콜 문서입니다.

## 개요

이 프로토콜은 Host PC와 STM32H723 MCU 간의 센서 테스트 통신을 정의합니다.

### 주요 기능

- 센서 연결 테스트 (VL53L0X, MLX90640)
- 테스트 스펙 설정 및 조회
- 개별/전체 센서 테스트 실행
- 테스트 결과 리포팅

### 통신 설정

| 항목 | 값 |
|------|-----|
| 인터페이스 | UART4 (PA11/RX, PA12/TX) |
| Baud Rate | 115200 bps |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |

## 문서 구조

| 문서 | 설명 |
|------|------|
| [frame-format.md](frame-format.md) | 프레임 구조 및 CRC |
| [commands.md](commands.md) | 명령어 레퍼런스 |
| [sensors.md](sensors.md) | 센서별 데이터 구조 |
| [examples.md](examples.md) | 통신 예제 |
| [python-client.md](python-client.md) | Python 클라이언트 사용법 |

## 빠른 시작

### 1. Python 클라이언트 설치

```bash
cd host_tools
pip install -r requirements.txt
```

### 2. 기본 사용 예제

```python
from psa_protocol import SerialTransport, PSAClient, VL53L0XSpec, SensorID

with SerialTransport('/dev/ttyUSB0') as transport:
    client = PSAClient(transport)

    # 연결 확인
    version = client.ping()
    print(f"Firmware v{version[0]}.{version[1]}.{version[2]}")

    # 센서 목록 조회
    sensors = client.get_sensor_list()

    # VL53L0X 테스트
    client.set_spec_vl53l0x(VL53L0XSpec(target_dist=500, tolerance=100))
    report = client.test_single(SensorID.VL53L0X)
    print(f"Result: {report.results[0].status_name}")
```

## 아키텍처

```
┌─────────────────┐     UART      ┌─────────────────┐
│    Host PC      │◄────────────►│   STM32H723     │
│  (Python/C++)   │   115200bps   │   MCU @ 384MHz  │
└─────────────────┘               └─────────────────┘
        │                                 │
        ▼                                 ▼
┌─────────────────┐               ┌─────────────────┐
│  psa_protocol   │               │  Protocol Layer │
│  Python Package │               │  (frame.c)      │
└─────────────────┘               └─────────────────┘
                                          │
                                          ▼
                                  ┌─────────────────┐
                                  │  Sensor Manager │
                                  │  VL53L0X/MLX    │
                                  └─────────────────┘
```

## 프레임 구조 요약

```
┌──────┬────────┬────────┬──────────┬──────┬──────┐
│ STX  │ LENGTH │  CMD   │ PAYLOAD  │ CRC  │ ETX  │
│ 0x02 │ 1 byte │ 1 byte │ N bytes  │ 1 B  │ 0x03 │
└──────┴────────┴────────┴──────────┴──────┴──────┘
```

- **STX**: 프레임 시작 (0x02)
- **LENGTH**: Payload 길이 (0-64)
- **CMD**: 명령 코드
- **PAYLOAD**: 데이터 (가변 길이)
- **CRC**: CRC-8 CCITT (LENGTH + CMD + PAYLOAD)
- **ETX**: 프레임 종료 (0x03)

## 지원 센서

| 센서 | ID | 설명 |
|------|-----|------|
| VL53L0X | 0x01 | ToF 거리 센서 (30-2000mm) |
| MLX90640 | 0x02 | IR 열화상 센서 (32x24 픽셀) |

## 관련 소스 코드

### 펌웨어 (C)
- `src/protocol/frame.c` - 프레임 파싱/빌드
- `src/protocol/commands.c` - 명령 처리
- `src/sensors/sensor_manager.c` - 센서 관리

### Host Tools (Python)
- `host_tools/psa_protocol/frame.py` - 프레임 처리
- `host_tools/psa_protocol/client.py` - 고수준 API
- `host_tools/psa_protocol/sensors.py` - 센서 데이터 구조
