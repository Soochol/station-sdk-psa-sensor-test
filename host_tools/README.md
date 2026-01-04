# Station SDK - PSA Sensor Test - Host Tools

Station SDK PSA 센서 테스트 펌웨어를 위한 Python 테스트 프레임워크입니다.

## 설치

```bash
cd host_tools
pip install -r requirements.txt
```

## 프로젝트 구조

```
host_tools/
├── psa_protocol/          # 프로토콜 구현 패키지
│   ├── constants.py       # 프로토콜 상수
│   ├── crc.py            # CRC-8 CCITT
│   ├── frame.py          # 프레임 파싱/빌드
│   ├── transport.py      # Serial 통신
│   ├── client.py         # 고수준 API
│   ├── sensors.py        # 센서 데이터 구조
│   └── exceptions.py     # 예외 클래스
│
├── tests/                 # pytest 테스트
│   ├── test_protocol/    # 단위 테스트 (하드웨어 불필요)
│   ├── test_integration/ # 통합 테스트
│   └── test_sensors/     # 센서별 테스트
│
├── conftest.py           # pytest 설정
├── pytest.ini            # pytest 옵션
└── requirements.txt      # 의존성
```

## 사용법

### Python 스크립트로 직접 사용

```python
from psa_protocol import SerialTransport, PSAClient
from psa_protocol import MLX90640Spec, VL53L0XSpec

# 연결
transport = SerialTransport('/dev/ttyUSB0', baudrate=115200)
transport.open()
client = PSAClient(transport)

# PING 테스트
major, minor, patch = client.ping()
print(f"Firmware v{major}.{minor}.{patch}")

# 센서 목록 조회
sensors = client.get_sensor_list()
for s in sensors:
    print(f"  - {s.name} (ID: 0x{s.sensor_id:02X})")

# MLX90640 테스트 (37°C ± 5°C)
spec = MLX90640Spec(target_temp=3700, tolerance=500)
client.set_spec_mlx90640(spec)
report = client.test_single(0x01)
print(f"MLX90640: {report.results[0].status_name}")

# VL53L0X 테스트 (500mm ± 100mm)
spec = VL53L0XSpec(target_dist=500, tolerance=100)
client.set_spec_vl53l0x(spec)
report = client.test_single(0x02)
print(f"VL53L0X: {report.results[0].status_name}")

# 전체 센서 테스트
report = client.test_all()
print(f"전체: {report.pass_count}/{report.sensor_count} 통과")

transport.close()
```

### pytest로 테스트 실행

```bash
# 단위 테스트 (하드웨어 불필요)
pytest tests/test_protocol/ -v

# 통합 테스트 (하드웨어 필요)
pytest tests/test_integration/ --serial-port=/dev/ttyUSB0

# 센서별 테스트
pytest tests/test_sensors/test_mlx90640.py --serial-port=/dev/ttyUSB0

# 전체 테스트
pytest tests/ --serial-port=/dev/ttyUSB0

# 하드웨어 테스트 건너뛰기
pytest tests/ --skip-hardware
```

### 환경 변수

```bash
export PSA_SERIAL_PORT=/dev/ttyUSB0  # 기본 시리얼 포트
pytest tests/
```

## 프로토콜 개요

### 프레임 포맷
```
[STX 0x02][LEN][CMD][PAYLOAD...][CRC-8][ETX 0x03]
```

### 명령어

| 코드 | 이름 | 설명 |
|------|------|------|
| 0x01 | PING | 연결 확인 |
| 0x10 | TEST_ALL | 전체 센서 테스트 |
| 0x11 | TEST_SINGLE | 개별 센서 테스트 |
| 0x12 | GET_SENSOR_LIST | 센서 목록 조회 |
| 0x20 | SET_SPEC | 테스트 스펙 설정 |
| 0x21 | GET_SPEC | 테스트 스펙 조회 |

### 센서 ID

| ID | 센서 |
|----|------|
| 0x01 | MLX90640 (IR 열화상) |
| 0x02 | VL53L0X (ToF 거리) |

### 테스트 결과

| 코드 | 상태 |
|------|------|
| 0x00 | PASS |
| 0x01 | FAIL_NO_ACK |
| 0x02 | FAIL_TIMEOUT |
| 0x03 | FAIL_INVALID |
| 0x04 | FAIL_INIT |
| 0x05 | FAIL_NO_SPEC |
| 0xFF | NOT_TESTED |

## 예제

### 간단한 테스트 스크립트

```python
#!/usr/bin/env python3
from psa_protocol import SerialTransport, PSAClient
from psa_protocol import MLX90640Spec, VL53L0XSpec, TestStatus

def main():
    with SerialTransport('/dev/ttyUSB0') as transport:
        client = PSAClient(transport)

        # 연결 확인
        version = client.ping()
        print(f"Connected to PSA Sensor Test v{version[0]}.{version[1]}.{version[2]}")

        # 스펙 설정
        client.set_spec_mlx90640(MLX90640Spec(target_temp=2500, tolerance=2000))
        client.set_spec_vl53l0x(VL53L0XSpec(target_dist=500, tolerance=500))

        # 테스트 실행
        report = client.test_all()

        # 결과 출력
        print(f"\n테스트 결과: {report.pass_count}/{report.sensor_count} 통과")
        for result in report.results:
            status = "✓" if result.passed else "✗"
            print(f"  {status} {result.sensor_name}: {result.status_name}")
            if result.result:
                print(f"      {result.result}")

if __name__ == '__main__':
    main()
```
