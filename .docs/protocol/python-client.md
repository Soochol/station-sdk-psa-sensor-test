# Python Client Guide

`psa_protocol` Python 패키지 사용 가이드입니다.

## 설치

```bash
cd host_tools
pip install -r requirements.txt
```

### 의존성

- `pyserial >= 3.5` - 시리얼 통신
- `pytest >= 7.4.0` - 테스트 실행

## 패키지 구조

```
psa_protocol/
├── __init__.py      # 공개 API
├── client.py        # PSAClient (고수준 API)
├── constants.py     # 프로토콜 상수
├── crc.py           # CRC-8 계산
├── exceptions.py    # 예외 클래스
├── frame.py         # 프레임 빌더/파서
├── sensors.py       # 센서 데이터 구조
└── transport.py     # 시리얼 전송 계층
```

## 빠른 시작

```python
from psa_protocol import SerialTransport, PSAClient
from psa_protocol import VL53L0XSpec, MLX90640Spec, SensorID

# 연결
with SerialTransport('/dev/ttyUSB0', baudrate=115200) as transport:
    client = PSAClient(transport)

    # PING
    version = client.ping()
    print(f"Firmware v{version[0]}.{version[1]}.{version[2]}")

    # 센서 목록
    sensors = client.get_sensor_list()
    for s in sensors:
        print(f"  {s.name} (ID: 0x{s.sensor_id:02X})")

    # VL53L0X 테스트
    client.set_spec_vl53l0x(VL53L0XSpec(target_dist=500, tolerance=100))
    report = client.test_single(SensorID.VL53L0X)

    result = report.results[0]
    print(f"Status: {result.status_name}")
    if result.result:
        print(f"Distance: {result.result.measured}mm")
```

## API Reference

### SerialTransport

시리얼 통신 계층입니다.

```python
class SerialTransport:
    def __init__(
        self,
        port: str,              # 시리얼 포트 ('/dev/ttyUSB0')
        baudrate: int = 115200, # 보드레이트
        timeout: float = 1.0    # 타임아웃 (초)
    )

    def open(self) -> None      # 포트 열기
    def close(self) -> None     # 포트 닫기
    def send(self, data: bytes) -> int     # 데이터 전송
    def receive(self, timeout: float) -> bytes  # 데이터 수신
    def flush(self) -> None     # 버퍼 비우기

    # Context manager 지원
    def __enter__(self) -> 'SerialTransport'
    def __exit__(self, ...)
```

#### 사용 예제

```python
# 방법 1: Context manager
with SerialTransport('/dev/ttyUSB0') as transport:
    # 사용...

# 방법 2: 수동 관리
transport = SerialTransport('/dev/ttyUSB0')
transport.open()
try:
    # 사용...
finally:
    transport.close()
```

### PSAClient

고수준 프로토콜 클라이언트입니다.

```python
class PSAClient:
    def __init__(
        self,
        transport: SerialTransport,
        response_timeout: float = 5.0,  # 응답 타임아웃
        retry_count: int = 3            # 재시도 횟수
    )

    # 연결 확인
    def ping(self) -> Tuple[int, int, int]

    # 센서 목록
    def get_sensor_list(self) -> List[SensorInfo]

    # 스펙 설정/조회
    def set_spec_vl53l0x(self, spec: VL53L0XSpec) -> bool
    def set_spec_mlx90640(self, spec: MLX90640Spec) -> bool
    def get_spec_vl53l0x(self) -> VL53L0XSpec
    def get_spec_mlx90640(self) -> MLX90640Spec

    # 테스트 실행
    def test_single(self, sensor_id: int, timeout: float = 10.0) -> TestReport
    def test_all(self, timeout: float = 15.0) -> TestReport

    # 편의 메서드
    def test_vl53l0x(self, target_mm: int, tolerance_mm: int) -> TestReport
    def test_mlx90640(self, target_celsius: float, tolerance_celsius: float) -> TestReport
```

### VL53L0XSpec

VL53L0X 테스트 스펙입니다.

```python
@dataclass
class VL53L0XSpec:
    target_dist: int   # 목표 거리 (mm)
    tolerance: int     # 허용 오차 (mm)

    def to_bytes(self) -> bytes        # 직렬화
    @classmethod
    def from_bytes(cls, data: bytes)   # 역직렬화
```

### MLX90640Spec

MLX90640 테스트 스펙입니다.

```python
@dataclass
class MLX90640Spec:
    target_temp: int   # 목표 온도 (x10 °C)
    tolerance: int     # 허용 오차 (x10 °C)
    pixel_x: int = 0xFF  # X 좌표 (0xFF = 평균)
    pixel_y: int = 0xFF  # Y 좌표 (0xFF = 평균)

    @property
    def target_celsius(self) -> float   # 섭씨 온도
    @property
    def tolerance_celsius(self) -> float
```

### TestReport

테스트 결과 리포트입니다.

```python
@dataclass
class TestReport:
    sensor_count: int           # 테스트한 센서 수
    pass_count: int             # 통과 수
    fail_count: int             # 실패 수
    timestamp: int              # 타임스탬프 (ms)
    results: List[SensorTestResult]  # 개별 결과

    @property
    def all_passed(self) -> bool  # 전체 통과 여부
```

### SensorTestResult

개별 센서 테스트 결과입니다.

```python
@dataclass
class SensorTestResult:
    sensor_id: int
    status: int
    result: Optional[Union[VL53L0XResult, MLX90640Result]]

    @property
    def status_name(self) -> str   # 상태 이름
    @property
    def sensor_name(self) -> str   # 센서 이름
    @property
    def passed(self) -> bool       # 통과 여부
```

### 결과 데이터 클래스

```python
@dataclass
class VL53L0XResult:
    measured: int      # 측정 거리 (mm)
    target: int        # 목표 거리 (mm)
    tolerance: int     # 허용 오차 (mm)
    diff: int          # 차이 (mm)

    @property
    def passed(self) -> bool  # diff <= tolerance

@dataclass
class MLX90640Result:
    max_temp: int      # 측정 온도 (x10 °C)
    target: int        # 목표 온도 (x10 °C)
    tolerance: int     # 허용 오차 (x10 °C)
    diff: int          # 차이 (x10 °C)

    @property
    def max_temp_celsius(self) -> float
    @property
    def passed(self) -> bool
```

## 예외 처리

```python
from psa_protocol.exceptions import (
    PSAProtocolError,  # 기본 예외
    TimeoutError,      # 타임아웃
    NAKError,          # NAK 응답
    ConnectionError,   # 연결 실패
)

try:
    report = client.test_single(SensorID.VL53L0X)
except TimeoutError as e:
    print(f"Timeout: {e.timeout}s, retries: {e.retries}")
except NAKError as e:
    print(f"NAK: {e.error_name} (0x{e.error_code:02X})")
except ConnectionError as e:
    print(f"Connection failed: {e}")
```

## 상수

```python
from psa_protocol.constants import (
    Command,      # 명령 코드
    Response,     # 응답 코드
    SensorID,     # 센서 ID
    TestStatus,   # 테스트 상태
    ErrorCode,    # 에러 코드
)

# 사용 예
Command.PING          # 0x01
Response.TEST_RESULT  # 0x80
SensorID.VL53L0X      # 0x01
TestStatus.PASS       # 0x00
ErrorCode.CRC_FAIL    # 0x05
```

## 고급 사용법

### 저수준 프레임 작업

```python
from psa_protocol.frame import Frame, FrameBuilder, FrameParser

# 프레임 빌드
frame = FrameBuilder.build_ping()
frame = FrameBuilder.build_test_single(SensorID.VL53L0X)
frame = FrameBuilder.build_set_spec(SensorID.VL53L0X, spec_bytes)

# 프레임 파싱
parser = FrameParser()
parser.feed(received_data)
result, frame, consumed = parser.parse()

if result == ParseResult.OK:
    print(f"CMD: 0x{frame.cmd:02X}")
    print(f"Payload: {frame.payload.hex()}")
```

### CRC 계산

```python
from psa_protocol.crc import CRC8

data = bytes([0x00, 0x01])  # LENGTH + CMD
crc = CRC8.calculate(data)
print(f"CRC: 0x{crc:02X}")  # 0x07

# 검증
is_valid = CRC8.verify(data, expected_crc)
```

### 디버그 로깅

```python
import logging

# 상세 로그 활성화
logging.basicConfig(level=logging.DEBUG)
logging.getLogger('psa_protocol').setLevel(logging.DEBUG)

# 결과:
# DEBUG TX (5 bytes): 02 00 01 07 03
# DEBUG RX (8 bytes): 02 03 01 01 00 00 db 03
```

## 테스트 실행

```bash
# 단위 테스트 (하드웨어 불필요)
pytest tests/test_protocol/ -v

# 통합 테스트 (하드웨어 필요)
pytest tests/test_integration/ --serial-port=/dev/ttyUSB0 -v

# 센서별 테스트
pytest tests/test_sensors/test_vl53l0x.py --serial-port=/dev/ttyUSB0 -v

# 전체 테스트
pytest tests/ --serial-port=/dev/ttyUSB0 -v

# 하드웨어 테스트 건너뛰기
pytest tests/ --skip-hardware
```

## 완전한 예제

```python
#!/usr/bin/env python3
"""PSA Sensor Test Example"""

from psa_protocol import (
    SerialTransport, PSAClient,
    VL53L0XSpec, MLX90640Spec, SensorID, TestStatus
)

def main():
    with SerialTransport('/dev/ttyUSB0') as transport:
        client = PSAClient(transport, response_timeout=10.0)

        # 연결 확인
        version = client.ping()
        print(f"Connected: PSA Sensor Test v{version[0]}.{version[1]}.{version[2]}")

        # 센서 목록 출력
        print("\nRegistered Sensors:")
        for sensor in client.get_sensor_list():
            print(f"  - {sensor.name} (0x{sensor.sensor_id:02X})")

        # VL53L0X 테스트
        print("\n=== VL53L0X Test ===")
        client.set_spec_vl53l0x(VL53L0XSpec(target_dist=300, tolerance=50))
        report = client.test_single(SensorID.VL53L0X)

        for result in report.results:
            status_icon = "✓" if result.passed else "✗"
            print(f"{status_icon} {result.sensor_name}: {result.status_name}")

            if result.result:
                r = result.result
                print(f"   Measured: {r.measured}mm")
                print(f"   Target: {r.target}mm ± {r.tolerance}mm")
                print(f"   Diff: {r.diff}mm")

        # 전체 테스트
        print("\n=== All Sensors Test ===")
        client.set_spec_mlx90640(MLX90640Spec(target_temp=250, tolerance=100))
        report = client.test_all()

        print(f"Result: {report.pass_count}/{report.sensor_count} passed")

if __name__ == '__main__':
    main()
```
