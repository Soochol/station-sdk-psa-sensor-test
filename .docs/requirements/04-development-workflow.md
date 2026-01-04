# Station SDK - PSA Sensor Test - Development Workflow

## 1. Development Environment Setup

### 1.1 Required Tools

| Tool | Version | Purpose |
|------|---------|---------|
| STM32CubeMX | 6.x | HAL 코드 생성 |
| VSCode | Latest | IDE |
| PlatformIO | Latest | 빌드/업로드 |
| ST-Link Driver | Latest | 디버거 |
| Git | Latest | 버전 관리 |

### 1.2 Installation Steps

#### 1. STM32CubeMX
```
https://www.st.com/en/development-tools/stm32cubemx.html
→ 다운로드 및 설치
→ STM32H7 패키지 설치 (Help → Manage embedded software packages)
```

#### 2. VSCode + PlatformIO
```
1. VSCode 설치: https://code.visualstudio.com/
2. Extensions에서 "PlatformIO IDE" 검색 → 설치
3. 재시작 후 PlatformIO 아이콘 확인
```

#### 3. ST-Link Driver
```
https://www.st.com/en/development-tools/stsw-link009.html
→ 다운로드 및 설치
```

---

## 2. Project Setup Workflow

### 2.1 Overall Flow

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  STM32CubeMX    │ ──▶ │  Code Generate  │ ──▶ │   PlatformIO    │
│  Configuration  │     │  (HAL Drivers)  │     │   Development   │
└─────────────────┘     └─────────────────┘     └─────────────────┘
        │                                               │
        ▼                                               ▼
┌─────────────────┐                            ┌─────────────────┐
│  Save .ioc      │                            │  Build/Upload   │
│  (Reusable)     │                            │  Debug          │
└─────────────────┘                            └─────────────────┘
```

### 2.2 Step-by-Step Setup

#### Step 1: Clone/Create Project
```bash
cd C:\code
mkdir PSA-sensor-test
cd PSA-sensor-test
git init
```

#### Step 2: STM32CubeMX 설정
1. STM32CubeMX 실행
2. New Project → STM32H723VGT 선택
3. Peripherals 설정 (참조: [02-hardware-configuration.md](02-hardware-configuration.md))
4. Clock Configuration
5. Project Manager:
   - Name: `PSA-sensor-test`
   - Location: `C:\code\PSA-sensor-test\CubeMX`
   - Toolchain: `Makefile`
6. **GENERATE CODE** 클릭

#### Step 3: PlatformIO 프로젝트 초기화
```bash
cd C:\code\PSA-sensor-test
pio init --board genericSTM32H723VG --project-option "framework=stm32cube"
```

#### Step 4: 디렉토리 구조 생성
```bash
mkdir include\protocol
mkdir include\sensors
mkdir include\test
mkdir include\hal
mkdir src\protocol
mkdir src\sensors
mkdir src\test
mkdir src\hal
mkdir lib\MLX90640_API
mkdir lib\VL53L0X_API
```

#### Step 5: platformio.ini 설정
`platformio.ini` 파일 수정 (참조: [03-project-structure.md](03-project-structure.md))

#### Step 6: 외부 라이브러리 추가

**MLX90640 API:**
```
https://github.com/melexis/mlx90640-library
→ lib/MLX90640_API/ 에 복사
```

**VL53L0X API:**
```
https://www.st.com/en/embedded-software/stsw-img005.html
→ lib/VL53L0X_API/ 에 복사
```

---

## 3. Development Workflow

### 3.1 Daily Development Cycle

```
┌────────────────────────────────────────────────────────────┐
│                    Development Cycle                        │
├────────────────────────────────────────────────────────────┤
│                                                             │
│   1. 코드 수정 (VSCode)                                     │
│         │                                                   │
│         ▼                                                   │
│   2. 빌드 (Ctrl+Alt+B 또는 pio run)                         │
│         │                                                   │
│         ├── Error? → 수정 → 재빌드                          │
│         │                                                   │
│         ▼                                                   │
│   3. 업로드 (Ctrl+Alt+U 또는 pio run -t upload)             │
│         │                                                   │
│         ▼                                                   │
│   4. 테스트 (시리얼 모니터 또는 테스트 클라이언트)          │
│         │                                                   │
│         ├── Bug? → 디버그 → 수정                            │
│         │                                                   │
│         ▼                                                   │
│   5. Git Commit                                             │
│                                                             │
└────────────────────────────────────────────────────────────┘
```

### 3.2 Common Commands

| Action | Command | Shortcut |
|--------|---------|----------|
| 빌드 | `pio run` | Ctrl+Alt+B |
| 업로드 | `pio run -t upload` | Ctrl+Alt+U |
| 클린 빌드 | `pio run -t clean` | - |
| 시리얼 모니터 | `pio device monitor` | - |
| 디버그 시작 | F5 (VSCode) | F5 |

### 3.3 Serial Monitor

```bash
# 기본 모니터
pio device monitor --baud 115200

# Raw 모드 (바이너리 데이터 확인용)
pio device monitor --baud 115200 --raw
```

---

## 4. CubeMX Re-generation

하드웨어 설정 변경 시 CubeMX에서 코드 재생성이 필요합니다.

### 4.1 Regeneration Steps

```
1. STM32CubeMX에서 .ioc 파일 열기
2. 설정 변경 (예: 새 peripheral 추가)
3. GENERATE CODE 클릭
4. PlatformIO에서 빌드 테스트
```

### 4.2 Safe Zones (User Code)

CubeMX 생성 파일 내 `USER CODE` 영역은 보존됩니다:

```c
/* USER CODE BEGIN 0 */
// 이 영역의 코드는 재생성 시 유지됨
/* USER CODE END 0 */
```

### 4.3 Recommended Practice

> **Best Practice:** CubeMX 생성 파일은 직접 수정하지 않고,
> 별도의 소스 파일(src/)에서 초기화 함수를 호출하는 방식을 권장합니다.

```c
// src/main.c - PlatformIO 메인 파일
#include "main.h"       // CubeMX 생성
#include "gpio.h"       // CubeMX 생성
#include "i2c.h"        // CubeMX 생성
#include "usart.h"      // CubeMX 생성

// CubeMX 생성 핸들 (extern)
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

int main(void) {
    // CubeMX 생성 초기화 함수 호출
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    // 사용자 코드
    Protocol_Init();
    SensorManager_Init();
    TestRunner_Init();

    while (1) {
        Protocol_Process();
    }
}
```

---

## 5. Debugging

### 5.1 Debug Configuration

`.vscode/launch.json`:
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "type": "platformio-debug",
            "request": "launch",
            "name": "PIO Debug",
            "executable": ".pio/build/stm32h723vg/firmware.elf",
            "projectEnvName": "stm32h723vg",
            "toolchainBinDir": "${env:HOME}/.platformio/packages/toolchain-gccarmnoneeabi/bin",
            "svdPath": "${env:HOME}/.platformio/platforms/ststm32/misc/svd/STM32H723.svd"
        }
    ]
}
```

### 5.2 Debug Features

| Feature | Description |
|---------|-------------|
| Breakpoints | 클릭으로 중단점 설정 |
| Step Over | F10 - 함수 건너뛰기 |
| Step Into | F11 - 함수 진입 |
| Step Out | Shift+F11 - 함수 탈출 |
| Variables | 변수 값 확인 |
| Registers | 레지스터 값 확인 |
| Peripherals | SVD 기반 주변장치 뷰 |

### 5.3 Printf Debugging (Optional)

```c
// config.h
#define DEBUG_UART_ENABLED 1

// debug.h
#if DEBUG_UART_ENABLED
    #define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...)
#endif

// syscalls.c (printf 리다이렉션)
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
```

---

## 6. Testing

### 6.1 Host Test Client (Python Example)

```python
# test_client.py
import serial
import struct

class SensorTestClient:
    STX = 0x02
    ETX = 0x03

    CMD_PING = 0x01
    CMD_TEST_ALL = 0x10
    CMD_SET_SPEC = 0x20

    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=10)

    def _calculate_crc(self, data):
        crc = 0
        for b in data:
            crc ^= b
        return crc

    def _build_frame(self, cmd, payload=b''):
        length = 1 + len(payload)
        data = bytes([length, cmd]) + payload
        crc = self._calculate_crc(data)
        return bytes([self.STX]) + data + bytes([crc, self.ETX])

    def _parse_response(self):
        # STX 찾기
        while True:
            b = self.ser.read(1)
            if b == bytes([self.STX]):
                break

        length = self.ser.read(1)[0]
        data = self.ser.read(length)
        crc = self.ser.read(1)[0]
        etx = self.ser.read(1)[0]

        # CRC 검증
        expected_crc = self._calculate_crc(bytes([length]) + data)
        if crc != expected_crc:
            raise ValueError("CRC mismatch")

        return data[0], data[1:]  # cmd, payload

    def ping(self):
        self.ser.write(self._build_frame(self.CMD_PING))
        cmd, payload = self._parse_response()
        return cmd == self.CMD_PING  # PONG

    def set_mlx90640_spec(self, target_temp, tolerance):
        # target_temp, tolerance: float (°C)
        target = int(target_temp * 100)
        tol = int(tolerance * 100)
        payload = struct.pack('<bhH', 0x01, target, tol)
        self.ser.write(self._build_frame(self.CMD_SET_SPEC, payload))
        cmd, _ = self._parse_response()
        return cmd == 0x82  # SPEC_ACK

    def set_vl53l0x_spec(self, target_dist, tolerance):
        # target_dist, tolerance: int (mm)
        payload = struct.pack('<bHH', 0x02, target_dist, tolerance)
        self.ser.write(self._build_frame(self.CMD_SET_SPEC, payload))
        cmd, _ = self._parse_response()
        return cmd == 0x82

    def test_all(self):
        self.ser.write(self._build_frame(self.CMD_TEST_ALL))
        cmd, payload = self._parse_response()
        return self._parse_test_result(payload)

    def _parse_test_result(self, payload):
        results = []
        count = payload[0]
        offset = 1

        for _ in range(count):
            sensor_id = payload[offset]
            status = payload[offset + 1]
            data_len = payload[offset + 2]
            data = payload[offset + 3:offset + 3 + data_len]
            offset += 3 + data_len

            results.append({
                'sensor_id': sensor_id,
                'status': status,
                'data': data
            })

        return results


# 사용 예시
if __name__ == '__main__':
    client = SensorTestClient('COM3')

    # 연결 확인
    print("Ping:", client.ping())

    # 스펙 설정
    client.set_mlx90640_spec(30.0, 1.0)  # 30°C ± 1°C
    client.set_vl53l0x_spec(500, 10)     # 500mm ± 10mm

    # 테스트 실행
    results = client.test_all()
    for r in results:
        status = "PASS" if r['status'] == 0 else f"FAIL({r['status']})"
        print(f"Sensor {r['sensor_id']}: {status}")
```

### 6.2 Running Test Client

```bash
# Python 환경 설정
pip install pyserial

# 테스트 실행
python test_client.py
```

---

## 7. Git Workflow

### 7.1 .gitignore

```gitignore
# PlatformIO
.pio/
.vscode/.browse.c_cpp.db*
.vscode/c_cpp_properties.json
.vscode/launch.json
.vscode/ipch

# STM32CubeMX
CubeMX/Drivers/
CubeMX/*.pdf
CubeMX/MDK-ARM/
CubeMX/STM32CubeIDE/

# Build
*.o
*.elf
*.bin
*.hex
*.map

# OS
.DS_Store
Thumbs.db

# IDE
*.swp
*~
```

### 7.2 Recommended Commit Flow

```bash
# 기능 개발
git checkout -b feature/add-mlx90640-driver
# ... 코드 작성 ...
git add .
git commit -m "feat: Add MLX90640 driver implementation"

# 메인 브랜치 머지
git checkout main
git merge feature/add-mlx90640-driver

# 태그 (릴리즈)
git tag -a v1.0.0 -m "Initial release"
```

### 7.3 Commit Message Convention

```
feat: 새로운 기능 추가
fix: 버그 수정
docs: 문서 수정
refactor: 코드 리팩토링
test: 테스트 코드
chore: 빌드, 설정 변경
```

---

## 8. Troubleshooting

### 8.1 Common Issues

| Issue | Solution |
|-------|----------|
| ST-Link not found | 드라이버 재설치, USB 케이블 확인 |
| Build fails with HAL errors | CubeMX 패키지 버전 확인, 재생성 |
| Upload fails | ST-Link 연결 확인, 보드 전원 확인 |
| I2C no response | Pull-up 저항, 센서 전원, 주소 확인 |
| UART no output | TX/RX 교차 연결, Baud rate 확인 |

### 8.2 Debug Checklist

```
☐ ST-Link 연결 상태 (LED 점등)
☐ 보드 전원 공급 (3.3V)
☐ CubeMX 설정과 실제 핀 연결 일치
☐ I2C Pull-up 저항 존재
☐ 센서 전원 공급
☐ UART TX↔RX 교차 연결
☐ 올바른 COM 포트 선택
☐ Baud rate 일치 (115200)
```

---

## 9. Next Steps

프로젝트 셋업 완료 후:

1. [ ] STM32CubeMX로 .ioc 파일 생성
2. [ ] PlatformIO 프로젝트 초기화
3. [ ] 기본 main.c로 LED 블링크 테스트
4. [ ] UART Echo 테스트
5. [ ] I2C 스캔 (센서 감지 확인)
6. [ ] 프로토콜 레이어 구현
7. [ ] 센서 드라이버 구현
8. [ ] 테스트 러너 구현
9. [ ] Host 테스트 클라이언트로 통합 테스트
