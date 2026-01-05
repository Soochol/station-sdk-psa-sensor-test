# Frame Format

PSA 프로토콜의 프레임 구조를 정의합니다.

## 기본 프레임 구조

```
┌──────┬────────┬────────┬──────────────────┬──────┬──────┐
│ STX  │ LENGTH │  CMD   │     PAYLOAD      │ CRC  │ ETX  │
│ 0x02 │ 1 byte │ 1 byte │    0-64 bytes    │ 1 B  │ 0x03 │
└──────┴────────┴────────┴──────────────────┴──────┴──────┘
  [0]     [1]      [2]      [3..N+2]         [N+3]  [N+4]
```

## 필드 설명

| 필드 | 오프셋 | 크기 | 값 | 설명 |
|------|--------|------|-----|------|
| STX | 0 | 1 | 0x02 | 프레임 시작 마커 |
| LENGTH | 1 | 1 | 0-64 | Payload 바이트 수 |
| CMD | 2 | 1 | - | 명령/응답 코드 |
| PAYLOAD | 3 | N | - | 명령별 데이터 |
| CRC | 3+N | 1 | - | CRC-8 체크섬 |
| ETX | 4+N | 1 | 0x03 | 프레임 종료 마커 |

## 프레임 크기

- **최소 크기**: 5 bytes (Payload 없음)
- **최대 크기**: 69 bytes (Payload 64 bytes)

```
최소: [STX][LEN=0][CMD][CRC][ETX] = 5 bytes
최대: [STX][LEN=64][CMD][PAYLOAD x 64][CRC][ETX] = 69 bytes
```

## CRC-8 계산

CRC-8 CCITT 알고리즘을 사용합니다.

### 특성
- **Polynomial**: 0x07 (x⁸ + x² + x + 1)
- **Initial Value**: 0x00
- **범위**: LENGTH + CMD + PAYLOAD

### C 구현

```c
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    // ... (256 entries)
};

uint8_t CRC8_Calculate(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}
```

### Python 구현

```python
class CRC8:
    POLY = 0x07
    INIT = 0x00

    @classmethod
    def calculate(cls, data: bytes) -> int:
        crc = cls.INIT
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ cls.POLY
                else:
                    crc <<= 1
            crc &= 0xFF
        return crc
```

### CRC 계산 예제

```
PING 프레임: [02 00 01 07 03]

CRC 계산 범위: [00 01] (LENGTH + CMD)
CRC 결과: 0x07

검증:
- Input: 0x00, 0x01
- Step 1: crc = 0x00 ^ 0x00 = 0x00 → table[0x00] = 0x00
- Step 2: crc = 0x00 ^ 0x01 = 0x01 → table[0x01] = 0x07
- Result: 0x07 ✓
```

## 바이트 순서

모든 멀티바이트 값은 **Big-Endian** (네트워크 바이트 순서)을 사용합니다.

```
예: uint16 값 500 (0x01F4)
전송 순서: [0x01] [0xF4]
           MSB    LSB
```

## 프레임 파싱 상태 머신

```
         ┌─────────────────────────────────────────┐
         │                                         │
         ▼                                         │
    ┌─────────┐   STX    ┌─────────┐              │
───►│  IDLE   │────────►│  LEN    │              │
    └─────────┘          └────┬────┘              │
         ▲                    │                    │
         │               LEN byte                  │
         │                    ▼                    │
         │              ┌─────────┐               │
         │              │   CMD   │               │
         │              └────┬────┘               │
         │                   │                    │
         │              CMD byte                  │
         │                   ▼                    │
         │              ┌─────────┐               │
         │              │ PAYLOAD │◄──┐           │
         │              └────┬────┘   │           │
         │                   │        │ N bytes   │
         │              All received  │           │
         │                   ▼        │           │
         │              ┌─────────┐───┘           │
         │              │   CRC   │               │
         │              └────┬────┘               │
         │                   │                    │
         │              CRC byte                  │
         │                   ▼                    │
         │              ┌─────────┐   ETX + OK    │
         │    Error     │   ETX   │───────────────┘
         └──────────────┴─────────┘
```

## 에러 처리

### 파싱 에러

| 에러 | 원인 | 복구 방법 |
|------|------|----------|
| FORMAT_ERROR | 잘못된 STX/ETX 또는 LENGTH > 64 | 다음 STX까지 스킵 |
| CRC_ERROR | CRC 불일치 | 프레임 폐기, NAK 응답 |
| TIMEOUT | 프레임 수신 미완료 | 버퍼 클리어, 재요청 |

### 복구 메커니즘

1. **STX 동기화**: 수신 시 항상 STX(0x02)를 찾을 때까지 스킵
2. **타임아웃**: 바이트 간 1초 타임아웃 적용
3. **CRC 실패**: NAK(0xFE) + ERR_CRC_FAIL(0x05) 응답

## 프레임 예제

### PING (페이로드 없음)

```
Host → MCU:
┌──────┬──────┬──────┬──────┬──────┐
│ 0x02 │ 0x00 │ 0x01 │ 0x07 │ 0x03 │
│ STX  │ LEN  │ PING │ CRC  │ ETX  │
└──────┴──────┴──────┴──────┴──────┘

MCU → Host:
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ 0x02 │ 0x03 │ 0x01 │ 0x01 │ 0x00 │ 0x00 │ 0xDB │ 0x03 │
│ STX  │ LEN  │ PONG │ Major│ Minor│ Patch│ CRC  │ ETX  │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
                      └────── Version 1.0.0 ──────┘
```

### TEST_SINGLE (페이로드 있음)

```
Host → MCU:
┌──────┬──────┬──────┬──────┬──────┬──────┐
│ 0x02 │ 0x01 │ 0x11 │ 0x01 │ 0x2E │ 0x03 │
│ STX  │ LEN  │ CMD  │ ID   │ CRC  │ ETX  │
└──────┴──────┴──────┴──────┴──────┴──────┘
                      └─ VL53L0X (0x01)
```
