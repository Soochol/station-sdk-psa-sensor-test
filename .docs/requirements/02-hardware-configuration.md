# PSA Sensor Test - Hardware Configuration

## 1. MCU Specification

### 1.1 Target MCU

| 항목 | 값 |
|------|-----|
| Part Number | STM32H723VGT6 |
| Core | ARM Cortex-M7 |
| Max Frequency | 550 MHz |
| Flash | 1 MB |
| SRAM | 564 KB |
| Package | LQFP100 |

### 1.2 Clock Configuration

```
┌─────────────────────────────────────────────────────────┐
│                   Clock Tree                             │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  HSE (8/25 MHz) ──┬── PLL1 ──── SYSCLK (275~550 MHz)    │
│                   │                                      │
│                   └── PLL2 ──── Peripheral Clocks        │
│                                                          │
│  HSI (64 MHz) ────── Backup Clock Source                │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

**권장 설정 (HSE 8MHz 기준):**

| 항목 | 값 |
|------|-----|
| HSE | 8 MHz (Crystal) |
| PLL1 Source | HSE |
| DIVM1 | /2 |
| MULN1 | ×137 |
| DIVP1 | /2 |
| SYSCLK | 274 MHz |
| HCLK | 274 MHz |
| APB1 | 137 MHz |
| APB2 | 137 MHz |

---

## 2. Peripheral Configuration

### 2.1 I2C1 (Sensor Communication)

| 항목 | 값 |
|------|-----|
| Mode | I2C |
| Speed Mode | Fast Mode (400 kHz) |
| Clock No Stretch | Disabled |
| Analog Filter | Enabled |
| Digital Filter | 0 |

**Pin Assignment:**

| Function | Pin | Alternate Function |
|----------|-----|-------------------|
| I2C1_SCL | PB6 | AF4 |
| I2C1_SDA | PB7 | AF4 |

> **Alternative Pins:** PB8 (SCL), PB9 (SDA)

### 2.2 USART1 (Host Communication)

| 항목 | 값 |
|------|-----|
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Hardware Flow Control | None |
| Oversampling | 16 |

**Pin Assignment:**

| Function | Pin | Alternate Function |
|----------|-----|-------------------|
| USART1_TX | PA9 | AF7 |
| USART1_RX | PA10 | AF7 |

**NVIC Settings:**
- USART1 global interrupt: **Enabled**
- Priority: 5 (조정 가능)

### 2.3 GPIO (Status LED - Optional)

| Pin | Mode | Label | Description |
|-----|------|-------|-------------|
| PC13 | GPIO_Output | LED_STATUS | 상태 표시 LED |

---

## 3. Sensor Specifications

### 3.1 MLX90640 (IR Thermal Sensor)

| 항목 | 값 |
|------|-----|
| Interface | I2C |
| I2C Address | 0x33 (7-bit) |
| Max I2C Speed | 1 MHz |
| Resolution | 32 × 24 pixels |
| Temperature Range | -40°C ~ 300°C |
| Accuracy | ±1°C (typical) |
| Field of View | 110° × 75° (wide) / 55° × 35° (narrow) |

**I2C Timing:**
```
┌──────────────────────────────────────────────────┐
│ Start → Address (0x33) → R/W → ACK → Data → Stop│
└──────────────────────────────────────────────────┘
```

**Key Registers:**

| Register | Address | Description |
|----------|---------|-------------|
| Status | 0x8000 | Device status |
| Control | 0x800D | Configuration |
| EEPROM | 0x2400~0x273F | Calibration data |
| RAM | 0x0400~0x06FF | Pixel data |

### 3.2 VL53L0X (ToF Distance Sensor)

| 항목 | 값 |
|------|-----|
| Interface | I2C |
| I2C Address | 0x29 (7-bit) |
| Max I2C Speed | 400 kHz |
| Measurement Range | 30mm ~ 2000mm |
| Accuracy | ±3% (typical) |
| Field of View | 25° |

**Key Registers:**

| Register | Address | Description |
|----------|---------|-------------|
| Model ID | 0xC0 | Should read 0xEE |
| Revision ID | 0xC2 | Revision number |
| Range Status | 0x14 | Measurement status |
| Range Value | 0x1E~0x1F | Distance (mm) |

---

## 4. Pin Map Summary

### 4.1 Required Pins

| Pin | Function | Description |
|-----|----------|-------------|
| PB6 | I2C1_SCL | Sensor I2C Clock |
| PB7 | I2C1_SDA | Sensor I2C Data |
| PA9 | USART1_TX | Host TX |
| PA10 | USART1_RX | Host RX |

### 4.2 Optional Pins

| Pin | Function | Description |
|-----|----------|-------------|
| PC13 | GPIO_Output | Status LED |
| PA13 | SWDIO | Debug (SWD) |
| PA14 | SWCLK | Debug (SWD) |

### 4.3 Visual Pin Map

```
                    STM32H723VGT (LQFP100)
                    ┌────────────────────┐
                    │                    │
     USART1_TX ──── │ PA9            PB6 │ ──── I2C1_SCL
     USART1_RX ──── │ PA10           PB7 │ ──── I2C1_SDA
                    │                    │
        SWDIO ──── │ PA13          PC13 │ ──── LED (Optional)
        SWCLK ──── │ PA14               │
                    │                    │
                    └────────────────────┘
```

---

## 5. STM32CubeMX Configuration Steps

### 5.1 New Project

1. STM32CubeMX 실행
2. **ACCESS TO MCU SELECTOR** 클릭
3. Part Number: `STM32H723VGT` 검색 → 선택
4. **Start Project** 클릭

### 5.2 System Core

#### RCC
```
System Core → RCC
├── High Speed Clock (HSE): Crystal/Ceramic Resonator
└── Low Speed Clock (LSE): Disable
```

#### SYS
```
System Core → SYS
├── Debug: Serial Wire
└── Timebase Source: SysTick
```

### 5.3 Connectivity

#### I2C1
```
Connectivity → I2C1
├── I2C: I2C
├── Configuration:
│   ├── Master Features:
│   │   └── I2C Speed Mode: Fast Mode
│   └── Slave Features: (기본값)
└── GPIO Settings:
    ├── PB6: I2C1_SCL
    └── PB7: I2C1_SDA
```

#### USART1
```
Connectivity → USART1
├── Mode: Asynchronous
├── Configuration:
│   ├── Baud Rate: 115200
│   ├── Word Length: 8 Bits
│   ├── Parity: None
│   └── Stop Bits: 1
├── NVIC Settings:
│   └── USART1 global interrupt: ✓ Enabled
└── GPIO Settings:
    ├── PA9: USART1_TX
    └── PA10: USART1_RX
```

### 5.4 GPIO (Optional LED)

```
Pinout View에서 PC13 클릭 → GPIO_Output
├── GPIO output level: Low
├── GPIO mode: Output Push Pull
├── GPIO Pull-up/Pull-down: No pull-up and no pull-down
├── Maximum output speed: Low
└── User Label: LED_STATUS
```

### 5.5 Clock Configuration

1. **Clock Configuration** 탭 이동
2. HSE 활성화 확인 (8 MHz 또는 25 MHz)
3. PLL Source Mux: **HSE** 선택
4. System Clock Mux: **PLLCLK** 선택
5. HCLK 목표값 입력 (예: 274 MHz)
6. **Resolve Clock Issues** 클릭하여 자동 최적화

### 5.6 Project Manager

```
Project Manager 탭:
├── Project:
│   ├── Project Name: PSA-sensor-test
│   ├── Project Location: C:\code\PSA-sensor-test\CubeMX
│   └── Toolchain/IDE: Makefile
│
├── Code Generator:
│   ├── STM32Cube MCU packages: Copy only necessary library files
│   ├── Generated files:
│   │   ├── ✓ Generate peripheral initialization as pair of .c/.h files
│   │   ├── ✓ Keep User Code when re-generating
│   │   └── ✓ Delete previously generated files when not re-generated
│   └── HAL Settings:
│       └── ✓ Set all free pins as analog (to optimize power)
│
└── Advanced Settings:
    └── Driver Selector: All HAL
```

### 5.7 Generate Code

1. **GENERATE CODE** 버튼 클릭
2. 생성 완료 후 폴더 구조 확인

---

## 6. Generated File Structure

```
CubeMX/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32h7xx_hal_conf.h
│   │   ├── stm32h7xx_it.h
│   │   └── gpio.h, i2c.h, usart.h
│   └── Src/
│       ├── main.c
│       ├── stm32h7xx_hal_msp.c
│       ├── stm32h7xx_it.c
│       ├── system_stm32h7xx.c
│       └── gpio.c, i2c.c, usart.c
├── Drivers/
│   ├── CMSIS/
│   └── STM32H7xx_HAL_Driver/
├── Makefile
└── PSA-sensor-test.ioc
```

---

## 7. Hardware Checklist

| 항목 | 확인 |
|------|------|
| STM32H723VGT 보드 준비 | ☐ |
| 외부 크리스탈 (HSE) 연결 | ☐ |
| ST-Link 연결 확인 | ☐ |
| I2C Pull-up 저항 (4.7kΩ) | ☐ |
| MLX90640 연결 (VCC, GND, SCL, SDA) | ☐ |
| VL53L0X 연결 (VCC, GND, SCL, SDA) | ☐ |
| UART-USB 변환기 연결 | ☐ |
| 전원 공급 확인 (3.3V) | ☐ |

---

## 8. I2C Bus Configuration

### 8.1 Pull-up Resistor

```
VCC (3.3V)
    │
    ├── 4.7kΩ ──── I2C_SCL ──── PB6
    │
    └── 4.7kΩ ──── I2C_SDA ──── PB7
```

> **Note:** 대부분의 센서 모듈에는 Pull-up 저항이 내장되어 있습니다.
> 여러 모듈 사용 시 총 저항값이 너무 낮아지지 않도록 확인하세요.

### 8.2 Multi-Sensor Bus

```
STM32H723VGT
    │
    ├── PB6 (SCL) ────┬──── MLX90640 (0x33)
    │                 │
    │                 └──── VL53L0X (0x29)
    │
    └── PB7 (SDA) ────┬──── MLX90640
                      │
                      └──── VL53L0X
```

두 센서가 동일한 I2C 버스를 공유하며, 서로 다른 I2C 주소를 사용합니다.
