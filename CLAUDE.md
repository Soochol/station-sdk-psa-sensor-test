# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PSA (Preconfigured Sensor Array) Sensor Test Firmware for STM32H723VGT6 @ 384MHz. Tests and validates sensor data via UART protocol for host PC communication. Currently implements VL53L0X ToF distance sensor with framework for MLX90640 IR thermal sensor.

## Build Commands

```bash
pio run                    # Build firmware
pio run -t upload          # Build and flash via ST-Link
pio run -t clean           # Clean build artifacts
pio device monitor         # Serial monitor (115200 baud)
```

## Architecture

Layered architecture with strict top-down dependencies:

```
Application (main.cpp)
    ↓
Protocol Layer (frame.c, commands.c) - UART frame parsing, command dispatch
    ↓
Test Layer (test_runner.c) - Sequential test execution, fail-fast on first failure
    ↓
Sensor Layer (sensor_manager.c, vl53l0x.c) - Driver registry and implementations
    ↓
HAL Wrapper (uart_handler.c, i2c_handler.c) - Hardware abstraction
    ↓
STM32H7 HAL (CubeMX generated in Core/)
```

## Sensor Driver Interface

All sensors implement `SensorDriver_t` (defined in [sensor_manager.h](include/sensors/sensor_manager.h)):

```c
typedef struct SensorDriver {
    SensorID_t      id;
    const char*     name;
    HAL_StatusTypeDef   (*init)(void);
    void                (*deinit)(void);
    void                (*set_spec)(const SensorSpec_t* spec);
    void                (*get_spec)(SensorSpec_t* spec);
    bool                (*has_spec)(void);
    TestStatus_t        (*run_test)(SensorResult_t* result);
    // Serialization functions...
} SensorDriver_t;
```

New sensors: implement interface, register in `SensorManager_Init()`.

## Communication Protocol

UART4 (115200-8-N-1) frame format:
```
[STX 0x02][LEN][CMD][PAYLOAD...][CRC-8][ETX 0x03]
```

Commands: PING(0x01), TEST_ALL(0x10), TEST_SINGLE(0x11), GET_SENSOR_LIST(0x12), SET_SPEC(0x20), GET_SPEC(0x21)

Test status codes: PASS(0x00), FAIL_NO_ACK(0x01), FAIL_TIMEOUT(0x02), FAIL_INVALID(0x03), FAIL_INIT(0x04), FAIL_NO_SPEC(0x05), NOT_TESTED(0xFF)

## Key Configuration

All settings centralized in [config.h](include/config.h):
- Firmware version, timeouts, I2C addresses, protocol constants, watchdog settings

Hardware pins:
- UART4: PA11(RX), PA12(TX)
- I2C1: PB6(SCL), PB7(SDA) - VL53L0X @ 0x29

## STM32CubeMX Integration

- `.ioc` file: Hardware configuration source of truth
- `Core/`: CubeMX-generated files (HAL MSP, interrupt handlers, system init)
- User code in `src/` - not `Core/Src/main.c` (excluded from build)
- Safe regeneration: CubeMX preserves user code blocks in Core/

## Host Test Tools

Python test framework in `host_tools/`:
```bash
cd host_tools
pip install -r requirements.txt
pytest tests/test_protocol/ -v              # Unit tests (no hardware)
pytest tests/ --serial-port=/dev/ttyUSB0    # Integration tests
```

## MCP Debugger

`embedded-debugger` MCP server is available for on-chip debugging via ST-Link. Use this for setting breakpoints, inspecting variables, stepping through code, and reading memory/registers on the target MCU.

## Documentation

Comprehensive docs in `docs/`:
- [01-protocol-specification.md](docs/01-protocol-specification.md) - Frame format, commands, examples
- [02-hardware-configuration.md](docs/02-hardware-configuration.md) - Pin assignments, peripherals
- [05-code-architecture.md](docs/05-code-architecture.md) - Detailed design, state machines, data flow
