"""
Protocol constants matching the firmware implementation.

Reference: include/protocol/protocol.h, include/sensors/sensor_types.h
"""

from enum import IntEnum

# Frame delimiters
STX = 0x02
ETX = 0x03

# Maximum payload size
MAX_PAYLOAD = 64


class Command(IntEnum):
    """Command codes (Host -> MCU)."""
    PING = 0x01
    TEST_ALL = 0x10
    TEST_SINGLE = 0x11
    GET_SENSOR_LIST = 0x12
    READ_SENSOR = 0x13
    SET_SPEC = 0x20
    GET_SPEC = 0x21


class Response(IntEnum):
    """Response codes (MCU -> Host)."""
    PONG = 0x01
    TEST_RESULT = 0x80
    SENSOR_LIST = 0x81
    SPEC_ACK = 0x82
    SPEC_DATA = 0x83
    SENSOR_DATA = 0x84
    NAK = 0xFE


class SensorID(IntEnum):
    """Sensor identification codes (must match MCU sensor_types.h)."""
    NONE = 0x00
    VL53L0X = 0x01      # ToF Distance Sensor
    MLX90640 = 0x02     # IR Thermal Array Sensor

    @classmethod
    def name_of(cls, sensor_id: int) -> str:
        """Get sensor name from ID."""
        names = {
            cls.NONE: "None",
            cls.MLX90640: "MLX90640",
            cls.VL53L0X: "VL53L0X",
        }
        return names.get(sensor_id, f"Unknown(0x{sensor_id:02X})")


class TestStatus(IntEnum):
    """Test status codes."""
    PASS = 0x00
    FAIL_NO_ACK = 0x01
    FAIL_TIMEOUT = 0x02
    FAIL_INVALID = 0x03
    FAIL_INIT = 0x04
    FAIL_NO_SPEC = 0x05
    NOT_TESTED = 0xFF

    @classmethod
    def name_of(cls, status: int) -> str:
        """Get status name from code."""
        names = {
            cls.PASS: "PASS",
            cls.FAIL_NO_ACK: "FAIL_NO_ACK",
            cls.FAIL_TIMEOUT: "FAIL_TIMEOUT",
            cls.FAIL_INVALID: "FAIL_INVALID",
            cls.FAIL_INIT: "FAIL_INIT",
            cls.FAIL_NO_SPEC: "FAIL_NO_SPEC",
            cls.NOT_TESTED: "NOT_TESTED",
        }
        return names.get(status, f"Unknown(0x{status:02X})")


class ErrorCode(IntEnum):
    """Error codes for NAK response."""
    NONE = 0x00
    UNKNOWN_CMD = 0x01
    INVALID_SENSOR_ID = 0x02
    INVALID_PAYLOAD = 0x03
    BUSY = 0x04
    CRC_FAIL = 0x05
    NO_SPEC = 0x06

    @classmethod
    def name_of(cls, error: int) -> str:
        """Get error name from code."""
        names = {
            cls.NONE: "NONE",
            cls.UNKNOWN_CMD: "UNKNOWN_CMD",
            cls.INVALID_SENSOR_ID: "INVALID_SENSOR_ID",
            cls.INVALID_PAYLOAD: "INVALID_PAYLOAD",
            cls.BUSY: "BUSY",
            cls.CRC_FAIL: "CRC_FAIL",
            cls.NO_SPEC: "NO_SPEC",
        }
        return names.get(error, f"Unknown(0x{error:02X})")
