"""
PSA Protocol - Python implementation of PSA Sensor Test communication protocol.

This package provides:
- Protocol constants and error codes
- CRC-8 CCITT calculation
- Frame parsing and building
- Serial transport layer
- High-level protocol client
- Sensor data structures
"""

from .constants import (
    STX, ETX, MAX_PAYLOAD,
    Command, Response, SensorID, TestStatus, ErrorCode
)
from .crc import CRC8
from .exceptions import (
    PSAProtocolError, NAKError, FrameError, CRCError, ConnectionError, TimeoutError
)
from .frame import Frame, FrameBuilder, FrameParser, ParseResult
from .sensors import (
    MLX90640Spec, MLX90640Result,
    VL53L0XSpec, VL53L0XResult,
    SensorInfo, SensorTestResult, TestReport
)
from .transport import SerialTransport
from .client import PSAClient

__version__ = "1.0.0"
__all__ = [
    # Constants
    "STX", "ETX", "MAX_PAYLOAD",
    "Command", "Response", "SensorID", "TestStatus", "ErrorCode",
    # CRC
    "CRC8",
    # Exceptions
    "PSAProtocolError", "NAKError", "FrameError", "CRCError",
    "ConnectionError", "TimeoutError",
    # Frame
    "Frame", "FrameBuilder", "FrameParser", "ParseResult",
    # Sensors
    "MLX90640Spec", "MLX90640Result",
    "VL53L0XSpec", "VL53L0XResult",
    "SensorInfo", "SensorTestResult", "TestReport",
    # Transport
    "SerialTransport",
    # Client
    "PSAClient",
]
