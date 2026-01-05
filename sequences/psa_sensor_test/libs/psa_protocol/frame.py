"""
Frame parsing and building.

Frame Format: [STX][LEN][CMD][PAYLOAD...][CRC][ETX]
- STX: 0x02 (Start of frame)
- LEN: Payload length (0-64), NOT including CMD
- CMD: Command code
- PAYLOAD: Command-specific data (0-64 bytes)
- CRC: CRC-8 CCITT of LEN+CMD+PAYLOAD
- ETX: 0x03 (End of frame)

Reference: src/protocol/frame.c
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

from .constants import STX, ETX, MAX_PAYLOAD, Command
from .crc import CRC8


class ParseResult(Enum):
    """Frame parse result codes."""
    OK = 0
    INCOMPLETE = 1
    CRC_ERROR = 2
    FORMAT_ERROR = 3


@dataclass
class Frame:
    """Protocol frame structure."""
    cmd: int
    payload: bytes = field(default_factory=bytes)

    def __post_init__(self):
        if isinstance(self.payload, (list, tuple)):
            self.payload = bytes(self.payload)
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError(f"Payload exceeds maximum size ({MAX_PAYLOAD})")

    @property
    def payload_len(self) -> int:
        return len(self.payload)


class FrameBuilder:
    """Builds frames for transmission."""

    @staticmethod
    def build(frame: Frame) -> bytes:
        """
        Build complete frame with CRC.

        Args:
            frame: Frame object with cmd and payload

        Returns:
            Complete frame bytes ready for transmission
        """
        # Frame: [STX][LEN][CMD][PAYLOAD...][CRC][ETX]
        length = len(frame.payload)

        # CRC covers: LEN + CMD + PAYLOAD
        crc_data = bytes([length, frame.cmd]) + frame.payload
        crc = CRC8.calculate(crc_data)

        return bytes([STX, length, frame.cmd]) + frame.payload + bytes([crc, ETX])

    @staticmethod
    def build_ping() -> bytes:
        """Build PING command frame."""
        return FrameBuilder.build(Frame(Command.PING))

    @staticmethod
    def build_test_all() -> bytes:
        """Build TEST_ALL command frame."""
        return FrameBuilder.build(Frame(Command.TEST_ALL))

    @staticmethod
    def build_test_single(sensor_id: int) -> bytes:
        """Build TEST_SINGLE command frame."""
        return FrameBuilder.build(Frame(Command.TEST_SINGLE, bytes([sensor_id])))

    @staticmethod
    def build_get_sensor_list() -> bytes:
        """Build GET_SENSOR_LIST command frame."""
        return FrameBuilder.build(Frame(Command.GET_SENSOR_LIST))

    @staticmethod
    def build_set_spec(sensor_id: int, spec_data: bytes) -> bytes:
        """Build SET_SPEC command frame."""
        payload = bytes([sensor_id]) + spec_data
        return FrameBuilder.build(Frame(Command.SET_SPEC, payload))

    @staticmethod
    def build_get_spec(sensor_id: int) -> bytes:
        """Build GET_SPEC command frame."""
        return FrameBuilder.build(Frame(Command.GET_SPEC, bytes([sensor_id])))


class FrameParser:
    """Parses frames from byte stream."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> None:
        """Add data to parse buffer."""
        self._buffer.extend(data)

    def parse(self) -> Tuple[ParseResult, Optional[Frame], int]:
        """
        Attempt to parse a frame from the buffer.

        Returns:
            Tuple of (result, frame, consumed_bytes)
            - result: ParseResult indicating success or error type
            - frame: Parsed Frame object if successful, None otherwise
            - consumed_bytes: Number of bytes consumed from buffer
        """
        consumed = 0

        # Search for STX
        while consumed < len(self._buffer) and self._buffer[consumed] != STX:
            consumed += 1

        if consumed >= len(self._buffer):
            self._buffer = bytearray()
            return (ParseResult.INCOMPLETE, None, 0)

        # Remove bytes before STX
        if consumed > 0:
            self._buffer = self._buffer[consumed:]
            consumed = 0

        # Need at least STX + LEN
        if len(self._buffer) < 2:
            return (ParseResult.INCOMPLETE, None, 0)

        # Get payload length
        payload_len = self._buffer[1]
        if payload_len > MAX_PAYLOAD:
            self._buffer = self._buffer[1:]
            return (ParseResult.FORMAT_ERROR, None, 1)

        # Expected frame size: STX + LEN + CMD + PAYLOAD + CRC + ETX
        expected_size = 1 + 1 + 1 + payload_len + 1 + 1

        if len(self._buffer) < expected_size:
            return (ParseResult.INCOMPLETE, None, 0)

        # Verify ETX
        if self._buffer[expected_size - 1] != ETX:
            self._buffer = self._buffer[1:]
            return (ParseResult.FORMAT_ERROR, None, 1)

        # Verify CRC (covers LEN + CMD + PAYLOAD)
        crc_data = bytes(self._buffer[1:expected_size - 2])
        calc_crc = CRC8.calculate(crc_data)
        recv_crc = self._buffer[expected_size - 2]

        if calc_crc != recv_crc:
            self._buffer = self._buffer[expected_size:]
            return (ParseResult.CRC_ERROR, None, expected_size)

        # Parse frame
        cmd = self._buffer[2]
        payload = bytes(self._buffer[3:3 + payload_len])

        self._buffer = self._buffer[expected_size:]
        return (ParseResult.OK, Frame(cmd, payload), expected_size)

    def clear(self) -> None:
        """Clear parse buffer."""
        self._buffer = bytearray()

    @property
    def buffer_size(self) -> int:
        """Get current buffer size."""
        return len(self._buffer)
