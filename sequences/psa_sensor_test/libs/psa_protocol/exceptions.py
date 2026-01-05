"""
Custom exceptions for PSA protocol.
"""

from .constants import ErrorCode


class PSAProtocolError(Exception):
    """Base exception for PSA protocol errors."""
    pass


class NAKError(PSAProtocolError):
    """MCU returned NAK response."""

    def __init__(self, error_code: int):
        self.error_code = error_code
        self.error_name = ErrorCode.name_of(error_code)
        super().__init__(
            f"NAK received: {self.error_name} (0x{error_code:02X})"
        )


class FrameError(PSAProtocolError):
    """Frame parsing or building error."""
    pass


class CRCError(FrameError):
    """CRC verification failed."""

    def __init__(self, expected: int, received: int):
        self.expected = expected
        self.received = received
        super().__init__(
            f"CRC mismatch: expected 0x{expected:02X}, received 0x{received:02X}"
        )


class ConnectionError(PSAProtocolError):
    """Serial connection error."""
    pass


class TimeoutError(PSAProtocolError):
    """Response timeout error."""

    def __init__(self, timeout: float, retries: int = 0):
        self.timeout = timeout
        self.retries = retries
        msg = f"No response within {timeout}s"
        if retries > 0:
            msg += f" after {retries} retries"
        super().__init__(msg)
