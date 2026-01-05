"""
Serial transport layer.

Provides reliable serial communication with background receive thread.
"""

import serial
import threading
import logging
from typing import Optional
from queue import Queue, Empty

from .exceptions import ConnectionError

logger = logging.getLogger(__name__)


class SerialTransport:
    """Serial communication transport layer."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
        read_timeout: float = 0.1
    ):
        """
        Initialize serial transport.

        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0' or 'COM3')
            baudrate: Baud rate (default: 115200)
            timeout: Default receive timeout in seconds
            read_timeout: Internal read timeout for background thread
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.read_timeout = read_timeout
        self._serial: Optional[serial.Serial] = None
        self._rx_queue: Queue = Queue()
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False

    def open(self) -> None:
        """Open serial port and start receive thread."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout
            )
            logger.info(f"Opened serial port {self.port} at {self.baudrate} bps")

            self._running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()

        except serial.SerialException as e:
            raise ConnectionError(f"Failed to open {self.port}: {e}") from e

    def close(self) -> None:
        """Close serial port and stop receive thread."""
        self._running = False

        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            logger.info(f"Closed serial port {self.port}")

    def send(self, data: bytes) -> int:
        """
        Send data over serial port.

        Args:
            data: Bytes to send

        Returns:
            Number of bytes sent

        Raises:
            ConnectionError: If port is not open
        """
        if not self._serial or not self._serial.is_open:
            raise ConnectionError("Serial port not open")

        try:
            count = self._serial.write(data)
            logger.debug(f"TX ({count} bytes): {data.hex(' ')}")
            return count
        except serial.SerialException as e:
            raise ConnectionError(f"Send failed: {e}") from e

    def receive(self, timeout: Optional[float] = None) -> bytes:
        """
        Receive data from queue.

        Args:
            timeout: Timeout in seconds (None uses default)

        Returns:
            Received bytes (empty if timeout)
        """
        try:
            data = self._rx_queue.get(timeout=timeout or self.timeout)
            return data
        except Empty:
            return b''

    def receive_all(self) -> bytes:
        """Receive all available data from queue."""
        data = bytearray()
        while True:
            try:
                data.extend(self._rx_queue.get_nowait())
            except Empty:
                break
        return bytes(data)

    def flush(self) -> None:
        """Flush receive queue."""
        while not self._rx_queue.empty():
            try:
                self._rx_queue.get_nowait()
            except Empty:
                break

        # Also flush serial buffers
        if self._serial and self._serial.is_open:
            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception:
                pass

    def _rx_loop(self) -> None:
        """Background receive thread."""
        while self._running and self._serial and self._serial.is_open:
            try:
                data = self._serial.read(256)
                if data:
                    logger.debug(f"RX ({len(data)} bytes): {data.hex(' ')}")
                    self._rx_queue.put(data)
            except serial.SerialException:
                break
            except Exception as e:
                logger.error(f"RX error: {e}")
                break

    @property
    def is_open(self) -> bool:
        """Check if port is open."""
        return self._serial is not None and self._serial.is_open

    def __enter__(self) -> 'SerialTransport':
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def __repr__(self) -> str:
        status = "open" if self.is_open else "closed"
        return f"SerialTransport({self.port}, {self.baudrate}, {status})"
