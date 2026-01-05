"""
High-level protocol client.

Provides a simple API for communicating with the PSA Sensor Test firmware.
"""

import time
import logging
from typing import List, Optional, Tuple

from .constants import Command, Response, SensorID, ErrorCode
from .frame import Frame, FrameBuilder, FrameParser, ParseResult
from .sensors import (
    MLX90640Spec, MLX90640Result,
    VL53L0XSpec, VL53L0XResult,
    SensorInfo, TestReport
)
from .transport import SerialTransport
from .exceptions import NAKError, TimeoutError, PSAProtocolError

logger = logging.getLogger(__name__)


class PSAClient:
    """High-level client for PSA Sensor Test protocol."""

    def __init__(
        self,
        transport: SerialTransport,
        response_timeout: float = 5.0,
        retry_count: int = 3
    ):
        """
        Initialize PSA client.

        Args:
            transport: Serial transport instance
            response_timeout: Timeout for response in seconds
            retry_count: Number of retries on timeout
        """
        self.transport = transport
        self.response_timeout = response_timeout
        self.retry_count = retry_count
        self._parser = FrameParser()

    def _send_and_receive(
        self,
        frame_data: bytes,
        expected_cmd: Optional[int] = None,
        timeout: Optional[float] = None
    ) -> Frame:
        """
        Send frame and wait for response.

        Args:
            frame_data: Frame bytes to send
            expected_cmd: Expected response command (None accepts any)
            timeout: Response timeout (None uses default)

        Returns:
            Received Frame

        Raises:
            NAKError: If NAK response received
            TimeoutError: If no response within timeout
        """
        self._parser.clear()
        self.transport.flush()

        timeout = timeout or self.response_timeout

        for attempt in range(self.retry_count):
            logger.debug(f"Sending frame (attempt {attempt + 1}): {frame_data.hex()}")
            self.transport.send(frame_data)

            start_time = time.time()
            while time.time() - start_time < timeout:
                data = self.transport.receive(timeout=0.1)
                if data:
                    self._parser.feed(data)

                    while True:
                        result, frame, _ = self._parser.parse()

                        if result == ParseResult.OK:
                            logger.debug(f"Received frame: cmd=0x{frame.cmd:02X}, "
                                        f"payload={frame.payload.hex() if frame.payload else 'none'}")

                            # Check for NAK
                            if frame.cmd == Response.NAK:
                                error_code = frame.payload[0] if frame.payload else 0
                                raise NAKError(error_code)

                            # Check expected command
                            if expected_cmd is None or frame.cmd == expected_cmd:
                                return frame

                        elif result == ParseResult.INCOMPLETE:
                            break
                        elif result == ParseResult.CRC_ERROR:
                            logger.warning("CRC error in received frame")
                        elif result == ParseResult.FORMAT_ERROR:
                            logger.warning("Format error in received frame")

            logger.warning(f"Timeout on attempt {attempt + 1}")

        raise TimeoutError(timeout, self.retry_count)

    def ping(self) -> Tuple[int, int, int]:
        """
        Send PING and return firmware version.

        Returns:
            Tuple of (major, minor, patch) version numbers
        """
        frame = self._send_and_receive(
            FrameBuilder.build_ping(),
            Response.PONG
        )
        major = frame.payload[0]
        minor = frame.payload[1]
        patch = frame.payload[2]
        logger.info(f"Firmware version: {major}.{minor}.{patch}")
        return (major, minor, patch)

    def get_sensor_list(self) -> List[SensorInfo]:
        """
        Get list of registered sensors.

        Returns:
            List of SensorInfo objects
        """
        frame = self._send_and_receive(
            FrameBuilder.build_get_sensor_list(),
            Response.SENSOR_LIST
        )

        sensors = []
        idx = 0
        count = frame.payload[idx]; idx += 1

        for _ in range(count):
            sensor_id = frame.payload[idx]; idx += 1
            name_len = frame.payload[idx]; idx += 1
            name = frame.payload[idx:idx + name_len].decode('ascii')
            idx += name_len
            sensors.append(SensorInfo(sensor_id, name))

        logger.info(f"Found {len(sensors)} sensors: {[s.name for s in sensors]}")
        return sensors

    def set_spec_mlx90640(self, spec: MLX90640Spec) -> bool:
        """
        Set MLX90640 specification.

        Args:
            spec: MLX90640Spec object with target and tolerance

        Returns:
            True if successful
        """
        frame = self._send_and_receive(
            FrameBuilder.build_set_spec(SensorID.MLX90640, spec.to_bytes()),
            Response.SPEC_ACK
        )
        success = frame.payload[0] == SensorID.MLX90640
        logger.info(f"Set MLX90640 spec: {spec} -> {'OK' if success else 'FAIL'}")
        return success

    def set_spec_vl53l0x(self, spec: VL53L0XSpec) -> bool:
        """
        Set VL53L0X specification.

        Args:
            spec: VL53L0XSpec object with target and tolerance

        Returns:
            True if successful
        """
        frame = self._send_and_receive(
            FrameBuilder.build_set_spec(SensorID.VL53L0X, spec.to_bytes()),
            Response.SPEC_ACK
        )
        success = frame.payload[0] == SensorID.VL53L0X
        logger.info(f"Set VL53L0X spec: {spec} -> {'OK' if success else 'FAIL'}")
        return success

    def get_spec_mlx90640(self) -> MLX90640Spec:
        """
        Get MLX90640 specification.

        Returns:
            MLX90640Spec object
        """
        frame = self._send_and_receive(
            FrameBuilder.build_get_spec(SensorID.MLX90640),
            Response.SPEC_DATA
        )
        spec = MLX90640Spec.from_bytes(frame.payload[1:])
        logger.info(f"Got MLX90640 spec: {spec}")
        return spec

    def get_spec_vl53l0x(self) -> VL53L0XSpec:
        """
        Get VL53L0X specification.

        Returns:
            VL53L0XSpec object
        """
        frame = self._send_and_receive(
            FrameBuilder.build_get_spec(SensorID.VL53L0X),
            Response.SPEC_DATA
        )
        spec = VL53L0XSpec.from_bytes(frame.payload[1:])
        logger.info(f"Got VL53L0X spec: {spec}")
        return spec

    def test_single(self, sensor_id: int, timeout: Optional[float] = None) -> TestReport:
        """
        Run test on single sensor.

        Args:
            sensor_id: Sensor ID to test
            timeout: Test timeout (None uses default, recommend 10s for sensor tests)

        Returns:
            TestReport with single sensor result
        """
        # Sensor tests take longer
        timeout = timeout or 10.0

        frame = self._send_and_receive(
            FrameBuilder.build_test_single(sensor_id),
            Response.TEST_RESULT,
            timeout=timeout
        )
        report = TestReport.from_bytes(frame.payload)
        logger.info(f"Single sensor test: {report}")
        return report

    def test_all(self, timeout: Optional[float] = None) -> TestReport:
        """
        Run test on all sensors.

        Args:
            timeout: Test timeout (None uses default, recommend 15s for all sensors)

        Returns:
            TestReport with all sensor results
        """
        # Testing all sensors takes longer
        timeout = timeout or 15.0

        frame = self._send_and_receive(
            FrameBuilder.build_test_all(),
            Response.TEST_RESULT,
            timeout=timeout
        )
        report = TestReport.from_bytes(frame.payload)
        logger.info(f"All sensors test: {report}")
        return report

    def test_mlx90640(
        self,
        target_celsius: float = 25.0,
        tolerance_celsius: float = 50.0
    ) -> TestReport:
        """
        Convenience method to test MLX90640 with given spec.

        Args:
            target_celsius: Target temperature in Celsius
            tolerance_celsius: Tolerance in Celsius

        Returns:
            TestReport with MLX90640 result
        """
        spec = MLX90640Spec(
            target_temp=int(target_celsius * 100),
            tolerance=int(tolerance_celsius * 100)
        )
        self.set_spec_mlx90640(spec)
        return self.test_single(SensorID.MLX90640)

    def test_vl53l0x(
        self,
        target_mm: int = 500,
        tolerance_mm: int = 500
    ) -> TestReport:
        """
        Convenience method to test VL53L0X with given spec.

        Args:
            target_mm: Target distance in mm
            tolerance_mm: Tolerance in mm

        Returns:
            TestReport with VL53L0X result
        """
        spec = VL53L0XSpec(target_dist=target_mm, tolerance=tolerance_mm)
        self.set_spec_vl53l0x(spec)
        return self.test_single(SensorID.VL53L0X)
