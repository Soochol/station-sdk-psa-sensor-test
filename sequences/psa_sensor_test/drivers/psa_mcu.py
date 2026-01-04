"""
PSA MCU Driver Module

Driver for communicating with STM32H723 MCU via UART protocol.
Uses libs/psa_protocol wrapper for clean imports.
"""

import asyncio
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Import BaseDriver - try relative import first, fallback to direct import
try:
    from .base import BaseDriver
except ImportError:
    from base import BaseDriver

# Add libs folder to sys.path for psa_protocol import
_libs_path = Path(__file__).parent.parent / "libs"
if str(_libs_path) not in sys.path:
    sys.path.insert(0, str(_libs_path))

from psa_protocol import (
    SerialTransport,
    PSAClient,
    VL53L0XSpec,
    MLX90640Spec,
    SensorID,
    TestReport,
)

logger = logging.getLogger(__name__)

# MCU는 온도를 celsius * 10 형태로 전송/수신
CELSIUS_MULTIPLIER = 10


class PSAMCUDriver(BaseDriver):
    """
    PSA MCU Driver for STM32H723 sensor test board.

    Communicates with MCU via UART protocol to test VL53L0X and MLX90640 sensors.

    Attributes:
        port: Serial port path
        baudrate: Communication speed
        timeout: Response timeout in seconds
    """

    def __init__(
        self,
        name: str = "PSAMCUDriver",
        config: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize PSA MCU driver.

        Args:
            name: Driver name
            config: Configuration with keys:
                - port: Serial port (default: "/dev/ttyUSB0")
                - baudrate: Baud rate (default: 115200)
                - timeout: Response timeout (default: 5.0)
        """
        super().__init__(name=name, config=config)

        self.port: str = self.config.get("port", "/dev/ttyUSB0")
        self.baudrate: int = self.config.get("baudrate", 115200)
        self.timeout: float = self.config.get("timeout", 5.0)

        self._transport: Optional[SerialTransport] = None
        self._client: Optional[PSAClient] = None
        self._firmware_version: Optional[Tuple[int, int, int]] = None

    async def connect(self) -> bool:
        """
        Connect to MCU via serial port.

        Returns:
            bool: True if connection successful
        """
        try:
            logger.info(f"Connecting to PSA MCU on {self.port} at {self.baudrate} bps")

            # Create transport and client (synchronous operations)
            self._transport = SerialTransport(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self._transport.open()

            self._client = PSAClient(
                transport=self._transport,
                response_timeout=self.timeout
            )

            # Verify connection with PING
            await asyncio.sleep(0.1)  # Brief delay for MCU ready
            self._firmware_version = await self._run_sync(self._client.ping)

            self._connected = True
            logger.info(f"Connected to PSA MCU, firmware v{self._firmware_version[0]}."
                       f"{self._firmware_version[1]}.{self._firmware_version[2]}")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to PSA MCU: {e}")
            await self.disconnect()
            return False

    async def disconnect(self) -> None:
        """Disconnect from MCU."""
        if self._transport:
            try:
                self._transport.close()
            except Exception:
                pass
            self._transport = None

        self._client = None
        self._connected = False
        logger.info("Disconnected from PSA MCU")

    async def reset(self) -> None:
        """Reset connection by re-pinging MCU."""
        if not self._connected or not self._client:
            raise RuntimeError("Not connected to MCU")

        self._firmware_version = await self._run_sync(self._client.ping)
        logger.info("MCU connection verified via PING")

    async def identify(self) -> str:
        """
        Return MCU identification string.

        Returns:
            str: Firmware version string
        """
        if self._firmware_version:
            return f"PSA-MCU,STM32H723,FW-{self._firmware_version[0]}." \
                   f"{self._firmware_version[1]}.{self._firmware_version[2]}"
        return "PSA-MCU,STM32H723,Unknown"

    # === Measurement Methods ===

    async def ping(self) -> str:
        """
        Send PING and return firmware version.

        Returns:
            str: Firmware version (e.g., "1.0.0")
        """
        if not self._client:
            raise RuntimeError("Not connected to MCU")

        version = await self._run_sync(self._client.ping)
        return f"{version[0]}.{version[1]}.{version[2]}"

    async def get_sensor_list(self) -> List[Dict[str, Any]]:
        """
        Get list of registered sensors.

        Returns:
            List of sensor info dictionaries
        """
        if not self._client:
            raise RuntimeError("Not connected to MCU")

        sensors = await self._run_sync(self._client.get_sensor_list)
        return [{"id": s.sensor_id, "name": s.name} for s in sensors]

    async def set_spec_vl53l0x(self, target_mm: int, tolerance_mm: int) -> bool:
        """
        Set VL53L0X test specification.

        Args:
            target_mm: Target distance in mm
            tolerance_mm: Tolerance in mm

        Returns:
            bool: True if successful
        """
        if not self._client:
            raise RuntimeError("Not connected to MCU")

        spec = VL53L0XSpec(target_dist=target_mm, tolerance=tolerance_mm)
        return await self._run_sync(self._client.set_spec_vl53l0x, spec)

    async def set_spec_mlx90640(
        self,
        target_celsius: float,
        tolerance_celsius: float,
    ) -> bool:
        """
        Set MLX90640 test specification.

        Args:
            target_celsius: Target temperature in Celsius
            tolerance_celsius: Tolerance in Celsius

        Returns:
            bool: True if successful
        """
        if not self._client:
            raise RuntimeError("Not connected to MCU")

        spec = MLX90640Spec(
            target_temp=int(target_celsius * CELSIUS_MULTIPLIER),
            tolerance=int(tolerance_celsius * CELSIUS_MULTIPLIER),
        )
        return await self._run_sync(self._client.set_spec_mlx90640, spec)

    async def test_vl53l0x(
        self,
        target_mm: int = 500,
        tolerance_mm: int = 100
    ) -> Dict[str, Any]:
        """
        Run VL53L0X distance test.

        Args:
            target_mm: Target distance in mm
            tolerance_mm: Tolerance in mm

        Returns:
            Dict with test results
        """
        if not self._client:
            raise RuntimeError("Not connected to MCU")

        # Set spec first
        await self.set_spec_vl53l0x(target_mm, tolerance_mm)

        # Run test (use longer timeout for sensor warmup)
        report = await self._run_sync(
            self._client.test_single,
            SensorID.VL53L0X,
            timeout=15.0
        )

        return self._parse_test_report(report, "VL53L0X")

    async def test_mlx90640(
        self,
        target_celsius: float = 25.0,
        tolerance_celsius: float = 10.0
    ) -> Dict[str, Any]:
        """
        Run MLX90640 temperature test.

        Args:
            target_celsius: Target temperature in Celsius
            tolerance_celsius: Tolerance in Celsius

        Returns:
            Dict with test results
        """
        if not self._client:
            raise RuntimeError("Not connected to MCU")

        # Set spec first (use average temperature)
        await self.set_spec_mlx90640(target_celsius, tolerance_celsius)

        # Run test (MLX90640 needs warmup time, use longer timeout)
        report = await self._run_sync(
            self._client.test_single,
            SensorID.MLX90640,
            timeout=20.0
        )

        return self._parse_test_report(report, "MLX90640")

    async def test_all(self) -> Dict[str, Any]:
        """
        Run test on all sensors.

        Returns:
            Dict with all test results
        """
        if not self._client:
            raise RuntimeError("Not connected to MCU")

        report = await self._run_sync(self._client.test_all, timeout=30.0)

        return {
            "passed": report.pass_count,
            "failed": report.fail_count,
            "timestamp": report.timestamp,
            "results": [
                {
                    "sensor_id": r.sensor_id,
                    "status": r.status,
                    "status_name": r.status_name,
                    "passed": r.passed,
                }
                for r in report.results
            ]
        }

    # === Helper Methods ===

    def _parse_test_report(self, report: TestReport, sensor_name: str) -> Dict[str, Any]:
        """Parse TestReport into dictionary."""
        result = {
            "sensor": sensor_name,
            "passed": report.pass_count > 0 and report.fail_count == 0,
            "timestamp": report.timestamp,
        }

        if report.results:
            r = report.results[0]
            result.update({
                "status": r.status,
                "status_name": r.status_name,
            })

            if r.result:
                if sensor_name == "VL53L0X":
                    result["measured_mm"] = r.result.measured
                    result["target_mm"] = r.result.target
                    result["tolerance_mm"] = r.result.tolerance
                    result["diff_mm"] = r.result.diff
                elif sensor_name == "MLX90640":
                    result["measured_celsius"] = r.result.max_temp / CELSIUS_MULTIPLIER
                    result["target_celsius"] = r.result.target / CELSIUS_MULTIPLIER
                    result["tolerance_celsius"] = r.result.tolerance / CELSIUS_MULTIPLIER
                    result["diff_celsius"] = r.result.diff / CELSIUS_MULTIPLIER

        return result

    async def _run_sync(self, func, *args, **kwargs) -> Any:
        """
        Run synchronous function in executor.

        The psa_protocol client uses synchronous serial communication,
        so we run it in a thread pool to avoid blocking.
        """
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, lambda: func(*args, **kwargs))
