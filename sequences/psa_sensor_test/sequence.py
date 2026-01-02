"""
PSA Sensor Test Sequence Module

Automated test sequence for VL53L0X ToF distance sensor and
MLX90640 IR thermal sensor via STM32H723 MCU.
"""

import logging
from typing import Any, Dict, List, Optional

from .drivers.psa_mcu import PSAMCUDriver

logger = logging.getLogger(__name__)


class PSASensorTestSequence:
    """
    PSA Sensor Test Sequence.

    Tests VL53L0X (ToF distance) and MLX90640 (IR thermal) sensors
    connected to STM32H723 MCU via UART protocol.

    Attributes:
        name: Sequence name
        version: Sequence version
    """

    name = "PSA Sensor Test"
    version = "1.0.0"

    def __init__(
        self,
        hardware: Optional[Dict[str, Any]] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize sequence.

        Args:
            hardware: Hardware driver instances dictionary
            parameters: Test parameters dictionary
        """
        self.hardware = hardware or {}
        self.parameters = parameters or {}
        self.results: List[Dict[str, Any]] = []

        # Get hardware driver
        self.mcu: Optional[PSAMCUDriver] = self.hardware.get("psa_mcu")
        if self.mcu is None:
            # Create default driver if not provided
            self.mcu = PSAMCUDriver(config={
                "port": self.parameters.get("port", "/dev/ttyUSB0"),
                "baudrate": self.parameters.get("baudrate", 115200),
                "timeout": self.parameters.get("timeout", 5.0),
            })

        # Load parameters with defaults
        self.vl53l0x_target_mm: int = self.parameters.get("vl53l0x_target_mm", 500)
        self.vl53l0x_tolerance_mm: int = self.parameters.get("vl53l0x_tolerance_mm", 100)
        self.mlx90640_target_celsius: float = self.parameters.get("mlx90640_target_celsius", 25.0)
        self.mlx90640_tolerance_celsius: float = self.parameters.get("mlx90640_tolerance_celsius", 10.0)

        logger.debug(f"Initialized {self.name} v{self.version}")
        logger.debug(f"VL53L0X: target={self.vl53l0x_target_mm}mm, tolerance={self.vl53l0x_tolerance_mm}mm")
        logger.debug(f"MLX90640: target={self.mlx90640_target_celsius}C, tolerance={self.mlx90640_tolerance_celsius}C")

    # === Step Methods ===

    async def initialize(self) -> Dict[str, Any]:
        """
        Initialize hardware and verify connection.

        Step 1: Connect to MCU and verify firmware version.

        Returns:
            Dict: Initialization result
        """
        logger.info("Step 1: Initializing PSA MCU connection")
        result: Dict[str, Any] = {
            "step": "initialize",
            "status": "passed",
            "data": {},
        }

        try:
            if self.mcu:
                connected = await self.mcu.connect()
                if not connected:
                    result["status"] = "failed"
                    result["error"] = "Failed to connect to PSA MCU"
                    return result

                # Get device info
                idn = await self.mcu.identify()
                result["data"]["device_idn"] = idn

                # Get firmware version
                version = await self.mcu.ping()
                result["data"]["firmware_version"] = version

                # Get sensor list
                sensors = await self.mcu.get_sensor_list()
                result["data"]["sensors"] = sensors
                result["data"]["sensor_count"] = len(sensors)

                logger.info(f"Connected: {idn}")
                logger.info(f"Sensors: {[s['name'] for s in sensors]}")

        except Exception as e:
            result["status"] = "failed"
            result["error"] = str(e)
            logger.exception(f"Initialization failed: {e}")

        self.results.append(result)
        return result

    async def test_vl53l0x(self) -> Dict[str, Any]:
        """
        Test VL53L0X ToF distance sensor.

        Step 2: Set spec and run distance measurement test.

        Returns:
            Dict: Test result
        """
        logger.info("Step 2: Testing VL53L0X ToF sensor")
        result: Dict[str, Any] = {
            "step": "test_vl53l0x",
            "status": "passed",
            "data": {},
        }

        try:
            if not self.mcu or not await self.mcu.is_connected():
                result["status"] = "failed"
                result["error"] = "MCU not connected"
                return result

            # Run VL53L0X test
            test_result = await self.mcu.test_vl53l0x(
                target_mm=self.vl53l0x_target_mm,
                tolerance_mm=self.vl53l0x_tolerance_mm
            )

            result["data"] = test_result

            # Check pass/fail
            if not test_result.get("passed", False):
                result["status"] = "failed"
                result["error"] = f"VL53L0X test failed: {test_result.get('status_name', 'Unknown')}"
                if "measured_mm" in test_result:
                    result["error"] += f" (measured={test_result['measured_mm']}mm)"

            logger.info(f"VL53L0X result: {test_result}")

        except Exception as e:
            result["status"] = "failed"
            result["error"] = str(e)
            logger.exception(f"VL53L0X test failed: {e}")

        self.results.append(result)
        return result

    async def test_mlx90640(self) -> Dict[str, Any]:
        """
        Test MLX90640 IR thermal sensor.

        Step 3: Set spec and run temperature measurement test.

        Returns:
            Dict: Test result
        """
        logger.info("Step 3: Testing MLX90640 IR thermal sensor")
        result: Dict[str, Any] = {
            "step": "test_mlx90640",
            "status": "passed",
            "data": {},
        }

        try:
            if not self.mcu or not await self.mcu.is_connected():
                result["status"] = "failed"
                result["error"] = "MCU not connected"
                return result

            # Run MLX90640 test
            test_result = await self.mcu.test_mlx90640(
                target_celsius=self.mlx90640_target_celsius,
                tolerance_celsius=self.mlx90640_tolerance_celsius
            )

            result["data"] = test_result

            # Check pass/fail
            if not test_result.get("passed", False):
                result["status"] = "failed"
                result["error"] = f"MLX90640 test failed: {test_result.get('status_name', 'Unknown')}"
                if "measured_celsius" in test_result:
                    result["error"] += f" (measured={test_result['measured_celsius']:.1f}C)"

            logger.info(f"MLX90640 result: {test_result}")

        except Exception as e:
            result["status"] = "failed"
            result["error"] = str(e)
            logger.exception(f"MLX90640 test failed: {e}")

        self.results.append(result)
        return result

    async def finalize(self) -> Dict[str, Any]:
        """
        Clean up resources and disconnect.

        Step 99 (cleanup): Always runs, even if previous steps failed.

        Returns:
            Dict: Cleanup result
        """
        logger.info("Step 99: Finalizing and cleaning up")
        result: Dict[str, Any] = {
            "step": "finalize",
            "status": "passed",
            "data": {"cleanup_completed": False},
        }

        try:
            if self.mcu and await self.mcu.is_connected():
                await self.mcu.disconnect()

            result["data"]["cleanup_completed"] = True
            result["data"]["total_steps"] = len(self.results)

            # Summarize results
            passed_count = sum(1 for r in self.results if r.get("status") == "passed")
            failed_count = sum(1 for r in self.results if r.get("status") == "failed")
            result["data"]["passed_steps"] = passed_count
            result["data"]["failed_steps"] = failed_count
            result["data"]["overall_passed"] = failed_count == 0

            logger.info(f"Cleanup completed. Passed: {passed_count}, Failed: {failed_count}")

        except Exception as e:
            result["status"] = "failed"
            result["error"] = str(e)
            logger.exception(f"Finalization failed: {e}")

        self.results.append(result)
        return result

    # === Parameter Properties ===

    def get_vl53l0x_target_mm(self) -> int:
        """VL53L0X target distance parameter."""
        return self.vl53l0x_target_mm

    def get_vl53l0x_tolerance_mm(self) -> int:
        """VL53L0X tolerance parameter."""
        return self.vl53l0x_tolerance_mm

    def get_mlx90640_target_celsius(self) -> float:
        """MLX90640 target temperature parameter."""
        return self.mlx90640_target_celsius

    def get_mlx90640_tolerance_celsius(self) -> float:
        """MLX90640 tolerance parameter."""
        return self.mlx90640_tolerance_celsius
