"""
PSA Sensor Test Sequence Module (SDK 2.0)

Automated test sequence for VL53L0X ToF distance sensor and
MLX90640 IR thermal sensor via STM32H723 MCU.

This module uses the SDK 2.0 SequenceBase pattern with:
- setup(): Hardware initialization
- run(): Step-by-step execution with emit_* helpers
- teardown(): Resource cleanup
"""

import logging
import time
from typing import Any, Dict, Optional

from station_service_sdk import (
    SequenceBase,
    RunResult,
    ExecutionContext,
    SetupError,
    HardwareError,
)

logger = logging.getLogger(__name__)

# Lazy import for driver - allows metadata extraction without dependencies
PSAMCUDriver = None


def _get_driver_class():
    """Load driver class at runtime."""
    global PSAMCUDriver
    if PSAMCUDriver is None:
        try:
            # Try relative import first (works when loaded as a package)
            from .drivers.psa_mcu import PSAMCUDriver as _Driver
            PSAMCUDriver = _Driver
        except ImportError:
            # Fallback to dynamic loading (works when run as subprocess)
            import importlib.util
            import sys
            from pathlib import Path

            drivers_path = Path(__file__).parent / "drivers"
            driver_path = drivers_path / "psa_mcu.py"

            if not driver_path.exists():
                raise ImportError(f"Driver module not found: {driver_path}")

            # Add drivers folder to sys.path for base import
            if str(drivers_path) not in sys.path:
                sys.path.insert(0, str(drivers_path))

            spec = importlib.util.spec_from_file_location("psa_mcu", driver_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"Failed to load driver module: {driver_path}")

            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            PSAMCUDriver = module.PSAMCUDriver
    return PSAMCUDriver


class PSASensorTestSequence(SequenceBase):
    """
    PSA Sensor Test Sequence (SDK 2.0).

    Tests VL53L0X (ToF distance) and MLX90640 (IR thermal) sensors
    connected to STM32H723 MCU via UART protocol.

    Attributes:
        name: Sequence identifier
        version: Semantic version
        description: Human-readable description
    """

    # Class-level metadata (required by SequenceBase)
    name = "psa_sensor_test"
    version = "2.0.0"
    description = "PSA 센서 테스트 시퀀스 (VL53L0X ToF, MLX90640 IR)"

    def __init__(
        self,
        context: ExecutionContext,
        hardware_config: Optional[Dict[str, Dict[str, Any]]] = None,
        parameters: Optional[Dict[str, Any]] = None,
        **kwargs,
    ) -> None:
        """
        Initialize sequence.

        Args:
            context: Execution context from Station Service
            hardware_config: Hardware configuration dictionary
            parameters: Test parameters dictionary
            **kwargs: Additional arguments for SequenceBase
        """
        super().__init__(
            context=context,
            hardware_config=hardware_config,
            parameters=parameters,
            **kwargs,
        )

        # MCU driver instance (initialized in setup)
        self.mcu: Optional[Any] = None

        # Load parameters with defaults
        self.port: str = self.get_parameter("port", "/dev/ttyUSB0")
        self.baudrate: int = self.get_parameter("baudrate", 115200)
        self.timeout: float = self.get_parameter("timeout", 5.0)

        # VL53L0X parameters
        self.vl53l0x_target_mm: int = self.get_parameter("vl53l0x_target_mm", 500)
        self.vl53l0x_tolerance_mm: int = self.get_parameter("vl53l0x_tolerance_mm", 100)

        # MLX90640 parameters
        self.mlx90640_target_celsius: float = self.get_parameter("mlx90640_target_celsius", 25.0)
        self.mlx90640_tolerance_celsius: float = self.get_parameter("mlx90640_tolerance_celsius", 10.0)

        # Sensor enable flags
        self.test_vl53l0x_enabled: bool = self.get_parameter("test_vl53l0x_enabled", True)
        self.test_mlx90640_enabled: bool = self.get_parameter("test_mlx90640_enabled", True)

        # Stop on first failure (default: True for manufacturing tests)
        self.stop_on_failure: bool = self.get_parameter("stop_on_failure", True)

        logger.debug(f"Initialized {self.name} v{self.version}")

    # =========================================================================
    # Lifecycle Methods (Required by SequenceBase)
    # =========================================================================

    async def setup(self) -> None:
        """
        Initialize hardware and verify connection.

        Connects to PSA MCU and validates firmware version.

        Raises:
            SetupError: If hardware connection fails
        """
        self.emit_log("info", "하드웨어 초기화 시작...")

        # Check if running in simulation mode (dry_run)
        if self.context.dry_run:
            self.emit_log("info", "시뮬레이션 모드 - 실제 하드웨어 연결 건너뜀")
            # Use mock hardware from context
            self.mcu = self.context.hardware.get("psa_mcu")
            if self.mcu:
                await self.mcu.connect()
            return

        # Real hardware connection
        try:
            hw_config = self.get_hardware_config("psa_mcu")
            port = hw_config.get("port", self.port)
            baudrate = hw_config.get("baudrate", self.baudrate)
            timeout = hw_config.get("timeout", self.timeout)

            self.emit_log("info", f"MCU 연결 중: {port} @ {baudrate} bps")

            driver_class = _get_driver_class()
            self.mcu = driver_class(config={
                "port": port,
                "baudrate": baudrate,
                "timeout": timeout,
            })

            connected = await self.mcu.connect()
            if not connected:
                raise SetupError("MCU 연결 실패", details={"error_code": "MCU_CONNECTION_FAILED"})

            # Get device info
            idn = await self.mcu.identify()
            self.emit_log("info", f"MCU 연결 완료: {idn}")

        except SetupError:
            raise
        except Exception as e:
            raise SetupError(f"하드웨어 초기화 실패: {e}", details={"original_error": str(e)})

    async def run(self) -> RunResult:
        """
        Execute the main test sequence.

        Runs all enabled sensor tests and collects measurements.

        Returns:
            RunResult with passed status and measurements
        """
        # Calculate total steps
        total_steps = 2  # ping_pong and initialize are always run
        if self.test_vl53l0x_enabled:
            total_steps += 1
        if self.test_mlx90640_enabled:
            total_steps += 1
        total_steps += 1  # finalize is always run

        current_step = 0
        all_passed = True
        measurements: Dict[str, Any] = {}

        # =====================================================================
        # Step 1: PING/PONG Communication Test
        # =====================================================================
        current_step += 1
        self.emit_step_start("ping_pong", current_step, total_steps, "PING/PONG 통신 테스트")
        start_time = time.time()

        try:
            self.check_abort()

            if self.mcu:
                # Send PING and receive PONG with firmware version
                fw_version = await self.mcu.ping()
                self.emit_log("info", f"PING/PONG 성공 - 펌웨어 버전: {fw_version}")

                measurements["firmware_version"] = fw_version
                measurements["ping_pong_passed"] = True

            duration = time.time() - start_time
            self.emit_step_complete("ping_pong", current_step, True, duration)

        except Exception as e:
            duration = time.time() - start_time
            self.emit_step_complete("ping_pong", current_step, False, duration, error=str(e))
            self.emit_error("PING_PONG_ERROR", str(e))
            measurements["ping_pong_passed"] = False
            all_passed = False
            if self.stop_on_failure:
                return {"passed": False, "measurements": measurements, "data": {"stopped_at": "ping_pong"}}

        # =====================================================================
        # Step 2: Initialize (Get Sensor List)
        # =====================================================================
        current_step += 1
        self.emit_step_start("initialize", current_step, total_steps, "센서 목록 조회")
        start_time = time.time()

        try:
            self.check_abort()

            if self.mcu:
                # Get sensor list
                sensors = await self.mcu.get_sensor_list()
                sensor_names = [s.get("name", "Unknown") for s in sensors]
                self.emit_log("info", f"감지된 센서: {sensor_names}")

                measurements["sensor_count"] = len(sensors)
                measurements["sensors"] = sensor_names

            duration = time.time() - start_time
            self.emit_step_complete("initialize", current_step, True, duration)

        except Exception as e:
            duration = time.time() - start_time
            self.emit_step_complete("initialize", current_step, False, duration, error=str(e))
            self.emit_error("INIT_ERROR", str(e))
            all_passed = False
            if self.stop_on_failure:
                return {"passed": False, "measurements": measurements, "data": {"stopped_at": "initialize"}}

        # =====================================================================
        # Step 3: VL53L0X Distance Test (if enabled)
        # =====================================================================
        if self.test_vl53l0x_enabled:
            current_step += 1
            self.emit_step_start("test_vl53l0x", current_step, total_steps, "VL53L0X 거리 테스트")
            start_time = time.time()

            try:
                self.check_abort()

                if self.mcu:
                    result = await self.mcu.test_vl53l0x(
                        target_mm=self.vl53l0x_target_mm,
                        tolerance_mm=self.vl53l0x_tolerance_mm,
                    )

                    measured_mm = result.get("measured_mm", 0)
                    passed = result.get("passed", False)

                    # Emit measurement with limits
                    self.emit_measurement(
                        name="vl53l0x_distance",
                        value=measured_mm,
                        unit="mm",
                        passed=passed,
                        min_value=self.vl53l0x_target_mm - self.vl53l0x_tolerance_mm,
                        max_value=self.vl53l0x_target_mm + self.vl53l0x_tolerance_mm,
                    )

                    measurements["vl53l0x_distance_mm"] = measured_mm
                    measurements["vl53l0x_passed"] = passed

                    if not passed:
                        all_passed = False
                        self.emit_log("warning", f"VL53L0X 테스트 실패: {result.get('status_name')}")

                duration = time.time() - start_time
                step_passed = result.get("passed", True) if self.mcu else True
                self.emit_step_complete(
                    "test_vl53l0x",
                    current_step,
                    step_passed,
                    duration,
                    measurements={"vl53l0x_distance": measured_mm} if self.mcu else None,
                )

                # Stop on measurement failure if configured
                if not step_passed and self.stop_on_failure:
                    return {"passed": False, "measurements": measurements, "data": {"stopped_at": "test_vl53l0x"}}

            except Exception as e:
                duration = time.time() - start_time
                self.emit_step_complete("test_vl53l0x", current_step, False, duration, error=str(e))
                self.emit_error("VL53L0X_ERROR", str(e))
                all_passed = False
                if self.stop_on_failure:
                    return {"passed": False, "measurements": measurements, "data": {"stopped_at": "test_vl53l0x"}}

        # =====================================================================
        # Step 4: MLX90640 Temperature Test (if enabled)
        # =====================================================================
        if self.test_mlx90640_enabled:
            current_step += 1
            self.emit_step_start("test_mlx90640", current_step, total_steps, "MLX90640 온도 테스트")
            start_time = time.time()

            try:
                self.check_abort()

                if self.mcu:
                    result = await self.mcu.test_mlx90640(
                        target_celsius=self.mlx90640_target_celsius,
                        tolerance_celsius=self.mlx90640_tolerance_celsius,
                    )

                    measured_celsius = result.get("measured_celsius", 0)
                    passed = result.get("passed", False)

                    # Emit measurement with limits
                    self.emit_measurement(
                        name="mlx90640_temperature",
                        value=measured_celsius,
                        unit="°C",
                        passed=passed,
                        min_value=self.mlx90640_target_celsius - self.mlx90640_tolerance_celsius,
                        max_value=self.mlx90640_target_celsius + self.mlx90640_tolerance_celsius,
                    )

                    measurements["mlx90640_temperature_c"] = measured_celsius
                    measurements["mlx90640_passed"] = passed

                    if not passed:
                        all_passed = False
                        self.emit_log("warning", f"MLX90640 테스트 실패: {result.get('status_name')}")

                duration = time.time() - start_time
                step_passed = result.get("passed", True) if self.mcu else True
                self.emit_step_complete(
                    "test_mlx90640",
                    current_step,
                    step_passed,
                    duration,
                    measurements={"mlx90640_temperature": measured_celsius} if self.mcu else None,
                )

                # Stop on measurement failure if configured
                if not step_passed and self.stop_on_failure:
                    return {"passed": False, "measurements": measurements, "data": {"stopped_at": "test_mlx90640"}}

            except Exception as e:
                duration = time.time() - start_time
                self.emit_step_complete("test_mlx90640", current_step, False, duration, error=str(e))
                self.emit_error("MLX90640_ERROR", str(e))
                all_passed = False
                if self.stop_on_failure:
                    return {"passed": False, "measurements": measurements, "data": {"stopped_at": "test_mlx90640"}}

        # =====================================================================
        # Step 5: Finalize
        # =====================================================================
        current_step += 1
        self.emit_step_start("finalize", current_step, total_steps, "결과 정리")
        start_time = time.time()

        try:
            # Summary
            self.emit_log(
                "info",
                f"테스트 완료 - 전체 결과: {'PASS' if all_passed else 'FAIL'}"
            )

            duration = time.time() - start_time
            self.emit_step_complete("finalize", current_step, True, duration)

        except Exception as e:
            duration = time.time() - start_time
            self.emit_step_complete("finalize", current_step, False, duration, error=str(e))

        return {
            "passed": all_passed,
            "measurements": measurements,
            "data": {
                "vl53l0x_enabled": self.test_vl53l0x_enabled,
                "mlx90640_enabled": self.test_mlx90640_enabled,
            },
        }

    async def teardown(self) -> None:
        """
        Clean up resources and disconnect hardware.

        Always called, even if setup or run failed.
        """
        self.emit_log("info", "리소스 정리 중...")

        try:
            if self.mcu:
                if hasattr(self.mcu, "is_connected"):
                    if await self.mcu.is_connected():
                        await self.mcu.disconnect()
                        self.emit_log("info", "MCU 연결 해제 완료")
                else:
                    # For mock hardware
                    await self.mcu.disconnect()

        except Exception as e:
            self.emit_log("warning", f"정리 중 오류 (무시됨): {e}")

        self.mcu = None
        self.emit_log("info", "리소스 정리 완료")
