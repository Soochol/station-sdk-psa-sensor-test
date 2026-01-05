"""
Sensor data structures.

Reference: include/sensors/sensor_types.h
All multi-byte values use Big-endian byte order.
"""

from dataclasses import dataclass
from typing import List, Optional, Union
import struct

from .constants import SensorID, TestStatus


@dataclass
class MLX90640Spec:
    """MLX90640 test specification."""
    target_temp: int   # Target temperature x100 (degrees C), int16
    tolerance: int     # Tolerance x100 (degrees C), uint16

    def to_bytes(self) -> bytes:
        """Serialize to big-endian bytes."""
        return struct.pack('>hH', self.target_temp, self.tolerance)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'MLX90640Spec':
        """Deserialize from big-endian bytes."""
        target, tolerance = struct.unpack('>hH', data[:4])
        return cls(target, tolerance)

    @property
    def target_celsius(self) -> float:
        """Target temperature in Celsius."""
        return self.target_temp / 100.0

    @property
    def tolerance_celsius(self) -> float:
        """Tolerance in Celsius."""
        return self.tolerance / 100.0

    def __repr__(self) -> str:
        return f"MLX90640Spec(target={self.target_celsius:.2f}C, tolerance=+/-{self.tolerance_celsius:.2f}C)"


@dataclass
class MLX90640Result:
    """MLX90640 test result."""
    max_temp: int      # Measured max temperature x100, int16
    target: int        # Target temperature x100, int16
    tolerance: int     # Tolerance x100, uint16
    diff: int          # Absolute difference x100, uint16

    @classmethod
    def from_bytes(cls, data: bytes) -> 'MLX90640Result':
        """Deserialize from big-endian bytes."""
        max_temp, target, tolerance, diff = struct.unpack('>hhHH', data[:8])
        return cls(max_temp, target, tolerance, diff)

    @property
    def max_temp_celsius(self) -> float:
        """Measured max temperature in Celsius."""
        return self.max_temp / 100.0

    @property
    def target_celsius(self) -> float:
        """Target temperature in Celsius."""
        return self.target / 100.0

    @property
    def tolerance_celsius(self) -> float:
        """Tolerance in Celsius."""
        return self.tolerance / 100.0

    @property
    def diff_celsius(self) -> float:
        """Difference in Celsius."""
        return self.diff / 100.0

    @property
    def passed(self) -> bool:
        """Check if measurement is within tolerance."""
        return self.diff <= self.tolerance

    def __repr__(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        return (f"MLX90640Result(max={self.max_temp_celsius:.2f}C, "
                f"target={self.target_celsius:.2f}C, "
                f"diff={self.diff_celsius:.2f}C, {status})")


@dataclass
class VL53L0XSpec:
    """VL53L0X test specification."""
    target_dist: int   # Target distance in mm, uint16
    tolerance: int     # Tolerance in mm, uint16

    def to_bytes(self) -> bytes:
        """Serialize to big-endian bytes."""
        return struct.pack('>HH', self.target_dist, self.tolerance)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'VL53L0XSpec':
        """Deserialize from big-endian bytes."""
        target, tolerance = struct.unpack('>HH', data[:4])
        return cls(target, tolerance)

    def __repr__(self) -> str:
        return f"VL53L0XSpec(target={self.target_dist}mm, tolerance=+/-{self.tolerance}mm)"


@dataclass
class VL53L0XResult:
    """VL53L0X test result."""
    measured: int      # Measured distance in mm, uint16
    target: int        # Target distance in mm, uint16
    tolerance: int     # Tolerance in mm, uint16
    diff: int          # Absolute difference in mm, uint16

    @classmethod
    def from_bytes(cls, data: bytes) -> 'VL53L0XResult':
        """Deserialize from big-endian bytes."""
        measured, target, tolerance, diff = struct.unpack('>HHHH', data[:8])
        return cls(measured, target, tolerance, diff)

    @property
    def passed(self) -> bool:
        """Check if measurement is within tolerance."""
        return self.diff <= self.tolerance

    def __repr__(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        return (f"VL53L0XResult(measured={self.measured}mm, "
                f"target={self.target}mm, diff={self.diff}mm, {status})")


@dataclass
class SensorInfo:
    """Sensor information from GET_SENSOR_LIST."""
    sensor_id: int
    name: str

    def __repr__(self) -> str:
        return f"SensorInfo(id=0x{self.sensor_id:02X}, name='{self.name}')"


@dataclass
class SensorTestResult:
    """Individual sensor test result."""
    sensor_id: int
    status: int
    result: Optional[Union[MLX90640Result, VL53L0XResult]]

    @property
    def status_name(self) -> str:
        """Get status name."""
        return TestStatus.name_of(self.status)

    @property
    def sensor_name(self) -> str:
        """Get sensor name."""
        return SensorID.name_of(self.sensor_id)

    @property
    def passed(self) -> bool:
        """Check if test passed."""
        return self.status == TestStatus.PASS

    def __repr__(self) -> str:
        return (f"SensorTestResult(sensor={self.sensor_name}, "
                f"status={self.status_name}, result={self.result})")


@dataclass
class TestReport:
    """Complete test report."""
    sensor_count: int
    pass_count: int
    fail_count: int
    timestamp: int
    results: List[SensorTestResult]

    @classmethod
    def from_bytes(cls, data: bytes) -> 'TestReport':
        """
        Deserialize from protocol bytes.

        Format:
        - sensor_count: uint8
        - pass_count: uint8
        - fail_count: uint8
        - timestamp: uint32 (big-endian)
        - For each sensor:
          - sensor_id: uint8
          - status: uint8
          - result_data: 8 bytes
        """
        idx = 0
        sensor_count = data[idx]; idx += 1
        pass_count = data[idx]; idx += 1
        fail_count = data[idx]; idx += 1
        timestamp = struct.unpack('>I', data[idx:idx+4])[0]; idx += 4

        results = []
        for _ in range(sensor_count):
            sensor_id = data[idx]; idx += 1
            status = data[idx]; idx += 1

            # Parse sensor-specific result (8 bytes)
            result_data = data[idx:idx+8]; idx += 8

            result: Optional[Union[MLX90640Result, VL53L0XResult]] = None
            if status in (TestStatus.PASS, TestStatus.FAIL_INVALID):
                if sensor_id == SensorID.MLX90640:
                    result = MLX90640Result.from_bytes(result_data)
                elif sensor_id == SensorID.VL53L0X:
                    result = VL53L0XResult.from_bytes(result_data)

            results.append(SensorTestResult(sensor_id, status, result))

        return cls(sensor_count, pass_count, fail_count, timestamp, results)

    @property
    def all_passed(self) -> bool:
        """Check if all tests passed."""
        return self.fail_count == 0 and self.pass_count == self.sensor_count

    def __repr__(self) -> str:
        return (f"TestReport(sensors={self.sensor_count}, "
                f"pass={self.pass_count}, fail={self.fail_count}, "
                f"timestamp={self.timestamp}ms)")
