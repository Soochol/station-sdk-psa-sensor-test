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
    """
    MLX90640 test specification.

    Temperature values are in 0.1°C units (x10).
    """
    target_temp: int   # Target temperature x10 (0.1°C units), int16
    tolerance: int     # Tolerance x10 (0.1°C units), int16

    def to_bytes(self) -> bytes:
        """Serialize to big-endian bytes."""
        return struct.pack('>hh', self.target_temp, self.tolerance)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'MLX90640Spec':
        """Deserialize from big-endian bytes."""
        target, tolerance = struct.unpack('>hh', data[:4])
        return cls(target, tolerance)

    @property
    def target_celsius(self) -> float:
        """Target temperature in Celsius."""
        return self.target_temp / 10.0

    @property
    def tolerance_celsius(self) -> float:
        """Tolerance in Celsius."""
        return self.tolerance / 10.0

    def __repr__(self) -> str:
        return f"MLX90640Spec(target={self.target_celsius:.1f}C, tolerance=+/-{self.tolerance_celsius:.1f}C)"


@dataclass
class MLX90640Result:
    """
    MLX90640 test result.

    All temperature values are in 0.1°C units (x10).
    Total size: 14 bytes (7 x int16).
    """
    measured: int      # Measured temperature x10, int16
    target: int        # Target temperature x10, int16
    tolerance: int     # Tolerance x10, int16
    diff: int          # Absolute difference x10, int16
    ambient: int       # Ambient temperature x10, int16
    min_temp: int      # Min pixel temperature x10, int16
    max_temp: int      # Max pixel temperature x10, int16

    @classmethod
    def from_bytes(cls, data: bytes) -> 'MLX90640Result':
        """Deserialize from big-endian bytes (14 bytes)."""
        measured, target, tolerance, diff, ambient, min_temp, max_temp = struct.unpack('>hhhhhhh', data[:14])
        return cls(measured, target, tolerance, diff, ambient, min_temp, max_temp)

    @property
    def measured_celsius(self) -> float:
        """Measured temperature in Celsius."""
        return self.measured / 10.0

    @property
    def target_celsius(self) -> float:
        """Target temperature in Celsius."""
        return self.target / 10.0

    @property
    def tolerance_celsius(self) -> float:
        """Tolerance in Celsius."""
        return self.tolerance / 10.0

    @property
    def diff_celsius(self) -> float:
        """Difference in Celsius."""
        return self.diff / 10.0

    @property
    def ambient_celsius(self) -> float:
        """Ambient temperature in Celsius."""
        return self.ambient / 10.0

    @property
    def min_temp_celsius(self) -> float:
        """Min pixel temperature in Celsius."""
        return self.min_temp / 10.0

    @property
    def max_temp_celsius(self) -> float:
        """Max pixel temperature in Celsius."""
        return self.max_temp / 10.0

    @property
    def passed(self) -> bool:
        """Check if measurement is within tolerance."""
        return abs(self.diff) <= abs(self.tolerance)

    def __repr__(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        return (f"MLX90640Result(measured={self.measured_celsius:.1f}C, "
                f"target={self.target_celsius:.1f}C, "
                f"diff={self.diff_celsius:.1f}C, "
                f"ambient={self.ambient_celsius:.1f}C, "
                f"min={self.min_temp_celsius:.1f}C, "
                f"max={self.max_temp_celsius:.1f}C, {status})")


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
        data_len = len(data)
        for _ in range(sensor_count):
            sensor_id = data[idx]; idx += 1
            status = data[idx]; idx += 1

            # Parse sensor-specific result (size depends on sensor type)
            # MCU always serializes result data regardless of status
            result: Optional[Union[MLX90640Result, VL53L0XResult]] = None
            if sensor_id == SensorID.MLX90640:
                result_size = 14  # MLX90640: 14 bytes
                remaining = data_len - idx
                if remaining >= result_size:
                    result_data = data[idx:idx+result_size]
                    idx += result_size
                    # Only parse result for statuses that have valid data
                    if status in (TestStatus.PASS, TestStatus.FAIL_INVALID):
                        result = MLX90640Result.from_bytes(result_data)
                else:
                    # Log warning: MCU didn't send expected result data
                    import logging
                    logging.getLogger(__name__).warning(
                        f"MLX90640: expected {result_size} bytes, got {remaining}. "
                        f"Data: {data.hex()}, idx={idx}, status={status}"
                    )
            elif sensor_id == SensorID.VL53L0X:
                result_size = 8   # VL53L0X: 8 bytes
                remaining = len(data) - idx
                if remaining >= result_size:
                    result_data = data[idx:idx+result_size]
                    idx += result_size
                    if status in (TestStatus.PASS, TestStatus.FAIL_INVALID):
                        result = VL53L0XResult.from_bytes(result_data)
            else:
                # Unknown sensor, skip 8 bytes as default if available
                remaining = len(data) - idx
                if remaining >= 8:
                    idx += 8

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
