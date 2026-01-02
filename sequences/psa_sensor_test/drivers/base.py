"""
Base Driver Module

Abstract base class for all hardware drivers.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional


class BaseDriver(ABC):
    """
    Abstract base driver class.

    All hardware drivers must inherit from this class and implement
    the abstract methods.

    Attributes:
        name: Driver identifier name
        config: Configuration dictionary
    """

    def __init__(
        self,
        name: str = "BaseDriver",
        config: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize driver.

        Args:
            name: Driver identifier name
            config: Configuration dictionary (port, baudrate, etc.)
        """
        self.name = name
        self.config = config or {}
        self._connected = False

    @abstractmethod
    async def connect(self) -> bool:
        """
        Connect to hardware.

        Returns:
            bool: True if connection successful
        """
        ...

    @abstractmethod
    async def disconnect(self) -> None:
        """
        Disconnect from hardware.

        Clean up resources and close connection.
        """
        ...

    @abstractmethod
    async def reset(self) -> None:
        """
        Reset hardware to initial state.

        Restore device to known safe state.
        """
        ...

    async def identify(self) -> str:
        """
        Return device identification string.

        Returns:
            str: Device ID string (e.g., "Manufacturer,Model,Serial,Version")
        """
        return "Unknown"

    async def is_connected(self) -> bool:
        """
        Check connection status.

        Returns:
            bool: True if connected
        """
        return self._connected

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name={self.name!r}, connected={self._connected})"
