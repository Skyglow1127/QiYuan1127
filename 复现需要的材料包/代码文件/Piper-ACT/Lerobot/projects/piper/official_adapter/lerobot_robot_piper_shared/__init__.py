"""Local hardware plugin. LeRobot owns the recording loop and dataset writer."""
from .devices import (
    PiperHttpCamera, PiperHttpCameraConfig,
    PiperSharedObserver, PiperSharedObserverConfig,
    PiperSharedLeader, PiperSharedLeaderConfig,
)

__all__ = ['PiperSharedObserver', 'PiperSharedObserverConfig',
           'PiperSharedLeader', 'PiperSharedLeaderConfig',
           'PiperHttpCamera', 'PiperHttpCameraConfig']
