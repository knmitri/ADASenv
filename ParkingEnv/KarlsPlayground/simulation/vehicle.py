"""Vehicle physics model - Base class for all vehicles."""

import numpy as np
from abc import ABC, abstractmethod


class Vehicle(ABC):
    """Abstract base class for vehicle models."""
    
    def __init__(self):
        """Initialize vehicle state."""
        self.x = 0.0           # Position X (meters)
        self.y = 0.0           # Position Y (meters)
        self.yaw = 0.0         # Heading angle (radians)
        self.vx = 0.0          # Forward velocity (m/s)
        self.steering_angle = 0.0  # Front wheel steering angle (radians)
    
    @property
    @abstractmethod
    def length(self) -> float:
        """Vehicle length in meters."""
        pass
    
    @property
    @abstractmethod
    def width(self) -> float:
        """Vehicle width in meters."""
        pass
    
    @property
    @abstractmethod
    def wheelbase(self) -> float:
        """Distance from front axle to rear axle in meters."""
        pass
    
    @abstractmethod
    def get_corners(self) -> np.ndarray:
        """Get the four corners of the vehicle footprint in global frame.
        
        Returns:
            Array of shape (4, 2) with corner positions [x, y]
        """
        pass
    
    @abstractmethod
    def update(self, acceleration: float, steering_angle: float, dt: float) -> None:
        """Update vehicle state using kinematic model.
        
        Args:
            acceleration: Acceleration command (m/s²)
            steering_angle: Steering angle at wheels (radians)
            dt: Time step (seconds)
        """
        pass
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi
