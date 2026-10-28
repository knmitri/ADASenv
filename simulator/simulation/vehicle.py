import numpy as np
from abc import ABC, abstractmethod

from .kinematics import VehicleGeometry, kinematic_bicycle_step

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
    

# AudiA5 implementation
class AudiA5(Vehicle):
    """Audi A5 vehicle model with realistic dimensions and kinematic bicycle model."""

    def __init__(self):
        """Initialize AudiA5 with configuration parameters.
        Dimensions from Audi A5 Sportback (2024):
        - Length: 4.67 m
        - Width: 1.86 m
        - Wheelbase: 2.82 m
        - Front overhang: 0.89 m
        - Rear overhang: 0.96 m
        Source: audi.com, car.info, car specifications databases
        """

        self._LENGTH = 4.67      # Front bumper to rear bumper (meters)
        self._WIDTH = 1.86       # Side mirror to side mirror (meters)
        self._WHEELBASE = 2.82   # Front axle to rear axle distance (meters)
        self._FRONT_OVERHANG = 0.89  # Distance from front axle to front bumper (meters)
        self._REAR_OVERHANG = 0.96   # Distance from rear axle to rear bumper (meters)
        self.MAX_SPEED = 240.0 / 3.6  # Convert km/h to m/s
        self.MAX_REVERSE_SPEED = 2.0
        self.MAX_STEERING_ANGLE = np.deg2rad(45); self.max_steer = self.MAX_STEERING_ANGLE
        self.MAX_STEERING_RATE = np.deg2rad(45)
        self.MAX_ACCELERATION = 9
        self.MAX_DECELERATION = -9
    
    @property
    def length(self) -> float:
        """Vehicle length in meters."""
        return self._LENGTH
    
    @property
    def width(self) -> float:
        """Vehicle width in meters."""
        return self._WIDTH
    
    @property
    def wheelbase(self) -> float:
        """Vehicle wheelbase in meters."""
        return self._WHEELBASE
    
    def get_corners(self) -> np.ndarray:
        """
        Get the four corners of the vehicle footprint in global frame.
        The reference point (self.x, self.y) is at the rear axle.
        Uses fixed front and rear overhangs for accurate geometry.
        Returns:
            Array of shape (4, 2) with corner positions [x, y]
        """
        return VehicleGeometry.from_vehicle(self).corners(self.x, self.y, self.yaw)
    
    def update(self, acceleration: float, steering_angle: float, dt: float) -> None:
        """Update vehicle state using kinematic bicycle model.
        
        Args:
            acceleration: Acceleration in m/s² (positive forward, negative reverse)
            steering_angle: Steering angle at front wheels (radians)
            dt: Time step (seconds)
        """
        # Clamp inputs to vehicle limits
        acceleration = np.clip(acceleration, self.MAX_DECELERATION, self.MAX_ACCELERATION)
        steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        # Steering can be commanded even at standstill (actuator state update).
        steer_err = steering_angle - self.steering_angle
        max_delta = self.MAX_STEERING_RATE * dt
        self.steering_angle += float(np.clip(steer_err, -max_delta, max_delta))
        self.steering_angle = float(np.clip(self.steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE))
        
        # Update velocity (allow negative for reversing)
        self.vx += acceleration * dt
        self.vx = np.clip(self.vx, -self.MAX_REVERSE_SPEED, self.MAX_SPEED)
        
        next_state = kinematic_bicycle_step(
            self.x,
            self.y,
            self.yaw,
            self.vx,
            self.steering_angle,
            dt,
            self._WHEELBASE,
        )
        self.x = next_state.x
        self.y = next_state.y
        self.yaw = next_state.yaw
