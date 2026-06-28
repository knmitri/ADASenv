"""Typed simulator command interface."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionCommand:
    """Desired motion command produced by a planner.

    Fields are expressed in vehicle coordinates. Positive longitudinal
    acceleration drives forward; negative drives reverse/braking. Steering is
    the commanded front mid-axle steering angle in radians.
    """

    longitudinal_acceleration_mps2: float
    front_mid_axle_steering_angle_rad: float


@dataclass(frozen=True)
class SimulatorActuatorCommand:
    """Actuator command consumed by the simulator."""

    longitudinal_acceleration_mps2: float
    front_mid_axle_steering_angle_rad: float

    def as_step_args(self) -> tuple[float, float]:
        """Return `(acceleration, steering)` args for the current simulator API."""
        return (
            self.longitudinal_acceleration_mps2,
            self.front_mid_axle_steering_angle_rad,
        )
