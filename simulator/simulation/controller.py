"""Simulator-side command clamping before vehicle execution."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simulator.simulation.vehicle import Vehicle

from .commands import MotionCommand, SimulatorActuatorCommand


@dataclass
class SimulatorController:
    """Convert planner output into simulator-safe actuator commands."""

    max_longitudinal_acceleration_mps2: float
    min_longitudinal_acceleration_mps2: float
    max_front_mid_axle_steering_angle_rad: float

    @classmethod
    def from_vehicle(cls, vehicle: Vehicle) -> "SimulatorController":
        return cls(
            max_longitudinal_acceleration_mps2=float(vehicle.MAX_ACCELERATION),
            min_longitudinal_acceleration_mps2=float(vehicle.MAX_DECELERATION),
            max_front_mid_axle_steering_angle_rad=float(vehicle.MAX_STEERING_ANGLE),
        )

    def apply_limits(self, command: MotionCommand) -> SimulatorActuatorCommand:
        """Clamp and normalize a planner command for the simulator interface."""
        acceleration = float(np.clip(
            command.longitudinal_acceleration_mps2,
            self.min_longitudinal_acceleration_mps2,
            self.max_longitudinal_acceleration_mps2,
        ))
        steering = float(np.clip(
            command.front_mid_axle_steering_angle_rad,
            -self.max_front_mid_axle_steering_angle_rad,
            self.max_front_mid_axle_steering_angle_rad,
        ))
        return SimulatorActuatorCommand(
            longitudinal_acceleration_mps2=acceleration,
            front_mid_axle_steering_angle_rad=steering,
        )
