"""Shared kinematic bicycle and footprint geometry helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .utils import normalize_angle


@dataclass(frozen=True)
class VehicleGeometry:
    """Vehicle geometry using the rear axle as the pose reference point."""

    length: float
    width: float
    wheelbase: float
    front_overhang: float
    rear_overhang: float

    @classmethod
    def from_vehicle(cls, vehicle: Any) -> "VehicleGeometry":
        front_overhang = float(getattr(vehicle, "_FRONT_OVERHANG", max(0.0, vehicle.length - vehicle.wheelbase) * 0.5))
        rear_overhang = float(getattr(vehicle, "_REAR_OVERHANG", max(0.0, vehicle.length - vehicle.wheelbase - front_overhang)))
        return cls(
            length=float(vehicle.length),
            width=float(vehicle.width),
            wheelbase=float(vehicle.wheelbase),
            front_overhang=front_overhang,
            rear_overhang=rear_overhang,
        )

    def corners(self, x: float, y: float, yaw: float) -> np.ndarray:
        local = np.array(
            [
                [-self.rear_overhang, -self.width / 2.0],
                [-self.rear_overhang, self.width / 2.0],
                [self.wheelbase + self.front_overhang, self.width / 2.0],
                [self.wheelbase + self.front_overhang, -self.width / 2.0],
            ],
            dtype=float,
        )
        cos_y = float(np.cos(yaw))
        sin_y = float(np.sin(yaw))
        rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]], dtype=float)
        return local @ rot.T + np.array([x, y], dtype=float)


@dataclass(frozen=True)
class BicycleState:
    x: float
    y: float
    yaw: float
    velocity: float


def kinematic_bicycle_step(
    x: float,
    y: float,
    yaw: float,
    velocity: float,
    steering_angle: float,
    dt: float,
    wheelbase: float,
) -> BicycleState:
    """Integrate a rear-axle kinematic bicycle with signed longitudinal velocity."""

    nx = float(x + velocity * dt * np.cos(yaw))
    ny = float(y + velocity * dt * np.sin(yaw))
    nyaw = float(yaw)
    if abs(velocity) > 1e-6:
        nyaw = normalize_angle(yaw + velocity * dt * np.tan(steering_angle) / max(wheelbase, 1e-3))
    return BicycleState(nx, ny, nyaw, float(velocity))


def kinematic_bicycle_arc_step(
    x: float,
    y: float,
    yaw: float,
    steering_angle: float,
    signed_distance: float,
    wheelbase: float,
) -> tuple[float, float, float]:
    """Integrate by signed arc distance instead of velocity and time."""

    nx = float(x + signed_distance * np.cos(yaw))
    ny = float(y + signed_distance * np.sin(yaw))
    nyaw = normalize_angle(yaw + signed_distance * np.tan(steering_angle) / max(wheelbase, 1e-3))
    return nx, ny, nyaw


def polygon_intersects_aabb(polygon: np.ndarray, center: tuple[float, float], size: tuple[float, float]) -> bool:
    """Return whether a convex polygon intersects an axis-aligned rectangle."""

    cx, cy = center
    width, height = size
    rect = np.array(
        [
            [cx - width / 2.0, cy - height / 2.0],
            [cx - width / 2.0, cy + height / 2.0],
            [cx + width / 2.0, cy + height / 2.0],
            [cx + width / 2.0, cy - height / 2.0],
        ],
        dtype=float,
    )

    axes: list[np.ndarray] = [np.array([1.0, 0.0]), np.array([0.0, 1.0])]
    for points in (polygon, rect):
        for i in range(len(points)):
            edge = points[(i + 1) % len(points)] - points[i]
            axis = np.array([-edge[1], edge[0]], dtype=float)
            norm = float(np.linalg.norm(axis))
            if norm > 1e-9:
                axes.append(axis / norm)

    for axis in axes:
        p_proj = polygon @ axis
        r_proj = rect @ axis
        if float(np.max(p_proj)) < float(np.min(r_proj)) or float(np.max(r_proj)) < float(np.min(p_proj)):
            return False
    return True
