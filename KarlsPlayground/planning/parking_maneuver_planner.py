"""Parking-specific maneuver primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .path_planner import PathWaypoint


@dataclass(frozen=True)
class ManeuverPhase:
    """One directional phase of a parking maneuver."""

    direction: int
    target_speed: float
    stop_at_end: bool
    waypoints: tuple[PathWaypoint, ...]


@dataclass(frozen=True)
class ParkingManeuver:
    """A complete parking maneuver expressed as explicit phases."""

    phases: tuple[ManeuverPhase, ...]

    def as_path(self) -> list[PathWaypoint]:
        path: list[PathWaypoint] = []
        for phase in self.phases:
            if path and phase.waypoints:
                last = path[-1]
                first = phase.waypoints[0]
                same_pose = abs(last.x - first.x) < 1e-6 and abs(last.y - first.y) < 1e-6
                if same_pose and last.direction == first.direction:
                    path.extend(phase.waypoints[1:])
                else:
                    path.extend(phase.waypoints)
            else:
                path.extend(phase.waypoints)
        return path


class ParkingManeuverPlanner:
    """Create deterministic low-speed parking maneuvers for perpendicular slots."""

    def __init__(self, vehicle) -> None:
        self.vehicle = vehicle

    @staticmethod
    def _bezier_points(
        start: tuple[float, float],
        goal: tuple[float, float],
        start_heading: float,
        goal_heading: float,
        samples: int = 48,
    ) -> list[tuple[float, float]]:
        p0 = np.array(start, dtype=float)
        p3 = np.array(goal, dtype=float)
        chord = max(1e-6, float(np.linalg.norm(p3 - p0)))
        handle = min(max(chord * 0.45, 2.0), 7.0)
        p1 = p0 + handle * np.array([np.cos(start_heading), np.sin(start_heading)], dtype=float)
        p2 = p3 - handle * np.array([np.cos(goal_heading), np.sin(goal_heading)], dtype=float)

        points: list[tuple[float, float]] = []
        for raw_t in np.linspace(0.0, 1.0, samples):
            t = float(raw_t)
            u = 1.0 - t
            p = u**3 * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t**3 * p3
            points.append((float(p[0]), float(p[1])))
        return points

    @staticmethod
    def _annotate(points: Sequence[tuple[float, float]], direction: int, heading: float) -> tuple[PathWaypoint, ...]:
        return tuple(PathWaypoint(float(x), float(y), direction, float(heading)) for x, y in points)

    def plan_perpendicular(
        self,
        start: tuple[float, float],
        start_heading: float,
        goal: tuple[float, float, float],
        y_bounds: tuple[float, float],
    ) -> ParkingManeuver:
        """Plan a two-phase forward/reverse perpendicular parking maneuver."""

        goal_x, goal_y, goal_heading = goal
        slot_axis = np.array([np.cos(goal_heading), np.sin(goal_heading)], dtype=float)
        lateral_axis = np.array([-np.sin(goal_heading), np.cos(goal_heading)], dtype=float)

        reverse_distance = max(2.4, self.vehicle.width + 0.5)
        seam = np.array([goal_x, goal_y], dtype=float) + reverse_distance * slot_axis

        # Approach from the aisle with a little lateral lead so the vehicle is close
        # to the slot row before committing to the straight reverse phase.
        start_arr = np.array(start, dtype=float)
        lateral_delta = float(np.dot(np.array([goal_x, goal_y], dtype=float) - start_arr, lateral_axis))
        approach_mid_y = float(np.clip(start[1] + 0.65 * lateral_delta, y_bounds[0], y_bounds[1]))
        approach_mid = (float((start[0] + seam[0]) * 0.45), approach_mid_y)

        approach_points = [start]
        approach_points.extend(
            self._bezier_points(
                start=approach_mid,
                goal=(float(seam[0]), float(seam[1])),
                start_heading=start_heading,
                goal_heading=goal_heading,
                samples=38,
            )
        )
        if np.hypot(approach_points[0][0] - approach_mid[0], approach_points[0][1] - approach_mid[1]) > 0.2:
            approach_points.insert(1, approach_mid)

        reverse_points = [
            (
                float(seam[0] + t * (goal_x - seam[0])),
                float(seam[1] + t * (goal_y - seam[1])),
            )
            for t in np.linspace(0.0, 1.0, 8)
        ]

        return ParkingManeuver(
            phases=(
                ManeuverPhase(
                    direction=1,
                    target_speed=0.9,
                    stop_at_end=True,
                    waypoints=self._annotate(approach_points, 1, goal_heading),
                ),
                ManeuverPhase(
                    direction=-1,
                    target_speed=0.35,
                    stop_at_end=True,
                    waypoints=self._annotate(reverse_points, -1, goal_heading),
                ),
            )
        )
