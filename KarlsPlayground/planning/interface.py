"""Planner interface for parking-motion-planner challenges."""

from __future__ import annotations

from typing import Any, Protocol

from KarlsPlayground.simulation.commands import MotionCommand

from .path_planner import PathWaypoint


class ParkingPlanner(Protocol):
    """Interface expected by the challenge runner."""

    def plan(
        self,
        start: tuple[float, float],
        start_heading: float,
        goal: tuple[float, float, float],
        static_obstacles: list[dict[str, Any]],
        start_direction: int | None = None,
    ) -> list[PathWaypoint]:
        """Prepare a parking maneuver and return the planned path for visualization."""

    def compute_motion_command(
        self,
        state: dict[str, Any],
        goal_heading: float,
        dynamic_obstacles: list[dict[str, Any]] | None = None,
    ) -> MotionCommand:
        """Return longitudinal acceleration and front steering angle."""
