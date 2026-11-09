"""Path metric extraction and trial aggregation for parking benchmarks."""

from __future__ import annotations

from collections import defaultdict
import math
from statistics import fmean, median, pstdev
from typing import Any, Iterable

import numpy as np

from simulator.simulation.kinematics import VehicleGeometry, polygon_contains_polygon
from simulator.simulation.utils import normalize_angle


def path_points_from_response(response: dict[str, Any]) -> list[dict[str, Any]]:
    raw_path = response.get("planned_path", response.get("path", []))
    points: list[dict[str, Any]] = []
    for raw_point in raw_path:
        if isinstance(raw_point, dict):
            point = {"x": float(raw_point["x"]), "y": float(raw_point["y"])}
            for key in ("heading", "direction", "curvature", "target_speed", "gear_change"):
                if key in raw_point:
                    point[key] = raw_point[key]
        else:
            point = {"x": float(raw_point[0]), "y": float(raw_point[1])}
            if len(raw_point) > 2:
                point["heading"] = float(raw_point[2])
            if len(raw_point) > 3:
                point["direction"] = int(raw_point[3])
        points.append(point)
    return points


def measure_path(path: list[dict[str, Any]], sim: Any) -> dict[str, Any]:
    """Measure path coordinates and validate poses with simulator geometry."""
    waypoint_count = len(path)
    path_length = None
    direction_changes = None
    path_collision = None
    path_valid = False

    coordinates: list[tuple[float, float]] = []
    headings: list[float] = []
    directions: list[int] = []
    headings_available = True
    directions_available = True
    try:
        for point in path:
            x, y = float(point["x"]), float(point["y"])
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("non-finite path coordinate")
            coordinates.append((x, y))
            if "heading" not in point:
                headings_available = False
            else:
                heading = float(point["heading"])
                if not math.isfinite(heading):
                    raise ValueError("non-finite path heading")
                headings.append(heading)
            if "direction" in point:
                directions.append(int(point["direction"]))
            else:
                directions_available = False
    except (KeyError, TypeError, ValueError, OverflowError):
        coordinates = []
        headings = []
        directions = []

    if coordinates:
        path_length = sum(
            math.hypot(x1 - x0, y1 - y0)
            for (x0, y0), (x1, y1) in zip(coordinates, coordinates[1:])
        )
    if directions_available and len(directions) == waypoint_count and waypoint_count:
        direction_changes = sum(a != b for a, b in zip(directions, directions[1:]))

    if headings_available and len(headings) == waypoint_count and waypoint_count:
        geometry = VehicleGeometry.from_vehicle(sim.vehicle)
        path_collision = False
        for (x, y), heading in zip(coordinates, headings):
            if sim.check_collision(geometry.corners(x, y, heading)):
                path_collision = True
                break

        _, _, goal_heading = sim.get_goal_pose()
        final_corners = geometry.corners(*coordinates[-1], headings[-1])
        reaches_goal = (
            polygon_contains_polygon(sim.goal_slot.get_corners(), final_corners)
            and abs(normalize_angle(headings[-1] - goal_heading)) < sim.heading_tolerance
        )
        path_valid = not path_collision and reaches_goal

    return {
        "path_length_m": path_length,
        "path_waypoint_count": waypoint_count,
        "path_direction_changes": direction_changes,
        "path_collision": path_collision,
        "path_valid": path_valid,
    }


def _numeric_values(trials: list[dict[str, Any]], field: str) -> list[float]:
    values = []
    for trial in trials:
        value = trial.get(field)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return values


def _percentile(values: list[float], percentile: float) -> float | None:
    return float(np.percentile(values, percentile)) if values else None


def aggregate_trials(trials: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """Return trial-count-weighted aggregates by planner/scenario and planner."""
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for trial in trials:
        groups[(str(trial["planner_id"]), str(trial.get("scenario_id", "all")))].append(trial)

    aggregates: list[dict[str, Any]] = []
    for (planner_id, scenario_id), group in sorted(groups.items()):
        count = len(group)
        planner_only = all(trial.get("benchmark_mode") == "planner-only" for trial in group)
        valid_plans = sum(bool(trial.get("valid_plan", False)) for trial in group)
        successful = sum(bool(trial.get("success", False)) for trial in group)
        collisions = sum(
            bool(
                trial.get("path_collision")
                if planner_only else trial.get("collision_occurred", False)
            )
            for trial in group
        )
        stuck = sum(bool(trial.get("stuck", False)) for trial in group)
        timeouts = sum(bool(trial.get("timed_out", False)) for trial in group)
        failures = sum(not bool(trial.get("success", False)) for trial in group)
        latency = _numeric_values(group, "initial_planning_latency_ms")
        path_lengths = _numeric_values(group, "path_length_m")
        steps = _numeric_values(group, "simulation_steps")
        position_errors = _numeric_values(group, "final_position_error_m")
        heading_errors = _numeric_values(group, "final_heading_error_deg")
        path_waypoints = _numeric_values(group, "path_waypoint_count")
        path_direction_changes = _numeric_values(group, "path_direction_changes")
        valid_paths = sum(bool(trial.get("path_valid", False)) for trial in group)
        planner_only_latency = _numeric_values(group, "planner_only_latency_ms")
        closed_loop_latency = _numeric_values(
            group, "closed_loop_initial_planning_latency_ms"
        )
        aggregates.append({
            "planner_id": planner_id,
            "scenario_id": scenario_id,
            "trial_count": count,
            "success_count": successful,
            "success_rate": None if planner_only else (successful / count if count else None),
            "parking_success_rate": (
                None if planner_only else (successful / count if count else None)
            ),
            "valid_plan_count": valid_plans,
            "valid_plan_rate": valid_plans / count if planner_only and count else None,
            "planning_success_rate": valid_plans / count if planner_only and count else None,
            "collision_count": collisions,
            "collision_rate": collisions / count if count else None,
            "path_collision_rate": collisions / count if planner_only and count else None,
            "stuck_count": stuck,
            "stuck_rate": stuck / count if count else None,
            "timeout_count": timeouts,
            "timeout_rate": timeouts / count if count else None,
            "failure_count": count - valid_plans if planner_only else failures,
            "failure_rate": (
                (count - valid_plans) / count if planner_only and count
                else (failures / count if count else None)
            ),
            "planning_latency_count": len(latency),
            "planning_latency_median_ms": median(latency) if latency else None,
            "planning_latency_p95_ms": _percentile(latency, 95),
            "planning_latency_max_ms": max(latency) if latency else None,
            "planning_latency_mean_ms": fmean(latency) if latency else None,
            "planning_latency_stddev_ms": pstdev(latency) if latency else None,
            "planner_only_latency_count": len(planner_only_latency),
            "planner_only_latency_median_ms": (
                median(planner_only_latency) if planner_only_latency else None
            ),
            "closed_loop_initial_planning_latency_count": len(closed_loop_latency),
            "closed_loop_initial_planning_latency_median_ms": (
                median(closed_loop_latency) if closed_loop_latency else None
            ),
            "path_length_count": len(path_lengths),
            "path_length_median": median(path_lengths) if path_lengths else None,
            "path_waypoint_count_median": (
                median(path_waypoints) if path_waypoints else None
            ),
            "path_direction_changes_median": (
                median(path_direction_changes) if path_direction_changes else None
            ),
            "path_valid_rate": valid_paths / count if count else None,
            "episode_steps_median": median(steps) if steps else None,
            "final_position_error_median": median(position_errors) if position_errors else None,
            "final_heading_error_deg_median": median(heading_errors) if heading_errors else None,
            "episode_duration_median_seconds": (
                median(_numeric_values(group, "episode_duration_seconds"))
                if _numeric_values(group, "episode_duration_seconds") else None
            ),
            "wall_clock_execution_median_seconds": (
                median(_numeric_values(group, "wall_clock_execution_seconds"))
                if _numeric_values(group, "wall_clock_execution_seconds") else None
            ),
            "simulated_duration_median_seconds": (
                median(_numeric_values(group, "simulated_duration_seconds"))
                if _numeric_values(group, "simulated_duration_seconds") else None
            ),
            "simulation_speed_factor_median": (
                median(_numeric_values(group, "simulation_speed_factor"))
                if _numeric_values(group, "simulation_speed_factor") else None
            ),
        })
    return aggregates
