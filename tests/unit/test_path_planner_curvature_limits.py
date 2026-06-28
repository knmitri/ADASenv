import numpy as np

from KarlsPlayground.planning import PathPlanner, PathWaypoint
from KarlsPlayground.simulation.vehicle import AudiA5


def _planner() -> PathPlanner:
    return PathPlanner(
        vehicle=AudiA5(),
        resolution=0.25,
        x_bounds=(0.0, 22.0),
        y_bounds=(0.0, 32.0),
    )


def test_curvature_gate_rejects_sharp_path():
    planner = _planner()

    # Right-angle corner over a very short distance -> impossible curvature.
    sharp_path = [
        PathWaypoint(0.0, 0.0, 1),
        PathWaypoint(0.25, 0.0, 1),
        PathWaypoint(0.25, 0.25, 1),
    ]

    assert not planner._has_achievable_curvature(sharp_path)


def test_curvature_gate_accepts_gentle_path():
    planner = _planner()

    gentle_path = [
        PathWaypoint(0.0, 0.0, 1),
        PathWaypoint(1.0, 0.0, 1),
        PathWaypoint(2.0, 0.15, 1),
        PathWaypoint(3.0, 0.45, 1),
    ]

    assert planner._has_achievable_curvature(gentle_path)


def test_planner_output_respects_achievable_curvature():
    planner = _planner()

    path = planner.plan(
        start=(10.0, 30.0),
        goal=(0.25, 11.75),
        obstacles=[],
        start_direction=1,
        start_heading=-np.pi / 2.0,
        goal_heading=0.0,
    )

    assert len(path) >= 2
    assert planner._has_achievable_curvature(path)
