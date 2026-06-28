import numpy as np

from KarlsPlayground.planning import BaselineParkingPlanner
from KarlsPlayground.planning import PathWaypoint
from KarlsPlayground.planning import TrajectoryTracker
from KarlsPlayground.simulation.vehicle import AudiA5


def test_tracker_switches_to_reverse_near_seam_and_keeps_it():
    tracker = TrajectoryTracker(lookahead_dist=1.2)
    path = [
        PathWaypoint(0.0, 0.0, 1, 0.0),
        PathWaypoint(4.0, 0.0, 1, 0.0),
        PathWaypoint(4.0, -1.2, -1, -np.pi / 2),
        PathWaypoint(4.0, -2.0, -1, -np.pi / 2),
    ]
    tracker.set_path(path)

    # Before the seam: tracker should keep forward instead of switching early.
    _, _, d1 = tracker._find_goal_point((3.9, -0.15), current_direction=1, lookahead=1.0)
    assert d1 == 1
    assert tracker.active_direction == 1

    # At the seam: tracker should commit to reverse.
    _, _, d1 = tracker._find_goal_point((4.0, -1.1), current_direction=1, lookahead=1.0)
    assert d1 == -1
    assert tracker.active_direction == -1

    # After commitment, keep reverse direction even if pose drifts.
    _, _, d2 = tracker._find_goal_point((2.8, 0.2), current_direction=-1, lookahead=1.0)
    assert d2 == -1
    assert tracker.active_direction == -1


def test_two_stage_plan_prefers_forward_approach_when_available():
    start = (10.0, 30.0)
    goal = (0.25, 8.25, 0.0)
    x_bounds = (0.0, 22.0)
    y_bounds = (0.0, 32.0)
    controller = BaselineParkingPlanner(
        planner_resolution=0.25,
        max_speed=2.0,
        max_reverse_speed=1.0,
        two_stage_parking=True,
        vehicle=AudiA5(),
        x_bounds=x_bounds,
        y_bounds=y_bounds,
    )

    path = controller.plan(
        start=start,
        start_heading=-np.pi / 2.0,
        goal=goal,
        static_obstacles=[],
        start_direction=None,
    )

    assert len(path) >= 3
    forward_count = sum(1 for p in path if p.direction > 0)
    reverse_count = sum(1 for p in path if p.direction < 0)
    assert forward_count >= 2
    assert reverse_count >= 1
