import numpy as np

from KarlsPlayground.planning import PathWaypoint
from KarlsPlayground.planning import TrajectoryTracker


def _state(x: float, y: float, yaw: float, vx: float = 0.5) -> dict:
    return {
        "position": (x, y),
        "heading": yaw,
        "velocity": (vx, 0.0),
        "steering_angle": 0.0,
    }


def test_fallback_tangent_uses_global_coordinates_translation_invariant():
    """Fallback tangent yaw should be invariant to world-frame translation."""
    tracker_a = TrajectoryTracker()
    tracker_b = TrajectoryTracker()

    # Use 3-tuples in spline_path to force fallback tangent calculation (no heading field).
    tracker_a.path = [PathWaypoint(0.0, 0.0, 1, 0.0), PathWaypoint(1.0, 0.0, 1, 0.0)]
    tracker_a.spline_path = [(0.0, 0.0, 1), (1.0, 0.0, 1)]
    tracker_a.current_index = 0

    tracker_b.path = [PathWaypoint(10.0, 5.0, 1, 0.0), PathWaypoint(11.0, 5.0, 1, 0.0)]
    tracker_b.spline_path = [(10.0, 5.0, 1), (11.0, 5.0, 1)]
    tracker_b.current_index = 0

    # Same relative pose w.r.t each path origin.
    accel_a, steer_a = tracker_a.compute_control(_state(0.1, 0.0, 0.0), goal_heading=0.0)
    accel_b, steer_b = tracker_b.compute_control(_state(10.1, 5.0, 0.0), goal_heading=0.0)

    assert np.isfinite(accel_a)
    assert np.isfinite(accel_b)
    assert abs(steer_a - steer_b) < 1e-6


def test_speed_dependent_steering_limiter_monotonic():
    tracker = TrajectoryTracker()

    l0 = tracker._steering_limit_from_speed(0.05)
    l1 = tracker._steering_limit_from_speed(0.5)
    l2 = tracker._steering_limit_from_speed(1.0)
    l3 = tracker._steering_limit_from_speed(2.0)

    assert l0 <= tracker.max_steering + 1e-9
    assert l3 >= 0.08 - 1e-9
    assert l0 >= l1 >= l2 >= l3


def test_compute_control_applies_speed_dependent_steering_limiter():
    tracker = TrajectoryTracker(lookahead_dist=1.0)
    path = [
        PathWaypoint(0.0, 0.0, 1, 0.0),
        PathWaypoint(5.0, 0.0, 1, 0.0),
    ]
    tracker.set_path(path)

    # High speed and large heading mismatch to provoke large raw steering demand.
    state = _state(x=0.0, y=0.0, yaw=np.pi / 2.0, vx=2.0)
    _, steering = tracker.compute_control(state, goal_heading=0.0)

    speed_limit = tracker._steering_limit_from_speed(2.0)
    assert abs(steering) <= speed_limit + 1e-9
