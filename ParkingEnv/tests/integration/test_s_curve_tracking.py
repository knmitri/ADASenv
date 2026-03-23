import numpy as np

from KarlsPlayground.testing import ControllerTestEnv


def test_s_curve_tracks_in_variable_speed_mode():
    env = ControllerTestEnv(dt=0.05)
    path = env.create_s_curve_path(start=(0.0, 0.0), num_points=50)
    result = env.run_test(path, "integration variable speed", max_steps=600)

    assert result["success"]
    assert result["final_pos_error"] < 0.31
    assert result["final_heading_error"] < 0.12


def test_s_curve_tracks_in_fixed_speed_mode():
    env = ControllerTestEnv(dt=0.05)
    path = env.create_s_curve_path(start=(0.0, 0.0), num_points=50)
    result = env.run_test(path, "integration fixed speed", max_steps=600, fixed_speed_mps=1.0)

    assert result["success"]
    assert result["final_pos_error"] < 0.31
    assert result["final_heading_error"] < 0.12
    assert abs(result["avg_velocity"] - 1.0) < 0.1


def test_initial_heading_override_is_applied():
    env = ControllerTestEnv(dt=0.05)
    path = env.create_s_curve_path(start=(10.0, 30.0), num_points=50)
    result = env.run_test(
        path,
        "integration heading override",
        max_steps=5,
        initial_heading=-np.pi / 2.0,
    )

    assert abs(result["start_pose"]["x"] - 10.0) < 1e-9
    assert abs(result["start_pose"]["y"] - 30.0) < 1e-9
    assert abs(result["start_pose"]["heading"] + np.pi / 2.0) < 1e-9
