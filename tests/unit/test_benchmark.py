from __future__ import annotations

import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
import numpy as np

from simulator.benchmark import (
    PLANNERS,
    _compiler_metadata,
    _runner_namespace,
    execute_planner_only_trial,
    execute_trial,
    generate_report,
    trial_schedule,
)
from simulator.benchmark_metrics import aggregate_trials, measure_path
from simulator.experiment_runner import _strict_jsonable
from simulator.simulation import AudiA5


class _GoalSlot:
    def get_corners(self):
        return np.array([
            [-20.0, -20.0],
            [20.0, -20.0],
            [20.0, 20.0],
            [-20.0, 20.0],
        ])


def _sim(collision=False):
    return SimpleNamespace(
        vehicle=AudiA5(),
        check_collision=lambda _corners: collision,
        get_goal_pose=lambda: (0.0, 0.0, 0.0),
        goal_slot=_GoalSlot(),
        heading_tolerance=0.1,
    )


def test_path_length_and_validity_use_coordinates_and_simulator_geometry():
    measured = measure_path(
        [
            {"x": -1.0, "y": 0.0, "heading": 0.0, "direction": 1},
            {"x": -0.5, "y": 0.0, "heading": 0.0, "direction": -1},
            {"x": 0.0, "y": 0.0, "heading": 0.0, "direction": -1},
        ],
        _sim(),
    )
    assert measured["path_length_m"] == pytest.approx(1.0)
    assert measured["path_waypoint_count"] == 3
    assert measured["path_direction_changes"] == 1
    assert measured["path_collision"] is False
    assert measured["path_valid"] is True


def test_path_validation_fails_closed_for_collision_or_missing_pose_fields():
    collided = measure_path(
        [{"x": 0.0, "y": 0.0, "heading": 0.0}],
        _sim(collision=True),
    )
    assert collided["path_collision"] is True
    assert collided["path_valid"] is False

    missing_heading = measure_path([{"x": 0.0, "y": 0.0}], _sim())
    assert missing_heading["path_length_m"] == 0.0
    assert missing_heading["path_collision"] is None
    assert missing_heading["path_valid"] is False


def test_aggregation_weights_trials_and_preserves_missing_metrics():
    trials = [
        {
            "planner_id": "hybrid_astar",
            "scenario_id": "easy",
            "success": True,
            "collision_occurred": False,
            "stuck": False,
            "timed_out": False,
            "initial_planning_latency_ms": 10.0,
            "path_length_m": 2.0,
            "simulation_steps": 20,
        },
        {
            "planner_id": "hybrid_astar",
            "scenario_id": "easy",
            "success": False,
            "collision_occurred": True,
            "stuck": False,
            "timed_out": False,
            "initial_planning_latency_ms": None,
            "path_length_m": None,
            "simulation_steps": None,
        },
        {
            "planner_id": "hybrid_astar",
            "scenario_id": "hard",
            "success": True,
            "collision_occurred": False,
            "stuck": False,
            "timed_out": False,
            "initial_planning_latency_ms": 30.0,
            "path_length_m": 4.0,
            "simulation_steps": 40,
        },
    ]
    aggregates = aggregate_trials(trials + [{**trial, "scenario_id": "all"} for trial in trials])
    overall = next(row for row in aggregates if row["scenario_id"] == "all")
    assert overall["trial_count"] == 3
    assert overall["success_rate"] == pytest.approx(2 / 3)
    assert overall["collision_rate"] == pytest.approx(1 / 3)
    assert overall["planning_latency_count"] == 2
    assert overall["planning_latency_median_ms"] == pytest.approx(20.0)
    assert overall["planning_latency_max_ms"] == pytest.approx(30.0)
    assert overall["path_length_count"] == 2


def test_trial_schedule_repeats_and_alternates_planner_order():
    schedule = trial_schedule(["scene"], repetitions=2, warmup_count=1)
    warmups = [item for item in schedule if item[3]]
    measured = [item for item in schedule if not item[3]]
    assert len(warmups) == len(PLANNERS)
    assert len(measured) == 2 * len(PLANNERS)
    assert [item[1] for item in measured[:2]] == list(PLANNERS)
    assert [item[1] for item in measured[2:]] == list(reversed(PLANNERS))


def test_failed_trial_is_recorded_and_report_is_written(tmp_path: Path):
    def timeout_runner(_args):
        raise TimeoutError("simulator timed out")

    trial = execute_trial(
        planner_id="hybrid_astar",
        scenario_id="scene",
        artifact=tmp_path / "scene.json",
        trial_index=1,
        is_warmup=False,
        catalog_path=tmp_path / "catalog.json",
        trial_path=tmp_path / "trial.json",
        log_path=tmp_path / "trial.log",
        max_steps=10,
        accept_timeout=1.0,
        socket_timeout=1.0,
        runner=timeout_runner,
    )
    assert trial["trial_status"] == "timeout"
    assert trial["timed_out"] is True
    assert "simulator timed out" in trial["failure_reason"]

    aggregates = aggregate_trials([trial])
    report = generate_report(
        tmp_path,
        {
            "benchmark_timestamp_utc": "test",
            "git_commit_sha": "abc",
            "build_configuration": "Release",
            "mode": "accelerated-closed-loop",
            "latency_label": "closed-loop initial planning latency",
            "scenario_suite": "test",
            "repetitions": 1,
            "warmup_count": 0,
            "tracker_id": "pure_pursuit",
        },
        aggregates,
        aggregate_trials([{**trial, "scenario_id": "all"}]),
    )
    text = report.read_text(encoding="utf-8")
    assert "hybrid_astar" in text
    assert "n/a" in text
    assert "Max plan (ms)" in text
    assert "Median simulated (s)" in text
    assert "Median episode wall (s)" in text
    assert "Speed factor" in text
    assert json.loads(json.dumps(trial))["trial_status"] == "timeout"


def test_non_finite_result_metrics_are_serialized_as_missing():
    assert _strict_jsonable({
        "position_error": float("nan"),
        "nested": [float("inf"), 1.0],
    }) == {
        "position_error": None,
        "nested": [None, 1.0],
    }


def test_compiler_metadata_uses_generated_cmake_identification(tmp_path: Path):
    compiler_metadata = tmp_path / "CMakeFiles" / "3.30.0"
    compiler_metadata.mkdir(parents=True)
    (compiler_metadata / "CMakeCXXCompiler.cmake").write_text(
        'set(CMAKE_CXX_COMPILER "C:/Program Files (x86)/toolchain/cl.exe")\n'
        'set(CMAKE_CXX_COMPILER_ID "MSVC")\n'
        'set(CMAKE_CXX_COMPILER_VERSION "19.40.1")\n',
        encoding="utf-8",
    )
    assert _compiler_metadata(tmp_path) == {
        "id": "MSVC",
        "version": "19.40.1",
        "path": "C:/Program Files (x86)/toolchain/cl.exe",
    }


def test_planner_only_uses_direct_planner_and_separate_latency_fields(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    class _Sim:
        vehicle = AudiA5()
        goal_slot = _GoalSlot()
        heading_tolerance = 0.1

        def check_collision(self, _corners):
            return False

        def get_goal_pose(self):
            return 0.0, 0.0, 0.0

        def close(self):
            pass

    request = {
        "type": "reset",
        "start": [0.0, 0.0],
        "start_heading": 0.0,
        "goal": [0.0, 0.0, 0.0],
        "simulation": {"dt": 0.05},
    }
    monkeypatch.setattr(
        "simulator.benchmark.build_plan_request",
        lambda *_args: (request, _Sim()),
    )
    calls = {}

    def fake_run(command, **kwargs):
        calls["command"] = command
        calls["kwargs"] = kwargs
        response = {
            "type": "ready",
            "planned_path": [
                {"x": -1.0, "y": 0.0, "heading": 0.0, "direction": 1},
                {"x": 0.0, "y": 0.0, "heading": 0.0, "direction": 1},
            ],
            "planning_diagnostics": {"planning_time_ms": 2.5, "solution_found": True},
        }
        return subprocess.CompletedProcess(command, 0, json.dumps(response), "")

    monkeypatch.setattr("simulator.benchmark.subprocess.run", fake_run)
    trial = execute_planner_only_trial(
        planner_id="hybrid_astar",
        scenario_id="scene",
        artifact=tmp_path / "scene.json",
        trial_index=1,
        is_warmup=False,
        executable=tmp_path / "bridge.exe",
        trial_path=tmp_path / "trial.json",
        log_path=tmp_path / "trial.log",
        max_steps=None,
        planner_timeout=3.0,
    )
    assert calls["command"] == [
        str(tmp_path / "bridge.exe"),
        "--plan-once",
        "--path-planner",
        "hybrid_astar",
    ]
    assert calls["kwargs"]["timeout"] == 3.0
    assert json.loads(calls["kwargs"]["input"])["type"] == "reset"
    assert trial["benchmark_mode"] == "planner-only"
    assert trial["planning_latency_source"] == "planner_only"
    assert trial["planner_only_latency_ms"] == pytest.approx(2.5)
    assert trial["closed_loop_initial_planning_latency_ms"] is None
    assert trial["path_length_m"] == pytest.approx(1.0)
    assert trial["path_waypoint_count"] == 2
    assert trial["path_direction_changes"] == 0
    assert trial["success"] is True
    assert trial["simulated_duration_seconds"] is None


def test_closed_loop_trial_separates_wall_and_simulated_durations(tmp_path: Path):
    result_path = tmp_path / "result.json"

    def successful_runner(_args):
        result_path.write_text(json.dumps({
            "success": True,
            "collision_occurred": False,
            "steps": 10,
            "initial_planning_latency_ms": 4.0,
            "episode_duration_seconds": 0.25,
        }), encoding="utf-8")
        return 0

    trial = execute_trial(
        planner_id="hybrid_astar",
        scenario_id="scene",
        artifact=tmp_path / "scene.json",
        trial_index=1,
        is_warmup=False,
        catalog_path=tmp_path / "catalog.json",
        trial_path=result_path,
        log_path=tmp_path / "log.txt",
        max_steps=None,
        accept_timeout=1.0,
        socket_timeout=1.0,
        runner=successful_runner,
    )
    assert trial["benchmark_mode"] == "accelerated-closed-loop"
    assert trial["closed_loop_initial_planning_latency_ms"] == 4.0
    assert trial["planner_only_latency_ms"] is None
    assert trial["simulated_duration_seconds"] == pytest.approx(0.5)
    assert trial["episode_wall_clock_seconds"] == pytest.approx(0.25)
    assert trial["simulation_speed_factor"] == pytest.approx(2.0)


def test_benchmark_cli_defaults_to_compatible_accelerated_closed_loop(
    monkeypatch: pytest.MonkeyPatch,
):
    from simulator.benchmark import parse_args

    monkeypatch.setattr("sys.argv", ["benchmark"])
    args = parse_args()
    assert args.mode == "accelerated-closed-loop"
    assert args.repetitions == 5
    assert args.warmup_count == 1


def test_benchmark_runner_namespace_disables_rendering(tmp_path: Path):
    args = _runner_namespace(
        planner_id="hybrid_astar",
        catalog_path=tmp_path / "catalog.json",
        artifact=tmp_path / "scene.json",
        result_path=tmp_path / "result.json",
        max_steps=None,
        accept_timeout=1.0,
        socket_timeout=1.0,
    )
    assert args.render is False
    assert args.save_render_dir is None
    assert args.trace_dir is None


def test_aggregation_keeps_planner_latency_sources_distinct():
    rows = aggregate_trials([
        {
            "planner_id": "hybrid_astar",
            "scenario_id": "scene",
            "initial_planning_latency_ms": 8.0,
            "planner_only_latency_ms": None,
            "closed_loop_initial_planning_latency_ms": 8.0,
            "success": True,
        },
        {
            "planner_id": "reeds_shepp_local",
            "scenario_id": "scene",
            "initial_planning_latency_ms": 2.0,
            "planner_only_latency_ms": 2.0,
            "closed_loop_initial_planning_latency_ms": None,
            "success": True,
        },
    ])
    by_planner = {row["planner_id"]: row for row in rows}
    assert by_planner["hybrid_astar"]["closed_loop_initial_planning_latency_median_ms"] == 8.0
    assert by_planner["hybrid_astar"]["planner_only_latency_median_ms"] is None
    assert by_planner["reeds_shepp_local"]["planner_only_latency_median_ms"] == 2.0
    assert by_planner["reeds_shepp_local"]["closed_loop_initial_planning_latency_median_ms"] is None
