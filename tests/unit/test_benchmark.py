from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
import numpy as np

from simulator.benchmark import (
    PLANNERS,
    _cmake_build_options,
    _executable_metadata,
    _release_executable,
    _release_executable_name,
    _cmake_behavior_metadata,
    _report_aggregate_rows,
    _compiler_metadata,
    _validate_planner_response,
    _runner_namespace,
    execute_planner_only_trial,
    execute_trial,
    generate_report,
    trial_schedule,
)
from simulator.benchmark_metrics import aggregate_trials, measure_path
from simulator.benchmark_metrics import path_points_from_response
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


@pytest.mark.parametrize(
    "path",
    [
        [],
        [{"x": "bad", "y": 0.0, "heading": 0.0}],
        [{"x": float("nan"), "y": 0.0, "heading": 0.0}],
        [{"x": 0.0, "y": 0.0, "heading": float("inf")}],
    ],
)
def test_empty_malformed_and_non_finite_paths_are_not_valid(path):
    result = measure_path(path, _sim())
    assert result["path_valid"] is False
    assert result["path_collision"] is None


def test_malformed_path_response_is_rejected():
    with pytest.raises((TypeError, ValueError, KeyError)):
        path_points_from_response({"planned_path": [{"x": "oops"}]})


@pytest.mark.parametrize(
    "response",
    [
        {"type": "ready", "planned_path": []},
        {"type": "error", "planned_path": [], "planning_diagnostics": {}},
        {
            "type": "ready",
            "planned_path": [],
            "planning_diagnostics": {"planning_time_ms": float("nan")},
        },
        {
            "type": "ready",
            "planned_path": [],
            "planning_diagnostics": {
                "expanded_nodes": True,
                "goal_connection_attempts": 0,
                "closest_goal_distance_m": 1.0,
                "planning_time_ms": 1.0,
                "time_limited": False,
                "solution_found": False,
                "partial_plan": False,
            },
        },
    ],
)
def test_planner_response_rejects_missing_or_invalid_diagnostics(response):
    with pytest.raises(ValueError):
        _validate_planner_response(response)


def test_empty_path_with_complete_diagnostics_is_valid_response_but_not_plan():
    path, diagnostics = _validate_planner_response({
        "type": "ready",
        "planned_path": [],
        "planning_diagnostics": {
            "expanded_nodes": 0,
            "goal_connection_attempts": 0,
            "closest_goal_distance_m": 1.0,
            "planning_time_ms": 0.25,
            "time_limited": False,
            "solution_found": False,
            "partial_plan": False,
        },
    })
    assert path == []
    assert diagnostics["solution_found"] is False


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


def test_executable_fingerprint_and_missing_file_metadata(tmp_path: Path):
    executable = tmp_path / "bridge"
    executable.write_bytes(b"fixture executable")
    metadata = _executable_metadata(executable)
    assert metadata["sha256"] == hashlib.sha256(
        b"fixture executable"
    ).hexdigest()
    assert metadata["size_bytes"] == len(b"fixture executable")
    assert metadata["available"] is True
    missing = _executable_metadata(tmp_path / "missing")
    assert missing == {
        "sha256": None,
        "size_bytes": None,
        "modified_utc": None,
        "available": False,
    }
    assert _cmake_behavior_metadata(tmp_path)["binary_commit_correspondence"].startswith(
        "not verified"
    )


def test_cmake_metadata_contains_relevant_build_options(tmp_path: Path):
    (tmp_path / "CMakeCache.txt").write_text(
        "CMAKE_BUILD_TYPE:STRING=Release\n"
        "CMAKE_GENERATOR:INTERNAL=Ninja\n"
        "CMAKE_CXX_FLAGS_RELEASE:STRING=-O3 -DNDEBUG\n"
        "ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL:BOOL=ON\n",
        encoding="utf-8",
    )
    options = _cmake_build_options(tmp_path)
    assert options["CMAKE_BUILD_TYPE"] == "Release"
    assert options["CMAKE_GENERATOR"] == "Ninja"
    assert options["CMAKE_CXX_FLAGS_RELEASE"] == "-O3 -DNDEBUG"
    assert options["ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL"] == "ON"


@pytest.mark.parametrize(
    ("platform_name", "build_type", "configurations", "executable_relative"),
    [
        ("nt", "", "Debug;Release", Path("Release") / "tcp_json_sim_bridge.exe"),
        ("posix", "Release", "", Path("tcp_json_sim_bridge")),
    ],
)
def test_release_executable_discovery_layouts(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    platform_name: str,
    build_type: str,
    configurations: str,
    executable_relative: Path,
):
    import simulator.benchmark as benchmark

    cache = (
        f"CMAKE_BUILD_TYPE:STRING={build_type}\n"
        f"CMAKE_CONFIGURATION_TYPES:STRING={configurations}\n"
    )
    (tmp_path / "CMakeCache.txt").write_text(cache, encoding="utf-8")
    executable = tmp_path / executable_relative
    executable.parent.mkdir(parents=True, exist_ok=True)
    executable.write_bytes(b"executable")
    monkeypatch.setattr(benchmark, "os", SimpleNamespace(name=platform_name))
    assert benchmark._release_executable(tmp_path) == executable.resolve()
    assert _release_executable_name(platform_name) == executable.name


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
            "planning_diagnostics": {
                "expanded_nodes": 4,
                "goal_connection_attempts": 1,
                "closest_goal_distance_m": 1.0,
                "planning_time_ms": 2.5,
                "time_limited": False,
                "solution_found": True,
                "partial_plan": False,
            },
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
    assert trial["success"] is None
    assert trial["parking_success"] is None
    assert trial["valid_plan"] is True
    assert trial["trial_status"] == "valid_plan"
    assert trial["collision_occurred"] is None
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


def test_planner_only_report_uses_valid_plan_not_parking_success_terms(tmp_path: Path):
    trial = {
        "planner_id": "hybrid_astar",
        "scenario_id": "scene",
        "benchmark_mode": "planner-only",
        "success": None,
        "valid_plan": True,
        "path_valid": True,
        "path_collision": False,
        "initial_planning_latency_ms": 3.0,
        "planner_only_latency_ms": 3.0,
        "path_length_m": 4.0,
        "path_waypoint_count": 8,
        "path_direction_changes": 1,
    }
    scenario = _report_aggregate_rows(aggregate_trials([trial]), "planner-only")
    assert "parking_success_rate" not in scenario[0]
    assert "success_count" not in scenario[0]
    assert "collision_rate" not in scenario[0]
    assert "path_collision_rate" in scenario[0]
    assert "episode_steps_median" not in scenario[0]
    planner = _report_aggregate_rows(
        aggregate_trials([{**trial, "scenario_id": "all"}]), "planner-only"
    )
    report = generate_report(
        tmp_path,
        {
            "benchmark_timestamp_utc": "test",
            "git_commit_sha": "abc",
            "build_configuration": "Release",
            "mode": "planner-only",
            "latency_label": "planner-only latency",
            "scenario_suite": "test",
            "repetitions": 1,
            "warmup_count": 0,
            "tracker_id": "pure_pursuit",
        },
        scenario,
        planner,
    )
    content = report.read_text(encoding="utf-8")
    assert "Valid plan rate" in content
    assert "Parking success rate" not in content
    assert "100.000%" in content
    assert "not a continuous swept-path certification" in content
    assert "Median simulated (s)" not in content


@pytest.mark.parametrize(
    ("outcome", "expected_status", "expected_reason"),
    [
        ("process_error", "invalid_plan", "returned 3"),
        ("timeout", "timeout", "TimeoutExpired"),
    ],
)
def test_planner_only_subprocess_failure_and_timeout_are_recorded(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    outcome: str,
    expected_status: str,
    expected_reason: str,
):
    class _Sim:
        vehicle = AudiA5()
        goal_slot = _GoalSlot()
        heading_tolerance = 0.1

        def close(self):
            pass

    monkeypatch.setattr(
        "simulator.benchmark.build_plan_request",
        lambda *_args: ({"type": "reset", "start": [0.0, 0.0]}, _Sim()),
    )

    def fake_run(command, **_kwargs):
        if outcome == "timeout":
            raise subprocess.TimeoutExpired(command, 0.01)
        return subprocess.CompletedProcess(command, 3, "", "planner crashed")

    monkeypatch.setattr("simulator.benchmark.subprocess.run", fake_run)
    trial = execute_planner_only_trial(
        planner_id="hybrid_astar",
        scenario_id="scene",
        artifact=tmp_path / "scene.json",
        trial_index=1,
        is_warmup=False,
        executable=tmp_path / "bridge",
        trial_path=tmp_path / "trial.json",
        log_path=tmp_path / "trial.log",
        max_steps=None,
        planner_timeout=1.0,
    )
    assert trial["trial_status"] == expected_status
    assert expected_reason in trial["failure_reason"]
    assert trial["valid_plan"] is False


def test_planner_only_trial_rejects_missing_diagnostics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    class _Sim:
        vehicle = AudiA5()
        goal_slot = _GoalSlot()
        heading_tolerance = 0.1

        def close(self):
            pass

    monkeypatch.setattr(
        "simulator.benchmark.build_plan_request",
        lambda *_args: ({"type": "reset"}, _Sim()),
    )
    monkeypatch.setattr(
        "simulator.benchmark.subprocess.run",
        lambda command, **_kwargs: subprocess.CompletedProcess(
            command, 0, json.dumps({"type": "ready", "planned_path": []}), ""
        ),
    )
    trial = execute_planner_only_trial(
        planner_id="hybrid_astar",
        scenario_id="scene",
        artifact=tmp_path / "scene.json",
        trial_index=1,
        is_warmup=False,
        executable=tmp_path / "bridge",
        trial_path=tmp_path / "trial.json",
        log_path=tmp_path / "trial.log",
        max_steps=None,
        planner_timeout=1.0,
    )
    assert trial["planning_diagnostics_valid"] is False
    assert "planning_diagnostics" in trial["failure_reason"]
    assert trial["valid_plan"] is False
