from __future__ import annotations

import json
from pathlib import Path
import subprocess

import numpy as np
import pytest

from simulator.benchmark import (
    PLANNERS,
    _release_executable,
    _validate_planner_response,
    build_plan_request,
)
from simulator.benchmark_metrics import measure_path, path_points_from_response
from simulator.adas_stack_catalog import PROJECT_ROOT
from simulator.experiment_runner import to_jsonable

SCENARIO = (
    PROJECT_ROOT
    / "artifacts"
    / "scenarios"
    / "benchmark"
    / "parallel_end_slot.json"
)


@pytest.mark.integration
@pytest.mark.parametrize("planner_id", PLANNERS)
def test_release_planner_only_integration(planner_id: str) -> None:
    try:
        executable = _release_executable(PROJECT_ROOT / "build")
    except (FileNotFoundError, ValueError) as exc:
        pytest.skip(f"Release simulator bridge unavailable: {exc}")

    request, sim = build_plan_request(SCENARIO, max_steps=200)
    try:
        assert request["type"] == "reset"
        serialized_request = json.dumps(
            to_jsonable(request), separators=(",", ":"), allow_nan=False
        )
        try:
            completed = subprocess.run(
                [str(executable), "--plan-once", "--path-planner", planner_id],
                input=serialized_request + "\n",
                capture_output=True,
                text=True,
                check=False,
                timeout=10.0,
            )
        except subprocess.TimeoutExpired as exc:
            pytest.fail(f"{planner_id} planner-only process exceeded 10-second timeout: {exc}")

        assert completed.returncode == 0, completed.stderr
        response = json.loads(completed.stdout)
        path, diagnostics = _validate_planner_response(response)
        assert response["type"] == "ready"
        assert isinstance(diagnostics["planning_time_ms"], (int, float))
        assert isinstance(diagnostics["solution_found"], bool)
        assert isinstance(diagnostics["expanded_nodes"], int)
        assert isinstance(diagnostics["goal_connection_attempts"], int)

        parsed_path = path_points_from_response(response)
        assert parsed_path == path
        path_metrics = measure_path(parsed_path, sim)
        assert path_metrics["path_waypoint_count"] == len(parsed_path)
        if parsed_path:
            assert diagnostics["solution_found"] or diagnostics["partial_plan"]
            assert all("heading" in point for point in parsed_path)
            assert isinstance(path_metrics["path_collision"], (bool, np.bool_))
            assert isinstance(path_metrics["path_valid"], (bool, np.bool_))
            assert path_metrics["path_length_m"] is not None
        else:
            assert diagnostics["solution_found"] is False
    finally:
        sim.close()
