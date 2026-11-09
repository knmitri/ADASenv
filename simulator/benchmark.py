"""Reproducible, paired benchmarks for the reference parking ADAS planners."""

from __future__ import annotations

import argparse
import contextlib
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import platform
import re
import subprocess
import time
import uuid
from typing import Any, Iterable

from simulator import experiment_runner
from simulator.adas_stack_catalog import PROJECT_ROOT
from simulator.benchmark_metrics import aggregate_trials
from simulator.config_loader import get_config
from simulator.simulation import AudiA5
from simulator.simulation.challenge import scenario_artifacts_for_suite
from simulator.simulation.environment import ParkingSimulationEnvironment
from simulator.simulation.scenarios import load_parking_scenario

PLANNERS = ("hybrid_astar", "reeds_shepp_local")
TRACKER_ID = "pure_pursuit"
TRIAL_FIELDS = (
    "planner_id", "scenario_id", "scenario_artifact", "trial_index", "is_warmup",
    "trial_status", "success", "collision_occurred", "stuck", "timed_out",
    "failure_reason", "initial_plan_valid", "path_valid", "path_collision",
    "final_path_valid", "final_path_collision",
    "benchmark_mode", "planning_latency_source",
    "planner_only_latency_ms", "closed_loop_initial_planning_latency_ms",
    "initial_planning_latency_ms", "planner_only_process_wall_seconds",
    "wall_clock_execution_seconds", "episode_wall_clock_seconds",
    "episode_duration_seconds", "simulated_duration_seconds",
    "simulation_speed_factor",
    "process_spawn_seconds", "stack_launch_to_connection_seconds", "handshake_seconds",
    "simulation_steps", "simulation_step_time_seconds", "path_length_m",
    "path_waypoint_count", "path_direction_changes", "final_position_error_m",
    "final_heading_error_deg", "runner_return_code",
)


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(_jsonable(value), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "item"):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _compiler_metadata(build_dir: Path) -> dict[str, str | None]:
    cache_path = build_dir / "CMakeCache.txt"
    entries: dict[str, str] = {}
    if cache_path.exists():
        for line in cache_path.read_text(encoding="utf-8", errors="replace").splitlines():
            if line and not line.startswith(("//", "#")) and "=" in line:
                key, value = line.split("=", 1)
                entries[key.split(":", 1)[0]] = value
    compiler = entries.get("CMAKE_CXX_COMPILER")
    compiler_id = entries.get("CMAKE_CXX_COMPILER_ID")
    compiler_version = entries.get("CMAKE_CXX_COMPILER_VERSION")
    compiler_metadata_files = sorted(
        (build_dir / "CMakeFiles").glob("*/CMakeCXXCompiler.cmake")
    )
    if compiler_metadata_files:
        compiler_text = compiler_metadata_files[-1].read_text(
            encoding="utf-8", errors="replace"
        )
        for key, current in (
            ("CMAKE_CXX_COMPILER_ID", compiler_id),
            ("CMAKE_CXX_COMPILER_VERSION", compiler_version),
            ("CMAKE_CXX_COMPILER", compiler),
        ):
            if current is None:
                match = re.search(
                    rf'^set\({key}\s+(?:"([^"\r\n]*)"|([^)\r\n]+))\)',
                    compiler_text,
                    re.MULTILINE,
                )
                if match:
                    value = match.group(1) or match.group(2).strip()
                    if key == "CMAKE_CXX_COMPILER_ID":
                        compiler_id = value
                    elif key == "CMAKE_CXX_COMPILER_VERSION":
                        compiler_version = value
                    else:
                        compiler = value
    return {
        "id": compiler_id,
        "version": compiler_version,
        "path": compiler,
    }


def _release_executable(build_dir: Path) -> Path:
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.exists():
        raise FileNotFoundError(f"No CMake cache found in {build_dir}; configure a Release build first.")
    entries: dict[str, str] = {}
    for line in cache_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line and not line.startswith(("//", "#")) and "=" in line:
            key, value = line.split("=", 1)
            entries[key.split(":", 1)[0]] = value
    configurations = entries.get("CMAKE_CONFIGURATION_TYPES", "")
    build_type = entries.get("CMAKE_BUILD_TYPE", "")
    if "Release" not in configurations.split(";") and build_type.lower() != "release":
        raise ValueError(
            f"{build_dir} is not configured for Release (build type={build_type!r}, "
            f"configurations={configurations!r})"
        )

    executable_name = "tcp_json_sim_bridge.exe" if os.name == "nt" else "tcp_json_sim_bridge"
    candidates = (
        build_dir / "Release" / executable_name,
        build_dir / executable_name,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        f"Release bridge executable not found. Checked: {', '.join(map(str, candidates))}"
    )


def _new_output_directory(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    for _ in range(10):
        output = root / f"benchmark_{stamp}_{uuid.uuid4().hex[:8]}"
        try:
            output.mkdir()
            return output
        except FileExistsError:
            continue
    raise FileExistsError(f"Unable to create a unique benchmark directory under {root}")


def trial_schedule(
    scenario_ids: list[str], repetitions: int, warmup_count: int
) -> list[tuple[str, str, int, bool]]:
    schedule: list[tuple[str, str, int, bool]] = []
    for scenario_index, scenario_id in enumerate(scenario_ids):
        for warmup in range(warmup_count):
            order = PLANNERS if (warmup + scenario_index) % 2 == 0 else tuple(reversed(PLANNERS))
            schedule.extend((scenario_id, planner, warmup + 1, True) for planner in order)
        for trial_index in range(1, repetitions + 1):
            order = PLANNERS if (trial_index + scenario_index) % 2 else tuple(reversed(PLANNERS))
            schedule.extend((scenario_id, planner, trial_index, False) for planner in order)
    return schedule


def _scenario_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git_sha() -> str | None:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=PROJECT_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def _git_worktree_dirty() -> bool | None:
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=PROJECT_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        return bool(result.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        return None


def _runner_namespace(
    *,
    planner_id: str,
    catalog_path: Path,
    artifact: Path,
    result_path: Path,
    max_steps: int | None,
    accept_timeout: float,
    socket_timeout: float,
) -> argparse.Namespace:
    return argparse.Namespace(
        max_steps=max_steps,
        scenario_artifact=[artifact],
        scenario_suite="benchmark",
        host="127.0.0.1",
        port=0,
        connect_host=None,
        wait_for_external_stack=False,
        adas_stack=planner_id,
        adas_stack_catalog=catalog_path,
        accept_timeout=accept_timeout,
        socket_timeout=socket_timeout,
        slots=None,
        all_empty_slots=False,
        log_interval=0,
        render=False,
        render_every=1,
        save_render_dir=None,
        trace_dir=None,
        result_json=result_path,
    )


def _trial_status(result: dict[str, Any]) -> str:
    if result.get("success"):
        return "success"
    if result.get("collision_occurred"):
        return "collision"
    if result.get("timed_out") or result.get("stuck"):
        return "timeout"
    return "failure"


def execute_trial(
    *,
    planner_id: str,
    scenario_id: str,
    artifact: Path,
    trial_index: int,
    is_warmup: bool,
    catalog_path: Path,
    trial_path: Path,
    log_path: Path,
    max_steps: int | None,
    accept_timeout: float,
    socket_timeout: float,
    runner: Any = experiment_runner.run_experiment,
) -> dict[str, Any]:
    output = io.StringIO()
    return_code: int | None = None
    failure_reason = None
    wall_started = time.perf_counter()
    try:
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            return_code = runner(_runner_namespace(
                planner_id=planner_id,
                catalog_path=catalog_path,
                artifact=artifact,
                result_path=trial_path,
                max_steps=max_steps,
                accept_timeout=accept_timeout,
                socket_timeout=socket_timeout,
            ))
        result = json.loads(trial_path.read_text(encoding="utf-8"))
    except Exception as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
        result = {}
        if trial_path.exists():
            try:
                result = json.loads(trial_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as result_error:
                failure_reason += (
                    f"; could not read partial result: "
                    f"{type(result_error).__name__}: {result_error}"
                )
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(output.getvalue(), encoding="utf-8")
    result.update({
        "benchmark_mode": "accelerated-closed-loop",
        "planning_latency_source": "closed_loop_initial",
        "closed_loop_initial_planning_latency_ms": result.get(
            "initial_planning_latency_ms"
        ),
        "planner_only_latency_ms": None,
        "wall_clock_execution_seconds": time.perf_counter() - wall_started,
        "planner_id": planner_id,
        "scenario_id": scenario_id,
        "scenario_artifact": str(artifact.resolve()),
        "trial_index": trial_index,
        "is_warmup": is_warmup,
        "failure_reason": failure_reason,
        "runner_return_code": return_code,
        "runner_log": str(log_path),
    })
    result["trial_status"] = (
        "timeout" if failure_reason and ("TimeoutError" in failure_reason or "timeout" in failure_reason.lower())
        else "failure" if failure_reason
        else _trial_status(result)
    )
    result.setdefault("success", False)
    result.setdefault("collision_occurred", False)
    result.setdefault("stuck", False)
    result.setdefault("timed_out", result["trial_status"] == "timeout")
    result["episode_wall_clock_seconds"] = result.get(
        "episode_wall_clock_seconds", result.get("episode_duration_seconds")
    )
    result["episode_duration_seconds"] = result.get(
        "episode_wall_clock_seconds"
    )
    dt = float(get_config()["simulation"]["dt"])
    steps = result.get("simulation_steps", result.get("steps"))
    result["simulated_duration_seconds"] = (
        float(steps) * dt if isinstance(steps, (int, float)) else None
    )
    episode_wall = result["episode_wall_clock_seconds"]
    result["simulation_speed_factor"] = (
        result["simulated_duration_seconds"] / episode_wall
        if result["simulated_duration_seconds"] is not None
        and isinstance(episode_wall, (int, float)) and episode_wall > 0 else None
    )
    final_path_metrics = result.get("final_commanded_path_metrics", {})
    result["final_path_valid"] = final_path_metrics.get("path_valid")
    result["final_path_collision"] = final_path_metrics.get("path_collision")
    result["simulation_steps"] = result.get("steps")
    return result


def build_plan_request(
    artifact: Path,
    max_steps: int | None,
) -> tuple[dict[str, Any], ParkingSimulationEnvironment]:
    """Build the same reset request used by the closed-loop TCP runner."""
    config = json.loads(json.dumps(get_config()))
    config["simulation"]["render"] = False
    if max_steps is not None:
        config["simulation"]["max_episode_steps"] = max_steps
    scenario = load_parking_scenario(artifact)
    vehicle = AudiA5()
    sim = ParkingSimulationEnvironment(vehicle=vehicle, config=config, scenario=scenario)
    sim.render_enabled = False
    state = sim.reset()
    request = experiment_runner.reset_payload_for_simulation(
        config, scenario, vehicle, sim, state
    )
    return request, sim


def execute_planner_only_trial(
    *,
    planner_id: str,
    scenario_id: str,
    artifact: Path,
    trial_index: int,
    is_warmup: bool,
    executable: Path,
    trial_path: Path,
    log_path: Path,
    max_steps: int | None,
    planner_timeout: float,
) -> dict[str, Any]:
    """Invoke the selected C++ planner directly, without TCP or simulation."""
    sim: ParkingSimulationEnvironment | None = None
    request_sha256: str | None = None
    started = time.perf_counter()
    process_wall_seconds: float | None = None
    failure_reason = None
    stdout = ""
    try:
        request, sim = build_plan_request(artifact, max_steps)
        request = experiment_runner.to_jsonable(request)
        request_sha256 = hashlib.sha256(
            json.dumps(request, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        ).hexdigest()
        process_started = time.perf_counter()
        completed = subprocess.run(
            [str(executable), "--plan-once", "--path-planner", planner_id],
            input=json.dumps(request, separators=(",", ":"), allow_nan=False) + "\n",
            capture_output=True,
            text=True,
            check=False,
            timeout=planner_timeout,
        )
        process_wall_seconds = time.perf_counter() - process_started
        stdout = completed.stdout
        if completed.returncode != 0:
            raise RuntimeError(
                f"planner process returned {completed.returncode}: {completed.stderr.strip()}"
            )
        response = json.loads(completed.stdout)
        if response.get("type") != "ready":
            raise RuntimeError(f"Expected ready response from planner-only process: {response}")
        points = response.get("planned_path", [])
        path_metrics = experiment_runner.measure_path(points, sim)
        diagnostics = response.get("planning_diagnostics", {})
        solution_found = bool(diagnostics.get("solution_found", False))
        path_valid = bool(path_metrics["path_valid"])
        success = solution_found and path_valid
        timed_out = bool(diagnostics.get("time_limited", False))
        result = {
            **path_metrics,
            "planning_diagnostics": diagnostics,
            "initial_planning_latency_ms": diagnostics.get("planning_time_ms"),
            "planner_only_latency_ms": diagnostics.get("planning_time_ms"),
            "closed_loop_initial_planning_latency_ms": None,
            "planning_latency_source": "planner_only",
            "initial_plan_valid": success,
            "success": success,
            "collision_occurred": path_metrics.get("path_collision"),
            "stuck": False,
            "timed_out": timed_out,
            "simulation_steps": None,
            "simulation_step_time_seconds": None,
            "episode_wall_clock_seconds": None,
            "episode_duration_seconds": None,
            "simulated_duration_seconds": None,
            "simulation_speed_factor": None,
            "wall_clock_execution_seconds": time.perf_counter() - started,
            "planner_only_process_wall_seconds": process_wall_seconds,
            "runner_return_code": completed.returncode,
            "trial_status": "timeout" if timed_out else "success" if success else "failure",
            "failure_reason": None,
        }
    except Exception as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
        timed_out = isinstance(exc, subprocess.TimeoutExpired)
        result = {
            "initial_planning_latency_ms": None,
            "planner_only_latency_ms": None,
            "closed_loop_initial_planning_latency_ms": None,
            "planning_latency_source": "planner_only",
            "path_length_m": None,
            "path_waypoint_count": None,
            "path_direction_changes": None,
            "path_valid": False,
            "path_collision": None,
            "initial_plan_valid": False,
            "success": False,
            "collision_occurred": None,
            "stuck": False,
            "timed_out": timed_out,
            "simulation_steps": None,
            "simulation_step_time_seconds": None,
            "episode_wall_clock_seconds": None,
            "episode_duration_seconds": None,
            "simulated_duration_seconds": None,
            "simulation_speed_factor": None,
            "wall_clock_execution_seconds": time.perf_counter() - started,
            "planner_only_process_wall_seconds": process_wall_seconds,
            "runner_return_code": None,
            "trial_status": "timeout" if timed_out else "failure",
            "failure_reason": failure_reason,
        }
    finally:
        if sim is not None:
            sim.close()

    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(stdout, encoding="utf-8")
    result.update({
        "benchmark_mode": "planner-only",
        "planner_id": planner_id,
        "scenario_id": scenario_id,
        "scenario_artifact": str(artifact.resolve()),
        "trial_index": trial_index,
        "is_warmup": is_warmup,
        "plan_request_sha256": request_sha256,
        "runner_log": str(log_path),
        "failure_reason": result.get("failure_reason") or failure_reason,
    })
    trial_path.parent.mkdir(parents=True, exist_ok=True)
    _write_json(trial_path, result)
    return result


def _write_csv(path: Path, rows: list[dict[str, Any]], fields: Iterable[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def generate_report(
    output_dir: Path,
    metadata: dict[str, Any],
    scenario_aggregates: list[dict[str, Any]],
    planner_aggregates: list[dict[str, Any]],
) -> Path:
    lines = [
        "# ADAS Parking Benchmark",
        "",
        f"- Run: `{metadata['benchmark_timestamp_utc']}`",
        f"- Commit: `{metadata.get('git_commit_sha') or 'unknown'}`",
        f"- Worktree dirty: `{metadata.get('git_worktree_dirty', 'unknown')}`",
        f"- Configuration: `{metadata['build_configuration']}`",
        f"- Mode: `{metadata['mode']}`",
        f"- Suite: `{metadata['scenario_suite']}`",
        f"- Repetitions/planner/scenario: {metadata['repetitions']}",
        f"- Warm-ups/planner/scenario: {metadata['warmup_count']}",
        f"- Tracker: `{metadata['tracker_id']}`",
        "",
        "## Planner aggregates",
        "",
        f"| Planner | Trials | Success | Collision | Stuck | Timeout | Failure | Median {metadata['latency_label']} (ms) | P95 plan (ms) | Max plan (ms) | Mean +/- SD (ms) | Median path (m) | Median simulated (s) | Median episode wall (s) | Speed factor | Median steps | Position error (m) | Heading error (deg) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in planner_aggregates:
        mean = row["planning_latency_mean_ms"]
        stddev = row["planning_latency_stddev_ms"]
        lines.append(
            f"| {row['planner_id']} | {row['trial_count']} | "
            f"{row['success_rate']:.1%} | {row['collision_rate']:.1%} | "
            f"{row['stuck_rate']:.1%} | {row['timeout_rate']:.1%} | "
            f"{row['failure_rate']:.1%} | {_fmt(row['planning_latency_median_ms'])} | "
            f"{_fmt(row['planning_latency_p95_ms'])} | "
            f"{_fmt(row['planning_latency_max_ms'])} | {_fmt(mean)} +/- {_fmt(stddev)} | "
            f"{_fmt(row['path_length_median'])} | "
            f"{_fmt(row['simulated_duration_median_seconds'])} | "
            f"{_fmt(row['episode_duration_median_seconds'])} | "
            f"{_fmt(row['simulation_speed_factor_median'])} | "
            f"{_fmt(row['episode_steps_median'])} | "
            f"{_fmt(row['final_position_error_median'])} | "
            f"{_fmt(row['final_heading_error_deg_median'])} |"
        )
    lines.extend(["", "## Scenario results", "",
                  f"| Scenario | Planner | Trials | Success | Collision | Stuck | Timeout | Failure | Median {metadata['latency_label']} (ms) | P95 plan (ms) | Max plan (ms) | Mean +/- SD (ms) | Median path (m) | Median simulated (s) | Median episode wall (s) | Speed factor | Median steps | Position error (m) | Heading error (deg) |",
                  "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"])
    for row in scenario_aggregates:
        lines.append(
            f"| {row['scenario_id']} | {row['planner_id']} | {row['trial_count']} | "
            f"{row['success_rate']:.1%} | {row['collision_rate']:.1%} | "
            f"{row['stuck_rate']:.1%} | {row['timeout_rate']:.1%} | "
            f"{row['failure_rate']:.1%} | {_fmt(row['planning_latency_median_ms'])} | "
            f"{_fmt(row['planning_latency_p95_ms'])} | "
            f"{_fmt(row['planning_latency_max_ms'])} | "
            f"{_fmt(row['planning_latency_mean_ms'])} +/- "
            f"{_fmt(row['planning_latency_stddev_ms'])} | "
            f"{_fmt(row['path_length_median'])} | "
            f"{_fmt(row['simulated_duration_median_seconds'])} | "
            f"{_fmt(row['episode_duration_median_seconds'])} | "
            f"{_fmt(row['simulation_speed_factor_median'])} | "
            f"{_fmt(row['episode_steps_median'])} | "
            f"{_fmt(row['final_position_error_median'])} | "
            f"{_fmt(row['final_heading_error_deg_median'])} |"
        )
    lines.extend([
        "",
        "## Metric definitions and limitations",
        "",
        "- The simulator and experiment loop contain no real-time pacing or sleeping. Performance modes disable rendering. The fixed `dt`, vehicle dynamics, controller cadence, actuator limits, and termination checks are unchanged.",
        f"- Planning latency is specific to `{metadata['mode']}`. Planner-only latency is the C++ planner diagnostic and excludes process startup; its process wall time includes launching the one-shot process. Closed-loop initial planning latency is the diagnostic from the initial ready response.",
        "- Simulated episode duration is `simulation_steps * dt`; episode wall duration is measured separately from reset through termination. The speed factor is simulated duration divided by episode wall duration. Full closed-loop trial wall time additionally includes bridge startup, connection/handshake, and shutdown.",
        "- Geometric path length is the sum of Euclidean distances between consecutive coordinates from the initial plan. It is not inferred from waypoint count.",
        "- Path validity checks each reported waypoint using the simulator's vehicle footprint, static obstacle, boundary, goal-slot, and heading rules. It is discrete waypoint validation, not continuous swept-path certification.",
        "- Simulation step time is measured only around the Python `sim.step()` call. It does not include TCP and command handling.",
        "- Failure trials are retained. Missing measurements remain blank and are not imputed; each aggregate reports the number of available planning-latency/path-length samples.",
        "- The built-in scenario artifacts are deterministic and disable random spawning. Planner order alternates by scenario and trial; warm-ups are recorded but excluded from aggregates.",
        "- Planner-only mode directly invokes the selected planner using the same serialized reset/PlanRequest fields as closed-loop mode. It bypasses TCP, tracking, and vehicle simulation, so episode duration and speed factor are not applicable; path validity is waypoint-level geometric validation.",
        "- Windows/Linux timing is hardware- and load-dependent. Compare runs on the same host/build when drawing performance conclusions.",
        "",
    ])
    path = output_dir / "benchmark_report.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def _fmt(value: Any) -> str:
    return "n/a" if value is None or not math.isfinite(float(value)) else f"{float(value):.3f}"


def _create_catalog(path: Path, executable: Path) -> None:
    _write_json(path, {
        "stacks": [{
            "id": planner,
            "name": planner,
            "executable": str(executable),
            "args": [
                "--tcp-client", "--path-planner", planner,
                "--trajectory-tracker", TRACKER_ID, "--host", "{host}",
                "--port", "{port}",
            ],
        } for planner in PLANNERS],
    })


def run_benchmark(args: argparse.Namespace) -> Path:
    executable = _release_executable(args.build_dir.resolve())
    artifacts = (
        [path.resolve() for path in args.scenario_artifact]
        if args.scenario_artifact
        else [path.resolve() for path in scenario_artifacts_for_suite(args.scenario_suite)]
    )
    if not artifacts:
        raise ValueError("The benchmark scenario selection is empty.")
    for artifact in artifacts:
        if not artifact.is_file():
            raise FileNotFoundError(f"Scenario artifact does not exist: {artifact}")
    scenario_ids = []
    for artifact in artifacts:
        scenario_id = json.loads(artifact.read_text(encoding="utf-8")).get(
            "scenario_id", artifact.stem
        )
        if not isinstance(scenario_id, str) or not scenario_id:
            raise ValueError(f"Invalid scenario_id in artifact {artifact}: {scenario_id!r}")
        scenario_ids.append(scenario_id)
    if len(set(scenario_ids)) != len(scenario_ids):
        raise ValueError(f"Scenario artifacts must have unique scenario IDs: {scenario_ids}")

    output_dir = _new_output_directory(args.output_root.resolve())
    catalog_path = output_dir / "benchmark_stacks.json"
    _create_catalog(catalog_path, executable)
    config = json.loads(json.dumps(get_config()))
    config["simulation"]["render"] = False
    if args.max_steps is not None:
        config["simulation"]["max_episode_steps"] = args.max_steps
    metadata = {
        "benchmark_timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit_sha": _git_sha(),
        "git_worktree_dirty": _git_worktree_dirty(),
        "build_configuration": "Release",
        "build_directory": str(args.build_dir.resolve()),
        "executable": str(executable),
        "operating_system": platform.platform(),
        "architecture": platform.machine(),
        "python_version": platform.python_version(),
        "compiler": _compiler_metadata(args.build_dir.resolve()),
        "scenario_suite": (
            "explicit" if args.scenario_artifact else args.scenario_suite
        ),
        "scenario_artifacts": [
            {"path": str(path), "artifact_id": scenario_id, "sha256": _scenario_hash(path)}
            for path, scenario_id in zip(artifacts, scenario_ids, strict=True)
        ],
        "planner_ids": list(PLANNERS),
        "tracker_id": TRACKER_ID,
        "mode": args.mode,
        "latency_label": (
            "planner-only latency"
            if args.mode == "planner-only"
            else "closed-loop initial planning latency"
        ),
        "rendering_enabled": False,
        "real_time_pacing_enabled": False,
        "vehicle_configuration": experiment_runner.vehicle_as_dict(AudiA5()),
        "simulation_configuration": _jsonable(config["simulation"]),
        "scenario_configuration": _jsonable(config.get("scenario", {})),
        "repetitions": args.repetitions,
        "warmup_count": args.warmup_count,
        "max_steps_override": args.max_steps,
        "accept_timeout_seconds": args.accept_timeout,
        "socket_timeout_seconds": args.socket_timeout,
        "planner_timeout_seconds": args.planner_timeout,
        "random_spawn": bool(config["simulation"].get("random_spawn", False)),
    }
    comparison_inputs = {
        "scenario_artifacts": metadata["scenario_artifacts"],
        "tracker_id": TRACKER_ID,
        "vehicle_configuration": metadata["vehicle_configuration"],
        "simulation_configuration": metadata["simulation_configuration"],
        "max_steps_override": args.max_steps,
    }
    metadata["comparison_input_sha256"] = hashlib.sha256(
        json.dumps(comparison_inputs, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    _write_json(output_dir / "metadata.json", metadata)

    artifact_by_id = dict(zip(scenario_ids, artifacts, strict=True))
    trials: list[dict[str, Any]] = []
    schedule = trial_schedule(scenario_ids, args.repetitions, args.warmup_count)
    for run_number, (scenario_id, planner_id, trial_index, is_warmup) in enumerate(schedule, 1):
        suffix = f"warmup_{trial_index:03d}" if is_warmup else f"trial_{trial_index:03d}"
        trial_path = output_dir / "trials" / (
            f"{scenario_id}__{planner_id}__{suffix}.json"
        )
        log_path = output_dir / "logs" / (
            f"{scenario_id}__{planner_id}__{suffix}.log"
        )
        if args.mode == "planner-only":
            trial = execute_planner_only_trial(
                planner_id=planner_id,
                scenario_id=scenario_id,
                artifact=artifact_by_id[scenario_id],
                trial_index=trial_index,
                is_warmup=is_warmup,
                executable=executable,
                trial_path=trial_path,
                log_path=log_path,
                max_steps=args.max_steps,
                planner_timeout=args.planner_timeout,
            )
        else:
            trial = execute_trial(
                planner_id=planner_id,
                scenario_id=scenario_id,
                artifact=artifact_by_id[scenario_id],
                trial_index=trial_index,
                is_warmup=is_warmup,
                catalog_path=catalog_path,
                trial_path=trial_path,
                log_path=log_path,
                max_steps=args.max_steps,
                accept_timeout=args.accept_timeout,
                socket_timeout=args.socket_timeout,
            )
        trial["metadata"] = metadata
        _write_json(trial_path, trial)
        trials.append(trial)
        print(
            f"[{run_number}/{len(schedule)}] {scenario_id} {planner_id} "
            f"{'warm-up' if is_warmup else f'trial {trial_index}'}: "
            f"{trial['trial_status']}"
            + (
                f" planner_ms={trial.get('planner_only_latency_ms')}"
                if args.mode == "planner-only"
                else f" simulated_s={trial.get('simulated_duration_seconds')} "
                     f"wall_s={trial.get('episode_wall_clock_seconds')} "
                     f"speed={trial.get('simulation_speed_factor')}"
            )
        )

    measured = [trial for trial in trials if not trial["is_warmup"]]
    scenario_aggregates = aggregate_trials(measured)
    planner_aggregates = aggregate_trials(
        [{**trial, "scenario_id": "all"} for trial in measured]
    )
    _write_json(output_dir / "raw_trials.json", trials)
    _write_csv(output_dir / "individual_trials.csv", measured, TRIAL_FIELDS)
    aggregate_fields = list(scenario_aggregates[0]) if scenario_aggregates else []
    _write_csv(output_dir / "scenario_aggregates.csv", scenario_aggregates, aggregate_fields)
    _write_csv(output_dir / "planner_aggregates.csv", planner_aggregates, aggregate_fields)
    generate_report(output_dir, metadata, scenario_aggregates, planner_aggregates)
    if args.plots:
        _write_plots(output_dir, scenario_aggregates)
    print(f"Benchmark artifacts: {output_dir}")
    return output_dir


def _write_plots(output_dir: Path, aggregates: list[dict[str, Any]]) -> None:
    import matplotlib.pyplot as plt

    metrics = (
        ("success_rate", "Success rate", "success_rate.png"),
        ("planning_latency_median_ms", "Median initial planning latency (ms)", "planning_latency.png"),
        ("path_length_median", "Median initial path length (m)", "path_length.png"),
    )
    scenario_ids = sorted({row["scenario_id"] for row in aggregates})
    for field, title, filename in metrics:
        figure, axis = plt.subplots(figsize=(max(8, len(scenario_ids) * 1.4), 5))
        width = 0.8 / len(PLANNERS)
        for planner_index, planner in enumerate(PLANNERS):
            values = []
            for scenario_id in scenario_ids:
                match = next(
                    (row for row in aggregates
                     if row["planner_id"] == planner and row["scenario_id"] == scenario_id),
                    None,
                )
                values.append(
                    match.get(field) if match is not None else None
                )
            positions = [index + planner_index * width for index in range(len(scenario_ids))]
            axis.bar(
                positions,
                [float("nan") if value is None else value for value in values],
                width,
                label=planner,
            )
        axis.set_title(title)
        axis.set_xticks(
            [index + width * (len(PLANNERS) - 1) / 2 for index in range(len(scenario_ids))],
            scenario_ids,
            rotation=30,
            ha="right",
        )
        axis.legend()
        figure.tight_layout()
        figure.savefig(output_dir / filename, dpi=140)
        plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-suite", choices=("benchmark", "smoke", "all"), default="benchmark")
    parser.add_argument("--scenario-artifact", type=Path, nargs="+", default=None)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--warmup-count", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--accept-timeout", type=float, default=120.0)
    parser.add_argument("--socket-timeout", type=float, default=30.0)
    parser.add_argument(
        "--mode",
        choices=("accelerated-closed-loop", "planner-only"),
        default="accelerated-closed-loop",
        help="Run the full non-rendered simulation loop or invoke only the planner.",
    )
    parser.add_argument(
        "--planner-timeout",
        type=float,
        default=10.0,
        help="Maximum wall-clock seconds allowed for one planner-only process.",
    )
    parser.add_argument("--build-dir", type=Path, default=PROJECT_ROOT / "build")
    parser.add_argument("--output-root", type=Path, default=PROJECT_ROOT / "artifacts" / "benchmarks")
    parser.add_argument("--plots", action="store_true")
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("--repetitions must be at least 1")
    if args.warmup_count < 0:
        parser.error("--warmup-count cannot be negative")
    if args.max_steps is not None and args.max_steps < 1:
        parser.error("--max-steps must be positive")
    if args.accept_timeout <= 0 or args.socket_timeout <= 0 or args.planner_timeout <= 0:
        parser.error("timeouts must be positive")
    return args


def main() -> int:
    output_dir = run_benchmark(parse_args())
    print(f"Report: {output_dir / 'benchmark_report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
