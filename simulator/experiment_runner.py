"""Run scenario-catalog experiments against an external C++ ADAS stack over TCP."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import socket
import subprocess
import time
from typing import Any

import numpy as np

from simulator.adas_stack_catalog import (
    DEFAULT_ADAS_STACK_CATALOG,
    AdasStackSpec,
    get_adas_stack,
    load_adas_stack_catalog,
)
from simulator.config_loader import get_config
from simulator.simulation import AudiA5, MotionCommand, ParkingSimulationEnvironment, SimulatorController
from simulator.simulation.challenge import (
    default_goal_slots,
    scenario_artifacts_for_suite,
    scenario_for_slot,
    slot_as_dict,
    scenario_slots_as_dict,
)
from simulator.simulation.scenarios import load_parking_scenario
from simulator.simulation.scenarios.parking import ParkingScenario
from simulator.simulation.visualization import SimulationVisualizer


PROTOCOL_VERSION = 1


class JsonLineConnection:
    """Line-delimited JSON connection for simulator-to-ADAS communication."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        reader = None
        writer = None
        try:
            reader = sock.makefile("r", encoding="utf-8", newline="\n")
            writer = sock.makefile("w", encoding="utf-8", newline="\n")
        except BaseException:
            try:
                if writer is not None:
                    writer.close()
            finally:
                try:
                    if reader is not None:
                        reader.close()
                finally:
                    sock.close()
            raise
        self.reader = reader
        self.writer = writer

    def send(self, payload: dict[str, Any]) -> None:
        self.writer.write(json.dumps(to_jsonable(payload), separators=(",", ":")) + "\n")
        self.writer.flush()

    def receive(self) -> dict[str, Any]:
        line = self.reader.readline()
        if not line:
            raise ConnectionError("Remote ADAS stack closed the TCP connection.")
        payload = json.loads(line)
        if payload.get("type") == "error":
            raise RuntimeError(f"Remote ADAS stack returned error: {payload.get('error', payload)}")
        return payload

    def close(self) -> None:
        try:
            self.reader.close()
        finally:
            try:
                self.writer.close()
            finally:
                self.sock.close()


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return to_jsonable(value.tolist())
    if hasattr(value, "item"):
        return value.item()
    return value


class ExperimentTraceLogger:
    """Write compact, replay-friendly simulator/ADAS exchange traces."""

    def __init__(self, directory: Path, scenario: ParkingScenario):
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = directory / (
            f"{scenario.scenario_id}_slot_{scenario.goal_slot_index}_{stamp}.jsonl"
        )
        self._file = self.path.open("w", encoding="utf-8")

    def write(self, event: str, **data: Any) -> None:
        self._file.write(json.dumps({"event": event, **to_jsonable(data)}, separators=(",", ":")) + "\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def vehicle_as_dict(vehicle: AudiA5) -> dict[str, float]:
    return {
        "length": float(vehicle.length),
        "width": float(vehicle.width),
        "wheelbase": float(vehicle.wheelbase),
        "max_steering_angle": float(vehicle.MAX_STEERING_ANGLE),
        "max_steering_rate": float(vehicle.MAX_STEERING_RATE),
        "max_acceleration": float(vehicle.MAX_ACCELERATION),
        "max_deceleration": float(vehicle.MAX_DECELERATION),
        "max_speed": float(vehicle.MAX_SPEED),
        "max_reverse_speed": float(vehicle.MAX_REVERSE_SPEED),
        "front_overhang": float(getattr(vehicle, "_FRONT_OVERHANG", 0.0)),
        "rear_overhang": float(getattr(vehicle, "_REAR_OVERHANG", 0.0)),
    }


def path_from_response(response: dict[str, Any]) -> list[tuple[float, float]]:
    raw_path = response.get("planned_path", response.get("path", []))
    path: list[tuple[float, float]] = []
    for point in raw_path:
        if isinstance(point, dict):
            path.append((float(point["x"]), float(point["y"])))
        else:
            path.append((float(point[0]), float(point[1])))
    return path


def run_remote_slot(
    connection: JsonLineConnection,
    config: dict[str, Any],
    scenario: ParkingScenario,
    log_interval: int,
    live_render: bool,
    render_every: int,
    save_render_dir: Path | None,
    trace_dir: Path | None,
) -> dict[str, Any]:
    cleanup = ExitStack()
    try:
        result = _run_remote_slot(
            connection,
            config,
            scenario,
            log_interval,
            live_render,
            render_every,
            save_render_dir,
            trace_dir,
            cleanup,
        )
    except BaseException as primary_error:
        try:
            cleanup.close()
        except BaseException as cleanup_error:
            raise primary_error from cleanup_error
        raise
    else:
        cleanup.close()
        return result


def _run_remote_slot(
    connection: JsonLineConnection,
    config: dict[str, Any],
    scenario: ParkingScenario,
    log_interval: int,
    live_render: bool,
    render_every: int,
    save_render_dir: Path | None,
    trace_dir: Path | None,
    cleanup: ExitStack,
) -> dict[str, Any]:
    vehicle = AudiA5()
    sim = ParkingSimulationEnvironment(vehicle=vehicle, config=config, scenario=scenario)
    cleanup.callback(sim.close)
    sim.render_enabled = False
    state = sim.reset()
    visualizer = SimulationVisualizer(config.get("visualization")) if live_render or save_render_dir else None
    if visualizer is not None:
        cleanup.callback(visualizer.close)
    simulator_controller = SimulatorController.from_vehicle(vehicle)
    goal_pose = sim.get_goal_pose()

    reset_payload = {
        "type": "reset",
        "protocol_version": PROTOCOL_VERSION,
        "scenario_id": scenario.scenario_id,
        "goal_slot_index": scenario.goal_slot_index,
        "scenario": scenario.as_dict(),
        "slots": scenario_slots_as_dict(scenario),
        "goal_pose": goal_pose,
        "goal": goal_pose,
        "goal_slot": slot_as_dict(scenario.goal_slot_index, scenario.goal_slot),
        "start": tuple(state["position"]),
        "start_heading": float(state["heading"]),
        "initial_state": state,
        "static_obstacles": [
            obstacle
            for obstacle in sim.get_static_obstacles()
            if not obstacle.get("boundary_wall", False)
        ],
        "vehicle": vehicle_as_dict(vehicle),
        "simulation": {
            "dt": float(config["simulation"]["dt"]),
            "max_episode_steps": int(config["simulation"]["max_episode_steps"]),
            "world_min_x": float(sim.world_min_x),
            "world_min_y": float(sim.world_min_y),
            "world_max_x": float(scenario.lot.lot_width),
            "world_max_y": float(scenario.lot.lot_length),
            "replan_position_threshold": 0.5,
            "replan_heading_threshold": 0.35,
            "goal_heading_tolerance": float(config["simulation"]["goal_heading_tolerance"]),
        },
    }
    trace = ExperimentTraceLogger(trace_dir, scenario) if trace_dir is not None else None
    if trace is not None:
        cleanup.callback(trace.close)

    plan_start = time.perf_counter()
    connection.send(reset_payload)
    ready = connection.receive()
    if ready.get("type") not in {"ready", "ack"}:
        raise RuntimeError(f"Expected ready response after reset, got {ready}")
    sim.planned_path = path_from_response(ready)
    if trace is not None:
        trace.write("reset", request=reset_payload, response=ready)
    plan_seconds = time.perf_counter() - plan_start

    info: dict[str, Any] = {}
    total_steps = 0
    path_signature = None
    while True:
        if live_render and visualizer is not None and total_steps % render_every == 0:
            visualizer.render(
                vehicle=vehicle,
                parking_lot=scenario.lot,
                goal_slot_index=scenario.goal_slot_index,
                trajectory=sim.trajectory,
                step_count=sim.step_count,
                planned_path=sim.planned_path,
                goal_pose=goal_pose,
            )

        state_before = state
        connection.send(
            {
                "type": "state",
                "step": total_steps,
                "state": state_before,
                "goal_pose": goal_pose,
                "dynamic_obstacles": sim.get_dynamic_obstacles(),
            }
        )
        response = connection.receive()
        if response.get("type") != "command":
            raise RuntimeError(f"Expected command response, got {response}")
        if "planned_path" in response or "path" in response:
            sim.planned_path = path_from_response(response)
            new_signature = (
                len(sim.planned_path),
                sim.planned_path[0] if sim.planned_path else None,
                sim.planned_path[-1] if sim.planned_path else None,
            )
            if trace is not None and new_signature != path_signature:
                trace.write("trajectory_update", step=total_steps, trajectory=response.get("planned_path", response.get("path", [])))
            path_signature = new_signature

        requested = MotionCommand(
            longitudinal_acceleration_mps2=float(response["longitudinal_acceleration_mps2"]),
            front_mid_axle_steering_angle_rad=float(response["front_mid_axle_steering_angle_rad"]),
        )
        actuator = simulator_controller.apply_limits(requested)
        acceleration, steering = actuator.as_step_args()

        if log_interval > 0 and total_steps % log_interval == 0:
            print(
                f"    step={total_steps:4d} "
                f"accel={acceleration: .2f} steer={steering: .2f} "
                f"vel={float(state['velocity'][0]): .2f}",
                flush=True,
            )

        state, info = sim.step(acceleration, steering)
        if trace is not None:
            trace_response = {
                key: value
                for key, value in response.items()
                if key not in {"planned_path", "path"}
            }
            trace.write(
                "step",
                step=total_steps,
                command={
                    "requested": trace_response,
                    "applied_acceleration_mps2": acceleration,
                    "applied_steering_rad": steering,
                },
                state_before=state_before,
                state_after=state,
                info=info,
            )
        total_steps += 1
        if info["done"]:
            break

    render_path = None
    if save_render_dir is not None and visualizer is not None:
        save_render_dir.mkdir(parents=True, exist_ok=True)
        visualizer.render(
            vehicle=vehicle,
            parking_lot=scenario.lot,
            goal_slot_index=scenario.goal_slot_index,
            trajectory=sim.trajectory,
            step_count=sim.step_count,
            planned_path=sim.planned_path,
            goal_pose=goal_pose,
        )
        status = (
            "pass" if info.get("goal_reached", False)
            else "stuck" if info.get("stuck", False)
            else "fail"
        )
        render_path = save_render_dir / f"{scenario.scenario_id}_slot_{scenario.goal_slot_index}_{status}.png"
        if visualizer.fig is not None:
            visualizer.fig.savefig(render_path, dpi=150)

    final_position = state["position"]
    final_heading = float(state["heading"])

    goal_reached = bool(info.get("goal_reached", False))
    collision_occurred = bool(info.get("collision_occurred", False))
    stuck = bool(info.get("stuck", False))
    result = {
        "slot": scenario.goal_slot_index,
        "success": goal_reached and not collision_occurred,
        "goal_reached": goal_reached,
        "steps": total_steps,
        "path_len": len(sim.planned_path),
        "plan_seconds": plan_seconds,
        "position_error": float(info.get("position_error", np.nan)),
        "heading_error_deg": float(np.rad2deg(abs(info.get("heading_error", np.nan)))),
        "collision_occurred": collision_occurred,
        "stuck": stuck,
        "final_x": float(final_position[0]),
        "final_y": float(final_position[1]),
        "final_heading_deg": float(np.rad2deg(final_heading)),
        "render_path": str(render_path) if render_path is not None else "",
    }
    if trace is not None:
        trace.write("result", result=result, final_info=info)
        result["trace_path"] = str(trace.path)
    connection.send({"type": "result", "result": result, "info": info})
    ack = connection.receive()
    if ack.get("type") != "ack":
        raise RuntimeError(f"Expected result ack, got {ack}")
    return result


def launch_adas_stack(stack: AdasStackSpec, host: str, port: int) -> subprocess.Popen:
    """Start the selected C++ ADAS stack."""

    if not stack.executable.exists():
        raise FileNotFoundError(
            f"ADAS stack executable not found: {stack.executable}\n"
            "Build it first, for example:\n"
            "  cmake -S . -B build\n"
            "  cmake --build build --config Release"
        )
    command = stack.command(host=host, port=port)
    print(f"Launching ADAS stack '{stack.stack_id}': {' '.join(command)}", flush=True)
    return subprocess.Popen(command)


def run_experiment(args: argparse.Namespace) -> int:
    process_holder: list[subprocess.Popen | None] = [None]
    socket_holder: list[socket.socket | None] = [None]
    try:
        return _run_experiment(args, process_holder, socket_holder)
    finally:
        try:
            if socket_holder[0] is not None:
                socket_holder[0].close()
        finally:
            _stop_stack_process(process_holder[0])


def _run_experiment(
    args: argparse.Namespace,
    process_holder: list[subprocess.Popen | None],
    socket_holder: list[socket.socket | None],
) -> int:
    config = get_config()
    config["simulation"] = dict(config["simulation"])
    if args.max_steps is not None:
        config["simulation"]["max_episode_steps"] = args.max_steps

    selected_artifacts = (
        args.scenario_artifact
        if args.scenario_artifact is not None
        else scenario_artifacts_for_suite(args.scenario_suite)
    )
    scenario_paths = [path.resolve() for path in selected_artifacts]
    base_scenarios = [load_parking_scenario(path) for path in scenario_paths]
    vehicle_for_fit = AudiA5()
    with socket.create_server((args.host, args.port), reuse_port=False) as server:
        actual_host, actual_port = server.getsockname()[:2]
        connect_host = args.connect_host or args.host
        if connect_host in ("0.0.0.0", "::"):
            connect_host = "127.0.0.1"

        print(f"Listening for ADAS stack on {actual_host}:{actual_port}", flush=True)
        if not args.wait_for_external_stack:
            stack = get_adas_stack(args.adas_stack, args.adas_stack_catalog)
            process_holder[0] = launch_adas_stack(stack, connect_host, int(actual_port))

        server.settimeout(args.accept_timeout)
        client_socket, address = server.accept()
        socket_holder[0] = client_socket

    connection = None
    try:
        print(f"ADAS stack connected from {address[0]}:{address[1]}", flush=True)
        client_socket.settimeout(args.socket_timeout)
        connection = JsonLineConnection(client_socket)
        connection.send(
            {
                "type": "hello",
                "protocol_version": PROTOCOL_VERSION,
                "role": "python_simulator",
                "command_interface": [
                    "longitudinal_acceleration_mps2",
                    "front_mid_axle_steering_angle_rad",
                ],
            }
        )
        hello = connection.receive()
        if hello.get("type") not in {"hello", "ack"}:
            raise RuntimeError(f"Expected hello response, got {hello}")

        print("Scenario artifacts:", flush=True)
        for path in scenario_paths:
            print(f"  {path}", flush=True)
        print(flush=True)

        results = []
        for base_scenario in base_scenarios:
            length_margin = base_scenario.lot.slot_length - vehicle_for_fit.length
            width_margin = base_scenario.lot.slot_width - vehicle_for_fit.width
            if args.slots is not None:
                slots = args.slots
            elif args.all_empty_slots:
                slots = default_goal_slots(base_scenario)
            else:
                slots = [base_scenario.goal_slot_index]
            invalid = [slot for slot in slots if slot < 0 or slot >= base_scenario.lot.num_slots]
            if invalid:
                raise ValueError(
                    f"Invalid slot indices {invalid} for {base_scenario.scenario_id}; "
                    f"valid range is 0..{base_scenario.lot.num_slots - 1}"
                )

            print(f"Scenario {base_scenario.scenario_id}:", flush=True)
            print(f"  Testing slots: {', '.join(str(slot) for slot in slots)}", flush=True)
            print(
                f"  Physical fit: slot={base_scenario.lot.slot_length:.2f}m x "
                f"{base_scenario.lot.slot_width:.2f}m, vehicle={vehicle_for_fit.length:.2f}m x "
                f"{vehicle_for_fit.width:.2f}m, margins=length {length_margin:.2f}m / "
                f"width {width_margin:.2f}m",
                flush=True,
            )
            for slot in slots:
                scenario = scenario_for_slot(base_scenario, slot)
                print(f"  Slot {slot}:", flush=True)
                result = run_remote_slot(
                    connection=connection,
                    config=config,
                    scenario=scenario,
                    log_interval=args.log_interval,
                    live_render=args.render,
                    render_every=max(1, args.render_every),
                    save_render_dir=args.save_render_dir,
                    trace_dir=args.trace_dir,
                )
                result["scenario_id"] = base_scenario.scenario_id
                results.append(result)
                status = (
                    "PASS" if result["success"]
                    else "STUCK" if result["stuck"]
                    else "FAIL"
                )
                print(
                    f"    {status} steps={result['steps']} path={result['path_len']} "
                    f"plan={result['plan_seconds']:.1f}s "
                    f"pos_err={result['position_error']:.3f}m "
                    f"heading_err={result['heading_error_deg']:.2f}deg "
                    f"collision={result['collision_occurred']} "
                    f"stuck={result['stuck']}",
                    flush=True,
                )
                print(
                    f"    final_pose=({result['final_x']:.2f}, {result['final_y']:.2f}, "
                    f"{result['final_heading_deg']:.1f}deg)",
                    flush=True,
                )
                if result["render_path"]:
                    print(f"    render={result['render_path']}", flush=True)
                if result.get("trace_path"):
                    print(f"    trace={result['trace_path']}", flush=True)
            print(flush=True)

        connection.send({"type": "shutdown"})
        ack = connection.receive()
        if ack.get("type") != "ack":
            raise RuntimeError(f"Expected shutdown ack, got {ack}")

        passed = sum(1 for result in results if result["success"])
        print(flush=True)
        print(f"Summary: {passed}/{len(results)} slots passed", flush=True)
        return 0 if passed == len(results) else 1
    finally:
        if connection is not None:
            connection.close()


def _stop_stack_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return

    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument(
        "--connect-host",
        default=None,
        help="Host passed to launched ADAS stacks. Defaults to --host, with 0.0.0.0 mapped to 127.0.0.1.",
    )
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--accept-timeout", type=float, default=120.0)
    parser.add_argument("--socket-timeout", type=float, default=30.0)
    parser.add_argument("--adas-stack", default="hybrid_astar")
    parser.add_argument("--adas-stack-catalog", type=Path, default=DEFAULT_ADAS_STACK_CATALOG)
    parser.add_argument(
        "--wait-for-external-stack",
        action="store_true",
        help="Do not launch the selected stack; wait for an already-running C++ stack to connect.",
    )
    parser.add_argument(
        "--list-adas-stacks",
        action="store_true",
        help="List configured ADAS stacks and exit.",
    )
    parser.add_argument(
        "--scenario-artifact",
        type=Path,
        nargs="+",
        default=None,
        help="Explicit scenario artifact(s). Overrides --scenario-suite.",
    )
    parser.add_argument(
        "--scenario-suite",
        choices=("smoke", "benchmark", "all"),
        default="smoke",
        help="Built-in scenario suite to run when --scenario-artifact is omitted (default: smoke).",
    )
    parser.add_argument("--slots", type=int, nargs="*", default=None)
    parser.add_argument(
        "--all-empty-slots",
        action="store_true",
        help="Run every unoccupied slot in each selected scenario instead of its declared goal slot.",
    )
    parser.add_argument("--log-interval", type=int, default=0)
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Override the scenario's simulation step limit for a diagnostic run.",
    )
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--render-every", type=int, default=5)
    parser.add_argument("--save-render-dir", type=Path, default=None)
    parser.add_argument(
        "--trace-dir",
        type=Path,
        default=None,
        help="Write JSONL state, command, trajectory, and result traces for each scenario slot.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list_adas_stacks:
        for stack in load_adas_stack_catalog(args.adas_stack_catalog).values():
            print(f"{stack.stack_id}: {stack.name} ({stack.executable})")
        return 0
    return run_experiment(args)


if __name__ == "__main__":
    raise SystemExit(main())
