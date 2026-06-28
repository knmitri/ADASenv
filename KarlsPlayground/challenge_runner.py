"""Run the stored parking scenario against every slot in the lot."""

from __future__ import annotations

import argparse
from dataclasses import replace
import importlib
from pathlib import Path
import time
from typing import Any

import numpy as np

from KarlsPlayground.config_loader import get_config
from KarlsPlayground.planning import BaselineParkingPlanner, ParkingPlanner
from KarlsPlayground.simulation.scenarios import load_parking_scenario
from KarlsPlayground.simulation.scenarios.parking import ParkingScenario
from KarlsPlayground.simulation import AudiA5, ParkingSimulationEnvironment, SimulatorController
from KarlsPlayground.simulation.visualization import SimulationVisualizer


DEFAULT_SCENARIO_ARTIFACT = (
    Path(__file__).resolve().parents[1] / "artifacts" / "scenarios" / "single_side_empty_parking_lot.json"
)


def scenario_for_slot(base: ParkingScenario, slot_index: int) -> ParkingScenario:
    """Return the same scenario layout with a different goal slot."""

    return replace(
        base,
        scenario_id=f"{base.scenario_id}_slot_{slot_index}",
        goal_slot_index=slot_index,
    )


def load_planner_class(spec: str):
    """Load a planner class from `module:ClassName`."""

    module_name, sep, class_name = spec.partition(":")
    if not sep:
        raise ValueError("Planner spec must use the form 'module:ClassName'")
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


def run_slot(
    config: dict[str, Any],
    scenario: ParkingScenario,
    log_interval: int,
    max_expansions: int,
    steer_count: int,
    live_render: bool,
    render_every: int,
    save_render_dir: Path | None,
    planner_class: type,
) -> dict[str, Any]:
    vehicle = AudiA5()
    sim = ParkingSimulationEnvironment(vehicle=vehicle, config=config, scenario=scenario)
    sim.render_enabled = False
    state = sim.reset()
    visualizer = SimulationVisualizer(config.get("visualization")) if live_render or save_render_dir else None

    planner: ParkingPlanner = planner_class(
        planner_resolution=config["planner"]["xy_resolution"],
        max_speed=config["planner"]["max_speed"],
        max_reverse_speed=config["planner"]["max_reverse_speed"],
        two_stage_parking=config["planner"]["two_stage_parking"],
        vehicle=vehicle,
        x_bounds=(sim.world_min_x, scenario.lot.lot_width),
        y_bounds=(sim.world_min_y, scenario.lot.lot_length),
        max_expansions=max_expansions,
        steer_count=steer_count,
    )
    simulator_controller = SimulatorController.from_vehicle(vehicle)

    goal_pose = sim.get_goal_pose()
    plan_start = time.perf_counter()
    path = planner.plan(
        start=tuple(state["position"]),
        start_heading=float(state["heading"]),
        goal=goal_pose,
        static_obstacles=sim.get_static_obstacles(),
        start_direction=None,
    )
    plan_seconds = time.perf_counter() - plan_start
    sim.planned_path = [(point.x, point.y) for point in path]

    info: dict[str, Any] = {}
    total_steps = 0
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

        requested = planner.compute_motion_command(
            state,
            goal_heading=goal_pose[2],
            dynamic_obstacles=[],
        )
        actuator = simulator_controller.apply_limits(requested)
        acceleration, steering = actuator.as_step_args()

        if log_interval > 0 and total_steps % log_interval == 0:
            print(
                f"    step={total_steps:4d} "
                f"accel={acceleration: .2f} steer={steering: .2f} "
                f"vel={float(state['velocity'][0]): .2f}"
            )

        state, info = sim.step(acceleration, steering)
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
        status = "pass" if info.get("goal_reached", False) else "fail"
        render_path = save_render_dir / f"slot_{scenario.goal_slot_index}_{status}.png"
        if visualizer.fig is not None:
            visualizer.fig.savefig(render_path, dpi=150)

    final_position = state["position"]
    final_heading = float(state["heading"])
    if visualizer is not None:
        visualizer.close()
    sim.close()

    return {
        "slot": scenario.goal_slot_index,
        "success": bool(info.get("goal_reached", False)),
        "steps": total_steps,
        "path_len": len(path),
        "plan_seconds": plan_seconds,
        "forward": sum(1 for point in path if point.direction > 0),
        "reverse": sum(1 for point in path if point.direction < 0),
        "position_error": float(info.get("position_error", np.nan)),
        "heading_error_deg": float(np.rad2deg(abs(info.get("heading_error", np.nan)))),
        "collision_occurred": bool(info.get("collision_occurred", False)),
        "final_x": float(final_position[0]),
        "final_y": float(final_position[1]),
        "final_heading_deg": float(np.rad2deg(final_heading)),
        "render_path": str(render_path) if render_path is not None else "",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-artifact", type=Path, default=DEFAULT_SCENARIO_ARTIFACT)
    parser.add_argument("--slots", type=int, nargs="*", default=None)
    parser.add_argument("--log-interval", type=int, default=0)
    parser.add_argument("--max-expansions", type=int, default=200)
    parser.add_argument("--steer-count", type=int, default=5)
    parser.add_argument(
        "--planner",
        default="KarlsPlayground.planning:BaselineParkingPlanner",
        help="Planner class as 'module:ClassName'.",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Show live matplotlib rendering while running each slot.",
    )
    parser.add_argument(
        "--render-every",
        type=int,
        default=5,
        help="Render every N simulation steps when --render is enabled.",
    )
    parser.add_argument(
        "--save-render-dir",
        type=Path,
        default=None,
        help="Save a final rendered PNG per slot into this directory.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base_scenario = load_parking_scenario(args.scenario_artifact.resolve())
    config = get_config()
    config["simulation"] = dict(config["simulation"])
    config["simulation"]["render"] = False
    planner_class = load_planner_class(args.planner)
    vehicle_for_fit = AudiA5()
    length_margin = base_scenario.lot.slot_length - vehicle_for_fit.length
    width_margin = base_scenario.lot.slot_width - vehicle_for_fit.width

    slots = args.slots if args.slots is not None else list(range(base_scenario.lot.num_slots))
    invalid = [slot for slot in slots if slot < 0 or slot >= base_scenario.lot.num_slots]
    if invalid:
        raise ValueError(f"Invalid slot indices {invalid}; valid range is 0..{base_scenario.lot.num_slots - 1}")

    print(f"Scenario artifact: {args.scenario_artifact.resolve()}", flush=True)
    print(f"Testing slots: {', '.join(str(slot) for slot in slots)}", flush=True)
    print(
        f"Planner budget: max_expansions={args.max_expansions}, steer_count={args.steer_count}",
        flush=True,
    )
    print(f"Planner: {args.planner}", flush=True)
    print(
        f"Physical fit: slot={base_scenario.lot.slot_length:.2f}m x "
        f"{base_scenario.lot.slot_width:.2f}m, vehicle={vehicle_for_fit.length:.2f}m x "
        f"{vehicle_for_fit.width:.2f}m, margins=length {length_margin:.2f}m / "
        f"width {width_margin:.2f}m",
        flush=True,
    )
    print(flush=True)

    results = []
    for slot in slots:
        scenario = scenario_for_slot(base_scenario, slot)
        print(f"Slot {slot}:", flush=True)
        result = run_slot(
            config,
            scenario,
            log_interval=args.log_interval,
            max_expansions=args.max_expansions,
            steer_count=args.steer_count,
            live_render=args.render,
            render_every=max(1, args.render_every),
            save_render_dir=args.save_render_dir,
            planner_class=planner_class,
        )
        results.append(result)
        status = "PASS" if result["success"] else "FAIL"
        print(
            f"  {status} steps={result['steps']} path={result['path_len']} "
            f"plan={result['plan_seconds']:.1f}s "
            f"dir=+{result['forward']}/-{result['reverse']} "
            f"pos_err={result['position_error']:.3f}m "
            f"heading_err={result['heading_error_deg']:.2f}deg "
            f"collision={result['collision_occurred']}",
            flush=True,
        )
        print(
            f"  final_pose=({result['final_x']:.2f}, {result['final_y']:.2f}, "
            f"{result['final_heading_deg']:.1f}deg)",
            flush=True,
        )
        if result["render_path"]:
            print(f"  render={result['render_path']}", flush=True)

    passed = sum(1 for result in results if result["success"])
    print(flush=True)
    print(f"Summary: {passed}/{len(results)} slots passed", flush=True)
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
