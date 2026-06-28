"""
Reusable demo runner for parking planner experiments.
This script sets up and runs a series of parking planner demo episodes,
logging results and printing summary statistics for each run.
"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
import sys
from typing import Any, Dict, List
import numpy as np


# Import planner and simulation classes, supporting both package and script execution modes
try:
    # Package mode: python -m KarlsPlayground.demo_runner
    from .config_loader import get_config
    from .planning import BaselineParkingPlanner
    from .simulation.scenarios import load_parking_scenario
    from .simulation import AudiA5, ParkingSimulationEnvironment, SimulatorController
except ImportError:
    # Script mode: python KarlsPlayground/demo_runner.py
    workspace_root = Path(__file__).resolve().parent.parent
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))
    from KarlsPlayground.config_loader import get_config
    from KarlsPlayground.planning import BaselineParkingPlanner
    from KarlsPlayground.simulation.scenarios import load_parking_scenario
    from KarlsPlayground.simulation import ParkingSimulationEnvironment, AudiA5, SimulatorController

LOGGER = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENARIO_ARTIFACT = PROJECT_ROOT / "artifacts" / "scenarios" / "single_side_empty_parking_lot.json"

def _setup_logging() -> None:
    """Configure logging to file for demo runs."""
    logging.basicConfig(
        filename="parking_demo.log",
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(name)s - %(message)s",
    )


def run_demo(config, scenario_artifact: Path = DEFAULT_SCENARIO_ARTIFACT) -> List[float]:
    """
    Run planner demo episodes and return per-episode scores.
    Sets up the simulation, planner, and runs multiple episodes, printing and logging results.
    """
    _setup_logging()
    scores: List[float] = []

    print("=" * 70)
    print("KarlsPlayground - Path Planner & Controller Test")
    print("=" * 70)

    if not scenario_artifact.exists():
        raise FileNotFoundError(
            f"Scenario artifact not found: {scenario_artifact}\n"
            "Generate it once with:\n"
            "  python scripts\\generate_parking_scenario.py"
        )

    scenario = load_parking_scenario(scenario_artifact)
    print(f"Loaded scenario artifact: {scenario_artifact}")
    print(f"Scenario id: {scenario.scenario_id}")

    # Instantiate vehicle and simulation environment
    vehicle = AudiA5()
    sim = ParkingSimulationEnvironment(vehicle=vehicle, config=config, scenario=scenario)
    
    x_bounds = (sim.world_min_x, sim.scenario.lot.lot_width)
    y_bounds = (sim.world_min_y, sim.scenario.lot.lot_length) 

    # Create the reference parking planner
    planner = BaselineParkingPlanner(
        planner_resolution=config['planner']['xy_resolution'],
        max_speed=config['planner']['max_speed'],
        max_reverse_speed=config['planner']['max_reverse_speed'],
        two_stage_parking=config['planner']['two_stage_parking'],
        vehicle=vehicle,
        x_bounds=x_bounds,
        y_bounds=y_bounds,
        max_expansions=config['planner'].get('max_expansions', 2000),
        steer_count=config['planner'].get('steer_count', 9),
    )
    simulator_controller = SimulatorController.from_vehicle(vehicle)
    
    goal_slots = [scenario.goal_slot_index]
    for episode, goal_slot_index in enumerate(goal_slots, start=1):
        """Print a banner for the current episode."""
        print(f"\nEpisode {episode}/{len(goal_slots)}")
        print("  Running stored parking scenario...")

        state = sim.reset()
        goal_slot = sim.scenario.goal_slot

        print(f"  Parking in slot {goal_slot_index}...")

        total_steps = 0
        # Get goal and bounds for planning
        goal_xyh = sim.get_goal_pose()

        static_obstacles: List[Dict[str, Any]] = sim.get_static_obstacles()

        # Plan path to goal slot
        path = planner.plan(
            tuple(state["position"]),
            start_heading=float(state["heading"]),
            goal=goal_xyh,
            static_obstacles=static_obstacles,
            start_direction=None,
        )
        sim.planned_path = [(point.x, point.y) for point in path]

        # Log episode details
        LOGGER.info(
            "episode=%s goal=%s start=%s path=%s",
            episode,
            goal_xyh,
            tuple(state["position"]),
            json.dumps([{"x": float(p.x), "y": float(p.y), "dir": int(p.direction)} for p in path]),
        )

        print(f"  Path length: {len(path)} waypoints")
    
        forward_count = sum(1 for point in path if point.direction > 0)
        reverse_count = sum(1 for point in path if point.direction < 0)

        print(f"  Directions: {forward_count} forward, {reverse_count} reverse")

        # Step through the episode until done
        while True:
            requested_command = planner.compute_motion_command(
                state,
                goal_heading=goal_xyh[2],
                dynamic_obstacles=[],
            )
            actuator_command = simulator_controller.apply_limits(requested_command)
            acceleration, steering_angle = actuator_command.as_step_args()

            # Print status every 50 steps
            if total_steps % 50 == 0:
                print(
                    f"    Step {total_steps}: long_accel={acceleration:.2f}, "
                    f"front_mid_steer={steering_angle:.2f}, vel={state['velocity'][0]:.2f}"
                )

            state, info = sim.step(
                acceleration,
                steering_angle,
            )
            total_steps += 1

            # Check for episode completion
            if info["done"]:
                if info.get("goal_reached", False):
                    print(f"  Goal reached in {total_steps} steps")
                else:
                    print(f"  Episode ended after {total_steps} steps (timeout/collision)")
                break

        # Score and print result for this episode
        score= 1.0 if info.get("goal_reached", False) else 0.0
        
        scores.append(score)
        print(f"  Grade: {int(score)}/1")

    # Print summary statistics
    avg_score = float(np.mean(scores)) if scores else 0.0
    print(f"\n{'=' * 70}")
    print(f"Test Complete - Average Score: {avg_score:.3f}/1")
    print(f"Individual Scores: {', '.join(str(int(score)) for score in scores)}")
    print(f"{'=' * 70}")

    sim.close()
    return scores



def main() -> int:
    """
    CLI entrypoint for running the demo from the command line.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario-artifact",
        type=Path,
        default=DEFAULT_SCENARIO_ARTIFACT,
        help="Path to a generated parking scenario artifact.",
    )
    args = parser.parse_args()

    config = get_config()
    run_demo(config, scenario_artifact=args.scenario_artifact.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
