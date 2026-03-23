"""Reusable demo runner for parking controller experiments."""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
from pathlib import Path
import sys
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np

try:
    # Package mode: python -m KarlsPlayground.demo_runner
    from .control import AdvancedParkingController
    from .simulation import AudiA5, ParkingSimulation
except ImportError:
    # Script mode: python KarlsPlayground/demo_runner.py
    workspace_root = Path(__file__).resolve().parent.parent
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))
    from KarlsPlayground.control import AdvancedParkingController
    from KarlsPlayground.simulation import AudiA5, ParkingSimulation

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class DemoConfig:
    """Runtime settings for the parking demo."""

    episodes: int = 12
    planner_resolution: float = 0.25
    lookahead_dist: float = 1.2
    max_speed: float = 1.8
    speed_factor: float = 5.0
    max_episode_steps: int = 900


def _score_episode(info: Dict[str, Any], steps: int, pos_error: float, heading_error: float) -> float:
    """Binary score: 1 if parked in goal slot, else 0."""
    return 1.0 if info.get("goal_reached", False) else 0.0


def _direction_changes(path: Sequence[Any]) -> int:
    return sum(1 for i in range(1, len(path)) if path[i].direction != path[i - 1].direction)


def _maneuver_blocks(path: Sequence[Any]) -> int:
    if not path:
        return 0
    return _direction_changes(path) + 1


def _max_path_curvature(path: Sequence[Any]) -> float:
    if len(path) < 3:
        return 0.0
    max_kappa = 0.0
    for i in range(1, len(path) - 1):
        x0, y0 = float(path[i - 1].x), float(path[i - 1].y)
        x1, y1 = float(path[i].x), float(path[i].y)
        x2, y2 = float(path[i + 1].x), float(path[i + 1].y)
        a1 = np.arctan2(y1 - y0, x1 - x0)
        a2 = np.arctan2(y2 - y1, x2 - x1)
        ds = np.hypot(x2 - x1, y2 - y1)
        if ds > 1e-6:
            kappa = abs(np.arctan2(np.sin(a2 - a1), np.cos(a2 - a1))) / ds
            max_kappa = max(max_kappa, float(kappa))
    return max_kappa


def _episode_banner(index: int, total: int) -> None:
    print(f"\nEpisode {index}/{total}")
    print("  Testing random parking slot...")


def _setup_logging() -> None:
    logging.basicConfig(
        filename="parking_demo.log",
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(name)s - %(message)s",
    )


def run_demo(config: DemoConfig | None = None) -> List[float]:
    """Run controller demo episodes and return per-episode scores."""
    config = config or DemoConfig()
    _setup_logging()

    print("=" * 70)
    print("KarlsPlayground - Path Planner & Controller Test")
    print("=" * 70)

    vehicle = AudiA5()
    simulation = ParkingSimulation(vehicle=vehicle, render=True, speed_factor=config.speed_factor)
    simulation.max_episode_steps = config.max_episode_steps
    controller = AdvancedParkingController(
        planner_resolution=config.planner_resolution,
        lookahead_dist=config.lookahead_dist,
        max_speed=config.max_speed,
    )

    scores: List[float] = []
    goal_slots = list(range(config.episodes))
    for episode, goal_slot in enumerate(goal_slots, start=1):
        _episode_banner(episode, config.episodes)
        state = simulation.reset(goal_slot_index=goal_slot)
        print(f"  Parking in slot {goal_slot}...")

        total_steps = 0
        goal_xy = tuple(state["goal_position"])
        x_bounds = (0.0, simulation.scenario.lot_width if simulation.scenario else simulation.parking_lot.width)
        y_bounds = (0.0, simulation.scenario.lot_depth if simulation.scenario else simulation.parking_lot.depth)
        static_obstacles: List[Dict[str, Any]] = []

        path = controller.plan(
            tuple(state["position"]),
            goal_xy,
            x_bounds,
            y_bounds,
            static_obstacles,
            start_direction=None,
            start_heading=float(state["heading"]),
            goal_heading=float(state["goal_heading"]),
            vehicle_size=(vehicle.length, vehicle.width),
        )
        waypoints = [(point.x, point.y, point.direction) for point in path]

        LOGGER.info(
            "episode=%s goal=%s start=%s waypoints=%s",
            episode,
            goal_xy,
            tuple(state["position"]),
            json.dumps([{"x": float(p.x), "y": float(p.y), "dir": int(p.direction)} for p in path]),
        )

        print(f"  Path length: {len(path)} waypoints")
        forward_count = sum(1 for point in path if point.direction > 0)
        reverse_count = sum(1 for point in path if point.direction < 0)
        print(f"  Directions: {forward_count} forward, {reverse_count} reverse")
        print(
            f"  Maneuver blocks: {_maneuver_blocks(path)} | "
            f"Max curvature: {_max_path_curvature(path):.3f} 1/m"
        )

        while True:
            acceleration, steering_angle = controller.compute_control(
                state,
                goal_heading=state["goal_heading"],
                dynamic_obstacles=[],
            )

            if total_steps % 50 == 0:
                print(
                    f"    Step {total_steps}: accel={acceleration:.2f}, "
                    f"steer={steering_angle:.2f}, vel={state['velocity'][0]:.2f}"
                )

            spline_path = controller.get_spline_path()
            spline_xy = [(point[0], point[1]) for point in spline_path] if spline_path else None
            state, info = simulation.step(
                acceleration,
                steering_angle,
                waypoints,
                planned_path=spline_xy,
                planned_path_width=vehicle.width,
            )
            total_steps += 1

            if info["done"] or controller.is_done(state):
                if info.get("goal_reached", False) or controller.is_done(state):
                    print(f"  Goal reached in {total_steps} steps")
                else:
                    print(f"  Episode ended after {total_steps} steps (timeout/collision)")
                break

        score = _score_episode(
            info,
            total_steps,
            info["position_error"],
            info["heading_error"],
        )
        scores.append(score)
        print(f"  Grade: {int(score)}/1")

    avg_score = float(np.mean(scores)) if scores else 0.0
    print(f"\n{'=' * 70}")
    print(f"Test Complete - Average Score: {avg_score:.3f}/1")
    print(f"Individual Scores: {', '.join(str(int(score)) for score in scores)}")
    print(f"{'=' * 70}")

    simulation.close()
    return scores


def main() -> int:
    """CLI entrypoint."""
    run_demo()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
