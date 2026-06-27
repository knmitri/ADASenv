"""Generate a parking scenario artifact once and store it locally."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_ROOT / "artifacts" / "scenarios" / "single_side_empty_parking_lot.json"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from KarlsPlayground.config_loader import get_config
from KarlsPlayground.scenarios import ParkingScenarioFactory, save_scenario


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--goal-slot", type=int, default=0)
    parser.add_argument("--occupancy", type=float, default=None)
    parser.add_argument("--seed", type=int, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = get_config()
    scenario_config = dict(config.get("scenario", {}))
    if args.occupancy is not None:
        scenario_config["occupancy"] = args.occupancy
    if args.seed is not None:
        scenario_config["seed"] = args.seed

    factory = ParkingScenarioFactory(
        parking_lot_config=config["parking_lot"],
        occupancy=float(scenario_config.get("occupancy", 0.5)),
        seed=int(scenario_config.get("seed", 42)),
    )
    scenario = factory.create(goal_slot_index=args.goal_slot)
    output_path = save_scenario(scenario, args.output.resolve())

    print(f"Saved scenario artifact: {output_path}")
    print(f"Scenario id: {scenario.scenario_id}")
    print(f"Goal slot: {scenario.goal_slot_index}")
    print(f"Static objects: {len(scenario.parked_cars)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
