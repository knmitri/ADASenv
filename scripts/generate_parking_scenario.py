"""Generate a parking scenario artifact once and store it locally."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

from _project import PROJECT_ROOT, add_project_root_to_path

DEFAULT_OUTPUT = PROJECT_ROOT / "artifacts" / "scenarios" / "single_side_empty_parking_lot.json"
add_project_root_to_path()

from simulator.config_loader import get_config
from simulator.simulation.scenarios import ParkingScenarioFactory, save_scenario


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--goal-slot", type=int, default=0)
    parser.add_argument("--occupancy", type=float, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--rows", type=int, default=None)
    parser.add_argument("--cols", type=int, default=None)
    parser.add_argument("--slot-width", type=float, default=None)
    parser.add_argument("--slot-length", type=float, default=None)
    parser.add_argument("--slot-orientation-deg", type=float, default=None)
    parser.add_argument("--scenario-id", type=str, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = get_config()
    parking_lot_config = dict(config["parking_lot"])
    scenario_config = dict(config.get("scenario", {}))
    if args.occupancy is not None:
        scenario_config["occupancy"] = args.occupancy
    if args.seed is not None:
        scenario_config["seed"] = args.seed
    if args.rows is not None:
        parking_lot_config["rows"] = args.rows
    if args.cols is not None:
        parking_lot_config["cols"] = args.cols
    if args.slot_width is not None:
        parking_lot_config["slot_width"] = args.slot_width
    if args.slot_length is not None:
        parking_lot_config["slot_length"] = args.slot_length
    if args.slot_orientation_deg is not None:
        parking_lot_config["slot_orientation_deg"] = args.slot_orientation_deg

    factory = ParkingScenarioFactory(
        parking_lot_config=parking_lot_config,
        occupancy=float(scenario_config.get("occupancy", 0.5)),
        seed=int(scenario_config.get("seed", 42)),
    )
    scenario = factory.create(goal_slot_index=args.goal_slot)
    if args.scenario_id is not None:
        scenario = replace(scenario, scenario_id=args.scenario_id)
    output_path = save_scenario(scenario, args.output.resolve())

    print(f"Saved scenario artifact: {output_path}")
    print(f"Scenario id: {scenario.scenario_id}")
    print(f"Goal slot: {scenario.goal_slot_index}")
    print(f"Static objects: {len(scenario.parked_cars)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
