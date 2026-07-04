"""Shared challenge scenario helpers owned by the simulation package."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

from simulator.simulation.parking_lot import ParkingSlot
from simulator.simulation.scenarios.parking import ParkingScenario


SCENARIO_DIR = Path(__file__).resolve().parents[2] / "artifacts" / "scenarios"
DEFAULT_SCENARIO_ARTIFACTS = (
    SCENARIO_DIR / "single_side_empty_parking_lot.json",
    SCENARIO_DIR / "single_side_empty_parallel_parking_lot.json",
    SCENARIO_DIR / "single_side_obstacle_parking_lot.json",
    SCENARIO_DIR / "single_side_obstacle_parallel_parking_lot.json",
)


def scenario_for_slot(base: ParkingScenario, slot_index: int) -> ParkingScenario:
    """Return the same scenario layout with a different goal slot."""

    return replace(
        base,
        scenario_id=f"{base.scenario_id}_slot_{slot_index}",
        goal_slot_index=slot_index,
    )


def default_goal_slots(scenario: ParkingScenario) -> list[int]:
    """Return slots that can be used as goals without overlapping parked cars."""

    occupied = {car.slot_index for car in scenario.parked_cars if car.slot_index is not None}
    return [slot for slot in range(scenario.lot.num_slots) if slot not in occupied]


def slot_as_dict(index: int, slot: ParkingSlot) -> dict[str, Any]:
    """Return slot geometry in a format suitable for remote ADAS stacks."""

    return {
        "index": int(index),
        "x": float(slot.x),
        "y": float(slot.y),
        "heading": float(slot.heading),
        "width": float(slot.width),
        "length": float(slot.length),
        "orientation": slot.orientation,
        "corners": slot.get_corners().tolist(),
    }


def scenario_slots_as_dict(scenario: ParkingScenario) -> list[dict[str, Any]]:
    """Return all parking-slot geometries for a scenario."""

    return [slot_as_dict(index, slot) for index, slot in enumerate(scenario.lot.get_all_slots())]
