"""Shared challenge scenario helpers owned by the simulation package."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

from simulator.simulation.parking_lot import ParkingSlot
from simulator.simulation.scenarios.parking import ParkingScenario


SCENARIO_DIR = Path(__file__).resolve().parents[2] / "artifacts" / "scenarios"
SMOKE_SCENARIO_ARTIFACTS = (
    SCENARIO_DIR / "smoke" / "empty_perpendicular.json",
    SCENARIO_DIR / "smoke" / "empty_parallel.json",
    SCENARIO_DIR / "smoke" / "perpendicular_between_two_cars.json",
    SCENARIO_DIR / "smoke" / "parallel_between_two_cars.json",
)
BENCHMARK_SCENARIO_ARTIFACTS = (
    SCENARIO_DIR / "benchmark" / "perpendicular_static_obstacles.json",
    SCENARIO_DIR / "benchmark" / "parallel_static_obstacles.json",
    SCENARIO_DIR / "benchmark" / "parallel_tight_between_two_cars.json",
    SCENARIO_DIR / "benchmark" / "parallel_offset_between_two_cars.json",
    SCENARIO_DIR / "benchmark" / "parallel_end_slot.json",
    SCENARIO_DIR / "benchmark" / "angled_60_between_two_cars.json",
    SCENARIO_DIR / "benchmark" / "perpendicular_bollard_at_entrance.json",
)
DEFAULT_SCENARIO_ARTIFACTS = SMOKE_SCENARIO_ARTIFACTS


def scenario_artifacts_for_suite(suite: str) -> tuple[Path, ...]:
    """Return the artifacts belonging to a named deterministic scenario suite."""

    suites = {
        "smoke": SMOKE_SCENARIO_ARTIFACTS,
        "benchmark": BENCHMARK_SCENARIO_ARTIFACTS,
        "all": SMOKE_SCENARIO_ARTIFACTS + BENCHMARK_SCENARIO_ARTIFACTS,
    }
    try:
        return suites[suite]
    except KeyError as exc:
        raise ValueError(f"Unknown scenario suite {suite!r}. Available: {', '.join(suites)}") from exc


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
