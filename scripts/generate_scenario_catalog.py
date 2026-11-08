"""Generate the deterministic built-in smoke and benchmark scenario catalogs."""

from __future__ import annotations

from pathlib import Path

from _project import PROJECT_ROOT, add_project_root_to_path

add_project_root_to_path()

from simulator.simulation.scenarios import ParkingScenario, ScenarioObject, save_scenario
from simulator.simulation.parking_lot import ParkingLot


SCENARIO_ROOT = PROJECT_ROOT / "artifacts" / "scenarios"
VEHICLE_LENGTH = 4.67
VEHICLE_WIDTH = 1.86


def build_scenario(
    scenario_id: str,
    *,
    orientation_deg: float,
    goal_slot: int,
    parked_slots: tuple[int, ...] = (),
    slot_length: float = 5.0,
    parked_offsets: dict[int, tuple[float, float]] | None = None,
    bollard_at_entrance: bool = False,
) -> ParkingScenario:
    lot = ParkingLot(
        {
            "rows": 6,
            "cols": 1,
            "layout": "single_side",
            "origin_x": 10.0,
            "origin_y": 0.0,
            "slot_width": 2.5,
            "slot_length": slot_length,
            "gap_between_slots": 0.0,
            "aisle_width": 6.0,
            "slot_orientation_deg": orientation_deg,
        }
    )
    offsets = parked_offsets or {}
    objects = [
        ScenarioObject(
            object_id=f"parked_car_{slot_index}",
            object_type="parked_vehicle",
            x=float(lot.get_slot(slot_index).x + offsets.get(slot_index, (0.0, 0.0))[0]),
            y=float(lot.get_slot(slot_index).y + offsets.get(slot_index, (0.0, 0.0))[1]),
            heading=float(lot.get_slot(slot_index).heading),
            length=VEHICLE_LENGTH,
            width=VEHICLE_WIDTH,
            slot_index=slot_index,
        )
        for slot_index in parked_slots
    ]

    if bollard_at_entrance:
        goal = lot.get_slot(goal_slot)
        objects.append(
            ScenarioObject(
                object_id="entrance_bollard",
                object_type="bollard",
                x=float(goal.x - lot.slot_length / 2.0 - 0.5),
                y=float(goal.y),
                heading=0.0,
                length=0.3,
                width=0.3,
            )
        )

    return ParkingScenario(
        scenario_id=scenario_id,
        lot=lot,
        goal_slot_index=goal_slot,
        parked_cars=tuple(objects),
    )


def main() -> None:
    catalog = (
        ("smoke", "empty_perpendicular", 90.0, 0, (), 5.0, None, False),
        ("smoke", "empty_parallel", 0.0, 0, (), 6.0, None, False),
        ("smoke", "perpendicular_between_two_cars", 90.0, 3, (2, 4), 5.0, None, False),
        ("smoke", "parallel_between_two_cars", 0.0, 3, (2, 4), 6.0, None, False),
        ("benchmark", "perpendicular_static_obstacles", 90.0, 2, (0, 1, 3, 4, 5), 5.0, None, False),
        ("benchmark", "parallel_static_obstacles", 0.0, 2, (0, 1, 3, 4, 5), 6.0, None, False),
        ("benchmark", "parallel_tight_between_two_cars", 0.0, 3, (2, 4), 5.0, None, False),
        (
            "benchmark",
            "parallel_offset_between_two_cars",
            0.0,
            3,
            (2, 4),
            6.0,
            {2: (-0.45, 0.0), 4: (0.45, 0.0)},
            False,
        ),
        ("benchmark", "parallel_end_slot", 0.0, 0, (1,), 6.0, None, False),
        ("benchmark", "angled_60_between_two_cars", 60.0, 3, (2, 4), 5.0, None, False),
        ("benchmark", "perpendicular_bollard_at_entrance", 90.0, 2, (1, 3), 5.0, None, True),
    )

    for suite, scenario_id, orientation, goal, parked, slot_length, offsets, bollard in catalog:
        scenario = build_scenario(
            scenario_id,
            orientation_deg=orientation,
            goal_slot=goal,
            parked_slots=parked,
            slot_length=slot_length,
            parked_offsets=offsets,
            bollard_at_entrance=bollard,
        )
        output_path = SCENARIO_ROOT / suite / f"{scenario_id}.json"
        save_scenario(scenario, output_path)
        print(f"Generated {output_path.relative_to(PROJECT_ROOT)}")


if __name__ == "__main__":
    main()
