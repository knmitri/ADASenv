from simulator.simulation.challenge import (
    BENCHMARK_SCENARIO_ARTIFACTS,
    DEFAULT_SCENARIO_ARTIFACTS,
    SMOKE_SCENARIO_ARTIFACTS,
    default_goal_slots,
    scenario_artifacts_for_suite,
)
from simulator.simulation import ParkingLot
from simulator.simulation.scenarios import load_parking_scenario
from simulator.simulation.scenarios.models import ScenarioObject
from simulator.simulation.scenarios.parking import ParkingScenario


def test_default_goal_slots_skip_occupied_slots() -> None:
    lot = ParkingLot(
        {
            "rows": 4,
            "cols": 1,
            "layout": "single_side",
            "origin_x": 10.0,
            "origin_y": 0.0,
            "slot_width": 2.5,
            "slot_length": 5.0,
            "gap_between_slots": 0.0,
            "aisle_width": 6.0,
            "slot_orientation_deg": 90.0,
        }
    )
    scenario = ParkingScenario(
        scenario_id="with_obstacles",
        lot=lot,
        goal_slot_index=0,
        parked_cars=(
            ScenarioObject("car_1", "parked_vehicle", 0.0, 0.0, 0.0, 4.67, 1.86, slot_index=1),
            ScenarioObject("car_3", "parked_vehicle", 0.0, 0.0, 0.0, 4.67, 1.86, slot_index=3),
        ),
    )

    assert default_goal_slots(scenario) == [0, 2]


def test_parallel_between_two_cars_scenario_has_a_single_middle_goal() -> None:
    scenario_path = next(
        path for path in SMOKE_SCENARIO_ARTIFACTS if path.name == "parallel_between_two_cars.json"
    )
    scenario = load_parking_scenario(scenario_path)

    assert scenario.goal_slot_index == 3
    assert scenario.lot.slot_orientation_type == "parallel"
    assert {car.slot_index for car in scenario.parked_cars} == {2, 4}
    assert default_goal_slots(scenario) == [0, 1, 3, 5]


def test_scenario_suites_are_complete_and_default_to_smoke() -> None:
    assert DEFAULT_SCENARIO_ARTIFACTS == SMOKE_SCENARIO_ARTIFACTS
    assert scenario_artifacts_for_suite("smoke") == SMOKE_SCENARIO_ARTIFACTS
    assert scenario_artifacts_for_suite("benchmark") == BENCHMARK_SCENARIO_ARTIFACTS
    assert scenario_artifacts_for_suite("all") == (
        SMOKE_SCENARIO_ARTIFACTS + BENCHMARK_SCENARIO_ARTIFACTS
    )
    assert all(path.is_file() for path in scenario_artifacts_for_suite("all"))
