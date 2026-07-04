from simulator.simulation.challenge import default_goal_slots
from simulator.simulation import ParkingLot
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
