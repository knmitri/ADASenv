import copy

import numpy as np

from simulator.simulation import AudiA5, ParkingLot, ParkingSimulationEnvironment
from simulator.simulation.scenarios.models import ScenarioObject
from simulator.simulation.scenarios.parking import ParkingScenario


def _config(speed_factor: float = 1.0) -> dict:
    return {
        "parking_lot": {
            "rows": 6,
            "cols": 1,
            "layout": "single_side",
            "origin_x": 10.0,
            "origin_y": 0.0,
            "slot_width": 2.5,
            "slot_length": 5.0,
            "gap_between_slots": 0.0,
            "aisle_width": 6.0,
            "slot_orientation_deg": 90.0,
        },
        "scenario": {"occupancy": 0.0, "seed": 42},
        "simulation": {
            "render": False,
            "dt": 0.05,
            "max_episode_steps": 10,
            "speed_factor": speed_factor,
            "random_spawn": False,
            "spawn_x": 0.0,
            "spawn_y": 0.0,
            "spawn_heading": 0.0,
            "world_min_x": -2.0,
            "world_min_y": -2.0,
            "collision_boundary_margin": 0.0,
            "goal_heading_tolerance_deg": 10.0,
        },
    }


def test_speed_factor_does_not_change_physics_step() -> None:
    states = []
    for speed_factor in (1.0, 10.0):
        config = _config(speed_factor=speed_factor)
        sim = ParkingSimulationEnvironment(AudiA5(), config)
        sim.reset()
        state, _ = sim.step(acceleration=1.0, steering_angle=0.0)
        states.append(state)

    np.testing.assert_allclose(states[0]["position"], states[1]["position"])
    np.testing.assert_allclose(states[0]["velocity"], states[1]["velocity"])


def test_single_side_lot_ends_at_back_of_slots() -> None:
    lot = ParkingLot(_config()["parking_lot"])

    assert lot.lot_width == 15.0
    assert lot.lot_length == 15.0
    assert lot.get_slot(0).x == 12.5


def test_single_side_parallel_slots_use_parallel_geometry() -> None:
    config = _config()
    config["parking_lot"]["rows"] = 2
    config["parking_lot"]["slot_orientation_deg"] = 0.0
    lot = ParkingLot(config["parking_lot"])

    assert np.isclose(lot.get_slot(0).heading, np.pi / 2)
    assert np.isclose(lot.get_slot(0).x, 11.25)
    assert np.isclose(lot.get_slot(1).y - lot.get_slot(0).y, 5.0)
    assert np.isclose(lot.lot_width, 12.5)
    assert np.isclose(lot.lot_length, 10.0)


def test_rotated_goal_success_uses_slot_polygon() -> None:
    config = _config()
    config["parking_lot"]["rows"] = 1
    config["parking_lot"]["slot_orientation_deg"] = 60.0
    config["simulation"]["world_min_x"] = -5.0
    config["simulation"]["world_min_y"] = -5.0

    sim = ParkingSimulationEnvironment(AudiA5(), config)
    sim.reset()
    goal_x, goal_y, goal_heading = sim.get_goal_pose()
    sim.vehicle.x = goal_x
    sim.vehicle.y = goal_y
    sim.vehicle.yaw = goal_heading

    _, info = sim.step(acceleration=0.0, steering_angle=0.0)

    assert info["goal_reached"]


def test_parked_cars_are_static_collisions() -> None:
    config = _config()
    lot = ParkingLot(config["parking_lot"])
    scenario = ParkingScenario(
        scenario_id="parked_car_collision",
        lot=lot,
        goal_slot_index=0,
        parked_cars=(
            ScenarioObject(
                object_id="parked_car",
                object_type="parked_vehicle",
                x=1.5,
                y=0.0,
                heading=0.0,
                length=4.67,
                width=1.86,
            ),
        ),
    )

    sim = ParkingSimulationEnvironment(AudiA5(), copy.deepcopy(config), scenario=scenario)
    sim.reset()
    _, info = sim.step(acceleration=0.0, steering_angle=0.0)

    assert info["collision"]
    assert info["collision_occurred"]


def test_static_obstacles_export_rotated_footprints() -> None:
    config = _config()
    lot = ParkingLot(config["parking_lot"])
    scenario = ParkingScenario(
        scenario_id="rotated_obstacle_export",
        lot=lot,
        goal_slot_index=0,
        parked_cars=(
            ScenarioObject(
                object_id="parked_car",
                object_type="parked_vehicle",
                x=3.0,
                y=4.0,
                heading=np.deg2rad(30.0),
                length=4.67,
                width=1.86,
            ),
        ),
    )

    sim = ParkingSimulationEnvironment(AudiA5(), copy.deepcopy(config), scenario=scenario)
    obstacles = sim.get_static_obstacles()

    assert "corners" in obstacles[0]
    assert len(obstacles[0]["corners"]) == 4
    assert np.isclose(obstacles[0]["heading"], np.deg2rad(30.0))
