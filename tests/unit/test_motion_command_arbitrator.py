import numpy as np

from simulator.simulation import MotionCommand, SimulatorController
from simulator.simulation import AudiA5


def test_arbitrator_clamps_to_vehicle_actuator_limits():
    vehicle = AudiA5()
    controller = SimulatorController.from_vehicle(vehicle)

    command = MotionCommand(
        longitudinal_acceleration_mps2=vehicle.MAX_ACCELERATION * 2.0,
        front_mid_axle_steering_angle_rad=vehicle.MAX_STEERING_ANGLE * 2.0,
    )
    actuator_command = controller.apply_limits(command)

    assert actuator_command.longitudinal_acceleration_mps2 == vehicle.MAX_ACCELERATION
    assert actuator_command.front_mid_axle_steering_angle_rad == vehicle.MAX_STEERING_ANGLE


def test_arbitrator_preserves_valid_negative_acceleration_and_steering():
    controller = SimulatorController.from_vehicle(AudiA5())

    command = MotionCommand(
        longitudinal_acceleration_mps2=-1.2,
        front_mid_axle_steering_angle_rad=-0.3,
    )
    actuator_command = controller.apply_limits(command)

    assert actuator_command.longitudinal_acceleration_mps2 == -1.2
    assert actuator_command.front_mid_axle_steering_angle_rad == -0.3
    assert actuator_command.as_step_args() == (-1.2, -0.3)


def test_arbitrator_clamps_reverse_and_left_limits():
    vehicle = AudiA5()
    controller = SimulatorController.from_vehicle(vehicle)

    command = MotionCommand(
        longitudinal_acceleration_mps2=vehicle.MAX_DECELERATION * 2.0,
        front_mid_axle_steering_angle_rad=-np.pi,
    )
    actuator_command = controller.apply_limits(command)

    assert actuator_command.longitudinal_acceleration_mps2 == vehicle.MAX_DECELERATION
    assert actuator_command.front_mid_axle_steering_angle_rad == -vehicle.MAX_STEERING_ANGLE
