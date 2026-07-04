#pragma once

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class PoseController {
public:
    explicit PoseController(VehicleParameters vehicle = {});

    void set_vehicle(VehicleParameters vehicle);

    MotionCommand pose_command(const VehicleState& state, Pose target) const;
    MotionCommand terminal_command(const VehicleState& state, Pose goal) const;
    MotionCommand stop_and_steer_command(const VehicleState& state, double target_steering) const;

private:
    VehicleParameters vehicle_;
};

}  // namespace parking_adas
