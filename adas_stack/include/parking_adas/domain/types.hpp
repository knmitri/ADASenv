#pragma once

#include <vector>

namespace parking_adas {

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
    double steering_angle = 0.0;
};

struct MotionCommand {
    double acceleration = 0.0;
    double steering = 0.0;
};

struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    int direction = 1;
    double heading = 0.0;
};

struct VehicleParameters {
    double wheelbase = 2.82;
    double max_steering_angle = 0.7853981633974483;
    double max_steering_rate = 0.7853981633974483;
    double max_acceleration = 9.0;
    double max_deceleration = -9.0;
    double max_speed = 2.0;
    double max_reverse_speed = 1.0;
};

struct PlanRequest {
    double start_x = 0.0;
    double start_y = 0.0;
    double start_heading = 0.0;
    Pose goal;
    VehicleParameters vehicle;
};

}  // namespace parking_adas
