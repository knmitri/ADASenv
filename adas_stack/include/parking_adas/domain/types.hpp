#pragma once

#include <vector>

namespace parking_adas {

struct Point2 {
    double x = 0.0;
    double y = 0.0;
};

using Footprint = std::vector<Point2>;

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct Obstacle {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double length = 0.0;
    double width = 0.0;
    Footprint footprint;
};

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
    double steering_angle = 0.0;
    std::vector<Obstacle> dynamic_obstacles;
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
    double curvature = 0.0;
    double target_speed = 0.0;
    bool gear_change = false;
};

struct VehicleParameters {
    double length = 4.67;
    double width = 1.86;
    double wheelbase = 2.82;
    double front_overhang = 0.89;
    double rear_overhang = 0.96;
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
    Footprint goal_slot_footprint;
    VehicleParameters vehicle;
    std::vector<Obstacle> static_obstacles;
    double world_min_x = -2.0;
    double world_min_y = -2.0;
    double world_max_x = 20.0;
    double world_max_y = 20.0;
    double dt = 0.05;
    double replan_position_threshold = 0.75;
    double replan_heading_threshold = 0.35;
    double goal_heading_tolerance = 0.17453292519943295;
};

using Path = std::vector<Waypoint>;
using Trajectory = std::vector<Waypoint>;
using PlanningRequest = PlanRequest;

struct PlanningDiagnostics {
    int expanded_nodes = 0;
    int goal_connection_attempts = 0;
    double closest_goal_distance = 0.0;
    double planning_time_ms = 0.0;
    bool time_limited = false;
    bool solution_found = false;
    bool partial_plan = false;
};

}  // namespace parking_adas
