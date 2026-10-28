#include "sim_bridge/protocol/simulator_protocol.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include "sim_bridge/protocol/json_fields.hpp"

namespace sim_bridge {

namespace {

std::string path_array(const std::vector<parking_adas::Waypoint>& path) {
    std::ostringstream out;
    out << std::setprecision(12);
    out << '[';
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i != 0) {
            out << ',';
        }
        out << "{\"x\":" << path.at(i).x
            << ",\"y\":" << path.at(i).y
            << ",\"direction\":" << path.at(i).direction
            << ",\"heading\":" << path.at(i).heading
            << ",\"curvature\":" << path.at(i).curvature
            << ",\"target_speed\":" << path.at(i).target_speed
            << ",\"gear_change\":" << (path.at(i).gear_change ? "true" : "false")
            << '}';
    }
    out << ']';
    return out.str();
}

}  // namespace

std::string message_type(const std::string& json_line) {
    return find_string(json_line, "type");
}

parking_adas::PlanRequest parse_reset_request(const std::string& json_line) {
    const std::vector<double> start = find_number_array(json_line, "start", {0.0, 0.0});
    const std::vector<double> goal = find_number_array(json_line, "goal", {0.0, 0.0, 0.0});

    parking_adas::PlanRequest request;
    request.start_x = start.at(0);
    request.start_y = start.at(1);
    request.start_heading = find_number(json_line, "start_heading", 0.0);
    request.goal = {goal.at(0), goal.at(1), goal.at(2)};
    const std::string vehicle = find_object(json_line, "vehicle");
    const std::string simulation = find_object(json_line, "simulation");
    request.vehicle.length = find_number(vehicle, "length", request.vehicle.length);
    request.vehicle.width = find_number(vehicle, "width", request.vehicle.width);
    request.vehicle.wheelbase = find_number(vehicle, "wheelbase", request.vehicle.wheelbase);
    request.vehicle.front_overhang = find_number(vehicle, "front_overhang", request.vehicle.front_overhang);
    request.vehicle.rear_overhang = find_number(vehicle, "rear_overhang", request.vehicle.rear_overhang);
    request.vehicle.max_steering_angle = find_number(
        vehicle,
        "max_steering_angle",
        request.vehicle.max_steering_angle
    );
    request.vehicle.max_steering_rate = find_number(
        vehicle,
        "max_steering_rate",
        request.vehicle.max_steering_rate
    );
    request.vehicle.max_acceleration = find_number(
        vehicle,
        "max_acceleration",
        request.vehicle.max_acceleration
    );
    request.vehicle.max_deceleration = find_number(
        vehicle,
        "max_deceleration",
        request.vehicle.max_deceleration
    );
    request.vehicle.max_speed = find_number(vehicle, "max_speed", request.vehicle.max_speed);
    request.vehicle.max_reverse_speed = std::abs(
        find_number(vehicle, "max_reverse_speed", request.vehicle.max_reverse_speed)
    );
    request.world_min_x = find_number(simulation, "world_min_x", request.world_min_x);
    request.world_min_y = find_number(simulation, "world_min_y", request.world_min_y);
    request.world_max_x = find_number(simulation, "world_max_x", request.world_max_x);
    request.world_max_y = find_number(simulation, "world_max_y", request.world_max_y);
    request.dt = find_number(simulation, "dt", request.dt);
    request.replan_position_threshold = find_number(simulation, "replan_position_threshold", request.replan_position_threshold);
    request.replan_heading_threshold = find_number(simulation, "replan_heading_threshold", request.replan_heading_threshold);
    request.goal_heading_tolerance = find_number(
        simulation, "goal_heading_tolerance", request.goal_heading_tolerance);
    for (const std::string& object : find_object_array(json_line, "static_obstacles")) {
        parking_adas::Obstacle obstacle{
            find_number(object, "x"),
            find_number(object, "y"),
            find_number(object, "heading"),
            find_number(object, "length"),
            find_number(object, "width"),
        };
        for (const auto& point : find_number_arrays(object, "corners")) {
            if (point.size() >= 2) obstacle.footprint.push_back({point[0], point[1]});
        }
        request.static_obstacles.push_back(obstacle);
    }
    const std::string slot = find_object(json_line, "goal_slot");
    for (const auto& point : find_number_arrays(slot, "corners")) {
        if (point.size() >= 2) request.goal_slot_footprint.push_back({point[0], point[1]});
    }
    return request;
}

parking_adas::VehicleState parse_state_request(const std::string& json_line) {
    const std::vector<double> position = find_number_array(json_line, "position", {0.0, 0.0});
    const std::vector<double> velocity = find_number_array(json_line, "velocity", {0.0});

    parking_adas::VehicleState state;
    state.x = position.at(0);
    state.y = position.at(1);
    state.yaw = find_number(json_line, "heading", 0.0);
    state.velocity = velocity.at(0);
    state.steering_angle = find_number(json_line, "steering_angle", 0.0);
    for (const std::string& object : find_object_array(json_line, "dynamic_obstacles")) {
        state.dynamic_obstacles.push_back({
            find_number(object, "x"), find_number(object, "y"),
            find_number(object, "heading"), find_number(object, "length"),
            find_number(object, "width"),
        });
    }
    return state;
}

std::string hello_response() {
    return "{\"type\":\"hello\",\"protocol_version\":1,\"stack_name\":\"reference_parking_stack\"}";
}

std::string ready_response(const std::vector<parking_adas::Waypoint>& path) {
    return ready_response(path, {});
}

std::string ready_response(const std::vector<parking_adas::Waypoint>& path,
                           const parking_adas::PlanningDiagnostics& diagnostics) {
    std::ostringstream out;
    out << std::setprecision(12)
        << "{\"type\":\"ready\",\"planned_path\":" << path_array(path)
        << ",\"planning_diagnostics\":{\"expanded_nodes\":" << diagnostics.expanded_nodes
        << ",\"goal_connection_attempts\":" << diagnostics.goal_connection_attempts
        << ",\"closest_goal_distance_m\":" << diagnostics.closest_goal_distance
        << ",\"planning_time_ms\":" << diagnostics.planning_time_ms
        << ",\"time_limited\":" << (diagnostics.time_limited ? "true" : "false")
        << ",\"solution_found\":" << (diagnostics.solution_found ? "true" : "false")
        << ",\"partial_plan\":" << (diagnostics.partial_plan ? "true" : "false")
        << "}}";
    return out.str();
}

std::string command_response(parking_adas::MotionCommand command, const std::vector<parking_adas::Waypoint>& path) {
    std::ostringstream out;
    out << std::setprecision(12)
        << "{\"type\":\"command\",\"longitudinal_acceleration_mps2\":" << command.acceleration
        << ",\"front_mid_axle_steering_angle_rad\":" << command.steering
        << ",\"planned_path\":" << path_array(path)
        << "}";
    return out.str();
}

std::string ack_response() {
    return "{\"type\":\"ack\"}";
}

std::string error_response(const std::string& message) {
    std::ostringstream out;
    out << "{\"type\":\"error\",\"error\":\"" << message << "\"}";
    return out.str();
}

}  // namespace sim_bridge
