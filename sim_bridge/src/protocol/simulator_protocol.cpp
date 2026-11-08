#include "sim_bridge/protocol/simulator_protocol.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "sim_bridge/protocol/json_fields.hpp"

namespace sim_bridge {

namespace {

Json path_array(const std::vector<parking_adas::Waypoint>& path) {
    Json result = Json::array();
    for (const parking_adas::Waypoint& point : path) {
        result.push_back({
            {"x", point.x},
            {"y", point.y},
            {"direction", point.direction},
            {"heading", point.heading},
            {"curvature", point.curvature},
            {"target_speed", point.target_speed},
            {"gear_change", point.gear_change},
        });
    }
    return result;
}

void require_message_type(const Json& request, const std::string& expected) {
    const std::string actual = required_string_field(request, "type", "request");
    if (actual != expected) {
        throw std::invalid_argument(
            "Expected message type '" + expected + "', got '" + actual + "'"
        );
    }
}

std::vector<std::vector<double>> optional_corners(
    const Json& object,
    const std::string& path
) {
    return optional_number_arrays_field(object, "corners", path, 2);
}

parking_adas::Obstacle parse_obstacle(const Json& value, const std::string& path) {
    if (!value.is_object()) {
        throw std::invalid_argument(path + " must be a JSON object");
    }
    parking_adas::Obstacle obstacle{
        required_number_field(value, "x", path),
        required_number_field(value, "y", path),
        required_number_field(value, "heading", path),
        required_number_field(value, "length", path),
        required_number_field(value, "width", path),
    };
    for (const std::vector<double>& point : optional_corners(value, path)) {
        obstacle.footprint.push_back({point[0], point[1]});
    }
    return obstacle;
}

Json parse_request_type(const std::string& json_line, const std::string& type) {
    Json request = parse_json_object(json_line, type + " request");
    require_message_type(request, type);
    return request;
}

}  // namespace

std::string message_type(const std::string& json_line) {
    const Json request = parse_json_object(json_line, "message");
    return required_string_field(request, "type", "request");
}

parking_adas::PlanRequest parse_reset_request(const std::string& json_line) {
    const Json request = parse_request_type(json_line, "reset");
    const std::vector<double> start =
        required_number_array_field(request, "start", "request", 2);
    const std::vector<double> goal = request.contains("goal")
        ? required_number_array_field(request, "goal", "request", 3)
        : required_number_array_field(request, "goal_pose", "request", 3);

    parking_adas::PlanRequest result;
    result.start_x = start[0];
    result.start_y = start[1];
    result.start_heading = required_number_field(request, "start_heading", "request");
    result.goal = {goal[0], goal[1], goal[2]};

    const Json vehicle = optional_object_field(request, "vehicle", "request");
    result.vehicle.length =
        optional_number_field(vehicle, "length", "vehicle", result.vehicle.length);
    result.vehicle.width =
        optional_number_field(vehicle, "width", "vehicle", result.vehicle.width);
    result.vehicle.wheelbase =
        optional_number_field(vehicle, "wheelbase", "vehicle", result.vehicle.wheelbase);
    result.vehicle.front_overhang = optional_number_field(
        vehicle, "front_overhang", "vehicle", result.vehicle.front_overhang);
    result.vehicle.rear_overhang = optional_number_field(
        vehicle, "rear_overhang", "vehicle", result.vehicle.rear_overhang);
    result.vehicle.max_steering_angle = optional_number_field(
        vehicle, "max_steering_angle", "vehicle", result.vehicle.max_steering_angle);
    result.vehicle.max_steering_rate = optional_number_field(
        vehicle, "max_steering_rate", "vehicle", result.vehicle.max_steering_rate);
    result.vehicle.max_acceleration = optional_number_field(
        vehicle, "max_acceleration", "vehicle", result.vehicle.max_acceleration);
    result.vehicle.max_deceleration = optional_number_field(
        vehicle, "max_deceleration", "vehicle", result.vehicle.max_deceleration);
    result.vehicle.max_speed =
        optional_number_field(vehicle, "max_speed", "vehicle", result.vehicle.max_speed);
    result.vehicle.max_reverse_speed = std::abs(optional_number_field(
        vehicle, "max_reverse_speed", "vehicle", result.vehicle.max_reverse_speed));

    const Json simulation = optional_object_field(request, "simulation", "request");
    result.world_min_x =
        optional_number_field(simulation, "world_min_x", "simulation", result.world_min_x);
    result.world_min_y =
        optional_number_field(simulation, "world_min_y", "simulation", result.world_min_y);
    result.world_max_x =
        optional_number_field(simulation, "world_max_x", "simulation", result.world_max_x);
    result.world_max_y =
        optional_number_field(simulation, "world_max_y", "simulation", result.world_max_y);
    result.dt = optional_number_field(simulation, "dt", "simulation", result.dt);
    result.replan_position_threshold = optional_number_field(
        simulation, "replan_position_threshold", "simulation",
        result.replan_position_threshold);
    result.replan_heading_threshold = optional_number_field(
        simulation, "replan_heading_threshold", "simulation",
        result.replan_heading_threshold);
    result.goal_heading_tolerance = optional_number_field(
        simulation, "goal_heading_tolerance", "simulation", result.goal_heading_tolerance);

    const Json slot = optional_object_field(request, "goal_slot", "request");
    for (const std::vector<double>& point : optional_corners(slot, "goal_slot")) {
        result.goal_slot_footprint.push_back({point[0], point[1]});
    }

    const Json obstacles = optional_array_field(request, "static_obstacles", "request");
    result.static_obstacles.reserve(obstacles.size());
    for (std::size_t index = 0; index < obstacles.size(); ++index) {
        result.static_obstacles.push_back(parse_obstacle(
            obstacles[index],
            "static_obstacles[" + std::to_string(index) + "]"
        ));
    }
    return result;
}

parking_adas::VehicleState parse_state_request(const std::string& json_line) {
    const Json request = parse_request_type(json_line, "state");
    Json state_values = request;
    if (request.contains("state")) {
        if (!request["state"].is_object()) {
            throw std::invalid_argument("state must be a JSON object");
        }
        state_values = request["state"];
    }

    const std::vector<double> position =
        required_number_array_field(state_values, "position", "state", 2);
    const std::vector<double> velocity =
        required_number_array_field(state_values, "velocity", "state", 1);

    parking_adas::VehicleState result;
    result.x = position[0];
    result.y = position[1];
    result.yaw = required_number_field(state_values, "heading", "state");
    result.velocity = velocity[0];
    result.steering_angle =
        optional_number_field(state_values, "steering_angle", "state", 0.0);

    const Json obstacles = optional_array_field(request, "dynamic_obstacles", "request");
    result.dynamic_obstacles.reserve(obstacles.size());
    for (std::size_t index = 0; index < obstacles.size(); ++index) {
        result.dynamic_obstacles.push_back(parse_obstacle(
            obstacles[index],
            "dynamic_obstacles[" + std::to_string(index) + "]"
        ));
    }
    return result;
}

std::string hello_response() {
    return Json{
        {"type", "hello"},
        {"protocol_version", 1},
        {"stack_name", "reference_parking_stack"},
    }.dump();
}

std::string ready_response(const std::vector<parking_adas::Waypoint>& path) {
    return ready_response(path, {});
}

std::string ready_response(
    const std::vector<parking_adas::Waypoint>& path,
    const parking_adas::PlanningDiagnostics& diagnostics
) {
    return Json{
        {"type", "ready"},
        {"planned_path", path_array(path)},
        {"planning_diagnostics", {
            {"expanded_nodes", diagnostics.expanded_nodes},
            {"goal_connection_attempts", diagnostics.goal_connection_attempts},
            {"closest_goal_distance_m", diagnostics.closest_goal_distance},
            {"planning_time_ms", diagnostics.planning_time_ms},
            {"time_limited", diagnostics.time_limited},
            {"solution_found", diagnostics.solution_found},
            {"partial_plan", diagnostics.partial_plan},
        }},
    }.dump();
}

std::string command_response(
    parking_adas::MotionCommand command,
    const std::vector<parking_adas::Waypoint>& path,
    const parking_adas::CommandDiagnostics& diagnostics
) {
    return Json{
        {"type", "command"},
        {"longitudinal_acceleration_mps2", command.acceleration},
        {"front_mid_axle_steering_angle_rad", command.steering},
        {"planned_path", path_array(path)},
        {"command_diagnostics", {
            {"replanned", diagnostics.replanned},
            {"safety_override", diagnostics.safety_override},
            {"replan_reason", diagnostics.replan_reason == nullptr
                ? std::string{} : diagnostics.replan_reason},
        }},
    }.dump();
}

std::string ack_response() {
    return Json{{"type", "ack"}}.dump();
}

std::string error_response(const std::string& message) {
    return Json{{"type", "error"}, {"error", message}}.dump();
}

}  // namespace sim_bridge
