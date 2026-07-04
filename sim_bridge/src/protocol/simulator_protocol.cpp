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
    request.vehicle.wheelbase = find_number(json_line, "wheelbase", request.vehicle.wheelbase);
    request.vehicle.max_steering_angle = find_number(
        json_line,
        "max_steering_angle",
        request.vehicle.max_steering_angle
    );
    request.vehicle.max_steering_rate = find_number(
        json_line,
        "max_steering_rate",
        request.vehicle.max_steering_rate
    );
    request.vehicle.max_acceleration = find_number(
        json_line,
        "max_acceleration",
        request.vehicle.max_acceleration
    );
    request.vehicle.max_deceleration = find_number(
        json_line,
        "max_deceleration",
        request.vehicle.max_deceleration
    );
    request.vehicle.max_speed = find_number(json_line, "max_speed", request.vehicle.max_speed);
    request.vehicle.max_reverse_speed = std::abs(
        find_number(json_line, "max_reverse_speed", request.vehicle.max_reverse_speed)
    );
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
    return state;
}

std::string hello_response() {
    return "{\"type\":\"hello\",\"protocol_version\":1,\"stack_name\":\"reference_parking_stack\"}";
}

std::string ready_response(const std::vector<parking_adas::Waypoint>& path) {
    return "{\"type\":\"ready\",\"planned_path\":" + path_array(path) + "}";
}

std::string command_response(parking_adas::MotionCommand command) {
    std::ostringstream out;
    out << std::setprecision(12)
        << "{\"type\":\"command\",\"longitudinal_acceleration_mps2\":" << command.acceleration
        << ",\"front_mid_axle_steering_angle_rad\":" << command.steering
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
