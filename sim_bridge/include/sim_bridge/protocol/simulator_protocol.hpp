#pragma once

#include <string>
#include <vector>

#include "parking_adas/domain/types.hpp"

namespace sim_bridge {

std::string message_type(const std::string& json_line);

parking_adas::PlanRequest parse_reset_request(const std::string& json_line);
parking_adas::VehicleState parse_state_request(const std::string& json_line);

std::string hello_response();
std::string ready_response(const std::vector<parking_adas::Waypoint>& path);
std::string command_response(parking_adas::MotionCommand command);
std::string ack_response();
std::string error_response(const std::string& message);

}  // namespace sim_bridge
