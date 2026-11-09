#include "parking_adas/reeds_shepp_local/reeds_shepp_local_planner.hpp"

#include <chrono>
#include <cmath>

#include "parking_adas/common/math.hpp"
#include "parking_adas/planning/planning_context.hpp"
#include "parking_adas/planning/reeds_shepp_connector.hpp"

namespace parking_adas {

std::vector<Waypoint> ReedsSheppLocalPlanner::plan(const PlanRequest& input_request) {
    const PlanningContext context(input_request);
    const PlanRequest& request = context.request();
    const auto started = std::chrono::steady_clock::now();
    diagnostics_ = {};
    diagnostics_.closest_goal_distance = std::hypot(
        request.goal.x - request.start_x, request.goal.y - request.start_y);

    const ReedsSheppConnector connector;
    std::vector<Waypoint> connection;
    const auto deadline = started + std::chrono::milliseconds(3000);
    const bool connected = connector.connect(
        {request.start_x, request.start_y, request.start_heading}, request.goal,
        request.start_direction, context, deadline, connection,
        &diagnostics_.reeds_shepp_connector);

    diagnostics_.goal_connection_attempts = 1;
    diagnostics_.planning_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    diagnostics_.time_limited = std::chrono::steady_clock::now() >= deadline;
    diagnostics_.solution_found = connected;
    if (!connected) return {};

    connection.insert(connection.begin(), {
        request.start_x,
        request.start_y,
        request.start_direction,
        request.start_heading,
        curvature_from_steering_angle(
            request.start_steering, request.vehicle.wheelbase),
        request.start_velocity,
        false,
    });
    diagnostics_.closest_goal_distance = 0.0;
    return connection;
}

}  // namespace parking_adas
