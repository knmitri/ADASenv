#include "parking_adas/reeds_shepp_local/reeds_shepp_local_planner.hpp"

#include <chrono>
#include <cmath>

#include "parking_adas/planning/collision_checker.hpp"
#include "parking_adas/planning/parking_goal_evaluator.hpp"
#include "parking_adas/planning/reeds_shepp_connector.hpp"

namespace parking_adas {

std::vector<Waypoint> ReedsSheppLocalPlanner::plan(const PlanRequest& request) {
    const auto started = std::chrono::steady_clock::now();
    diagnostics_ = {};
    diagnostics_.closest_goal_distance = std::hypot(
        request.goal.x - request.start_x, request.goal.y - request.start_y);

    const CollisionScene collision_scene(request);
    const CollisionChecker collision_checker(collision_scene);
    const ParkingGoalEvaluator goal_evaluator(request);
    const ReedsSheppConnector connector;
    std::vector<Waypoint> connection;
    const auto deadline = started + std::chrono::milliseconds(3000);
    const bool connected = connector.connect(
        {request.start_x, request.start_y, request.start_heading}, request.goal,
        request.start_direction, request, collision_checker, goal_evaluator,
        deadline, connection);

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
        std::tan(request.start_steering) / request.vehicle.wheelbase,
        request.start_velocity,
        false,
    });
    diagnostics_.closest_goal_distance = 0.0;
    return connection;
}

}  // namespace parking_adas
