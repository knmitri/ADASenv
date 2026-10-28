#include "parking_adas/stack/parking_adas_stack.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "parking_adas/common/math.hpp"
#include "parking_adas/planning/collision_checker.hpp"

namespace parking_adas {

std::vector<Waypoint> ParkingAdasStack::reset(const PlanRequest& request) {
    request_ = request;
    vehicle_ = request.vehicle;

    std::vector<Waypoint> planned_path = planner_.plan(request);
    trajectory_ = generate_trajectory(std::move(planned_path));

    trajectory_tracker_.reset(trajectory_, vehicle_);
    return trajectory_;
}

MotionCommand ParkingAdasStack::command(const VehicleState& state) {
    if (trajectory_tracker_.needs_replan(
            state,
            request_.replan_position_threshold,
            request_.replan_heading_threshold)) {

        request_.start_x = state.x;
        request_.start_y = state.y;
        request_.start_heading = state.yaw;

        PlanRequest replanning_request = request_;

        replanning_request.static_obstacles.insert(
            replanning_request.static_obstacles.end(),
            state.dynamic_obstacles.begin(),
            state.dynamic_obstacles.end());

        std::vector<Waypoint> replacement =
            generate_trajectory(planner_.plan(replanning_request));

        if (!replacement.empty()) {
            trajectory_ = std::move(replacement);
            trajectory_tracker_.reset(trajectory_, vehicle_);
        }
    }

    const MotionCommand command = trajectory_tracker_.command(state);

    PlanRequest safety_request = request_;

    safety_request.static_obstacles.insert(
        safety_request.static_obstacles.end(),
        state.dynamic_obstacles.begin(),
        state.dynamic_obstacles.end());

    const CollisionChecker collision_checker(safety_request);

    const double predicted_velocity =
        state.velocity + command.acceleration * request_.dt;

    const Pose predicted = integrate_pose_by_distance(
        {state.x, state.y, state.yaw},
        command.steering,
        predicted_velocity * request_.dt,
        vehicle_.wheelbase,
        request_.dt);

    if (collision_checker.is_collision(predicted)) {

        safety_request.start_x = state.x;
        safety_request.start_y = state.y;
        safety_request.start_heading = state.yaw;

        std::vector<Waypoint> replacement =
            generate_trajectory(planner_.plan(safety_request));

        if (!replacement.empty()) {
            trajectory_ = std::move(replacement);
            trajectory_tracker_.reset(trajectory_, vehicle_);

            const MotionCommand replacement_command =
                trajectory_tracker_.command(state);

            const double replacement_velocity =
                state.velocity +
                replacement_command.acceleration * request_.dt;

            const Pose replacement_pose =
                integrate_pose_by_distance(
                    {state.x, state.y, state.yaw},
                    replacement_command.steering,
                    replacement_velocity * request_.dt,
                    vehicle_.wheelbase,
                    request_.dt);

            if (!collision_checker.is_collision(replacement_pose)) {
                return replacement_command;
            }
        }

        const double braking =
            state.velocity > 0.0
                ? vehicle_.max_deceleration
                : state.velocity < 0.0
                    ? vehicle_.max_acceleration
                    : 0.0;

        return {braking, state.steering_angle};
    }

    return command;
}

std::vector<Waypoint> ParkingAdasStack::generate_trajectory(
    std::vector<Waypoint> path) const {

    if (path.size() < 2) {
        return path;
    }

    // Preserve the planner's collision-checked poses and curvatures. The tracker
    // owns actuator-rate handling; altering geometry here would create a second,
    // unvalidated trajectory.
    constexpr double kForwardSpeed = 0.25;
    constexpr double kReverseSpeed = 0.25;
    for (std::size_t index = 0; index < path.size(); ++index) {
        if (index > 0 && path[index].direction != path[index - 1].direction) {
            path[index - 1].gear_change = true;
            path[index - 1].target_speed = 0.0;
        }
        if (index + 1 == path.size()) {
            path[index].target_speed = 0.0;
        } else if (!path[index].gear_change) {
            path[index].target_speed = path[index].direction < 0
                ? -std::min(vehicle_.max_reverse_speed, kReverseSpeed)
                : std::min(vehicle_.max_speed, kForwardSpeed);
        }
    }
    return path;
}

}  // namespace parking_adas
