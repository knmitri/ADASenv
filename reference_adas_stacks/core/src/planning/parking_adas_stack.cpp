#include "parking_adas/stack/parking_adas_stack.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "parking_adas/common/math.hpp"
#include "parking_adas/planning/planning_context.hpp"

namespace parking_adas {

ParkingAdasStack::ParkingAdasStack()
    : ParkingAdasStack(make_path_planner("hybrid_astar")) {}

ParkingAdasStack::ParkingAdasStack(
    std::unique_ptr<PathPlanner> planner,
    std::unique_ptr<TrajectoryTracker> trajectory_tracker,
    std::unique_ptr<TrajectoryGenerator> trajectory_generator,
    std::unique_ptr<TrajectoryOptimizer> trajectory_optimizer)
    : trajectory_tracker_(std::move(trajectory_tracker)),
      planner_(std::move(planner)),
      trajectory_generator_(std::move(trajectory_generator)),
      trajectory_optimizer_(std::move(trajectory_optimizer)) {
    if (!planner_ || !trajectory_tracker_ || !trajectory_generator_ || !trajectory_optimizer_) {
        throw std::invalid_argument("parking stack pipeline stages must not be null");
    }
}

std::vector<Waypoint> ParkingAdasStack::reset(const PlanRequest& request) {
    request_ = request;
    vehicle_ = request.vehicle;
    command_diagnostics_ = {};

    std::vector<Waypoint> planned_path = planner_->plan(request);
    trajectory_ = generate_trajectory(std::move(planned_path));

    trajectory_tracker_->reset(trajectory_, vehicle_);
    return trajectory_;
}

MotionCommand ParkingAdasStack::command(const VehicleState& state) {
    command_diagnostics_ = {};
    const TrackingStatus tracking_status = trajectory_tracker_->assess(state);
    if (planner_->should_replan(tracking_status, request_)) {

        request_.start_x = state.x;
        request_.start_y = state.y;
        request_.start_heading = state.yaw;

        PlanRequest replanning_request = request_;
        replanning_request.start_direction = state.velocity < -0.05 ? -1 : 1;
        replanning_request.start_velocity = state.velocity;
        replanning_request.start_steering = state.steering_angle;

        replanning_request.static_obstacles.insert(
            replanning_request.static_obstacles.end(),
            state.dynamic_obstacles.begin(),
            state.dynamic_obstacles.end());

        std::vector<Waypoint> replacement =
            generate_trajectory(planner_->plan(replanning_request));

        if (!replacement.empty()) {
            trajectory_ = std::move(replacement);
            trajectory_tracker_->reset(trajectory_, vehicle_);
            command_diagnostics_ = {true, false, "tracker"};
        }
    }

    const MotionCommand command = trajectory_tracker_->command(state);

    PlanRequest safety_request = request_;

    safety_request.static_obstacles.insert(
        safety_request.static_obstacles.end(),
        state.dynamic_obstacles.begin(),
        state.dynamic_obstacles.end());

    // The simulator is authoritative for contact. Use its physical footprint for
    // one-step intervention; planning remains conservative through its default
    // clearance so a legal near-boundary manoeuvre is not rejected mid-execution.
    const PlanningContext safety_context(safety_request, 0.0);

    const double predicted_velocity =
        state.velocity + command.acceleration * request_.dt;

    const Pose predicted = integrate_pose_by_distance(
        {state.x, state.y, state.yaw},
        command.steering,
        predicted_velocity * request_.dt,
        vehicle_.wheelbase,
        request_.dt);

    if (safety_context.collision_checker().is_collision(predicted)) {
        command_diagnostics_.safety_override = true;

        safety_request.start_x = state.x;
        safety_request.start_y = state.y;
        safety_request.start_heading = state.yaw;
        safety_request.start_direction = state.velocity < -0.05 ? -1 : 1;
        safety_request.start_velocity = state.velocity;
        safety_request.start_steering = state.steering_angle;

        std::vector<Waypoint> replacement =
            generate_trajectory(planner_->plan(safety_request));

        if (!replacement.empty()) {
            trajectory_ = std::move(replacement);
            trajectory_tracker_->reset(trajectory_, vehicle_);
            command_diagnostics_ = {true, true, "predicted_collision"};

            const MotionCommand replacement_command =
                trajectory_tracker_->command(state);

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

            if (!safety_context.collision_checker().is_collision(replacement_pose)) {
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
    return trajectory_optimizer_->optimize(
        trajectory_generator_->generate(std::move(path), vehicle_), request_);
}

}  // namespace parking_adas
