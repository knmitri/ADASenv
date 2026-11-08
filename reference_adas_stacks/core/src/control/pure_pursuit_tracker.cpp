#include "parking_adas/control/pure_pursuit_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "parking_adas/common/math.hpp"

namespace parking_adas {

void PurePursuitTrajectoryTracker::reset(const std::vector<Waypoint>& trajectory, VehicleParameters vehicle) {
    trajectory_ = trajectory;
    vehicle_ = vehicle;
    progress_index_ = 0;
    stall_cycles_ = 0;
    terminal_replan_cycles_ = 0;
    has_previous_position_ = false;
}

MotionCommand PurePursuitTrajectoryTracker::command(const VehicleState& state) {
    if (trajectory_.empty()) {
        const double acceleration = state.velocity > 0.0
            ? vehicle_.max_deceleration
            : state.velocity < 0.0 ? vehicle_.max_acceleration : 0.0;
        return {acceleration, state.steering_angle};
    }

    std::size_t gear_boundary = trajectory_.size() - 1;
    for (std::size_t index = progress_index_; index < trajectory_.size(); ++index) {
        if (trajectory_[index].gear_change) {
            gear_boundary = index;
            break;
        }
    }
    const std::size_t search_end = std::min(
        std::min(trajectory_.size(), progress_index_ + 12), gear_boundary + 1);
    std::size_t closest = progress_index_;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = progress_index_; index < search_end; ++index) {
        const Waypoint& point = trajectory_.at(index);
        const double distance = std::hypot(point.x - state.x, point.y - state.y);
        if (distance < closest_distance) {
            closest_distance = distance;
            closest = index;
        }
    }
    progress_index_ = std::min(closest, progress_index_ + 3);

    if (gear_boundary + 1 < trajectory_.size()) {
        const Waypoint& boundary = trajectory_[gear_boundary];
        const double boundary_distance = std::hypot(boundary.x - state.x, boundary.y - state.y);
        if (boundary_distance < 0.15) {
            if (std::abs(state.velocity) > 0.05) {
                const double braking = state.velocity > 0.0
                    ? vehicle_.max_deceleration
                    : vehicle_.max_acceleration;
                return {braking, state.steering_angle};
            }
            progress_index_ = gear_boundary + 1;
            return command(state);
        }
    }
    const unsigned int base_lookahead = trajectory_[progress_index_].direction < 0
        ? (std::abs(state.velocity) < 0.15 ? 3U : 4U)
        : (std::abs(state.velocity) < 0.15 ? 5U : 8U);
    std::size_t lookahead = std::min(trajectory_.size() - 1, progress_index_ + base_lookahead);
    lookahead = std::min(lookahead, gear_boundary);
    const Waypoint& target = trajectory_.at(lookahead);
    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double distance = std::max(0.15, std::hypot(dx, dy));
    const double bearing = std::atan2(dy, dx);
    const double direction = target.direction < 0 ? -1.0 : 1.0;
    const double alpha = normalize_angle(bearing - state.yaw + (direction < 0.0 ? kPi : 0.0));
    const double curvature = 2.0 * std::sin(alpha) / distance;
    const double heading_error = normalize_angle(target.heading - state.yaw);
    const double steering = direction * (
        std::atan(vehicle_.wheelbase * curvature) + 0.35 * heading_error);

    const bool final_point = progress_index_ + 2 >= trajectory_.size();
    const double final_distance = std::hypot(
        trajectory_.back().x - state.x, trajectory_.back().y - state.y);
    const bool approaching_gear_change = target.gear_change && distance > 0.2;
    const bool at_terminal_target = lookahead + 1 >= trajectory_.size() || final_distance < 0.5;
    const double gear_speed = target.direction < 0
        ? -std::min(vehicle_.max_reverse_speed, 0.25)
        : std::min(vehicle_.max_speed, 0.25);
    const double desired_speed = approaching_gear_change
        ? gear_speed
        : at_terminal_target && final_distance < 0.08
        ? 0.0
        : (at_terminal_target || final_point ? direction * 0.2 : target.target_speed);
    return {
        clamp_value(1.8 * (desired_speed - state.velocity), -1.5, 1.5),
        clamp_value(steering, -vehicle_.max_steering_angle, vehicle_.max_steering_angle),
    };
}

TrackingStatus PurePursuitTrajectoryTracker::assess(const VehicleState& state) {
    TrackingStatus status;
    if (trajectory_.empty()) {
        status.trajectory_empty = true;
        return status;
    }
    const std::size_t end = std::min(trajectory_.size(), progress_index_ + 12);
    double closest_distance = std::numeric_limits<double>::infinity();
    std::size_t closest = progress_index_;
    for (std::size_t index = progress_index_; index < end; ++index) {
        const double distance = std::hypot(trajectory_[index].x - state.x, trajectory_[index].y - state.y);
        if (distance < closest_distance) { closest_distance = distance; closest = index; }
    }
    const double displacement = has_previous_position_
        ? std::hypot(state.x - previous_position_.x, state.y - previous_position_.y)
        : 0.0;
    previous_position_ = {state.x, state.y};
    has_previous_position_ = true;
    if (std::abs(state.velocity) > 0.1 && displacement < 0.002 &&
        !trajectory_[closest].gear_change) {
        ++stall_cycles_;
    } else {
        stall_cycles_ = 0;
    }
    progress_index_ = std::max(progress_index_, std::min(closest, progress_index_ + 3));
    const double terminal_distance = std::hypot(
        trajectory_.back().x - state.x, trajectory_.back().y - state.y);
    const double terminal_heading_error = std::abs(
        normalize_angle(trajectory_.back().heading - state.yaw));
    const bool at_trajectory_end =
        closest + 1 >= trajectory_.size() && closest_distance < 0.35;
    if (at_trajectory_end &&
        (terminal_distance > 0.08 || terminal_heading_error > 0.05) &&
        std::abs(state.velocity) < 0.10) {
        ++terminal_replan_cycles_;
    } else {
        terminal_replan_cycles_ = 0;
    }
    status.at_trajectory_end = at_trajectory_end;
    status.stalled = stall_cycles_ > 40 || terminal_replan_cycles_ > 10;
    status.cross_track_error = closest_distance;
    status.terminal_distance = terminal_distance;
    status.terminal_heading_error = terminal_heading_error;
    return status;
}

}  // namespace parking_adas
