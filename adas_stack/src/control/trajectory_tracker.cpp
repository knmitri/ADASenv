#include "parking_adas/control/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "parking_adas/common/math.hpp"

namespace parking_adas {

void TrajectoryTracker::reset(const std::vector<Waypoint>& trajectory, VehicleParameters vehicle) {
    trajectory_ = trajectory;
    vehicle_ = vehicle;
    progress_index_ = 0;
    stall_cycles_ = 0;
    terminal_replan_cycles_ = 0;
}

MotionCommand TrajectoryTracker::command(const VehicleState& state) {
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
    const double steering = std::atan(vehicle_.wheelbase * curvature) + direction * 0.35 * heading_error;

    const bool final_point = progress_index_ + 2 >= trajectory_.size();
    const double final_distance = std::hypot(
        trajectory_.back().x - state.x, trajectory_.back().y - state.y
    );
    const bool near_trajectory_end = progress_index_ + 12 >= trajectory_.size();
    if (near_trajectory_end && final_distance < 1.0) {
        const Pose goal{
            trajectory_.back().x, trajectory_.back().y, trajectory_.back().heading};
        const double axis_x = std::cos(goal.yaw);
        const double axis_y = std::sin(goal.yaw);
        const double lateral_x = -std::sin(goal.yaw);
        const double lateral_y = std::cos(goal.yaw);
        const double error_x = state.x - goal.x;
        const double error_y = state.y - goal.y;
        const double longitudinal_error = error_x * axis_x + error_y * axis_y;
        const double lateral_error = error_x * lateral_x + error_y * lateral_y;
        const double terminal_error = normalize_angle(goal.yaw - state.yaw);
        double target_speed = clamp_value(-0.6 * longitudinal_error, -0.2, 0.2);
        if (std::abs(target_speed) < 0.04 &&
            (std::abs(terminal_error) > 0.03 || std::abs(lateral_error) > 0.025)) {
            target_speed = longitudinal_error < 0.0 ? 0.08 : -0.08;
        }
        double terminal_steering = 2.0 * terminal_error +
            std::atan2(1.2 * lateral_error, std::abs(state.velocity) + 0.25);
        if (target_speed < 0.0) terminal_steering = -terminal_steering;
        return {
            clamp_value(1.2 * (target_speed - state.velocity), -1.0, 1.0),
            clamp_value(terminal_steering, -vehicle_.max_steering_angle, vehicle_.max_steering_angle),
        };
    }
    const double terminal_heading_error = normalize_angle(trajectory_.back().heading - state.yaw);
    const double terminal_heading_gain = final_distance < 3.0 ? 0.6 : 0.0;
    const double commanded_steering = steering + direction * terminal_heading_gain * terminal_heading_error;
    const bool approaching_gear_change = target.gear_change && distance > 0.2;
    const bool at_terminal_target = lookahead + 1 >= trajectory_.size() || final_distance < 0.5;
    const double gear_speed = target.direction < 0
        ? -std::min(vehicle_.max_reverse_speed, 0.25)
        : std::min(vehicle_.max_speed, 0.25);
    const double desired_speed = approaching_gear_change
        ? gear_speed
        : at_terminal_target && final_distance < 0.2
        ? 0.0
        : (at_terminal_target || final_point ? direction * 0.2 : target.target_speed);
    return {
        clamp_value(1.8 * (desired_speed - state.velocity), -1.5, 1.5),
        clamp_value(commanded_steering, -vehicle_.max_steering_angle, vehicle_.max_steering_angle),
    };
}

bool TrajectoryTracker::needs_replan(const VehicleState& state, double position_threshold,
                                     double heading_threshold) {
    if (trajectory_.empty()) return true;
    const std::size_t end = std::min(trajectory_.size(), progress_index_ + 12);
    double closest_distance = std::numeric_limits<double>::infinity();
    std::size_t closest = progress_index_;
    for (std::size_t index = progress_index_; index < end; ++index) {
        const double distance = std::hypot(trajectory_[index].x - state.x, trajectory_[index].y - state.y);
        if (distance < closest_distance) { closest_distance = distance; closest = index; }
    }
    if (closest <= progress_index_ && std::abs(state.velocity) > 0.1 &&
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
    const bool terminal_pose_unreached = at_trajectory_end &&
        (terminal_distance > 0.08 || terminal_heading_error > 0.05);
    if (terminal_pose_unreached && std::abs(state.velocity) < 0.10) {
        ++terminal_replan_cycles_;
    } else {
        terminal_replan_cycles_ = 0;
    }
    const bool reached_trajectory_end = at_trajectory_end &&
        terminal_distance < 0.08 && terminal_heading_error < 0.05 &&
        std::abs(state.velocity) < 0.08;
    return closest_distance > position_threshold ||
           reached_trajectory_end ||
           (closest + 12 < trajectory_.size() &&
            std::abs(normalize_angle(trajectory_[closest].heading - state.yaw)) > heading_threshold) ||
           stall_cycles_ > 40 || terminal_replan_cycles_ > 10;
}

}  // namespace parking_adas
