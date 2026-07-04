#include "parking_adas/control/pose_controller.hpp"

#include <algorithm>
#include <cmath>

#include "parking_adas/common/math.hpp"

namespace parking_adas {

PoseController::PoseController(VehicleParameters vehicle)
    : vehicle_(vehicle) {}

void PoseController::set_vehicle(VehicleParameters vehicle) {
    vehicle_ = vehicle;
}

MotionCommand PoseController::pose_command(const VehicleState& state, Pose target) const {
    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double distance = std::hypot(dx, dy);
    const double target_bearing = std::atan2(dy, dx);
    double alpha = normalize_angle(target_bearing - state.yaw);
    double beta = normalize_angle(target.yaw - target_bearing);
    double direction = 1.0;
    if (std::abs(alpha) > kPi / 2.0) {
        direction = -1.0;
        alpha = normalize_angle(alpha + kPi);
        beta = normalize_angle(beta + kPi);
    }

    const double omega = 1.8 * alpha - 0.7 * beta;
    const double target_speed = direction * std::min(0.8, std::max(0.12, 0.45 * distance));
    const double steering = std::atan2(
        vehicle_.wheelbase * omega,
        std::max(std::abs(target_speed), 0.1)
    );
    return {
        clamp_value(1.4 * (target_speed - state.velocity), -1.5, 1.5),
        clamp_value(steering, -vehicle_.max_steering_angle, vehicle_.max_steering_angle),
    };
}

MotionCommand PoseController::terminal_command(const VehicleState& state, Pose goal) const {
    const double axis_x = std::cos(goal.yaw);
    const double axis_y = std::sin(goal.yaw);
    const double lateral_x = -std::sin(goal.yaw);
    const double lateral_y = std::cos(goal.yaw);
    const double err_x = state.x - goal.x;
    const double err_y = state.y - goal.y;
    const double longitudinal_error = err_x * axis_x + err_y * axis_y;
    const double lateral_error = err_x * lateral_x + err_y * lateral_y;
    const double heading_error = normalize_angle(goal.yaw - state.yaw);
    const double yaw_relative_to_goal = normalize_angle(state.yaw - goal.yaw);

    double target_speed = clamp_value(-0.6 * longitudinal_error, -0.2, 0.2);
    const double lateral_authority = std::sin(yaw_relative_to_goal);
    if (std::abs(lateral_error) > 0.035 && std::abs(lateral_authority) > 0.025) {
        target_speed = -std::copysign(0.12, lateral_error * lateral_authority);
    } else if (
        std::abs(target_speed) < 0.04 &&
        (std::abs(heading_error) > 0.03 || std::abs(lateral_error) > 0.025)
    ) {
        target_speed = longitudinal_error < 0.0 ? 0.08 : -0.08;
    }

    double steering = 2.0 * heading_error + std::atan2(1.2 * lateral_error, std::abs(state.velocity) + 0.25);
    if (target_speed < 0.0) {
        steering = -steering;
    }
    return {
        clamp_value(1.2 * (target_speed - state.velocity), -1.0, 1.0),
        clamp_value(steering, -0.35, 0.35),
    };
}

MotionCommand PoseController::stop_and_steer_command(
    const VehicleState& state,
    double target_steering
) const {
    return {clamp_value(-2.0 * state.velocity, -1.5, 1.5), target_steering};
}

}  // namespace parking_adas
