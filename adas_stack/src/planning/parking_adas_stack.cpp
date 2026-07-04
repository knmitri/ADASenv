#include "parking_adas/stack/parking_adas_stack.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "parking_adas/common/math.hpp"
#include "parking_adas/planning/path_geometry.hpp"

namespace parking_adas {

std::vector<Waypoint> ParkingAdasStack::reset(const PlanRequest& request) {
    vehicle_ = request.vehicle;
    pose_controller_.set_vehicle(vehicle_);
    goal_ = request.goal;
    phase_ = 0;
    phase_distance_ = 0.0;

    if (std::abs(std::cos(goal_.yaw)) < 0.25) {
        mode_ = Mode::Parallel;
        pull_pose_ = parallel_pull_pose(goal_);
        return parallel_visualization_path(
            request.start_x,
            request.start_y,
            request.start_heading,
            goal_,
            pull_pose_
        );
    }

    if (std::abs(std::sin(goal_.yaw)) < 0.25) {
        mode_ = Mode::Perpendicular;
        target_pose_ = perpendicular_target_pose(goal_);
        seam_pose_ = perpendicular_seam_pose(target_pose_);
        return perpendicular_visualization_path(
            request.start_x,
            request.start_y,
            request.start_heading,
            target_pose_,
            seam_pose_
        );
    }

    mode_ = Mode::Pose;
    return bezier_path(
        request.start_x,
        request.start_y,
        goal_.x,
        goal_.y,
        request.start_heading,
        goal_.yaw,
        1,
        32
    );
}

MotionCommand ParkingAdasStack::command(const VehicleState& state) {
    if (mode_ == Mode::Parallel) {
        return parallel_command(state);
    }
    if (mode_ == Mode::Perpendicular) {
        return perpendicular_command(state);
    }
    return pose_controller_.pose_command(state, goal_);
}

Pose ParkingAdasStack::parallel_pull_pose(Pose goal) const {
    Pose pose = goal;
    pose = integrate_pose_by_distance(
        pose,
        vehicle_.max_steering_angle,
        parallel_arc_distance_,
        vehicle_.wheelbase
    );
    pose = integrate_pose_by_distance(
        pose,
        -vehicle_.max_steering_angle,
        parallel_arc_distance_,
        vehicle_.wheelbase
    );
    return pose;
}

std::vector<Waypoint> ParkingAdasStack::parallel_visualization_path(
    double start_x,
    double start_y,
    double start_heading,
    Pose goal,
    Pose pull_pose
) const {
    std::vector<Waypoint> path = bezier_path(
        start_x,
        start_y,
        pull_pose.x,
        pull_pose.y,
        start_heading,
        pull_pose.yaw,
        1,
        48
    );

    Pose pose = pull_pose;
    for (const double steering : {-vehicle_.max_steering_angle, vehicle_.max_steering_angle}) {
        for (int i = 0; i < 40; ++i) {
            pose = integrate_pose_by_distance(
                pose,
                steering,
                -parallel_arc_distance_ / 40.0,
                vehicle_.wheelbase
            );
            path.push_back({pose.x, pose.y, -1, pose.yaw});
        }
    }
    path.push_back({goal.x, goal.y, -1, goal.yaw});
    return path;
}

Pose ParkingAdasStack::perpendicular_target_pose(Pose goal) const {
    const double lateral_x = -std::sin(goal.yaw);
    const double lateral_y = std::cos(goal.yaw);
    return {
        goal.x + perpendicular_lateral_bias_ * lateral_x,
        goal.y + perpendicular_lateral_bias_ * lateral_y,
        goal.yaw,
    };
}

Pose ParkingAdasStack::perpendicular_seam_pose(Pose target) const {
    return {
        target.x + perpendicular_reverse_distance_ * std::cos(target.yaw),
        target.y + perpendicular_reverse_distance_ * std::sin(target.yaw),
        target.yaw,
    };
}

std::vector<Waypoint> ParkingAdasStack::perpendicular_visualization_path(
    double start_x,
    double start_y,
    double start_heading,
    Pose target,
    Pose seam
) const {
    std::vector<Waypoint> path = bezier_path(
        start_x,
        start_y,
        seam.x,
        seam.y,
        start_heading,
        seam.yaw,
        1,
        48
    );
    for (int i = 0; i < 16; ++i) {
        const double t = static_cast<double>(i) / 15.0;
        path.push_back({
            seam.x + t * (target.x - seam.x),
            seam.y + t * (target.y - seam.y),
            -1,
            target.yaw,
        });
    }
    return path;
}

MotionCommand ParkingAdasStack::parallel_command(const VehicleState& state) {
    if (phase_ == 0) {
        const bool settled_at_pull =
            std::hypot(state.x - pull_pose_.x, state.y - pull_pose_.y) < 0.35 &&
            std::abs(normalize_angle(state.yaw - pull_pose_.yaw)) < 0.15 &&
            std::abs(state.velocity) < 0.22;
        if (settled_at_pull) {
            phase_ = 1;
            phase_distance_ = 0.0;
        } else {
            return pose_controller_.pose_command(state, pull_pose_);
        }
    }

    const std::vector<std::string> names = {
        "pre_arc_in",
        "arc_in",
        "pre_counter_arc",
        "counter_arc",
    };
    const std::vector<double> distances = {
        0.0,
        parallel_arc_distance_,
        0.0,
        parallel_arc_distance_,
    };
    const std::vector<double> steering = {
        -vehicle_.max_steering_angle,
        -vehicle_.max_steering_angle,
        vehicle_.max_steering_angle,
        vehicle_.max_steering_angle,
    };
    const std::vector<double> target_speeds = {0.0, -0.25, 0.0, -0.25};

    if (phase_ >= 1 && phase_ <= static_cast<int>(names.size())) {
        const int idx = phase_ - 1;
        if (names.at(idx).find("pre_") == 0) {
            const MotionCommand command = pose_controller_.stop_and_steer_command(state, steering.at(idx));
            if (
                std::abs(state.velocity) < 0.04 &&
                std::abs(normalize_angle(state.steering_angle - steering.at(idx))) < 0.04
            ) {
                ++phase_;
                phase_distance_ = 0.0;
            }
            return command;
        }

        phase_distance_ += std::abs(state.velocity) * command_dt_;
        if (phase_distance_ >= distances.at(idx)) {
            ++phase_;
            phase_distance_ = 0.0;
        }
        return {
            clamp_value(1.3 * (target_speeds.at(idx) - state.velocity), -1.0, 1.0),
            steering.at(idx),
        };
    }

    return pose_controller_.terminal_command(state, goal_);
}

MotionCommand ParkingAdasStack::perpendicular_command(const VehicleState& state) {
    if (phase_ == 0) {
        const bool settled_at_seam =
            std::hypot(state.x - seam_pose_.x, state.y - seam_pose_.y) < 0.35 &&
            std::abs(normalize_angle(state.yaw - seam_pose_.yaw)) < 0.15 &&
            std::abs(state.velocity) < 0.22;
        if (settled_at_seam) {
            phase_ = 1;
            phase_distance_ = 0.0;
        } else {
            return pose_controller_.pose_command(state, seam_pose_);
        }
    }

    if (phase_ == 1) {
        const MotionCommand command = pose_controller_.stop_and_steer_command(state, 0.0);
        if (std::abs(state.velocity) < 0.04 && std::abs(state.steering_angle) < 0.04) {
            phase_ = 2;
            phase_distance_ = 0.0;
        }
        return command;
    }

    if (phase_ == 2) {
        phase_distance_ += std::abs(state.velocity) * command_dt_;
        if (phase_distance_ >= perpendicular_reverse_distance_) {
            phase_ = 3;
            phase_distance_ = 0.0;
        }
        return {clamp_value(1.3 * (-0.25 - state.velocity), -1.0, 1.0), 0.0};
    }

    const double distance_to_target = std::hypot(state.x - target_pose_.x, state.y - target_pose_.y);
    if (distance_to_target < 0.45) {
        return pose_controller_.terminal_command(state, target_pose_);
    }
    return pose_controller_.pose_command(state, target_pose_);
}

}  // namespace parking_adas
