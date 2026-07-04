#pragma once

#include <vector>

#include "parking_adas/control/pose_controller.hpp"
#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class ParkingAdasStack {
public:
    std::vector<Waypoint> reset(const PlanRequest& request);
    MotionCommand command(const VehicleState& state);

private:
    enum class Mode {
        Pose,
        Parallel,
        Perpendicular,
    };

    VehicleParameters vehicle_;
    PoseController pose_controller_;
    Mode mode_ = Mode::Pose;
    Pose goal_;
    Pose pull_pose_;
    Pose target_pose_;
    Pose seam_pose_;
    int phase_ = 0;
    double phase_distance_ = 0.0;
    double command_dt_ = 0.05;
    double parallel_arc_distance_ = 4.6;
    double perpendicular_reverse_distance_ = 2.4;
    double perpendicular_lateral_bias_ = 0.2;

    Pose parallel_pull_pose(Pose goal) const;
    std::vector<Waypoint> parallel_visualization_path(
        double start_x,
        double start_y,
        double start_heading,
        Pose goal,
        Pose pull_pose
    ) const;

    Pose perpendicular_target_pose(Pose goal) const;
    Pose perpendicular_seam_pose(Pose target) const;
    std::vector<Waypoint> perpendicular_visualization_path(
        double start_x,
        double start_y,
        double start_heading,
        Pose target,
        Pose seam
    ) const;

    MotionCommand parallel_command(const VehicleState& state);
    MotionCommand perpendicular_command(const VehicleState& state);
};

}  // namespace parking_adas
