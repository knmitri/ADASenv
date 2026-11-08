#pragma once

#include <cstddef>
#include <vector>

#include "parking_adas/control/trajectory_tracker.hpp"

namespace parking_adas {

class PurePursuitTrajectoryTracker final : public TrajectoryTracker {
public:
    void reset(const std::vector<Waypoint>& trajectory, VehicleParameters vehicle) override;
    MotionCommand command(const VehicleState& state) override;
    TrackingStatus assess(const VehicleState& state) override;
    const char* name() const override { return "pure_pursuit"; }

private:
    std::vector<Waypoint> trajectory_;
    VehicleParameters vehicle_;
    std::size_t progress_index_ = 0;
    std::size_t stall_cycles_ = 0;
    std::size_t terminal_replan_cycles_ = 0;
    Point2 previous_position_{};
    bool has_previous_position_ = false;
};

}  // namespace parking_adas
