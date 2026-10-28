#pragma once

#include <cstddef>
#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class TrajectoryTracker {
public:
    void reset(const std::vector<Waypoint>& trajectory, VehicleParameters vehicle);
    MotionCommand command(const VehicleState& state);
    bool needs_replan(const VehicleState& state, double position_threshold, double heading_threshold);

private:
    std::vector<Waypoint> trajectory_;
    VehicleParameters vehicle_;
    std::size_t progress_index_ = 0;
    std::size_t stall_cycles_ = 0;
    std::size_t terminal_replan_cycles_ = 0;
};

}  // namespace parking_adas
