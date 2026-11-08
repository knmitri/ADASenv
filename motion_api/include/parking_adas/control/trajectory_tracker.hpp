#pragma once

#include <memory>
#include <string>
#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class TrajectoryTracker {
public:
    virtual ~TrajectoryTracker() = default;

    virtual void reset(const std::vector<Waypoint>& trajectory, VehicleParameters vehicle) = 0;
    virtual MotionCommand command(const VehicleState& state) = 0;
    virtual TrackingStatus assess(const VehicleState& state) = 0;
    virtual const char* name() const = 0;
};

std::unique_ptr<TrajectoryTracker> make_trajectory_tracker(const std::string& tracker_id);

}  // namespace parking_adas
