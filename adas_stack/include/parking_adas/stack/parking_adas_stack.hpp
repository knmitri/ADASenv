#pragma once

#include <vector>

#include "parking_adas/control/trajectory_tracker.hpp"
#include "parking_adas/domain/types.hpp"
#include "parking_adas/planning/hybrid_a_star.hpp"

namespace parking_adas {

class ParkingAdasStack {
public:
    std::vector<Waypoint> reset(const PlanRequest& request);
    MotionCommand command(const VehicleState& state);
    const std::vector<Waypoint>& trajectory() const { return trajectory_; }
    const PlanningDiagnostics& planning_diagnostics() const { return planner_.diagnostics(); }

private:
    VehicleParameters vehicle_;
    TrajectoryTracker trajectory_tracker_;
    std::vector<Waypoint> trajectory_;
    PlanRequest request_;
    HybridAStarPlanner planner_;
    std::vector<Waypoint> generate_trajectory(std::vector<Waypoint> path) const;

};

}  // namespace parking_adas
