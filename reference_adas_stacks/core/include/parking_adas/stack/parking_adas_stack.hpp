#pragma once

#include <memory>
#include <vector>

#include "parking_adas/control/trajectory_tracker.hpp"
#include "parking_adas/domain/types.hpp"
#include "parking_adas/planning/path_planner.hpp"
#include "parking_adas/planning/trajectory_generator.hpp"
#include "parking_adas/planning/trajectory_optimizer.hpp"

namespace parking_adas {

class ParkingAdasStack {
public:
    ParkingAdasStack();
    explicit ParkingAdasStack(
        std::unique_ptr<PathPlanner> planner,
        std::unique_ptr<TrajectoryTracker> trajectory_tracker =
            make_trajectory_tracker("pure_pursuit"),
        std::unique_ptr<TrajectoryGenerator> trajectory_generator =
            std::make_unique<SpeedProfileTrajectoryGenerator>(),
        std::unique_ptr<TrajectoryOptimizer> trajectory_optimizer =
            std::make_unique<KinematicTrajectoryOptimizer>());
    std::vector<Waypoint> reset(const PlanRequest& request);
    MotionCommand command(const VehicleState& state);
    const std::vector<Waypoint>& trajectory() const { return trajectory_; }
    const PlanningDiagnostics& planning_diagnostics() const { return planner_->diagnostics(); }
    const CommandDiagnostics& command_diagnostics() const { return command_diagnostics_; }

private:
    VehicleParameters vehicle_;
    std::unique_ptr<TrajectoryTracker> trajectory_tracker_;
    std::vector<Waypoint> trajectory_;
    PlanRequest request_;
    std::unique_ptr<PathPlanner> planner_;
    std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
    std::unique_ptr<TrajectoryOptimizer> trajectory_optimizer_;
    CommandDiagnostics command_diagnostics_;
    std::vector<Waypoint> generate_trajectory(std::vector<Waypoint> path) const;

};

}  // namespace parking_adas
