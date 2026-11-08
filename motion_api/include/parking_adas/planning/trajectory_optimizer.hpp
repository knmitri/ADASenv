#pragma once

#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class TrajectoryOptimizer {
public:
    virtual ~TrajectoryOptimizer() = default;
    virtual std::vector<Waypoint> optimize(std::vector<Waypoint> trajectory,
                                           const PlanRequest& request) const = 0;
};

class KinematicTrajectoryOptimizer final : public TrajectoryOptimizer {
public:
    std::vector<Waypoint> optimize(std::vector<Waypoint> trajectory,
                                   const PlanRequest& request) const override;
};

}  // namespace parking_adas
