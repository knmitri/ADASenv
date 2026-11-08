#pragma once

#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class TrajectoryGenerator {
public:
    virtual ~TrajectoryGenerator() = default;
    virtual std::vector<Waypoint> generate(std::vector<Waypoint> path,
                                           const VehicleParameters& vehicle) const = 0;
};

class SpeedProfileTrajectoryGenerator final : public TrajectoryGenerator {
public:
    std::vector<Waypoint> generate(std::vector<Waypoint> path,
                                   const VehicleParameters& vehicle) const override;
};

}  // namespace parking_adas
