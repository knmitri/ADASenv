#include "parking_adas/planning/trajectory_generator.hpp"

#include <algorithm>

namespace parking_adas {

std::vector<Waypoint> SpeedProfileTrajectoryGenerator::generate(
    std::vector<Waypoint> path, const VehicleParameters& vehicle) const {
    constexpr double kForwardSpeed = 0.25;
    constexpr double kReverseSpeed = 0.25;
    for (std::size_t index = 0; index < path.size(); ++index) {
        if (index > 0 && path[index].direction != path[index - 1].direction) {
            path[index - 1].gear_change = true;
            path[index - 1].target_speed = 0.0;
        }
        if (index + 1 == path.size()) {
            path[index].target_speed = 0.0;
        } else if (!path[index].gear_change) {
            path[index].target_speed = path[index].direction < 0
                ? -std::min(vehicle.max_reverse_speed, kReverseSpeed)
                : std::min(vehicle.max_speed, kForwardSpeed);
        }
    }
    return path;
}

}  // namespace parking_adas
