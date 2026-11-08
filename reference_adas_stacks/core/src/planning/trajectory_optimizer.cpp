#include "parking_adas/planning/trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>

namespace parking_adas {

std::vector<Waypoint> KinematicTrajectoryOptimizer::optimize(
    std::vector<Waypoint> trajectory, const PlanRequest& request) const {
    const double maximum_curvature = std::tan(request.vehicle.max_steering_angle) /
        request.vehicle.wheelbase;
    for (Waypoint& point : trajectory) {
        point.curvature = std::max(-maximum_curvature,
                                   std::min(maximum_curvature, point.curvature));
        point.target_speed = std::max(-request.vehicle.max_reverse_speed,
                                      std::min(request.vehicle.max_speed, point.target_speed));
    }
    return trajectory;
}

}  // namespace parking_adas
