#include "parking_adas/planning/path_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace parking_adas {

std::vector<Waypoint> bezier_path(
    double start_x,
    double start_y,
    double goal_x,
    double goal_y,
    double start_heading,
    double goal_heading,
    int direction,
    int samples
) {
    const double chord = std::max(1e-6, std::hypot(goal_x - start_x, goal_y - start_y));
    const double handle = std::min(std::max(chord * 0.5, 2.0), 7.0);
    const double p0x = start_x;
    const double p0y = start_y;
    const double p1x = start_x + handle * std::cos(start_heading);
    const double p1y = start_y + handle * std::sin(start_heading);
    const double p2x = goal_x - handle * std::cos(goal_heading);
    const double p2y = goal_y - handle * std::sin(goal_heading);
    const double p3x = goal_x;
    const double p3y = goal_y;

    std::vector<Waypoint> path;
    path.reserve(static_cast<std::size_t>(samples));
    for (int i = 0; i < samples; ++i) {
        const double t = samples == 1 ? 1.0 : static_cast<double>(i) / static_cast<double>(samples - 1);
        const double u = 1.0 - t;
        Waypoint wp;
        wp.x = u * u * u * p0x + 3.0 * u * u * t * p1x + 3.0 * u * t * t * p2x + t * t * t * p3x;
        wp.y = u * u * u * p0y + 3.0 * u * u * t * p1y + 3.0 * u * t * t * p2y + t * t * t * p3y;
        wp.direction = direction;
        wp.heading = goal_heading;
        path.push_back(wp);
    }
    return path;
}

}  // namespace parking_adas
