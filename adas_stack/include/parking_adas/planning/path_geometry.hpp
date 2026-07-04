#pragma once

#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

std::vector<Waypoint> bezier_path(
    double start_x,
    double start_y,
    double goal_x,
    double goal_y,
    double start_heading,
    double goal_heading,
    int direction,
    int samples = 48
);

}  // namespace parking_adas
