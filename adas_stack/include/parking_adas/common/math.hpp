#pragma once

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

constexpr double kPi = 3.14159265358979323846;

double clamp_value(double value, double low, double high);
double normalize_angle(double angle);

Pose integrate_pose_by_distance(
    Pose pose,
    double steering,
    double signed_distance,
    double wheelbase,
    double step = 0.05
);

}  // namespace parking_adas
