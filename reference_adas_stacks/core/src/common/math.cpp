#include "parking_adas/common/math.hpp"

#include <algorithm>
#include <cmath>

namespace parking_adas {

double clamp_value(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double curvature_from_steering_angle(double steering_angle_radians,
                                     double wheelbase_metres) {
    return std::tan(steering_angle_radians) / wheelbase_metres;
}

Pose integrate_pose_by_distance(
    Pose pose,
    double steering,
    double signed_distance,
    double wheelbase,
    double step
) {
    const int steps = std::max(1, static_cast<int>(std::abs(signed_distance) / step));
    const double ds = signed_distance / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
        pose.x += ds * std::cos(pose.yaw);
        pose.y += ds * std::sin(pose.yaw);
        pose.yaw = normalize_angle(pose.yaw + ds * std::tan(steering) / std::max(wheelbase, 1e-3));
    }
    return pose;
}

}  // namespace parking_adas
