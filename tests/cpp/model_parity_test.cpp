#include <cmath>
#include <stdexcept>

#include "parking_adas/common/math.hpp"

int main() {
    const parking_adas::Pose initial{1.25, -0.75, 0.4};
    const double velocity = -0.6;
    const double steering = 0.22;
    const double dt = 0.05;
    const double wheelbase = 2.82;
    const parking_adas::Pose actual = parking_adas::integrate_pose_by_distance(
        initial, steering, velocity * dt, wheelbase, dt);
    // Expected values are the independent Python bicycle-model fixture.
    if (std::abs(actual.x - 1.2223681702) >= 1e-9 ||
        std::abs(actual.y - -0.7616825503) >= 1e-9 ||
        std::abs(parking_adas::normalize_angle(actual.yaw - 0.3976210700)) >= 1e-9) {
        throw std::runtime_error("C++ bicycle integration diverges from the Python fixture");
    }
    return 0;
}
