#include <cmath>
#include <stdexcept>

#include "parking_adas/common/math.hpp"

int main() {
    constexpr double test_wheelbase = 2.82;
    for (const double steering_angle : {-0.6, -0.22, 0.0, 0.22, 0.6}) {
        const double expected_curvature = std::tan(steering_angle) / test_wheelbase;
        const double actual_curvature =
            parking_adas::curvature_from_steering_angle(
                steering_angle, test_wheelbase);
        if (actual_curvature != expected_curvature) {
            throw std::runtime_error(
                "shared steering-to-curvature conversion changed its convention");
        }
    }
    const double positive_curvature =
        parking_adas::curvature_from_steering_angle(0.22, test_wheelbase);
    const double negative_curvature =
        parking_adas::curvature_from_steering_angle(-0.22, test_wheelbase);
    if (!(positive_curvature > 0.0 && negative_curvature < 0.0)) {
        throw std::runtime_error("steering sign does not match curvature sign");
    }

    const parking_adas::Pose origin{};
    const parking_adas::Pose forward_left =
        parking_adas::integrate_pose_by_distance(
            origin, 0.22, 0.1, test_wheelbase, 0.1);
    const parking_adas::Pose reverse_left =
        parking_adas::integrate_pose_by_distance(
            origin, 0.22, -0.1, test_wheelbase, 0.1);
    if (!(forward_left.yaw > 0.0 && reverse_left.yaw < 0.0)) {
        throw std::runtime_error(
            "reverse travel did not reverse the yaw response for the same steering angle");
    }

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
