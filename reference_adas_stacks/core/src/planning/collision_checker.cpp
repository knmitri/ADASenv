#include "parking_adas/planning/collision_checker.hpp"

#include <algorithm>
#include <cmath>

#include "parking_adas/common/math.hpp"
#include "parking_adas/common/footprint_geometry.hpp"

namespace parking_adas {

CollisionChecker::CollisionChecker(const CollisionScene& scene)
    : scene_(scene) {}

bool CollisionChecker::is_collision(const Pose& pose) const {
    ++pose_checks_;
    if (scene_.invalid_geometry() || !scene_.valid() || !std::isfinite(pose.x) ||
        !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
        return true;
    }
    const Polygon ego = vehicle_footprint(pose, scene_.vehicle(), scene_.clearance());
    for (const Point2& corner : ego) {
        if (corner.x < scene_.world_min_x() || corner.x > scene_.world_max_x() ||
            corner.y < scene_.world_min_y() || corner.y > scene_.world_max_y()) {
            return true;
        }
    }
    for (const Polygon& obstacle : scene_.obstacle_polygons()) {
        if (polygons_overlap(ego, obstacle)) return true;
    }
    return false;
}

bool CollisionChecker::is_swept_collision(
    const Pose& from, const Pose& to, double translation_bound,
    double heading_change_bound) const {
    ++swept_checks_;
    if (!std::isfinite(translation_bound) || translation_bound < 0.0 ||
        !std::isfinite(heading_change_bound) || heading_change_bound < 0.0 ||
        !std::isfinite(from.x) || !std::isfinite(from.y) ||
        !std::isfinite(from.yaw) || !std::isfinite(to.x) ||
        !std::isfinite(to.y) || !std::isfinite(to.yaw)) {
        return true;
    }
    const double center_displacement = std::hypot(to.x - from.x, to.y - from.y);
    const double actual_heading_change = std::abs(normalize_angle(to.yaw - from.yaw));
    constexpr double kBoundTolerance = 1e-10;
    if (center_displacement > translation_bound + kBoundTolerance ||
        actual_heading_change > heading_change_bound + kBoundTolerance) {
        return true;
    }
    if (is_collision(from) || is_collision(to)) return true;

    const VehicleParameters& vehicle = scene_.vehicle();
    const double clearance = scene_.clearance();
    const double half_width = vehicle.width / 2.0 + clearance;
    const double front_extent =
        vehicle.wheelbase + vehicle.front_overhang + clearance;
    const double rear_extent = vehicle.rear_overhang + clearance;
    const double radius = std::hypot(std::max(front_extent, rear_extent), half_width);
    const double swept_bound = translation_bound +
        radius * heading_change_bound + kBoundTolerance;
    const Polygon ego = vehicle_footprint(from, vehicle, clearance);

    for (const Point2& corner : ego) {
        if (std::min({corner.x - scene_.world_min_x(),
                      scene_.world_max_x() - corner.x,
                      corner.y - scene_.world_min_y(),
                      scene_.world_max_y() - corner.y}) <= swept_bound) {
            return true;
        }
    }

    for (const Polygon& obstacle : scene_.obstacle_polygons()) {
        ++obstacle_distance_checks_;
        if (polygon_distance(ego, obstacle) <= swept_bound) return true;
    }
    return false;
}

CollisionChecker::Statistics CollisionChecker::statistics() const {
    return {
        pose_checks_.load(),
        swept_checks_.load(),
        obstacle_distance_checks_.load(),
    };
}

void CollisionChecker::reset_statistics() const {
    pose_checks_ = 0;
    swept_checks_ = 0;
    obstacle_distance_checks_ = 0;
}

}  // namespace parking_adas
