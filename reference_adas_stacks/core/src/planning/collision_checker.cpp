#include "parking_adas/planning/collision_checker.hpp"

#include <algorithm>
#include <cmath>

#include "parking_adas/common/math.hpp"
#include "parking_adas/common/footprint_geometry.hpp"

namespace parking_adas {

namespace {

bool valid_vehicle(const VehicleParameters& vehicle, double clearance) {
    return std::isfinite(vehicle.width) && vehicle.width > 0.0 &&
        std::isfinite(vehicle.wheelbase) && vehicle.wheelbase > 0.0 &&
        std::isfinite(vehicle.front_overhang) && vehicle.front_overhang >= 0.0 &&
        std::isfinite(vehicle.rear_overhang) && vehicle.rear_overhang >= 0.0 &&
        std::isfinite(vehicle.length) && vehicle.length > 0.0 &&
        std::isfinite(clearance) && clearance >= 0.0;
}

bool valid_world(const PlanRequest& request) {
    return std::isfinite(request.world_min_x) && std::isfinite(request.world_min_y) &&
        std::isfinite(request.world_max_x) && std::isfinite(request.world_max_y) &&
        request.world_min_x < request.world_max_x &&
        request.world_min_y < request.world_max_y;
}

}  // namespace

CollisionChecker::CollisionChecker(const PlanRequest& request, double clearance)
    : request_(request), clearance_(clearance) {
    obstacle_polygons_.reserve(request_.static_obstacles.size());
    for (const Obstacle& obstacle : request_.static_obstacles) {
        if (!obstacle.footprint.empty() && obstacle.footprint.size() != 4) {
            invalid_geometry_ = true;
            continue;
        }
        if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
            !std::isfinite(obstacle.yaw)) {
            invalid_geometry_ = true;
            continue;
        }
        if (obstacle.footprint.empty() &&
            (!std::isfinite(obstacle.length) || obstacle.length <= 0.0 ||
             !std::isfinite(obstacle.width) || obstacle.width <= 0.0)) {
            invalid_geometry_ = true;
            continue;
        }
        const Polygon polygon = obstacle_footprint(obstacle);
        if (!valid_convex_polygon(polygon)) {
            invalid_geometry_ = true;
            continue;
        }
        obstacle_polygons_.push_back(polygon);
    }
}

bool CollisionChecker::is_collision(const Pose& pose) const {
    ++pose_checks_;
    if (invalid_geometry_ || !valid_vehicle(request_.vehicle, clearance_) ||
        !valid_world(request_) || !std::isfinite(pose.x) ||
        !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
        return true;
    }
    const Polygon ego = vehicle_footprint(pose, request_.vehicle, clearance_);
    for (const Point2& corner : ego) {
        if (corner.x < request_.world_min_x || corner.x > request_.world_max_x ||
            corner.y < request_.world_min_y || corner.y > request_.world_max_y) {
            return true;
        }
    }
    for (const Polygon& obstacle : obstacle_polygons_) {
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

    const double half_width = request_.vehicle.width / 2.0 + clearance_;
    const double front_extent =
        request_.vehicle.wheelbase + request_.vehicle.front_overhang + clearance_;
    const double rear_extent = request_.vehicle.rear_overhang + clearance_;
    const double radius = std::hypot(std::max(front_extent, rear_extent), half_width);
    const double swept_bound = translation_bound +
        radius * heading_change_bound + kBoundTolerance;
    const Polygon ego = vehicle_footprint(from, request_.vehicle, clearance_);

    for (const Point2& corner : ego) {
        if (std::min({corner.x - request_.world_min_x,
                      request_.world_max_x - corner.x,
                      corner.y - request_.world_min_y,
                      request_.world_max_y - corner.y}) <= swept_bound) {
            return true;
        }
    }

    for (const Polygon& obstacle : obstacle_polygons_) {
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
