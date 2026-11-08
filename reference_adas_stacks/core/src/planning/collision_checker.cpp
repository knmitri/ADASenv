#include "parking_adas/planning/collision_checker.hpp"

#include "parking_adas/common/footprint_geometry.hpp"

namespace parking_adas {
CollisionChecker::CollisionChecker(const PlanRequest& request, double clearance)
    : request_(request), clearance_(clearance) {}

bool CollisionChecker::is_collision(const Pose& pose) const {
    const Polygon ego = vehicle_footprint(pose, request_.vehicle, clearance_);
    for (const Point2& corner : ego) {
        if (corner.x < request_.world_min_x || corner.x > request_.world_max_x ||
            corner.y < request_.world_min_y || corner.y > request_.world_max_y) return true;
    }
    for (const Obstacle& obstacle : request_.static_obstacles) {
        if (polygons_overlap(ego, obstacle_footprint(obstacle))) return true;
    }
    return false;
}

}  // namespace parking_adas
