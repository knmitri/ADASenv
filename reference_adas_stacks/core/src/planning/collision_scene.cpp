#include "parking_adas/planning/collision_scene.hpp"

#include <cmath>
#include <utility>

namespace parking_adas {

struct CollisionScene::Data {
    VehicleParameters vehicle;
    double clearance;
    double world_min_x;
    double world_min_y;
    double world_max_x;
    double world_max_y;
    bool valid;
    bool invalid_geometry;
    std::vector<Polygon> obstacle_polygons;
};

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

std::shared_ptr<const CollisionScene::Data> CollisionScene::make_data(
    const PlanRequest& request,
    double clearance
) {
    auto data = std::make_shared<CollisionScene::Data>();
    data->vehicle = request.vehicle;
    data->clearance = clearance;
    data->world_min_x = request.world_min_x;
    data->world_min_y = request.world_min_y;
    data->world_max_x = request.world_max_x;
    data->world_max_y = request.world_max_y;
    data->valid = valid_vehicle(request.vehicle, clearance) && valid_world(request);
    data->invalid_geometry = false;
    data->obstacle_polygons.reserve(request.static_obstacles.size());
    for (const Obstacle& obstacle : request.static_obstacles) {
        if (!obstacle.footprint.empty() && obstacle.footprint.size() != 4) {
            data->invalid_geometry = true;
            continue;
        }
        if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
            !std::isfinite(obstacle.yaw)) {
            data->invalid_geometry = true;
            continue;
        }
        if (obstacle.footprint.empty() &&
            (!std::isfinite(obstacle.length) || obstacle.length <= 0.0 ||
             !std::isfinite(obstacle.width) || obstacle.width <= 0.0)) {
            data->invalid_geometry = true;
            continue;
        }
        const Polygon polygon = obstacle_footprint(obstacle);
        if (!valid_convex_polygon(polygon)) {
            data->invalid_geometry = true;
            continue;
        }
        data->obstacle_polygons.push_back(polygon);
    }
    return data;
}

CollisionScene::CollisionScene(const PlanRequest& request, double clearance)
    : data_(make_data(request, clearance)) {}

const VehicleParameters& CollisionScene::vehicle() const {
    return data_->vehicle;
}

double CollisionScene::clearance() const {
    return data_->clearance;
}

double CollisionScene::world_min_x() const {
    return data_->world_min_x;
}

double CollisionScene::world_min_y() const {
    return data_->world_min_y;
}

double CollisionScene::world_max_x() const {
    return data_->world_max_x;
}

double CollisionScene::world_max_y() const {
    return data_->world_max_y;
}

bool CollisionScene::valid() const {
    return data_->valid;
}

bool CollisionScene::invalid_geometry() const {
    return data_->invalid_geometry;
}

const std::vector<Polygon>& CollisionScene::obstacle_polygons() const {
    return data_->obstacle_polygons;
}

}  // namespace parking_adas
