#pragma once

#include <array>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

using Polygon = std::array<Point2, 4>;

Polygon vehicle_footprint(const Pose& pose, const VehicleParameters& vehicle,
                          double clearance = 0.0);
Polygon obstacle_footprint(const Obstacle& obstacle);
Polygon footprint_polygon(const Footprint& footprint);
bool valid_convex_polygon(const Polygon& polygon);
bool polygons_overlap(const Polygon& first, const Polygon& second);
double polygon_distance(const Polygon& first, const Polygon& second);
bool polygon_contains_polygon(const Polygon& container, const Polygon& inner);

}  // namespace parking_adas
