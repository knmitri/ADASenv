#include "parking_adas/common/footprint_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace parking_adas {
namespace {

Polygon rectangle(double x, double y, double yaw, double front_extent, double width,
                  double rear_extent) {
    const std::array<Point2, 4> local{{
        {-rear_extent, -width / 2.0}, {-rear_extent, width / 2.0},
        {front_extent, width / 2.0}, {front_extent, -width / 2.0},
    }};
    Polygon result{};
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    for (std::size_t index = 0; index < local.size(); ++index) {
        result[index] = {
            x + local[index].x * cosine - local[index].y * sine,
            y + local[index].x * sine + local[index].y * cosine,
        };
    }
    return result;
}

bool point_in_convex_polygon(const Point2& point, const Polygon& polygon) {
    bool has_positive = false;
    bool has_negative = false;
    constexpr double epsilon = 1e-9;
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const Point2& first = polygon[index];
        const Point2& second = polygon[(index + 1) % polygon.size()];
        const double cross = (second.x - first.x) * (point.y - first.y) -
            (second.y - first.y) * (point.x - first.x);
        has_positive = has_positive || cross > epsilon;
        has_negative = has_negative || cross < -epsilon;
        if (has_positive && has_negative) return false;
    }
    return true;
}

}  // namespace

Polygon vehicle_footprint(const Pose& pose, const VehicleParameters& vehicle, double clearance) {
    return rectangle(
        pose.x, pose.y, pose.yaw,
        vehicle.wheelbase + vehicle.front_overhang + clearance,
        vehicle.width + 2.0 * clearance,
        vehicle.rear_overhang + clearance
    );
}

Polygon obstacle_footprint(const Obstacle& obstacle) {
    return obstacle.footprint.size() == 4
        ? footprint_polygon(obstacle.footprint)
        : rectangle(obstacle.x, obstacle.y, obstacle.yaw, obstacle.length / 2.0,
                    obstacle.width, obstacle.length / 2.0);
}

Polygon footprint_polygon(const Footprint& footprint) {
    Polygon result{};
    for (std::size_t index = 0; index < result.size() && index < footprint.size(); ++index) {
        result[index] = footprint[index];
    }
    return result;
}

bool polygons_overlap(const Polygon& first, const Polygon& second) {
    for (const Polygon* polygon : {&first, &second}) {
        for (std::size_t index = 0; index < polygon->size(); ++index) {
            const Point2& first_edge_point = polygon->at(index);
            const Point2& second_edge_point = polygon->at((index + 1) % polygon->size());
            const double axis_x = -(second_edge_point.y - first_edge_point.y);
            const double axis_y = second_edge_point.x - first_edge_point.x;
            double first_min = std::numeric_limits<double>::infinity();
            double first_max = -first_min;
            double second_min = first_min;
            double second_max = -first_min;
            for (const Point2& point : first) {
                const double projection = point.x * axis_x + point.y * axis_y;
                first_min = std::min(first_min, projection);
                first_max = std::max(first_max, projection);
            }
            for (const Point2& point : second) {
                const double projection = point.x * axis_x + point.y * axis_y;
                second_min = std::min(second_min, projection);
                second_max = std::max(second_max, projection);
            }
            if (first_max < second_min || second_max < first_min) return false;
        }
    }
    return true;
}

bool polygon_contains_polygon(const Polygon& container, const Polygon& inner) {
    return std::all_of(inner.begin(), inner.end(), [&](const Point2& point) {
        return point_in_convex_polygon(point, container);
    });
}

}  // namespace parking_adas
