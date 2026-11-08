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

double cross(const Point2& first, const Point2& second, const Point2& point) {
    return (second.x - first.x) * (point.y - first.y) -
        (second.y - first.y) * (point.x - first.x);
}

double squared_distance(const Point2& first, const Point2& second) {
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return dx * dx + dy * dy;
}

double point_segment_distance_squared(const Point2& point, const Point2& first,
                                      const Point2& second) {
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 1e-24) return squared_distance(point, first);
    const double projection = std::clamp(
        ((point.x - first.x) * dx + (point.y - first.y) * dy) / length_squared,
        0.0, 1.0);
    return squared_distance(
        point, {first.x + projection * dx, first.y + projection * dy});
}

bool segments_intersect(const Point2& a, const Point2& b,
                        const Point2& c, const Point2& d) {
    constexpr double epsilon = 1e-12;
    const double ab_c = cross(a, b, c);
    const double ab_d = cross(a, b, d);
    const double cd_a = cross(c, d, a);
    const double cd_b = cross(c, d, b);
    if (((ab_c > epsilon && ab_d < -epsilon) ||
         (ab_c < -epsilon && ab_d > epsilon)) &&
        ((cd_a > epsilon && cd_b < -epsilon) ||
         (cd_a < -epsilon && cd_b > epsilon))) {
        return true;
    }
    return (std::abs(ab_c) <= epsilon &&
            point_segment_distance_squared(c, a, b) <= epsilon * epsilon) ||
        (std::abs(ab_d) <= epsilon &&
         point_segment_distance_squared(d, a, b) <= epsilon * epsilon) ||
        (std::abs(cd_a) <= epsilon &&
         point_segment_distance_squared(a, c, d) <= epsilon * epsilon) ||
        (std::abs(cd_b) <= epsilon &&
         point_segment_distance_squared(b, c, d) <= epsilon * epsilon);
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

bool valid_convex_polygon(const Polygon& polygon) {
    for (const Point2& point : polygon) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    }
    double twice_area = 0.0;
    bool has_positive_turn = false;
    bool has_negative_turn = false;
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const Point2& current = polygon[index];
        const Point2& next = polygon[(index + 1) % polygon.size()];
        const Point2& after = polygon[(index + 2) % polygon.size()];
        if (squared_distance(current, next) <= 1e-24) return false;
        twice_area += current.x * next.y - current.y * next.x;
        const double turn = cross(current, next, after);
        has_positive_turn = has_positive_turn || turn > 1e-12;
        has_negative_turn = has_negative_turn || turn < -1e-12;
        if (has_positive_turn && has_negative_turn) return false;
    }
    return std::abs(twice_area) > 1e-12 &&
        (has_positive_turn || has_negative_turn);
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

double polygon_distance(const Polygon& first, const Polygon& second) {
    if (polygons_overlap(first, second)) return 0.0;
    double minimum_squared = std::numeric_limits<double>::infinity();
    for (std::size_t first_index = 0; first_index < first.size(); ++first_index) {
        const Point2& first_a = first[first_index];
        const Point2& first_b = first[(first_index + 1) % first.size()];
        for (std::size_t second_index = 0; second_index < second.size();
             ++second_index) {
            const Point2& second_a = second[second_index];
            const Point2& second_b = second[(second_index + 1) % second.size()];
            if (segments_intersect(first_a, first_b, second_a, second_b)) return 0.0;
            minimum_squared = std::min(minimum_squared,
                point_segment_distance_squared(first_a, second_a, second_b));
            minimum_squared = std::min(minimum_squared,
                point_segment_distance_squared(first_b, second_a, second_b));
            minimum_squared = std::min(minimum_squared,
                point_segment_distance_squared(second_a, first_a, first_b));
            minimum_squared = std::min(minimum_squared,
                point_segment_distance_squared(second_b, first_a, first_b));
        }
    }
    return std::sqrt(minimum_squared);
}

bool polygon_contains_polygon(const Polygon& container, const Polygon& inner) {
    return std::all_of(inner.begin(), inner.end(), [&](const Point2& point) {
        return point_in_convex_polygon(point, container);
    });
}

}  // namespace parking_adas
