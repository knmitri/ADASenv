#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "parking_adas/common/math.hpp"
#include "parking_adas/planning/collision_checker.hpp"
#include "parking_adas/planning/parking_goal_evaluator.hpp"
#include "parking_adas/planning/planning_context.hpp"
#include "parking_adas/planning/reeds_shepp_connector.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSampleSpacing = 0.10;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

parking_adas::Pose integrate_segment(
    parking_adas::Pose pose, parking_adas::ReedsSheppSegmentType type,
    double signed_length, double radius) {
    double curvature = 0.0;
    if (type == parking_adas::ReedsSheppSegmentType::Left) {
        curvature = 1.0 / radius;
    } else if (type == parking_adas::ReedsSheppSegmentType::Right) {
        curvature = -1.0 / radius;
    }
    const double next_yaw = pose.yaw + curvature * signed_length;
    if (curvature == 0.0) {
        pose.x += signed_length * std::cos(pose.yaw);
        pose.y += signed_length * std::sin(pose.yaw);
    } else {
        pose.x += (std::sin(next_yaw) - std::sin(pose.yaw)) / curvature;
        pose.y += (-std::cos(next_yaw) + std::cos(pose.yaw)) / curvature;
    }
    pose.yaw = parking_adas::normalize_angle(next_yaw);
    return pose;
}

parking_adas::Pose integrate_path(
    const parking_adas::Pose& start,
    const parking_adas::ReedsSheppPath& path, double radius) {
    parking_adas::Pose pose = start;
    for (const parking_adas::ReedsSheppSegment& segment : path.segments) {
        pose = integrate_segment(pose, segment.type, segment.signed_length, radius);
    }
    return pose;
}

int direction_of(double signed_length) {
    return signed_length > 1e-10 ? 1 : signed_length < -1e-10 ? -1 : 0;
}

int first_direction(const parking_adas::ReedsSheppPath& path) {
    for (const parking_adas::ReedsSheppSegment& segment : path.segments) {
        const int direction = direction_of(segment.signed_length);
        if (direction != 0) return direction;
    }
    return 0;
}

int cusp_count(const parking_adas::ReedsSheppPath& path) {
    int cusps = 0;
    int previous = 0;
    for (const parking_adas::ReedsSheppSegment& segment : path.segments) {
        const int direction = direction_of(segment.signed_length);
        if (direction == 0) continue;
        if (previous != 0 && direction != previous) ++cusps;
        previous = direction;
    }
    return cusps;
}

parking_adas::PlanRequest open_request(const parking_adas::Pose& goal) {
    parking_adas::PlanRequest request;
    request.goal = goal;
    request.goal_heading_tolerance = kPi;
    request.goal_slot_footprint = {
        {-2000000.0, -2000000.0}, {2000000.0, -2000000.0},
        {2000000.0, 2000000.0}, {-2000000.0, 2000000.0},
    };
    request.world_min_x = -2000000.0;
    request.world_min_y = -2000000.0;
    request.world_max_x = 2000000.0;
    request.world_max_y = 2000000.0;
    return request;
}

parking_adas::Footprint square(double x, double y, double half_size) {
    return {{x - half_size, y - half_size}, {x + half_size, y - half_size},
            {x + half_size, y + half_size}, {x - half_size, y + half_size}};
}

std::vector<parking_adas::Pose> sample_path(
    const parking_adas::Pose& start,
    const parking_adas::ReedsSheppPath& path, double radius) {
    std::vector<parking_adas::Pose> poses{start};
    parking_adas::Pose pose = start;
    for (const parking_adas::ReedsSheppSegment& segment : path.segments) {
        const double requested = std::ceil(std::abs(segment.signed_length) /
                                           kSampleSpacing);
        const int count = std::max(1, static_cast<int>(requested));
        const double step = segment.signed_length / count;
        for (int index = 0; index < count; ++index) {
            pose = integrate_segment(pose, segment.type, step, radius);
            poses.push_back(pose);
        }
    }
    return poses;
}

bool clear_path(const parking_adas::Pose& start,
                const parking_adas::ReedsSheppPath& path, double radius,
                const parking_adas::CollisionChecker& checker) {
    for (const parking_adas::Pose& pose : sample_path(start, path, radius)) {
        if (checker.is_collision(pose)) return false;
    }
    return true;
}

double polyline_length(const parking_adas::Pose& start,
                       const std::vector<parking_adas::Waypoint>& path) {
    if (path.empty()) return 0.0;
    double length = std::hypot(path.front().x - start.x, path.front().y - start.y);
    for (std::size_t index = 1; index < path.size(); ++index) {
        length += std::hypot(path[index].x - path[index - 1].x,
                             path[index].y - path[index - 1].y);
    }
    return length;
}

bool same_candidate(const parking_adas::ReedsSheppPath& first,
                    const parking_adas::ReedsSheppPath& second,
                    bool reflection, bool time_flip) {
    if (first.segments.size() != second.segments.size()) return false;
    for (std::size_t index = 0; index < first.segments.size(); ++index) {
        auto expected_type = first.segments[index].type;
        if (reflection) {
            if (expected_type == parking_adas::ReedsSheppSegmentType::Left) {
                expected_type = parking_adas::ReedsSheppSegmentType::Right;
            } else if (expected_type == parking_adas::ReedsSheppSegmentType::Right) {
                expected_type = parking_adas::ReedsSheppSegmentType::Left;
            }
        }
        const double expected_length = first.segments[index].signed_length *
            (time_flip ? -1.0 : 1.0);
        if (second.segments[index].type != expected_type ||
            std::abs(second.segments[index].signed_length - expected_length) > 1e-7) {
            return false;
        }
    }
    return true;
}

void test_candidate_invariants() {
    constexpr double radius = 2.82;
    const parking_adas::Pose start{1.25, -2.5, 0.7};
    const parking_adas::Pose goal{4.0, 1.2, -2.1};
    const parking_adas::ReedsSheppConnector connector;
    const auto candidates = connector.analytical_candidates(start, goal, radius);
    require(!candidates.empty(), "candidate invariant fixture produced no paths");

    double previous_length = -1.0;
    for (const parking_adas::ReedsSheppPath& candidate : candidates) {
        require(candidate.geometric_length >= previous_length,
                "candidate paths are not sorted shortest-first");
        previous_length = candidate.geometric_length;
        double summed_length = 0.0;
        for (const parking_adas::ReedsSheppSegment& segment : candidate.segments) {
            require(std::isfinite(segment.signed_length),
                    "candidate contains a non-finite segment length");
            summed_length += std::abs(segment.signed_length);
            const double curvature =
                segment.type == parking_adas::ReedsSheppSegmentType::Straight
                    ? 0.0 : 1.0 / radius;
            require(std::abs(curvature) <= 1.0 / radius + 1e-12,
                    "candidate exceeds its curvature limit");
        }
        require(std::abs(summed_length - candidate.geometric_length) < 1e-8,
                "geometric length is not the sum of absolute segment lengths");
        const parking_adas::Pose endpoint = integrate_path(start, candidate, radius);
        const double normalized_position_error =
            std::hypot(endpoint.x - goal.x, endpoint.y - goal.y) / radius;
        const double normalized_goal_distance =
            std::hypot(goal.x - start.x, goal.y - start.y) / radius;
        require(normalized_position_error <=
                    1.01e-10 * std::max(1.0, normalized_goal_distance) &&
                    std::abs(parking_adas::normalize_angle(
                        endpoint.yaw - goal.yaw)) <= 1.01e-10,
                "independent segment integration missed the candidate endpoint");
    }

    const double cosine = std::cos(start.yaw);
    const double sine = std::sin(start.yaw);
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double local_x = cosine * dx + sine * dy;
    const double local_y = -sine * dx + cosine * dy;
    const double relative_yaw = parking_adas::normalize_angle(goal.yaw - start.yaw);
    const auto reflected = connector.analytical_candidates(
        start, {start.x + cosine * local_x + sine * local_y,
                start.y + sine * local_x - cosine * local_y,
                parking_adas::normalize_angle(start.yaw - relative_yaw)}, radius);
    const auto time_flipped = connector.analytical_candidates(
        start, {start.x - cosine * local_x - sine * local_y,
                start.y - sine * local_x + cosine * local_y,
                parking_adas::normalize_angle(start.yaw - relative_yaw)}, radius);
    const auto combined = connector.analytical_candidates(
        start, {start.x - cosine * local_x + sine * local_y,
                start.y - sine * local_x - cosine * local_y, goal.yaw}, radius);
    for (const parking_adas::ReedsSheppPath& candidate : candidates) {
        require(std::any_of(reflected.begin(), reflected.end(),
                    [&](const parking_adas::ReedsSheppPath& image) {
                        return same_candidate(candidate, image, true, false);
                    }),
                "reflection did not preserve candidate segment ordering");
        require(std::any_of(time_flipped.begin(), time_flipped.end(),
                    [&](const parking_adas::ReedsSheppPath& image) {
                        return same_candidate(candidate, image, false, true);
                    }),
                "time reversal did not negate lengths without reordering segments");
        require(std::any_of(combined.begin(), combined.end(),
                    [&](const parking_adas::ReedsSheppPath& image) {
                        return same_candidate(candidate, image, true, true);
                    }),
                "combined reflection/time reversal changed segment ordering");
    }

    const auto identity = connector.analytical_candidates(start, start, radius);
    require(!identity.empty() && identity.front().geometric_length < 1e-10,
            "coincident pose lost the zero-length candidate");
    require(std::any_of(identity.front().segments.begin(), identity.front().segments.end(),
                [](const parking_adas::ReedsSheppSegment& segment) {
                    return std::abs(segment.signed_length) <= 1e-10;
                }),
            "degenerate identity path did not retain zero-length segments");
    const auto near_identity = connector.analytical_candidates(
        {}, {1e-9, -1e-9, 1e-8}, radius);
    require(!near_identity.empty() &&
                near_identity.front().geometric_length > 1e-5,
            "near-coincident pose was accepted within an overly loose endpoint tolerance");
}

void test_cusp_markers() {
    constexpr double radius = 2.82;
    const parking_adas::ReedsSheppConnector connector;
    parking_adas::Pose target{};
    bool found = false;
    const std::vector<double> headings{-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0};
    for (int x = -6; x <= 6 && !found; ++x) {
        for (int y = -6; y <= 6 && !found; ++y) {
            for (double yaw : headings) {
                const parking_adas::Pose goal{0.8 * x, 0.8 * y, yaw};
                const auto candidates = connector.analytical_candidates({}, goal, radius);
                const auto feasible = std::find_if(
                    candidates.begin(), candidates.end(),
                    [](const parking_adas::ReedsSheppPath& path) {
                        return first_direction(path) == 1 && cusp_count(path) >= 2;
                    });
                if (feasible != candidates.end()) {
                    target = goal;
                    found = true;
                    break;
                }
            }
        }
    }
    require(found, "no deterministic two-cusp connector fixture was found");

    const parking_adas::PlanRequest request = open_request(target);
    const parking_adas::PlanningContext context(request);
    std::vector<parking_adas::Waypoint> waypoints;
    require(connector.connect({}, target, 1, context,
                std::chrono::steady_clock::now() + std::chrono::seconds(2),
                waypoints, parking_adas::ReedsSheppImplementation::Analytical),
            "connector did not accept the open two-cusp fixture");

    const auto candidates = connector.analytical_candidates({}, target, radius);
    const auto expected = std::find_if(
        candidates.begin(), candidates.end(),
        [](const parking_adas::ReedsSheppPath& path) {
            return first_direction(path) == 1;
        });
    require(expected != candidates.end() && cusp_count(*expected) >= 2,
            "shortest admissible connector path does not contain two cusps");
    const parking_adas::ReedsSheppPath selected = *expected;

    std::vector<parking_adas::Pose> expected_stops;
    parking_adas::Pose pose{};
    int previous_direction = 0;
    for (const parking_adas::ReedsSheppSegment& segment : selected.segments) {
        const int direction = direction_of(segment.signed_length);
        if (direction == 0) continue;
        if (previous_direction != 0 && direction != previous_direction) {
            expected_stops.push_back(pose);
        }
        pose = integrate_segment(pose, segment.type, segment.signed_length, radius);
        previous_direction = direction;
    }

    std::vector<const parking_adas::Waypoint*> stop_markers;
    for (std::size_t index = 0; index < waypoints.size(); ++index) {
        const parking_adas::Waypoint& point = waypoints[index];
        require(std::abs(point.curvature) <= 1.0 / radius + 1e-12,
                "sampled waypoint exceeds maximum curvature");
        require(std::abs(point.curvature) < 1e-12 ||
                    std::abs(std::abs(point.curvature) - 1.0 / radius) < 1e-12,
                "sampled curvature does not match the selected steering primitive");
        require(point.direction == 1 || point.direction == -1,
                "sampled waypoint has an invalid gear direction");
        if (index > 0 && point.direction != waypoints[index - 1].direction) {
            require(waypoints[index - 1].gear_change,
                    "gear transition is missing its preceding stop marker");
        }
        if (!point.gear_change) continue;
        require(index + 1 < waypoints.size(),
                "gear-change marker has no following waypoint");
        require(point.target_speed == 0.0 &&
                    point.direction != waypoints[index + 1].direction,
                "gear-change marker is not a zero-speed cusp stop");
        stop_markers.push_back(&point);
    }
    require(stop_markers.size() == expected_stops.size() &&
                stop_markers.size() >= 2,
            "connector gear-change count differs from the selected path");
    require(!waypoints.empty() &&
                waypoints.front().direction == first_direction(selected),
            "sampled gear direction differs from the signed first segment");
    for (std::size_t index = 0; index < stop_markers.size(); ++index) {
        require(std::hypot(stop_markers[index]->x - expected_stops[index].x,
                           stop_markers[index]->y - expected_stops[index].y) < 1e-8,
                "gear-change stop marker is not at the exact segment boundary");
    }
    require(std::abs(polyline_length({}, waypoints) - selected.geometric_length) < 0.05,
            "sampled connector length disagrees with the selected candidate");
}

void test_blocked_shortest_candidate_fallback() {
    constexpr double radius = 2.82;
    const parking_adas::ReedsSheppConnector connector;
    const std::vector<parking_adas::Pose> goals{
        {4.0, 2.0, 1.2}, {3.0, -4.0, -1.7}, {0.0, 0.0, kPi},
        {5.0, 0.0, kPi}, {-2.0, 5.0, 2.4}, {6.0, -3.0, -2.2},
    };
    bool exercised_longer_candidate = false;
    for (const parking_adas::Pose& goal : goals) {
        const auto candidates = connector.analytical_candidates({}, goal, radius);
        if (candidates.size() < 2 || first_direction(candidates.front()) == 0) continue;
        const parking_adas::Pose start{};
        const std::vector<parking_adas::Pose> shortest_samples =
            sample_path(start, candidates.front(), radius);
        for (std::size_t sample = 2; sample + 2 < shortest_samples.size(); ++sample) {
            const parking_adas::Pose& blocked_pose = shortest_samples[sample];
            parking_adas::PlanRequest request = open_request(goal);
            request.static_obstacles.push_back({
                blocked_pose.x, blocked_pose.y, 0.0, 0.0, 0.0,
                square(blocked_pose.x, blocked_pose.y, 0.12),
            });
            const parking_adas::PlanningContext context(request);
            const parking_adas::CollisionChecker& checker =
                context.collision_checker();
            if (clear_path(start, candidates.front(), radius, checker)) continue;
            const auto longer_clear = std::find_if(
                candidates.begin() + 1, candidates.end(),
                [&](const parking_adas::ReedsSheppPath& candidate) {
                    return candidate.geometric_length >
                               candidates.front().geometric_length + 0.05 &&
                           first_direction(candidate) == first_direction(candidates.front()) &&
                           clear_path(start, candidate, radius, checker);
                });
            if (longer_clear == candidates.end()) continue;
            const std::size_t blocked_candidate_count =
                static_cast<std::size_t>(std::count_if(
                    candidates.begin(), longer_clear,
                    [&](const parking_adas::ReedsSheppPath& candidate) {
                        return !clear_path(start, candidate, radius, checker);
                    }));
            require(blocked_candidate_count > 0,
                    "blocked-candidate fixture did not reject any candidates");

            std::vector<parking_adas::Waypoint> path;
            const bool connected = connector.connect(
                start, goal, first_direction(candidates.front()), context,
                std::chrono::steady_clock::now() + std::chrono::seconds(3),
                path, parking_adas::ReedsSheppImplementation::Analytical);
            if (!connected || path.empty() ||
                polyline_length(start, path) <=
                    candidates.front().geometric_length + 0.02) {
                continue;
            }
            require(std::all_of(path.begin(), path.end(),
                        [&](const parking_adas::Waypoint& point) {
                            return !checker.is_collision(
                                {point.x, point.y, point.heading});
                        }),
                    "connector returned a colliding longer candidate");
            std::cout << "Blocked-candidate selection test: "
                          << blocked_candidate_count
                          << " shorter candidates rejected before a longer clear path\n";
            exercised_longer_candidate = true;
            break;
        }
        if (exercised_longer_candidate) break;
    }
    require(exercised_longer_candidate,
            "blocked shortest candidate did not yield a longer collision-free selection");
}

void test_deadline_and_failure_output() {
    const parking_adas::ReedsSheppConnector connector;
    const parking_adas::Pose start{};
    parking_adas::PlanRequest invalid = open_request({2.0, 0.0, 0.0});
    invalid.vehicle.wheelbase = 0.0;
    const parking_adas::PlanningContext invalid_context(invalid);
    std::vector<parking_adas::Waypoint> path{{9.0, 9.0, 1, 0.0, 0.0, 0.5, false}};
    require(!connector.connect(start, invalid.goal, 1, invalid_context,
                std::chrono::steady_clock::now() +
                    std::chrono::seconds(1), path,
                parking_adas::ReedsSheppImplementation::Analytical) && path.empty(),
            "analytical failure did not clear a pre-populated output vector");

    parking_adas::PlanRequest request = open_request({1000000.0, 0.0, 0.0});
    const parking_adas::PlanningContext context(request);
    path = {{9.0, 9.0, 1, 0.0, 0.0, 0.5, false}};
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(50);
    const auto sampling_start = std::chrono::steady_clock::now();
    const bool connected = connector.connect(start, request.goal, 1, context,
        deadline, path,
        parking_adas::ReedsSheppImplementation::Analytical);
    const auto sampling_elapsed = std::chrono::steady_clock::now() - sampling_start;
    require(!connected && path.empty(),
            "expired sampling deadline returned success or stale output");
    require(sampling_elapsed >= std::chrono::milliseconds(40),
            "large-path test did not reach the deadline while sampling");
    std::cout << "Analytical deadline test: large-path connection aborted after "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     sampling_elapsed).count()
              << " ms with empty output\n";

    request.goal = {2.0, 0.0, 0.0};
    const parking_adas::PlanningContext expired_context(request);
    path = {{9.0, 9.0, 1, 0.0, 0.0, 0.5, false}};
    const auto expired_start = std::chrono::steady_clock::now();
    require(!connector.connect(start, request.goal, 1, expired_context,
                std::chrono::steady_clock::time_point::min(), path,
                parking_adas::ReedsSheppImplementation::Analytical) && path.empty(),
            "already-expired deadline returned success or stale output");
    const auto expired_elapsed = std::chrono::steady_clock::now() - expired_start;
    std::cout << "Already-expired deadline test: connector returned after "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     expired_elapsed).count()
              << " us (candidate enumeration itself has no cancellation check)\n";

    path = {{9.0, 9.0, 1, 0.0, 0.0, 0.5, false}};
    require(!connector.connect(start, {2.0, 0.0, 0.0}, 1, invalid_context,
                std::chrono::steady_clock::time_point::min(), path,
                parking_adas::ReedsSheppImplementation::Numerical) && path.empty(),
            "numerical fallback failure did not clear the output vector");
}

void test_default_dispatch() {
    const parking_adas::ReedsSheppConnector connector;
    const parking_adas::Pose start{};
    const parking_adas::Pose goal{2.0, 0.0, 0.0};
    const parking_adas::PlanRequest request = open_request(goal);
    const parking_adas::PlanningContext context(request);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(2);
    std::vector<parking_adas::Waypoint> default_path;
    std::vector<parking_adas::Waypoint> explicit_path;
    require(connector.connect(start, goal, 1, context,
                deadline, default_path),
            "default connector dispatch failed");
#if defined(ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL) && ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL
    constexpr auto expected = parking_adas::ReedsSheppImplementation::Analytical;
#else
    constexpr auto expected = parking_adas::ReedsSheppImplementation::Numerical;
#endif
    require(connector.connect(start, goal, 1, context,
                deadline, explicit_path, expected),
            "explicit default-mode connector call failed");
    require(default_path.size() == explicit_path.size() &&
                std::equal(default_path.begin(), default_path.end(), explicit_path.begin(),
                    [](const parking_adas::Waypoint& first,
                       const parking_adas::Waypoint& second) {
                        return first.x == second.x && first.y == second.y &&
                            first.direction == second.direction &&
                            first.heading == second.heading &&
                            first.curvature == second.curvature &&
                            first.target_speed == second.target_speed &&
                            first.gear_change == second.gear_change;
                    }),
            "default connector dispatch differs from its configured implementation");
}

}  // namespace

int main() {
    try {
        test_candidate_invariants();
        test_cusp_markers();
        test_blocked_shortest_candidate_fallback();
        test_deadline_and_failure_output();
        test_default_dispatch();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
