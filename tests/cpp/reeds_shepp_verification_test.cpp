#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "parking_adas/common/footprint_geometry.hpp"
#include "parking_adas/common/math.hpp"
#include "parking_adas/hybrid_astar/hybrid_a_star.hpp"
#include "parking_adas/planning/collision_checker.hpp"
#include "parking_adas/planning/parking_goal_evaluator.hpp"
#include "parking_adas/planning/planning_context.hpp"
#include "parking_adas/planning/reeds_shepp_connector.hpp"
#include "parking_adas/reeds_shepp_local/reeds_shepp_local_planner.hpp"

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

using parking_adas::AnalyticalConnectorCounters;
using parking_adas::NumericalConnectorCounters;
using parking_adas::ReedsSheppConnectorDiagnostics;
using parking_adas::ReedsSheppImplementation;

// Size of the numerical solver's enumeration space, derived from the canonical
// word table: 6 three-segment words, 6 four-segment words and 2 five-segment
// words; 2^n direction masks per word; 6 seeds per (word, mask). Exactly half of
// the masks start in each gear.
constexpr int kNumericalSeedsPerVariant = 6;
constexpr int kNumericalDirectionVariants = 6 * 8 + 6 * 16 + 2 * 32;
constexpr int kNumericalCombinationSpace =
    kNumericalSeedsPerVariant * kNumericalDirectionVariants;
constexpr int kNumericalCombinationsPerGear = kNumericalCombinationSpace / 2;

parking_adas::Footprint rectangle(double min_x, double min_y, double max_x,
                                  double max_y) {
    return {{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}};
}

parking_adas::Obstacle fixed_obstacle(const parking_adas::Footprint& footprint) {
    return {0.0, 0.0, 0.0, 0.0, 0.0, footprint};
}

// A wall across the whole corridor between the start and a goal at x = 10.
parking_adas::PlanRequest wall_request() {
    parking_adas::PlanRequest request = open_request({10.0, 0.0, 0.0});
    request.static_obstacles.push_back(
        fixed_obstacle(rectangle(5.0, -1000.0, 6.0, 1000.0)));
    return request;
}

// A goal slot that can never contain the vehicle at the requested goal.
parking_adas::PlanRequest unreachable_slot_request(const parking_adas::Pose& goal) {
    parking_adas::PlanRequest request = open_request(goal);
    request.goal_slot_footprint = rectangle(50.0, 50.0, 52.0, 52.0);
    return request;
}

struct ConnectOutcome {
    bool connected = false;
    std::vector<parking_adas::Waypoint> path;
    ReedsSheppConnectorDiagnostics diagnostics;
};

ConnectOutcome run_connect(
    const parking_adas::PlanningContext& context, const parking_adas::Pose& start,
    const parking_adas::Pose& goal, int initial_direction,
    ReedsSheppImplementation implementation,
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(20)) {
    const parking_adas::ReedsSheppConnector connector;
    ConnectOutcome outcome;
    outcome.connected = connector.connect(
        start, goal, initial_direction, context, deadline, outcome.path,
        implementation, &outcome.diagnostics);
    return outcome;
}

bool same_counters(const NumericalConnectorCounters& a,
                   const NumericalConnectorCounters& b) {
    return a.combinations_attempted == b.combinations_attempted &&
        a.solver_failures == b.solver_failures &&
        a.solver_converged == b.solver_converged &&
        a.initial_direction_rejections == b.initial_direction_rejections &&
        a.cost_pruned_candidates == b.cost_pruned_candidates &&
        a.candidates_sampled == b.candidates_sampled &&
        a.pose_collision_rejections == b.pose_collision_rejections &&
        a.goal_rejections == b.goal_rejections &&
        a.invalid_candidate_rejections == b.invalid_candidate_rejections &&
        a.deadline_rejections == b.deadline_rejections &&
        a.accepted_candidates == b.accepted_candidates &&
        a.initial_pose_collision_observations == b.initial_pose_collision_observations;
}

bool same_counters(const AnalyticalConnectorCounters& a,
                   const AnalyticalConnectorCounters& b) {
    return a.geometric_candidates_generated == b.geometric_candidates_generated &&
        a.candidates_examined == b.candidates_examined &&
        a.initial_pose_collision_rejections == b.initial_pose_collision_rejections &&
        a.initial_direction_rejections == b.initial_direction_rejections &&
        a.swept_collision_rejections == b.swept_collision_rejections &&
        a.goal_rejections == b.goal_rejections &&
        a.invalid_candidate_rejections == b.invalid_candidate_rejections &&
        a.deadline_rejections == b.deadline_rejections &&
        a.accepted_candidates == b.accepted_candidates;
}

// Everything except wall-clock time.
bool same_diagnostics(const ReedsSheppConnectorDiagnostics& a,
                      const ReedsSheppConnectorDiagnostics& b) {
    return a.implementation == b.implementation &&
        a.connector_attempts == b.connector_attempts &&
        a.accepted_connector_attempts == b.accepted_connector_attempts &&
        a.solution_accepted == b.solution_accepted &&
        a.invalid_configuration == b.invalid_configuration &&
        a.selected_path_length_m == b.selected_path_length_m &&
        a.selected_segment_count == b.selected_segment_count &&
        a.selected_waypoint_count == b.selected_waypoint_count &&
        a.selected_path_family == b.selected_path_family &&
        same_counters(a.numerical, b.numerical) &&
        same_counters(a.analytical, b.analytical);
}

// Internal-consistency checks that must hold for any diagnostics produced by a
// single connect() call, accepted or not. `attempts` is 1 for a single call and
// larger for planner-level aggregates.
void require_consistent(const ReedsSheppConnectorDiagnostics& d, int attempts,
                        int accepted_attempts, const std::string& label) {
    require(d.connector_attempts == attempts, label + ": connector_attempts");
    require(d.accepted_connector_attempts == accepted_attempts,
            label + ": accepted_connector_attempts");
    require(d.solution_accepted == (accepted_attempts > 0),
            label + ": solution_accepted disagrees with accepted attempts");
    require(std::isfinite(d.planning_time_ms) && d.planning_time_ms >= 0.0,
            label + ": planning time is not a finite non-negative value");
    if (d.solution_accepted) {
        require(d.selected_waypoint_count > 0 && d.selected_segment_count > 0 &&
                    d.selected_path_length_m > 0.0,
                label + ": accepted solution lacks selected-path metadata");
    } else {
        require(d.selected_waypoint_count == 0 && d.selected_segment_count == 0 &&
                    d.selected_path_length_m == 0.0 &&
                    d.selected_path_family.empty(),
                label + ": rejected call retained selected-path metadata");
    }

    const NumericalConnectorCounters& n = d.numerical;
    const AnalyticalConnectorCounters& a = d.analytical;
    if (d.implementation == ReedsSheppImplementation::Numerical) {
        require(same_counters(a, AnalyticalConnectorCounters{}),
                label + ": numerical call touched analytical counters");
        require(n.combinations_attempted == n.solver_failures + n.solver_converged,
                label + ": attempted != failed + converged");
        require(n.solver_converged == n.cost_pruned_candidates + n.candidates_sampled,
                label + ": converged != cost-pruned + sampled");
        require(n.candidates_sampled ==
                    n.accepted_candidates + n.pose_collision_rejections +
                        n.goal_rejections + n.invalid_candidate_rejections,
                label + ": sampled != accepted + rejected");
        require((n.accepted_candidates > 0) == d.solution_accepted,
                label + ": numerical accepted_candidates disagrees with outcome");
        require(d.selected_path_family.empty(),
                label + ": numerical connector reported a path family");
    } else {
        require(same_counters(n, NumericalConnectorCounters{}),
                label + ": analytical call touched numerical counters");
        require(a.candidates_examined <= a.geometric_candidates_generated,
                label + ": examined more candidates than were generated");
        require(a.candidates_examined ==
                    a.accepted_candidates + a.initial_pose_collision_rejections +
                        a.initial_direction_rejections +
                        a.swept_collision_rejections + a.goal_rejections +
                        a.invalid_candidate_rejections + a.deadline_rejections,
                label + ": examined != accepted + rejected");
        require(a.accepted_candidates == accepted_attempts,
                label + ": analytical accepted_candidates disagrees with outcome");
    }
}

void require_consistent(const ConnectOutcome& outcome, const std::string& label) {
    require_consistent(outcome.diagnostics, 1, outcome.connected ? 1 : 0, label);
    require(outcome.connected == outcome.diagnostics.solution_accepted,
            label + ": return value disagrees with solution_accepted");
    if (outcome.connected) {
        require(outcome.diagnostics.selected_waypoint_count ==
                    static_cast<int>(outcome.path.size()),
                label + ": selected_waypoint_count differs from the returned path");
    } else {
        require(outcome.path.empty(), label + ": failed call returned waypoints");
    }
}

// First direction of travel, 0 for an all-zero-length candidate.
bool direction_mismatch(const parking_adas::ReedsSheppPath& path, int initial_direction) {
    const int first = first_direction(path);
    return first != 0 && initial_direction != 0 && first != initial_direction;
}

int nonzero_segments(const parking_adas::ReedsSheppPath& path) {
    return static_cast<int>(std::count_if(
        path.segments.begin(), path.segments.end(),
        [](const parking_adas::ReedsSheppSegment& segment) {
            return std::abs(segment.signed_length) > 1e-10;
        }));
}

void test_diagnostics_reset_between_calls() {
    const parking_adas::Pose start{};
    const parking_adas::Pose goal{2.0, 0.0, 0.0};
    const parking_adas::ReedsSheppConnector connector;
    const parking_adas::PlanRequest open = open_request(goal);
    const parking_adas::PlanningContext open_context(open);

    // A diagnostics object full of stale values must be wiped on every call,
    // including calls that fail before doing any real work.
    ReedsSheppConnectorDiagnostics diagnostics;
    const auto poison = [&diagnostics]() {
        diagnostics = {};
        diagnostics.implementation = ReedsSheppImplementation::Analytical;
        diagnostics.connector_attempts = 77;
        diagnostics.accepted_connector_attempts = 77;
        diagnostics.planning_time_ms = 1.0e9;
        diagnostics.solution_accepted = true;
        diagnostics.invalid_configuration = true;
        diagnostics.selected_path_length_m = 123.0;
        diagnostics.selected_segment_count = 9;
        diagnostics.selected_waypoint_count = 999;
        diagnostics.selected_path_family = "STALE";
        NumericalConnectorCounters& n = diagnostics.numerical;
        n.combinations_attempted = n.solver_failures = n.solver_converged = 999;
        n.initial_direction_rejections = n.cost_pruned_candidates = 999;
        n.candidates_sampled = n.pose_collision_rejections = n.goal_rejections = 999;
        n.invalid_candidate_rejections = n.deadline_rejections = 999;
        n.accepted_candidates = 999;
        n.initial_pose_collision_observations = 999;
        AnalyticalConnectorCounters& a = diagnostics.analytical;
        a.geometric_candidates_generated = a.candidates_examined = 999;
        a.initial_pose_collision_rejections = a.initial_direction_rejections = 999;
        a.swept_collision_rejections = a.goal_rejections = 999;
        a.invalid_candidate_rejections = a.deadline_rejections = 999;
        a.accepted_candidates = 999;
    };

    const auto expired = std::chrono::steady_clock::time_point::min();
    std::vector<parking_adas::Waypoint> path;
    for (ReedsSheppImplementation implementation :
         {ReedsSheppImplementation::Numerical, ReedsSheppImplementation::Analytical}) {
        const char* name = implementation == ReedsSheppImplementation::Numerical
            ? "numerical" : "analytical";

        // Reference: what a failing call looks like on a fresh object.
        ReedsSheppConnectorDiagnostics fresh_failure;
        require(!connector.connect(start, goal, 1, open_context, expired, path,
                    implementation, &fresh_failure),
                std::string(name) + ": expired-deadline call unexpectedly succeeded");

        poison();
        require(!connector.connect(start, goal, 1, open_context, expired, path,
                    implementation, &diagnostics),
                std::string(name) + ": expired-deadline call unexpectedly succeeded");
        require(same_diagnostics(diagnostics, fresh_failure),
                std::string(name) + ": failed call retained stale diagnostics");
        require(diagnostics.planning_time_ms < 1.0e6,
                std::string(name) + ": planning time kept its stale value");

        // success -> failure on the same object.
        poison();
        require(connector.connect(start, goal, 1, open_context,
                    std::chrono::steady_clock::now() + std::chrono::seconds(20),
                    path, implementation, &diagnostics),
                std::string(name) + ": open-space connection failed");
        require(diagnostics.solution_accepted && diagnostics.selected_waypoint_count > 0,
                std::string(name) + ": accepted call lacks selected-path metadata");
        require(diagnostics.connector_attempts == 1,
                std::string(name) + ": accepted call kept a stale attempt count");
        require(!connector.connect(start, goal, 1, open_context, expired, path,
                    implementation, &diagnostics),
                std::string(name) + ": expired-deadline call unexpectedly succeeded");
        require(same_diagnostics(diagnostics, fresh_failure),
                std::string(name) + ": failure after success kept the accepted path's values");
    }

    // Consecutive successful calls must not accumulate counters.
    for (ReedsSheppImplementation implementation :
         {ReedsSheppImplementation::Numerical, ReedsSheppImplementation::Analytical}) {
        const ConnectOutcome first = run_connect(open_context, start, goal, 1, implementation);
        const ConnectOutcome second = run_connect(open_context, start, goal, 1, implementation);
        require(first.connected && second.connected &&
                    same_diagnostics(first.diagnostics, second.diagnostics),
                "identical consecutive calls reported different (accumulated?) diagnostics");
        require_consistent(second, "consecutive call");
    }

    // The legacy signatures without a diagnostics pointer keep working.
    require(connector.connect(start, goal, 1, open_context,
                std::chrono::steady_clock::now() + std::chrono::seconds(20), path) &&
                !path.empty(),
            "legacy connect() overload without diagnostics failed");
    require(connector.connect(start, goal, 1, open_context,
                std::chrono::steady_clock::now() + std::chrono::seconds(20), path,
                ReedsSheppImplementation::Analytical) && !path.empty(),
            "legacy implementation-selecting overload without diagnostics failed");
}

void test_numerical_counters() {
    const parking_adas::Pose start{};
    const parking_adas::Pose goal{2.0, 0.0, 0.0};

    // Success in open space: the enumeration covers exactly the matching-gear
    // half of the space, and the skipped half is accounted as direction rejections.
    {
        const parking_adas::PlanRequest request = open_request(goal);
        const parking_adas::PlanningContext context(request);
        for (int direction : {1, -1}) {
            const ConnectOutcome outcome = run_connect(
                context, start, goal, direction, ReedsSheppImplementation::Numerical);
            const std::string label = "numerical open-space direction " +
                std::to_string(direction);
            require_consistent(outcome, label);
            const NumericalConnectorCounters& n = outcome.diagnostics.numerical;
            require(outcome.diagnostics.implementation ==
                        ReedsSheppImplementation::Numerical,
                    label + ": wrong implementation tag");
            require(n.combinations_attempted == kNumericalCombinationsPerGear,
                    label + ": solver was not run on exactly the matching-gear half");
            require(n.initial_direction_rejections == kNumericalCombinationsPerGear,
                    label + ": non-matching-gear combinations were not counted");
            require(n.pose_collision_rejections == 0 && n.goal_rejections == 0 &&
                        n.invalid_candidate_rejections == 0 &&
                        n.deadline_rejections == 0,
                    label + ": open space produced collision/goal/deadline rejections");
            require(n.initial_pose_collision_observations == 0,
                    label + ": a clear start pose was reported as colliding");
            require(n.solver_failures > 0 && n.solver_converged > 0,
                    label + ": expected both solver failures and convergences");
            if (direction == 1) {
                require(outcome.connected, label + ": forward connection failed");
                require(n.accepted_candidates >= 1,
                        label + ": no accepted candidate recorded");
                require(std::abs(outcome.diagnostics.selected_path_length_m -
                                 polyline_length(start, outcome.path)) < 0.05,
                        label + ": selected length disagrees with the waypoints");
                require(outcome.diagnostics.selected_segment_count >= 1 &&
                            outcome.diagnostics.selected_segment_count <= 5,
                        label + ": implausible selected segment count");
            }
        }
    }

    // initial_direction == 0 matches no numerical direction mask.
    {
        const parking_adas::PlanRequest request = open_request(goal);
        const parking_adas::PlanningContext context(request);
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 0, ReedsSheppImplementation::Numerical);
        require_consistent(outcome, "numerical direction 0");
        const NumericalConnectorCounters& n = outcome.diagnostics.numerical;
        require(!outcome.connected && n.combinations_attempted == 0 &&
                    n.initial_direction_rejections == kNumericalCombinationSpace &&
                    n.solver_failures == 0 && n.candidates_sampled == 0,
                "numerical initial-direction rejection did not cover the whole space");
    }

    // Pose collisions: a wall spans the corridor, so every converged candidate
    // is rejected while its poses are sampled.
    {
        const parking_adas::PlanRequest request = wall_request();
        const parking_adas::PlanningContext context(request);
        const parking_adas::Pose wall_goal = request.goal;
        const ConnectOutcome outcome = run_connect(
            context, start, wall_goal, 1, ReedsSheppImplementation::Numerical);
        require_consistent(outcome, "numerical wall");
        const NumericalConnectorCounters& n = outcome.diagnostics.numerical;
        require(!outcome.connected && n.accepted_candidates == 0,
                "numerical connector crossed a wall");
        require(n.solver_converged > 0 && n.pose_collision_rejections > 0,
                "wall fixture produced no pose-collision rejections");
        require(n.pose_collision_rejections == n.candidates_sampled &&
                    n.goal_rejections == 0 && n.invalid_candidate_rejections == 0 &&
                    n.cost_pruned_candidates == 0 && n.deadline_rejections == 0,
                "wall fixture rejections were not all attributed to pose collisions");
    }

    // Goal-evaluator rejection: collision-free candidates that end outside the slot.
    {
        const parking_adas::PlanRequest request = unreachable_slot_request(goal);
        const parking_adas::PlanningContext context(request);
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Numerical);
        require_consistent(outcome, "numerical unreachable slot");
        const NumericalConnectorCounters& n = outcome.diagnostics.numerical;
        require(!outcome.connected && n.candidates_sampled > 0 &&
                    n.goal_rejections == n.candidates_sampled &&
                    n.pose_collision_rejections == 0 && n.accepted_candidates == 0 &&
                    n.cost_pruned_candidates == 0,
                "slot fixture rejections were not all attributed to the goal evaluator");
    }

    // Deadline: an expired deadline stops the search before any solver work.
    {
        const parking_adas::PlanRequest request = open_request(goal);
        const parking_adas::PlanningContext context(request);
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Numerical,
            std::chrono::steady_clock::time_point::min());
        require_consistent(outcome, "numerical expired deadline");
        const NumericalConnectorCounters& n = outcome.diagnostics.numerical;
        require(!outcome.connected && n.deadline_rejections == 1 &&
                    n.combinations_attempted == 0 &&
                    n.initial_direction_rejections == 0,
                "numerical expired deadline did no or wrongly counted work");
    }

    // A zero steering limit gives zero curvature, which the numerical connector
    // rejects up front; this is reported rather than counted as search work.
    {
        parking_adas::PlanRequest request = open_request(goal);
        request.vehicle.max_steering_angle = 0.0;
        const parking_adas::PlanningContext context(request);
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Numerical);
        require_consistent(outcome, "numerical invalid vehicle");
        require(!outcome.connected && outcome.diagnostics.invalid_configuration &&
                    outcome.diagnostics.numerical.combinations_attempted == 0,
                "numerical invalid configuration was not reported");
    }

    // Initial pose collision is OBSERVED, not enforced: the numerical connector
    // never validates the start pose, so a path that leaves a start-pose overlap
    // is still accepted (the analytical connector rejects every candidate here).
    // A thin obstacle just behind the vehicle overlaps the inflated start
    // footprint by 6 cm but is cleared after the first few centimetres of
    // forward travel.
    {
        const parking_adas::PlanRequest clear_request = open_request(goal);
        const parking_adas::PlanningContext clear_context(clear_request);
        const ConnectOutcome clear = run_connect(
            clear_context, start, goal, 1, ReedsSheppImplementation::Numerical);
        require(clear.connected, "clear-start numerical baseline failed");

        parking_adas::PlanRequest request = open_request(goal);
        request.static_obstacles.push_back(
            fixed_obstacle(rectangle(-1.2, -0.2, -1.0, 0.2)));
        const parking_adas::PlanningContext context(request);
        require(context.collision_checker().is_collision(start),
                "initial-collision fixture does not collide at the start pose");

        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Numerical);
        require_consistent(outcome, "numerical initial collision");
        const NumericalConnectorCounters& n = outcome.diagnostics.numerical;
        require(n.initial_pose_collision_observations == 1,
                "numerical connector did not report the colliding start pose");
        // Behavior is unchanged: the search ran in full on the matching-gear half
        // and still accepted a path; the observation is not a rejection.
        require(outcome.connected && n.accepted_candidates >= 1,
                "numerical connector wrongly rejected a path because of the start pose");
        require(n.combinations_attempted == kNumericalCombinationsPerGear &&
                    n.initial_direction_rejections == kNumericalCombinationsPerGear,
                "start-pose observation changed the numerical search space");
        require(n.goal_rejections == 0 && n.invalid_candidate_rejections == 0 &&
                    n.deadline_rejections == 0 &&
                    !outcome.diagnostics.invalid_configuration,
                "start-pose observation produced unrelated rejections");
        require(std::abs(outcome.diagnostics.selected_path_length_m -
                         clear.diagnostics.selected_path_length_m) < 1e-9 &&
                    outcome.diagnostics.selected_segment_count ==
                        clear.diagnostics.selected_segment_count &&
                    outcome.path.size() == clear.path.size(),
                "start-pose observation changed the selected numerical path");
        // Waypoint-by-waypoint identity with the clear-start path: continuous
        // values within 1e-12, discrete values exactly.
        require(outcome.path.size() == clear.path.size(),
                "colliding-start and clear-start paths have different lengths");
        constexpr double kWaypointTolerance = 1e-12;
        for (std::size_t index = 0; index < outcome.path.size(); ++index) {
            const parking_adas::Waypoint& got = outcome.path[index];
            const parking_adas::Waypoint& want = clear.path[index];
            const std::string where = " at waypoint " + std::to_string(index);
            require(std::abs(got.x - want.x) <= kWaypointTolerance,
                    "start-pose observation changed waypoint x" + where);
            require(std::abs(got.y - want.y) <= kWaypointTolerance,
                    "start-pose observation changed waypoint y" + where);
            require(std::abs(got.heading - want.heading) <= kWaypointTolerance,
                    "start-pose observation changed waypoint heading" + where);
            require(got.direction == want.direction,
                    "start-pose observation changed waypoint direction" + where);
            require(std::abs(got.curvature - want.curvature) <= kWaypointTolerance,
                    "start-pose observation changed waypoint curvature" + where);
        }
        for (const parking_adas::Waypoint& waypoint : outcome.path) {
            require(!context.collision_checker().is_collision(
                        {waypoint.x, waypoint.y, waypoint.heading}),
                    "numerical path contains a colliding sampled pose");
        }

        // The analytical connector validates the same start pose and rejects.
        const ConnectOutcome analytical = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Analytical);
        require_consistent(analytical, "analytical initial collision (observation fixture)");
        const AnalyticalConnectorCounters& a = analytical.diagnostics.analytical;
        require(!analytical.connected &&
                    a.initial_pose_collision_rejections == a.geometric_candidates_generated &&
                    a.accepted_candidates == 0,
                "analytical connector did not reject the colliding start pose");
        require(same_counters(analytical.diagnostics.numerical, NumericalConnectorCounters{}),
                "analytical call reported a numerical start-pose observation");

        // The observation is deterministic and, when aggregated over calls, sums.
        const ConnectOutcome again = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Numerical);
        require(same_diagnostics(again.diagnostics, outcome.diagnostics),
                "numerical start-pose observation is not deterministic");
        ReedsSheppConnectorDiagnostics total;
        parking_adas::accumulate_connector_diagnostics(total, outcome.diagnostics);
        parking_adas::accumulate_connector_diagnostics(total, again.diagnostics);
        require(total.numerical.initial_pose_collision_observations == 2,
                "start-pose observations were not summed across connector calls");
    }
}

void test_analytical_counters() {
    constexpr double radius = 2.82;
    const parking_adas::Pose start{};
    const parking_adas::Pose goal{2.0, 0.0, 0.0};
    const parking_adas::ReedsSheppConnector connector;
    const auto candidates = connector.analytical_candidates(start, goal, radius);
    require(candidates.size() > 3, "analytical counter fixture has too few candidates");
    const int generated = static_cast<int>(candidates.size());

    // Success and initial-direction rejection: the open-space fixture accepts the
    // first candidate whose first gear matches, so the number of direction
    // rejections is the index of that candidate in the shortest-first list. If no
    // candidate has a matching first gear, every candidate is rejected by the gear
    // constraint.
    {
        const auto check_case = [&](const parking_adas::Pose& case_goal, int direction,
                                    bool expect_connection, bool expect_leading_rejections) {
            const std::string label = "analytical open-space goal (" +
                std::to_string(case_goal.x) + "," + std::to_string(case_goal.y) + "," +
                std::to_string(case_goal.yaw) + ") direction " + std::to_string(direction);
            const parking_adas::PlanRequest request = open_request(case_goal);
            const parking_adas::PlanningContext context(request);
            const auto case_candidates =
                connector.analytical_candidates(start, case_goal, radius);
            const int case_generated = static_cast<int>(case_candidates.size());
            const auto expected = std::find_if(
                case_candidates.begin(), case_candidates.end(),
                [&](const parking_adas::ReedsSheppPath& path) {
                    return !direction_mismatch(path, direction);
                });
            require((expected != case_candidates.end()) == expect_connection,
                    label + ": fixture does not have the expected admissibility");
            const int expected_index = static_cast<int>(expected - case_candidates.begin());
            require(!expect_connection || (expected_index > 0) == expect_leading_rejections,
                    label + ": fixture does not have the expected leading rejections");

            const ConnectOutcome outcome = run_connect(
                context, start, case_goal, direction, ReedsSheppImplementation::Analytical);
            require_consistent(outcome, label);
            const AnalyticalConnectorCounters& a = outcome.diagnostics.analytical;
            require(outcome.connected == expect_connection, label + ": unexpected outcome");
            require(a.geometric_candidates_generated == case_generated,
                    label + ": generated count differs from analytical_candidates()");
            require(a.initial_pose_collision_rejections == 0 &&
                        a.swept_collision_rejections == 0 && a.goal_rejections == 0 &&
                        a.invalid_candidate_rejections == 0 &&
                        a.deadline_rejections == 0,
                    label + ": unexpected non-direction rejections in open space");
            if (!expect_connection) {
                require(a.initial_direction_rejections == case_generated &&
                            a.candidates_examined == case_generated &&
                            a.accepted_candidates == 0,
                        label + ": gear rejections do not cover every candidate");
                return;
            }
            require(a.initial_direction_rejections == expected_index &&
                        a.candidates_examined == expected_index + 1 &&
                        a.accepted_candidates == 1,
                    label + ": rejection count does not match the shortest-first order");
            const ReedsSheppConnectorDiagnostics& d = outcome.diagnostics;
            require(d.implementation == ReedsSheppImplementation::Analytical &&
                        d.selected_path_family == expected->family &&
                        std::abs(d.selected_path_length_m - expected->geometric_length) < 1e-12 &&
                        d.selected_segment_count == nonzero_segments(*expected) &&
                        d.selected_waypoint_count ==
                            static_cast<int>(outcome.path.size()),
                    label + ": selected-path metadata differs from the chosen candidate");
            require(std::abs(polyline_length(start, outcome.path) -
                             d.selected_path_length_m) < 0.05,
                    label + ": selected length disagrees with the waypoints");
        };

        // Straight ahead: first candidate accepted, nothing rejected.
        check_case(goal, 1, true, false);
        // Every candidate to a point straight ahead starts in forward gear.
        check_case(goal, -1, false, false);
        // Straight reverse accepted by the first candidate in reverse gear.
        check_case({-2.0, 0.0, 0.0}, -1, true, false);

        // Mixed gears: find a goal whose shortest candidate starts forward while a
        // longer one starts in reverse, then request reverse so that the leading
        // candidates are rejected by the gear constraint before one is accepted.
        bool found = false;
        for (int x = -4; x <= 4 && !found; ++x) {
            for (int y = -4; y <= 4 && !found; ++y) {
                for (double yaw : {-2.4, -1.2, 0.0, 1.2, 2.4}) {
                    const parking_adas::Pose mixed{1.0 * x, 1.0 * y, yaw};
                    const auto mixed_candidates =
                        connector.analytical_candidates(start, mixed, radius);
                    if (mixed_candidates.size() < 2 ||
                        first_direction(mixed_candidates.front()) != 1) {
                        continue;
                    }
                    const auto reverse = std::find_if(
                        mixed_candidates.begin(), mixed_candidates.end(),
                        [](const parking_adas::ReedsSheppPath& path) {
                            return first_direction(path) == -1;
                        });
                    if (reverse == mixed_candidates.end()) continue;
                    const parking_adas::PlanRequest probe = open_request(mixed);
                    const parking_adas::PlanningContext probe_context(probe);
                    // Only keep goals the reverse candidate can actually reach
                    // (the start pose must be clear and the goal evaluator satisfied).
                    if (!run_connect(probe_context, start, mixed, -1,
                                     ReedsSheppImplementation::Analytical).connected) {
                        continue;
                    }
                    check_case(mixed, -1, true, true);
                    found = true;
                    break;
                }
            }
        }
        require(found, "no mixed-gear fixture found for the leading-rejection case");
    }

    // Initial pose collision: every examined candidate fails before sampling.
    {
        parking_adas::PlanRequest request = open_request(goal);
        request.static_obstacles.push_back(
            fixed_obstacle(rectangle(-0.5, -0.5, 0.5, 0.5)));
        const parking_adas::PlanningContext context(request);
        require(context.collision_checker().is_collision(start),
                "initial-collision fixture does not collide at the start pose");
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Analytical);
        require_consistent(outcome, "analytical initial collision");
        const AnalyticalConnectorCounters& a = outcome.diagnostics.analytical;
        require(!outcome.connected && a.geometric_candidates_generated == generated &&
                    a.candidates_examined == generated &&
                    a.initial_pose_collision_rejections == generated &&
                    a.initial_direction_rejections == 0 &&
                    a.swept_collision_rejections == 0 && a.goal_rejections == 0,
                "start-collision rejections do not cover every candidate");
    }

    // Swept collision: a wall spans the corridor. Each candidate is rejected
    // either by the gear constraint or by the swept validator, never both.
    {
        const parking_adas::PlanRequest request = wall_request();
        const parking_adas::PlanningContext context(request);
        const parking_adas::Pose wall_goal = request.goal;
        const auto wall_candidates = connector.analytical_candidates(
            start, wall_goal, radius);
        const int wall_generated = static_cast<int>(wall_candidates.size());
        const int mismatches = static_cast<int>(std::count_if(
            wall_candidates.begin(), wall_candidates.end(),
            [](const parking_adas::ReedsSheppPath& path) {
                return direction_mismatch(path, 1);
            }));
        require(wall_generated > mismatches,
                "wall fixture needs gear-matched candidates to reach the swept validator");
        const ConnectOutcome outcome = run_connect(
            context, start, wall_goal, 1, ReedsSheppImplementation::Analytical);
        require_consistent(outcome, "analytical wall");
        const AnalyticalConnectorCounters& a = outcome.diagnostics.analytical;
        require(!outcome.connected && a.geometric_candidates_generated == wall_generated &&
                    a.candidates_examined == wall_generated &&
                    a.initial_direction_rejections == mismatches &&
                    a.swept_collision_rejections == wall_generated - mismatches &&
                    a.initial_pose_collision_rejections == 0 && a.goal_rejections == 0 &&
                    a.invalid_candidate_rejections == 0 && a.deadline_rejections == 0,
                "wall rejections are not split between gear and swept collisions");
    }

    // Goal-evaluator rejection: every collision-free candidate ends outside the slot.
    {
        const parking_adas::PlanRequest request = unreachable_slot_request(goal);
        const parking_adas::PlanningContext context(request);
        const int mismatches = static_cast<int>(std::count_if(
            candidates.begin(), candidates.end(),
            [](const parking_adas::ReedsSheppPath& path) {
                return direction_mismatch(path, 1);
            }));
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Analytical);
        require_consistent(outcome, "analytical unreachable slot");
        const AnalyticalConnectorCounters& a = outcome.diagnostics.analytical;
        require(!outcome.connected && a.candidates_examined == generated &&
                    a.initial_direction_rejections == mismatches &&
                    a.goal_rejections == generated - mismatches &&
                    a.swept_collision_rejections == 0 &&
                    a.initial_pose_collision_rejections == 0,
                "slot rejections are not attributed to the goal evaluator");
    }

    // Swept validation is stricter than pose checks: an obstacle that every pose
    // clears by a few centimetres still rejects the straight candidate.
    {
        parking_adas::PlanRequest request = open_request({6.0, 0.0, 0.0});
        // Inflated half-width is 0.93 + 0.1 = 1.03 m; this edge is 5 cm beyond it.
        request.static_obstacles.push_back(
            fixed_obstacle(rectangle(2.95, 1.08, 3.05, 1.18)));
        const parking_adas::PlanningContext context(request);
        const auto straight_candidates = connector.analytical_candidates(
            start, request.goal, radius);
        require(!straight_candidates.empty() &&
                    std::abs(straight_candidates.front().geometric_length - 6.0) < 1e-9,
                "straight-candidate fixture changed");
        require(clear_path(start, straight_candidates.front(), radius,
                           context.collision_checker()),
                "pose-level checks reject the lateral-clearance fixture");
        const ConnectOutcome outcome = run_connect(
            context, start, request.goal, 1, ReedsSheppImplementation::Analytical);
        require_consistent(outcome, "analytical lateral clearance");
        require(outcome.diagnostics.analytical.swept_collision_rejections == 0,
                "analytical swept bound (2.5 cm) rejected a 5 cm clearance");

        // Tighten the gap below the analytical swept bound: pose checks still pass
        // but the swept validator now rejects the straight candidate.
        parking_adas::PlanRequest tight = open_request({6.0, 0.0, 0.0});
        tight.static_obstacles.push_back(
            fixed_obstacle(rectangle(2.95, 1.045, 3.05, 1.145)));
        const parking_adas::PlanningContext tight_context(tight);
        require(clear_path(start, straight_candidates.front(), radius,
                           tight_context.collision_checker()),
                "pose-level checks reject the tight lateral-clearance fixture");
        const ConnectOutcome tight_outcome = run_connect(
            tight_context, start, tight.goal, 1, ReedsSheppImplementation::Analytical);
        require_consistent(tight_outcome, "analytical tight lateral clearance");
        require(tight_outcome.diagnostics.analytical.swept_collision_rejections >= 1,
                "analytical swept validation missed a sub-bound clearance");
        require(tight_outcome.diagnostics.analytical.initial_pose_collision_rejections == 0,
                "tight clearance fixture was attributed to the start pose");
    }

    // Invalid vehicle configuration.
    {
        parking_adas::PlanRequest request = open_request(goal);
        request.vehicle.wheelbase = 0.0;
        const parking_adas::PlanningContext context(request);
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Analytical);
        require_consistent(outcome, "analytical invalid vehicle");
        require(!outcome.connected && outcome.diagnostics.invalid_configuration &&
                    outcome.diagnostics.analytical.geometric_candidates_generated == 0,
                "analytical invalid configuration was not reported");
    }
}

void test_deadline_counters() {
    const parking_adas::Pose start{};

    // Expired before the first candidate: the deadline check trips on candidate 0.
    {
        const parking_adas::Pose goal{2.0, 0.0, 0.0};
        const parking_adas::PlanRequest request = open_request(goal);
        const parking_adas::PlanningContext context(request);
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Analytical,
            std::chrono::steady_clock::time_point::min());
        require_consistent(outcome, "analytical expired deadline");
        const AnalyticalConnectorCounters& a = outcome.diagnostics.analytical;
        require(!outcome.connected && a.geometric_candidates_generated > 0 &&
                    a.candidates_examined == 1 && a.deadline_rejections == 1 &&
                    a.accepted_candidates == 0 && a.initial_direction_rejections == 0 &&
                    a.swept_collision_rejections == 0 && a.goal_rejections == 0,
                "analytical expired deadline was not attributed to the deadline");
    }

    // Expiry while sampling a very long candidate.
    {
        const parking_adas::Pose goal{1000000.0, 0.0, 0.0};
        const parking_adas::PlanRequest request = open_request(goal);
        const parking_adas::PlanningContext context(request);
        const ConnectOutcome outcome = run_connect(
            context, start, goal, 1, ReedsSheppImplementation::Analytical,
            std::chrono::steady_clock::now() + std::chrono::milliseconds(50));
        require_consistent(outcome, "analytical sampling deadline");
        const AnalyticalConnectorCounters& a = outcome.diagnostics.analytical;
        require(!outcome.connected && a.deadline_rejections >= 1 &&
                    a.accepted_candidates == 0 && a.swept_collision_rejections == 0 &&
                    a.goal_rejections == 0,
                "analytical sampling deadline was not attributed to the deadline");
    }
}

void test_counter_accounting_sweep() {
    const std::vector<parking_adas::Pose> goals{
        {2.0, 0.0, 0.0}, {4.0, 2.0, 1.2}, {-3.0, 1.0, 2.0}, {0.0, 0.0, kPi},
    };
    int checked = 0;
    for (const parking_adas::Pose& goal : goals) {
        const std::vector<parking_adas::PlanRequest> requests{
            open_request(goal), unreachable_slot_request(goal)};
        for (const parking_adas::PlanRequest& request : requests) {
            const parking_adas::PlanningContext context(request);
            for (int direction : {1, -1}) {
                for (ReedsSheppImplementation implementation :
                     {ReedsSheppImplementation::Numerical,
                      ReedsSheppImplementation::Analytical}) {
                    const ConnectOutcome outcome = run_connect(
                        context, {}, goal, direction, implementation);
                    require_consistent(outcome, "accounting sweep");
                    if (implementation == ReedsSheppImplementation::Numerical) {
                        const NumericalConnectorCounters& n = outcome.diagnostics.numerical;
                        require(n.combinations_attempted +
                                        n.initial_direction_rejections ==
                                    kNumericalCombinationSpace &&
                                    n.deadline_rejections == 0,
                                "numerical enumeration space is not fully accounted for");
                    }
                    ++checked;
                }
            }
        }
    }
    std::cout << "Diagnostics accounting sweep: " << checked
              << " connector calls satisfied every counter identity\n";
}

struct SweptValidation {
    int pairs = 0;
    int failures = 0;
    int first_failure = -1;
};

// Validates consecutive waypoints (starting from `start`) with the swept
// collision checker, using bounds derived from the numerical sampling: every
// waypoint lies at most one 0.10 m arc step from its predecessor, so the
// translation bound is 0.10 m and the heading change is at most |curvature| *
// 0.10 rad.
SweptValidation validate_swept(const parking_adas::Pose& start,
                               const std::vector<parking_adas::Waypoint>& path,
                               const parking_adas::CollisionChecker& checker) {
    SweptValidation result;
    parking_adas::Pose previous = start;
    for (std::size_t index = 0; index < path.size(); ++index) {
        const parking_adas::Waypoint& waypoint = path[index];
        const parking_adas::Pose current{waypoint.x, waypoint.y, waypoint.heading};
        const bool collision = checker.is_swept_collision(
            previous, current, kSampleSpacing + 1e-9,
            std::abs(waypoint.curvature) * kSampleSpacing + 1e-9);
        ++result.pairs;
        if (collision) {
            ++result.failures;
            if (result.first_failure < 0) result.first_failure = static_cast<int>(index);
        }
        previous = current;
    }
    return result;
}

void test_numerical_swept_validation() {
    const parking_adas::Pose start{};

    // Open space: record how many numerical paths also pass swept validation.
    {
        const std::vector<parking_adas::Pose> goals{
            {2.0, 0.0, 0.0}, {4.0, 2.0, 0.6}, {6.0, -1.0, 0.0}, {-3.0, 0.0, 0.0},
            {3.0, 3.0, 1.2},
        };
        int accepted = 0;
        int swept_clean = 0;
        for (const parking_adas::Pose& goal : goals) {
            const parking_adas::PlanRequest request = open_request(goal);
            const parking_adas::PlanningContext context(request);
            for (int direction : {1, -1}) {
                const ConnectOutcome outcome = run_connect(
                    context, start, goal, direction, ReedsSheppImplementation::Numerical);
                require_consistent(outcome, "numerical swept open space");
                if (!outcome.connected) continue;
                ++accepted;
                const SweptValidation swept = validate_swept(
                    start, outcome.path, context.collision_checker());
                require(swept.pairs == static_cast<int>(outcome.path.size()),
                        "swept validation skipped waypoint pairs");
                require(swept.failures == 0,
                        "open-space numerical path failed swept validation");
                ++swept_clean;
            }
        }
        require(accepted >= 4, "too few numerical open-space paths were produced");
        std::cout << "Numerical swept validation (open space): " << swept_clean << "/"
                  << accepted << " accepted paths passed swept validation\n";
    }

    // Lateral-clearance fixture: every waypoint pose clears the obstacle, so the
    // numerical connector accepts the path, yet a conservative swept check over
    // the same 0.10 m steps rejects it. The connector is not modified.
    {
        parking_adas::PlanRequest request = open_request({6.0, 0.0, 0.0});
        request.static_obstacles.push_back(
            fixed_obstacle(rectangle(2.95, 1.08, 3.05, 1.18)));
        const parking_adas::PlanningContext context(request);
        const ConnectOutcome outcome = run_connect(
            context, start, request.goal, 1, ReedsSheppImplementation::Numerical);
        require_consistent(outcome, "numerical lateral clearance");
        require(outcome.connected, "numerical connector rejected the lateral-clearance fixture");
        {
            // The obstacle is cleared by every pose of the straight candidate, so
            // the fixture must behave like open space for the cheapest path, and the
            // solver (which is obstacle-independent) must do identical work.
            const parking_adas::PlanRequest open = open_request(request.goal);
            const parking_adas::PlanningContext open_context(open);
            const ConnectOutcome baseline = run_connect(
                open_context, start, open.goal, 1, ReedsSheppImplementation::Numerical);
            require(baseline.connected &&
                        baseline.diagnostics.numerical.pose_collision_rejections == 0,
                    "open-space baseline for the lateral-clearance fixture failed");
            const NumericalConnectorCounters& n = outcome.diagnostics.numerical;
            const NumericalConnectorCounters& b = baseline.diagnostics.numerical;
            require(n.combinations_attempted == b.combinations_attempted &&
                        n.solver_failures == b.solver_failures &&
                        n.solver_converged == b.solver_converged &&
                        n.initial_direction_rejections == b.initial_direction_rejections,
                    "obstacle changed the obstacle-independent solver workload");
            require(n.combinations_attempted == kNumericalCombinationsPerGear,
                    "lateral-clearance fixture did not search the matching-gear half");
            require(n.initial_pose_collision_observations == 0 && n.goal_rejections == 0 &&
                        n.invalid_candidate_rejections == 0 && n.deadline_rejections == 0,
                    "lateral-clearance fixture produced unrelated rejections");
            require(n.accepted_candidates >= 1 &&
                        n.pose_collision_rejections ==
                            n.candidates_sampled - n.accepted_candidates,
                    "lateral-clearance rejections are not all pose collisions");
            require(std::abs(outcome.diagnostics.selected_path_length_m -
                             baseline.diagnostics.selected_path_length_m) < 1e-9 &&
                        outcome.diagnostics.selected_segment_count ==
                            baseline.diagnostics.selected_segment_count &&
                        outcome.diagnostics.selected_waypoint_count ==
                            baseline.diagnostics.selected_waypoint_count,
                    "the 5 cm clearance changed the selected numerical path");
        }
        for (const parking_adas::Waypoint& waypoint : outcome.path) {
            require(!context.collision_checker().is_collision(
                        {waypoint.x, waypoint.y, waypoint.heading}),
                    "numerical connector returned a pose-colliding waypoint");
        }
        const SweptValidation swept = validate_swept(
            start, outcome.path, context.collision_checker());
        require(swept.failures > 0 && swept.first_failure >= 0,
                "swept validation did not flag the sub-bound lateral clearance");
        std::cout << "Numerical swept validation (5 cm lateral clearance): connector "
                  << "accepted " << outcome.path.size() << " waypoints, swept check "
                  << "rejected " << swept.failures << "/" << swept.pairs
                  << " steps (first at waypoint " << swept.first_failure << ")\n";

        // Re-running is deterministic.
        const ConnectOutcome again = run_connect(
            context, start, request.goal, 1, ReedsSheppImplementation::Numerical);
        const SweptValidation repeat = validate_swept(
            start, again.path, context.collision_checker());
        require(again.connected && repeat.failures == swept.failures &&
                    repeat.first_failure == swept.first_failure &&
                    same_diagnostics(again.diagnostics, outcome.diagnostics),
                "numerical swept validation is not deterministic");
    }

    // Endpoint-only checks are insufficient: a thin obstacle sits on the swept
    // arc between two consecutive 0.10 m samples (default 0.1 m clearance), so
    // neither sample pose collides but the motion between them does.
    {
        parking_adas::PlanRequest request = open_request({});
        const double steering = request.vehicle.max_steering_angle;
        const double curvature = std::tan(steering) / request.vehicle.wheelbase;
        const parking_adas::Pose first{};
        const parking_adas::Pose middle = parking_adas::integrate_pose_by_distance(
            first, steering, 0.05, request.vehicle.wheelbase, 0.05);
        const parking_adas::Pose second = parking_adas::integrate_pose_by_distance(
            first, steering, 0.10, request.vehicle.wheelbase, 0.10);
        const parking_adas::Polygon middle_footprint =
            parking_adas::vehicle_footprint(middle, request.vehicle, 0.1);
        const parking_adas::Point2 corner = middle_footprint[3];
        request.static_obstacles.push_back(fixed_obstacle(
            rectangle(corner.x - 0.003, corner.y - 0.003,
                      corner.x + 0.003, corner.y + 0.003)));
        const parking_adas::PlanningContext context(request);
        const parking_adas::CollisionChecker& checker = context.collision_checker();
        require(!checker.is_collision(first) && !checker.is_collision(second),
                "hidden-obstacle fixture collides at a sample endpoint");
        require(checker.is_collision(middle),
                "hidden-obstacle fixture does not collide between the samples");
        require(checker.is_swept_collision(
                    first, second, kSampleSpacing, curvature * kSampleSpacing),
                "swept check missed an obstacle between 0.10 m samples");
        std::cout << "Endpoint-only fixture: both 0.10 m sample poses are clear, the "
                  << "intermediate pose collides, swept check rejects the step\n";
    }
}

void test_planner_diagnostics() {
    const parking_adas::Pose start{};
    const parking_adas::Pose goal{2.0, 0.0, 0.0};

    // ReedsSheppLocalPlanner exposes the diagnostics of its single connect() call
    // and resets them on the next plan().
    {
        parking_adas::ReedsSheppLocalPlanner planner;
        const parking_adas::PlanRequest request = open_request(goal);
        const std::vector<parking_adas::Waypoint> path = planner.plan(request);
        require(!path.empty(), "local planner failed in open space");
        const parking_adas::PlanningDiagnostics& diagnostics = planner.diagnostics();
        const ReedsSheppConnectorDiagnostics& connector_diagnostics =
            diagnostics.reeds_shepp_connector;
        require_consistent(connector_diagnostics, 1, 1, "local planner");
        require(diagnostics.goal_connection_attempts ==
                    connector_diagnostics.connector_attempts &&
                    diagnostics.solution_found,
                "local planner attempt count disagrees with connector diagnostics");
        // plan() prepends the start waypoint to the connector's waypoints.
        require(connector_diagnostics.selected_waypoint_count ==
                    static_cast<int>(path.size()) - 1,
                "local planner selected_waypoint_count differs from its path");

        const parking_adas::PlanningContext context(request);
        const ConnectOutcome direct = run_connect(
            context, {request.start_x, request.start_y, request.start_heading}, goal,
            request.start_direction,
#if defined(ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL) && ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL
            ReedsSheppImplementation::Analytical
#else
            ReedsSheppImplementation::Numerical
#endif
        );
        require(same_diagnostics(direct.diagnostics, connector_diagnostics),
                "local planner diagnostics differ from a direct default connect()");

        const parking_adas::PlanRequest failing = unreachable_slot_request(goal);
        require(planner.plan(failing).empty(), "unreachable slot unexpectedly planned");
        const ReedsSheppConnectorDiagnostics& after =
            planner.diagnostics().reeds_shepp_connector;
        require_consistent(after, 1, 0, "local planner after failure");
        require(!planner.diagnostics().solution_found,
                "local planner kept a stale solution flag");
    }

    // HybridAStarPlanner: a start close to the goal connects on the first
    // expansion, so the aggregate equals one direct connect() call.
    {
        parking_adas::HybridAStarPlanner planner;
        const parking_adas::PlanRequest request = open_request(goal);
        require(!planner.plan(request).empty(), "hybrid planner failed in open space");
        const parking_adas::PlanningDiagnostics& diagnostics = planner.diagnostics();
        const ReedsSheppConnectorDiagnostics& aggregate =
            diagnostics.reeds_shepp_connector;
        require_consistent(aggregate, 1, 1, "hybrid first-expansion");
        require(diagnostics.goal_connection_attempts == 1 && diagnostics.solution_found,
                "hybrid planner did not connect on the first expansion");

        const parking_adas::PlanningContext context(request);
        const ConnectOutcome direct = run_connect(
            context, start, goal, request.start_direction,
#if defined(ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL) && ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL
            ReedsSheppImplementation::Analytical
#else
            ReedsSheppImplementation::Numerical
#endif
        );
        require(same_diagnostics(direct.diagnostics, aggregate),
                "hybrid aggregate differs from the equivalent direct connect()");
    }

    // Hybrid A* aggregates every goal-connection attempt: total attempts are
    // counted separately from the candidate-level counters inside them.
    {
        parking_adas::HybridAStarPlanner planner;
        const parking_adas::PlanRequest request = unreachable_slot_request(goal);
        planner.plan(request);
        const parking_adas::PlanningDiagnostics& diagnostics = planner.diagnostics();
        const ReedsSheppConnectorDiagnostics& aggregate =
            diagnostics.reeds_shepp_connector;
        require(diagnostics.goal_connection_attempts > 1,
                "hybrid failure fixture made too few goal-connection attempts");
        require_consistent(aggregate, diagnostics.goal_connection_attempts, 0,
                           "hybrid aggregate");
        require(!aggregate.solution_accepted && !diagnostics.solution_found,
                "hybrid planner reported a solution for an unreachable slot");
        if (aggregate.implementation == ReedsSheppImplementation::Numerical) {
            const NumericalConnectorCounters& n = aggregate.numerical;
            if (n.deadline_rejections == 0) {
                require(n.combinations_attempted + n.initial_direction_rejections ==
                            aggregate.connector_attempts * kNumericalCombinationSpace,
                        "hybrid numerical aggregate does not cover every attempt's space");
            }
            require(n.combinations_attempted > aggregate.connector_attempts,
                    "candidate attempts were not distinguished from connector attempts");
        } else {
            require(aggregate.analytical.geometric_candidates_generated >
                        aggregate.connector_attempts,
                    "candidate attempts were not distinguished from connector attempts");
        }
        require(aggregate.planning_time_ms <= diagnostics.planning_time_ms + 1.0,
                "aggregate connector time exceeds the total planning time");
        std::cout << "Hybrid aggregate: " << aggregate.connector_attempts
                  << " connector attempts covering "
                  << (aggregate.implementation == ReedsSheppImplementation::Numerical
                          ? aggregate.numerical.combinations_attempted
                          : aggregate.analytical.candidates_examined)
                  << " candidate attempts\n";
    }
}

}  // namespace

int main() {
    try {
        test_candidate_invariants();
        test_cusp_markers();
        test_blocked_shortest_candidate_fallback();
        test_deadline_and_failure_output();
        test_default_dispatch();
        test_diagnostics_reset_between_calls();
        test_numerical_counters();
        test_analytical_counters();
        test_deadline_counters();
        test_counter_accounting_sweep();
        test_numerical_swept_validation();
        test_planner_diagnostics();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
