#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "parking_adas/common/math.hpp"
#include "parking_adas/control/pure_pursuit_tracker.hpp"
#include "parking_adas/control/trajectory_tracker.hpp"
#include "parking_adas/planning/collision_checker.hpp"
#include "parking_adas/hybrid_astar/hybrid_a_star.hpp"
#include "parking_adas/planning/parking_goal_evaluator.hpp"
#include "parking_adas/planning/reeds_shepp_connector.hpp"
#include "parking_adas/reeds_shepp_local/reeds_shepp_local_planner.hpp"
#include "parking_adas/stack/parking_adas_stack.hpp"
#include "sim_bridge/protocol/simulator_protocol.hpp"

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

double distance_to_segment(const parking_adas::Pose& pose,
                           const parking_adas::Waypoint& first,
                           const parking_adas::Waypoint& second) {
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double length_squared = dx * dx + dy * dy;
    const double projection = length_squared > 1e-12
        ? std::max(0.0, std::min(1.0,
            ((pose.x - first.x) * dx + (pose.y - first.y) * dy) / length_squared))
        : 0.0;
    const double closest_x = first.x + projection * dx;
    const double closest_y = first.y + projection * dy;
    return std::hypot(pose.x - closest_x, pose.y - closest_y);
}

std::string reeds_shepp_word(const parking_adas::ReedsSheppPath& path) {
    std::string word;
    for (const parking_adas::ReedsSheppSegment& segment : path.segments) {
        switch (segment.type) {
            case parking_adas::ReedsSheppSegmentType::Left:
                word.push_back('L');
                break;
            case parking_adas::ReedsSheppSegmentType::Straight:
                word.push_back('S');
                break;
            case parking_adas::ReedsSheppSegmentType::Right:
                word.push_back('R');
                break;
        }
    }
    return word;
}

struct TrajectoryLeg {
    int direction;
    double steering;
    double distance;
};

struct TrackerCase {
    std::string name;
    std::vector<TrajectoryLeg> legs;
    double sample_spacing = 0.10;
};

std::vector<parking_adas::Waypoint> make_test_trajectory(
    const std::vector<TrajectoryLeg>& legs,
    const parking_adas::VehicleParameters& vehicle,
    double sample_spacing) {
    std::vector<parking_adas::Waypoint> path;
    parking_adas::Pose pose{};
    int previous_direction = 0;

    for (const TrajectoryLeg& leg : legs) {
        const double curvature = std::tan(leg.steering) / vehicle.wheelbase;
        if (path.empty()) {
            path.push_back({pose.x, pose.y, leg.direction, pose.yaw, curvature,
                            leg.direction * 0.25, false});
        } else if (leg.direction != previous_direction) {
            path.back().gear_change = true;
            path.back().target_speed = 0.0;
            path.push_back({pose.x, pose.y, leg.direction, pose.yaw, curvature,
                            0.0, false});
        }

        const int samples = std::max(
            1, static_cast<int>(std::ceil(leg.distance / sample_spacing)));
        const double step = leg.distance / static_cast<double>(samples);
        for (int sample = 0; sample < samples; ++sample) {
            pose = parking_adas::integrate_pose_by_distance(
                pose, leg.steering, leg.direction * step, vehicle.wheelbase, step);
            const double target_speed = leg.direction > 0
                ? std::min(vehicle.max_speed, 0.25)
                : -std::min(vehicle.max_reverse_speed, 0.25);
            path.push_back({pose.x, pose.y, leg.direction, pose.yaw, curvature,
                            target_speed, false});
        }
        previous_direction = leg.direction;
    }

    if (!path.empty()) path.back().target_speed = 0.0;
    return path;
}

struct TrajectoryTrackingError {
    double cross_track = std::numeric_limits<double>::infinity();
    double heading = std::numeric_limits<double>::infinity();
};

TrajectoryTrackingError error_to_trajectory(
    const parking_adas::Pose& pose,
    const std::vector<parking_adas::Waypoint>& path) {
    TrajectoryTrackingError error;
    for (std::size_t index = 1; index < path.size(); ++index) {
        if (path[index - 1].direction != path[index].direction) continue;
        const parking_adas::Waypoint& first = path[index - 1];
        const parking_adas::Waypoint& second = path[index];
        const double dx = second.x - first.x;
        const double dy = second.y - first.y;
        const double length_squared = dx * dx + dy * dy;
        const double projection = length_squared > 1e-12
            ? std::max(0.0, std::min(1.0,
                ((pose.x - first.x) * dx + (pose.y - first.y) * dy) / length_squared))
            : 0.0;
        const double closest_x = first.x + projection * dx;
        const double closest_y = first.y + projection * dy;
        const double cross_track = std::hypot(pose.x - closest_x, pose.y - closest_y);
        if (cross_track < error.cross_track) {
            const double heading_delta = parking_adas::normalize_angle(
                second.heading - first.heading);
            error.cross_track = cross_track;
            error.heading = std::abs(parking_adas::normalize_angle(
                pose.yaw - first.heading - projection * heading_delta));
        }
    }
    return error;
}

void require_tracker_tolerance(const TrackerCase& test_case) {
    constexpr double dt = 0.05;
    constexpr double max_cross_track_error = 0.25;
    constexpr double max_path_heading_error = 0.35;
    constexpr double terminal_position_tolerance = 0.20;
    constexpr double terminal_heading_tolerance = 0.17453292519943295;
    constexpr double terminal_speed_tolerance = 0.05;
    constexpr int max_steps = 4000;

    const parking_adas::VehicleParameters vehicle;
    const std::vector<parking_adas::Waypoint> path =
        make_test_trajectory(test_case.legs, vehicle, test_case.sample_spacing);
    parking_adas::PurePursuitTrajectoryTracker tracker;
    tracker.reset(path, vehicle);
    parking_adas::VehicleState state{
        path.front().x, path.front().y, path.front().heading, 0.0, 0.0, {}};
    double maximum_cross_track_error = 0.0;
    double maximum_path_heading_error = 0.0;
    bool reached_terminal_pose = false;

    for (int step = 0; step < max_steps; ++step) {
        tracker.assess(state);
        const parking_adas::MotionCommand command = tracker.command(state);
        require(std::isfinite(command.acceleration) && std::isfinite(command.steering),
                test_case.name + " produced a non-finite command");
        require(std::abs(command.steering) <= vehicle.max_steering_angle + 1e-9,
                test_case.name + " exceeded the steering limit");

        state.velocity = std::max(-vehicle.max_reverse_speed,
            std::min(vehicle.max_speed, state.velocity + command.acceleration * dt));
        const double max_steering_delta = vehicle.max_steering_rate * dt;
        const double steering_delta = std::max(-max_steering_delta,
            std::min(max_steering_delta, command.steering - state.steering_angle));
        state.steering_angle = std::max(-vehicle.max_steering_angle,
            std::min(vehicle.max_steering_angle, state.steering_angle + steering_delta));

        const parking_adas::Pose next = parking_adas::integrate_pose_by_distance(
            {state.x, state.y, state.yaw}, state.steering_angle,
            state.velocity * dt, vehicle.wheelbase, dt);
        state.x = next.x;
        state.y = next.y;
        state.yaw = next.yaw;

        const TrajectoryTrackingError tracking_error = error_to_trajectory(
            {state.x, state.y, state.yaw}, path);
        maximum_cross_track_error = std::max(
            maximum_cross_track_error, tracking_error.cross_track);
        maximum_path_heading_error = std::max(
            maximum_path_heading_error, tracking_error.heading);
        const parking_adas::Waypoint& goal = path.back();
        const double terminal_distance = std::hypot(goal.x - state.x, goal.y - state.y);
        const double terminal_heading_error = std::abs(
            parking_adas::normalize_angle(goal.heading - state.yaw));
        if (terminal_distance <= terminal_position_tolerance &&
            terminal_heading_error <= terminal_heading_tolerance &&
            std::abs(state.velocity) <= terminal_speed_tolerance) {
            reached_terminal_pose = true;
            break;
        }
    }

    require(maximum_cross_track_error <= max_cross_track_error,
            test_case.name + " exceeded the cross-track tolerance: " +
                std::to_string(maximum_cross_track_error));
    require(maximum_path_heading_error <= max_path_heading_error,
            test_case.name + " exceeded the path-heading tolerance: " +
                std::to_string(maximum_path_heading_error));
    require(reached_terminal_pose,
            test_case.name + " did not settle within the terminal pose tolerance");
}

parking_adas::PlanRequest base_request() {
    parking_adas::PlanRequest request;
    request.start_x = 0.0;
    request.start_y = 0.0;
    request.start_heading = 0.0;
    request.goal = {2.0, 0.0, 0.0};
    request.goal_slot_footprint = {{
        {0.0, -1.5}, {8.0, -1.5}, {8.0, 1.5}, {0.0, 1.5},
    }};
    request.world_min_x = -2.0;
    request.world_min_y = -2.0;
    request.world_max_x = 12.0;
    request.world_max_y = 12.0;
    request.vehicle.max_speed = 2.0;
    request.vehicle.max_reverse_speed = 1.0;
    return request;
}

void test_collision_checker() {
    parking_adas::PlanRequest request = base_request();
    parking_adas::CollisionChecker checker(request);
    parking_adas::ParkingGoalEvaluator goal_evaluator(request);
    require(!checker.is_collision({0.0, 0.0, 0.0}), "clear pose reported as collision");
    require(checker.is_collision({-1.5, 0.0, 0.0}), "out-of-bounds pose was accepted");
    require(goal_evaluator.is_satisfied({2.0, 0.0, 0.0}),
            "goal slot did not accept a fully contained vehicle");
    require(!goal_evaluator.is_satisfied({-0.8, 0.0, 0.0}),
            "goal slot accepted a vehicle outside its footprint");
    request.static_obstacles.push_back({3.0, 0.0, 0.0, 4.67, 1.86, {}});
    parking_adas::CollisionChecker obstacle_checker(request);
    require(obstacle_checker.is_collision({3.0, 0.0, 0.0}), "obstacle footprint was ignored");
}

void test_swept_collision_checker() {
    parking_adas::PlanRequest request = base_request();
    request.world_min_x = -20.0;
    request.world_min_y = -20.0;
    request.world_max_x = 20.0;
    request.world_max_y = 20.0;
    parking_adas::CollisionChecker clear_checker(request, 0.0);

    require(!clear_checker.is_swept_collision(
                {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, 2.0, 0.0),
            "clear straight sweep was rejected");
    parking_adas::PlanRequest near_obstacle_request = request;
    near_obstacle_request.static_obstacles.push_back(
        {1.0, 1.15, 0.0, 0.2, 0.2, {}});
    parking_adas::CollisionChecker near_obstacle_checker(
        near_obstacle_request, 0.0);
    require(!near_obstacle_checker.is_swept_collision(
                {0.0, 0.0, 0.0}, {0.02, 0.0, 0.0}, 0.02, 0.0) &&
                !near_obstacle_checker.is_swept_collision(
                    {0.02, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.02, 0.0),
            "near-obstacle straight forward/reverse sweep was rejected");
    const double curvature = std::tan(request.vehicle.max_steering_angle) /
        request.vehicle.wheelbase;
    const parking_adas::Pose forward_turn =
        parking_adas::integrate_pose_by_distance(
            {0.0, 0.0, 0.0}, request.vehicle.max_steering_angle,
            1.0, request.vehicle.wheelbase, 1.0);
    require(!clear_checker.is_swept_collision(
                {0.0, 0.0, 0.0}, forward_turn, 1.0, curvature),
            "clear tight forward turn was rejected");
    const parking_adas::Pose reverse_turn =
        parking_adas::integrate_pose_by_distance(
            {0.0, 0.0, 0.0}, request.vehicle.max_steering_angle,
            -1.0, request.vehicle.wheelbase, 1.0);
    require(!clear_checker.is_swept_collision(
                {0.0, 0.0, 0.0}, reverse_turn, 1.0, curvature),
            "clear tight reverse turn was rejected");

    parking_adas::PlanRequest rotation_request = request;
    rotation_request.static_obstacles.push_back({
        3.71, 0.93, 0.0, 0.0, 0.0,
        {{3.69, 0.91}, {3.73, 0.91}, {3.73, 0.95}, {3.69, 0.95}},
    });
    parking_adas::CollisionChecker rotation_checker(rotation_request, 0.0);
    require(!rotation_checker.is_collision({0.0, 0.0, -0.1}) &&
                !rotation_checker.is_collision({0.0, 0.0, 0.1}),
            "rotation-gap test endpoints unexpectedly collide");
    require(rotation_checker.is_swept_collision(
                {0.0, 0.0, -0.1}, {0.0, 0.0, 0.1}, 0.0, 0.2),
            "swept checker missed an obstacle between rotation samples");

    parking_adas::PlanRequest narrow_request = request;
    const double steering = request.vehicle.max_steering_angle;
    const double arc_curvature = std::tan(steering) / request.vehicle.wheelbase;
    const parking_adas::Pose arc_middle = parking_adas::integrate_pose_by_distance(
        {0.0, 0.0, 0.0}, steering, 0.05, request.vehicle.wheelbase, 0.05);
    const parking_adas::Pose arc_end = parking_adas::integrate_pose_by_distance(
        {0.0, 0.0, 0.0}, steering, 0.10, request.vehicle.wheelbase, 0.10);
    const parking_adas::Polygon middle_footprint =
        parking_adas::vehicle_footprint(arc_middle, request.vehicle);
    const parking_adas::Point2 corner = middle_footprint[3];
    narrow_request.static_obstacles.push_back({
        corner.x, corner.y, 0.0, 0.0, 0.0,
        {{corner.x - 0.003, corner.y - 0.003},
         {corner.x + 0.003, corner.y - 0.003},
         {corner.x + 0.003, corner.y + 0.003},
         {corner.x - 0.003, corner.y + 0.003}},
    });
    parking_adas::CollisionChecker narrow_checker(narrow_request, 0.0);
    require(!narrow_checker.is_collision({0.0, 0.0, 0.0}),
            "narrow-obstacle fixture collides at the first 10 cm sample endpoint");
    require(!narrow_checker.is_collision(arc_end),
            "narrow-obstacle fixture collides at the second 10 cm sample endpoint");
    require(narrow_checker.is_collision(arc_middle) &&
                narrow_checker.is_swept_collision(
                    {0.0, 0.0, 0.0}, arc_end, 0.10,
                    arc_curvature * 0.10),
            "swept checker missed a narrow obstacle between 10 cm samples");

    parking_adas::PlanRequest reverse_narrow_request = request;
    const parking_adas::Pose reverse_middle =
        parking_adas::integrate_pose_by_distance(
            {0.0, 0.0, 0.0}, steering, -0.05,
            request.vehicle.wheelbase, 0.05);
    const parking_adas::Pose reverse_end =
        parking_adas::integrate_pose_by_distance(
            {0.0, 0.0, 0.0}, steering, -0.10,
            request.vehicle.wheelbase, 0.10);
    const parking_adas::Polygon reverse_middle_footprint =
        parking_adas::vehicle_footprint(reverse_middle, request.vehicle);
    const parking_adas::Point2 reverse_corner = reverse_middle_footprint[0];
    reverse_narrow_request.static_obstacles.push_back({
        reverse_corner.x, reverse_corner.y, 0.0, 0.0, 0.0,
        {{reverse_corner.x - 0.003, reverse_corner.y - 0.003},
         {reverse_corner.x + 0.003, reverse_corner.y - 0.003},
         {reverse_corner.x + 0.003, reverse_corner.y + 0.003},
         {reverse_corner.x - 0.003, reverse_corner.y + 0.003}},
    });
    parking_adas::CollisionChecker reverse_narrow_checker(
        reverse_narrow_request, 0.0);
    require(!reverse_narrow_checker.is_collision({0.0, 0.0, 0.0}) &&
                !reverse_narrow_checker.is_collision(reverse_end) &&
                reverse_narrow_checker.is_collision(reverse_middle) &&
                reverse_narrow_checker.is_swept_collision(
                    {0.0, 0.0, 0.0}, reverse_end, 0.10,
                    arc_curvature * 0.10),
            "swept checker missed a narrow obstacle on a reverse tight turn");

    parking_adas::PlanRequest front_request = request;
    front_request.static_obstacles.push_back(
        {3.7, 0.9, 0.0, 0.2, 0.2, {}});
    parking_adas::CollisionChecker front_checker(front_request, 0.0);
    require(front_checker.is_collision({0.0, 0.0, 0.0}),
            "front-corner collision was not detected");

    parking_adas::PlanRequest rear_request = request;
    rear_request.static_obstacles.push_back(
        {-0.95, 0.9, 0.0, 0.2, 0.2, {}});
    parking_adas::CollisionChecker rear_checker(rear_request, 0.0);
    require(rear_checker.is_collision({0.0, 0.0, 0.0}),
            "rear-corner collision was not detected");

    require(!clear_checker.is_swept_collision(
                {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0),
            "clear zero-length sweep was rejected");
    require(clear_checker.is_collision({0.0, 0.0, 0.0}) == false,
            "clear start pose was rejected");
    require(clear_checker.is_collision({19.0, 0.0, 0.0}),
            "goal footprint extending beyond the world boundary was accepted");
    parking_adas::PlanRequest colliding_goal_request = request;
    colliding_goal_request.static_obstacles.push_back(
        {2.0, 0.0, 0.0, 0.2, 0.2, {}});
    parking_adas::CollisionChecker colliding_goal_checker(
        colliding_goal_request, 0.0);
    require(colliding_goal_checker.is_collision(colliding_goal_request.goal),
            "colliding goal pose was accepted");

    parking_adas::PlanRequest boundary_request = request;
    boundary_request.world_max_x =
        request.vehicle.wheelbase + request.vehicle.front_overhang;
    parking_adas::CollisionChecker boundary_checker(boundary_request, 0.0);
    require(!boundary_checker.is_collision({0.0, 0.0, 0.0}),
            "footprint grazing the boundary was treated as outside");
    require(boundary_checker.is_swept_collision(
                {0.0, 0.0, 0.0}, {0.01, 0.0, 0.0}, 0.01, 0.0),
            "swept checker failed to conservatively reject boundary grazing");

    parking_adas::PlanRequest multiple_request = request;
    multiple_request.static_obstacles = {
        {10.0, 10.0, 0.0, 1.0, 1.0, {}},
        {0.0, 0.0, 0.0, 0.2, 0.2, {}},
    };
    parking_adas::CollisionChecker multiple_checker(multiple_request, 0.0);
    require(multiple_checker.is_collision({0.0, 0.0, 0.0}),
            "collision with a later obstacle was not detected");

    parking_adas::PlanRequest malformed_request = request;
    malformed_request.static_obstacles.push_back({
        0.0, 0.0, 0.0, 1.0, 1.0,
        {{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.2}, {0.0, 1.0}},
    });
    parking_adas::CollisionChecker malformed_checker(malformed_request, 0.0);
    require(malformed_checker.is_collision({10.0, 10.0, 0.0}),
            "malformed obstacle polygon did not fail closed");
}

void test_collision_check_performance() {
    parking_adas::PlanRequest request = base_request();
    request.world_min_x = -50.0;
    request.world_min_y = -50.0;
    request.world_max_x = 50.0;
    request.world_max_y = 50.0;
    request.static_obstacles = {
        {20.0, 20.0, 0.2, 4.0, 2.0, {}},
        {-20.0, 20.0, -0.3, 4.0, 2.0, {}},
        {20.0, -20.0, 0.5, 4.0, 2.0, {}},
        {-20.0, -20.0, -0.5, 4.0, 2.0, {}},
        {0.0, 20.0, 0.0, 4.0, 2.0, {}},
    };
    const std::array<std::pair<double, double>, 2> legs{{
        {0.0, 4.0}, {request.vehicle.max_steering_angle, 2.0},
    }};
    constexpr int kRepetitions = 20;
    constexpr double kLegacySpacing = 0.10;
    constexpr double kSweptPointMotionBound = 0.025;
    const double body_radius = std::hypot(
        std::max(request.vehicle.wheelbase + request.vehicle.front_overhang,
                 request.vehicle.rear_overhang) + 0.1,
        request.vehicle.width / 2.0 + 0.1);

    auto measure = [&](bool swept) {
        parking_adas::CollisionChecker checker(request);
        double total_ms = 0.0;
        double worst_ms = 0.0;
        for (int repetition = 0; repetition < kRepetitions; ++repetition) {
            const auto begin = std::chrono::steady_clock::now();
            parking_adas::Pose pose{};
            require(!checker.is_collision(pose),
                    "performance fixture unexpectedly collides at start");
            for (const auto& leg : legs) {
                const double curvature = std::tan(leg.first) /
                    request.vehicle.wheelbase;
                const double motion_rate =
                    1.0 + body_radius * std::abs(curvature);
                const double maximum_step = swept
                    ? kSweptPointMotionBound / motion_rate
                    : kLegacySpacing;
                const int samples = std::max(1, static_cast<int>(std::ceil(
                    std::abs(leg.second) / maximum_step)));
                const double step = leg.second / samples;
                for (int sample = 0; sample < samples; ++sample) {
                    const parking_adas::Pose previous = pose;
                    pose = parking_adas::integrate_pose_by_distance(
                        pose, leg.first, step, request.vehicle.wheelbase, step);
                    const bool collision = swept
                        ? checker.is_swept_collision(
                            previous, pose, std::abs(step),
                            std::abs(curvature * step))
                        : checker.is_collision(pose);
                    require(!collision,
                            "performance fixture unexpectedly collides along path");
                }
            }
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - begin).count();
            total_ms += elapsed_ms;
            worst_ms = std::max(worst_ms, elapsed_ms);
        }
        return std::array<double, 5>{
            total_ms / kRepetitions, worst_ms,
            static_cast<double>(checker.statistics().pose_checks) / kRepetitions,
            static_cast<double>(checker.statistics().swept_checks) / kRepetitions,
            static_cast<double>(checker.statistics().obstacle_distance_checks) /
                kRepetitions,
        };
    };

    const auto legacy = measure(false);
    const auto swept = measure(true);
    std::cout << "collision-benchmark"
              << " legacy_avg_ms=" << legacy[0]
              << " legacy_worst_ms=" << legacy[1]
              << " legacy_pose_checks=" << legacy[2]
              << " legacy_swept_checks=" << legacy[3]
              << " swept_avg_ms=" << swept[0]
              << " swept_worst_ms=" << swept[1]
              << " swept_pose_checks=" << swept[2]
              << " swept_swept_checks=" << swept[3]
              << " swept_obstacle_distance_checks=" << swept[4] << '\n';
}

void test_hybrid_a_star() {
    const parking_adas::PlanRequest request = base_request();
    parking_adas::HybridAStarPlanner planner;
    const std::vector<parking_adas::Waypoint> path = planner.plan(request);
    require(!path.empty(), "planner returned no path for an open request");
    for (const parking_adas::Waypoint& point : path) {
        require(std::abs(point.target_speed) <= request.vehicle.max_speed + 1e-9,
            "planner exceeded forward speed limit");
        require(point.target_speed >= -request.vehicle.max_reverse_speed - 1e-9,
            "planner exceeded reverse speed limit");
    }
    parking_adas::CollisionChecker checker(request);
    parking_adas::ParkingGoalEvaluator goal_evaluator(request);
    require(goal_evaluator.is_satisfied({path.back().x, path.back().y, path.back().heading}),
            "planner endpoint does not place the whole vehicle in the goal slot");
    for (const parking_adas::Waypoint& point : path) {
        require(!checker.is_collision({point.x, point.y, point.heading}),
                "planner returned a colliding waypoint");
    }
}

void test_reeds_shepp_connector() {
    parking_adas::PlanRequest request = base_request();
    parking_adas::CollisionChecker collision_checker(request);
    parking_adas::ParkingGoalEvaluator goal_evaluator(request);
    parking_adas::ReedsSheppConnector connector;
    std::vector<parking_adas::Waypoint> path;

    require(connector.connect(
                {request.start_x, request.start_y, request.start_heading},
                request.goal,
                1,
                request,
                collision_checker,
                goal_evaluator,
                std::chrono::steady_clock::now() + std::chrono::seconds(1),
                path),
            "Reeds-Shepp connector did not find an open forward connection");
    require(!path.empty(), "Reeds-Shepp connector returned an empty path");
    require(goal_evaluator.is_satisfied(
                {path.back().x, path.back().y, path.back().heading}),
            "Reeds-Shepp connector did not reach the goal pose");

    request.goal = {-2.0, 0.0, 0.0};
    request.goal_slot_footprint = {{
        {-6.0, -1.5}, {2.0, -1.5}, {2.0, 1.5}, {-6.0, 1.5},
    }};
    request.world_min_x = -8.0;
    parking_adas::CollisionChecker reverse_collision_checker(request);
    parking_adas::ParkingGoalEvaluator reverse_goal_evaluator(request);
    path.clear();
    require(connector.connect(
                {request.start_x, request.start_y, request.start_heading},
                request.goal,
                -1,
                request,
                reverse_collision_checker,
                reverse_goal_evaluator,
                std::chrono::steady_clock::now() + std::chrono::seconds(1),
                path),
            "Reeds-Shepp connector did not find an open reverse connection");
    require(path.front().direction < 0,
            "Reeds-Shepp connector lost the requested reverse direction");
    require(reverse_goal_evaluator.is_satisfied(
                {path.back().x, path.back().y, path.back().heading}),
            "reverse Reeds-Shepp connection did not reach the goal pose");
}

void test_analytical_reeds_shepp_collision_connection() {
    const parking_adas::PlanRequest request = base_request();
    const parking_adas::CollisionChecker collision_checker(request);
    const parking_adas::ParkingGoalEvaluator goal_evaluator(request);
    const parking_adas::ReedsSheppConnector connector;
    std::vector<parking_adas::Waypoint> path;
    const bool connected = connector.connect(
        {request.start_x, request.start_y, request.start_heading},
        request.goal, request.start_direction, request, collision_checker,
        goal_evaluator, std::chrono::steady_clock::now() +
            std::chrono::seconds(1), path,
        parking_adas::ReedsSheppImplementation::Analytical);
    require(connected && !path.empty(),
            "analytical connector failed to select a collision-free candidate");
    require(goal_evaluator.is_satisfied(
                {path.back().x, path.back().y, path.back().heading}),
            "analytical connector returned a path outside the parking goal");
    for (const parking_adas::Waypoint& point : path) {
        require(!collision_checker.is_collision(
                    {point.x, point.y, point.heading}),
                "analytical connector returned a colliding sampled waypoint");
    }

    std::vector<parking_adas::Waypoint> numerical_path;
    require(connector.connect(
                {request.start_x, request.start_y, request.start_heading},
                request.goal, request.start_direction, request, collision_checker,
                goal_evaluator, std::chrono::steady_clock::now() +
                    std::chrono::seconds(1), numerical_path,
                parking_adas::ReedsSheppImplementation::Numerical) &&
                !numerical_path.empty(),
            "explicit numerical connector selection failed");
}

parking_adas::Pose integrate_reeds_shepp_path(
    const parking_adas::Pose& start,
    const parking_adas::ReedsSheppPath& path,
    double turning_radius) {
    parking_adas::Pose pose = start;
    for (const parking_adas::ReedsSheppSegment& segment : path.segments) {
        double curvature = 0.0;
        if (segment.type == parking_adas::ReedsSheppSegmentType::Left) {
            curvature = 1.0 / turning_radius;
        } else if (segment.type == parking_adas::ReedsSheppSegmentType::Right) {
            curvature = -1.0 / turning_radius;
        }
        const double next_yaw = pose.yaw + curvature * segment.signed_length;
        if (std::abs(curvature) < 1e-12) {
            pose.x += segment.signed_length * std::cos(pose.yaw);
            pose.y += segment.signed_length * std::sin(pose.yaw);
        } else {
            pose.x += (std::sin(next_yaw) - std::sin(pose.yaw)) / curvature;
            pose.y += (-std::cos(next_yaw) + std::cos(pose.yaw)) / curvature;
        }
        pose.yaw = parking_adas::normalize_angle(next_yaw);
    }
    return pose;
}

void test_analytical_reeds_shepp_candidates() {
    const parking_adas::ReedsSheppConnector connector;
    const parking_adas::Pose start{0.0, 0.0, 0.0};
    constexpr double radius = 2.82;

    // These exact straight-line distances are also OMPL ReedsSheppStateSpace
    // reference cases for radius=2.82 m.
    for (const parking_adas::Pose& goal :
         {parking_adas::Pose{2.0, 0.0, 0.0},
          parking_adas::Pose{-2.0, 0.0, 0.0}}) {
        const std::vector<parking_adas::ReedsSheppPath> candidates =
            connector.analytical_candidates(start, goal, radius);
        require(!candidates.empty(), "analytical solver missed a straight reference path");
        require(std::abs(candidates.front().geometric_length - 2.0) < 1e-8,
                "analytical shortest length differs from the OMPL straight reference");
        for (const parking_adas::ReedsSheppPath& candidate : candidates) {
            const parking_adas::Pose endpoint =
                integrate_reeds_shepp_path(start, candidate, radius);
            require(std::hypot(endpoint.x - goal.x, endpoint.y - goal.y) < 2e-6,
                    "analytical candidate does not close in position");
            require(std::abs(parking_adas::normalize_angle(endpoint.yaw - goal.yaw)) < 2e-6,
                    "analytical candidate does not close in heading");
        }
    }

    const auto zero_length_candidates =
        connector.analytical_candidates(start, start, radius);
    require(!zero_length_candidates.empty() &&
                zero_length_candidates.front().geometric_length < 1e-10,
            "analytical solver did not retain the zero-length identity path");
    parking_adas::PlanRequest zero_request = base_request();
    zero_request.goal = start;
    zero_request.goal_slot_footprint = {
        {-2.0, -1.5}, {8.0, -1.5}, {8.0, 1.5}, {-2.0, 1.5},
    };
    const parking_adas::CollisionChecker zero_collision_checker(zero_request);
    const parking_adas::ParkingGoalEvaluator zero_goal_evaluator(zero_request);
    std::vector<parking_adas::Waypoint> zero_path;
    require(connector.connect(
                start, start, 1, zero_request, zero_collision_checker,
                zero_goal_evaluator, std::chrono::steady_clock::now() +
                    std::chrono::seconds(1), zero_path,
                parking_adas::ReedsSheppImplementation::Analytical) &&
                zero_path.size() == 1,
            "analytical connector rejected a collision-free zero-length path");

    std::set<std::string> families;
    std::set<std::string> words;
    for (int x = -6; x <= 6; ++x) {
        for (int y = -6; y <= 6; ++y) {
            for (double yaw : {-2.4, -1.2, 0.0, 1.2, 2.4}) {
                const parking_adas::Pose goal{0.7 * x, 0.7 * y, yaw};
                const auto candidates =
                    connector.analytical_candidates(start, goal, radius);
                require(!candidates.empty(),
                        "analytical solver failed to connect a finite pose pair");
                for (const parking_adas::ReedsSheppPath& candidate : candidates) {
                    require(std::isfinite(candidate.geometric_length),
                            "analytical candidate has non-finite length");
                    const parking_adas::Pose endpoint =
                        integrate_reeds_shepp_path(start, candidate, radius);
                    require(std::hypot(endpoint.x - goal.x, endpoint.y - goal.y) < 2e-6 &&
                            std::abs(parking_adas::normalize_angle(
                                endpoint.yaw - goal.yaw)) < 2e-6,
                            "symmetry-generated candidate does not reach its target");
                    families.insert(candidate.family);
                    words.insert(reeds_shepp_word(candidate));
                }
            }
        }
    }

    const auto shortest_length = [&](const parking_adas::Pose& goal) {
        const auto candidates = connector.analytical_candidates(start, goal, radius);
        require(!candidates.empty(), "symmetry comparison produced no path");
        return candidates.front().geometric_length;
    };
    for (const parking_adas::Pose& goal :
         {parking_adas::Pose{2.3, -1.7, 1.1},
          parking_adas::Pose{-3.2, 2.1, -2.0},
          parking_adas::Pose{0.0, 0.0, 3.14159265358979323846}}) {
        const double reference = shortest_length(goal);
        const double reflected_length =
            shortest_length({goal.x, -goal.y, -goal.yaw});
        const double time_flipped_length =
            shortest_length({-goal.x, goal.y, -goal.yaw});
        const double cosine = std::cos(goal.yaw);
        const double sine = std::sin(goal.yaw);
        const double reverse_x = -goal.x * cosine - goal.y * sine;
        const double reverse_y = goal.x * sine - goal.y * cosine;
        const double reversed_length = shortest_length(
            {reverse_x, reverse_y, -goal.yaw});
        require(std::abs(reference - reflected_length) < 1e-7 &&
                std::abs(reference - time_flipped_length) < 1e-7 &&
                std::abs(reference - reversed_length) < 1e-7,
                "Reeds-Shepp symmetry changed the shortest path length");
    }
    const parking_adas::ReedsSheppPath ccscc_witness{
        "CCSCC",
        {
            {parking_adas::ReedsSheppSegmentType::Left, 0.6 * radius},
            {parking_adas::ReedsSheppSegmentType::Right, -0.5 * radius * 3.14159265358979323846},
            {parking_adas::ReedsSheppSegmentType::Straight, -0.5 * radius},
            {parking_adas::ReedsSheppSegmentType::Left, -0.5 * radius * 3.14159265358979323846},
            {parking_adas::ReedsSheppSegmentType::Right, 0.4 * radius},
        },
        0.0,
    };
    const parking_adas::Pose ccscc_goal =
        integrate_reeds_shepp_path(start, ccscc_witness, radius);
    for (const parking_adas::Pose& goal :
         {ccscc_goal,
          parking_adas::Pose{ccscc_goal.x, -ccscc_goal.y, -ccscc_goal.yaw}}) {
        for (const parking_adas::ReedsSheppPath& candidate :
             connector.analytical_candidates(start, goal, radius)) {
            families.insert(candidate.family);
            words.insert(reeds_shepp_word(candidate));
        }
    }
    for (const char* family : {"CSC", "CCC", "CCCC", "CCSC", "CCSCC"}) {
        require(families.count(family) != 0,
                std::string("analytical canonical family produced no candidates: ") + family);
    }
    for (const char* word : {
             "LRL", "RLR", "LRLR", "RLRL", "LRSL", "RLSR",
             "LSRL", "RSLR", "LRSR", "RLSL", "RSRL", "LSLR",
             "LSR", "RSL", "LSL", "RSR", "LRSLR", "RLSRL"}) {
        require(words.count(word) != 0,
                std::string("analytical solver did not emit canonical word: ") + word);
    }
}

void test_reeds_shepp_local_planner() {
    const parking_adas::PlanRequest request = base_request();
    parking_adas::ReedsSheppLocalPlanner planner;
    const std::vector<parking_adas::Waypoint> path = planner.plan(request);
    require(!path.empty(), "local Reeds-Shepp planner returned no open-space path");
    require(planner.diagnostics().solution_found,
            "local Reeds-Shepp planner did not report a solution");
    require(path.front().x == request.start_x && path.front().y == request.start_y,
            "local Reeds-Shepp planner did not retain the start pose");
    parking_adas::ParkingGoalEvaluator goal_evaluator(request);
    require(goal_evaluator.is_satisfied(
                {path.back().x, path.back().y, path.back().heading}),
            "local Reeds-Shepp planner endpoint does not satisfy the parking goal");
}

void test_tracker() {
    const parking_adas::PlanRequest request = base_request();
    const std::vector<parking_adas::Waypoint> path{
        {0.0, 0.0, 1, 0.0, 0.0, 0.25, false},
        {1.0, 0.0, 1, 0.0, 0.0, 0.25, false},
        {2.0, 0.0, 1, 0.0, 0.0, 0.0, true},
        {1.0, 0.0, -1, 0.0, 0.0, -0.25, false},
    };
    parking_adas::PurePursuitTrajectoryTracker tracker;
    parking_adas::VehicleParameters vehicle;
    tracker.reset(path, vehicle);
    const parking_adas::MotionCommand command = tracker.command({0.0, 0.0, 0.0, 0.0, 0.0, {}});
    require(command.acceleration > 0.0, "tracker did not command motion toward path");
    require(std::abs(command.steering) <= vehicle.max_steering_angle,
            "tracker exceeded steering limit");
    parking_adas::HybridAStarPlanner replan_planner;
    require(replan_planner.should_replan(
                tracker.assess({5.0, 5.0, 0.0, 0.0, 0.0, {}}), request),
            "planner missed large tracking error");
    parking_adas::PurePursuitTrajectoryTracker dynamic_obstacle_tracker;
    dynamic_obstacle_tracker.reset(path, vehicle);
    require(!replan_planner.should_replan(
                dynamic_obstacle_tracker.assess(
                    {0.0, 0.0, 0.0, 0.0, 0.0, {{{1.0, 1.0, 0.0, 1.0, 1.0, {}}}}}),
                request),
            "planner replanned merely because a dynamic obstacle was reported");

    parking_adas::TrackingStatus terminal_status;
    terminal_status.at_trajectory_end = true;
    terminal_status.terminal_distance = 0.1;
    require(replan_planner.should_replan(terminal_status, request),
            "planner did not replan after missing the terminal position");
    terminal_status.terminal_distance = 0.0;
    terminal_status.terminal_heading_error = 0.1;
    require(replan_planner.should_replan(terminal_status, request),
            "planner did not replan after missing the terminal heading");
    terminal_status.terminal_heading_error = 0.01;
    require(!replan_planner.should_replan(terminal_status, request),
            "planner replanned despite reaching the terminal pose");
}

void test_tracker_stops_before_gear_change() {
    const std::vector<parking_adas::Waypoint> path{
        {0.0, 0.0, 1, 0.0, 0.0, 0.25, false},
        {1.0, 0.0, 1, 0.0, 0.0, 0.0, true},
        {0.8, 0.0, -1, 0.0, 0.0, -0.25, false},
    };
    parking_adas::PurePursuitTrajectoryTracker tracker;
    parking_adas::VehicleParameters vehicle;
    tracker.reset(path, vehicle);

    const parking_adas::MotionCommand braking = tracker.command({0.92, 0.0, 0.0, 0.20, 0.0, {}});
    require(braking.acceleration < 0.0,
            "tracker did not stop before crossing a forward-to-reverse gear change");

    const parking_adas::MotionCommand reverse = tracker.command({0.92, 0.0, 0.0, 0.0, 0.0, {}});
    require(reverse.acceleration < 0.0,
            "tracker did not begin the reverse leg after stopping at a gear change");
}

void test_tracker_closed_loop() {
    const std::vector<parking_adas::Waypoint> path{
        {0.0, 0.0, 1, 0.0, 0.0, 0.25, false},
        {1.0, 0.0, 1, 0.0, 0.0, 0.25, false},
        {2.0, 0.0, 1, 0.0, 0.0, 0.0, true},
        {1.0, 0.0, -1, 0.0, 0.0, -0.25, false},
        {-1.0, 0.0, -1, 0.0, 0.0, 0.0, false},
    };
    parking_adas::PurePursuitTrajectoryTracker tracker;
    parking_adas::VehicleParameters vehicle;
    tracker.reset(path, vehicle);

    parking_adas::VehicleState state{};
    bool commanded_reverse = false;
    double maximum_forward_x = state.x;
    for (int step = 0; step < 500; ++step) {
        const parking_adas::MotionCommand command = tracker.command(state);
        require(std::isfinite(command.acceleration) && std::isfinite(command.steering),
                "closed-loop tracker emitted a non-finite command");
        require(std::abs(command.steering) <= vehicle.max_steering_angle + 1e-9,
                "closed-loop tracker exceeded steering limit");
        state.steering_angle = command.steering;
        state.velocity = std::max(-vehicle.max_reverse_speed,
            std::min(vehicle.max_speed, state.velocity + command.acceleration * 0.05));
        if (state.velocity < -0.02) commanded_reverse = true;
        const parking_adas::Pose next = parking_adas::integrate_pose_by_distance(
            {state.x, state.y, state.yaw}, state.steering_angle,
            state.velocity * 0.05, vehicle.wheelbase, 0.05);
        state.x = next.x;
        state.y = next.y;
        state.yaw = next.yaw;
        maximum_forward_x = std::max(maximum_forward_x, state.x);
    }
    require(maximum_forward_x > 1.0, "closed-loop tracker did not make forward progress");
    require(commanded_reverse, "closed-loop tracker never commanded reverse motion");
}

void test_tracker_s_curve_divergence() {
    const std::vector<parking_adas::Waypoint> forward_path{
        {0.0, 0.0, 1, 0.0, 0.0, 0.22, false},
        {0.8, 0.18, 1, 0.35, 0.30, 0.22, false},
        {1.6, 0.58, 1, 0.55, 0.28, 0.22, false},
        {2.4, 0.30, 1, -0.55, -0.70, 0.22, false},
        {3.2, -0.42, 1, -0.65, -0.30, 0.22, false},
        {4.1, -0.62, 1, 0.15, 0.55, 0.22, false},
        {5.0, -0.20, 1, 0.55, 0.30, 0.22, false},
        {5.8, 0.0, 1, 0.25, 0.0, 0.0, false},
    };
    std::vector<parking_adas::Waypoint> path = forward_path;
    path.back().gear_change = true;
    path.back().target_speed = 0.0;
    for (std::size_t index = forward_path.size() - 2; index > 0; --index) {
        parking_adas::Waypoint reverse = forward_path[index];
        reverse.direction = -1;
        reverse.target_speed = -0.22;
        path.push_back(reverse);
    }
    parking_adas::Waypoint reverse_start = forward_path.front();
    reverse_start.direction = -1;
    reverse_start.target_speed = 0.0;
    path.push_back(reverse_start);

    parking_adas::PurePursuitTrajectoryTracker tracker;
    parking_adas::VehicleParameters vehicle;
    tracker.reset(path, vehicle);

    parking_adas::VehicleState state{};
    double maximum_divergence = 0.0;
    for (int step = 0; step < 1400; ++step) {
        const parking_adas::MotionCommand command = tracker.command(state);
        require(std::isfinite(command.acceleration) && std::isfinite(command.steering),
                "S-curve tracker emitted a non-finite command");
        require(std::abs(command.steering) <= vehicle.max_steering_angle + 1e-9,
                "S-curve tracker exceeded steering limit");
        double nearest_distance = 1e9;
        const parking_adas::Pose current_pose{state.x, state.y, state.yaw};
        for (std::size_t index = 1; index < path.size(); ++index) {
            nearest_distance = std::min(
                nearest_distance, distance_to_segment(current_pose, path[index - 1], path[index]));
        }
        maximum_divergence = std::max(maximum_divergence, nearest_distance);
        state.steering_angle = command.steering;
        state.velocity = std::max(-vehicle.max_reverse_speed,
            std::min(vehicle.max_speed, state.velocity + command.acceleration * 0.05));
        const parking_adas::Pose next = parking_adas::integrate_pose_by_distance(
            {state.x, state.y, state.yaw}, state.steering_angle,
            state.velocity * 0.05, vehicle.wheelbase, 0.05);
        state.x = next.x;
        state.y = next.y;
        state.yaw = next.yaw;
    }
        require(maximum_divergence < 0.10,
            "S-curve round trip diverged from the path: " + std::to_string(maximum_divergence));
        require(std::hypot(state.x, state.y) < 0.8,
            "S-curve reverse leg did not return to the start position");
        require(std::abs(parking_adas::normalize_angle(state.yaw)) < 0.35,
            "S-curve reverse leg did not return to the start heading");
}

void test_tracker_reverse_motion() {
    const std::vector<parking_adas::Waypoint> curved_reverse_path{
        {0.0, 0.0, -1, 0.0, -0.2, -0.25, false},
        {-1.0, -0.1, -1, 0.1, -0.2, -0.25, false},
        {-2.0, -0.3, -1, 0.2, -0.2, -0.25, false},
        {-3.0, -0.6, -1, 0.3, -0.2, -0.25, false},
        {-4.0, -1.0, -1, 0.4, -0.2, 0.0, false},
    };
    parking_adas::PurePursuitTrajectoryTracker curved_tracker;
    parking_adas::VehicleParameters vehicle;
    curved_tracker.reset(curved_reverse_path, vehicle);
    const parking_adas::MotionCommand curved_command =
        curved_tracker.command({0.0, 0.0, 0.0, 0.0, 0.0, {}});
    require(curved_command.steering < 0.0,
            "reverse tracker steered the wrong way for a left-curving reverse path");

    const std::vector<parking_adas::Waypoint> reverse_path{
        {2.0, 0.0, -1, 0.0, 0.0, -0.25, false},
        {1.0, 0.0, -1, 0.0, 0.0, -0.25, false},
        {0.0, 0.0, -1, 0.0, 0.0, 0.0, false},
    };
    parking_adas::PurePursuitTrajectoryTracker tracker;
    tracker.reset(reverse_path, vehicle);

    parking_adas::VehicleState state{2.0, 0.0, 0.0, 0.0, 0.0, {}};
    double maximum_x = state.x;
    bool commanded_reverse = false;
    for (int step = 0; step < 300; ++step) {
        const parking_adas::MotionCommand command = tracker.command(state);
        require(std::isfinite(command.acceleration) && std::isfinite(command.steering),
                "reverse tracker emitted a non-finite command");
        require(std::abs(command.steering) <= vehicle.max_steering_angle + 1e-9,
                "reverse tracker exceeded steering limit");
        state.steering_angle = command.steering;
        state.velocity = std::max(-vehicle.max_reverse_speed,
            std::min(vehicle.max_speed, state.velocity + command.acceleration * 0.05));
        commanded_reverse = commanded_reverse || state.velocity < -0.02;
        const parking_adas::Pose next = parking_adas::integrate_pose_by_distance(
            {state.x, state.y, state.yaw}, state.steering_angle,
            state.velocity * 0.05, vehicle.wheelbase, 0.05);
        state.x = next.x;
        state.y = next.y;
        state.yaw = next.yaw;
        maximum_x = std::max(maximum_x, state.x);
    }
    require(commanded_reverse, "reverse tracker never commanded negative velocity");
    require(state.x < 1.2, "reverse tracker did not travel backward along the path");
    require(maximum_x <= 2.05, "reverse tracker moved forward beyond its starting pose");
}

void test_tracker_trajectory_families() {
    const std::vector<TrackerCase> cases{
        {"straight forward", {{1, 0.0, 3.0}}},
        {"straight reverse", {{-1, 0.0, 3.0}}},
        {"left arc forward", {{1, 0.30, 4.0}}},
        {"left arc forward, dense", {{1, 0.30, 4.0}}, 0.05},
        {"left arc forward, sparse", {{1, 0.30, 4.0}}, 0.40},
        {"right arc forward", {{1, -0.30, 4.0}}},
        {"left arc reverse", {{-1, 0.30, 4.0}}},
        {"right arc reverse", {{-1, -0.30, 4.0}}},
        {"forward S-curve", {{1, 0.24, 2.5}, {1, -0.24, 2.5}}},
        {"reverse S-curve", {{-1, 0.24, 2.5}, {-1, -0.24, 2.5}}},
        {"forward-reverse straight", {{1, 0.0, 2.0}, {-1, 0.0, 2.0}}},
        {"reverse-forward curved", {{-1, 0.20, 2.0}, {1, -0.20, 2.0}}},
        {"multi-gear maneuver", {
            {1, 0.20, 2.0}, {-1, 0.20, 2.0}, {1, -0.20, 2.0}}},
    };

    for (const TrackerCase& test_case : cases) {
        require_tracker_tolerance(test_case);
    }
}

void test_stack_orchestration() {
    const parking_adas::PlanRequest request = base_request();
    parking_adas::ParkingAdasStack stack;
    const std::vector<parking_adas::Waypoint> trajectory = stack.reset(request);
    require(!trajectory.empty(), "stack reset returned no trajectory");
    const parking_adas::MotionCommand command = stack.command(
        {request.start_x, request.start_y, request.start_heading, 0.0, 0.0, {}});
    require(std::isfinite(command.acceleration) && std::isfinite(command.steering),
            "stack emitted a non-finite command");
}

void test_protocol() {
    const std::string reset = R"({
        "type":"reset",
        "start":[1.0,2.0],
        "start_heading":0.25,
        "goal":[5.0,6.0,1.5],
        "vehicle":{"length":4.7,"width":1.9,"wheelbase":2.8,"front_overhang":0.9,"rear_overhang":1.0,"max_speed":2.0,"max_reverse_speed":1.0},
        "simulation":{"dt":0.05,"world_min_x":-2,"world_min_y":-3,"world_max_x":20,"world_max_y":30},
        "goal_slot":{"corners":[[4,5],[6,5],[6,7],[4,7]]},
        "static_obstacles":[{"x":3,"y":4,"heading":0.2,"length":4,"width":2,"corners":[[1,1],[2,1],[2,2],[1,2]]}]
    })";
    const parking_adas::PlanRequest request = sim_bridge::parse_reset_request(reset);
    require(request.start_x == 1.0 && request.start_y == 2.0, "reset start was not parsed");
    require(request.vehicle.front_overhang == 0.9 && request.vehicle.rear_overhang == 1.0,
            "vehicle footprint dimensions were not parsed");
    require(request.goal_slot_footprint.size() == 4, "goal slot footprint was not parsed");
    require(request.static_obstacles.size() == 1 && request.static_obstacles[0].footprint.size() == 4,
            "obstacle footprint was not parsed");
    const std::vector<parking_adas::Waypoint> path{{1.0, 2.0, 1, 0.2, 0.1, 0.25, false}};
    const std::string ready = sim_bridge::ready_response(path);
    require(ready.find("curvature") != std::string::npos &&
            ready.find("target_speed") != std::string::npos &&
            ready.find("gear_change") != std::string::npos,
            "ready response omitted trajectory fields");
}

}  // namespace

int main() {
    try {
        test_collision_checker();
        test_swept_collision_checker();
        test_collision_check_performance();
        test_hybrid_a_star();
        test_reeds_shepp_connector();
        test_analytical_reeds_shepp_collision_connection();
        test_analytical_reeds_shepp_candidates();
        test_reeds_shepp_local_planner();
        test_tracker();
        test_tracker_stops_before_gear_change();
        test_tracker_closed_loop();
        test_tracker_s_curve_divergence();
        test_tracker_reverse_motion();
        test_tracker_trajectory_families();
        test_stack_orchestration();
        test_protocol();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
