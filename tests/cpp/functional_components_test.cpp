#include <cmath>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
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
        test_hybrid_a_star();
        test_reeds_shepp_connector();
        test_reeds_shepp_local_planner();
        test_tracker();
        test_tracker_stops_before_gear_change();
        test_tracker_closed_loop();
        test_tracker_s_curve_divergence();
        test_tracker_reverse_motion();
        test_stack_orchestration();
        test_protocol();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
