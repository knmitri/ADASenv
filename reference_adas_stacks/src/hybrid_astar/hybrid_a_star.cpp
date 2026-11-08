#include "parking_adas/hybrid_astar/hybrid_a_star.hpp"

#include <cmath>
#include <chrono>
#include <queue>
#include <string>
#include <unordered_map>

#include "parking_adas/common/math.hpp"
#include "parking_adas/planning/collision_checker.hpp"
#include "parking_adas/planning/parking_goal_evaluator.hpp"
#include "parking_adas/planning/reeds_shepp_connector.hpp"

namespace parking_adas {
namespace {
struct Node {
    Pose pose;
    int parent;
    int direction;
    double steering;
    double velocity;
    double cost;
    double run_distance;
};
struct QueueItem { double score; int index; bool operator<(const QueueItem& other) const { return score > other.score; } };

std::string key(const Pose& pose, int direction, double velocity, double steering) {
    return std::to_string(static_cast<int>(std::round(pose.x / 0.35))) + ":" +
           std::to_string(static_cast<int>(std::round(pose.y / 0.35))) + ":" +
           std::to_string(static_cast<int>(std::round(normalize_angle(pose.yaw) / 0.26))) + ":" +
           std::to_string(direction) + ":" +
           std::to_string(static_cast<int>(std::round(velocity / 0.1))) + ":" +
           std::to_string(static_cast<int>(std::round(steering / 0.1)));
}

double heuristic(const Pose& pose, const Pose& goal) {
    return std::hypot(goal.x - pose.x, goal.y - pose.y) + 1.2 * std::abs(normalize_angle(goal.yaw - pose.yaw));
}

bool propagate(const Node& current, int direction, double target_steering,
               const PlanRequest& request, const CollisionChecker& checker,
               Node& next, double primitive_duration = 0.5) {
    const int steps = std::max(1, static_cast<int>(std::ceil(primitive_duration / request.dt)));
    const double target_speed = direction > 0
        ? std::min(request.vehicle.max_speed, 0.5)
        : -std::min(request.vehicle.max_reverse_speed, 0.5);
    next = current;
    next.direction = direction;
    next.steering = current.steering;
    next.velocity = current.velocity;
    for (int step = 0; step < steps; ++step) {
        const double steering_delta = request.vehicle.max_steering_rate * request.dt;
        next.steering = clamp_value(
            next.steering + clamp_value(target_steering - next.steering,
                                        -steering_delta, steering_delta),
            -request.vehicle.max_steering_angle,
            request.vehicle.max_steering_angle);
        const double acceleration = clamp_value(
            (target_speed - next.velocity) / request.dt,
            request.vehicle.max_deceleration,
            request.vehicle.max_acceleration);
        next.velocity += acceleration * request.dt;
        next.velocity = clamp_value(next.velocity,
                                    -request.vehicle.max_reverse_speed,
                                    request.vehicle.max_speed);
        next.pose = integrate_pose_by_distance(
            next.pose, next.steering, next.velocity * request.dt,
            request.vehicle.wheelbase, request.dt);
        if (checker.is_collision(next.pose)) return false;
    }
    next.run_distance = direction == current.direction
        ? current.run_distance + std::abs(next.velocity) * primitive_duration
        : 0.0;
    return true;
}

}  // namespace

std::vector<Waypoint> HybridAStarPlanner::plan(const PlanRequest& request) {
    const auto plan_start = std::chrono::steady_clock::now();
    diagnostics_ = {};
    diagnostics_.closest_goal_distance = std::hypot(
        request.start_x - request.goal.x, request.start_y - request.goal.y);
    const CollisionScene collision_scene(request);
    const CollisionChecker collision_checker(collision_scene);
    const ParkingGoalEvaluator goal_evaluator(request);
    const ReedsSheppConnector reeds_shepp_connector;
    std::vector<Node> nodes{{
        {request.start_x, request.start_y, request.start_heading},
        -1,
        request.start_direction,
        request.start_steering,
        request.start_velocity,
        0.0,
        0.0,
    }};
    std::priority_queue<QueueItem> open;

    std::unordered_map<std::string, double> best;
    open.push({heuristic(nodes.front().pose, request.goal), 0});
    best.emplace(key(nodes.front().pose, nodes.front().direction,
                     nodes.front().velocity, nodes.front().steering), 0.0);
    const std::vector<double> steering = {-request.vehicle.max_steering_angle, -0.5 * request.vehicle.max_steering_angle,
        0.0, 0.5 * request.vehicle.max_steering_angle, request.vehicle.max_steering_angle};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
    int goal_connection_attempts = 0;
    constexpr int max_goal_connection_attempts = 8;
    int best_index = 0;
    double best_goal_score = heuristic(nodes.front().pose, request.goal);
    const auto path_to = [&](int terminal_index) {
        std::vector<Waypoint> path;
        for (int index = terminal_index; index >= 0; index = nodes.at(index).parent) {
            const Node& node = nodes.at(index);
            path.push_back({node.pose.x, node.pose.y, node.direction, node.pose.yaw,
                            std::tan(node.steering) / request.vehicle.wheelbase, node.velocity});
        }
        std::reverse(path.begin(), path.end());
        return path;
    };
    for (int expanded = 0;
         !open.empty() && expanded < 15000 && std::chrono::steady_clock::now() < deadline;
         ++expanded) {
        const int current_index = open.top().index;
        const Node current = nodes.at(current_index); open.pop();
        ++diagnostics_.expanded_nodes;
        const double goal_distance = std::hypot(
            current.pose.x - request.goal.x, current.pose.y - request.goal.y);
        diagnostics_.closest_goal_distance = std::min(
            diagnostics_.closest_goal_distance, goal_distance);
        const double current_goal_score = heuristic(current.pose, request.goal);
        if (current_goal_score < best_goal_score) {
            best_goal_score = current_goal_score;
            best_index = current_index;
        }
        const double goal_heading_error = std::abs(
            normalize_angle(current.pose.yaw - request.goal.yaw));
        if (goal_distance < 3.0 && goal_heading_error < 2.4 &&
            goal_connection_attempts++ < max_goal_connection_attempts) {
            diagnostics_.goal_connection_attempts = goal_connection_attempts;
            std::vector<Waypoint> connection;
            if (reeds_shepp_connector.connect(
                    current.pose, request.goal, current.direction, request,
                    collision_checker, goal_evaluator, deadline, connection)) {
                std::vector<Waypoint> path = path_to(current_index);
                path.insert(path.end(), connection.begin(), connection.end());
                diagnostics_.planning_time_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - plan_start).count();
                diagnostics_.solution_found = true;
                return path;
            }
        }
        for (int direction : {-1, 1}) for (double steer : steering) {
            if (direction != current.direction && current.run_distance < 1.0) continue;
            Node next;
            if (!propagate(current, direction, steer, request, collision_checker, next)) continue;
            const double distance = std::hypot(next.pose.x - current.pose.x,
                                               next.pose.y - current.pose.y);
            const double cost = current.cost + distance +
                (direction != current.direction ? 4.0 : 0.0) + 0.08 * std::abs(steer);
            const std::string state_key = key(next.pose, direction, next.velocity, next.steering);
            auto found = best.find(state_key); if (found != best.end() && found->second <= cost) continue;
            best[state_key] = cost;
            next.parent = current_index;
            next.cost = cost;
            nodes.push_back(next);
            const int index = static_cast<int>(nodes.size()) - 1;
            open.push({cost + heuristic(next.pose, request.goal), index});
        }
    }
    diagnostics_.planning_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - plan_start).count();
    diagnostics_.time_limited = std::chrono::steady_clock::now() >= deadline;
    if (best_index == 0) return {};
    diagnostics_.partial_plan = true;
    return path_to(best_index);
}

}  // namespace parking_adas
