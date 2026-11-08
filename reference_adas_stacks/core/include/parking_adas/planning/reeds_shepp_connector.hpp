#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class CollisionChecker;
class ParkingGoalEvaluator;

enum class ReedsSheppSegmentType {
    Left,
    Straight,
    Right,
};

struct ReedsSheppSegment {
    ReedsSheppSegmentType type = ReedsSheppSegmentType::Straight;
    double signed_length = 0.0;
};

struct ReedsSheppPath {
    std::string family;
    std::vector<ReedsSheppSegment> segments;
    double geometric_length = 0.0;
};

enum class ReedsSheppImplementation {
    Numerical,
    Analytical,
};

class ReedsSheppConnector {
public:
    // Enumerates all feasible canonical analytical paths in metres. This stage
    // is purely geometric and does not consult obstacles.
    std::vector<ReedsSheppPath> analytical_candidates(
        const Pose& start,
        const Pose& goal,
        double turning_radius) const;

    bool connect(
        const Pose& start,
        const Pose& goal,
        int initial_direction,
        const PlanRequest& request,
        const CollisionChecker& collision_checker,
        const ParkingGoalEvaluator& goal_evaluator,
        const std::chrono::steady_clock::time_point& deadline,
        std::vector<Waypoint>& result) const;

    bool connect(
        const Pose& start,
        const Pose& goal,
        int initial_direction,
        const PlanRequest& request,
        const CollisionChecker& collision_checker,
        const ParkingGoalEvaluator& goal_evaluator,
        const std::chrono::steady_clock::time_point& deadline,
        std::vector<Waypoint>& result,
        ReedsSheppImplementation implementation) const;

private:
    bool connect_numerical(
        const Pose& start,
        const Pose& goal,
        int initial_direction,
        const PlanRequest& request,
        const CollisionChecker& collision_checker,
        const ParkingGoalEvaluator& goal_evaluator,
        const std::chrono::steady_clock::time_point& deadline,
        std::vector<Waypoint>& result) const;
};

}  // namespace parking_adas
