#pragma once

#include <chrono>
#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class CollisionChecker;
class ParkingGoalEvaluator;

class ReedsSheppConnector {
public:
    bool connect(
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
