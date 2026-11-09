#pragma once

#include "parking_adas/domain/types.hpp"
#include "parking_adas/planning/collision_checker.hpp"
#include "parking_adas/planning/collision_scene.hpp"
#include "parking_adas/planning/parking_goal_evaluator.hpp"

namespace parking_adas {

class PlanningContext {
public:
    // Owns a request snapshot and the query objects constructed from that snapshot.
    explicit PlanningContext(const PlanRequest& request, double clearance = 0.1);

    const PlanRequest& request() const { return request_; }
    const CollisionScene& collision_scene() const { return collision_scene_; }
    const CollisionChecker& collision_checker() const { return collision_checker_; }
    const ParkingGoalEvaluator& goal_evaluator() const { return goal_evaluator_; }

    PlanningContext(const PlanningContext&) = delete;
    PlanningContext& operator=(const PlanningContext&) = delete;

private:
    const PlanRequest request_;
    const CollisionScene collision_scene_;
    const CollisionChecker collision_checker_;
    const ParkingGoalEvaluator goal_evaluator_;
};

}  // namespace parking_adas
