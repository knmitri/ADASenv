#include "parking_adas/planning/planning_context.hpp"

namespace parking_adas {

PlanningContext::PlanningContext(const PlanRequest& request, double clearance)
    : request_(request),
      collision_scene_(request_, clearance),
      collision_checker_(collision_scene_),
      goal_evaluator_(request_) {}

}  // namespace parking_adas
