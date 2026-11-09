#include "parking_adas/planning/parking_goal_evaluator.hpp"

#include <cmath>

#include "parking_adas/common/footprint_geometry.hpp"
#include "parking_adas/common/math.hpp"

namespace parking_adas {

ParkingGoalEvaluator::ParkingGoalEvaluator(const PlanRequest& request)
    : goal_(request.goal),
      goal_slot_footprint_(request.goal_slot_footprint),
      vehicle_(request.vehicle),
      goal_heading_tolerance_(request.goal_heading_tolerance) {}

bool ParkingGoalEvaluator::is_satisfied(const Pose& pose) const {
    if (goal_slot_footprint_.size() != 4 ||
        std::abs(normalize_angle(pose.yaw - goal_.yaw)) > goal_heading_tolerance_) {
        return false;
    }
    return polygon_contains_polygon(
        footprint_polygon(goal_slot_footprint_),
        vehicle_footprint(pose, vehicle_)
    );
}

}  // namespace parking_adas
