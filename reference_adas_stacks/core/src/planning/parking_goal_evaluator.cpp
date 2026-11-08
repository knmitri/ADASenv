#include "parking_adas/planning/parking_goal_evaluator.hpp"

#include <cmath>

#include "parking_adas/common/footprint_geometry.hpp"
#include "parking_adas/common/math.hpp"

namespace parking_adas {

ParkingGoalEvaluator::ParkingGoalEvaluator(const PlanRequest& request) : request_(request) {}

bool ParkingGoalEvaluator::is_satisfied(const Pose& pose) const {
    if (request_.goal_slot_footprint.size() != 4 ||
        std::abs(normalize_angle(pose.yaw - request_.goal.yaw)) > request_.goal_heading_tolerance) {
        return false;
    }
    return polygon_contains_polygon(
        footprint_polygon(request_.goal_slot_footprint),
        vehicle_footprint(pose, request_.vehicle)
    );
}

}  // namespace parking_adas
