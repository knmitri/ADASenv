#pragma once

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class ParkingGoalEvaluator {
public:
    explicit ParkingGoalEvaluator(const PlanRequest& request);
    bool is_satisfied(const Pose& pose) const;

private:
    Pose goal_;
    Footprint goal_slot_footprint_;
    VehicleParameters vehicle_;
    double goal_heading_tolerance_;
};

}  // namespace parking_adas
