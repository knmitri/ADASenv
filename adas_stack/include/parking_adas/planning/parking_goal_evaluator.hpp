#pragma once

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class ParkingGoalEvaluator {
public:
    explicit ParkingGoalEvaluator(const PlanRequest& request);
    bool is_satisfied(const Pose& pose) const;

private:
    PlanRequest request_;
};

}  // namespace parking_adas
