#pragma once

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class CollisionChecker {
public:
    CollisionChecker(const PlanRequest& request, double clearance = 0.1);
    bool is_collision(const Pose& pose) const;

private:
    PlanRequest request_;
    double clearance_;
};

}  // namespace parking_adas
