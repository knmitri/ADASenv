#pragma once

#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class HybridAStarPlanner {
public:
    std::vector<Waypoint> plan(const PlanRequest& request) const;
    const PlanningDiagnostics& diagnostics() const { return diagnostics_; }

private:
    mutable PlanningDiagnostics diagnostics_;
};

}  // namespace parking_adas
