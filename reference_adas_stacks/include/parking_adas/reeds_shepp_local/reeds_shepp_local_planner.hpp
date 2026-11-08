#pragma once

#include "parking_adas/planning/path_planner.hpp"

namespace parking_adas {

class ReedsSheppLocalPlanner final : public PathPlanner {
public:
    std::vector<Waypoint> plan(const PlanRequest& request) override;
    const PlanningDiagnostics& diagnostics() const override { return diagnostics_; }
    const char* name() const override { return "reeds_shepp_local"; }

private:
    PlanningDiagnostics diagnostics_;
};

}  // namespace parking_adas
