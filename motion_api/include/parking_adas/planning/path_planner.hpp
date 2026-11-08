#pragma once

#include <memory>
#include <string>
#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class PathPlanner {
public:
    virtual ~PathPlanner() = default;

    virtual std::vector<Waypoint> plan(const PlanRequest& request) = 0;
    virtual bool should_replan(const TrackingStatus& status,
                               const PlanRequest& request) const;
    virtual const PlanningDiagnostics& diagnostics() const = 0;
    virtual const char* name() const = 0;
};

std::unique_ptr<PathPlanner> make_path_planner(const std::string& planner_id);

}  // namespace parking_adas
