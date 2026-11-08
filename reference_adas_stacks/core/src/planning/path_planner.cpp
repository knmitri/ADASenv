#include "parking_adas/planning/path_planner.hpp"

#include <stdexcept>

#include "parking_adas/hybrid_astar/hybrid_a_star.hpp"
#include "parking_adas/reeds_shepp_local/reeds_shepp_local_planner.hpp"

namespace parking_adas {

bool PathPlanner::should_replan(const TrackingStatus& status,
                                const PlanRequest& request) const {
    if (status.trajectory_empty || status.stalled) return true;
    if (status.cross_track_error > request.replan_position_threshold) return true;
    if (!status.at_trajectory_end) return false;
    return status.terminal_distance > 0.08 ||
        status.terminal_heading_error > 0.05;
}

std::unique_ptr<PathPlanner> make_path_planner(const std::string& planner_id) {
    if (planner_id == "hybrid_astar") return std::make_unique<HybridAStarPlanner>();
    if (planner_id == "reeds_shepp_local") return std::make_unique<ReedsSheppLocalPlanner>();
    throw std::invalid_argument("unknown path planner: " + planner_id);
}

}  // namespace parking_adas
