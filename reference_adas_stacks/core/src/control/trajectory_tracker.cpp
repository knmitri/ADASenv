#include "parking_adas/control/trajectory_tracker.hpp"

#include <stdexcept>

#include "parking_adas/control/pure_pursuit_tracker.hpp"

namespace parking_adas {

std::unique_ptr<TrajectoryTracker> make_trajectory_tracker(const std::string& tracker_id) {
    if (tracker_id == "pure_pursuit") return std::make_unique<PurePursuitTrajectoryTracker>();
    throw std::invalid_argument("unknown trajectory tracker: " + tracker_id);
}

}  // namespace parking_adas
