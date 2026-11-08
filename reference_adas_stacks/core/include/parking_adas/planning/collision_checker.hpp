#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

#include "parking_adas/common/footprint_geometry.hpp"
#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class CollisionChecker {
public:
    struct Statistics {
        std::uint64_t pose_checks = 0;
        std::uint64_t swept_checks = 0;
        std::uint64_t obstacle_distance_checks = 0;
    };

    CollisionChecker(const PlanRequest& request, double clearance = 0.1);
    bool is_collision(const Pose& pose) const;
    bool is_swept_collision(const Pose& from, const Pose& to,
                            double translation_bound,
                            double heading_change_bound) const;
    Statistics statistics() const;
    void reset_statistics() const;

private:
    PlanRequest request_;
    double clearance_;
    std::vector<Polygon> obstacle_polygons_;
    bool invalid_geometry_ = false;
    mutable std::atomic<std::uint64_t> pose_checks_{0};
    mutable std::atomic<std::uint64_t> swept_checks_{0};
    mutable std::atomic<std::uint64_t> obstacle_distance_checks_{0};
};

}  // namespace parking_adas
