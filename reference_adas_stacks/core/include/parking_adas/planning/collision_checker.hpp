#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

#include "parking_adas/planning/collision_scene.hpp"
#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class CollisionChecker {
public:
    struct Statistics {
        std::uint64_t pose_checks = 0;
        std::uint64_t swept_checks = 0;
        std::uint64_t obstacle_distance_checks = 0;
    };

    // Copies the immutable scene handle; the checker remains valid independently
    // of the caller's CollisionScene object.
    explicit CollisionChecker(const CollisionScene& scene);
    bool is_collision(const Pose& pose) const;
    bool is_swept_collision(const Pose& from, const Pose& to,
                            double translation_bound,
                            double heading_change_bound) const;
    Statistics statistics() const;
    void reset_statistics() const;

private:
    CollisionScene scene_;
    mutable std::atomic<std::uint64_t> pose_checks_{0};
    mutable std::atomic<std::uint64_t> swept_checks_{0};
    mutable std::atomic<std::uint64_t> obstacle_distance_checks_{0};
};

}  // namespace parking_adas
