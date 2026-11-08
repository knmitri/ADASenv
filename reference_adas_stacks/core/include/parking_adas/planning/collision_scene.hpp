#pragma once

#include <memory>
#include <vector>

#include "parking_adas/common/footprint_geometry.hpp"
#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class CollisionChecker;

class CollisionScene {
public:
    explicit CollisionScene(const PlanRequest& request, double clearance = 0.1);

private:
    struct Data;
    static std::shared_ptr<const Data> make_data(
        const PlanRequest& request,
        double clearance
    );
    const std::shared_ptr<const Data> data_;

    const VehicleParameters& vehicle() const;
    double clearance() const;
    double world_min_x() const;
    double world_min_y() const;
    double world_max_x() const;
    double world_max_y() const;
    bool valid() const;
    bool invalid_geometry() const;
    const std::vector<Polygon>& obstacle_polygons() const;

    friend class CollisionChecker;
};

}  // namespace parking_adas
