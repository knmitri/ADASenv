#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "parking_adas/domain/types.hpp"

namespace parking_adas {

class PlanningContext;

enum class ReedsSheppSegmentType {
    Left,
    Straight,
    Right,
};

struct ReedsSheppSegment {
    ReedsSheppSegmentType type = ReedsSheppSegmentType::Straight;
    double signed_length = 0.0;
};

struct ReedsSheppPath {
    std::string family;
    std::vector<ReedsSheppSegment> segments;
    double geometric_length = 0.0;
};

// ReedsSheppImplementation is declared in parking_adas/domain/types.hpp so the
// planner diagnostics can name the connector implementation.

// Outcome of validating/sampling a single connector candidate.
enum class ReedsSheppRejectionReason {
    Accepted,
    InitialPoseCollision,
    InitialDirectionMismatch,
    PoseCollision,
    SweptCollision,
    GoalEvaluatorRejection,
    DeadlineExpired,
    InvalidCandidate,
};

const char* to_string(ReedsSheppRejectionReason reason);

// Adds one connector call's diagnostics into an aggregate (used by planners
// that issue several connect() calls). Counters and planning time are summed;
// the selected-path fields follow the most recent accepted call.
void accumulate_connector_diagnostics(
    ReedsSheppConnectorDiagnostics& total,
    const ReedsSheppConnectorDiagnostics& call);

class ReedsSheppConnector {
public:
    // Enumerates all feasible canonical analytical paths in metres. This stage
    // is purely geometric and does not consult obstacles.
    std::vector<ReedsSheppPath> analytical_candidates(
        const Pose& start,
        const Pose& goal,
        double turning_radius) const;

    // `diagnostics`, when non-null, is fully reset at the start of the call and
    // describes only this call (also on failure).
    bool connect(
        const Pose& start,
        const Pose& goal,
        int initial_direction,
        const PlanningContext& context,
        const std::chrono::steady_clock::time_point& deadline,
        std::vector<Waypoint>& result,
        ReedsSheppConnectorDiagnostics* diagnostics = nullptr) const;

    bool connect(
        const Pose& start,
        const Pose& goal,
        int initial_direction,
        const PlanningContext& context,
        const std::chrono::steady_clock::time_point& deadline,
        std::vector<Waypoint>& result,
        ReedsSheppImplementation implementation,
        ReedsSheppConnectorDiagnostics* diagnostics = nullptr) const;

private:
    bool connect_numerical(
        const Pose& start,
        const Pose& goal,
        int initial_direction,
        const PlanningContext& context,
        const std::chrono::steady_clock::time_point& deadline,
        std::vector<Waypoint>& result,
        ReedsSheppConnectorDiagnostics& diagnostics) const;
};

}  // namespace parking_adas
