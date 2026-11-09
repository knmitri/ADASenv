#pragma once

#include <string>
#include <vector>

namespace parking_adas {

struct Point2 {
    double x = 0.0;
    double y = 0.0;
};

using Footprint = std::vector<Point2>;

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct Obstacle {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double length = 0.0;
    double width = 0.0;
    Footprint footprint;
};

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
    double steering_angle = 0.0;
    std::vector<Obstacle> dynamic_obstacles;
};

struct MotionCommand {
    double acceleration = 0.0;
    double steering = 0.0;
};

struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    int direction = 1;
    double heading = 0.0;
    double curvature = 0.0;
    double target_speed = 0.0;
    bool gear_change = false;
};

struct VehicleParameters {
    double length = 4.67;
    double width = 1.86;
    double wheelbase = 2.82;
    double front_overhang = 0.89;
    double rear_overhang = 0.96;
    double max_steering_angle = 0.7853981633974483;
    double max_steering_rate = 0.7853981633974483;
    double max_acceleration = 9.0;
    double max_deceleration = -9.0;
    double max_speed = 2.0;
    double max_reverse_speed = 1.0;
};

struct PlanRequest {
    double start_x = 0.0;
    double start_y = 0.0;
    double start_heading = 0.0;
    int start_direction = 1;
    double start_velocity = 0.0;
    double start_steering = 0.0;
    Pose goal;
    Footprint goal_slot_footprint;
    VehicleParameters vehicle;
    std::vector<Obstacle> static_obstacles;
    double world_min_x = -2.0;
    double world_min_y = -2.0;
    double world_max_x = 20.0;
    double world_max_y = 20.0;
    double dt = 0.05;
    double replan_position_threshold = 0.75;
    double replan_heading_threshold = 0.35;
    double goal_heading_tolerance = 0.17453292519943295;
};

using Path = std::vector<Waypoint>;
using Trajectory = std::vector<Waypoint>;
using PlanningRequest = PlanRequest;

enum class ReedsSheppImplementation {
    Numerical,
    Analytical,
};

// Counters for the numerical Reeds-Shepp connector. Its unit of work is a
// word/direction-mask/seed combination handed to the iterative solver. These
// are NOT analytical geometric candidates.
//
// Accounting identities for one connect() call:
//   combinations_attempted = solver_failures + solver_converged
//   solver_converged       = cost_pruned_candidates + candidates_sampled
//   candidates_sampled     = accepted_candidates + pose_collision_rejections +
//                            goal_rejections + invalid_candidate_rejections
// initial_direction_rejections is counted in the same unit (word/direction/
// seed combinations) but those combinations never reach the solver, so it is
// disjoint from combinations_attempted. deadline_rejections counts deadline
// expiry events that terminated the search (not individual combinations).
// accepted_candidates counts candidates that passed every check and became the
// best-so-far, so one call may report more than one; only the cheapest is returned.
//
// initial_pose_collision_observations is OBSERVATIONAL ONLY. Unlike the
// analytical connector, the numerical connector does not validate the start
// pose: it only checks poses sampled along candidates. This field records that
// the start pose was found colliding by an independent probe. The probe runs
// once per numerical connector call that reaches the numerical search, so the
// value is 0 or 1 per call (summed over calls by planners). A call that returns
// early for an invalid vehicle configuration (non-positive curvature) never
// reaches the search, performs no probe and reports 0. The field is not a
// rejection, does not appear in any accounting identity, and never alters
// candidate acceptance, so it is NOT equivalent to
// AnalyticalConnectorCounters::initial_pose_collision_rejections.
struct NumericalConnectorCounters {
    int combinations_attempted = 0;
    int solver_failures = 0;
    int solver_converged = 0;
    int initial_direction_rejections = 0;
    int cost_pruned_candidates = 0;
    int candidates_sampled = 0;
    int pose_collision_rejections = 0;
    int goal_rejections = 0;
    int invalid_candidate_rejections = 0;
    int deadline_rejections = 0;
    int accepted_candidates = 0;
    int initial_pose_collision_observations = 0;
};

// Counters for the analytical Reeds-Shepp connector. Its unit of work is one
// closed-form geometric candidate from the shortest-first candidate list.
//
// Accounting identities for one connect() call:
//   candidates_examined <= geometric_candidates_generated
//   candidates_examined = accepted_candidates + initial_pose_collision_rejections +
//                         initial_direction_rejections + swept_collision_rejections +
//                         goal_rejections + invalid_candidate_rejections +
//                         deadline_rejections
// The analytical connector stops at the first accepted candidate, so
// candidates after it (or after a deadline) are generated but not examined.
struct AnalyticalConnectorCounters {
    int geometric_candidates_generated = 0;
    int candidates_examined = 0;
    int initial_pose_collision_rejections = 0;
    int initial_direction_rejections = 0;
    int swept_collision_rejections = 0;
    int goal_rejections = 0;
    int invalid_candidate_rejections = 0;
    int deadline_rejections = 0;
    int accepted_candidates = 0;
};

// Structured diagnostics for ReedsSheppConnector::connect(). A single call
// reports connector_attempts == 1; Hybrid A* sums the per-call values over all
// of its goal-connection attempts. Only the counter block that matches
// `implementation` is meaningful.
struct ReedsSheppConnectorDiagnostics {
    ReedsSheppImplementation implementation = ReedsSheppImplementation::Numerical;
    int connector_attempts = 0;
    int accepted_connector_attempts = 0;
    double planning_time_ms = 0.0;
    bool solution_accepted = false;
    bool invalid_configuration = false;
    double selected_path_length_m = 0.0;
    int selected_segment_count = 0;
    int selected_waypoint_count = 0;
    std::string selected_path_family;
    NumericalConnectorCounters numerical;
    AnalyticalConnectorCounters analytical;
};

struct PlanningDiagnostics {
    int expanded_nodes = 0;
    int goal_connection_attempts = 0;
    double closest_goal_distance = 0.0;
    double planning_time_ms = 0.0;
    bool time_limited = false;
    bool solution_found = false;
    bool partial_plan = false;
    ReedsSheppConnectorDiagnostics reeds_shepp_connector;
};

struct CommandDiagnostics {
    bool replanned = false;
    bool safety_override = false;
    const char* replan_reason = "none";
};

struct TrackingStatus {
    bool trajectory_empty = false;
    bool at_trajectory_end = false;
    bool stalled = false;
    double cross_track_error = 0.0;
    double terminal_distance = 0.0;
    double terminal_heading_error = 0.0;
};

}  // namespace parking_adas
