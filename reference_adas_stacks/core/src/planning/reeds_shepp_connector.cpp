#include "parking_adas/planning/reeds_shepp_connector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "parking_adas/common/math.hpp"
#include "parking_adas/planning/planning_context.hpp"

namespace parking_adas {
namespace {

constexpr double kArc = 1.0;
constexpr double kStraight = 0.0;
constexpr double kSampleSpacing = 0.10;
constexpr double kMaxSegmentLength = 12.0;
constexpr double kPositionTolerance = 0.015;
constexpr double kHeadingTolerance = 0.01;

struct Segment {
    double curvature_sign = 0.0;
    int direction = 1;
    double length = 0.0;
};

struct Candidate {
    std::vector<Segment> segments;
    double cost = std::numeric_limits<double>::infinity();
};

Pose integrate_segment(const Pose& pose, double curvature, double distance) {
    if (std::abs(curvature) < 1e-9) {
        return {pose.x + distance * std::cos(pose.yaw),
                pose.y + distance * std::sin(pose.yaw), pose.yaw};
    }

    const double yaw_delta = curvature * distance;
    return {
        pose.x + (std::sin(pose.yaw + yaw_delta) - std::sin(pose.yaw)) / curvature,
        pose.y + (-std::cos(pose.yaw + yaw_delta) + std::cos(pose.yaw)) / curvature,
        normalize_angle(pose.yaw + yaw_delta),
    };
}

Pose integrate_segments(const Pose& start, const std::vector<Segment>& segments,
                        double max_curvature) {
    Pose pose = start;
    for (const Segment& segment : segments) {
        pose = integrate_segment(
            pose, segment.curvature_sign * max_curvature,
            segment.direction * segment.length);
    }
    return pose;
}

std::array<double, 3> residual(const Pose& pose, const Pose& goal) {
    return {pose.x - goal.x, pose.y - goal.y,
            normalize_angle(pose.yaw - goal.yaw)};
}

bool solve_linear_system(std::vector<std::vector<double>> matrix,
                         std::vector<double> vector,
                         std::vector<double>& solution) {
    const std::size_t size = vector.size();
    for (std::size_t column = 0; column < size; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < size; ++row) {
            if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (std::abs(matrix[pivot][column]) < 1e-10) return false;
        std::swap(matrix[column], matrix[pivot]);
        std::swap(vector[column], vector[pivot]);
        const double divisor = matrix[column][column];
        for (std::size_t entry = column; entry < size; ++entry) {
            matrix[column][entry] /= divisor;
        }
        vector[column] /= divisor;
        for (std::size_t row = 0; row < size; ++row) {
            if (row == column) continue;
            const double factor = matrix[row][column];
            for (std::size_t entry = column; entry < size; ++entry) {
                matrix[row][entry] -= factor * matrix[column][entry];
            }
            vector[row] -= factor * vector[column];
        }
    }
    solution = std::move(vector);
    return true;
}

double squared_error(const std::array<double, 3>& value) {
    return value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
}

bool solve_pattern(std::vector<Segment>& segments, const Pose& start, const Pose& goal,
                   double max_curvature) {
    constexpr double perturbation = 1e-4;
    constexpr double damping = 1e-5;
    for (int iteration = 0; iteration < 60; ++iteration) {
        const Pose endpoint = integrate_segments(start, segments, max_curvature);
        const std::array<double, 3> error = residual(endpoint, goal);
        if (std::hypot(error[0], error[1]) < kPositionTolerance &&
            std::abs(error[2]) < kHeadingTolerance) {
            return true;
        }

        const std::size_t count = segments.size();
        std::vector<std::array<double, 3>> jacobian(count);
        for (std::size_t index = 0; index < count; ++index) {
            segments[index].length += perturbation;
            const std::array<double, 3> perturbed = residual(
                integrate_segments(start, segments, max_curvature), goal);
            segments[index].length -= perturbation;
            for (int row = 0; row < 3; ++row) {
                jacobian[index][row] = (perturbed[row] - error[row]) / perturbation;
            }
        }

        std::vector<std::vector<double>> normal(
            count, std::vector<double>(count, 0.0));
        std::vector<double> rhs(count, 0.0);
        for (std::size_t row = 0; row < count; ++row) {
            for (std::size_t column = 0; column < count; ++column) {
                for (int component = 0; component < 3; ++component) {
                    normal[row][column] +=
                        jacobian[row][component] * jacobian[column][component];
                }
            }
            normal[row][row] += damping;
            for (int component = 0; component < 3; ++component) {
                rhs[row] -= jacobian[row][component] * error[component];
            }
        }

        std::vector<double> update;
        if (!solve_linear_system(normal, rhs, update)) return false;

        const double initial_error = squared_error(error);
        bool improved = false;
        for (double scale = 1.0; scale >= 1.0 / 64.0; scale *= 0.5) {
            std::vector<Segment> trial = segments;
            for (std::size_t index = 0; index < count; ++index) {
                trial[index].length = std::max(
                    0.0, std::min(kMaxSegmentLength,
                                  trial[index].length + scale * update[index]));
            }
            if (squared_error(residual(integrate_segments(start, trial, max_curvature), goal)) <
                initial_error) {
                segments = std::move(trial);
                improved = true;
                break;
            }
        }
        if (!improved) return false;
    }
    return false;
}

std::vector<std::vector<double>> canonical_words() {
    return {
        {kArc, kStraight, kArc}, {-kArc, kStraight, -kArc},
        {kArc, kStraight, -kArc}, {-kArc, kStraight, kArc},
        {kArc, -kArc, kArc}, {-kArc, kArc, -kArc},
        {kArc, -kArc, kStraight, kArc}, {-kArc, kArc, kStraight, -kArc},
        {kArc, kStraight, -kArc, kArc}, {-kArc, kStraight, kArc, -kArc},
        {kArc, -kArc, kArc, -kArc}, {-kArc, kArc, -kArc, kArc},
        {kArc, -kArc, kStraight, kArc, -kArc},
        {-kArc, kArc, kStraight, -kArc, kArc},
    };
}

std::vector<std::vector<double>> seeds(std::size_t segment_count, double distance) {
    const double base = std::max(0.15, distance / static_cast<double>(segment_count));
    std::vector<std::vector<double>> result;
    result.push_back(std::vector<double>(segment_count, base));
    result.push_back(std::vector<double>(segment_count, 0.2));
    std::vector<double> outer(segment_count, 0.15);
    outer.front() = std::max(0.2, distance * 0.45);
    outer.back() = std::max(0.2, distance * 0.45);
    result.push_back(std::move(outer));
    std::vector<double> leading(segment_count, 0.1);
    leading.front() = std::max(0.2, distance * 0.75);
    result.push_back(std::move(leading));
    std::vector<double> trailing(segment_count, 0.1);
    trailing.back() = std::max(0.2, distance * 0.75);
    result.push_back(std::move(trailing));
    result.push_back(std::vector<double>(segment_count, std::max(0.5, distance * 0.6)));
    return result;
}

ReedsSheppRejectionReason sample_candidate(const Candidate& candidate, const Pose& start,
                                          const PlanningContext& context,
                                          std::vector<Waypoint>& result) {
    const PlanRequest& request = context.request();
    Pose pose = start;
    const double max_curvature = curvature_from_steering_angle(
        request.vehicle.max_steering_angle, request.vehicle.wheelbase);
    std::vector<Waypoint> sampled;
    for (const Segment& segment : candidate.segments) {
        if (segment.length < 1e-4) continue;
        const int samples = std::max(1, static_cast<int>(std::ceil(segment.length / kSampleSpacing)));
        const double step = segment.length / static_cast<double>(samples);
        for (int index = 0; index < samples; ++index) {
            pose = integrate_segment(
                pose, segment.curvature_sign * max_curvature, segment.direction * step);
            if (context.collision_checker().is_collision(pose)) {
                return ReedsSheppRejectionReason::PoseCollision;
            }
            sampled.push_back({
                pose.x, pose.y, segment.direction, pose.yaw,
                segment.curvature_sign * max_curvature,
                segment.direction > 0 ? 0.25 : -0.25,
                false,
            });
        }
    }
    if (sampled.empty()) return ReedsSheppRejectionReason::InvalidCandidate;
    if (!context.goal_evaluator().is_satisfied(pose)) {
        return ReedsSheppRejectionReason::GoalEvaluatorRejection;
    }
    result = std::move(sampled);
    return ReedsSheppRejectionReason::Accepted;
}

}  // namespace

const char* to_string(ReedsSheppRejectionReason reason) {
    switch (reason) {
        case ReedsSheppRejectionReason::Accepted: return "accepted";
        case ReedsSheppRejectionReason::InitialPoseCollision: return "initial_pose_collision";
        case ReedsSheppRejectionReason::InitialDirectionMismatch: return "initial_direction_mismatch";
        case ReedsSheppRejectionReason::PoseCollision: return "pose_collision";
        case ReedsSheppRejectionReason::SweptCollision: return "swept_collision";
        case ReedsSheppRejectionReason::GoalEvaluatorRejection: return "goal_evaluator_rejection";
        case ReedsSheppRejectionReason::DeadlineExpired: return "deadline_expired";
        case ReedsSheppRejectionReason::InvalidCandidate: return "invalid_candidate";
    }
    return "unknown";
}

void accumulate_connector_diagnostics(
    ReedsSheppConnectorDiagnostics& total,
    const ReedsSheppConnectorDiagnostics& call) {
    if (call.connector_attempts == 0) return;
    total.implementation = call.implementation;
    total.connector_attempts += call.connector_attempts;
    total.accepted_connector_attempts += call.accepted_connector_attempts;
    total.planning_time_ms += call.planning_time_ms;
    total.invalid_configuration = total.invalid_configuration || call.invalid_configuration;
    if (call.solution_accepted) {
        total.solution_accepted = true;
        total.selected_path_length_m = call.selected_path_length_m;
        total.selected_segment_count = call.selected_segment_count;
        total.selected_waypoint_count = call.selected_waypoint_count;
        total.selected_path_family = call.selected_path_family;
    }
    NumericalConnectorCounters& n = total.numerical;
    const NumericalConnectorCounters& cn = call.numerical;
    n.combinations_attempted += cn.combinations_attempted;
    n.solver_failures += cn.solver_failures;
    n.solver_converged += cn.solver_converged;
    n.initial_direction_rejections += cn.initial_direction_rejections;
    n.cost_pruned_candidates += cn.cost_pruned_candidates;
    n.candidates_sampled += cn.candidates_sampled;
    n.pose_collision_rejections += cn.pose_collision_rejections;
    n.goal_rejections += cn.goal_rejections;
    n.invalid_candidate_rejections += cn.invalid_candidate_rejections;
    n.deadline_rejections += cn.deadline_rejections;
    n.accepted_candidates += cn.accepted_candidates;
    n.initial_pose_collision_observations += cn.initial_pose_collision_observations;
    AnalyticalConnectorCounters& a = total.analytical;
    const AnalyticalConnectorCounters& ca = call.analytical;
    a.geometric_candidates_generated += ca.geometric_candidates_generated;
    a.candidates_examined += ca.candidates_examined;
    a.initial_pose_collision_rejections += ca.initial_pose_collision_rejections;
    a.initial_direction_rejections += ca.initial_direction_rejections;
    a.swept_collision_rejections += ca.swept_collision_rejections;
    a.goal_rejections += ca.goal_rejections;
    a.invalid_candidate_rejections += ca.invalid_candidate_rejections;
    a.deadline_rejections += ca.deadline_rejections;
    a.accepted_candidates += ca.accepted_candidates;
}

bool ReedsSheppConnector::connect_numerical(
    const Pose& start,
    const Pose& goal,
    int initial_direction,
    const PlanningContext& context,
    const std::chrono::steady_clock::time_point& deadline,
    std::vector<Waypoint>& result,
    ReedsSheppConnectorDiagnostics& diagnostics) const {
    NumericalConnectorCounters& counters = diagnostics.numerical;
    const PlanRequest& request = context.request();
    const double max_curvature = curvature_from_steering_angle(
        request.vehicle.max_steering_angle, request.vehicle.wheelbase);
    if (max_curvature <= 1e-9) {
        diagnostics.invalid_configuration = true;
        return false;
    }

    Candidate best;
    // Observation only: the numerical search never validates the start pose, so
    // this probe is reported but must not influence candidate acceptance.
    if (context.collision_checker().is_collision(start)) {
        ++counters.initial_pose_collision_observations;
    }
    bool deadline_expired = false;
    const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
    for (const std::vector<double>& word : canonical_words()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            deadline_expired = true;
            break;
        }
        const std::size_t count = word.size();
        const unsigned direction_variants = 1U << count;
        const int seeds_per_variant = static_cast<int>(seeds(count, distance).size());
        for (unsigned mask = 0; mask < direction_variants; ++mask) {
            if (std::chrono::steady_clock::now() >= deadline) {
                deadline_expired = true;
                break;
            }
            const int first_direction = (mask & 1U) == 0U ? 1 : -1;
            if (first_direction != initial_direction) {
                counters.initial_direction_rejections += seeds_per_variant;
                continue;
            }
            for (const std::vector<double>& seed : seeds(count, distance)) {
                std::vector<Segment> segments;
                segments.reserve(count);
                for (std::size_t index = 0; index < count; ++index) {
                    segments.push_back({
                        word[index], (mask & (1U << index)) == 0U ? 1 : -1,
                        seed[index],
                    });
                }
                ++counters.combinations_attempted;
                if (!solve_pattern(segments, start, goal, max_curvature)) {
                    ++counters.solver_failures;
                    continue;
                }
                ++counters.solver_converged;

                double cost = 0.0;
                for (std::size_t index = 0; index < count; ++index) {
                    cost += segments[index].length;
                    if (index > 0 && segments[index].direction != segments[index - 1].direction) {
                        cost += 0.35;
                    }
                }
                Candidate candidate{std::move(segments), cost};
                if (!(candidate.cost < best.cost)) {
                    ++counters.cost_pruned_candidates;
                    continue;
                }
                std::vector<Waypoint> sampled;
                ++counters.candidates_sampled;
                const ReedsSheppRejectionReason reason =
                    sample_candidate(candidate, start, context, sampled);
                switch (reason) {
                    case ReedsSheppRejectionReason::Accepted:
                        ++counters.accepted_candidates;
                        best = std::move(candidate);
                        result = std::move(sampled);
                        diagnostics.selected_path_length_m = 0.0;
                        diagnostics.selected_segment_count = 0;
                        for (const Segment& segment : best.segments) {
                            if (segment.length < 1e-4) continue;
                            diagnostics.selected_path_length_m += segment.length;
                            ++diagnostics.selected_segment_count;
                        }
                        diagnostics.selected_waypoint_count =
                            static_cast<int>(result.size());
                        break;
                    case ReedsSheppRejectionReason::PoseCollision:
                        ++counters.pose_collision_rejections;
                        break;
                    case ReedsSheppRejectionReason::GoalEvaluatorRejection:
                        ++counters.goal_rejections;
                        break;
                    default:
                        ++counters.invalid_candidate_rejections;
                        break;
                }
            }
        }
        if (deadline_expired) break;
    }
    if (deadline_expired) ++counters.deadline_rejections;
    return std::isfinite(best.cost);
}

}  // namespace parking_adas
