#include "parking_adas/planning/reeds_shepp_connector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "parking_adas/common/math.hpp"
#include "parking_adas/planning/planning_context.hpp"

namespace parking_adas {
namespace {

constexpr double kAnalyticPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kAnalyticPi;
constexpr double kFormulaTolerance = 1e-10;
constexpr double kEndpointTolerance = 1e-10;
constexpr double kSweptPointMotionBound = 0.025;

struct NormalizedPose {
    double x;
    double y;
    double yaw;
};

struct FormulaLengths {
    double first = 0.0;
    double second = 0.0;
    double third = 0.0;
};

using Formula = bool (*)(double, double, double, FormulaLengths&);

using SegmentTypes = std::vector<ReedsSheppSegmentType>;

double wrap_to_pi(double angle) {
    double wrapped = std::fmod(angle, kTwoPi);
    if (wrapped < -kAnalyticPi) {
        wrapped += kTwoPi;
    } else if (wrapped > kAnalyticPi) {
        wrapped -= kTwoPi;
    }
    return wrapped;
}

void polar(double x, double y, double& radius, double& angle) {
    radius = std::hypot(x, y);
    angle = std::atan2(y, x);
}

bool nonnegative(double& value) {
    if (value < -kFormulaTolerance) return false;
    if (std::abs(value) <= kFormulaTolerance) value = 0.0;
    return true;
}

bool nonpositive(double& value) {
    if (value > kFormulaTolerance) return false;
    if (std::abs(value) <= kFormulaTolerance) value = 0.0;
    return true;
}

void tau_omega(double u, double v, double xi, double eta, double phi,
               double& tau, double& omega) {
    const double delta = wrap_to_pi(u - v);
    const double a = std::sin(u) - std::sin(delta);
    const double b = std::cos(u) - std::cos(delta) - 1.0;
    const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
    const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    tau = wrap_to_pi(t2 < 0.0 ? t1 + kAnalyticPi : t1);
    omega = wrap_to_pi(tau - u + v - phi);
}

// Reeds-Shepp paper formula 8.1: L+ S+ L+.
bool l_plus_s_plus_l(double x, double y, double phi, FormulaLengths& out) {
    double radius = 0.0;
    polar(x - std::sin(phi), y - 1.0 + std::cos(phi), radius, out.first);
    if (!nonnegative(out.first)) return false;
    out.second = radius;
    out.third = wrap_to_pi(phi - out.first);
    return nonnegative(out.third);
}

// Reeds-Shepp paper formula 8.2: L+ S+ R+.
bool l_plus_s_plus_r(double x, double y, double phi, FormulaLengths& out) {
    double radius = 0.0;
    double angle = 0.0;
    polar(x + std::sin(phi), y - 1.0 - std::cos(phi), radius, angle);
    const double squared_radius = radius * radius;
    if (squared_radius < 4.0 - kFormulaTolerance) return false;
    out.second = std::sqrt(std::max(0.0, squared_radius - 4.0));
    const double theta = std::atan2(2.0, out.second);
    out.first = wrap_to_pi(angle + theta);
    out.third = wrap_to_pi(out.first - phi);
    return nonnegative(out.first) && nonnegative(out.third);
}

// Reeds-Shepp paper formulas 8.3/8.4 (corrected): L+ R- L+.
bool l_plus_r_minus_l(double x, double y, double phi, FormulaLengths& out) {
    double radius = 0.0;
    double angle = 0.0;
    polar(x - std::sin(phi), y - 1.0 + std::cos(phi), radius, angle);
    if (radius > 4.0 + kFormulaTolerance) return false;
    out.second = -2.0 * std::asin(std::min(1.0, radius / 4.0));
    out.first = wrap_to_pi(angle + 0.5 * out.second + kAnalyticPi);
    out.third = wrap_to_pi(phi - out.first + out.second);
    return nonnegative(out.first) && nonpositive(out.second);
}

// Reeds-Shepp paper formula 8.7: L+ R+ L- R-.
bool l_plus_r_plus_l_minus_r_minus(
    double x, double y, double phi, FormulaLengths& out) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const double rho = 0.25 * (2.0 + std::hypot(xi, eta));
    if (rho > 1.0 + kFormulaTolerance) return false;
    out.second = std::acos(std::max(-1.0, std::min(1.0, rho)));
    tau_omega(out.second, -out.second, xi, eta, phi, out.first, out.third);
    return nonnegative(out.first) && nonpositive(out.third);
}

// Reeds-Shepp paper formula 8.8: L+ R- L- R+.
bool l_plus_r_minus_l_minus_r_plus(
    double x, double y, double phi, FormulaLengths& out) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if (rho < -kFormulaTolerance || rho > 1.0 + kFormulaTolerance) return false;
    out.second = -std::acos(std::max(-1.0, std::min(1.0, rho)));
    if (out.second < -0.5 * kAnalyticPi - kFormulaTolerance) return false;
    tau_omega(out.second, out.second, xi, eta, phi, out.first, out.third);
    return nonnegative(out.first) && nonnegative(out.third);
}

// Reeds-Shepp paper formula 8.9: L+ R- S- L-.
bool l_plus_r_minus_s_minus_l_minus(
    double x, double y, double phi, FormulaLengths& out) {
    const double xi = x - std::sin(phi);
    const double eta = y - 1.0 + std::cos(phi);
    double radius = 0.0;
    double angle = 0.0;
    polar(xi, eta, radius, angle);
    if (radius < 2.0 - kFormulaTolerance) return false;
    const double root = std::sqrt(std::max(0.0, radius * radius - 4.0));
    out.second = 2.0 - root;
    out.first = wrap_to_pi(angle + std::atan2(root, -2.0));
    out.third = wrap_to_pi(phi - 0.5 * kAnalyticPi - out.first);
    return nonnegative(out.first) && nonpositive(out.second) &&
        nonpositive(out.third);
}

// Reeds-Shepp paper formula 8.10: L+ R- S- R-.
bool l_plus_r_minus_s_minus_r_minus(
    double x, double y, double phi, FormulaLengths& out) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    double radius = 0.0;
    polar(-eta, xi, radius, out.first);
    if (radius < 2.0 - kFormulaTolerance) return false;
    out.second = 2.0 - radius;
    out.third = wrap_to_pi(out.first + 0.5 * kAnalyticPi - phi);
    return nonnegative(out.first) && nonpositive(out.second) &&
        nonpositive(out.third);
}

// Reeds-Shepp paper formula 8.11 (corrected): L+ R- S- L- R+.
bool l_plus_r_minus_s_minus_l_minus_r_plus(
    double x, double y, double phi, FormulaLengths& out) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    double radius = 0.0;
    double angle = 0.0;
    polar(xi, eta, radius, angle);
    if (radius < 2.0 - kFormulaTolerance) return false;
    out.second = 4.0 - std::sqrt(std::max(0.0, radius * radius - 4.0));
    if (!nonpositive(out.second)) return false;
    out.first = wrap_to_pi(std::atan2(
        (4.0 - out.second) * xi - 2.0 * eta,
        -2.0 * xi + (out.second - 4.0) * eta));
    out.third = wrap_to_pi(out.first - phi);
    return nonnegative(out.first) && nonnegative(out.third);
}

SegmentTypes reflected(const SegmentTypes& types) {
    SegmentTypes result = types;
    for (ReedsSheppSegmentType& type : result) {
        if (type == ReedsSheppSegmentType::Left) {
            type = ReedsSheppSegmentType::Right;
        } else if (type == ReedsSheppSegmentType::Right) {
            type = ReedsSheppSegmentType::Left;
        }
    }
    return result;
}

NormalizedPose normalized_goal(const Pose& start, const Pose& goal, double radius) {
    const double cosine = std::cos(start.yaw);
    const double sine = std::sin(start.yaw);
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    return {
        (cosine * dx + sine * dy) / radius,
        (-sine * dx + cosine * dy) / radius,
        wrap_to_pi(goal.yaw - start.yaw),
    };
}

double turn_sign(ReedsSheppSegmentType type) {
    if (type == ReedsSheppSegmentType::Left) return 1.0;
    if (type == ReedsSheppSegmentType::Right) return -1.0;
    return 0.0;
}

bool endpoint_matches(const SegmentTypes& types, const std::vector<double>& lengths,
                      const NormalizedPose& goal) {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    for (std::size_t index = 0; index < types.size(); ++index) {
        const double curvature = turn_sign(types[index]);
        const double distance = lengths[index];
        if (std::abs(curvature) < 1e-12) {
            x += distance * std::cos(yaw);
            y += distance * std::sin(yaw);
        } else {
            const double next_yaw = yaw + curvature * distance;
            x += (std::sin(next_yaw) - std::sin(yaw)) / curvature;
            y += (-std::cos(next_yaw) + std::cos(yaw)) / curvature;
            yaw = next_yaw;
        }
    }
    return std::hypot(x - goal.x, y - goal.y) <=
            kEndpointTolerance * std::max(1.0, std::hypot(goal.x, goal.y)) &&
        std::abs(wrap_to_pi(yaw - goal.yaw)) <= kEndpointTolerance;
}

void append_path(std::vector<ReedsSheppPath>& paths, const std::string& family,
                 const SegmentTypes& types, const std::vector<double>& normalized_lengths,
                 double radius, const NormalizedPose& goal) {
    if (types.size() != normalized_lengths.size() ||
        !endpoint_matches(types, normalized_lengths, goal)) {
        return;
    }

    ReedsSheppPath path;
    path.family = family;
    path.segments.reserve(types.size());
    for (std::size_t index = 0; index < types.size(); ++index) {
        const double length = normalized_lengths[index] * radius;
        path.segments.push_back({types[index], length});
        path.geometric_length += std::abs(length);
    }
    if (!std::isfinite(path.geometric_length)) return;
    paths.push_back(std::move(path));
}

std::vector<double> values(const FormulaLengths& lengths) {
    return {lengths.first, lengths.second, lengths.third};
}

template <typename MakeLengths>
void append_four_symmetries(std::vector<ReedsSheppPath>& paths,
                            const std::string& family, Formula formula,
                            const SegmentTypes& base_types, MakeLengths make_lengths,
                            const NormalizedPose& goal, double radius) {
    const SegmentTypes mirror_types = reflected(base_types);
    struct Transform {
        double x_sign;
        double y_sign;
        double phi_sign;
        bool time_flip;
        bool reflection;
    };
    constexpr std::array<Transform, 4> transforms{{
        {1.0, 1.0, 1.0, false, false},
        {-1.0, 1.0, -1.0, true, false},
        {1.0, -1.0, -1.0, false, true},
        {-1.0, -1.0, 1.0, true, true},
    }};

    for (const Transform& transform : transforms) {
        FormulaLengths result;
        if (!formula(transform.x_sign * goal.x, transform.y_sign * goal.y,
                     transform.phi_sign * goal.yaw, result)) {
            continue;
        }
        std::vector<double> lengths = make_lengths(result);
        if (transform.time_flip) {
            for (double& length : lengths) length = -length;
        }
        append_path(paths, family,
                    transform.reflection ? mirror_types : base_types,
                    lengths, radius, goal);
    }
}

template <typename MakeLengths>
void append_backward_symmetries(
    std::vector<ReedsSheppPath>& paths, const std::string& family,
    Formula formula, const NormalizedPose& goal,
    const SegmentTypes& base_types, const SegmentTypes& mirror_types,
    MakeLengths make_lengths, double radius) {
    const double cosine = std::cos(goal.yaw);
    const double sine = std::sin(goal.yaw);
    const double x_back = goal.x * cosine + goal.y * sine;
    const double y_back = goal.x * sine - goal.y * cosine;
    struct Transform {
        double x;
        double y;
        double phi;
        bool time_flip;
        bool reflection;
    };
    constexpr std::array<Transform, 4> transforms{{
        {1.0, 1.0, 1.0, false, false},
        {-1.0, 1.0, -1.0, true, false},
        {1.0, -1.0, -1.0, false, true},
        {-1.0, -1.0, 1.0, true, true},
    }};
    for (const Transform& transform : transforms) {
        FormulaLengths result;
        if (!formula(transform.x * x_back, transform.y * y_back,
                     transform.phi * goal.yaw, result)) {
            continue;
        }
        std::vector<double> lengths = make_lengths(result);
        if (transform.time_flip) {
            for (double& length : lengths) length = -length;
        }
        append_path(paths, family,
                    transform.reflection ? mirror_types : base_types,
                    lengths, radius, goal);
    }
}

bool finite_pose(const Pose& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

double segment_curvature(ReedsSheppSegmentType type, double radius) {
    return turn_sign(type) / radius;
}

Pose integrate_exact(Pose pose, double curvature, double signed_distance) {
    if (std::abs(curvature) < 1e-12) {
        pose.x += signed_distance * std::cos(pose.yaw);
        pose.y += signed_distance * std::sin(pose.yaw);
        return pose;
    }
    const double next_yaw = pose.yaw + curvature * signed_distance;
    pose.x += (std::sin(next_yaw) - std::sin(pose.yaw)) / curvature;
    pose.y += (-std::cos(next_yaw) + std::cos(pose.yaw)) / curvature;
    pose.yaw = normalize_angle(next_yaw);
    return pose;
}

ReedsSheppRejectionReason sample_candidate(
                     const ReedsSheppPath& candidate, const Pose& start,
                     int initial_direction, const PlanningContext& context,
                     double radius,
                     const std::chrono::steady_clock::time_point& deadline,
                     std::vector<Waypoint>& result) {
    const PlanRequest& request = context.request();
    result.clear();
    if (context.collision_checker().is_collision(start)) {
        return ReedsSheppRejectionReason::InitialPoseCollision;
    }

    int first_direction = 0;
    for (const ReedsSheppSegment& segment : candidate.segments) {
        if (std::abs(segment.signed_length) > 1e-10) {
            first_direction = segment.signed_length > 0.0 ? 1 : -1;
            break;
        }
    }
    if (first_direction != 0 && initial_direction != 0 &&
        first_direction != initial_direction) {
        return ReedsSheppRejectionReason::InitialDirectionMismatch;
    }

    Pose pose = start;
    int previous_direction = 0;
    std::vector<Waypoint> sampled;
    constexpr double kMaximumConnectorClearance = 0.1;
    const double body_radius = std::hypot(
        std::max(request.vehicle.wheelbase + request.vehicle.front_overhang,
                 request.vehicle.rear_overhang) + kMaximumConnectorClearance,
        request.vehicle.width / 2.0 + kMaximumConnectorClearance);
    for (const ReedsSheppSegment& segment : candidate.segments) {
        if (std::chrono::steady_clock::now() >= deadline) {
            return ReedsSheppRejectionReason::DeadlineExpired;
        }
        if (std::abs(segment.signed_length) <= 1e-10) continue;

        const int direction = segment.signed_length > 0.0 ? 1 : -1;
        if (previous_direction != 0 && direction != previous_direction &&
            !sampled.empty()) {
            sampled.back().gear_change = true;
            sampled.back().target_speed = 0.0;
        }
        const double curvature = segment_curvature(segment.type, radius);
        const double point_motion_per_metre =
            1.0 + body_radius * std::abs(curvature);
        const double requested_samples = std::ceil(
            std::abs(segment.signed_length) * point_motion_per_metre /
            kSweptPointMotionBound);
        if (!std::isfinite(requested_samples) ||
            requested_samples > static_cast<double>(std::numeric_limits<int>::max())) {
            return ReedsSheppRejectionReason::InvalidCandidate;
        }
        const int sample_count = std::max(1, static_cast<int>(requested_samples));
        const double step = segment.signed_length / sample_count;
        for (int index = 0; index < sample_count; ++index) {
            if (std::chrono::steady_clock::now() >= deadline) {
                return ReedsSheppRejectionReason::DeadlineExpired;
            }
            const Pose previous_pose = pose;
            pose = integrate_exact(pose, curvature, step);
            if (context.collision_checker().is_swept_collision(
                    previous_pose, pose, std::abs(step),
                    std::abs(curvature * step))) {
                return ReedsSheppRejectionReason::SweptCollision;
            }
            sampled.push_back({
                pose.x, pose.y, direction, pose.yaw, curvature,
                direction > 0 ? 0.25 : -0.25, false,
            });
        }
        previous_direction = direction;
    }

    if (sampled.empty()) {
        sampled.push_back({
            pose.x, pose.y, initial_direction < 0 ? -1 : 1,
            pose.yaw, 0.0, 0.0, false,
        });
    }
    if (!context.goal_evaluator().is_satisfied(pose)) {
        return ReedsSheppRejectionReason::GoalEvaluatorRejection;
    }
    result = std::move(sampled);
    return ReedsSheppRejectionReason::Accepted;
}

}  // namespace

std::vector<ReedsSheppPath> ReedsSheppConnector::analytical_candidates(
    const Pose& start, const Pose& goal, double turning_radius) const {
    std::vector<ReedsSheppPath> candidates;
    if (!finite_pose(start) || !finite_pose(goal) ||
        !std::isfinite(turning_radius) || turning_radius <= 0.0) {
        return candidates;
    }
    const NormalizedPose target = normalized_goal(start, goal, turning_radius);

    const SegmentTypes lsl{
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Straight,
        ReedsSheppSegmentType::Left};
    const SegmentTypes lsr{
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Straight,
        ReedsSheppSegmentType::Right};
    const SegmentTypes lrl{
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Right,
        ReedsSheppSegmentType::Left};
    const SegmentTypes lrlr{
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Right,
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Right};
    const SegmentTypes lrsl{
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Right,
        ReedsSheppSegmentType::Straight, ReedsSheppSegmentType::Left};
    const SegmentTypes lrsr{
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Right,
        ReedsSheppSegmentType::Straight, ReedsSheppSegmentType::Right};
    const SegmentTypes lsrsl{
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Straight,
        ReedsSheppSegmentType::Right, ReedsSheppSegmentType::Left};
    const SegmentTypes rsrl{
        ReedsSheppSegmentType::Right, ReedsSheppSegmentType::Straight,
        ReedsSheppSegmentType::Right, ReedsSheppSegmentType::Left};
    const SegmentTypes lrsrl{
        ReedsSheppSegmentType::Left, ReedsSheppSegmentType::Right,
        ReedsSheppSegmentType::Straight, ReedsSheppSegmentType::Left,
        ReedsSheppSegmentType::Right};

    // CSC: equations 8.1 and 8.2, each with time-flip and reflection images.
    append_four_symmetries(candidates, "CSC", l_plus_s_plus_l, lsl,
        [](const FormulaLengths& v) { return values(v); }, target, turning_radius);
    append_four_symmetries(candidates, "CSC", l_plus_s_plus_r, lsr,
        [](const FormulaLengths& v) { return values(v); }, target, turning_radius);

    // CCC: equation 8.3/8.4 plus its backward construction.
    append_four_symmetries(candidates, "CCC", l_plus_r_minus_l, lrl,
        [](const FormulaLengths& v) { return values(v); }, target, turning_radius);
    append_backward_symmetries(candidates, "CCC", l_plus_r_minus_l, target,
        lrl, reflected(lrl),
        [](const FormulaLengths& v) {
            return std::vector<double>{v.third, v.second, v.first};
        }, turning_radius);

    // CCCC: equations 8.7 and 8.8.
    append_four_symmetries(candidates, "CCCC", l_plus_r_plus_l_minus_r_minus, lrlr,
        [](const FormulaLengths& v) {
            return std::vector<double>{v.first, v.second, -v.second, v.third};
        }, target, turning_radius);
    append_four_symmetries(candidates, "CCCC", l_plus_r_minus_l_minus_r_plus, lrlr,
        [](const FormulaLengths& v) {
            return std::vector<double>{v.first, v.second, v.second, v.third};
        }, target, turning_radius);

    // CCSC: equations 8.9 and 8.10 plus the backwards families.
    append_four_symmetries(candidates, "CCSC", l_plus_r_minus_s_minus_l_minus, lrsl,
        [](const FormulaLengths& v) {
            return std::vector<double>{v.first, -0.5 * kAnalyticPi, v.second, v.third};
        }, target, turning_radius);
    append_four_symmetries(candidates, "CCSC", l_plus_r_minus_s_minus_r_minus, lrsr,
        [](const FormulaLengths& v) {
            return std::vector<double>{v.first, -0.5 * kAnalyticPi, v.second, v.third};
        }, target, turning_radius);
    append_backward_symmetries(candidates, "CCSC", l_plus_r_minus_s_minus_l_minus,
        target, lsrsl, reflected(lsrsl),
        [](const FormulaLengths& v) {
            return std::vector<double>{v.third, v.second, -0.5 * kAnalyticPi, v.first};
        }, turning_radius);
    append_backward_symmetries(candidates, "CCSC", l_plus_r_minus_s_minus_r_minus,
        target, rsrl, reflected(rsrl),
        [](const FormulaLengths& v) {
            return std::vector<double>{v.third, v.second, -0.5 * kAnalyticPi, v.first};
        }, turning_radius);

    // CCSCC: corrected equation 8.11.
    append_four_symmetries(candidates, "CCSCC",
        l_plus_r_minus_s_minus_l_minus_r_plus, lrsrl,
        [](const FormulaLengths& v) {
            return std::vector<double>{
                v.first, -0.5 * kAnalyticPi, v.second,
                -0.5 * kAnalyticPi, v.third};
        }, target, turning_radius);

    std::stable_sort(candidates.begin(), candidates.end(),
        [](const ReedsSheppPath& first, const ReedsSheppPath& second) {
            return first.geometric_length < second.geometric_length;
        });
    return candidates;
}

bool ReedsSheppConnector::connect(
    const Pose& start, const Pose& goal, int initial_direction,
    const PlanningContext& context,
    const std::chrono::steady_clock::time_point& deadline,
    std::vector<Waypoint>& result,
    ReedsSheppConnectorDiagnostics* diagnostics) const {
#if defined(ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL) && ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL
    return connect(start, goal, initial_direction, context, deadline, result,
                   ReedsSheppImplementation::Analytical, diagnostics);
#else
    return connect(start, goal, initial_direction, context, deadline, result,
                   ReedsSheppImplementation::Numerical, diagnostics);
#endif
}

namespace {

bool connect_analytical(
    const ReedsSheppConnector& connector,
    const Pose& start, const Pose& goal, int initial_direction,
    const PlanningContext& context,
    const std::chrono::steady_clock::time_point& deadline,
    std::vector<Waypoint>& result,
    ReedsSheppConnectorDiagnostics& diagnostics) {
    const PlanRequest& request = context.request();
    AnalyticalConnectorCounters& counters = diagnostics.analytical;

    const double steering = std::abs(request.vehicle.max_steering_angle);
    const double wheelbase = request.vehicle.wheelbase;
    if (!std::isfinite(steering) || !std::isfinite(wheelbase) ||
        steering <= 1e-12 || steering >= 0.5 * kAnalyticPi ||
        wheelbase <= 1e-12) {
        diagnostics.invalid_configuration = true;
        return false;
    }
    const double turning_radius = wheelbase / std::tan(steering);
    if (!std::isfinite(turning_radius) || turning_radius <= 0.0) {
        diagnostics.invalid_configuration = true;
        return false;
    }

    const std::vector<ReedsSheppPath> candidates =
        connector.analytical_candidates(start, goal, turning_radius);
    counters.geometric_candidates_generated = static_cast<int>(candidates.size());
    for (const ReedsSheppPath& candidate : candidates) {
        if (std::chrono::steady_clock::now() >= deadline) {
            ++counters.candidates_examined;
            ++counters.deadline_rejections;
            break;
        }
        ++counters.candidates_examined;
        std::vector<Waypoint> sampled;
        const ReedsSheppRejectionReason reason = sample_candidate(
            candidate, start, initial_direction, context, turning_radius,
            deadline, sampled);
        switch (reason) {
            case ReedsSheppRejectionReason::Accepted:
                ++counters.accepted_candidates;
                result = std::move(sampled);
                diagnostics.selected_path_length_m = candidate.geometric_length;
                diagnostics.selected_segment_count = static_cast<int>(std::count_if(
                    candidate.segments.begin(), candidate.segments.end(),
                    [](const ReedsSheppSegment& segment) {
                        return std::abs(segment.signed_length) > 1e-10;
                    }));
                diagnostics.selected_waypoint_count = static_cast<int>(result.size());
                diagnostics.selected_path_family = candidate.family;
                return true;
            case ReedsSheppRejectionReason::InitialPoseCollision:
                ++counters.initial_pose_collision_rejections;
                break;
            case ReedsSheppRejectionReason::InitialDirectionMismatch:
                ++counters.initial_direction_rejections;
                break;
            case ReedsSheppRejectionReason::SweptCollision:
                ++counters.swept_collision_rejections;
                break;
            case ReedsSheppRejectionReason::GoalEvaluatorRejection:
                ++counters.goal_rejections;
                break;
            case ReedsSheppRejectionReason::DeadlineExpired:
                ++counters.deadline_rejections;
                break;
            case ReedsSheppRejectionReason::PoseCollision:
            case ReedsSheppRejectionReason::InvalidCandidate:
                ++counters.invalid_candidate_rejections;
                break;
        }
    }
    return false;
}

}  // namespace

bool ReedsSheppConnector::connect(
    const Pose& start, const Pose& goal, int initial_direction,
    const PlanningContext& context,
    const std::chrono::steady_clock::time_point& deadline,
    std::vector<Waypoint>& result,
    ReedsSheppImplementation implementation,
    ReedsSheppConnectorDiagnostics* out_diagnostics) const {
    const auto call_started = std::chrono::steady_clock::now();
    ReedsSheppConnectorDiagnostics diagnostics;
    diagnostics.implementation = implementation;
    diagnostics.connector_attempts = 1;
    result.clear();

    const bool connected = implementation == ReedsSheppImplementation::Numerical
        ? connect_numerical(start, goal, initial_direction, context, deadline,
                            result, diagnostics)
        : connect_analytical(*this, start, goal, initial_direction, context,
                             deadline, result, diagnostics);

    diagnostics.solution_accepted = connected;
    diagnostics.accepted_connector_attempts = connected ? 1 : 0;
    diagnostics.planning_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - call_started).count();
    if (out_diagnostics != nullptr) *out_diagnostics = std::move(diagnostics);
    return connected;
}

}  // namespace parking_adas
