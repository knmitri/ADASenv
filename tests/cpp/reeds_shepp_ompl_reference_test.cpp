#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include "parking_adas/planning/reeds_shepp_connector.hpp"

#ifndef OMPL_REFERENCE_VERSION
#define OMPL_REFERENCE_VERSION "unknown"
#endif

#ifndef OMPL_TEST_CONFIGURATION
#define OMPL_TEST_CONFIGURATION "unknown"
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;

struct TestPose {
    double x;
    double y;
    double yaw;
};

struct TestSegment {
    parking_adas::ReedsSheppSegmentType type;
    double length;
};

void set_pose(ompl::base::State* state, const TestPose& pose) {
    auto* se2 = state->as<ompl::base::SE2StateSpace::StateType>();
    se2->setXY(pose.x, pose.y);
    se2->setYaw(pose.yaw);
}

TestPose integrate(const std::vector<TestSegment>& segments, double radius) {
    TestPose pose{0.0, 0.0, 0.0};
    for (const TestSegment& segment : segments) {
        double curvature = 0.0;
        if (segment.type == parking_adas::ReedsSheppSegmentType::Left) {
            curvature = 1.0 / radius;
        } else if (segment.type == parking_adas::ReedsSheppSegmentType::Right) {
            curvature = -1.0 / radius;
        }
        const double next_yaw = pose.yaw + curvature * segment.length;
        if (curvature == 0.0) {
            pose.x += segment.length * std::cos(pose.yaw);
            pose.y += segment.length * std::sin(pose.yaw);
        } else {
            pose.x += (std::sin(next_yaw) - std::sin(pose.yaw)) / curvature;
            pose.y += (-std::cos(next_yaw) + std::cos(pose.yaw)) / curvature;
        }
        pose.yaw = next_yaw;
    }
    return pose;
}

double percentile(const std::vector<double>& sorted_values, double fraction) {
    const std::size_t index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(sorted_values.size()))) - 1;
    return sorted_values.at(index);
}

}  // namespace

int main() {
    std::cout << "OMPL version=" << OMPL_REFERENCE_VERSION
              << ", configuration=" << OMPL_TEST_CONFIGURATION;
#if defined(_MSC_VER)
    std::cout << ", compiler=MSVC " << _MSC_VER;
#elif defined(__clang__)
    std::cout << ", compiler=Clang " << __clang_version__;
#elif defined(__GNUC__)
    std::cout << ", compiler=GCC " << __VERSION__;
#endif
    std::cout << '\n';
    constexpr double turning_radius = 2.82;
    const std::array<double, 7> coordinates{
        -6.0, -3.0, -1.0, 0.0, 1.0, 3.0, 6.0,
    };
    const std::array<double, 7> headings{
        -kPi, -2.0, -0.5, 0.0, 0.5, 2.0, kPi,
    };

    std::vector<std::pair<std::string, TestPose>> cases;
    for (double x : coordinates) {
        for (double y : coordinates) {
            for (double yaw : headings) {
                cases.push_back({"grid", {x, y, yaw}});
            }
        }
    }

    cases.push_back({"forward-only", integrate({
        {parking_adas::ReedsSheppSegmentType::Straight, 7.0}}, turning_radius)});
    cases.push_back({"reverse-only", integrate({
        {parking_adas::ReedsSheppSegmentType::Straight, -7.0}}, turning_radius)});
    cases.push_back({"mixed-direction", integrate({
        {parking_adas::ReedsSheppSegmentType::Left, 1.1 * turning_radius},
        {parking_adas::ReedsSheppSegmentType::Straight, -2.5 * turning_radius},
        {parking_adas::ReedsSheppSegmentType::Right, -0.9 * turning_radius},
        {parking_adas::ReedsSheppSegmentType::Straight, 4.0 * turning_radius}},
        turning_radius)});
    cases.push_back({"multiple-cusps", integrate({
        {parking_adas::ReedsSheppSegmentType::Left, 0.8 * turning_radius},
        {parking_adas::ReedsSheppSegmentType::Right, -1.2 * turning_radius},
        {parking_adas::ReedsSheppSegmentType::Straight, -1.5 * turning_radius},
        {parking_adas::ReedsSheppSegmentType::Left, 0.7 * turning_radius},
        {parking_adas::ReedsSheppSegmentType::Right, -0.6 * turning_radius}},
        turning_radius)});
    cases.push_back({"close", {1e-9, -1e-9, 1e-8}});
    cases.push_back({"large-separation", {1000.0, -750.0, 2.4}});
    cases.push_back({"coincident-opposite-heading", {0.0, 0.0, kPi}});
    cases.push_back({"difficult-heading", {-0.01, 0.02, -kPi + 1e-10}});

    constexpr std::uint32_t random_seed = 0x5EED1234U;
    std::mt19937 generator(random_seed);
    std::uniform_real_distribution<double> position(-100.0, 100.0);
    std::uniform_real_distribution<double> heading(-kPi, kPi);
    constexpr std::size_t random_case_count = 1000;
    for (std::size_t index = 0; index < random_case_count; ++index) {
        cases.push_back({"random", {position(generator), position(generator),
                                    heading(generator)}});
    }

    parking_adas::ReedsSheppConnector connector;
    ompl::base::ReedsSheppStateSpace reference(turning_radius);
    ompl::base::State* from = reference.allocState();
    ompl::base::State* to = reference.allocState();
    set_pose(from, {0.0, 0.0, 0.0});

    std::vector<double> errors;
    errors.reserve(cases.size());
    for (const auto& test_case : cases) {
        const std::string& category = test_case.first;
        const TestPose& goal = test_case.second;
        set_pose(to, goal);
        const auto candidates = connector.analytical_candidates(
            {0.0, 0.0, 0.0}, {goal.x, goal.y, goal.yaw}, turning_radius);
        if (candidates.empty()) {
            reference.freeState(to);
            reference.freeState(from);
            std::cerr << "Analytical solver returned no path for " << category << '\n';
            return 1;
        }

        const double expected = reference.distance(from, to);
        const double actual = candidates.front().geometric_length;
        const double error = std::abs(actual - expected);
        const double tolerance = 2e-8 * std::max(1.0, expected);
        if (error > tolerance) {
            std::cerr << std::setprecision(17)
                      << "OMPL distance mismatch for " << category << " pose ("
                      << goal.x << ", " << goal.y << ", " << goal.yaw
                      << "): analytical=" << actual << ", OMPL=" << expected
                      << ", absolute_error=" << error << '\n';
            std::cerr << "candidate segments:";
            for (const parking_adas::ReedsSheppSegment& segment :
                 candidates.front().segments) {
                std::cerr << ' '
                          << static_cast<int>(segment.type) << ':'
                          << segment.signed_length;
            }
            std::cerr << '\n';
            reference.freeState(to);
            reference.freeState(from);
            return 1;
        }
        errors.push_back(error);
    }

    reference.freeState(to);
    reference.freeState(from);
    std::sort(errors.begin(), errors.end());
    double error_sum = 0.0;
    for (double error : errors) error_sum += error;
    std::cout << std::setprecision(12)
              << "OMPL shortest-distance comparison passed: cases=" << errors.size()
              << ", grid=343, random_seed=" << random_seed
              << ", random_cases=" << random_case_count
              << ", max_abs_error=" << errors.back()
              << ", mean_abs_error=" << error_sum / static_cast<double>(errors.size())
              << ", p50_abs_error=" << percentile(errors, 0.50)
              << ", p95_abs_error=" << percentile(errors, 0.95)
              << ", p99_abs_error=" << percentile(errors, 0.99) << '\n';
    return 0;
}
