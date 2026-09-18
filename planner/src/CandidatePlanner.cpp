// Copyright by BeeX [2026]

#include <planner/BiRrtStar.h>
#include <planner/CandidatePlanner.h>
#include <planner/ObstacleGrid.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>

namespace planner {
namespace {

using Clock = std::chrono::steady_clock;

// Covers the gap between blade samples and between a voxel centre and its corners.
double bladeRadius(double voxel_size, double blade_sample_step) {
    return (voxel_size + blade_sample_step) * std::sqrt(3.0) / 2.0;
}

std::string tally(const std::vector<Outcome> &outcomes) {
    std::map<std::string, int> counts;
    for (const Outcome outcome : outcomes) {
        ++counts[outcomeName(outcome)];
    }
    std::string text;
    for (const auto &count : counts) {
        text += (text.empty() ? "" : ", ") + std::to_string(count.second) + " " + count.first;
    }
    return text.empty() ? "no candidates" : text;
}

}  // namespace

CandidatePlanner::CandidatePlanner(const PlannerConfig &config)
        : config_(config), body_(kine::ArmModel(config.arm), config.arm.jaw, config.blade_sample_step) {}

bool CandidatePlanner::snapIntoLimits(const kine::JointAngles &reading, kine::JointAngles &out) const {
    const kine::ArmModel &model = body_.model();
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        if (reading[j] < model.lowerLimit(j) - config_.start_tolerance
            || reading[j] > model.upperLimit(j) + config_.start_tolerance) {
            return false;
        }
        out[j] = std::min(std::max(reading[j], model.lowerLimit(j)), model.upperLimit(j));
    }
    return true;
}

PlanOutcome CandidatePlanner::plan(const std::vector<Candidate> &candidates, const msgs::ObstacleMap &map,
                                   const kine::JointAngles &start_reading,
                                   const std::function<bool()> &cancelled) const {
    const Clock::time_point started = Clock::now();
    PlanOutcome             result;
    result.outcomes.assign(candidates.size(), Outcome::UNREACHABLE);

    kine::JointAngles start;
    if (!snapIntoLimits(start_reading, start)) {
        result.summary = "the arm is outside its joint limits, so nothing can be planned from here";
        return result;
    }

    std::unique_ptr<ObstacleGrid> grid;
    try {
        grid.reset(new ObstacleGrid(map, config_.link_radius, bladeRadius(map.voxel_size_m, config_.blade_sample_step)));
    } catch (const std::invalid_argument &e) {
        result.summary = e.what();
        return result;
    }
    CollisionChecker checker(body_, *grid, config_.safety_floor_z, config_.link_sample_step);

    const GraspSettings grasp{config_.grasp_point_from_mount, config_.max_approach_deviation,
                              config_.joint_cost_weights};
    const auto remaining = [&]() {
        return config_.max_planning_time_s - std::chrono::duration<double>(Clock::now() - started).count();
    };

    std::vector<GraspGoal> goals;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (cancelled()) {
            result.success = false;
            result.summary = "cancelled";
            return result;
        }
        result.outcomes[i] = findGraspGoals(candidates[i], i, start, grasp, checker, goals);
    }
    std::sort(goals.begin(), goals.end(),
              [](const GraspGoal &a, const GraspGoal &b) { return a.straight_cost < b.straight_cost; });

    BiRrtStar rrt(checker, config_.rrt, config_.joint_cost_weights);
    int       paths_found = 0;
    for (const GraspGoal &goal : goals) {
        if (paths_found >= config_.paths_to_compare || remaining() < config_.rrt.time_budget_s) {
            break;
        }
        if (cancelled()) {
            result.success = false;
            result.summary = "cancelled";
            return result;
        }

        ++result.goals_tried;
        JointPath corners;
        if (!rrt.plan(start, goal.joints, corners, remaining())) {
            if (result.outcomes[goal.candidate] != Outcome::OK) {
                result.outcomes[goal.candidate] = Outcome::NO_PATH;
            }
            continue;
        }

        ++paths_found;
        result.outcomes[goal.candidate] = Outcome::OK;
        const double cost               = pathTravel(config_.joint_cost_weights, corners);
        if (!result.success || cost < result.cost) {
            result.success      = true;
            result.chosen_index = goal.candidate;
            result.path         = corners;
            result.cost         = cost;
        }
    }

    const double seconds = std::chrono::duration<double>(Clock::now() - started).count();
    char         line[256];
    if (result.success) {
        std::snprintf(line, sizeof(line), "chose candidate %zu of %zu, cost %.3f rad over %zu waypoints (%.2f s). ",
                      result.chosen_index, candidates.size(), result.cost, result.path.size(), seconds);
    } else {
        std::snprintf(line, sizeof(line), "no path to any of %zu candidates (%.2f s). ", candidates.size(), seconds);
    }
    result.summary = line + tally(result.outcomes);
    return result;
}

}  // namespace planner
