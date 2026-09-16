// Copyright by BeeX [2026]

#ifndef PLANNER_PLANNERCONFIG_H
#define PLANNER_PLANNERCONFIG_H

#include <kine/ArmModel.h>
#include <params/Params.h>

#include <array>
#include <string>

namespace planner {

// Angles in radians.
struct RrtSettings {
    int    max_iterations    = 0;
    int    refine_iterations = 0;
    double time_budget_s     = 0.0;
    double extend_step       = 0.0;
    double rewire_gamma      = 0.0;
    double edge_check_step   = 0.0;
    int    shortcut_attempts = 0;
    int    random_seed       = 0;
};

// Lengths in metres, angles in radians.
struct PlannerConfig {
    kine::ArmConfig arm;
    std::string     base_frame;
    std::array<std::string, kine::JOINT_COUNT> joint_names;

    double            safety_floor_z         = 0.0;
    double            link_radius            = 0.0;
    double            link_sample_step       = 0.0;
    double            blade_sample_step      = 0.0;
    double            grasp_point_from_mount = 0.0;
    double            max_approach_deviation = 0.0;
    double            start_tolerance        = 0.0;
    double            joint_state_timeout_s  = 0.0;
    kine::JointAngles joint_cost_weights{};
    int               paths_to_compare    = 0;
    double            max_planning_time_s = 0.0;
    RrtSettings       rrt;

    std::string topic_joint_states;
    std::string action_plan;
    std::string topic_plan_result;
};

PlannerConfig loadPlannerConfig(params::Params &planner, params::Params &arm, params::Params &jaws,
                                params::Params &topics);

}  // namespace planner

#endif  // PLANNER_PLANNERCONFIG_H
