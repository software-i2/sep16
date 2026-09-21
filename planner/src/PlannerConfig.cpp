// Copyright by BeeX [2026]

#include <kine/ReadConfig.h>
#include <planner/PlannerConfig.h>

namespace planner {

PlannerConfig loadPlannerConfig(params::Params &planner, params::Params &arm, params::Params &jaws,
                                params::Params &topics) {
    PlannerConfig c;
    c.arm        = kine::readArmConfig(arm, jaws);
    c.base_frame = arm.text("base_frame");
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        c.joint_names[j] = arm.text(std::string("joint_names/") + kine::JOINT_KEYS[j]);
    }

    c.safety_floor_z         = planner.number("safety_floor_z_m");
    c.link_radius            = planner.number("link_radius_m");
    c.link_sample_step       = planner.number("link_sample_step_m");
    c.blade_sample_step      = planner.number("blade_sample_step_m");
    c.grasp_point_from_mount = planner.number("grasp_point_from_mount_m");
    c.max_approach_deviation = kine::degToRad(planner.number("max_approach_deviation_deg"));
    c.start_tolerance        = kine::degToRad(planner.number("start_tolerance_deg"));
    c.joint_state_timeout_s  = planner.number("joint_state_timeout_s");
    c.transform_wait_s       = planner.number("transform_wait_s");
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        c.joint_cost_weights[j] = planner.number(std::string("joint_cost_weights/") + kine::JOINT_KEYS[j]);
        planner.require(c.joint_cost_weights[j] > 0.0, std::string("joint_cost_weights/") + kine::JOINT_KEYS[j],
                        "positive");
    }
    c.paths_to_compare    = planner.whole("paths_to_compare");
    c.max_planning_time_s = planner.number("max_planning_time_s");

    c.rrt.max_iterations    = planner.whole("rrt/max_iterations");
    c.rrt.refine_iterations = planner.whole("rrt/refine_iterations");
    c.rrt.time_budget_s     = planner.number("rrt/time_budget_s");
    c.rrt.extend_step       = kine::degToRad(planner.number("rrt/extend_step_deg"));
    c.rrt.rewire_gamma      = planner.number("rrt/rewire_gamma");
    c.rrt.edge_check_step   = kine::degToRad(planner.number("rrt/edge_check_step_deg"));
    c.rrt.shortcut_attempts = planner.whole("rrt/shortcut_attempts");
    c.rrt.random_seed       = planner.whole("rrt/random_seed");

    planner.require(c.link_radius >= 0.0, "link_radius_m", "zero or more");
    planner.require(c.link_sample_step > 0.0, "link_sample_step_m", "positive");
    planner.require(c.blade_sample_step > 0.0, "blade_sample_step_m", "positive");
    planner.require(c.max_approach_deviation >= 0.0, "max_approach_deviation_deg", "zero or more");
    planner.require(c.start_tolerance >= 0.0, "start_tolerance_deg", "zero or more");
    planner.require(c.joint_state_timeout_s > 0.0, "joint_state_timeout_s", "positive");
    planner.require(c.paths_to_compare > 0, "paths_to_compare", "positive");
    planner.require(c.max_planning_time_s > 0.0, "max_planning_time_s", "positive");
    planner.require(c.rrt.max_iterations > 0, "rrt/max_iterations", "positive");
    planner.require(c.rrt.refine_iterations >= 0, "rrt/refine_iterations", "zero or more");
    planner.require(c.rrt.time_budget_s > 0.0, "rrt/time_budget_s", "positive");
    planner.require(c.rrt.extend_step > 0.0, "rrt/extend_step_deg", "positive");
    planner.require(c.rrt.rewire_gamma > 0.0, "rrt/rewire_gamma", "positive");
    planner.require(c.rrt.edge_check_step > 0.0, "rrt/edge_check_step_deg", "positive");
    planner.require(c.rrt.shortcut_attempts >= 0, "rrt/shortcut_attempts", "zero or more");
    planner.require(c.rrt.random_seed >= 0, "rrt/random_seed", "zero or more");

    c.topic_joint_states = topics.text("joint_states");
    c.action_plan        = topics.text("planner_plan");
    c.topic_plan_result  = topics.text("planner_result");
    return c;
}

}  // namespace planner
