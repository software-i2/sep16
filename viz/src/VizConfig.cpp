// Copyright by BeeX [2026]

#include <kine/ReadConfig.h>
#include <viz/VizConfig.h>

namespace viz {
namespace {

std_msgs::ColorRGBA readColour(params::Params &viz, const std::string &name) {
    const std::vector<double> rgba = viz.numbers("colours/" + name, 4);
    std_msgs::ColorRGBA       colour;
    colour.r = static_cast<float>(rgba[0]);
    colour.g = static_cast<float>(rgba[1]);
    colour.b = static_cast<float>(rgba[2]);
    colour.a = static_cast<float>(rgba[3]);
    return colour;
}

CloudView readView(params::Params &viz, params::Params &topics, const std::string &name,
                   const std::string &poses_key, const std::string &map_key) {
    CloudView view;
    view.grasp_pose         = readColour(viz, name + "/grasp_pose");
    view.obstacle           = readColour(viz, name + "/obstacle");
    view.handle_cell        = readColour(viz, name + "/handle_cell");
    view.topic_grasp_poses  = topics.text(poses_key);
    view.topic_obstacle_map = topics.text(map_key);
    return view;
}

}  // namespace

VizConfig loadVizConfig(params::Params &viz, params::Params &arm, params::Params &jaws, params::Params &planner,
                        params::Params &topics) {
    VizConfig c;
    c.arm        = kine::readArmConfig(arm, jaws);
    c.base_frame = arm.text("base_frame");
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        c.joint_names[j] = arm.text(std::string("joint_names/") + kine::JOINT_KEYS[j]);
    }
    c.link_radius = planner.number("link_radius_m");
    c.floor_guard = kine::readFloorGuard(planner);

    c.arm_body_rate_hz    = viz.number("arm_body_rate_hz");
    c.blade_draw_step     = viz.number("blade_draw_step_m");
    c.path_draw_step      = kine::degToRad(viz.number("path_draw_step_deg"));
    c.grasp_pose_size     = viz.number("grasp_pose_size_m");
    c.chosen_grasp_length = viz.number("chosen_grasp_length_m");
    c.line_width          = viz.number("line_width_m");
    viz.require(c.arm_body_rate_hz > 0.0, "arm_body_rate_hz", "positive");
    viz.require(c.blade_draw_step > 0.0, "blade_draw_step_m", "positive");
    viz.require(c.path_draw_step > 0.0, "path_draw_step_deg", "positive");

    c.initial   = readView(viz, topics, "initial", "viz_grasp_poses", "viz_obstacle_map");
    c.reprocess = readView(viz, topics, "reprocess", "viz_reprocess_grasp_poses", "viz_reprocess_obstacle_map");

    c.arm_body_colour     = readColour(viz, "arm_body");
    c.chosen_grasp_colour = readColour(viz, "chosen_grasp");
    c.planned_path_colour = readColour(viz, "planned_path");
    c.floor_colour        = readColour(viz, "floor");

    c.topic_joint_states = topics.text("joint_states");
    c.topic_cloud_result = topics.text("cloud_result");
    c.topic_plan_result  = topics.text("planner_result");
    c.topic_arm_body     = topics.text("viz_arm_body");
    c.topic_floor        = topics.text("viz_floor");
    c.topic_chosen_grasp = topics.text("viz_chosen_grasp");
    c.topic_planned_path = topics.text("viz_planned_path");
    return c;
}

}  // namespace viz
