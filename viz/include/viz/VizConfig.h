// Copyright by BeeX [2026]

#ifndef VIZ_VIZCONFIG_H
#define VIZ_VIZCONFIG_H

#include <kine/ArmBody.h>
#include <kine/ArmModel.h>
#include <params/Params.h>
#include <std_msgs/ColorRGBA.h>

#include <array>
#include <string>

namespace viz {

// One drawing of a cloud result: its own colours on its own topics, so the two passes can be
// told apart and switched on and off separately.
struct CloudView {
    std_msgs::ColorRGBA grasp_pose;
    std_msgs::ColorRGBA obstacle;
    std_msgs::ColorRGBA handle_cell;
    std::string         topic_grasp_poses;
    std::string         topic_obstacle_map;
};

// Lengths in metres, angles in radians.
struct VizConfig {
    kine::ArmConfig                            arm;
    std::string                                base_frame;
    std::array<std::string, kine::JOINT_COUNT> joint_names;
    double                                     link_radius = 0.0;
    kine::FloorGuard                           floor_guard;

    double arm_body_rate_hz    = 0.0;
    double blade_draw_step     = 0.0;
    double path_draw_step      = 0.0;
    double grasp_pose_size     = 0.0;
    double chosen_grasp_length = 0.0;
    double line_width          = 0.0;

    CloudView initial;    // the survey taken before the vehicle parks
    CloudView reprocess;  // the fresh look taken from the park pose

    std_msgs::ColorRGBA arm_body_colour;
    std_msgs::ColorRGBA chosen_grasp_colour;
    std_msgs::ColorRGBA planned_path_colour;
    std_msgs::ColorRGBA floor_colour;

    std::string topic_joint_states;
    std::string topic_cloud_result;
    std::string topic_plan_result;
    std::string topic_arm_body;
    std::string topic_floor;
    std::string topic_chosen_grasp;
    std::string topic_planned_path;
};

VizConfig loadVizConfig(params::Params &viz, params::Params &arm, params::Params &jaws, params::Params &planner,
                        params::Params &topics);

}  // namespace viz

#endif  // VIZ_VIZCONFIG_H
