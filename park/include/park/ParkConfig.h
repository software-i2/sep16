// Copyright by BeeX [2026]

#ifndef PARK_PARKCONFIG_H
#define PARK_PARKCONFIG_H

#include <kine/ArmBody.h>
#include <kine/ArmModel.h>
#include <kine/Joints.h>
#include <params/Params.h>
#include <park/PoseSearch.h>

#include <array>
#include <string>

namespace park {

// Lengths in metres, angles in radians.
struct ParkConfig {
    kine::ArmConfig arm;
    std::string     base_frame;
    std::string     body_frame;
    std::string     locked_frame;
    std::string     scene_frame;
    std::array<std::string, kine::JOINT_COUNT> joint_names;

    std::array<double, 3> camera_mount_position{{0.0, 0.0, 0.0}};
    std::array<double, 3> camera_mount_rpy{{0.0, 0.0, 0.0}};
    std::array<double, 3> camera_frame_rpy{{0.0, 0.0, 0.0}};
    std::string           camera_anchor_frame;
    std::array<double, 3> arm_mount_position{{0.0, 0.0, 0.0}};
    std::array<double, 3> arm_mount_rpy{{0.0, 0.0, 0.0}};

    double           grasp_point_from_mount = 0.0;
    kine::FloorGuard floor_guard;
    double           blade_sample_step = 0.0;

    double      reach_cell         = 0.0;
    int         reach_roll_samples = 0;
    int         reach_axis_samples = 0;
    std::string reach_cache_path;

    SearchBox search;
    int       verify_count  = 0;
    double    verify_budget = 0.0;
    double    edge_check_step  = 0.0;
    int       transit_samples  = 0;
    int       min_routable     = 0;
    int       verify_stride = 1;
    int       verify_exact  = 0;

    double            link_radius            = 0.0;
    double            link_sample_step       = 0.0;
    double            max_approach_deviation = 0.0;
    kine::JointAngles joint_cost_weights{};
    kine::JointAngles arm_home{};  // reported radians, the pose the arm plans from

    double move_speed_m_s = 0.0;
    double move_yaw_speed = 0.0;
    double move_rate_hz   = 0.0;

    std::string topic_joint_states;
    std::string action_park;
    std::string topic_park_result;
    std::string service_reset;
    std::string topic_locked_cloud;
};

ParkConfig loadParkConfig(params::Params &vehicle, params::Params &arm, params::Params &jaws,
                          params::Params &planner, params::Params &camera, params::Params &topics);

}  // namespace park

#endif  // PARK_PARKCONFIG_H
