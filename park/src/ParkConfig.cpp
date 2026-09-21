// Copyright by BeeX [2026]

#include <kine/ReadConfig.h>
#include <park/ParkConfig.h>

namespace park {

ParkConfig loadParkConfig(params::Params &vehicle, params::Params &arm, params::Params &jaws,
                          params::Params &planner, params::Params &camera, params::Params &topics) {
    ParkConfig c;
    c.arm          = kine::readArmConfig(arm, jaws);
    c.base_frame   = arm.text("base_frame");
    c.body_frame   = vehicle.text("body_frame");
    c.locked_frame = vehicle.text("locked_frame");
    c.scene_frame  = vehicle.text("scene_frame");
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        c.joint_names[j] = arm.text(std::string("joint_names/") + kine::JOINT_KEYS[j]);
    }

    const std::vector<double> mount_position = vehicle.numbers("arm_mount_position_m", 3);
    const std::vector<double> mount_rpy      = vehicle.numbers("arm_mount_rpy_deg", 3);
    for (int i = 0; i < 3; ++i) {
        c.arm_mount_position[i] = mount_position[i];
        c.arm_mount_rpy[i]      = kine::degToRad(mount_rpy[i]);
    }

    c.grasp_point_from_mount = planner.number("grasp_point_from_mount_m");
    c.floor_guard            = kine::readFloorGuard(planner);
    c.blade_sample_step      = planner.number("blade_sample_step_m");

    c.reach_cell         = vehicle.number("reach_table/cell_m");
    c.reach_roll_samples = vehicle.whole("reach_table/roll_samples");
    c.reach_axis_samples = vehicle.whole("reach_table/axis_samples");
    c.reach_cache_path   = vehicle.text("reach_table/cache_path");

    c.search.box_xy      = vehicle.number("search/box_xy_m");
    c.search.box_z       = vehicle.number("search/box_z_m");
    c.search.box_yaw     = kine::degToRad(vehicle.number("search/box_yaw_deg"));
    c.search.coarse_step = vehicle.number("search/coarse_step_m");
    c.search.coarse_yaw  = kine::degToRad(vehicle.number("search/coarse_step_deg"));
    c.search.fine_step   = vehicle.number("search/fine_step_m");
    c.search.fine_yaw    = kine::degToRad(vehicle.number("search/fine_step_deg"));
    c.search.refine_count = vehicle.whole("search/refine_count");
    c.search.min_grasps   = vehicle.whole("search/min_grasps");
    c.search.standoff_min = vehicle.number("search/standoff_min_m");
    c.search.standoff_max = vehicle.number("search/standoff_max_m");

    const std::vector<double> camera_position  = camera.numbers("mount_position_m", 3);
    const std::vector<double> camera_mount_rpy  = camera.numbers("mount_rpy_deg", 3);
    const std::vector<double> camera_frame_rpy  = camera.numbers("frame_rpy_deg", 3);
    for (int i = 0; i < 3; ++i) {
        c.camera_mount_position[i] = camera_position[i];
        c.camera_mount_rpy[i]      = kine::degToRad(camera_mount_rpy[i]);
        c.camera_frame_rpy[i]      = kine::degToRad(camera_frame_rpy[i]);
    }
    c.camera_anchor_frame = vehicle.text("camera_anchor_frame");
    c.verify_count       = vehicle.whole("search/verify_count");
    c.verify_budget      = vehicle.number("search/verify_budget_s");
    c.edge_check_step    = kine::degToRad(planner.number("rrt/edge_check_step_deg"));
    c.transit_samples    = vehicle.whole("search/transit_samples");
    c.min_routable       = vehicle.whole("search/min_routable");
    c.verify_stride      = vehicle.whole("search/verify_blade_stride");
    c.verify_exact       = vehicle.whole("search/verify_exact_count");

    c.link_radius            = planner.number("link_radius_m");
    c.link_sample_step       = planner.number("link_sample_step_m");
    c.max_approach_deviation = kine::degToRad(planner.number("max_approach_deviation_deg"));
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        c.joint_cost_weights[j] = planner.number(std::string("joint_cost_weights/") + kine::JOINT_KEYS[j]);
        c.arm_home[j]           = kine::degToRad(arm.number(std::string("home_deg/") + kine::JOINT_KEYS[j]));
    }

    c.move_speed_m_s = vehicle.number("move/speed_m_s");
    c.move_rate_hz   = vehicle.number("move/rate_hz");

    c.drift_per_metre     = vehicle.number("drift/per_metre_m");
    c.drift_yaw_per_metre = kine::degToRad(vehicle.number("drift/yaw_per_metre_deg"));

    c.topic_joint_states = topics.text("joint_states");
    c.action_park        = topics.text("park_park");
    c.topic_park_result  = topics.text("park_result");
    c.service_reset      = topics.text("park_reset");
    c.topic_locked_cloud = topics.text("park_locked_cloud");

    vehicle.require(c.reach_cell > 0.0, "reach_table/cell_m", "positive");
    vehicle.require(c.reach_roll_samples >= 0, "reach_table/roll_samples", "zero or more");
    vehicle.require(c.reach_roll_samples == 0 || c.reach_axis_samples >= 2, "reach_table/axis_samples",
                    "at least 2 when roll_samples is above zero");
    vehicle.require(c.search.box_xy > 0.0, "search/box_xy_m", "positive");
    vehicle.require(c.search.box_z >= 0.0, "search/box_z_m", "zero or more");
    vehicle.require(c.search.box_yaw >= 0.0, "search/box_yaw_deg", "zero or more");
    vehicle.require(c.search.coarse_step > 0.0, "search/coarse_step_m", "positive");
    vehicle.require(c.search.coarse_yaw > 0.0, "search/coarse_step_deg", "positive");
    vehicle.require(c.search.fine_step > 0.0 && c.search.fine_step <= c.search.coarse_step, "search/fine_step_m",
                    "positive and no larger than the coarse step");
    vehicle.require(c.search.fine_yaw > 0.0 && c.search.fine_yaw <= c.search.coarse_yaw, "search/fine_step_deg",
                    "positive and no larger than the coarse step");
    vehicle.require(c.search.refine_count > 0, "search/refine_count", "positive");
    vehicle.require(c.search.min_grasps > 0, "search/min_grasps", "positive");
    vehicle.require(c.search.standoff_min >= 0.0 && c.search.standoff_min < c.search.standoff_max,
                    "search/standoff_min_m", "zero or more and below standoff_max_m");
    vehicle.require(c.verify_count > 0, "search/verify_count", "positive");
    vehicle.require(c.verify_budget > 0.0, "search/verify_budget_s", "positive");
    vehicle.require(c.verify_stride > 0, "search/verify_blade_stride", "positive");
    vehicle.require(c.verify_exact > 0, "search/verify_exact_count", "positive");
    vehicle.require(c.transit_samples > 0, "search/transit_samples", "positive");
    vehicle.require(c.min_routable >= 0, "search/min_routable", "zero or more");
    vehicle.require(c.move_speed_m_s > 0.0, "move/speed_m_s", "positive");
    vehicle.require(c.move_rate_hz > 0.0, "move/rate_hz", "positive");
    vehicle.require(c.drift_per_metre >= 0.0, "drift/per_metre_m", "zero or more");
    vehicle.require(c.drift_yaw_per_metre >= 0.0, "drift/yaw_per_metre_deg", "zero or more");
    return c;
}

}  // namespace park
