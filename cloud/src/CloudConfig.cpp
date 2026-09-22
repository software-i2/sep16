// Copyright by BeeX [2026]

#include <cloud/CloudConfig.h>
#include <kine/ReadConfig.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cloud {
namespace {

void readConsensus(params::Params &cloud, ConsensusSettings &s) {
    const std::string key   = "candidate_averaging/consensus/";
    s.min_agreeing_frames   = cloud.whole(key + "min_agreeing_frames");
    s.match_distance        = cloud.number(key + "match_distance_m");
    s.max_axis_angle        = cloud.number(key + "max_axis_angle_deg") * M_PI / 180.0;
    s.max_pose_gap          = cloud.number(key + "max_pose_gap_m");
    s.end_tolerance         = cloud.number(key + "end_tolerance_m");
    s.outlier_distance      = cloud.number(key + "outlier_distance_m");
    s.max_spread            = cloud.number(key + "max_spread_m");
    s.duplicate_distance    = cloud.number(key + "duplicate_distance_m");
    s.max_drift             = cloud.number(key + "max_drift_m");
    s.drift_direction_ratio     = cloud.number(key + "drift_direction_ratio");
    s.drift_significance_sigmas = cloud.number(key + "drift_significance_sigmas");
    cloud.require(s.min_agreeing_frames > 0, key + "min_agreeing_frames", "positive");
}

}  // namespace

CloudConfig loadCloudConfig(params::Params &cloud, params::Params &camera, params::Params &arm, params::Params &jaws,
                            params::Params &topics) {
    CloudConfig c;
    c.base_frame    = arm.text("base_frame");
    c.camera_frame  = camera.text("frame");
    c.anchor_frame  = cloud.text("anchor_frame");
    c.camera.width  = camera.whole("intrinsics/width_px");
    c.camera.height = camera.whole("intrinsics/height_px");
    c.camera.fx     = camera.number("intrinsics/fx_px");
    c.camera.fy     = camera.number("intrinsics/fy_px");
    c.camera.cx     = camera.number("intrinsics/cx_px");
    c.camera.cy     = camera.number("intrinsics/cy_px");
    camera.require(c.camera.width > 0 && c.camera.height > 0, "intrinsics", "a positive image size");

    c.switches.outlier_filter      = cloud.flag("switches/outlier_filter");
    c.switches.handle_carving      = cloud.flag("switches/handle_carving");
    c.switches.corridor_carving    = cloud.flag("switches/corridor_carving");
    c.switches.candidate_averaging = cloud.flag("switches/candidate_averaging");
    c.switches.obstacle_averaging  = cloud.flag("switches/obstacle_averaging");

    c.frames_to_collect = cloud.whole("frames_to_collect");
    c.frames_to_advance = cloud.whole("frames_to_advance");
    c.fresh_frames      = cloud.whole("fresh_collect/frames_to_collect");
    c.max_reuse_gap_s   = cloud.number("max_reuse_gap_s");
    c.frame_timeout_s   = cloud.number("frame_timeout_s");
    c.transform_wait_s  = cloud.number("transform_wait_s");
    c.voxel_size        = cloud.number("voxel_size_m");
    c.crop_margin       = cloud.number("crop_margin_m");
    c.approach_column   = cloud.whole("pose_axes/approach_column");
    c.bar_column        = cloud.whole("pose_axes/bar_column");

    cloud.require(c.frames_to_collect > 0, "frames_to_collect", "positive");
    cloud.require(c.frames_to_advance > 0 && c.frames_to_advance <= c.frames_to_collect, "frames_to_advance",
                  "between 1 and frames_to_collect");
    cloud.require(c.fresh_frames > 0, "fresh_collect/frames_to_collect", "positive");
    cloud.require(c.max_reuse_gap_s > 0.0, "max_reuse_gap_s", "positive");
    cloud.require(c.frame_timeout_s > 0.0, "frame_timeout_s", "positive");
    cloud.require(c.transform_wait_s >= 0.0, "transform_wait_s", "zero or more");
    cloud.require(c.voxel_size > 0.0, "voxel_size_m", "positive");
    cloud.require(c.crop_margin >= 0.0, "crop_margin_m", "zero or more");
    cloud.require(c.approach_column >= 0 && c.approach_column <= 2, "pose_axes/approach_column", "0, 1 or 2");
    cloud.require(c.bar_column >= 0 && c.bar_column <= 2 && c.bar_column != c.approach_column,
                  "pose_axes/bar_column", "0, 1 or 2 and not the approach column");

    c.outlier_filter.depth_tolerance         = cloud.number("outlier_filter/depth_tolerance_m");
    c.outlier_filter.min_agreeing_neighbours = cloud.whole("outlier_filter/min_agreeing_neighbours");
    c.outlier_filter.slope_window_px         = cloud.whole("outlier_filter/slope_window_px");
    cloud.require(c.outlier_filter.slope_window_px >= 3 && c.outlier_filter.slope_window_px % 2 == 1,
                  "outlier_filter/slope_window_px", "odd and at least 3");

    c.occupancy.min_points_per_voxel = cloud.whole("occupancy/min_points_per_voxel");
    c.occupancy.free_space_tolerance = cloud.number("occupancy/free_space_tolerance_m");
    const double max_tilt            = cloud.number("occupancy/max_surface_tilt_deg");
    cloud.require(max_tilt >= 0.0 && max_tilt < 90.0, "occupancy/max_surface_tilt_deg", "between 0 and 90");
    c.occupancy.max_ray_stretch = 1.0 / std::max(std::cos(max_tilt * M_PI / 180.0), 1e-3);
    c.handle_region.radius           = cloud.number("handle_region/radius_m");
    c.handle_region.max_pose_gap     = cloud.number("handle_region/max_pose_gap_m");
    c.corridor.length                = cloud.number("corridor_carving/length_m");
    c.corridor.radius                = cloud.number("corridor_carving/radius_m");

    // Every method's settings are read, so switching method is only a change of name.
    c.averaging_method = cloud.text("candidate_averaging/method");
    readConsensus(cloud, c.consensus);

    c.min_frames_occupied = cloud.whole("obstacle_averaging/min_frames_occupied");
    cloud.require(c.min_frames_occupied > 0 && c.min_frames_occupied <= c.frames_to_collect,
                  "obstacle_averaging/min_frames_occupied", "between 1 and frames_to_collect");
    c.vote_radius_voxels = cloud.whole("obstacle_averaging/vote_radius_voxels");
    cloud.require(c.vote_radius_voxels >= 0 && c.vote_radius_voxels <= 2, "obstacle_averaging/vote_radius_voxels",
                  "between 0 and 2");
    cloud.require(c.consensus.min_agreeing_frames <= c.frames_to_collect,
                  "candidate_averaging/consensus/min_agreeing_frames", "at most frames_to_collect");

    try {
        const kine::ArmModel model(kine::readArmConfig(arm, jaws));
        c.candidate_reach = model.reachFromBase(model.tipDistance());
        // The snapshot is planned against from wherever the park search sends the arm, so it has to
        // hold what the arm could reach from there, not only from where it stood when the frame was taken.
        c.crop_radius     = c.candidate_reach + c.crop_margin;
    } catch (const std::invalid_argument &e) {
        arm.require(false, "links", std::string("a usable arm (") + e.what() + ")");
    }

    c.topic_cloud       = topics.text("camera_cloud");
    c.topic_grasp_poses = topics.text("camera_grasp_poses");
    c.action_collect    = topics.text("cloud_collect");
    c.topic_result      = topics.text("cloud_result");
    return c;
}

}  // namespace cloud
