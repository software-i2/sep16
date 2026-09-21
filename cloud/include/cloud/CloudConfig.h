// Copyright by BeeX [2026]

#ifndef CLOUD_CLOUDCONFIG_H
#define CLOUD_CLOUDCONFIG_H

#include <params/Params.h>

#include <array>
#include <string>

namespace cloud {

struct CameraModel {
    int    width  = 0;
    int    height = 0;
    double fx     = 0.0;
    double fy     = 0.0;
    double cx     = 0.0;
    double cy     = 0.0;
};

struct Switches {
    bool outlier_filter      = false;
    bool handle_classifier   = false;
    bool handle_carving      = false;
    bool corridor_carving    = false;
    bool candidate_averaging = false;
    bool obstacle_averaging  = false;
};

struct OutlierFilterSettings {
    double depth_tolerance         = 0.0;
    int    min_agreeing_neighbours = 0;
    int    slope_window_px         = 0;
};

struct OccupancySettings {
    int    min_points_per_voxel   = 0;
    double free_space_tolerance   = 0.0;
};

struct HandleRegionSettings {
    double radius       = 0.0;
    double max_pose_gap = 0.0;
};

struct CorridorSettings {
    double length = 0.0;
    double radius = 0.0;
};

struct CurvatureSettings {
    double                threshold          = 0.0;
    double                curvature_chord    = 0.0;
    double                bend_chord         = 0.0;
    double                smoothing_window   = 0.0;
    double                min_chord_fraction = 0.0;
    double                min_bend_span_px   = 0.0;
    std::array<double, 2> feature_mean{};
    std::array<double, 2> feature_std{};
    std::array<double, 2> weights{};
    double                bias = 0.0;
};

// Lengths in metres, angles in radians.
struct ConsensusSettings {
    int    min_agreeing_frames   = 0;
    double match_distance        = 0.0;
    double max_axis_angle        = 0.0;
    double max_pose_gap          = 0.0;
    double end_tolerance         = 0.0;
    double outlier_distance      = 0.0;
    double max_spread            = 0.0;
    double duplicate_distance    = 0.0;
    double max_drift                 = 0.0;
    double drift_direction_ratio     = 0.0;
    double drift_significance_sigmas = 0.0;
};

// Lengths in metres.
struct CloudConfig {
    std::string base_frame;
    std::string camera_frame;
    CameraModel camera;
    Switches    switches;

    int    frames_to_collect = 0;
    int    frames_to_advance = 0;
    double max_reuse_gap_s   = 0.0;
    double frame_timeout_s   = 0.0;
    double voxel_size        = 0.0;
    double candidate_reach   = 0.0;  // how far the jaw tips can get from the arm base origin
    double crop_radius       = 0.0;  // candidate_reach; obstacles further out cannot touch the arm
    int    approach_column   = 0;
    int    bar_column        = 0;

    OutlierFilterSettings outlier_filter;
    OccupancySettings     occupancy;
    HandleRegionSettings  handle_region;
    CorridorSettings      corridor;

    std::string       classifier_method;
    CurvatureSettings curvature;

    std::string       averaging_method;
    ConsensusSettings consensus;

    int min_frames_occupied = 0;
    int vote_radius_voxels  = 0;

    std::string topic_cloud;
    std::string topic_grasp_poses;
    std::string action_collect;
    std::string topic_result;
};

CloudConfig loadCloudConfig(params::Params &cloud, params::Params &camera, params::Params &arm, params::Params &jaws,
                            params::Params &topics);

}  // namespace cloud

#endif  // CLOUD_CLOUDCONFIG_H
