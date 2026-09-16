// Copyright by BeeX [2026]

#include <cloud/CloudPipeline.h>
#include <cloud/DepthImage.h>
#include <cloud/Occupancy.h>
#include <cloud/OutlierFilter.h>
#include <cloud/Regions.h>

#include <cstdio>

namespace cloud {

CloudPipeline::CloudPipeline(const CloudConfig &config)
        : config_(config),
          grid_(config.crop_radius, config.voxel_size),
          classifier_(makeHandleClassifier(config)),
          averaging_(makeCandidateAveraging(config)) {}

CloudPipeline::FrameResult CloudPipeline::processFrame(const CameraFrame &frame) const {
    FrameResult result;
    for (const GraspPose &pose : frame.poses) {
        result.poses.push_back(transformPose(frame.camera_to_base, pose));
    }

    result.is_handle = config_.switches.handle_classifier ? classifier_->isHandle(frame.poses)
                                                          : std::vector<bool>(frame.poses.size(), true);
    for (size_t i = 0; i < result.poses.size(); ++i) {
        if (!result.is_handle[i]) {
            continue;
        }
        result.handle_poses.push_back(result.poses[i]);
        if (result.poses[i].point.norm() <= config_.candidate_reach) {
            result.candidates.push_back(result.poses[i]);
        }
    }

    // Points near any grasp pose are kept whatever the outlier filter says: they are the handle.
    const std::vector<uint8_t> near_poses = handleRegion(grid_, result.poses, config_.handle_region);

    const DepthImage                     depth(config_.camera, frame.points);
    const std::unique_ptr<OutlierFilter> filter(
            config_.switches.outlier_filter ? new OutlierFilter(config_.outlier_filter, depth) : nullptr);

    result.occupied = occupiedCells(frame, config_.camera, grid_, config_.crop_radius, depth, filter.get(),
                                    near_poses, config_.occupancy);
    return result;
}

CloudOutput CloudPipeline::process(const std::vector<CameraFrame> &frames) const {
    CloudOutput output;
    if (frames.empty()) {
        output.summary = "no frames";
        return output;
    }

    std::vector<FrameResult> results;
    for (const CameraFrame &frame : frames) {
        results.push_back(processFrame(frame));
    }
    const FrameResult &newest = results.back();

    for (size_t i = 0; i < newest.poses.size(); ++i) {
        (newest.is_handle[i] ? output.handle_poses : output.rope_poses).push_back(newest.poses[i]);
    }

    std::string averaging_summary = "newest frame only";
    if (config_.switches.candidate_averaging) {
        std::vector<std::vector<GraspPose>> per_frame;
        for (const FrameResult &result : results) {
            per_frame.push_back(result.candidates);
        }
        AveragedCandidates averaged = averaging_->average(per_frame);
        output.candidates           = averaged.candidates;
        averaging_summary           = averaged.summary;
    } else {
        output.candidates = newest.candidates;
    }

    std::vector<uint32_t>  occupied   = newest.occupied;
    std::vector<GraspPose> carve_from = newest.handle_poses;
    if (config_.switches.obstacle_averaging) {
        std::vector<std::vector<uint32_t>> per_frame;
        carve_from.clear();
        for (const FrameResult &result : results) {
            per_frame.push_back(result.occupied);
            carve_from.insert(carve_from.end(), result.handle_poses.begin(), result.handle_poses.end());
        }
        occupied = voteOccupied(per_frame, grid_.cellCount(), config_.min_frames_occupied);
    }
    carve_from.insert(carve_from.end(), output.candidates.begin(), output.candidates.end());

    std::vector<uint8_t> handle(static_cast<size_t>(grid_.cellCount()), 0);
    if (config_.switches.handle_carving) {
        handle = handleRegion(grid_, carve_from, config_.handle_region);
    }
    if (config_.switches.corridor_carving) {
        const std::vector<uint8_t> corridor = corridorRegion(grid_, carve_from, config_.corridor);
        for (size_t cell = 0; cell < handle.size(); ++cell) {
            handle[cell] |= corridor[cell];
        }
    }
    for (uint32_t cell : occupied) {
        (handle[cell] != 0 ? output.handle_cells : output.obstacle_cells).push_back(cell);
    }

    char line[512];
    std::snprintf(line, sizeof(line),
                  "frames used %zu; newest has %zu poses, %zu handle; %zu candidates (%s); %zu obstacle cells, "
                  "%zu handle cells",
                  frames.size(), newest.poses.size(), output.handle_poses.size(), output.candidates.size(),
                  averaging_summary.c_str(), output.obstacle_cells.size(), output.handle_cells.size());
    output.summary = line;
    return output;
}

}  // namespace cloud
