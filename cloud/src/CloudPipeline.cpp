// Copyright by BeeX [2026]

#include <cloud/CloudPipeline.h>
#include <cloud/DepthImage.h>
#include <cloud/Occupancy.h>
#include <cloud/OutlierFilter.h>
#include <cloud/Regions.h>

#include <algorithm>
#include <cstdio>
#include <limits>

namespace cloud {

CloudPipeline::CloudPipeline(const CloudConfig &config)
        : config_(config), averaging_(makeCandidateAveraging(config)) {}

VoxelGrid CloudPipeline::fitGrid(const std::vector<CameraFrame> &frames) const {
    const double    crop_radius2 = config_.crop_radius * config_.crop_radius;
    Eigen::Vector3d low          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d high         = -low;
    bool            any          = false;

    const auto cover = [&](const Eigen::Vector3d &point) {
        low  = low.cwiseMin(point);
        high = high.cwiseMax(point);
        any  = true;
    };
    for (const CameraFrame &frame : frames) {
        for (const Eigen::Vector3f &point : frame.points) {
            const Eigen::Vector3d in_base = frame.camera_to_base * point.cast<double>();
            if (in_base.squaredNorm() <= crop_radius2) {
                cover(in_base);
            }
        }
        for (const GraspPose &pose : frame.poses) {
            cover(frame.camera_to_base * pose.point);
        }
    }
    if (!any) {
        return VoxelGrid(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(config_.voxel_size), config_.voxel_size);
    }

    // Carving reaches beyond the measured points, so the box has to hold the regions too.
    const double margin = std::max(config_.handle_region.radius, config_.corridor.length + config_.corridor.radius)
                          + config_.voxel_size;
    return VoxelGrid(low - Eigen::Vector3d::Constant(margin), high + Eigen::Vector3d::Constant(margin),
                     config_.voxel_size);
}

CloudPipeline::FrameResult CloudPipeline::processFrame(const CameraFrame &frame, const VoxelGrid &grid,
                                                       bool want_occupancy) const {
    FrameResult result;
    for (const GraspPose &pose : frame.poses) {
        result.poses.push_back(transformPose(frame.camera_to_base, pose));
    }

    for (const GraspPose &pose : result.poses) {
        if (pose.point.norm() <= config_.candidate_reach) {
            result.candidates.push_back(pose);
        }
    }
    if (!want_occupancy) {
        return result;
    }

    // Points near any grasp pose are kept whatever the outlier filter says: they are the handle.
    const std::vector<uint8_t> near_poses = handleRegion(grid, result.poses, config_.handle_region);

    const DepthImage                     depth(config_.camera, frame.points);
    const std::unique_ptr<OutlierFilter> filter(
            config_.switches.outlier_filter ? new OutlierFilter(config_.outlier_filter, depth) : nullptr);

    result.occupied =
            occupiedCells(frame, config_.camera, grid, depth, filter.get(), near_poses, config_.occupancy);
    return result;
}

CloudOutput CloudPipeline::process(const std::vector<CameraFrame> &frames) const {
    CloudOutput output;
    if (frames.empty()) {
        output.summary = "no frames";
        return output;
    }
    output.grid            = fitGrid(frames);
    const VoxelGrid &grid  = output.grid;

    // Without the vote only the newest frame's occupancy is ever read, so the rest are not built.
    std::vector<FrameResult> results;
    for (size_t i = 0; i < frames.size(); ++i) {
        const bool want_occupancy = config_.switches.obstacle_averaging || i + 1 == frames.size();
        results.push_back(processFrame(frames[i], grid, want_occupancy));
    }
    const FrameResult &newest = results.back();
    output.poses              = newest.poses;

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
    std::vector<GraspPose> carve_from = newest.poses;
    if (config_.switches.obstacle_averaging) {
        std::vector<std::vector<uint32_t>> per_frame;
        carve_from.clear();
        for (const FrameResult &result : results) {
            per_frame.push_back(result.occupied);
            carve_from.insert(carve_from.end(), result.poses.begin(), result.poses.end());
        }
        occupied = voteOccupied(per_frame, grid, config_.min_frames_occupied, config_.vote_radius_voxels);
    }
    carve_from.insert(carve_from.end(), output.candidates.begin(), output.candidates.end());

    std::vector<uint8_t> handle(static_cast<size_t>(grid.cellCount()), 0);
    if (config_.switches.handle_carving) {
        handle = handleRegion(grid, carve_from, config_.handle_region);
    }
    if (config_.switches.corridor_carving) {
        const std::vector<uint8_t> corridor = corridorRegion(grid, carve_from, config_.corridor);
        for (size_t cell = 0; cell < handle.size(); ++cell) {
            handle[cell] |= corridor[cell];
        }
    }
    for (uint32_t cell : occupied) {
        (handle[cell] != 0 ? output.handle_cells : output.obstacle_cells).push_back(cell);
    }

    char line[512];
    std::snprintf(line, sizeof(line),
                  "frames used %zu; newest has %zu poses; %zu candidates (%s); %zu obstacle cells, "
                  "%zu handle cells; grid %ldx%ldx%ld of %.1f mm within %.2f m",
                  frames.size(), newest.poses.size(), output.candidates.size(),
                  averaging_summary.c_str(), output.obstacle_cells.size(), output.handle_cells.size(), grid.sizeX(),
                  grid.sizeY(), grid.sizeZ(), config_.voxel_size * 1000.0, config_.crop_radius);
    output.summary = line;
    return output;
}

}  // namespace cloud
