// Copyright by BeeX [2026]

#ifndef CLOUD_CLOUDPIPELINE_H
#define CLOUD_CLOUDPIPELINE_H

#include <cloud/CandidateAveraging.h>
#include <cloud/VoxelGrid.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cloud {

// Everything in the arm base frame.
struct CloudOutput {
    std::vector<GraspPose> poses;  // newest frame
    std::vector<GraspPose> candidates;
    std::vector<uint32_t>  obstacle_cells;
    std::vector<uint32_t>  handle_cells;
    VoxelGrid              grid;  // the box the cells are indexed in, fitted to this scene
    std::string            summary;
};

// Collected frames in, candidates and obstacle map out. Each stage follows its switch in the config.
class CloudPipeline {
public:
    // Throws std::invalid_argument when a configured method does not exist.
    explicit CloudPipeline(const CloudConfig &config);

    // Frames oldest first.
    CloudOutput process(const std::vector<CameraFrame> &frames) const;

private:
    struct FrameResult {
        std::vector<GraspPose> poses;       // arm base frame
        std::vector<GraspPose> candidates;  // poses within reach
        std::vector<uint32_t>  occupied;
    };

    // The grid is fitted to the collected scene, so it only exists once the frames are in hand.
    VoxelGrid   fitGrid(const std::vector<CameraFrame> &frames) const;
    FrameResult processFrame(const CameraFrame &frame, const VoxelGrid &grid, bool want_occupancy) const;

    CloudConfig                         config_;
    std::unique_ptr<CandidateAveraging> averaging_;
};

}  // namespace cloud

#endif  // CLOUD_CLOUDPIPELINE_H
