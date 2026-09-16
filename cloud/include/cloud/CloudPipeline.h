// Copyright by BeeX [2026]

#ifndef CLOUD_CLOUDPIPELINE_H
#define CLOUD_CLOUDPIPELINE_H

#include <cloud/CandidateAveraging.h>
#include <cloud/HandleClassifier.h>
#include <cloud/VoxelGrid.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cloud {

// Everything in the arm base frame.
struct CloudOutput {
    std::vector<GraspPose> handle_poses;  // newest frame
    std::vector<GraspPose> rope_poses;    // newest frame
    std::vector<GraspPose> candidates;
    std::vector<uint32_t>  obstacle_cells;
    std::vector<uint32_t>  handle_cells;
    std::string            summary;
};

// Collected frames in, candidates and obstacle map out. Each stage follows its switch in the config.
class CloudPipeline {
public:
    // Throws std::invalid_argument when a configured method does not exist.
    explicit CloudPipeline(const CloudConfig &config);

    // Frames oldest first.
    CloudOutput process(const std::vector<CameraFrame> &frames) const;

    const VoxelGrid &grid() const { return grid_; }

private:
    struct FrameResult {
        std::vector<GraspPose> poses;       // arm base frame
        std::vector<bool>      is_handle;
        std::vector<GraspPose> handle_poses;
        std::vector<GraspPose> candidates;  // handle poses within reach
        std::vector<uint32_t>  occupied;
    };

    FrameResult processFrame(const CameraFrame &frame) const;

    CloudConfig                         config_;
    VoxelGrid                           grid_;
    std::unique_ptr<HandleClassifier>   classifier_;
    std::unique_ptr<CandidateAveraging> averaging_;
};

}  // namespace cloud

#endif  // CLOUD_CLOUDPIPELINE_H
