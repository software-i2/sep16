// Copyright by BeeX [2026]

#ifndef CLOUD_REGIONS_H
#define CLOUD_REGIONS_H

#include <cloud/Frame.h>
#include <cloud/VoxelGrid.h>

#include <cstdint>
#include <vector>

namespace cloud {

// Cells around the handle: within radius of each pose, and of the bar between consecutive poses closer than max_pose_gap.
std::vector<uint8_t> handleRegion(const VoxelGrid &grid, const std::vector<GraspPose> &poses,
                                  const HandleRegionSettings &settings);

// Cells within radius of the corridor running back from each pose along its approach.
std::vector<uint8_t> corridorRegion(const VoxelGrid &grid, const std::vector<GraspPose> &poses,
                                    const CorridorSettings &settings);

}  // namespace cloud

#endif  // CLOUD_REGIONS_H
