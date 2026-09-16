// Copyright by BeeX [2026]

#ifndef CLOUD_OCCUPANCY_H
#define CLOUD_OCCUPANCY_H

#include <cloud/DepthImage.h>
#include <cloud/OutlierFilter.h>
#include <cloud/VoxelGrid.h>

#include <cstdint>
#include <vector>

namespace cloud {

// Sorted occupied cells from the points within crop_radius; a null `filter` keeps every point.
std::vector<uint32_t> occupiedCells(const CameraFrame &frame, const CameraModel &camera, const VoxelGrid &grid,
                                    double crop_radius, const DepthImage &depth, const OutlierFilter *filter,
                                    const std::vector<uint8_t> &handle_region, const OccupancySettings &settings);

// Cells occupied in at least `min_frames` of the frames, sorted.
std::vector<uint32_t> voteOccupied(const std::vector<std::vector<uint32_t>> &frames, long cell_count, int min_frames);

}  // namespace cloud

#endif  // CLOUD_OCCUPANCY_H
