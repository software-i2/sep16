// Copyright by BeeX [2026]

#ifndef CLOUD_OCCUPANCY_H
#define CLOUD_OCCUPANCY_H

#include <cloud/DepthImage.h>
#include <cloud/OutlierFilter.h>
#include <cloud/VoxelGrid.h>

#include <cstdint>
#include <vector>

namespace cloud {

// Sorted occupied cells from the points that fall inside the grid; a null `filter` keeps every point.
std::vector<uint32_t> occupiedCells(const CameraFrame &frame, const CameraModel &camera, const VoxelGrid &grid,
                                    const DepthImage &depth, const OutlierFilter *filter,
                                    const std::vector<uint8_t> &handle_region, const OccupancySettings &settings);

// Cells occupied in at least `min_frames` of the frames, sorted. A frame votes for a cell when it saw any cell
// within `vote_radius` voxels of it occupied, so drifting surfaces are not thinned; only measured cells are returned.
std::vector<uint32_t> voteOccupied(const std::vector<std::vector<uint32_t>> &frames, const VoxelGrid &grid,
                                   int min_frames, int vote_radius);

}  // namespace cloud

#endif  // CLOUD_OCCUPANCY_H
