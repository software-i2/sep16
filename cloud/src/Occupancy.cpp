// Copyright by BeeX [2026]

#include <cloud/Occupancy.h>

#include <algorithm>
#include <limits>

namespace cloud {
namespace {

// True when the measured surface in the cell's pixel lies clearly behind the cell, so the camera saw through it.
bool seenThrough(const Eigen::Vector3d &centre_in_camera, const CameraModel &camera, const DepthImage &depth,
                 double tolerance) {
    int u = 0;
    int v = 0;
    if (!pixelOf(camera, centre_in_camera, u, v) || !depth.seen(u, v)) {
        return false;
    }
    return -centre_in_camera.z() < depth.at(u, v) - tolerance;
}

}  // namespace

std::vector<uint32_t> occupiedCells(const CameraFrame &frame, const CameraModel &camera, const VoxelGrid &grid,
                                    double crop_radius, const DepthImage &depth, const OutlierFilter *filter,
                                    const std::vector<uint8_t> &handle_region, const OccupancySettings &settings) {
    constexpr uint8_t kMaxCount = std::numeric_limits<uint8_t>::max();

    std::vector<uint8_t> hits(static_cast<size_t>(grid.cellCount()), 0);
    std::vector<long>    touched;
    const double         crop_radius2 = crop_radius * crop_radius;

    for (const Eigen::Vector3f &point : frame.points) {
        const Eigen::Vector3d in_camera = point.cast<double>();
        const Eigen::Vector3d in_base   = frame.camera_to_base * in_camera;
        if (in_base.squaredNorm() > crop_radius2) {
            continue;
        }
        const long cell = grid.cellOf(in_base);
        if (cell < 0) {
            continue;
        }
        if (filter != nullptr && handle_region[cell] == 0) {
            int u = 0;
            int v = 0;
            if (pixelOf(camera, in_camera, u, v) && !filter->supported(u, v)) {
                continue;
            }
        }
        if (hits[cell] == 0) {
            touched.push_back(cell);
        }
        hits[cell] = std::min<int>(hits[cell] + 1, kMaxCount);
    }

    const Eigen::Isometry3d base_to_camera = frame.camera_to_base.inverse();
    std::vector<uint32_t>   occupied;
    for (const long cell : touched) {
        if (hits[cell] >= settings.min_points_per_voxel || handle_region[cell] != 0
            || !seenThrough(base_to_camera * grid.centreOf(cell), camera, depth, settings.free_space_tolerance)) {
            occupied.push_back(static_cast<uint32_t>(cell));
        }
    }
    std::sort(occupied.begin(), occupied.end());
    return occupied;
}

std::vector<uint32_t> voteOccupied(const std::vector<std::vector<uint32_t>> &frames, long cell_count, int min_frames) {
    std::vector<uint8_t> votes(static_cast<size_t>(cell_count), 0);
    for (const std::vector<uint32_t> &frame : frames) {
        for (uint32_t cell : frame) {
            ++votes[cell];
        }
    }
    std::vector<uint32_t> occupied;
    for (long cell = 0; cell < cell_count; ++cell) {
        if (votes[cell] >= min_frames) {
            occupied.push_back(static_cast<uint32_t>(cell));
        }
    }
    return occupied;
}

}  // namespace cloud
