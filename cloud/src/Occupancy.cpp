// Copyright by BeeX [2026]

#include <cloud/Occupancy.h>

#include <algorithm>
#include <array>
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

std::vector<uint32_t> voteOccupied(const std::vector<std::vector<uint32_t>> &frames, const VoxelGrid &grid,
                                   int min_frames, int vote_radius) {
    const size_t         cell_count = static_cast<size_t>(grid.cellCount());
    std::vector<uint8_t> votes(cell_count, 0);
    std::vector<uint8_t> counted(cell_count, 0);

    std::vector<uint32_t> measured;
    for (size_t k = 0; k < frames.size(); ++k) {
        const uint8_t mark = static_cast<uint8_t>(k + 1);
        for (uint32_t cell : frames[k]) {
            measured.push_back(cell);
            const std::array<long, 3> centre = grid.coordinatesOf(static_cast<long>(cell));
            for (int dz = -vote_radius; dz <= vote_radius; ++dz) {
                for (int dy = -vote_radius; dy <= vote_radius; ++dy) {
                    for (int dx = -vote_radius; dx <= vote_radius; ++dx) {
                        const long x = centre[0] + dx;
                        const long y = centre[1] + dy;
                        const long z = centre[2] + dz;
                        if (x < 0 || y < 0 || z < 0 || x >= grid.size() || y >= grid.size() || z >= grid.size()) {
                            continue;
                        }
                        const long neighbour = grid.cellAt(x, y, z);
                        if (counted[neighbour] != mark) {
                            counted[neighbour] = mark;
                            ++votes[neighbour];
                        }
                    }
                }
            }
        }
    }

    std::sort(measured.begin(), measured.end());
    measured.erase(std::unique(measured.begin(), measured.end()), measured.end());

    std::vector<uint32_t> occupied;
    for (uint32_t cell : measured) {
        if (votes[cell] >= min_frames) {
            occupied.push_back(cell);
        }
    }
    return occupied;
}

}  // namespace cloud
