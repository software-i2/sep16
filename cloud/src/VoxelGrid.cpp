// Copyright by BeeX [2026]

#include <cloud/VoxelGrid.h>

#include <algorithm>
#include <cmath>

namespace cloud {

VoxelGrid::VoxelGrid(const Eigen::Vector3d &low, const Eigen::Vector3d &high, double voxel_size)
        : voxel_size_(voxel_size), origin_(low) {
    const Eigen::Vector3d span = (high - low) / voxel_size;
    size_x_ = std::max(1L, static_cast<long>(std::ceil(span.x())));
    size_y_ = std::max(1L, static_cast<long>(std::ceil(span.y())));
    size_z_ = std::max(1L, static_cast<long>(std::ceil(span.z())));
}

long VoxelGrid::cellOf(const Eigen::Vector3d &point) const {
    const Eigen::Vector3d local = (point - origin_) / voxel_size_;
    const double          x     = std::floor(local.x());
    const double          y     = std::floor(local.y());
    const double          z     = std::floor(local.z());
    if (x < 0.0 || y < 0.0 || z < 0.0 || x >= size_x_ || y >= size_y_ || z >= size_z_) {
        return -1;
    }
    return cellAt(static_cast<long>(x), static_cast<long>(y), static_cast<long>(z));
}

std::array<long, 3> VoxelGrid::coordinatesOf(long cell) const {
    return {cell % size_x_, (cell / size_x_) % size_y_, cell / (size_x_ * size_y_)};
}

Eigen::Vector3d VoxelGrid::centreOf(long cell) const {
    const std::array<long, 3> c = coordinatesOf(cell);
    return origin_ + voxel_size_ * Eigen::Vector3d(c[0] + 0.5, c[1] + 0.5, c[2] + 0.5);
}

}  // namespace cloud
