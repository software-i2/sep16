// Copyright by BeeX [2026]

#include <cloud/VoxelGrid.h>

#include <cmath>

namespace cloud {

VoxelGrid::VoxelGrid(double half_extent, double voxel_size)
        : voxel_size_(voxel_size),
          size_(static_cast<long>(std::ceil(2.0 * half_extent / voxel_size))),
          origin_(Eigen::Vector3d::Constant(-0.5 * static_cast<double>(size_) * voxel_size)) {}

long VoxelGrid::cellOf(const Eigen::Vector3d &point) const {
    const Eigen::Vector3d local = (point - origin_) / voxel_size_;
    const double          x     = std::floor(local.x());
    const double          y     = std::floor(local.y());
    const double          z     = std::floor(local.z());
    if (x < 0.0 || y < 0.0 || z < 0.0 || x >= size_ || y >= size_ || z >= size_) {
        return -1;
    }
    return cellAt(static_cast<long>(x), static_cast<long>(y), static_cast<long>(z));
}

std::array<long, 3> VoxelGrid::coordinatesOf(long cell) const {
    return {cell % size_, (cell / size_) % size_, cell / (size_ * size_)};
}

Eigen::Vector3d VoxelGrid::centreOf(long cell) const {
    const std::array<long, 3> c = coordinatesOf(cell);
    return origin_ + voxel_size_ * Eigen::Vector3d(c[0] + 0.5, c[1] + 0.5, c[2] + 0.5);
}

}  // namespace cloud
