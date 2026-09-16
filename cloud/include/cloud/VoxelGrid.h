// Copyright by BeeX [2026]

#ifndef CLOUD_VOXELGRID_H
#define CLOUD_VOXELGRID_H

#include <Eigen/Core>

#include <array>
#include <cstdint>

namespace cloud {

// A cube of voxels centred on the arm base origin. Cell index = x + size * (y + size * z).
class VoxelGrid {
public:
    VoxelGrid(double half_extent, double voxel_size);

    long              cellOf(const Eigen::Vector3d &point) const;  // -1 outside
    long              cellAt(long x, long y, long z) const { return x + size_ * (y + size_ * z); }
    std::array<long, 3> coordinatesOf(long cell) const;
    Eigen::Vector3d   centreOf(long cell) const;

    double          voxelSize() const { return voxel_size_; }
    long            size() const { return size_; }
    long            cellCount() const { return size_ * size_ * size_; }
    Eigen::Vector3d origin() const { return origin_; }

private:
    double          voxel_size_;
    long            size_;
    Eigen::Vector3d origin_;
};

}  // namespace cloud

#endif  // CLOUD_VOXELGRID_H
