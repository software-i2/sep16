// Copyright by BeeX [2026]

#ifndef CLOUD_VOXELGRID_H
#define CLOUD_VOXELGRID_H

#include <Eigen/Core>

#include <array>
#include <cstdint>

namespace cloud {

// A box of voxels fitted to the scene, in the arm base frame. Cell index = x + size_x * (y + size_y * z).
class VoxelGrid {
public:
    VoxelGrid() = default;
    VoxelGrid(const Eigen::Vector3d &low, const Eigen::Vector3d &high, double voxel_size);

    long                cellOf(const Eigen::Vector3d &point) const;  // -1 outside
    long                cellAt(long x, long y, long z) const { return x + size_x_ * (y + size_y_ * z); }
    std::array<long, 3> coordinatesOf(long cell) const;
    Eigen::Vector3d     centreOf(long cell) const;

    double          voxelSize() const { return voxel_size_; }
    long            sizeX() const { return size_x_; }
    long            sizeY() const { return size_y_; }
    long            sizeZ() const { return size_z_; }
    long            sizeOn(int axis) const { return axis == 0 ? size_x_ : (axis == 1 ? size_y_ : size_z_); }
    long            cellCount() const { return size_x_ * size_y_ * size_z_; }
    Eigen::Vector3d origin() const { return origin_; }

private:
    double          voxel_size_ = 1.0;
    long            size_x_     = 0;
    long            size_y_     = 0;
    long            size_z_     = 0;
    Eigen::Vector3d origin_     = Eigen::Vector3d::Zero();
};

}  // namespace cloud

#endif  // CLOUD_VOXELGRID_H
