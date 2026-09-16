// Copyright by BeeX [2026]

#include <cloud/Regions.h>

#include <algorithm>
#include <cmath>

namespace cloud {
namespace {

constexpr double kShortestSegment = 1e-9;

// Marks cells whose centre lies within `radius` of the segment a-b.
void markCapsule(const VoxelGrid &grid, const Eigen::Vector3d &a, const Eigen::Vector3d &b, double radius,
                 std::vector<uint8_t> &mask) {
    const Eigen::Vector3d low  = (a.cwiseMin(b) - Eigen::Vector3d::Constant(radius) - grid.origin()) / grid.voxelSize();
    const Eigen::Vector3d high = (a.cwiseMax(b) + Eigen::Vector3d::Constant(radius) - grid.origin()) / grid.voxelSize();
    const auto clampAxis       = [&](double value) {
        return std::min(std::max(static_cast<long>(std::floor(value)), 0L), grid.size() - 1);
    };

    const Eigen::Vector3d segment = b - a;
    const double          length2 = std::max(segment.squaredNorm(), kShortestSegment * kShortestSegment);
    for (long z = clampAxis(low.z()); z <= clampAxis(high.z()); ++z) {
        for (long y = clampAxis(low.y()); y <= clampAxis(high.y()); ++y) {
            for (long x = clampAxis(low.x()); x <= clampAxis(high.x()); ++x) {
                const long            cell   = grid.cellAt(x, y, z);
                const Eigen::Vector3d centre = grid.centreOf(cell);
                const double t = std::min(std::max((centre - a).dot(segment) / length2, 0.0), 1.0);
                if ((centre - (a + t * segment)).norm() <= radius) {
                    mask[cell] = 1;
                }
            }
        }
    }
}

}  // namespace

std::vector<uint8_t> handleRegion(const VoxelGrid &grid, const std::vector<GraspPose> &poses,
                                  const HandleRegionSettings &settings) {
    std::vector<uint8_t> mask(static_cast<size_t>(grid.cellCount()), 0);

    // The cell holding each pose plus every cell whose offset from it is within the radius.
    const int reach = static_cast<int>(std::ceil(settings.radius / grid.voxelSize()));
    for (const GraspPose &pose : poses) {
        const long cell = grid.cellOf(pose.point);
        if (cell < 0) {
            continue;
        }
        const std::array<long, 3> c = grid.coordinatesOf(cell);
        for (int dz = -reach; dz <= reach; ++dz) {
            for (int dy = -reach; dy <= reach; ++dy) {
                for (int dx = -reach; dx <= reach; ++dx) {
                    const long x = c[0] + dx;
                    const long y = c[1] + dy;
                    const long z = c[2] + dz;
                    const double offset2 = (dx * dx + dy * dy + dz * dz) * grid.voxelSize() * grid.voxelSize();
                    if (offset2 <= settings.radius * settings.radius && x >= 0 && y >= 0 && z >= 0
                        && x < grid.size() && y < grid.size() && z < grid.size()) {
                        mask[grid.cellAt(x, y, z)] = 1;
                    }
                }
            }
        }
    }

    for (size_t i = 1; i < poses.size(); ++i) {
        const double gap = (poses[i].point - poses[i - 1].point).norm();
        if (gap > kShortestSegment && gap <= settings.max_pose_gap) {
            markCapsule(grid, poses[i - 1].point, poses[i].point, settings.radius, mask);
        }
    }
    return mask;
}

std::vector<uint8_t> corridorRegion(const VoxelGrid &grid, const std::vector<GraspPose> &poses,
                                    const CorridorSettings &settings) {
    std::vector<uint8_t> mask(static_cast<size_t>(grid.cellCount()), 0);
    for (const GraspPose &pose : poses) {
        const double length = pose.approach.norm();
        if (length > kShortestSegment) {
            markCapsule(grid, pose.point, pose.point - pose.approach * (settings.length / length), settings.radius, mask);
        }
    }
    return mask;
}

}  // namespace cloud
