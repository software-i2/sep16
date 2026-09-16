// Copyright by BeeX [2026]

#ifndef PLANNER_OBSTACLEGRID_H
#define PLANNER_OBSTACLEGRID_H

#include <msgs/ObstacleMap.h>

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace planner {

// The obstacle map inflated for lookups. Anything outside the map counts as free.
class ObstacleGrid {
public:
    // Links keep link_radius from every cell, blades blade_radius from obstacle cells. Throws on a malformed map.
    ObstacleGrid(const msgs::ObstacleMap &map, double link_radius, double blade_radius);

    bool linkBlocked(const Eigen::Vector3d &point) const;
    bool bladeBlocked(const Eigen::Vector3d &point) const;

private:
    long cellAt(const Eigen::Vector3d &point) const;
    void inflate(const std::vector<uint32_t> &cells, double radius, std::vector<uint8_t> &layer) const;

    Eigen::Vector3d      origin_;
    double               voxel_size_ = 0.0;
    long                 size_x_     = 0;
    long                 size_y_     = 0;
    long                 size_z_     = 0;
    std::vector<uint8_t> link_blocked_;
    std::vector<uint8_t> blade_blocked_;
};

}  // namespace planner

#endif  // PLANNER_OBSTACLEGRID_H
