// Copyright by BeeX [2026]

#include <planner/ObstacleGrid.h>

#include <array>
#include <cmath>
#include <stdexcept>

namespace planner {

ObstacleGrid::ObstacleGrid(const msgs::ObstacleMap &map, double link_radius, double blade_radius)
        : origin_(map.origin.x, map.origin.y, map.origin.z),
          voxel_size_(map.voxel_size_m),
          size_x_(map.size_x),
          size_y_(map.size_y),
          size_z_(map.size_z) {
    if (!(voxel_size_ > 0.0)) {
        throw std::invalid_argument("the obstacle map has no voxel size");
    }
    const size_t cell_count = static_cast<size_t>(size_x_ * size_y_ * size_z_);
    for (const std::vector<uint32_t> *cells : {&map.obstacle_cells, &map.handle_cells}) {
        for (uint32_t cell : *cells) {
            if (cell >= cell_count) {
                throw std::invalid_argument("the obstacle map lists a cell outside its own size");
            }
        }
    }

    std::vector<uint32_t> occupied = map.obstacle_cells;
    occupied.insert(occupied.end(), map.handle_cells.begin(), map.handle_cells.end());

    link_blocked_.assign(cell_count, 0);
    inflate(occupied, link_radius, link_blocked_);

    blade_blocked_.assign(cell_count, 0);
    inflate(map.obstacle_cells, blade_radius, blade_blocked_);
    for (uint32_t cell : map.handle_cells) {
        blade_blocked_[cell] = 1;
    }
}

bool ObstacleGrid::linkBlocked(const Eigen::Vector3d &point) const {
    const long cell = cellAt(point);
    return cell >= 0 && link_blocked_[cell] != 0;
}

bool ObstacleGrid::bladeBlocked(const Eigen::Vector3d &point) const {
    const long cell = cellAt(point);
    return cell >= 0 && blade_blocked_[cell] != 0;
}

long ObstacleGrid::cellAt(const Eigen::Vector3d &query) const {
    const Eigen::Vector3d point = query_to_map_ * query;
    const double          x     = std::floor((point.x() - origin_.x()) / voxel_size_);
    const double          y     = std::floor((point.y() - origin_.y()) / voxel_size_);
    const double          z     = std::floor((point.z() - origin_.z()) / voxel_size_);
    if (x < 0.0 || y < 0.0 || z < 0.0 || x >= size_x_ || y >= size_y_ || z >= size_z_) {
        return -1;
    }
    return static_cast<long>(x) + size_x_ * (static_cast<long>(y) + size_y_ * static_cast<long>(z));
}

// Marks every cell whose centre lies within `radius` of the centre of a listed cell.
void ObstacleGrid::inflate(const std::vector<uint32_t> &cells, double radius, std::vector<uint8_t> &layer) const {
    const int                       reach = static_cast<int>(std::floor(radius / voxel_size_));
    std::vector<std::array<int, 3>> offsets;
    for (int dz = -reach; dz <= reach; ++dz) {
        for (int dy = -reach; dy <= reach; ++dy) {
            for (int dx = -reach; dx <= reach; ++dx) {
                if ((dx * dx + dy * dy + dz * dz) * voxel_size_ * voxel_size_ <= radius * radius) {
                    offsets.push_back({dx, dy, dz});
                }
            }
        }
    }

    for (uint32_t cell : cells) {
        const long x = cell % size_x_;
        const long y = (cell / size_x_) % size_y_;
        const long z = cell / (size_x_ * size_y_);
        for (const std::array<int, 3> &offset : offsets) {
            const long nx = x + offset[0];
            const long ny = y + offset[1];
            const long nz = z + offset[2];
            if (nx >= 0 && ny >= 0 && nz >= 0 && nx < size_x_ && ny < size_y_ && nz < size_z_) {
                layer[nx + size_x_ * (ny + size_y_ * nz)] = 1;
            }
        }
    }
}

}  // namespace planner
