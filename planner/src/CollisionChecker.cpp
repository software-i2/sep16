// Copyright by BeeX [2026]

#include <planner/CollisionChecker.h>

#include <algorithm>
#include <cmath>

namespace planner {

CollisionChecker::CollisionChecker(const kine::ArmBody &body, const ObstacleGrid &grid, double safety_floor_z,
                                   double link_sample_step, size_t blade_stride)
        : body_(body),
          grid_(grid),
          safety_floor_z_(safety_floor_z),
          link_sample_step_(link_sample_step),
          blade_stride_(blade_stride) {}

Verdict CollisionChecker::check(const kine::JointAngles &joints) {
    if (!body_.model().withinLimits(joints)) {
        return Verdict::JOINT_LIMIT;
    }

    body_.pose(joints, pose_, blade_stride_);
    if (kine::lowestPoint(pose_) < safety_floor_z_) {
        return Verdict::FLOOR;
    }

    for (const kine::Segment &link : pose_.links) {
        const Eigen::Vector3d span  = link.end - link.start;
        const int             steps = std::max(1, static_cast<int>(std::ceil(span.norm() / link_sample_step_)));
        for (int n = 0; n <= steps; ++n) {
            if (grid_.linkBlocked(link.start + span * (static_cast<double>(n) / steps))) {
                return Verdict::OBSTACLE;
            }
        }
    }
    for (const Eigen::Vector3d &point : pose_.blade_points) {
        if (grid_.bladeBlocked(point)) {
            return Verdict::OBSTACLE;
        }
    }
    return Verdict::CLEAR;
}

}  // namespace planner
