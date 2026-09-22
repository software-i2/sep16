// Copyright by BeeX [2026]

#include <park/PoseSearch.h>

#include <algorithm>
#include <cmath>

namespace park {
namespace {

constexpr double kParallelTolerance = 1e-3;

Eigen::Vector3d turn(const Eigen::Vector3d &v, double cos_yaw, double sin_yaw) {
    return Eigen::Vector3d(v.x() * cos_yaw - v.y() * sin_yaw, v.x() * sin_yaw + v.y() * cos_yaw, v.z());
}

}  // namespace

PoseSearch::PoseSearch(const kine::ReachTable &table, const SearchBox &box, const Eigen::Isometry3d &body_to_arm,
                       const Eigen::Vector3d &camera_in_body)
        : table_(table), box_(box), body_to_arm_(body_to_arm), camera_in_body_(camera_in_body) {}

bool PoseSearch::admits(const Grasp &grasp, const Pose &pose) const {
    const double cos_yaw = std::cos(-pose.yaw);
    const double sin_yaw = std::sin(-pose.yaw);

    const Eigen::Vector3d moved(grasp.point.x() - pose.x, grasp.point.y() - pose.y, grasp.point.z() - pose.z);
    const Eigen::Vector3d in_body = turn(moved, cos_yaw, sin_yaw);
    const Eigen::Vector3d bar_in_body = turn(grasp.bar_axis, cos_yaw, sin_yaw);

    // Too close and the depth is rubbish, too far and the arm cannot follow it in. Outside the
    // band the grasp does not count at all; it is not traded off against anything.
    const double standoff = (in_body - camera_in_body_).norm();
    if (standoff < box_.standoff_min || standoff > box_.standoff_max) {
        return false;
    }

    const Eigen::Vector3d point = body_to_arm_ * in_body;
    const Eigen::Vector3d bar   = body_to_arm_.linear() * bar_in_body;
    if (bar.norm() < kParallelTolerance) {
        return false;
    }
    const Eigen::Vector3d unit_bar = bar.normalized();

    double base_angle = 0.0;
    if (!table_.baseAngleFor(std::atan2(point.y(), point.x()), base_angle)) {
        return false;
    }

    const double radial = std::hypot(point.x(), point.y());
    const double step   = table_.spec().cell;
    const double probes[5][2] = {{radial, point.z()},        {radial - step, point.z()}, {radial + step, point.z()},
                                 {radial, point.z() - step}, {radial, point.z() + step}};
    for (const auto &probe : probes) {
        const kine::ReachCell *cell = table_.cellAt(probe[0], probe[1]);
        if (!cell) {
            continue;
        }
        for (int b = 0; b < kine::K_REACH_BRANCHES; ++b) {
            const kine::ReachBranch &branch = cell->branch[b];
            if (!branch.reached()) {
                continue;
            }
            const Eigen::Vector3d approach = kine::approachDirection(branch.approach, base_angle);
            const double          along    = unit_bar.dot(approach);
            if (std::sqrt(std::max(0.0, 1.0 - along * along)) >= kParallelTolerance) {
                return true;
            }
        }
    }
    return false;
}

Scored PoseSearch::scoreOne(const std::vector<Grasp> &grasps, const Pose &pose) const {
    Scored out;
    out.pose = pose;
    for (const Grasp &grasp : grasps) {
        if (admits(grasp, pose)) {
            ++out.admitted;
        }
    }
    // Valid grasps, less what the drive costs. travel_cost is how many grasps a metre is worth
    // giving up, so at zero this is the grasp count and nothing else; the drive is time the
    // scene spends moving, so a pose that holds no more than a nearer one should lose.
    out.travel = std::hypot(std::hypot(pose.x, pose.y), pose.z);
    out.score  = out.admitted - box_.travel_cost * out.travel;
    return out;
}

void PoseSearch::shortlist(const std::vector<Grasp> &grasps, size_t count, std::vector<Scored> &out,
                           size_t &poses_scored) const {
    poses_scored = 0;
    out.clear();
    if (grasps.empty() || count == 0) {
        return;
    }

    // Everything the arm could ever reach sits inside one sphere about the arm base, so a body
    // pose that puts the whole cloud outside it cannot admit anything and is skipped whole.
    Eigen::Vector3d centre(0.0, 0.0, 0.0);
    for (const Grasp &grasp : grasps) {
        centre += grasp.point;
    }
    centre /= static_cast<double>(grasps.size());
    double spread = 0.0;
    for (const Grasp &grasp : grasps) {
        spread = std::max(spread, (grasp.point - centre).norm());
    }
    const double cull = table_.spec().radial_min + table_.spec().cells_r * table_.spec().cell + spread
                        + body_to_arm_.translation().norm() + box_.coarse_step;

    std::vector<Scored> coarse;
    for (double x = -box_.box_xy; x <= box_.box_xy + 1e-9; x += box_.coarse_step) {
        for (double y = -box_.box_xy; y <= box_.box_xy + 1e-9; y += box_.coarse_step) {
            for (double z = -box_.box_z; z <= box_.box_z + 1e-9; z += box_.coarse_step) {
                if ((centre - Eigen::Vector3d(x, y, z)).norm() > cull) {
                    continue;
                }
                for (double yaw = -box_.box_yaw; yaw <= box_.box_yaw + 1e-9; yaw += box_.coarse_yaw) {
                    Pose pose;
                    pose.x   = x;
                    pose.y   = y;
                    pose.z   = z;
                    pose.yaw = yaw;
                    coarse.push_back(scoreOne(grasps, pose));
                    ++poses_scored;
                }
            }
        }
    }
    if (coarse.empty()) {
        return;
    }
    std::sort(coarse.begin(), coarse.end(), [](const Scored &a, const Scored &b) { return a.score > b.score; });

    std::vector<Scored> found;
    const size_t        refine = std::min<size_t>(std::max(1, box_.refine_count), coarse.size());
    for (size_t i = 0; i < refine; ++i) {
        const Pose &seed = coarse[i].pose;
        for (double dx = -box_.coarse_step; dx <= box_.coarse_step + 1e-9; dx += box_.fine_step) {
            for (double dy = -box_.coarse_step; dy <= box_.coarse_step + 1e-9; dy += box_.fine_step) {
                for (double dz = -box_.coarse_step; dz <= box_.coarse_step + 1e-9; dz += box_.fine_step) {
                    for (double dyaw = -box_.coarse_yaw; dyaw <= box_.coarse_yaw + 1e-9; dyaw += box_.fine_yaw) {
                        Pose pose;
                        pose.x   = seed.x + dx;
                        pose.y   = seed.y + dy;
                        pose.z   = seed.z + dz;
                        pose.yaw = seed.yaw + dyaw;
                        if (std::fabs(pose.x) > box_.box_xy || std::fabs(pose.y) > box_.box_xy
                            || std::fabs(pose.z) > box_.box_z || std::fabs(pose.yaw) > box_.box_yaw) {
                            continue;
                        }
                        const Scored scored = scoreOne(grasps, pose);
                        ++poses_scored;
                        if (scored.admitted >= box_.min_grasps) {
                            found.push_back(scored);
                        }
                    }
                }
            }
        }
    }

    // Every coarse pose that admits anything, not just the refined few. Reach peaks where the
    // arm is deepest in the scene, so a shortlist ranked by reach alone is all bad standoffs;
    // the coarse grid is at least spread across the box and gives the collision check
    // somewhere further back to find.
    for (const Scored &option : coarse) {
        if (option.admitted >= box_.min_grasps) {
            found.push_back(option);
        }
    }
    std::sort(found.begin(), found.end(), [](const Scored &a, const Scored &b) { return a.score > b.score; });
    if (found.size() > count) {
        found.resize(count);
    }
    out.swap(found);
}

}  // namespace park
