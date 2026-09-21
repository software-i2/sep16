// Copyright by BeeX [2026]

#ifndef PARK_POSESEARCH_H
#define PARK_POSESEARCH_H

#include <kine/Reach.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace park {

// A grasp the vehicle is trying to bring into reach, in the frame the search is asked in.
struct Grasp {
    Eigen::Vector3d point;
    Eigen::Vector3d bar_axis;
};

// Where the vehicle body goes, relative to where it is now.
struct Pose {
    double x   = 0.0;
    double y   = 0.0;
    double z   = 0.0;
    double yaw = 0.0;
};

struct Scored {
    Pose   pose;
    int    admitted = 0;
    double travel   = 0.0;
    double score    = 0.0;
};

// Bounded so the search cannot explode. Steps are metres and radians.
struct SearchBox {
    double box_xy       = 0.0;
    double box_z        = 0.0;
    double box_yaw      = 0.0;
    double coarse_step  = 0.0;
    double coarse_yaw   = 0.0;
    double fine_step    = 0.0;
    double fine_yaw     = 0.0;
    int    refine_count  = 0;
    int    min_grasps    = 0;
    double standoff_min  = 0.0;  // a grasp only counts while the camera is this far from it
    double standoff_max  = 0.0;
};

// Coarse sweep then a fine pass around the best cells, scored by how many grasps the reach
// table admits. The table models reach, the base window and the jaw roll; it models neither
// obstacles nor the vehicle, which are axis-dependent and the table is not, so a pose it likes
// still has to survive the planner.
class PoseSearch {
public:
    PoseSearch(const kine::ReachTable &table, const SearchBox &box, const Eigen::Isometry3d &body_to_arm,
               const Eigen::Vector3d &camera_in_body);

    // Whether one candidate is reachable with the arm base placed by `pose`.
    bool admits(const Grasp &grasp, const Pose &pose) const;

    Scored scoreOne(const std::vector<Grasp> &grasps, const Pose &pose) const;

    // The `count` best poses in the box by reach alone, best first, and how many the search
    // looked at. Reach is all this can rank on, so the caller is expected to put the
    // shortlist through a real collision check before believing any of it.
    void shortlist(const std::vector<Grasp> &grasps, size_t count, std::vector<Scored> &out,
                   size_t &poses_scored) const;

private:
    const kine::ReachTable &table_;
    SearchBox               box_;
    Eigen::Isometry3d       body_to_arm_;
    Eigen::Vector3d         camera_in_body_;
};

}  // namespace park

#endif  // PARK_POSESEARCH_H
