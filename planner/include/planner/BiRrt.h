// Copyright by BeeX [2026]

#ifndef PLANNER_BIRRT_H
#define PLANNER_BIRRT_H

#include <planner/CollisionChecker.h>
#include <planner/JointSpace.h>
#include <planner/PlannerConfig.h>

#include <chrono>
#include <random>

namespace planner {

// Bi-directional RRT-Connect in joint space. Every edge of a returned path has been collision
// checked. The search stops at the first join and leaves path quality to shortcut().
class BiRrt {
public:
    BiRrt(CollisionChecker &checker, const RrtSettings &settings, const kine::JointAngles &weights);

    // Corners from start to goal; the start itself is never checked, so the arm can always leave where it is.
    bool plan(const kine::JointAngles &start, const kine::JointAngles &goal, JointPath &corners, double budget_s);

private:
    struct Node {
        kine::JointAngles joints;
        int               parent = -1;
    };
    using Tree = std::vector<Node>;

    bool              edgeClear(const kine::JointAngles &a, const kine::JointAngles &b);
    kine::JointAngles randomJoints();
    int               nearest(const Tree &tree, const kine::JointAngles &target, double &distance) const;
    int               extend(Tree &tree, const kine::JointAngles &target);
    int               connect(Tree &tree, const kine::JointAngles &target);
    void              shortcut(JointPath &corners);

    CollisionChecker                     &checker_;
    RrtSettings                           settings_;
    kine::JointAngles                     weights_;
    std::mt19937_64                       random_;
    std::chrono::steady_clock::time_point started_;
    double                                budget_ = 0.0;
};

}  // namespace planner

#endif  // PLANNER_BIRRT_H
