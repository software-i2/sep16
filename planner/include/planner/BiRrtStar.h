// Copyright by BeeX [2026]

#ifndef PLANNER_BIRRTSTAR_H
#define PLANNER_BIRRTSTAR_H

#include <planner/CollisionChecker.h>
#include <planner/JointSpace.h>
#include <planner/PlannerConfig.h>

#include <random>
#include <utility>

namespace planner {

// Bi-directional RRT* in joint space. Every edge of a returned path has been collision checked.
class BiRrtStar {
public:
    BiRrtStar(CollisionChecker &checker, const RrtSettings &settings, const kine::JointAngles &weights);

    // Corners from start to goal; the start itself is never checked, so the arm can always leave where it is.
    bool plan(const kine::JointAngles &start, const kine::JointAngles &goal, JointPath &corners);

private:
    struct Node {
        kine::JointAngles joints;
        int               parent = -1;
        double            cost   = 0.0;
        std::vector<int>  children;
    };
    using Tree = std::vector<Node>;

    bool              edgeClear(const kine::JointAngles &a, const kine::JointAngles &b);
    kine::JointAngles randomJoints();
    int               nearest(const Tree &tree, const kine::JointAngles &target, double &distance) const;
    int               extend(Tree &tree, const kine::JointAngles &target);
    int               connect(Tree &tree, const kine::JointAngles &target);
    void              reparent(Tree &tree, int node, int new_parent, double new_cost);
    void              shortcut(JointPath &corners);

    CollisionChecker &checker_;
    RrtSettings       settings_;
    kine::JointAngles weights_;
    std::mt19937_64   random_;
};

}  // namespace planner

#endif  // PLANNER_BIRRTSTAR_H
