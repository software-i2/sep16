// Copyright by BeeX [2026]

#ifndef PLANNER_GRASPGOALS_H
#define PLANNER_GRASPGOALS_H

#include <planner/CollisionChecker.h>

#include <Eigen/Core>

#include <vector>

namespace planner {

// What happened to one candidate. The failures are ordered by how far the check got.
enum class Outcome { UNREACHABLE, JOINT_LIMIT, OFF_APPROACH, NO_JAW_ROLL, FLOOR, OBSTACLE, NOT_PLANNED, NO_PATH, OK };

const char *outcomeName(Outcome outcome);

struct Candidate {
    Eigen::Vector3d point;
    Eigen::Vector3d bar_axis;
    Eigen::Vector3d approach;
};

struct GraspSettings {
    double            grasp_point_from_mount = 0.0;
    double            max_approach_deviation = 0.0;
    kine::JointAngles joint_cost_weights{};
};

// A posture that holds a candidate and is itself allowed.
struct GraspGoal {
    size_t            candidate     = 0;
    kine::JointAngles joints        {};
    double            straight_cost = 0.0;
};

// Adds every allowed holding posture to `goals`; returns NOT_PLANNED if any, else the furthest failure.
Outcome findGraspGoals(const Candidate &candidate, size_t index, const kine::JointAngles &start,
                       const GraspSettings &settings, CollisionChecker &checker, std::vector<GraspGoal> &goals);

}  // namespace planner

#endif  // PLANNER_GRASPGOALS_H
