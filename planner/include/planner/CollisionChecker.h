// Copyright by BeeX [2026]

#ifndef PLANNER_COLLISIONCHECKER_H
#define PLANNER_COLLISIONCHECKER_H

#include <kine/ArmBody.h>
#include <planner/ObstacleGrid.h>

namespace planner {

enum class Verdict { CLEAR, JOINT_LIMIT, FLOOR, OBSTACLE };

// Whether one posture is allowed: inside the joint limits, above the safety floor, clear of obstacles.
class CollisionChecker {
public:
    CollisionChecker(const kine::ArmBody &body, const ObstacleGrid &grid, double safety_floor_z,
                     double link_sample_step);

    Verdict check(const kine::JointAngles &joints);

    const kine::ArmModel &model() const { return body_.model(); }

private:
    const kine::ArmBody &body_;
    const ObstacleGrid  &grid_;
    double               safety_floor_z_;
    double               link_sample_step_;
    kine::BodyPose       pose_;
};

}  // namespace planner

#endif  // PLANNER_COLLISIONCHECKER_H
