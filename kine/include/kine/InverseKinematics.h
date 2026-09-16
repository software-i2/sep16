// Copyright by BeeX [2026]

#ifndef KINE_INVERSEKINEMATICS_H
#define KINE_INVERSEKINEMATICS_H

#include <kine/ArmModel.h>

#include <Eigen/Core>

namespace kine {

enum class IkResult { SOLVED, TOO_FAR, TOO_CLOSE, JOINT_LIMIT };

// Puts the point `along` metres down the wrist axis on `target`; the wrist roll is copied from `seed`.
IkResult solvePosition(const ArmModel &model,
                       const Eigen::Vector3d &target,
                       double along,
                       bool elbow_up,
                       const JointAngles &seed,
                       JointAngles &out);

// The equivalent of `angle` (plus or minus a full turn) inside the joint's limits, nearest to `seed`.
bool fitIntoLimits(const ArmModel &model, int joint, double angle, double seed, double &out);

// The 2 wrist rolls (half a turn apart) that close the blades across `bar_axis`; 0 if the bar lies along the approach.
int wristRollsAcrossBar(const ArmModel &model, const JointAngles &joints, const Eigen::Vector3d &bar_axis,
                        double rolls[2]);

}  // namespace kine

#endif  // KINE_INVERSEKINEMATICS_H
