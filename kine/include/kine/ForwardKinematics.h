// Copyright by BeeX [2026]

#ifndef KINE_FORWARDKINEMATICS_H
#define KINE_FORWARDKINEMATICS_H

#include <kine/ArmModel.h>

#include <Eigen/Core>

namespace kine {

// Pivots and tool points in the arm base frame.
struct ArmPoints {
    Eigen::Vector3d shoulder;
    Eigen::Vector3d elbow;
    Eigen::Vector3d wrist;
    Eigen::Vector3d jaw_mount;
    Eigen::Vector3d throat;
    Eigen::Vector3d tip;
};

// Unit directions of the jaw: down the wrist axis, along the blade hinge, and the line the blades close on.
struct JawAxes {
    Eigen::Vector3d approach;
    Eigen::Vector3d hinge;
    Eigen::Vector3d closing;
};

ArmPoints forwardKinematics(const ArmModel &model, const JointAngles &joints);

JawAxes jawAxes(const ArmModel &model, const JointAngles &joints);

}  // namespace kine

#endif  // KINE_FORWARDKINEMATICS_H
