// Copyright by BeeX [2026]

#include <kine/ForwardKinematics.h>
#include <kine/InverseKinematics.h>

#include <algorithm>
#include <limits>

namespace kine {
namespace {

constexpr double kGeometryTolerance = 1e-9;
constexpr double kParallelTolerance = 1e-3;

}  // namespace

bool fitIntoLimits(const ArmModel &model, int joint, double angle, double seed, double &out) {
    bool   found   = false;
    double nearest = std::numeric_limits<double>::max();
    for (int turns = -1; turns <= 1; ++turns) {
        const double candidate = angle + 2.0 * M_PI * turns;
        if (candidate < model.lowerLimit(joint) || candidate > model.upperLimit(joint)) {
            continue;
        }
        if (std::fabs(candidate - seed) < nearest) {
            nearest = std::fabs(candidate - seed);
            out     = candidate;
            found   = true;
        }
    }
    return found;
}

IkResult solvePosition(const ArmModel &model,
                       const Eigen::Vector3d &target,
                       double along,
                       bool elbow_up,
                       const JointAngles &seed,
                       JointAngles &out) {
    const ArmDimensions &d      = model.dimensions();
    const double         radial = std::hypot(target.x(), target.y());

    const double base_angle = radial >= kGeometryTolerance ? std::atan2(target.y(), target.x()) : seed[BASE];
    if (!fitIntoLimits(model, BASE, base_angle, seed[BASE], out[BASE])) {
        return IkResult::JOINT_LIMIT;
    }

    const PlanarLink upper   = model.upperArm();
    const PlanarLink forearm = model.forearmTo(along);

    const double x        = radial - d.base_axis_to_shoulder_x;
    const double z        = target.z() - d.base_to_base_axis_z - d.base_axis_to_shoulder_z;
    const double distance = std::hypot(x, z);
    if (distance > upper.length + forearm.length + kGeometryTolerance) {
        return IkResult::TOO_FAR;
    }
    if (distance < std::fabs(upper.length - forearm.length) - kGeometryTolerance) {
        return IkResult::TOO_CLOSE;
    }

    const double cos_bend = std::max(-1.0, std::min(1.0, (distance * distance - upper.length * upper.length
                                                          - forearm.length * forearm.length)
                                                                 / (2.0 * upper.length * forearm.length)));
    const double bend     = elbow_up ? std::acos(cos_bend) : -std::acos(cos_bend);
    const double upper_direction =
            std::atan2(x, z) - std::atan2(forearm.length * std::sin(bend), upper.length + forearm.length * std::cos(bend));

    const double shoulder = upper_direction - upper.angle;
    const double elbow    = model.elbowSign() * (upper_direction + bend - forearm.angle - shoulder);

    if (!fitIntoLimits(model, SHOULDER, shoulder, seed[SHOULDER], out[SHOULDER])
        || !fitIntoLimits(model, ELBOW, elbow, seed[ELBOW], out[ELBOW])) {
        return IkResult::JOINT_LIMIT;
    }
    out[WRIST] = seed[WRIST];
    return IkResult::SOLVED;
}

int wristRollsAcrossBar(const ArmModel &model, const JointAngles &joints, const Eigen::Vector3d &bar_axis,
                        double rolls[2]) {
    if (bar_axis.norm() < kParallelTolerance) {
        return 0;
    }
    JointAngles at_zero = joints;
    at_zero[WRIST]      = 0.0;
    const JawAxes axes  = jawAxes(model, at_zero);

    // The hinge at roll r is hinge(0) cos r + closing(0) sin r.
    const Eigen::Vector3d bar = bar_axis.normalized();
    const double          u   = axes.hinge.dot(bar);
    const double          v   = axes.closing.dot(bar);
    if (std::hypot(u, v) < kParallelTolerance) {
        return 0;
    }
    rolls[0] = std::atan2(v, u);
    rolls[1] = wrapToPi(rolls[0] + M_PI);
    return 2;
}

}  // namespace kine
