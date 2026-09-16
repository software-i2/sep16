// Copyright by BeeX [2026]

#include <kine/ForwardKinematics.h>

namespace kine {
namespace {

// A point in the arm plane (x forward, z up) turned about the base axis into the base frame.
Eigen::Vector3d toBaseFrame(double x, double z, double base_height, double base_angle) {
    return Eigen::Vector3d(x * std::cos(base_angle), x * std::sin(base_angle), z + base_height);
}

// Turns a direction about +z, the base axis.
Eigen::Vector3d turnAboutBase(double x, double y, double z, double base_angle) {
    const double c = std::cos(base_angle);
    const double s = std::sin(base_angle);
    return Eigen::Vector3d(x * c - y * s, x * s + y * c, z);
}

}  // namespace

ArmPoints forwardKinematics(const ArmModel &model, const JointAngles &joints) {
    const ArmDimensions &d       = model.dimensions();
    const PlanarLink     upper   = model.upperArm();
    const PlanarLink     forearm = model.forearmTo(0.0);

    const double upper_angle = upper.angle + joints[SHOULDER];
    const double wrist_axis  = model.elbowSign() * joints[ELBOW] + joints[SHOULDER];

    const double shoulder_x = d.base_axis_to_shoulder_x;
    const double shoulder_z = d.base_axis_to_shoulder_z;
    const double elbow_x    = shoulder_x + upper.length * std::sin(upper_angle);
    const double elbow_z    = shoulder_z + upper.length * std::cos(upper_angle);
    const double wrist_x    = elbow_x + forearm.length * std::sin(forearm.angle + wrist_axis);
    const double wrist_z    = elbow_z + forearm.length * std::cos(forearm.angle + wrist_axis);

    const double axis_x = std::sin(wrist_axis);
    const double axis_z = std::cos(wrist_axis);
    const double base_z = d.base_to_base_axis_z;
    const double base   = joints[BASE];

    const auto alongWrist = [&](double distance) {
        return toBaseFrame(wrist_x + distance * axis_x, wrist_z + distance * axis_z, base_z, base);
    };

    ArmPoints points;
    points.shoulder  = toBaseFrame(shoulder_x, shoulder_z, base_z, base);
    points.elbow     = toBaseFrame(elbow_x, elbow_z, base_z, base);
    points.wrist     = toBaseFrame(wrist_x, wrist_z, base_z, base);
    points.jaw_mount = alongWrist(d.wrist_to_jaw_mount);
    points.throat    = alongWrist(model.throatDistance());
    points.tip       = alongWrist(model.tipDistance());
    return points;
}

JawAxes jawAxes(const ArmModel &model, const JointAngles &joints) {
    const double wrist_axis = model.elbowSign() * joints[ELBOW] + joints[SHOULDER];
    const double sa         = std::sin(wrist_axis);
    const double ca         = std::cos(wrist_axis);

    const double roll = model.dimensions().hinge_roll_at_wrist_zero + joints[WRIST];
    const double cr   = std::cos(roll);
    const double sr   = std::sin(roll);

    JawAxes axes;
    axes.approach = turnAboutBase(sa, 0.0, ca, joints[BASE]);
    axes.hinge    = turnAboutBase(cr * ca, sr, -cr * sa, joints[BASE]);
    axes.closing  = turnAboutBase(-sr * ca, cr, sr * sa, joints[BASE]);
    return axes;
}

}  // namespace kine
