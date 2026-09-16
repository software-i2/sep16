// Copyright by BeeX [2026]

#ifndef KINE_ARMMODEL_H
#define KINE_ARMMODEL_H

#include <kine/Joints.h>

#include <vector>

namespace kine {

// Link offsets of the planar arm, metres; angles radians.
struct ArmDimensions {
    double base_to_base_axis_z      = 0.0;
    double base_axis_to_shoulder_x  = 0.0;
    double base_axis_to_shoulder_z  = 0.0;
    double shoulder_to_elbow_x      = 0.0;
    double shoulder_to_elbow_z      = 0.0;
    double elbow_frame_yaw          = 0.0;
    double elbow_to_wrist_x         = 0.0;
    double elbow_to_wrist_z         = 0.0;
    double wrist_to_jaw_mount       = 0.0;
    double jaw_mount_to_throat      = 0.0;
    double jaw_mount_to_tip         = 0.0;
    double hinge_roll_at_wrist_zero = 0.0;
};

// Model angle = direction_sign * (reported angle - zero_offset), radians.
struct JointConvention {
    JointAngles zero_offset{};
    JointAngles direction_sign{};
};

// Vendor limits in reported radians.
struct JointLimits {
    JointAngles min{};
    JointAngles max{};
};

// One 5 mm slice of a jaw blade, in the blade's own frame, metres.
struct BladeBand {
    double approach_min;
    double approach_max;
    double closing_min;
    double closing_max;
    double hinge_half_width;
};

struct JawShape {
    double                 palm_length              = 0.0;
    double                 open_width               = 0.0;
    double                 blade_rotation_per_metre = 0.0;
    double                 hinge_offset_closing     = 0.0;
    double                 hinge_offset_approach    = 0.0;
    std::vector<BladeBand> blade_profile;
};

struct ArmConfig {
    ArmDimensions   dimensions;
    JointConvention convention;
    JointLimits     limits;
    JawShape        jaw;
};

// A link in the arm plane: length, and angle from +z towards +x.
struct PlanarLink {
    double length = 0.0;
    double angle  = 0.0;
};

class ArmModel {
public:
    // Throws std::invalid_argument when the configuration cannot describe this arm.
    explicit ArmModel(const ArmConfig &config);

    const ArmDimensions &dimensions() const { return dimensions_; }

    double      toModel(int joint, double reported) const;
    double      toReported(int joint, double model) const;
    JointAngles toModel(const JointAngles &reported) const;
    JointAngles toReported(const JointAngles &model) const;

    double lowerLimit(int joint) const { return lower_[joint]; }
    double upperLimit(int joint) const { return upper_[joint]; }
    bool   withinLimits(const JointAngles &model) const;

    PlanarLink upperArm() const { return upper_arm_; }
    PlanarLink forearmTo(double along_wrist_axis) const;
    double     elbowSign() const { return elbow_sign_; }

    double throatDistance() const;
    double tipDistance() const;
    double reachFromBase(double along_wrist_axis) const;

private:
    ArmDimensions   dimensions_;
    JointConvention convention_;
    JointAngles     lower_{};
    JointAngles     upper_{};
    PlanarLink      upper_arm_;
    double          elbow_sign_ = 1.0;
};

}  // namespace kine

#endif  // KINE_ARMMODEL_H
