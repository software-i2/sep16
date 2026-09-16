// Copyright by BeeX [2026]

#ifndef KINE_READCONFIG_H
#define KINE_READCONFIG_H

#include <kine/ArmModel.h>

#include <string>
#include <vector>

namespace kine {

// Reads bringup/config/arm.yaml and jaws.yaml through any reader offering number() and table(), e.g. params::Params.
template <class Reader>
ArmConfig readArmConfig(Reader &arm, Reader &jaws) {
    ArmConfig c;

    ArmDimensions &d           = c.dimensions;
    d.base_to_base_axis_z      = arm.number("links/base_to_base_axis_z_m");
    d.base_axis_to_shoulder_x  = arm.number("links/base_axis_to_shoulder_x_m");
    d.base_axis_to_shoulder_z  = arm.number("links/base_axis_to_shoulder_z_m");
    d.shoulder_to_elbow_x      = arm.number("links/shoulder_to_elbow_x_m");
    d.shoulder_to_elbow_z      = arm.number("links/shoulder_to_elbow_z_m");
    d.elbow_frame_yaw          = degToRad(arm.number("links/elbow_frame_yaw_deg"));
    d.elbow_to_wrist_x         = arm.number("links/elbow_to_wrist_x_m");
    d.elbow_to_wrist_z         = arm.number("links/elbow_to_wrist_z_m");
    d.wrist_to_jaw_mount       = arm.number("links/wrist_to_jaw_mount_m");
    d.jaw_mount_to_throat      = jaws.number("mount_to_throat_m");
    d.jaw_mount_to_tip         = jaws.number("mount_to_tip_m");
    d.hinge_roll_at_wrist_zero = degToRad(jaws.number("hinge_roll_at_wrist_zero_deg"));

    for (int j = 0; j < JOINT_COUNT; ++j) {
        const std::string name        = JOINT_KEYS[j];
        c.convention.zero_offset[j]    = degToRad(arm.number("joint_convention/zero_offset_deg/" + name));
        c.convention.direction_sign[j] = arm.number("joint_convention/direction_sign/" + name);
        c.limits.min[j]                = degToRad(arm.number("joint_limits_deg/" + name + "/min"));
        c.limits.max[j]                = degToRad(arm.number("joint_limits_deg/" + name + "/max"));
    }

    JawShape &jaw                = c.jaw;
    jaw.palm_length              = jaws.number("palm_length_m");
    jaw.open_width               = jaws.number("open_width_m");
    jaw.blade_rotation_per_metre = jaws.number("blade_rotation_rad_per_m");
    jaw.hinge_offset_closing     = jaws.number("hinge_offset_closing_m");
    jaw.hinge_offset_approach    = jaws.number("hinge_offset_approach_m");
    for (const std::vector<double> &row : jaws.table("blade_profile_m", 5)) {
        jaw.blade_profile.push_back({row[0], row[1], row[2], row[3], row[4]});
    }
    return c;
}

}  // namespace kine

#endif  // KINE_READCONFIG_H
