// Copyright by BeeX [2026]

#include <kine/ArmModel.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace kine {
namespace {

constexpr double kShortestLink = 1e-6;
constexpr double kYawTolerance = 1e-3;

}  // namespace

ArmModel::ArmModel(const ArmConfig &config)
        : dimensions_(config.dimensions), convention_(config.convention) {
    const double yaw = std::fabs(wrapToPi(dimensions_.elbow_frame_yaw));
    if (yaw < kYawTolerance) {
        elbow_sign_ = 1.0;
    } else if (std::fabs(yaw - M_PI) < kYawTolerance) {
        elbow_sign_ = -1.0;
    } else {
        throw std::invalid_argument("elbow_frame_yaw_deg must be 0 or 180");
    }

    upper_arm_.length = std::hypot(dimensions_.shoulder_to_elbow_x, dimensions_.shoulder_to_elbow_z);
    upper_arm_.angle  = std::atan2(dimensions_.shoulder_to_elbow_x, dimensions_.shoulder_to_elbow_z);
    if (upper_arm_.length < kShortestLink || forearmTo(0.0).length < kShortestLink) {
        throw std::invalid_argument("the upper arm or the forearm has no length");
    }

    for (int j = 0; j < JOINT_COUNT; ++j) {
        if (std::fabs(convention_.direction_sign[j]) != 1.0) {
            throw std::invalid_argument(std::string("direction_sign of ") + JOINT_KEYS[j] + " must be 1 or -1");
        }
        if (!(config.limits.min[j] < config.limits.max[j])) {
            throw std::invalid_argument(std::string("joint limit min of ") + JOINT_KEYS[j] + " must be below max");
        }
        const double a = toModel(j, config.limits.min[j]);
        const double b = toModel(j, config.limits.max[j]);
        lower_[j]      = std::min(a, b);
        upper_[j]      = std::max(a, b);
    }
}

double ArmModel::toModel(int joint, double reported) const {
    return convention_.direction_sign[joint] * (reported - convention_.zero_offset[joint]);
}

double ArmModel::toReported(int joint, double model) const {
    return convention_.zero_offset[joint] + convention_.direction_sign[joint] * model;
}

JointAngles ArmModel::toModel(const JointAngles &reported) const {
    JointAngles out;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        out[j] = toModel(j, reported[j]);
    }
    return out;
}

JointAngles ArmModel::toReported(const JointAngles &model) const {
    JointAngles out;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        out[j] = toReported(j, model[j]);
    }
    return out;
}

bool ArmModel::withinLimits(const JointAngles &model) const {
    for (int j = 0; j < JOINT_COUNT; ++j) {
        if (model[j] < lower_[j] || model[j] > upper_[j]) {
            return false;
        }
    }
    return true;
}

PlanarLink ArmModel::forearmTo(double along_wrist_axis) const {
    const double x = elbow_sign_ * dimensions_.elbow_to_wrist_x;
    const double z = dimensions_.elbow_to_wrist_z + along_wrist_axis;
    PlanarLink   link;
    link.length = std::hypot(x, z);
    link.angle  = std::atan2(x, z);
    return link;
}

double ArmModel::throatDistance() const {
    return dimensions_.wrist_to_jaw_mount + dimensions_.jaw_mount_to_throat;
}

double ArmModel::tipDistance() const {
    return dimensions_.wrist_to_jaw_mount + dimensions_.jaw_mount_to_tip;
}

double ArmModel::reachFromBase(double along_wrist_axis) const {
    const double shoulder = std::hypot(dimensions_.base_axis_to_shoulder_x,
                                       dimensions_.base_to_base_axis_z + dimensions_.base_axis_to_shoulder_z);
    return shoulder + upper_arm_.length + forearmTo(along_wrist_axis).length;
}

}  // namespace kine
