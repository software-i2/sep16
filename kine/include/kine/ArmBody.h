// Copyright by BeeX [2026]

#ifndef KINE_ARMBODY_H
#define KINE_ARMBODY_H

#include <kine/ArmModel.h>

#include <Eigen/Core>

#include <array>
#include <vector>

namespace kine {

enum BodyLink : int { UPPER_ARM = 0, FOREARM, WRIST_TUBE, PALM, BODY_LINK_COUNT };

constexpr const char *BODY_LINK_NAMES[BODY_LINK_COUNT] = {"upper_arm", "forearm", "wrist_tube", "palm"};

struct Segment {
    Eigen::Vector3d start;
    Eigen::Vector3d end;
};

// The collision shape at one posture: link centrelines, the tool points, and samples of both open jaw blades.
struct BodyPose {
    std::array<Segment, BODY_LINK_COUNT> links;
    Eigen::Vector3d                      throat;
    Eigen::Vector3d                      tip;
    std::vector<Eigen::Vector3d>         blade_points;
};

class ArmBody {
public:
    // Samples the blade profile every `blade_sample_step` metres. Throws std::invalid_argument on a bad shape.
    ArmBody(const ArmModel &model, const JawShape &jaw, double blade_sample_step);

    void pose(const JointAngles &joints, BodyPose &out) const;

    const ArmModel &model() const { return model_; }
    size_t          pointsPerBlade() const { return blade_local_.size() / 2; }

private:
    ArmModel                     model_;
    JawShape                     jaw_;
    std::vector<Eigen::Vector3d> blade_local_;
};

// Lowest height any part of the body reaches.
double lowestPoint(const BodyPose &body);

}  // namespace kine

#endif  // KINE_ARMBODY_H
