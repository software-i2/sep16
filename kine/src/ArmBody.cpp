// Copyright by BeeX [2026]

#include <kine/ArmBody.h>
#include <kine/ForwardKinematics.h>

#include <algorithm>
#include <stdexcept>

namespace kine {

ArmBody::ArmBody(const ArmModel &model, const JawShape &jaw, double blade_sample_step) : model_(model), jaw_(jaw) {
    if (!(blade_sample_step > 0.0)) {
        throw std::invalid_argument("blade sample step must be positive");
    }
    if (jaw_.blade_profile.empty()) {
        throw std::invalid_argument("jaws blade_profile_m is empty");
    }

    const double opening = jaw_.open_width * jaw_.blade_rotation_per_metre;
    const double c       = std::cos(opening);
    const double s       = std::sin(opening);

    // Local frame of the jaw mount: x down the approach, y along the hinge, z along the closing line.
    for (const double side : {1.0, -1.0}) {
        for (const BladeBand &band : jaw_.blade_profile) {
            const double span_hinge    = 2.0 * band.hinge_half_width;
            const double span_closing  = band.closing_max - band.closing_min;
            const double span_approach = band.approach_max - band.approach_min;
            if (span_hinge < 0.0 || span_closing < 0.0 || span_approach < 0.0) {
                throw std::invalid_argument("a blade_profile_m row has its min above its max");
            }
            const int steps_hinge    = std::max(1, static_cast<int>(std::ceil(span_hinge / blade_sample_step)));
            const int steps_closing  = std::max(1, static_cast<int>(std::ceil(span_closing / blade_sample_step)));
            const int steps_approach = std::max(1, static_cast<int>(std::ceil(span_approach / blade_sample_step)));

            for (int i = 0; i <= steps_hinge; ++i) {
                const double hinge = -band.hinge_half_width + span_hinge * i / steps_hinge;
                for (int k = 0; k <= steps_closing; ++k) {
                    const double closing = band.closing_min + span_closing * k / steps_closing;
                    for (int m = 0; m <= steps_approach; ++m) {
                        const double approach = band.approach_min + span_approach * m / steps_approach;
                        blade_local_.emplace_back(jaw_.hinge_offset_approach - closing * s + approach * c,
                                                  hinge,
                                                  side * (jaw_.hinge_offset_closing + closing * c + approach * s));
                    }
                }
            }
        }
    }
}

void ArmBody::pose(const JointAngles &joints, BodyPose &out) const {
    const ArmPoints points = forwardKinematics(model_, joints);
    const JawAxes   axes   = jawAxes(model_, joints);

    out.links[UPPER_ARM]  = {points.shoulder, points.elbow};
    out.links[FOREARM]    = {points.elbow, points.wrist};
    out.links[WRIST_TUBE] = {points.wrist, points.jaw_mount};
    out.links[PALM]       = {points.jaw_mount, points.jaw_mount + axes.approach * jaw_.palm_length};
    out.throat            = points.throat;
    out.tip               = points.tip;

    out.blade_points.resize(blade_local_.size());
    for (size_t i = 0; i < blade_local_.size(); ++i) {
        const Eigen::Vector3d &local = blade_local_[i];
        out.blade_points[i] = points.jaw_mount + axes.approach * local.x() + axes.hinge * local.y()
                              + axes.closing * local.z();
    }
}

double lowestPoint(const BodyPose &body) {
    double lowest = std::min(body.throat.z(), body.tip.z());
    for (const Segment &link : body.links) {
        lowest = std::min({lowest, link.start.z(), link.end.z()});
    }
    for (const Eigen::Vector3d &point : body.blade_points) {
        lowest = std::min(lowest, point.z());
    }
    return lowest;
}

}  // namespace kine
