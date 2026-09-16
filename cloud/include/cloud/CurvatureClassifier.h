// Copyright by BeeX [2026]

#ifndef CLOUD_CURVATURECLASSIFIER_H
#define CLOUD_CURVATURECLASSIFIER_H

#include <cloud/HandleClassifier.h>

namespace cloud {

// Logistic regression on how much the pose trace bends: in 3D over one chord, and in the image over another.
class CurvatureClassifier : public HandleClassifier {
public:
    CurvatureClassifier(const CurvatureSettings &settings, const CameraModel &camera);

    std::vector<bool> isHandle(const std::vector<GraspPose> &poses) const override;

    // Probability of handle per pose.
    std::vector<double> scores(const std::vector<GraspPose> &poses) const;

private:
    CurvatureSettings settings_;
    CameraModel       camera_;
};

}  // namespace cloud

#endif  // CLOUD_CURVATURECLASSIFIER_H
