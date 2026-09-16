// Copyright by BeeX [2026]

#ifndef CLOUD_HANDLECLASSIFIER_H
#define CLOUD_HANDLECLASSIFIER_H

#include <cloud/Frame.h>

#include <memory>
#include <vector>

namespace cloud {

// Decides, per grasp pose, whether it lies on a handle or on rope.
class HandleClassifier {
public:
    virtual ~HandleClassifier() = default;

    // Poses in the camera frame, in the order vision traced them.
    virtual std::vector<bool> isHandle(const std::vector<GraspPose> &poses) const = 0;
};

// The classifier named by handle_classifier/method. Throws std::invalid_argument on an unknown name.
std::unique_ptr<HandleClassifier> makeHandleClassifier(const CloudConfig &config);

}  // namespace cloud

#endif  // CLOUD_HANDLECLASSIFIER_H
