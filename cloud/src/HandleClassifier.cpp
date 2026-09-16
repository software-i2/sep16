// Copyright by BeeX [2026]

#include <cloud/CurvatureClassifier.h>
#include <cloud/HandleClassifier.h>

#include <stdexcept>

namespace cloud {

// Add a new method here and give it a section under handle_classifier in cloud.yaml.
std::unique_ptr<HandleClassifier> makeHandleClassifier(const CloudConfig &config) {
    if (config.classifier_method == "curvature") {
        return std::unique_ptr<HandleClassifier>(new CurvatureClassifier(config.curvature, config.camera));
    }
    throw std::invalid_argument("unknown handle_classifier/method '" + config.classifier_method + "'");
}

}  // namespace cloud
