// Copyright by BeeX [2026]

#include <cloud/DepthImage.h>

#include <algorithm>

namespace cloud {

DepthImage::DepthImage(const CameraModel &camera, const std::vector<Eigen::Vector3f> &points)
        : width_(camera.width),
          height_(camera.height),
          depth_(static_cast<size_t>(camera.width) * camera.height, std::numeric_limits<double>::infinity()) {
    for (const Eigen::Vector3f &point : points) {
        int u = 0;
        int v = 0;
        if (pixelOf(camera, point.cast<double>(), u, v)) {
            double &depth = depth_[static_cast<size_t>(v) * width_ + u];
            depth         = std::min(depth, -static_cast<double>(point.z()));
        }
    }
}

}  // namespace cloud
