// Copyright by BeeX [2026]

#ifndef CLOUD_DEPTHIMAGE_H
#define CLOUD_DEPTHIMAGE_H

#include <cloud/Frame.h>

#include <limits>
#include <vector>

namespace cloud {

// The nearest depth seen in each pixel, rebuilt from a camera-frame cloud.
class DepthImage {
public:
    DepthImage(const CameraModel &camera, const std::vector<Eigen::Vector3f> &points);

    int    width() const { return width_; }
    int    height() const { return height_; }
    double at(int u, int v) const { return depth_[static_cast<size_t>(v) * width_ + u]; }
    bool   seen(int u, int v) const { return at(u, v) < std::numeric_limits<double>::infinity(); }

private:
    int                 width_;
    int                 height_;
    std::vector<double> depth_;
};

}  // namespace cloud

#endif  // CLOUD_DEPTHIMAGE_H
