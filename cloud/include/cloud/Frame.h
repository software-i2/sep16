// Copyright by BeeX [2026]

#ifndef CLOUD_FRAME_H
#define CLOUD_FRAME_H

#include <cloud/CloudConfig.h>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

namespace cloud {

// Points closer to the camera plane than this cannot be projected.
constexpr double kMinimumDepth = 1e-6;

// Where the jaws could close, as vision offers it.
struct GraspPose {
    Eigen::Vector3d point;
    Eigen::Vector3d bar_axis;
    Eigen::Vector3d approach;
};

// One capture: the cloud and grasp poses in the camera frame, and where the camera was.
struct CameraFrame {
    std::vector<Eigen::Vector3f> points;
    std::vector<GraspPose>       poses;
    Eigen::Isometry3d            camera_to_base = Eigen::Isometry3d::Identity();
};

inline GraspPose transformPose(const Eigen::Isometry3d &transform, const GraspPose &pose) {
    return {transform * pose.point, transform.linear() * pose.bar_axis, transform.linear() * pose.approach};
}

// Pixel coordinates of a camera-frame point; the camera looks down -z. False behind the camera.
inline bool projectToPixel(const CameraModel &camera, const Eigen::Vector3d &point, double &u, double &v) {
    const double depth = -point.z();
    if (!(depth > kMinimumDepth)) {
        return false;
    }
    u = camera.cx + camera.fx * point.x() / depth;
    v = camera.cy - camera.fy * point.y() / depth;
    return true;
}

// The pixel a camera-frame point lands in. False behind the camera or outside the image.
inline bool pixelOf(const CameraModel &camera, const Eigen::Vector3d &point, int &u, int &v) {
    double pu = 0.0;
    double pv = 0.0;
    if (!projectToPixel(camera, point, pu, pv)) {
        return false;
    }
    u = static_cast<int>(std::lround(pu));
    v = static_cast<int>(std::lround(pv));
    return u >= 0 && v >= 0 && u < camera.width && v < camera.height;
}

}  // namespace cloud

#endif  // CLOUD_FRAME_H
