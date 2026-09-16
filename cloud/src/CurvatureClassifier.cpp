// Copyright by BeeX [2026]

#include <cloud/CurvatureClassifier.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace cloud {
namespace {

constexpr double kNotMeasured      = std::numeric_limits<double>::quiet_NaN();
constexpr double kSmallestDenominator = 1e-12;

// Index of the first pose at or past arc length `L[i] + offset`, clamped to the trace.
size_t poseAtArcLength(const std::vector<double> &arc_length, size_t i, double offset) {
    const auto found = std::lower_bound(arc_length.begin(), arc_length.end(), arc_length[i] + offset);
    return std::min(static_cast<size_t>(found - arc_length.begin()), arc_length.size() - 1);
}

// Median of the measured values within half a window either side, along the trace.
std::vector<double> smoothAlongTrace(const std::vector<double> &values, const std::vector<double> &arc_length,
                                     double window) {
    std::vector<double> out(values.size(), kNotMeasured);
    std::vector<double> nearby;
    for (size_t i = 0; i < values.size(); ++i) {
        nearby.clear();
        for (size_t k = poseAtArcLength(arc_length, i, -window / 2.0); k <= poseAtArcLength(arc_length, i, window / 2.0);
             ++k) {
            if (std::isfinite(values[k])) {
                nearby.push_back(values[k]);
            }
        }
        if (nearby.empty()) {
            continue;
        }
        std::sort(nearby.begin(), nearby.end());
        const size_t middle = nearby.size() / 2;
        out[i] = nearby.size() % 2 == 1 ? nearby[middle] : 0.5 * (nearby[middle - 1] + nearby[middle]);
    }
    return out;
}

// Circumscribed-circle curvature through the poses one chord behind and one chord ahead.
std::vector<double> curvature(const std::vector<Eigen::Vector3d> &points, const std::vector<double> &arc_length,
                              double chord, double min_fraction) {
    std::vector<double> out(points.size(), kNotMeasured);
    for (size_t i = 0; i < points.size(); ++i) {
        const size_t j = poseAtArcLength(arc_length, i, -chord);
        const size_t k = poseAtArcLength(arc_length, i, chord);
        if (arc_length[i] - arc_length[j] < min_fraction * chord || arc_length[k] - arc_length[i] < min_fraction * chord) {
            continue;
        }
        const Eigen::Vector3d &a = points[j];
        const Eigen::Vector3d &b = points[i];
        const Eigen::Vector3d &c = points[k];
        const double denominator = (b - a).norm() * (c - b).norm() * (c - a).norm();
        if (denominator > kSmallestDenominator) {
            out[i] = 2.0 * (b - a).cross(c - a).norm() / denominator;
        }
    }
    return out;
}

// Furthest the image trace strays, in pixels, from the straight line between the poses one chord either side.
std::vector<double> imageBend(const std::vector<Eigen::Vector2d> &pixels, const std::vector<double> &arc_length,
                              double chord, double min_span_px) {
    std::vector<double> out(pixels.size(), kNotMeasured);
    for (size_t i = 0; i < pixels.size(); ++i) {
        const size_t          j    = poseAtArcLength(arc_length, i, -chord);
        const size_t          k    = poseAtArcLength(arc_length, i, chord);
        const Eigen::Vector2d span = pixels[k] - pixels[j];
        const double          n    = span.norm();
        if (n < min_span_px) {
            continue;
        }
        double furthest = 0.0;
        for (size_t q = j; q <= k; ++q) {
            const Eigen::Vector2d d = pixels[q] - pixels[j];
            furthest                = std::max(furthest, std::fabs(span.x() * d.y() - span.y() * d.x()) / n);
        }
        out[i] = furthest;
    }
    return out;
}

double measuredOrZero(double value) { return std::isfinite(value) ? value : 0.0; }

}  // namespace

CurvatureClassifier::CurvatureClassifier(const CurvatureSettings &settings, const CameraModel &camera)
        : settings_(settings), camera_(camera) {}

std::vector<double> CurvatureClassifier::scores(const std::vector<GraspPose> &poses) const {
    if (poses.empty()) {
        return {};
    }

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2d> pixels;
    std::vector<double>          arc_length;
    for (size_t i = 0; i < poses.size(); ++i) {
        points.push_back(poses[i].point);
        double u = 0.0;
        double v = 0.0;
        if (!projectToPixel(camera_, poses[i].point, u, v)) {
            u = v = 0.0;
        }
        pixels.emplace_back(u, v);
        arc_length.push_back(i == 0 ? 0.0 : arc_length[i - 1] + (points[i] - points[i - 1]).norm());
    }

    const std::vector<double> bend3d =
            smoothAlongTrace(curvature(points, arc_length, settings_.curvature_chord, settings_.min_chord_fraction),
                             arc_length, settings_.smoothing_window);
    const std::vector<double> bend2d =
            smoothAlongTrace(imageBend(pixels, arc_length, settings_.bend_chord, settings_.min_bend_span_px),
                             arc_length, settings_.smoothing_window);

    std::vector<double> out(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
        const double features[2] = {std::log1p(measuredOrZero(bend3d[i])), std::log1p(measuredOrZero(bend2d[i]))};
        double       logit       = settings_.bias;
        for (int f = 0; f < 2; ++f) {
            logit += settings_.weights[f] * (features[f] - settings_.feature_mean[f]) / settings_.feature_std[f];
        }
        out[i] = 1.0 / (1.0 + std::exp(-logit));
    }
    return out;
}

std::vector<bool> CurvatureClassifier::isHandle(const std::vector<GraspPose> &poses) const {
    const std::vector<double> probability = scores(poses);
    std::vector<bool>         out(probability.size());
    for (size_t i = 0; i < probability.size(); ++i) {
        out[i] = probability[i] >= settings_.threshold;
    }
    return out;
}

}  // namespace cloud
