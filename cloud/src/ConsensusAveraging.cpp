// Copyright by BeeX [2026]

#include <cloud/ConsensusAveraging.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace cloud {
namespace {

constexpr double kShortest = 1e-9;
constexpr double kNotMeasured = std::numeric_limits<double>::quiet_NaN();

// A frame's drift is only solved for when at least one agreeing spot constrains it fully in its plane.
constexpr double kMinimumConstraint = 1.0;

Eigen::Vector3d unit(const Eigen::Vector3d &v) { return v / std::max(v.norm(), kShortest); }

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const size_t middle = values.size() / 2;
    return values.size() % 2 == 1 ? values[middle] : 0.5 * (values[middle - 1] + values[middle]);
}

// One frame's candidates as a chain: consecutive poses close enough are joined into bars.
class Chain {
public:
    Chain(const std::vector<GraspPose> &poses, double max_gap) : poses_(poses) {
        const size_t n = poses.size();
        tangent_.assign(n, Eigen::Vector3d::Constant(kNotMeasured));
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            if (i > 0 && joined(i - 1, max_gap)) {
                sum += unit(poses[i].point - poses[i - 1].point);
            }
            if (i + 1 < n && joined(i, max_gap)) {
                sum += unit(poses[i + 1].point - poses[i].point);
            }
            if (sum.norm() > kShortest) {
                tangent_[i] = sum.normalized();
            }
        }
    }

    const std::vector<GraspPose> &poses() const { return poses_; }

    // Direction of the bar at pose i: the chain tangent, or the pose's own bar axis at a loose end.
    Eigen::Vector3d direction(size_t i) const {
        return std::isfinite(tangent_[i].x()) ? tangent_[i] : unit(poses_[i].bar_axis);
    }

    bool joined(size_t i, double max_gap) const {
        const double length = (poses_[i + 1].point - poses_[i].point).norm();
        return length > kShortest && length <= max_gap;
    }

private:
    const std::vector<GraspPose> &poses_;
    std::vector<Eigen::Vector3d>  tangent_;
};

// Where the chain crosses the plane through `anchor` with normal `normal`, nearest the anchor, within the match distance.
bool crossPlane(const Eigen::Vector3d &anchor, const Eigen::Vector3d &normal, const Chain &chain,
                const ConsensusSettings &s, GraspPose &out) {
    const std::vector<GraspPose> &poses = chain.poses();
    const double                  cos_tolerance = std::cos(s.max_axis_angle);

    std::vector<double> side(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
        side[i] = (poses[i].point - anchor).dot(normal);
    }

    double nearest = s.match_distance;
    bool   found   = false;
    for (size_t i = 0; i + 1 < poses.size(); ++i) {
        const Eigen::Vector3d bar    = poses[i + 1].point - poses[i].point;
        const double          length = bar.norm();
        if (!chain.joined(i, s.max_pose_gap) || std::fabs(bar.dot(normal)) < cos_tolerance * length
            || side[i] * side[i + 1] > 0.0) {
            continue;
        }
        const double          across = side[i] - side[i + 1];
        const double          t      = std::fabs(across) > 0.0 ? side[i] / across : 0.0;
        const Eigen::Vector3d point  = poses[i].point + t * bar;
        const double          distance = (point - anchor).norm();
        if (distance <= nearest) {
            const Eigen::Vector3d next_axis =
                    poses[i + 1].bar_axis.dot(poses[i].bar_axis) >= 0.0 ? poses[i + 1].bar_axis : -poses[i + 1].bar_axis;
            out     = {point, (1.0 - t) * poses[i].bar_axis + t * next_axis,
                       (1.0 - t) * poses[i].approach + t * poses[i + 1].approach};
            nearest = distance;
            found   = true;
        }
    }
    if (found) {
        return true;
    }

    // No bar crosses: a chain end lying close to the plane still counts.
    for (size_t j = 0; j < poses.size(); ++j) {
        if (std::fabs(side[j]) > s.end_tolerance) {
            continue;
        }
        const Eigen::Vector3d direction = chain.direction(j);
        const double          c         = direction.dot(normal);
        if (std::fabs(c) < cos_tolerance) {
            continue;
        }
        const Eigen::Vector3d point    = poses[j].point - direction * (side[j] / c);
        const double          distance = (point - anchor).norm();
        if (distance <= nearest) {
            out     = {point, poses[j].bar_axis, poses[j].approach};
            nearest = distance;
            found   = true;
        }
    }
    return found;
}

enum class Refusal { NONE, TOO_FEW_FRAMES, OUTLIERS, SPREAD };

struct Spot {
    Refusal                                         refusal = Refusal::NONE;
    Eigen::Vector3d                                 normal;
    GraspPose                                       pose;
    std::vector<std::pair<size_t, Eigen::Vector3d>> crossings;  // frame, crossing point
};

Spot agreeOn(const Eigen::Vector3d &anchor, const Eigen::Vector3d &normal, const std::vector<Chain> &chains,
             const ConsensusSettings &s) {
    Spot spot;
    spot.normal = normal;

    std::vector<std::pair<size_t, GraspPose>> found;
    for (size_t k = 0; k < chains.size(); ++k) {
        GraspPose crossing;
        if (crossPlane(anchor, normal, chains[k], s, crossing)) {
            found.emplace_back(k, crossing);
            spot.crossings.emplace_back(k, crossing.point);
        }
    }
    if (static_cast<int>(found.size()) < s.min_agreeing_frames) {
        spot.refusal = Refusal::TOO_FEW_FRAMES;
        return spot;
    }

    const Eigen::Vector3d helper = std::fabs(normal.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d e1     = unit(normal.cross(helper));
    const Eigen::Vector3d e2     = normal.cross(e1);

    std::vector<double> along_e1;
    std::vector<double> along_e2;
    for (const auto &f : found) {
        along_e1.push_back((f.second.point - anchor).dot(e1));
        along_e2.push_back((f.second.point - anchor).dot(e2));
    }
    const double middle_e1 = median(along_e1);
    const double middle_e2 = median(along_e2);

    std::vector<const GraspPose *> inliers;
    for (size_t i = 0; i < found.size(); ++i) {
        if (std::hypot(along_e1[i] - middle_e1, along_e2[i] - middle_e2) <= s.outlier_distance) {
            inliers.push_back(&found[i].second);
        }
    }
    if (static_cast<int>(inliers.size()) < s.min_agreeing_frames) {
        spot.refusal = Refusal::OUTLIERS;
        return spot;
    }

    Eigen::Vector3d centre = Eigen::Vector3d::Zero();
    for (const GraspPose *inlier : inliers) {
        centre += inlier->point;
    }
    centre /= static_cast<double>(inliers.size());

    double squared = 0.0;
    for (const GraspPose *inlier : inliers) {
        squared += (inlier->point - centre).squaredNorm();
    }
    if (std::sqrt(squared / inliers.size()) > s.max_spread) {
        spot.refusal = Refusal::SPREAD;
        return spot;
    }

    const Eigen::Vector3d reference = inliers.front()->bar_axis;
    Eigen::Vector3d       axis      = Eigen::Vector3d::Zero();
    Eigen::Vector3d       approach  = Eigen::Vector3d::Zero();
    for (const GraspPose *inlier : inliers) {
        axis += inlier->bar_axis.dot(reference) >= 0.0 ? inlier->bar_axis : -inlier->bar_axis;
        approach += inlier->approach;
    }
    spot.pose = {centre, unit(axis), unit(approach)};
    return spot;
}

// How far the whole scene moved over the frames, from how each frame's crossings sit against the agreed spots.
void estimateDrift(const std::vector<Spot> &spots, size_t frame_count, const ConsensusSettings &s, double &drift,
                   double &standard_error) {
    std::vector<Eigen::Matrix3d> constraint(frame_count, Eigen::Matrix3d::Zero());
    std::vector<Eigen::Vector3d> offset(frame_count, Eigen::Vector3d::Zero());
    for (const Spot &spot : spots) {
        const Eigen::Matrix3d in_plane = Eigen::Matrix3d::Identity() - spot.normal * spot.normal.transpose();
        for (const auto &crossing : spot.crossings) {
            constraint[crossing.first] += in_plane;
            offset[crossing.first] += crossing.second - spot.pose.point;
        }
    }

    std::vector<size_t>          frames;
    std::vector<Eigen::Vector3d> shift;
    for (size_t k = 0; k < frame_count; ++k) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(constraint[k]);
        const Eigen::Vector3d                          w = solver.eigenvalues();
        if (w(2) < kMinimumConstraint) {
            continue;
        }
        Eigen::Vector3d solved = Eigen::Vector3d::Zero();
        for (int c = 0; c < 3; ++c) {
            if (w(c) > s.drift_direction_ratio * w(2)) {
                const Eigen::Vector3d v = solver.eigenvectors().col(c);
                solved += v * (v.dot(offset[k]) / w(c));
            }
        }
        frames.push_back(k);
        shift.push_back(solved);
    }

    drift          = kNotMeasured;
    standard_error = kNotMeasured;
    if (frames.size() < 2) {
        return;
    }

    double          mean_frame = 0.0;
    Eigen::Vector3d mean_shift = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < frames.size(); ++i) {
        mean_frame += static_cast<double>(frames[i]);
        mean_shift += shift[i];
    }
    mean_frame /= static_cast<double>(frames.size());
    mean_shift /= static_cast<double>(frames.size());

    double          sxx   = 0.0;
    Eigen::Vector3d slope = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < frames.size(); ++i) {
        const double centred = static_cast<double>(frames[i]) - mean_frame;
        sxx += centred * centred;
        slope += centred * (shift[i] - mean_shift);
    }
    slope /= sxx;

    const double span = static_cast<double>(frames.back() - frames.front());
    drift             = slope.norm() * span;
    if (frames.size() < 3) {
        return;
    }

    double residual2 = 0.0;
    for (size_t i = 0; i < frames.size(); ++i) {
        const double centred = static_cast<double>(frames[i]) - mean_frame;
        residual2 += (shift[i] - mean_shift - centred * slope).squaredNorm();
    }
    const double residual = std::sqrt(residual2 / (3.0 * static_cast<double>(frames.size() - 2)));
    standard_error        = residual * span / std::sqrt(sxx);
}

}  // namespace

AveragedCandidates ConsensusAveraging::average(const std::vector<std::vector<GraspPose>> &frames) const {
    AveragedCandidates result;
    if (frames.empty()) {
        result.summary = "consensus: no frames";
        return result;
    }

    std::vector<Chain> chains;
    for (const std::vector<GraspPose> &poses : frames) {
        chains.emplace_back(poses, settings_.max_pose_gap);
    }

    // Anchors are tried newest frame first, so the spots follow the most recent view.
    std::vector<size_t> order(1, frames.size() - 1);
    for (size_t k = 0; k + 1 < frames.size(); ++k) {
        order.push_back(k);
    }

    std::vector<Eigen::Vector3d> tried;
    std::vector<Spot>            accepted;
    int                          too_few = 0, outliers = 0, spread = 0;
    const double                 duplicate2 = settings_.duplicate_distance * settings_.duplicate_distance;
    const auto nearTried = [&](const Eigen::Vector3d &p) {
        return std::any_of(tried.begin(), tried.end(),
                           [&](const Eigen::Vector3d &q) { return (q - p).squaredNorm() <= duplicate2; });
    };

    for (const size_t k : order) {
        for (size_t i = 0; i < frames[k].size(); ++i) {
            const Eigen::Vector3d &anchor = frames[k][i].point;
            if (nearTried(anchor)) {
                continue;
            }
            const Spot spot = agreeOn(anchor, chains[k].direction(i), chains, settings_);
            tried.push_back(anchor);

            if (spot.refusal == Refusal::TOO_FEW_FRAMES) {
                ++too_few;
            } else if (spot.refusal == Refusal::OUTLIERS) {
                ++outliers;
            } else if (spot.refusal == Refusal::SPREAD) {
                ++spread;
            } else if (std::none_of(accepted.begin(), accepted.end(), [&](const Spot &other) {
                           return (other.pose.point - spot.pose.point).norm() <= settings_.duplicate_distance;
                       })) {
                tried.push_back(spot.pose.point);
                accepted.push_back(spot);
            }
        }
    }

    double drift          = 0.0;
    double standard_error = 0.0;
    estimateDrift(accepted, frames.size(), settings_, drift, standard_error);
    const bool drifting = std::isfinite(drift) && drift > settings_.max_drift
                          && !(drift <= settings_.drift_significance_sigmas * standard_error);

    const char *verdict = drifting ? "DRIFTING" : !accepted.empty() ? "OK" : (outliers + spread > 0) ? "NOISY" : "NONE";
    if (!drifting) {
        for (const Spot &spot : accepted) {
            result.candidates.push_back(spot.pose);
        }
    }

    char line[256];
    std::snprintf(line, sizeof(line),
                  "consensus %s over %zu frames: %zu spots agree; refused %d seen in too few frames, %d with outliers, "
                  "%d too spread; drift %.1f mm",
                  verdict, frames.size(), accepted.size(), too_few, outliers, spread,
                  std::isfinite(drift) ? drift * 1000.0 : 0.0);
    result.summary = line;
    return result;
}

}  // namespace cloud
