// Copyright by BeeX [2026]

#ifndef PLANNER_JOINTSPACE_H
#define PLANNER_JOINTSPACE_H

#include <kine/Joints.h>

#include <cmath>
#include <vector>

namespace planner {

using JointPath = std::vector<kine::JointAngles>;

// Sum of weight * joint travel: the cost a grasp is ranked by.
inline double jointTravel(const kine::JointAngles &weights, const kine::JointAngles &from,
                          const kine::JointAngles &to) {
    double travel = 0.0;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        travel += weights[j] * std::fabs(to[j] - from[j]);
    }
    return travel;
}

inline double pathTravel(const kine::JointAngles &weights, const JointPath &path) {
    double travel = 0.0;
    for (size_t k = 1; k < path.size(); ++k) {
        travel += jointTravel(weights, path[k - 1], path[k]);
    }
    return travel;
}

// Weighted straight-line distance: what the RRT* trees grow by.
inline double jointDistance(const kine::JointAngles &weights, const kine::JointAngles &a,
                            const kine::JointAngles &b) {
    double sum = 0.0;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        const double d = weights[j] * (b[j] - a[j]);
        sum += d * d;
    }
    return std::sqrt(sum);
}

inline kine::JointAngles interpolate(const kine::JointAngles &a, const kine::JointAngles &b, double fraction) {
    kine::JointAngles out;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        out[j] = a[j] + (b[j] - a[j]) * fraction;
    }
    return out;
}

}  // namespace planner

#endif  // PLANNER_JOINTSPACE_H
