// Copyright by BeeX [2026]

#ifndef KINE_JOINTS_H
#define KINE_JOINTS_H

#include <array>
#include <cmath>

namespace kine {

enum Joint : int { BASE = 0, SHOULDER, ELBOW, WRIST, JOINT_COUNT };

// Names used for the joints in every config file.
constexpr const char *JOINT_KEYS[JOINT_COUNT] = {"base", "shoulder", "elbow", "wrist"};

using JointAngles = std::array<double, JOINT_COUNT>;

inline double degToRad(double degrees) { return degrees * M_PI / 180.0; }
inline double radToDeg(double radians) { return radians * 180.0 / M_PI; }

inline double wrapToPi(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace kine

#endif  // KINE_JOINTS_H
