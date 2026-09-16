// Copyright by BeeX [2026]

#ifndef DRIVER_DRIVERCONFIG_H
#define DRIVER_DRIVERCONFIG_H

#include <params/Params.h>

#include <array>
#include <cstdint>
#include <string>

namespace driver {

enum JointIndex : int { JAW = 0, WRIST, ELBOW, SHOULDER, BASE, JOINT_COUNT };

// Names used for the joints in every config file.
constexpr const char *JOINT_KEYS[JOINT_COUNT] = {"jaw", "wrist", "elbow", "shoulder", "base"};

// One joint in ROS units: radians for rotary joints, metres for the jaw.
struct JointSetup {
    std::string name;
    uint8_t     device_id       = 0;
    double      min             = 0.0;
    double      max             = 0.0;
    double      home            = 0.0;
    double      home_speed      = 0.0;
    double      simulated_speed = 0.0;
    double      wire_per_unit   = 0.0;
};

struct DriverConfig {
    std::array<JointSetup, JOINT_COUNT> joints;
    double                              jaw_open_width = 0.0;

    bool        simulated = false;
    std::string serial_port;
    int         baud_rate = 0;

    double poll_rate_hz        = 0.0;
    double reply_timeout_s     = 0.0;
    int    connect_attempts    = 0;
    double connect_retry_s     = 0.0;
    double connect_nudge_speed = 0.0;

    std::string topic_joint_states;
    std::string topic_joint_targets;
    std::string service_home;
    std::string service_open_jaw;
    std::string service_close_jaw;
    std::string service_standby;
};

DriverConfig loadDriverConfig(params::Params &driver, params::Params &arm, params::Params &jaws,
                              params::Params &topics);

}  // namespace driver

#endif  // DRIVER_DRIVERCONFIG_H
