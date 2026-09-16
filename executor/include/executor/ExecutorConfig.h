// Copyright by BeeX [2026]

#ifndef EXECUTOR_EXECUTORCONFIG_H
#define EXECUTOR_EXECUTORCONFIG_H

#include <kine/Joints.h>
#include <params/Params.h>

#include <array>
#include <string>

namespace executor {

// Angles in radians.
struct FollowSettings {
    double max_joint_step              = 0.0;
    double arrival_tolerance           = 0.0;
    double arrival_timeout_s           = 0.0;
    double blocked_min_commanded_step  = 0.0;
    double blocked_min_follow_fraction = 0.0;
    int    blocked_strikes             = 0;
};

struct ExecutorConfig {
    std::array<std::string, kine::JOINT_COUNT> joint_names;
    FollowSettings                             follow;
    double                                     rate_hz            = 0.0;
    double                                     start_tolerance    = 0.0;
    double                                     feedback_timeout_s = 0.0;

    std::string topic_joint_states;
    std::string topic_joint_targets;
    std::string action_execute;
    std::string service_driver_standby;
};

ExecutorConfig loadExecutorConfig(params::Params &executor, params::Params &arm, params::Params &topics);

}  // namespace executor

#endif  // EXECUTOR_EXECUTORCONFIG_H
