// Copyright by BeeX [2026]

#ifndef TASK_TASKCONFIG_H
#define TASK_TASKCONFIG_H

#include <params/Params.h>

#include <string>

namespace task {

// Lengths in metres.
struct TaskConfig {
    double rate_hz       = 0.0;
    double server_wait_s = 0.0;
    double retry_delay_s = 0.0;
    int    max_attempts  = 0;

    std::string base_frame;
    int         reverify_attempts       = 0;
    double      reverify_match_distance = 0.0;
    double      reverify_transform_wait = 0.0;

    std::string jaw_joint_name;
    double      jaw_closed_width     = 0.0;
    double      jaw_settle_tolerance = 0.0;
    double      jaw_settle_time_s    = 0.0;
    double      jaw_grabbed_margin   = 0.0;
    double      jaw_timeout_s        = 0.0;

    std::string service_start;
    std::string service_stop;
    std::string topic_state;
    std::string topic_joint_states;
    std::string action_collect;
    std::string action_park;
    std::string action_plan;
    std::string action_execute;
    std::string service_close_jaw;
    std::string service_standby;
};

TaskConfig loadTaskConfig(params::Params &task, params::Params &arm, params::Params &topics);

}  // namespace task

#endif  // TASK_TASKCONFIG_H
