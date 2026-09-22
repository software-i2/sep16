// Copyright by BeeX [2026]

#ifndef TASK_TASKCONFIG_H
#define TASK_TASKCONFIG_H

#include <params/Params.h>

#include <string>

namespace task {

// Lengths in metres.
struct TaskConfig {
    double rate_hz          = 0.0;
    double server_wait_s    = 0.0;
    double stream_timeout_s = 0.0;
    double spot_search_s    = 0.0;  // how long to keep surveying with nothing worth parking for in view
    double repark_search_s  = 0.0;  // how long to keep reparking while the search never moves the vehicle
    double retry_delay_s    = 0.0;
    int    retarget_attempts = 0;  // looks from one park pose before it is given up on
    int    park_attempts     = 0;  // park poses tried before the pick is failed

    std::string base_frame;
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
    std::string topic_camera_cloud;
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
