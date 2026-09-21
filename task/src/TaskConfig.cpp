// Copyright by BeeX [2026]

#include <task/TaskConfig.h>

namespace task {

TaskConfig loadTaskConfig(params::Params &task, params::Params &arm, params::Params &topics) {
    TaskConfig c;
    c.rate_hz       = task.number("rate_hz");
    c.server_wait_s = task.number("server_wait_s");
    c.retry_delay_s = task.number("retry_delay_s");
    c.max_attempts  = task.whole("max_attempts");

    c.jaw_joint_name       = arm.text("joint_names/jaw");
    c.jaw_closed_width     = arm.number("jaw_limits_m/min");
    c.jaw_settle_tolerance = task.number("jaw/settle_tolerance_m");
    c.jaw_settle_time_s    = task.number("jaw/settle_time_s");
    c.jaw_grabbed_margin   = task.number("jaw/grabbed_margin_m");
    c.jaw_timeout_s        = task.number("jaw/timeout_s");

    task.require(c.rate_hz > 0.0, "rate_hz", "positive");
    task.require(c.server_wait_s > 0.0, "server_wait_s", "positive");
    task.require(c.retry_delay_s >= 0.0, "retry_delay_s", "zero or more");
    task.require(c.max_attempts >= 0, "max_attempts", "zero or more");
    task.require(c.jaw_settle_tolerance > 0.0, "jaw/settle_tolerance_m", "positive");
    task.require(c.jaw_settle_time_s > 0.0, "jaw/settle_time_s", "positive");
    task.require(c.jaw_grabbed_margin > 0.0, "jaw/grabbed_margin_m", "positive");
    task.require(c.jaw_timeout_s > c.jaw_settle_time_s, "jaw/timeout_s", "longer than jaw/settle_time_s");

    c.service_start      = topics.text("task_start");
    c.service_stop       = topics.text("task_stop");
    c.topic_state        = topics.text("task_state");
    c.topic_joint_states = topics.text("joint_states");
    c.action_collect     = topics.text("cloud_collect");
    c.action_park        = topics.text("park_park");
    c.action_plan        = topics.text("planner_plan");
    c.action_execute     = topics.text("executor_execute");
    c.service_close_jaw  = topics.text("driver_close_jaw");
    c.service_standby    = topics.text("driver_standby");
    return c;
}

}  // namespace task
