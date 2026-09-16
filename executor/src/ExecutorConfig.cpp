// Copyright by BeeX [2026]

#include <executor/ExecutorConfig.h>

namespace executor {

ExecutorConfig loadExecutorConfig(params::Params &executor, params::Params &arm, params::Params &topics) {
    ExecutorConfig c;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        c.joint_names[j] = arm.text(std::string("joint_names/") + kine::JOINT_KEYS[j]);
    }

    c.rate_hz            = executor.number("rate_hz");
    c.start_tolerance    = kine::degToRad(executor.number("start_tolerance_deg"));
    c.feedback_timeout_s = executor.number("feedback_timeout_s");

    FollowSettings &f             = c.follow;
    f.max_joint_step              = kine::degToRad(executor.number("max_joint_step_deg"));
    f.arrival_tolerance           = kine::degToRad(executor.number("arrival_tolerance_deg"));
    f.arrival_timeout_s           = executor.number("arrival_timeout_s");
    f.blocked_min_commanded_step  = kine::degToRad(executor.number("blocked_detection/min_commanded_step_deg"));
    f.blocked_min_follow_fraction = executor.number("blocked_detection/min_follow_fraction");
    f.blocked_strikes             = executor.whole("blocked_detection/strikes");

    executor.require(c.rate_hz > 0.0, "rate_hz", "positive");
    executor.require(c.start_tolerance >= 0.0, "start_tolerance_deg", "zero or more");
    executor.require(c.feedback_timeout_s > 0.0, "feedback_timeout_s", "positive");
    executor.require(f.max_joint_step > 0.0, "max_joint_step_deg", "positive");
    executor.require(f.arrival_tolerance > 0.0, "arrival_tolerance_deg", "positive");
    executor.require(f.arrival_timeout_s > 0.0, "arrival_timeout_s", "positive");
    executor.require(f.blocked_min_commanded_step > 0.0, "blocked_detection/min_commanded_step_deg", "positive");
    executor.require(f.blocked_min_follow_fraction > 0.0, "blocked_detection/min_follow_fraction", "positive");
    executor.require(f.blocked_strikes > 0, "blocked_detection/strikes", "positive");

    c.topic_joint_states     = topics.text("joint_states");
    c.topic_joint_targets    = topics.text("joint_targets");
    c.action_execute         = topics.text("executor_execute");
    c.service_driver_standby = topics.text("driver_standby");
    return c;
}

}  // namespace executor
