// Copyright by BeeX [2026]

#include <driver/DriverConfig.h>
#include <driver/Protocol.h>

#include <cmath>

namespace driver {

DriverConfig loadDriverConfig(params::Params &driver, params::Params &arm, params::Params &jaws,
                              params::Params &topics) {
    DriverConfig c;

    c.simulated       = driver.flag("simulated");
    c.serial_port     = driver.text("serial_port");
    c.baud_rate       = driver.whole("baud_rate");
    c.poll_rate_hz    = driver.number("poll_rate_hz");
    c.reply_timeout_s = driver.number("reply_timeout_s");
    c.connect_attempts    = driver.whole("connect_attempts");
    c.connect_retry_s     = driver.number("connect_retry_s");
    c.connect_nudge_speed = driver.number("connect_nudge_speed_deg_s") * M_PI / 180.0;

    const double home_joint_speed      = driver.number("home_joint_speed_deg_s") * M_PI / 180.0;
    const double home_jaw_speed        = driver.number("home_jaw_speed_m_s");
    const double simulated_joint_speed = driver.number("simulated_arm/joint_speed_deg_s") * M_PI / 180.0;
    const double simulated_jaw_speed   = driver.number("simulated_arm/jaw_speed_m_s");

    driver.require(c.poll_rate_hz > 0.0, "poll_rate_hz", "positive");
    driver.require(c.reply_timeout_s > 0.0, "reply_timeout_s", "positive");
    driver.require(c.connect_attempts > 0, "connect_attempts", "positive");
    driver.require(home_joint_speed > 0.0, "home_joint_speed_deg_s", "positive");
    driver.require(home_jaw_speed > 0.0, "home_jaw_speed_m_s", "positive");
    driver.require(simulated_joint_speed > 0.0, "simulated_arm/joint_speed_deg_s", "positive");
    driver.require(simulated_jaw_speed > 0.0, "simulated_arm/jaw_speed_m_s", "positive");

    for (int j = 0; j < JOINT_COUNT; ++j) {
        const std::string key = JOINT_KEYS[j];
        JointSetup       &joint = c.joints[j];
        joint.name              = arm.text("joint_names/" + key);

        const int device_id = driver.whole("device_ids/" + key);
        driver.require(device_id > 0 && device_id < 256, "device_ids/" + key, "between 1 and 255");
        joint.device_id = static_cast<uint8_t>(device_id);

        if (j == JAW) {
            c.jaw_open_width      = jaws.number("open_width_m");
            joint.min             = arm.number("jaw_limits_m/min");
            joint.max             = arm.number("jaw_limits_m/max");
            joint.home            = c.jaw_open_width;
            joint.home_speed      = home_jaw_speed;
            joint.simulated_speed = simulated_jaw_speed;
            joint.wire_per_unit   = WIRE_UNITS_PER_METRE;
        } else {
            joint.min             = arm.number("joint_limits_deg/" + key + "/min") * M_PI / 180.0;
            joint.max             = arm.number("joint_limits_deg/" + key + "/max") * M_PI / 180.0;
            joint.home            = arm.number("home_deg/" + key) * M_PI / 180.0;
            joint.home_speed      = home_joint_speed;
            joint.simulated_speed = simulated_joint_speed;
            joint.wire_per_unit   = WIRE_UNITS_PER_RADIAN;
        }
        arm.require(joint.min < joint.max, "limits of " + key, "min below max");
        arm.require(joint.home >= joint.min && joint.home <= joint.max, "home of " + key, "inside its limits");
    }

    c.topic_joint_states  = topics.text("joint_states");
    c.topic_joint_targets = topics.text("joint_targets");
    c.service_home        = topics.text("driver_home");
    c.service_open_jaw    = topics.text("driver_open_jaw");
    c.service_close_jaw   = topics.text("driver_close_jaw");
    c.service_standby     = topics.text("driver_standby");
    return c;
}

}  // namespace driver
