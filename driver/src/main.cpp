// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <driver/ArmLink.h>
#include <driver/DriverNode.h>
#include <driver/SerialPort.h>
#include <driver/SimulatedArm.h>

#include <chrono>
#include <thread>

DECLARE_ROS_NODE_HANDLE

namespace {

std::shared_ptr<driver::BytePort> openPort(const driver::DriverConfig &config) {
    if (config.simulated) {
        std::vector<driver::SimulatedArm::Joint> joints;
        for (const driver::JointSetup &joint : config.joints) {
            joints.push_back({joint.device_id, static_cast<float>(joint.min * joint.wire_per_unit),
                              static_cast<float>(joint.max * joint.wire_per_unit),
                              static_cast<float>(joint.home * joint.wire_per_unit),
                              static_cast<float>(joint.simulated_speed * joint.wire_per_unit)});
        }
        return std::make_shared<driver::SimulatedArm>(joints);
    }

    auto serial = std::make_shared<driver::SerialPort>(config.serial_port, config.baud_rate);
    if (!serial->isOpen()) {
        LOG_ERROR("[driver] %s", serial->openError().c_str());
        return nullptr;
    }
    return serial;
}

// The arm sleeps until it sees traffic: nudge the wrist and wait for it to report velocity mode.
bool wakeArm(driver::ArmLink &arm, const driver::DriverConfig &config) {
    const uint8_t wrist = config.joints[driver::WRIST].device_id;
    for (int attempt = 1; attempt <= config.connect_attempts; ++attempt) {
        uint8_t mode = 0;
        arm.setVelocity(wrist, static_cast<float>(config.connect_nudge_speed * driver::WIRE_UNITS_PER_RADIAN));
        if (arm.readMode(wrist, mode) && mode == driver::mode::VELOCITY) {
            return true;
        }
        LOG_WARN("[driver] no answer from the arm, attempt %d of %d", attempt, config.connect_attempts);
        std::this_thread::sleep_for(std::chrono::duration<double>(config.connect_retry_s));
    }
    return false;
}

void releaseAll(driver::ArmLink &arm, const driver::DriverConfig &config) {
    for (const driver::JointSetup &joint : config.joints) {
        arm.setStandby(joint.device_id);
    }
}

}  // namespace

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    INIT_ROS_NODE("driver", 0, "driver/alive")

    params::Params driver_params("/driver");
    params::Params arm_params("/arm");
    params::Params jaws_params("/jaws");
    params::Params topics_params("/topics");
    const driver::DriverConfig config =
            driver::loadDriverConfig(driver_params, arm_params, jaws_params, topics_params);

    const std::string problems = driver_params.errors() + driver_params.unreadKeys() + arm_params.errors()
                                 + jaws_params.errors() + topics_params.errors();
    if (!problems.empty()) {
        LOG_ERROR("[driver] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    const std::shared_ptr<driver::BytePort> port = openPort(config);
    if (!port) {
        return 1;
    }
    const auto arm = std::make_shared<driver::ArmLink>(port, config.reply_timeout_s);
    if (!wakeArm(*arm, config)) {
        LOG_ERROR("[driver] the arm did not answer on %s", config.serial_port.c_str());
        return 1;
    }
    releaseAll(*arm, config);

    driver::DriverNode node(config, arm);
    LOG_INFO("[driver] talking to %s at %.1f Hz", config.simulated ? "the simulated arm" : config.serial_port.c_str(),
             config.poll_rate_hz);

    ROS_ASYNC_SPIN(2)
    ros::Rate rate(config.poll_rate_hz);
    while (IS_ROS_NODE_OK()) {
        node.poll();
        rate.sleep();
    }

    releaseAll(*arm, config);
    ROS_SHUTDOWN();
    return 0;
}
