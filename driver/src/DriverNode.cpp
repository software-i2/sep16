// Copyright by BeeX [2026]

#include <driver/DriverNode.h>

#include <algorithm>
#include <cmath>

namespace driver {

DriverNode::DriverNode(const DriverConfig &config, std::shared_ptr<ArmLink> arm)
        : config_(config), arm_(std::move(arm)) {
    INIT_ROS_PUBLISHER(pub_joint_states_, sensor_msgs::JointState, config_.topic_joint_states, 1);
    INIT_ROS_SUBSCRIBER(sub_joint_targets_, config_.topic_joint_targets, 1, &DriverNode::onJointTargets);
    INIT_ROS_SERVICE_SERVER(srv_home_, config_.service_home, &DriverNode::onHome);
    INIT_ROS_SERVICE_SERVER(srv_open_jaw_, config_.service_open_jaw, &DriverNode::onOpenJaw);
    INIT_ROS_SERVICE_SERVER(srv_close_jaw_, config_.service_close_jaw, &DriverNode::onCloseJaw);
    INIT_ROS_SERVICE_SERVER(srv_standby_, config_.service_standby, &DriverNode::onStandby);
}

void DriverNode::poll() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int j = 0; j < JOINT_COUNT; ++j) {
        float wire = 0.0f;
        if (arm_->readPosition(config_.joints[j].device_id, wire)) {
            position_[j] = wire / config_.joints[j].wire_per_unit;
            read_at_[j]  = ros::Time::now();
        }
    }
    if (homing_) {
        stepHome();
    }
    publishJointStates();
}

void DriverNode::stepHome() {
    bool still_moving = false;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        const JointSetup &joint  = config_.joints[j];
        const double      step   = joint.home_speed / config_.poll_rate_hz;
        const double      gap    = joint.home - position_[j];
        const double      target = std::fabs(gap) <= step ? joint.home : position_[j] + std::copysign(step, gap);
        still_moving |= std::fabs(gap) > step;
        arm_->setPosition(joint.device_id, static_cast<float>(target * joint.wire_per_unit));
    }
    homing_ = still_moving;
    if (!homing_) {
        LOG_INFO("[driver] home reached");
    }
}

// Stamped with the oldest reading it carries, so a joint that stopped answering shows up as stale.
void DriverNode::publishJointStates() {
    ros::Time oldest = read_at_[0];
    for (int j = 1; j < JOINT_COUNT; ++j) {
        oldest = std::min(oldest, read_at_[j]);
    }
    if (oldest.isZero()) {
        if (!warned_silent_) {
            warned_silent_ = true;
            LOG_WARN("[driver] not every joint has answered yet, so joint_states is not published");
        }
        return;
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = oldest;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        msg.name.push_back(config_.joints[j].name);
        msg.position.push_back(position_[j]);
    }
    PUBLISH_ROS(pub_joint_states_, msg);
}

bool DriverNode::moveJoint(int joint, double position, std::string &why) {
    const JointSetup &setup = config_.joints[joint];
    if (!std::isfinite(position) || position < setup.min || position > setup.max) {
        why = setup.name + " target " + std::to_string(position) + " is outside [" + std::to_string(setup.min) + ", "
              + std::to_string(setup.max) + "]";
        return false;
    }
    homing_ = false;
    if (!arm_->setPosition(setup.device_id, static_cast<float>(position * setup.wire_per_unit))) {
        why = "the command to " + setup.name + " could not be written";
        return false;
    }
    return true;
}

// Either every target in the message is legal and sent, or none is.
void DriverNode::onJointTargets(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->name.size() != msg->position.size()) {
        LOG_WARN("[driver] joint targets dropped: %zu names but %zu positions", msg->name.size(), msg->position.size());
        return;
    }

    std::vector<int> joints;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const auto found = std::find_if(config_.joints.begin(), config_.joints.end(),
                                        [&](const JointSetup &joint) { return joint.name == msg->name[i]; });
        if (found == config_.joints.end()) {
            LOG_WARN("[driver] joint targets dropped: no joint called %s", msg->name[i].c_str());
            return;
        }
        const JointSetup &joint = *found;
        if (!std::isfinite(msg->position[i]) || msg->position[i] < joint.min || msg->position[i] > joint.max) {
            LOG_WARN("[driver] joint targets dropped: %s target %.4f is outside [%.4f, %.4f]", joint.name.c_str(),
                     msg->position[i], joint.min, joint.max);
            return;
        }
        joints.push_back(static_cast<int>(found - config_.joints.begin()));
    }

    std::string why;
    for (size_t i = 0; i < joints.size(); ++i) {
        if (!moveJoint(joints[i], msg->position[i], why)) {
            LOG_WARN("[driver] %s", why.c_str());
        }
    }
}

bool DriverNode::onHome(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int j = 0; j < JOINT_COUNT; ++j) {
        if (read_at_[j].isZero()) {
            res.success = false;
            res.message = config_.joints[j].name + " has not reported a position yet";
            return true;
        }
    }
    homing_     = true;
    res.success = true;
    res.message = "moving home with the jaw open";
    LOG_INFO("[driver] %s", res.message.c_str());
    return true;
}

bool DriverNode::onOpenJaw(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res) {
    std::lock_guard<std::mutex> lock(mutex_);
    res.success = moveJoint(JAW, config_.jaw_open_width, res.message);
    return true;
}

bool DriverNode::onCloseJaw(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res) {
    std::lock_guard<std::mutex> lock(mutex_);
    res.success = moveJoint(JAW, config_.joints[JAW].min, res.message);
    return true;
}

bool DriverNode::onStandby(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res) {
    std::lock_guard<std::mutex> lock(mutex_);
    homing_     = false;
    res.success = true;
    for (const JointSetup &joint : config_.joints) {
        res.success &= arm_->setStandby(joint.device_id);
    }
    res.message = res.success ? "every joint released" : "a standby command could not be written";
    if (res.success) {
        LOG_INFO("[driver] %s", res.message.c_str());
    } else {
        LOG_ERROR("[driver] %s", res.message.c_str());
    }
    return true;
}

}  // namespace driver
