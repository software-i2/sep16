// Copyright by BeeX [2026]

#include <executor/ExecutorNode.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>

namespace executor {

ExecutorNode::ExecutorNode(const ExecutorConfig &config)
        : config_(config),
          server_(*mainNodeHandle, config.action_execute, boost::bind(&ExecutorNode::onExecute, this, _1), false) {
    INIT_ROS_SUBSCRIBER(sub_joint_states_, config_.topic_joint_states, 1, &ExecutorNode::onJointStates);
    INIT_ROS_PUBLISHER(pub_joint_targets_, sensor_msgs::JointState, config_.topic_joint_targets, 1);
    INIT_ROS_SERVICE_CLIENT(cli_standby_, std_srvs::Trigger, config_.service_driver_standby);
    server_.start();
}

void ExecutorNode::onJointStates(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    latest_joints_ = *msg;
}

bool ExecutorNode::freshJoints(kine::JointAngles &joints) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    if (latest_joints_.name.empty()
        || (ros::Time::now() - latest_joints_.header.stamp).toSec() > config_.feedback_timeout_s) {
        return false;
    }
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        const auto found = std::find(latest_joints_.name.begin(), latest_joints_.name.end(), config_.joint_names[j]);
        const size_t index = static_cast<size_t>(found - latest_joints_.name.begin());
        if (found == latest_joints_.name.end() || index >= latest_joints_.position.size()) {
            return false;
        }
        joints[j] = latest_joints_.position[index];
    }
    return true;
}

// Puts the path's columns into this node's joint order.
bool ExecutorNode::readPath(const msgs::JointPath &path, std::vector<kine::JointAngles> &corners,
                            std::string &why) const {
    std::array<size_t, kine::JOINT_COUNT> column{};
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        const auto found = std::find(path.joint_names.begin(), path.joint_names.end(), config_.joint_names[j]);
        if (found == path.joint_names.end()) {
            why = "the path has no " + config_.joint_names[j];
            return false;
        }
        column[j] = static_cast<size_t>(found - path.joint_names.begin());
    }
    if (path.waypoints.empty()) {
        why = "the path is empty";
        return false;
    }
    for (const msgs::Waypoint &waypoint : path.waypoints) {
        if (waypoint.positions.size() != path.joint_names.size()) {
            why = "a waypoint has the wrong number of positions";
            return false;
        }
        kine::JointAngles corner;
        for (int j = 0; j < kine::JOINT_COUNT; ++j) {
            corner[j] = waypoint.positions[column[j]];
            if (!std::isfinite(corner[j])) {
                why = "a waypoint is not finite";
                return false;
            }
        }
        corners.push_back(corner);
    }
    return true;
}

void ExecutorNode::sendTarget(const kine::JointAngles &target) {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name.assign(config_.joint_names.begin(), config_.joint_names.end());
    msg.position.assign(target.begin(), target.end());
    PUBLISH_ROS(pub_joint_targets_, msg);
}

void ExecutorNode::releaseArm() {
    std_srvs::Trigger srv;
    if (!CALL_SRV_ROS(cli_standby_, srv)) {
        LOG_ERROR("[executor] %s did not answer; the arm may still be holding its last target",
                  config_.service_driver_standby.c_str());
    }
}

void ExecutorNode::finish(uint8_t outcome, const std::string &message) {
    msgs::ExecuteResult result;
    result.outcome = outcome;
    result.message = message;
    if (outcome == msgs::ExecuteResult::REACHED) {
        LOG_INFO("[executor] %s", message.c_str());
        server_.setSucceeded(result, message);
    } else if (outcome == msgs::ExecuteResult::STOPPED) {
        LOG_WARN("[executor] %s", message.c_str());
        server_.setPreempted(result, message);
    } else {
        LOG_ERROR("[executor] %s", message.c_str());
        server_.setAborted(result, message);
    }
}

void ExecutorNode::onExecute(const msgs::ExecuteGoalConstPtr &goal) {
    std::vector<kine::JointAngles> corners;
    std::string                    why;
    if (!readPath(goal->path, corners, why)) {
        finish(msgs::ExecuteResult::REFUSED, "refused: " + why);
        return;
    }

    kine::JointAngles joints;
    if (!freshJoints(joints)) {
        finish(msgs::ExecuteResult::REFUSED, "refused: no fresh joint_states");
        return;
    }
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        if (std::fabs(joints[j] - corners.front()[j]) > config_.start_tolerance) {
            finish(msgs::ExecuteResult::REFUSED,
                   "refused: the arm is not where the path starts (" + config_.joint_names[j] + " is "
                           + std::to_string(kine::radToDeg(joints[j] - corners.front()[j])) + " deg off)");
            return;
        }
    }

    PathFollower follower(config_.follow);
    follower.load(corners);
    LOG_INFO("[executor] following %zu waypoints", follower.waypointCount());

    msgs::ExecuteFeedback feedback;
    feedback.waypoint_count = static_cast<uint32_t>(follower.waypointCount());
    ros::Rate rate(config_.rate_hz);

    while (ros::ok()) {
        if (server_.isPreemptRequested()) {
            releaseArm();
            finish(msgs::ExecuteResult::STOPPED, "stopped on request, arm released");
            return;
        }

        if (freshJoints(joints)) {
            follower.measure(joints);
        } else {
            follower.loseMeasurement();
        }

        kine::JointAngles target;
        bool              send  = false;
        const FollowState state = follower.tick(ros::Time::now().toSec(), target, send);
        if (send) {
            sendTarget(target);
        }

        feedback.waypoint = static_cast<uint32_t>(follower.sent());
        server_.publishFeedback(feedback);

        if (state == FollowState::REACHED) {
            finish(msgs::ExecuteResult::REACHED, "reached the end of the path");
            return;
        }
        if (state == FollowState::STALLED) {
            releaseArm();
            finish(msgs::ExecuteResult::STALLED, "did not arrive within the arrival timeout, arm released");
            return;
        }
        if (state == FollowState::BLOCKED) {
            releaseArm();
            finish(msgs::ExecuteResult::BLOCKED, config_.joint_names[follower.blockedJoint()]
                                                         + " stopped following, so the arm hit something; arm released");
            return;
        }
        rate.sleep();
    }
    finish(msgs::ExecuteResult::STOPPED, "shutting down");
}

}  // namespace executor
