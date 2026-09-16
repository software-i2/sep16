// Copyright by BeeX [2026]

#ifndef EXECUTOR_EXECUTORNODE_H
#define EXECUTOR_EXECUTORNODE_H

#include <actionlib/server/simple_action_server.h>
#include <bx_msgs/RosBindings.hpp>
#include <executor/PathFollower.h>
#include <msgs/ExecuteAction.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <mutex>

namespace executor {

// Serves the execute action: drives the arm along a joint path and releases it on any fault.
class ExecutorNode {
public:
    explicit ExecutorNode(const ExecutorConfig &config);

private:
    void onJointStates(const sensor_msgs::JointState::ConstPtr &msg);
    void onExecute(const msgs::ExecuteGoalConstPtr &goal);

    bool readPath(const msgs::JointPath &path, std::vector<kine::JointAngles> &corners, std::string &why) const;
    bool freshJoints(kine::JointAngles &joints);
    void sendTarget(const kine::JointAngles &target);
    void releaseArm();
    void finish(uint8_t outcome, const std::string &message);

    ExecutorConfig                                     config_;
    actionlib::SimpleActionServer<msgs::ExecuteAction> server_;

    std::mutex              joints_mutex_;
    sensor_msgs::JointState latest_joints_;

    DECLARE_ROS_SUBSCRIBER(sub_joint_states_, sensor_msgs::JointState)
    DECLARE_ROS_PUBLISHER(pub_joint_targets_, sensor_msgs::JointState)
    DECLARE_ROS_SERVICE_CLIENT(cli_standby_, std_srvs::Trigger)
};

}  // namespace executor

#endif  // EXECUTOR_EXECUTORNODE_H
