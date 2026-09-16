// Copyright by BeeX [2026]

#ifndef DRIVER_DRIVERNODE_H
#define DRIVER_DRIVERNODE_H

#include <bx_msgs/RosBindings.hpp>
#include <driver/ArmLink.h>
#include <driver/DriverConfig.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <mutex>

namespace driver {

class DriverNode {
public:
    DriverNode(const DriverConfig &config, std::shared_ptr<ArmLink> arm);

    // Reads every joint, steps the home ramp, publishes joint_states. Call at poll_rate_hz.
    void poll();

private:
    void onJointTargets(const sensor_msgs::JointState::ConstPtr &msg);
    bool onHome(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onOpenJaw(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onCloseJaw(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onStandby(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool moveJoint(int joint, double position, std::string &why);
    void stepHome();
    void publishJointStates();

    DriverConfig             config_;
    std::shared_ptr<ArmLink> arm_;
    std::mutex               mutex_;

    std::array<double, JOINT_COUNT>    position_{};
    std::array<ros::Time, JOINT_COUNT> read_at_{};
    bool                               homing_        = false;
    bool                               warned_silent_ = false;

    DECLARE_ROS_PUBLISHER(pub_joint_states_, sensor_msgs::JointState)
    DECLARE_ROS_SUBSCRIBER(sub_joint_targets_, sensor_msgs::JointState)
    DECLARE_ROS_SERVICE_SERVER(srv_home_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_open_jaw_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_close_jaw_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_standby_, std_srvs::Trigger)
};

}  // namespace driver

#endif  // DRIVER_DRIVERNODE_H
