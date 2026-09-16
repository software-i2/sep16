// Copyright by BeeX [2026]

#ifndef VIZ_VIZNODE_H
#define VIZ_VIZNODE_H

#include <bx_msgs/RosBindings.hpp>
#include <kine/ArmBody.h>
#include <msgs/CloudResult.h>
#include <msgs/GraspPlan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <viz/VizConfig.h>

#include <mutex>

namespace viz {

// The only node that draws. Everything it shows comes from topics the other nodes already publish.
class VizNode {
public:
    // Throws std::invalid_argument when the arm configuration is not usable.
    explicit VizNode(const VizConfig &config);

    // Redraws the collision body at the latest joints. Call at arm_body_rate_hz.
    void drawArmBody();

private:
    void onJointStates(const sensor_msgs::JointState::ConstPtr &msg);
    void onCloudResult(const msgs::CloudResult::ConstPtr &msg);
    void onPlanResult(const msgs::GraspPlan::ConstPtr &msg);

    void drawFloor();
    bool modelJoints(const std::vector<std::string> &names, const std::vector<double> &positions,
                     kine::JointAngles &out) const;
    visualization_msgs::Marker marker(const std::string &name, int type, const std_msgs::ColorRGBA &colour) const;

    VizConfig     config_;
    kine::ArmBody body_;

    std::mutex              joints_mutex_;
    sensor_msgs::JointState latest_joints_;

    DECLARE_ROS_SUBSCRIBER(sub_joint_states_, sensor_msgs::JointState)
    DECLARE_ROS_SUBSCRIBER(sub_cloud_result_, msgs::CloudResult)
    DECLARE_ROS_SUBSCRIBER(sub_plan_result_, msgs::GraspPlan)
    DECLARE_ROS_PUBLISHER(pub_arm_body_, visualization_msgs::MarkerArray)
    DECLARE_ROS_PUBLISHER(pub_floor_, visualization_msgs::Marker)
    DECLARE_ROS_PUBLISHER(pub_obstacle_map_, sensor_msgs::PointCloud2)
    DECLARE_ROS_PUBLISHER(pub_grasp_poses_, visualization_msgs::Marker)
    DECLARE_ROS_PUBLISHER(pub_chosen_grasp_, visualization_msgs::MarkerArray)
    DECLARE_ROS_PUBLISHER(pub_planned_path_, visualization_msgs::Marker)
};

}  // namespace viz

#endif  // VIZ_VIZNODE_H
