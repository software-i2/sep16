// Copyright by BeeX [2026]

#ifndef CLOUD_CLOUDNODE_H
#define CLOUD_CLOUDNODE_H

#include <actionlib/server/simple_action_server.h>
#include <bx_msgs/RosBindings.hpp>
#include <cloud/CloudPipeline.h>
#include <geometry_msgs/PoseArray.h>
#include <msgs/CloudResult.h>
#include <msgs/CollectAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <mutex>

namespace cloud {

// Serves the collect action: keeps a rolling buffer of the newest camera frames, then runs the pipeline.
class CloudNode {
public:
    CloudNode(const CloudConfig &config, const CloudPipeline &pipeline);

private:
    void onCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void onGraspPoses(const geometry_msgs::PoseArray::ConstPtr &msg);
    void onCollect(const msgs::CollectGoalConstPtr &goal);

    // Pairs the latest cloud and pose array once their stamps match.
    void pairLatest();
    bool toFrame(const sensor_msgs::PointCloud2 &cloud, const geometry_msgs::PoseArray &poses, CameraFrame &frame,
                 std::string &why);
    msgs::CloudResult toMessage(const CloudOutput &output, size_t frames_used) const;
    void              finishFailed(const std::string &why);

    CloudConfig                                        config_;
    const CloudPipeline                               &pipeline_;
    actionlib::SimpleActionServer<msgs::CollectAction> server_;
    tf2_ros::Buffer                                    tf_buffer_;
    tf2_ros::TransformListener                         tf_listener_;

    std::mutex                         frames_mutex_;
    sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
    geometry_msgs::PoseArray::ConstPtr latest_poses_;
    std::deque<CameraFrame>            frames_;
    size_t                             arrived_since_window_ = 0;
    ros::Time                          window_served_;
    bool                               subscribed_ = false;
    std::string                        frame_problem_;

    DECLARE_ROS_SUBSCRIBER(sub_cloud_, sensor_msgs::PointCloud2)
    DECLARE_ROS_SUBSCRIBER(sub_grasp_poses_, geometry_msgs::PoseArray)
    DECLARE_ROS_PUBLISHER(pub_result_, msgs::CloudResult)
};

}  // namespace cloud

#endif  // CLOUD_CLOUDNODE_H
