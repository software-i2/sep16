// Copyright by BeeX [2026]

#ifndef PARK_PARKNODE_H
#define PARK_PARKNODE_H

#include <actionlib/server/simple_action_server.h>
#include <bx_msgs/RosBindings.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <kine/Reach.h>
#include <msgs/CloudResult.h>
#include <msgs/ParkAction.h>
#include <msgs/ParkPose.h>
#include <park/ParkConfig.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <planner/GraspGoals.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <mutex>

namespace park {

// Serves the park action: searches the bounded box for a body pose that brings the locked
// scene into the arm's reach, then walks the vehicle there.
//
// The vehicle is simulated: the node owns locked_frame -> body_frame and drives it. A real
// vehicle replaces that with a setpoint client and this broadcast goes away.
class ParkNode {
public:
    explicit ParkNode(const ParkConfig &config);

private:
    void onPark(const msgs::ParkGoalConstPtr &goal);
    void broadcast(const Pose &pose, const ros::Time &stamp);
    bool walkTo(const Pose &target, std::string &why);

    // The scene frame is where the arm base stood when the snapshot was taken. It is latched
    // once and then left alone: the vehicle moves underneath it, which is the whole point.
    void lockScene(const Pose &at);

    // How many of the candidates the arm could actually hold from `pose`, body and all,
    // against the obstacle map the cloud came with. The map stays where it was built and the
    // grid is told where the arm would be standing instead.
    struct Verdict {
        int held     = 0;  // candidates the arm could hold from here, body and all
        int routable = 0;  // of those, the ones a straight joint-space line from home reaches
    };

    Verdict verify(const std::vector<Grasp> &grasps, const Pose &pose, planner::ObstacleGrid &grid,
                   const kine::JointAngles &start, size_t blade_stride, bool check_routes) const;

    // Whether the arm, held as it is now, stays clear of the scene for the whole drive. The
    // vehicle carries the arm with it, so a pose that is fine to stand in is worth nothing if
    // getting there rakes the arm through the mine.
    bool transitClear(const Pose &from, const Pose &to, planner::ObstacleGrid &grid,
                      const kine::JointAngles &held, size_t blade_stride) const;

    void onJointStates(const sensor_msgs::JointState::ConstPtr &msg);

    // Puts the vehicle back where it started and forgets the lock, so a soak run can repeat a
    // pick from the same place instead of drifting away over the session.
    bool onReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    Eigen::Isometry3d queryToMap(const Pose &pose) const;

    // The arm as it stands, in model radians, or the configured home when nothing has arrived.
    kine::JointAngles currentJoints() const;

    // /tf is not latched, so the body and scene transforms have to keep being sent or they
    // age out of every listener's buffer and the arm loses its way back to the world.
    void onTick(const ros::TimerEvent &);

    msgs::ParkPose toMessage(const Scored &scored, size_t offered) const;

    ParkConfig                                      config_;
    kine::ReachTable                                table_;
    Eigen::Isometry3d                               body_to_arm_;
    std::unique_ptr<PoseSearch>                     search_;
    std::unique_ptr<kine::ArmBody>                  body_;
    actionlib::SimpleActionServer<msgs::ParkAction> server_;
    tf2_ros::TransformBroadcaster                   broadcaster_;
    ros::Timer                                      tick_;

    std::mutex where_mutex_;
    Pose       where_;  // the body's pose in locked_frame, as far as this node has driven it

    mutable std::mutex      joints_mutex_;
    sensor_msgs::JointState latest_joints_;

    std::mutex        scene_mutex_;
    bool              scene_locked_ = false;
    Eigen::Isometry3d locked_to_scene_ = Eigen::Isometry3d::Identity();

    DECLARE_ROS_SERVICE_SERVER(srv_reset_, std_srvs::Trigger)
    DECLARE_ROS_SUBSCRIBER(sub_joint_states_, sensor_msgs::JointState)
    DECLARE_ROS_PUBLISHER(pub_result_, msgs::ParkPose)
    DECLARE_ROS_PUBLISHER(pub_locked_cloud_, msgs::CloudResult)
};

}  // namespace park

#endif  // PARK_PARKNODE_H
