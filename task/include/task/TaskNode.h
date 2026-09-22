// Copyright by BeeX [2026]

#ifndef TASK_TASKNODE_H
#define TASK_TASKNODE_H

#include <actionlib/client/simple_action_client.h>
#include <bx_msgs/RosBindings.hpp>
#include <msgs/CollectAction.h>
#include <msgs/ExecuteAction.h>
#include <msgs/ParkAction.h>
#include <msgs/PlanAction.h>
#include <msgs/TaskState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <task/StateMachine.h>
#include <task/TaskConfig.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <mutex>
#include <string>

namespace task {

// Runs the pick after one task/start: wait for the camera, survey, park, survey again from
// where the vehicle ended up, plan, execute, close the jaw.
class TaskNode {
public:
    explicit TaskNode(const TaskConfig &config);

    // Checks the running step and moves the state machine on. Call at rate_hz.
    void tick();

private:
    // Which of the two survey passes is running. COLLECT and PROCESS behave the same in both;
    // only what they hand on afterwards differs.
    enum class Phase { SPOT, GRASP };

    bool onStart(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void onJointStates(const sensor_msgs::JointState::ConstPtr &msg);
    void onCameraCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void onCollectFeedback(const msgs::CollectFeedbackConstPtr &feedback);
    void onParkFeedback(const msgs::ParkFeedbackConstPtr &feedback);

    void apply(Event event, const std::string &message);
    void enter(State state, const std::string &message);
    void publishState(const std::string &message);

    Event checkStream(std::string &message);
    Event checkCollect(std::string &message);
    Event checkResurvey(std::string &message);
    Event checkPark(std::string &message);
    Event checkPlan(std::string &message);
    Event checkExecute(std::string &message);
    Event checkJaw(std::string &message);
    Event checkRetarget(std::string &message);
    Event checkRepark(std::string &message);

    // How far the pre-move snapshot turned out to be wrong: the dead reckoning of the drive
    // plus whatever the scene did while it was under way. Empty when there is nothing to say.
    std::string comparedWithSnapshot();
    std::string freshMapExtent() const;

    bool callTrigger(ros::ServiceClient &client, const std::string &name, std::string &why);

    TaskConfig config_;

    std::mutex        mutex_;
    State             state_         = State::READY;
    Phase             phase_         = Phase::SPOT;
    ros::Time         entered_at_;
    ros::Time         run_started_;
    ros::Time         surveying_since_;  // start of the current streak of empty surveys; zero when there is none
    ros::Time         reparking_since_;  // start of the current streak of reparks that never moved the vehicle
    State             left_             = State::READY;  // the state just finished, and how long it ran, for the log
    double            left_after_       = -1.0;
    uint32_t          park_attempt_  = 0;
    uint32_t          look_attempt_  = 0;
    bool              grabbed_       = false;
    msgs::CloudResult cloud_;
    msgs::CloudResult locked_;
    msgs::GraspPlan   plan_;
    std::atomic<bool> processing_{false};
    std::atomic<bool> driving_{false};

    std::mutex jaw_mutex_;
    double     jaw_position_ = 0.0;
    ros::Time  jaw_stamp_;
    ros::Time  jaw_closed_at_;
    ros::Time  jaw_last_seen_;
    double     jaw_still_position_ = 0.0;
    ros::Time  jaw_still_since_;

    std::mutex frame_mutex_;
    ros::Time  frame_seen_at_;

    actionlib::SimpleActionClient<msgs::CollectAction> collect_;
    actionlib::SimpleActionClient<msgs::ParkAction>    park_;
    actionlib::SimpleActionClient<msgs::PlanAction>    planner_;
    actionlib::SimpleActionClient<msgs::ExecuteAction> executor_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    DECLARE_ROS_SERVICE_SERVER(srv_start_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_stop_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_CLIENT(cli_close_jaw_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_CLIENT(cli_standby_, std_srvs::Trigger)
    DECLARE_ROS_SUBSCRIBER(sub_joint_states_, sensor_msgs::JointState)
    DECLARE_ROS_SUBSCRIBER(sub_camera_cloud_, sensor_msgs::PointCloud2)
    DECLARE_ROS_PUBLISHER(pub_state_, msgs::TaskState)
};

}  // namespace task

#endif  // TASK_TASKNODE_H
