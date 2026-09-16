// Copyright by BeeX [2026]

#ifndef TASK_TASKNODE_H
#define TASK_TASKNODE_H

#include <actionlib/client/simple_action_client.h>
#include <bx_msgs/RosBindings.hpp>
#include <msgs/CollectAction.h>
#include <msgs/ExecuteAction.h>
#include <msgs/PlanAction.h>
#include <msgs/TaskState.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <task/StateMachine.h>
#include <task/TaskConfig.h>

#include <atomic>
#include <mutex>

namespace task {

// Runs the pick after one task/start: collect, process, plan, execute, close the jaw.
class TaskNode {
public:
    explicit TaskNode(const TaskConfig &config);

    // Checks the running step and moves the state machine on. Call at rate_hz.
    void tick();

private:
    bool onStart(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void onJointStates(const sensor_msgs::JointState::ConstPtr &msg);
    void onCollectFeedback(const msgs::CollectFeedbackConstPtr &feedback);

    void apply(Event event, const std::string &message);
    void enter(State state, const std::string &message);
    void publishState(const std::string &message);

    Event checkCollect(std::string &message);
    Event checkPlan(std::string &message);
    Event checkExecute(std::string &message);
    Event checkJaw(std::string &message);
    Event checkRetry(std::string &message);

    bool callTrigger(ros::ServiceClient &client, const std::string &name, std::string &why);

    TaskConfig config_;

    std::mutex        mutex_;
    State             state_      = State::IDLE;
    ros::Time         entered_at_;
    uint32_t          attempt_    = 0;
    bool              grabbed_    = false;
    msgs::CloudResult cloud_;
    msgs::GraspPlan   plan_;
    std::atomic<bool> processing_{false};

    std::mutex jaw_mutex_;
    double     jaw_position_ = 0.0;
    ros::Time  jaw_stamp_;
    ros::Time  jaw_closed_at_;
    ros::Time  jaw_last_seen_;
    double     jaw_still_position_ = 0.0;
    ros::Time  jaw_still_since_;

    actionlib::SimpleActionClient<msgs::CollectAction> collect_;
    actionlib::SimpleActionClient<msgs::PlanAction>    planner_;
    actionlib::SimpleActionClient<msgs::ExecuteAction> executor_;

    DECLARE_ROS_SERVICE_SERVER(srv_start_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_stop_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_CLIENT(cli_close_jaw_, std_srvs::Trigger)
    DECLARE_ROS_SERVICE_CLIENT(cli_standby_, std_srvs::Trigger)
    DECLARE_ROS_SUBSCRIBER(sub_joint_states_, sensor_msgs::JointState)
    DECLARE_ROS_PUBLISHER(pub_state_, msgs::TaskState)
};

}  // namespace task

#endif  // TASK_TASKNODE_H
