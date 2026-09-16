// Copyright by BeeX [2026]

#ifndef PLANNER_PLANNERNODE_H
#define PLANNER_PLANNERNODE_H

#include <actionlib/server/simple_action_server.h>
#include <bx_msgs/RosBindings.hpp>
#include <msgs/GraspPlan.h>
#include <msgs/PlanAction.h>
#include <planner/CandidatePlanner.h>
#include <sensor_msgs/JointState.h>

#include <mutex>

namespace planner {

// Serves the plan action: plans from the current joints to the best candidate, publishes the result latched.
class PlannerNode {
public:
    explicit PlannerNode(const PlannerConfig &config);

private:
    void onJointStates(const sensor_msgs::JointState::ConstPtr &msg);
    void onPlan(const msgs::PlanGoalConstPtr &goal);

    bool            currentJoints(kine::JointAngles &model_joints, std::string &why);
    msgs::GraspPlan toMessage(const PlanOutcome &outcome, const msgs::CloudResult &cloud) const;

    PlannerConfig                                   config_;
    CandidatePlanner                                planner_;
    actionlib::SimpleActionServer<msgs::PlanAction> server_;

    std::mutex              joints_mutex_;
    sensor_msgs::JointState latest_joints_;

    DECLARE_ROS_SUBSCRIBER(sub_joint_states_, sensor_msgs::JointState)
    DECLARE_ROS_PUBLISHER(pub_result_, msgs::GraspPlan)
};

}  // namespace planner

#endif  // PLANNER_PLANNERNODE_H
