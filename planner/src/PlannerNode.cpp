// Copyright by BeeX [2026]

#include <planner/PlannerNode.h>

#include <boost/bind.hpp>

namespace planner {

PlannerNode::PlannerNode(const PlannerConfig &config)
        : config_(config),
          planner_(config),
          server_(*mainNodeHandle, config.action_plan, boost::bind(&PlannerNode::onPlan, this, _1), false) {
    INIT_ROS_SUBSCRIBER(sub_joint_states_, config_.topic_joint_states, 1, &PlannerNode::onJointStates);
    pub_result_ = mainNodeHandle->advertise<msgs::GraspPlan>(config_.topic_plan_result, 1, true);
    server_.start();
}

void PlannerNode::onJointStates(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    latest_joints_ = *msg;
}

bool PlannerNode::currentJoints(kine::JointAngles &model_joints, std::string &why) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    if (latest_joints_.name.empty()) {
        why = "no joint_states received yet";
        return false;
    }
    const double age = (ros::Time::now() - latest_joints_.header.stamp).toSec();
    if (age > config_.joint_state_timeout_s) {
        why = "joint_states are " + std::to_string(age) + " s old";
        return false;
    }

    kine::JointAngles reported;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        const auto found = std::find(latest_joints_.name.begin(), latest_joints_.name.end(), config_.joint_names[j]);
        const size_t index = static_cast<size_t>(found - latest_joints_.name.begin());
        if (found == latest_joints_.name.end() || index >= latest_joints_.position.size()) {
            why = "joint_states has no " + config_.joint_names[j];
            return false;
        }
        reported[j] = latest_joints_.position[index];
    }
    model_joints = planner_.model().toModel(reported);
    return true;
}

void PlannerNode::onPlan(const msgs::PlanGoalConstPtr &goal) {
    msgs::PlanResult result;

    std::vector<Candidate> candidates;
    for (const msgs::GraspPose &pose : goal->cloud.candidates) {
        candidates.push_back({Eigen::Vector3d(pose.point.x, pose.point.y, pose.point.z),
                              Eigen::Vector3d(pose.bar_axis.x, pose.bar_axis.y, pose.bar_axis.z),
                              Eigen::Vector3d(pose.approach.x, pose.approach.y, pose.approach.z)});
    }

    kine::JointAngles start;
    std::string       why;
    if (!currentJoints(start, why)) {
        result.plan.header.stamp    = ros::Time::now();
        result.plan.header.frame_id = config_.base_frame;
        result.plan.summary         = "cannot plan: " + why;
        LOG_ERROR("[planner] %s", result.plan.summary.c_str());
        PUBLISH_ROS(pub_result_, result.plan);
        server_.setAborted(result, result.plan.summary);
        return;
    }

    const PlanOutcome outcome =
            planner_.plan(candidates, goal->cloud.obstacles, start, [this]() { return server_.isPreemptRequested(); });

    result.plan = toMessage(outcome, goal->cloud);
    PUBLISH_ROS(pub_result_, result.plan);

    if (server_.isPreemptRequested()) {
        LOG_WARN("[planner] cancelled");
        server_.setPreempted(result, "cancelled");
        return;
    }
    if (outcome.success) {
        LOG_INFO("[planner] %s", outcome.summary.c_str());
    } else {
        LOG_WARN("[planner] %s", outcome.summary.c_str());
    }
    server_.setSucceeded(result, outcome.summary);
}

msgs::GraspPlan PlannerNode::toMessage(const PlanOutcome &outcome, const msgs::CloudResult &cloud) const {
    msgs::GraspPlan plan;
    plan.header.stamp    = ros::Time::now();
    plan.header.frame_id = config_.base_frame;
    plan.success         = outcome.success;
    plan.summary         = outcome.summary;
    plan.cost            = outcome.cost;
    plan.chosen_index    = static_cast<uint32_t>(outcome.chosen_index);
    if (outcome.success) {
        plan.chosen = cloud.candidates[outcome.chosen_index];
    }

    plan.path.joint_names.assign(config_.joint_names.begin(), config_.joint_names.end());
    for (const kine::JointAngles &corner : outcome.path) {
        const kine::JointAngles reported = planner_.model().toReported(corner);
        msgs::Waypoint          waypoint;
        waypoint.positions.assign(reported.begin(), reported.end());
        plan.path.waypoints.push_back(waypoint);
    }

    for (const Outcome candidate_outcome : outcome.outcomes) {
        plan.candidate_outcomes.push_back(outcomeName(candidate_outcome));
    }
    return plan;
}

}  // namespace planner
