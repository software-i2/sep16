// Copyright by BeeX [2026]

#include <task/TaskNode.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace task {

TaskNode::TaskNode(const TaskConfig &config)
        : config_(config),
          collect_(*mainNodeHandle, config.action_collect, false),
          park_(*mainNodeHandle, config.action_park, false),
          planner_(*mainNodeHandle, config.action_plan, false),
          executor_(*mainNodeHandle, config.action_execute, false) {
    INIT_ROS_SERVICE_SERVER(srv_start_, config_.service_start, &TaskNode::onStart);
    INIT_ROS_SERVICE_SERVER(srv_stop_, config_.service_stop, &TaskNode::onStop);
    INIT_ROS_SERVICE_CLIENT(cli_close_jaw_, std_srvs::Trigger, config_.service_close_jaw);
    INIT_ROS_SERVICE_CLIENT(cli_standby_, std_srvs::Trigger, config_.service_standby);
    INIT_ROS_SUBSCRIBER(sub_joint_states_, config_.topic_joint_states, 1, &TaskNode::onJointStates);
    pub_state_ = mainNodeHandle->advertise<msgs::TaskState>(config_.topic_state, 1, true);

    entered_at_ = ros::Time::now();
    publishState("waiting for task/start");
}

void TaskNode::tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string                 message;
    Event                       event = Event::NONE;
    switch (state_) {
    case State::COLLECT:
    case State::PROCESS:
        event = checkCollect(message);
        break;
    case State::PARK:
        event = checkPark(message);
        break;
    case State::PLAN:
        event = checkPlan(message);
        break;
    case State::EXECUTE:
        event = checkExecute(message);
        break;
    case State::JAWCLOSING:
        event = checkJaw(message);
        break;
    case State::RETRY:
        event = checkRetry(message);
        break;
    default:
        break;
    }
    apply(event, message);
}

bool TaskNode::onStart(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res) {
    const ros::Duration wait(config_.server_wait_s);
    if (!collect_.waitForServer(wait) || !park_.waitForServer(wait) || !planner_.waitForServer(wait)
        || !executor_.waitForServer(wait)) {
        res.success = false;
        res.message = "the cloud, park, planner or executor node is not running";
        LOG_WARN("[task] start refused: %s", res.message.c_str());
        return true;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (nextState(state_, Event::START) == state_) {
        res.success = false;
        res.message = std::string("already running, in ") + stateName(state_);
        LOG_WARN("[task] start refused: %s", res.message.c_str());
        return true;
    }
    attempt_ = 0;
    grabbed_ = false;
    parked_  = false;
    apply(Event::START, "started");
    res.success = true;
    res.message = "started";
    return true;
}

bool TaskNode::onStop(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res) {
    std::lock_guard<std::mutex> lock(mutex_);
    apply(Event::STOP, "stopped on request");
    res.success = true;
    res.message = "stopped, arm released";
    return true;
}

void TaskNode::onJointStates(const sensor_msgs::JointState::ConstPtr &msg) {
    const auto found = std::find(msg->name.begin(), msg->name.end(), config_.jaw_joint_name);
    const size_t index = static_cast<size_t>(found - msg->name.begin());
    if (found == msg->name.end() || index >= msg->position.size()) {
        return;
    }
    std::lock_guard<std::mutex> lock(jaw_mutex_);
    jaw_position_ = msg->position[index];
    jaw_stamp_    = msg->header.stamp;
}

void TaskNode::onCollectFeedback(const msgs::CollectFeedbackConstPtr &feedback) {
    if (feedback->stage == msgs::CollectFeedback::PROCESSING) {
        processing_ = true;
    }
}

void TaskNode::apply(Event event, const std::string &message) {
    const State next = nextState(state_, event);
    if (next != state_) {
        enter(next, message);
    }
}

// Runs what a state does on entry, then announces it.
void TaskNode::enter(State state, const std::string &message) {
    state_      = state;
    entered_at_ = ros::Time::now();
    std::string why;

    switch (state) {
    case State::COLLECT:
        ++attempt_;
        processing_ = false;
        collect_.sendGoal(msgs::CollectGoal(), actionlib::SimpleActionClient<msgs::CollectAction>::SimpleDoneCallback(),
                          actionlib::SimpleActionClient<msgs::CollectAction>::SimpleActiveCallback(),
                          boost::bind(&TaskNode::onCollectFeedback, this, _1));
        break;

    case State::PARK: {
        msgs::ParkGoal goal;
        goal.cloud = cloud_;
        park_.sendGoal(goal);
        break;
    }

    case State::PLAN: {
        msgs::PlanGoal goal;
        goal.cloud = cloud_;
        planner_.sendGoal(goal);
        break;
    }

    case State::EXECUTE: {
        msgs::ExecuteGoal goal;
        goal.path = plan_.path;
        executor_.sendGoal(goal);
        break;
    }

    case State::JAWCLOSING:
        if (!callTrigger(cli_close_jaw_, config_.service_close_jaw, why)) {
            enter(State::FAILED, "could not close the jaw: " + why);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(jaw_mutex_);
            jaw_closed_at_   = ros::Time::now();
            jaw_last_seen_   = ros::Time();
            jaw_still_since_ = ros::Time();
        }
        break;

    case State::RETRY: {
        char line[128];
        std::snprintf(line, sizeof(line), "; attempt %u found nothing to grab, trying again in %.1f s", attempt_,
                      config_.retry_delay_s);
        publishState(message + line);
        return;
    }

    case State::STOPPED:
        collect_.cancelAllGoals();
        planner_.cancelAllGoals();
        executor_.cancelAllGoals();
        if (!callTrigger(cli_standby_, config_.service_standby, why)) {
            publishState(message + "; standby failed, the arm may still be powered: " + why);
            return;
        }
        break;

    default:
        break;
    }
    publishState(message);
}

void TaskNode::publishState(const std::string &message) {
    msgs::TaskState msg;
    msg.state   = stateName(state_);
    msg.message = message;
    msg.attempt = attempt_;
    msg.grabbed = grabbed_;
    PUBLISH_ROS(pub_state_, msg);

    if (state_ == State::FAILED) {
        LOG_ERROR("[task] %s: %s", msg.state.c_str(), message.c_str());
    } else if (state_ == State::RETRY || state_ == State::STOPPED) {
        LOG_WARN("[task] %s: %s", msg.state.c_str(), message.c_str());
    } else {
        LOG_INFO("[task] %s: %s", msg.state.c_str(), message.c_str());
    }
}

Event TaskNode::checkCollect(std::string &message) {
    if (processing_ && state_ == State::COLLECT) {
        message = "frames collected";
        return Event::PROCESSING;
    }
    const actionlib::SimpleClientGoalState goal = collect_.getState();
    if (goal == actionlib::SimpleClientGoalState::SUCCEEDED) {
        cloud_  = collect_.getResult()->cloud;
        message = cloud_.summary;
        return cloud_.candidates.empty() ? Event::NO_CANDIDATES : Event::CANDIDATES_FOUND;
    }
    if (goal.isDone()) {
        message = "cloud: " + goal.getText();
        return Event::FAILURE;
    }
    return Event::NONE;
}

Event TaskNode::checkPark(std::string &message) {
    const actionlib::SimpleClientGoalState goal = park_.getState();
    if (goal == actionlib::SimpleClientGoalState::SUCCEEDED) {
        const msgs::ParkResultConstPtr result = park_.getResult();
        message = result->message;
        if (!result->success) {
            return Event::NO_PARK;
        }
        // Everything after this plans against the snapshot, in the frame the arm base stood
        // in when it was taken. Collecting again here would look at the target from too close.
        cloud_ = result->locked;
        parked_ = true;
        return Event::PARKED;
    }
    if (goal.isDone()) {
        message = "park: " + goal.getText();
        return Event::FAILURE;
    }
    return Event::NONE;
}

Event TaskNode::checkPlan(std::string &message) {
    const actionlib::SimpleClientGoalState goal = planner_.getState();
    if (goal == actionlib::SimpleClientGoalState::SUCCEEDED) {
        plan_   = planner_.getResult()->plan;
        message = plan_.summary;
        if (plan_.success) {
            return Event::PLAN_FOUND;
        }
        // Collecting again after a park is worse than useless: the target is now too close to
        // see well, and on a replayed bag the scene simply follows the camera, so every retry
        // parks further forward and the handle stays exactly as far away as it was.
        if (parked_) {
            message += "; the park pose was reachable but blocked, and the scene is locked, so "
                       "there is nothing to gain by looking again";
            return Event::FAILURE;
        }
        return Event::NO_PLAN;
    }
    if (goal.isDone()) {
        message = "planner: " + goal.getText();
        return Event::FAILURE;
    }
    return Event::NONE;
}

Event TaskNode::checkExecute(std::string &message) {
    const actionlib::SimpleClientGoalState goal = executor_.getState();
    if (goal == actionlib::SimpleClientGoalState::SUCCEEDED) {
        message = executor_.getResult()->message;
        return Event::REACHED;
    }
    if (goal.isDone()) {
        message = "executor: " + goal.getText();
        return Event::FAILURE;
    }
    return Event::NONE;
}

// The jaw has settled once its reading holds still for settle_time; stopping short of closed means it holds something.
Event TaskNode::checkJaw(std::string &message) {
    char line[200];
    {
        std::lock_guard<std::mutex> lock(jaw_mutex_);
        if (jaw_stamp_ > jaw_closed_at_ && jaw_stamp_ != jaw_last_seen_) {
            jaw_last_seen_ = jaw_stamp_;
            if (jaw_still_since_.isZero() || std::fabs(jaw_position_ - jaw_still_position_) > config_.jaw_settle_tolerance) {
                jaw_still_position_ = jaw_position_;
                jaw_still_since_    = jaw_stamp_;
            } else if ((jaw_stamp_ - jaw_still_since_).toSec() >= config_.jaw_settle_time_s) {
                grabbed_ = jaw_position_ > config_.jaw_closed_width + config_.jaw_grabbed_margin;
                std::snprintf(line, sizeof(line),
                              grabbed_ ? "holding something: the jaw stopped at %.1f mm, %.1f mm short of closed"
                                       : "closed on nothing: the jaw closed to %.1f mm, %.1f mm from fully closed",
                              jaw_position_ * 1000.0,
                              std::max(0.0, jaw_position_ - config_.jaw_closed_width) * 1000.0);
                message = line;
                return Event::JAW_SETTLED;
            }
        }
    }
    if ((ros::Time::now() - entered_at_).toSec() >= config_.jaw_timeout_s) {
        grabbed_ = false;
        std::snprintf(line, sizeof(line), "the jaw did not settle within %.1f s, so there is no grip verdict",
                      config_.jaw_timeout_s);
        message = line;
        return Event::JAW_SETTLED;
    }
    return Event::NONE;
}

Event TaskNode::checkRetry(std::string &message) {
    if (config_.max_attempts > 0 && attempt_ >= static_cast<uint32_t>(config_.max_attempts)) {
        message = "gave up after " + std::to_string(attempt_) + " attempts";
        return Event::GAVE_UP;
    }
    if ((ros::Time::now() - entered_at_).toSec() >= config_.retry_delay_s) {
        message = "trying again";
        return Event::RETRY_DUE;
    }
    return Event::NONE;
}

bool TaskNode::callTrigger(ros::ServiceClient &client, const std::string &name, std::string &why) {
    std_srvs::Trigger srv;
    if (!CALL_SRV_ROS(client, srv)) {
        why = name + " did not answer";
        return false;
    }
    if (!srv.response.success) {
        why = srv.response.message;
        return false;
    }
    return true;
}

}  // namespace task
