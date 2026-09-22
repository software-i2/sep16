// Copyright by BeeX [2026]

#include <task/TaskNode.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace task {
namespace {

std::string after(double delay_s) {
    if (delay_s <= 0.0) {
        return "";
    }
    char line[32];
    std::snprintf(line, sizeof(line), " in %.1f s", delay_s);
    return line;
}

}  // namespace

TaskNode::TaskNode(const TaskConfig &config)
        : config_(config),
          collect_(*mainNodeHandle, config.action_collect, false),
          park_(*mainNodeHandle, config.action_park, false),
          planner_(*mainNodeHandle, config.action_plan, false),
          executor_(*mainNodeHandle, config.action_execute, false),
          tf_listener_(tf_buffer_) {
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
    case State::RECOLLECT:
    case State::REPROCESS:
        event = checkRecollect(message);
        break;
    case State::REPLAN:
        event = checkReplan(message);
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
    case State::REVERIFY:
        event = checkReverify(message);
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
    attempt_          = 0;
    reverify_attempt_ = 0;
    grabbed_          = false;
    locked_           = msgs::CloudResult();
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

    case State::PLAN:
    case State::REPLAN: {
        msgs::PlanGoal goal;
        goal.cloud = cloud_;
        planner_.sendGoal(goal);
        break;
    }

    case State::RECOLLECT: {
        ++reverify_attempt_;
        processing_ = false;
        msgs::CollectGoal goal;
        goal.fresh = true;
        collect_.sendGoal(goal, actionlib::SimpleActionClient<msgs::CollectAction>::SimpleDoneCallback(),
                          actionlib::SimpleActionClient<msgs::CollectAction>::SimpleActiveCallback(),
                          boost::bind(&TaskNode::onCollectFeedback, this, _1));
        break;
    }

    case State::REVERIFY: {
        char line[160];
        std::snprintf(line, sizeof(line),
                      "; look %u of %d from the park pose did not end in a grasp, looking again%s",
                      reverify_attempt_, config_.reverify_attempts, after(config_.retry_delay_s).c_str());
        publishState(message + line);
        return;
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
        std::snprintf(line, sizeof(line), "; attempt %u found nothing to grab, trying again%s", attempt_,
                      after(config_.retry_delay_s).c_str());
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

    if (message.empty()) {
        return;
    }
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
        cloud_            = result->locked;
        locked_           = result->locked;
        reverify_attempt_ = 0;

        // Staying put leaves the snapshot describing exactly where the arm stands, so it is
        // still what to plan against. Once the vehicle has driven, it describes somewhere the
        // arm no longer is, and only the camera can say where the handle went.
        const bool moved = result->chosen.travel > 0.0 || std::fabs(result->chosen.yaw) > 0.0;
        return moved ? Event::PARKED_MOVED : Event::PARKED_STAYED;
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
        return plan_.success ? Event::PLAN_FOUND : Event::NO_PLAN;
    }
    if (goal.isDone()) {
        message = "planner: " + goal.getText();
        return Event::FAILURE;
    }
    return Event::NONE;
}

Event TaskNode::checkRecollect(std::string &message) {
    if (processing_ && state_ == State::RECOLLECT) {
        return Event::REPROCESSING;
    }
    const actionlib::SimpleClientGoalState goal = collect_.getState();
    if (goal == actionlib::SimpleClientGoalState::SUCCEEDED) {
        cloud_  = collect_.getResult()->cloud;
        message = cloud_.summary + freshMapExtent() + comparedWithSnapshot();
        return cloud_.candidates.empty() ? Event::REVERIFY_RETRY : Event::RECANDIDATES_FOUND;
    }
    if (goal.isDone()) {
        message = "cloud: " + goal.getText();
        return Event::FAILURE;
    }
    return Event::NONE;
}

Event TaskNode::checkReplan(std::string &message) {
    const actionlib::SimpleClientGoalState goal = planner_.getState();
    if (goal == actionlib::SimpleClientGoalState::SUCCEEDED) {
        plan_   = planner_.getResult()->plan;
        message = plan_.summary;
        return plan_.success ? Event::REPLAN_FOUND : Event::REVERIFY_RETRY;
    }
    if (goal.isDone()) {
        message = "planner: " + goal.getText();
        return Event::FAILURE;
    }
    return Event::NONE;
}

// The fresh map is built from one viewpoint 200 to 300 mm off the handle, so it covers far
// less scene than the one collected before the drive did, and unseen space counts as free.
// These two numbers are what a map that has shrunk too far looks like from the outside.
std::string TaskNode::freshMapExtent() const {
    const msgs::ObstacleMap &map = cloud_.obstacles;
    char                     line[200];
    std::snprintf(line, sizeof(line), "; fresh map %.2f by %.2f by %.2f m holding %zu obstacle cells",
                  map.size_x * map.voxel_size_m, map.size_y * map.voxel_size_m, map.size_z * map.voxel_size_m,
                  map.obstacle_cells.size());
    return line;
}

std::string TaskNode::comparedWithSnapshot() {
    if (locked_.candidates.empty() || cloud_.candidates.empty() || locked_.header.frame_id.empty()) {
        return "";
    }

    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(config_.base_frame, locked_.header.frame_id, ros::Time(0),
                                               ros::Duration(config_.reverify_transform_wait));
    } catch (const tf2::TransformException &e) {
        LOG_WARN("[task] the snapshot cannot be compared with what was just measured: %s", e.what());
        return "";
    }
    const geometry_msgs::Vector3    &t = transform.transform.translation;
    const geometry_msgs::Quaternion &q = transform.transform.rotation;
    const tf2::Transform into(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(t.x, t.y, t.z));

    std::vector<double> gaps;
    for (const msgs::GraspPose &was : locked_.candidates) {
        const tf2::Vector3 before = into * tf2::Vector3(was.point.x, was.point.y, was.point.z);
        double             nearest = -1.0;
        for (const msgs::GraspPose &now : cloud_.candidates) {
            const double gap = before.distance(tf2::Vector3(now.point.x, now.point.y, now.point.z));
            if (nearest < 0.0 || gap < nearest) {
                nearest = gap;
            }
        }
        if (nearest >= 0.0 && nearest <= config_.reverify_match_distance) {
            gaps.push_back(nearest);
        }
    }

    char line[200];
    if (gaps.empty()) {
        LOG_WARN("[task] none of the %zu candidates in the snapshot are within %.0f mm of any of the %zu measured "
                 "from the park pose",
                 locked_.candidates.size(), config_.reverify_match_distance * 1000.0, cloud_.candidates.size());
        std::snprintf(line, sizeof(line), "; nothing from before the move matches what is there now");
        return line;
    }
    std::sort(gaps.begin(), gaps.end());
    const double median = gaps[gaps.size() / 2];
    LOG_INFO("[task] %zu of %zu candidates found again after the drive, %.1f mm out at the median and %.1f mm at "
             "the worst: that is what planning against the snapshot would have cost",
             gaps.size(), locked_.candidates.size(), median * 1000.0, gaps.back() * 1000.0);
    std::snprintf(line, sizeof(line), "; the snapshot was %.1f mm out at the median, %.1f mm at the worst",
                  median * 1000.0, gaps.back() * 1000.0);
    return line;
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

// Nothing has moved since the park, so looking again costs only the frames it waits for. What
// it cannot do is find a way out of a pose the arm genuinely cannot work from, which is why
// the budget is small and running out of it flags the pick rather than driving somewhere else.
Event TaskNode::checkReverify(std::string &message) {
    if (config_.reverify_attempts > 0 && reverify_attempt_ >= static_cast<uint32_t>(config_.reverify_attempts)) {
        message = "looked again from the park pose " + std::to_string(reverify_attempt_)
                  + " times and still could not plan a grasp";
        return Event::GAVE_UP;
    }
    if ((ros::Time::now() - entered_at_).toSec() >= config_.retry_delay_s) {
        message = "looking again";
        return Event::REVERIFY_DUE;
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
