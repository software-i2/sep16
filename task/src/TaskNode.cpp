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
    INIT_ROS_SUBSCRIBER(sub_camera_cloud_, config_.topic_camera_cloud, 1, &TaskNode::onCameraCloud);
    pub_state_ = mainNodeHandle->advertise<msgs::TaskState>(config_.topic_state, 1, true);

    entered_at_ = ros::Time::now();
    publishState("waiting for task/start");
}

void TaskNode::tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string                 message;
    Event                       event = Event::NONE;
    switch (state_) {
    case State::STREAM:
        event = checkStream(message);
        break;
    case State::COLLECT:
    case State::PROCESS:
        event = checkCollect(message);
        break;
    case State::PICKSPOT:
    case State::GOTOSPOT:
        event = checkPark(message);
        break;
    case State::PICKGRASP:
        event = checkPlan(message);
        break;
    case State::GOTOGRASP:
        event = checkExecute(message);
        break;
    case State::CLOSEJAW:
        event = checkJaw(message);
        break;
    case State::RESURVEY:
        event = checkResurvey(message);
        break;
    case State::RETARGET:
        event = checkRetarget(message);
        break;
    case State::REPARK:
        event = checkRepark(message);
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
    park_attempt_    = 0;
    look_attempt_    = 0;
    grabbed_         = false;
    phase_           = Phase::SPOT;
    run_started_     = ros::Time::now();
    surveying_since_ = ros::Time();
    reparking_since_ = ros::Time();
    locked_          = msgs::CloudResult();
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

// Only that a frame arrived, not what was in it. STREAM is asking whether the camera is
// producing at all, which is the one thing no action server can answer before it is asked.
void TaskNode::onCameraCloud(const sensor_msgs::PointCloud2::ConstPtr & /*msg*/) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_seen_at_ = ros::Time::now();
}

void TaskNode::onCollectFeedback(const msgs::CollectFeedbackConstPtr &feedback) {
    if (feedback->stage == msgs::CollectFeedback::PROCESSING) {
        processing_ = true;
    }
}

// The park action searches and then drives under one goal, so its feedback is the only thing
// that says the search is over and the vehicle is committed.
void TaskNode::onParkFeedback(const msgs::ParkFeedbackConstPtr &feedback) {
    if (feedback->stage == "moving") {
        driving_ = true;
    }
}

void TaskNode::apply(Event event, const std::string &message) {
    const State next = nextState(state_, event);
    if (next == state_) {
        return;
    }
    // Which survey pass COLLECT is about to run is decided by what sent it there: the spot
    // pass starts a fresh survey, everything after a park is choosing what to grab.
    if (next == State::COLLECT) {
        phase_ = (state_ == State::STREAM || state_ == State::RESURVEY || state_ == State::REPARK) ? Phase::SPOT
                                                                                                   : Phase::GRASP;
    }
    enter(next, message);
}

// Runs what a state does on entry, then announces it.
void TaskNode::enter(State state, const std::string &message) {
    left_       = state_;
    left_after_ = entered_at_.isZero() ? -1.0 : (ros::Time::now() - entered_at_).toSec();
    state_      = state;
    entered_at_ = ros::Time::now();
    std::string why;

    switch (state) {
    case State::COLLECT: {
        processing_ = false;
        msgs::CollectGoal goal;
        // Anything buffered on the grasp pass was seen from somewhere else, or with the arm
        // somewhere else, so it cannot be blended with what is there now.
        goal.fresh = phase_ == Phase::GRASP;
        collect_.sendGoal(goal, actionlib::SimpleActionClient<msgs::CollectAction>::SimpleDoneCallback(),
                          actionlib::SimpleActionClient<msgs::CollectAction>::SimpleActiveCallback(),
                          boost::bind(&TaskNode::onCollectFeedback, this, _1));
        break;
    }

    // Something is in view to park for, so the survey budget has done its job and starts over
    // if it is ever needed again.
    case State::PICKSPOT: {
        driving_        = false;
        surveying_since_ = ros::Time();
        msgs::ParkGoal goal;
        goal.cloud = cloud_;
        park_.sendGoal(goal, actionlib::SimpleActionClient<msgs::ParkAction>::SimpleDoneCallback(),
                       actionlib::SimpleActionClient<msgs::ParkAction>::SimpleActiveCallback(),
                       boost::bind(&TaskNode::onParkFeedback, this, _1));
        break;
    }

    // A parking spot is only spent once the vehicle actually drives to one. Searching from
    // where it already stands and electing not to move costs nothing and is not counted; the
    // allowance is three places to stand, not three searches.
    case State::GOTOSPOT:
        ++park_attempt_;
        reparking_since_ = ros::Time();
        break;

    case State::PICKGRASP: {
        msgs::PlanGoal goal;
        goal.cloud = cloud_;
        planner_.sendGoal(goal);
        break;
    }

    case State::GOTOGRASP: {
        msgs::ExecuteGoal goal;
        goal.path = plan_.path;
        executor_.sendGoal(goal);
        break;
    }

    case State::CLOSEJAW:
        if (!callTrigger(cli_close_jaw_, config_.service_close_jaw, why)) {
            enter(State::FAIL, "could not close the jaw: " + why);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(jaw_mutex_);
            jaw_closed_at_   = ros::Time::now();
            jaw_last_seen_   = ros::Time();
            jaw_still_since_ = ros::Time();
        }
        break;

    // The budget runs from the first empty survey of a streak, not from each one, or looking
    // again would reset it every time and never run out.
    case State::RESURVEY: {
        if (surveying_since_.isZero()) {
            surveying_since_ = ros::Time::now();
        }
        char line[160];
        std::snprintf(line, sizeof(line), "; nothing to park for after %.0f s of %.0f, looking again%s",
                      (ros::Time::now() - surveying_since_).toSec(), config_.spot_search_s,
                      after(config_.retry_delay_s).c_str());
        publishState(message + line);
        return;
    }

    case State::RETARGET: {
        ++look_attempt_;
        char line[160];
        std::snprintf(line, sizeof(line), "; look %u of %d from the park pose gave no grasp, looking again%s",
                      look_attempt_, config_.retarget_attempts, after(config_.retry_delay_s).c_str());
        publishState(message + line);
        return;
    }

    case State::REPARK: {
        look_attempt_ = 0;
        if (reparking_since_.isZero()) {
            reparking_since_ = ros::Time::now();
        }
        char line[176];
        std::snprintf(line, sizeof(line), "; nothing worked from here, %u of %d parking spots used, surveying for "
                                          "somewhere else%s",
                      park_attempt_, config_.park_attempts, after(config_.retry_delay_s).c_str());
        publishState(message + line);
        return;
    }

    // Everything is cancelled and the arm released. Both a stop and contact land here, and
    // leaving takes another task/start.
    case State::ESTOP:
        collect_.cancelAllGoals();
        park_.cancelAllGoals();
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
    msg.attempt = park_attempt_;
    msg.grabbed = grabbed_;
    PUBLISH_ROS(pub_state_, msg);

    if (message.empty()) {
        return;
    }
    // What the step just finished cost, and how long the whole pick has been going. The stage
    // named in the brackets is the one the time belongs to, not the one being entered.
    char spent[64] = "";
    if (left_after_ >= 0.0) {
        std::snprintf(spent, sizeof(spent), "[%s %.1fs, run %.1fs] ", stateName(left_), left_after_,
                      run_started_.isZero() ? 0.0 : (ros::Time::now() - run_started_).toSec());
    }
    if (state_ == State::FAIL || state_ == State::ESTOP) {
        LOG_ERROR("[task] %s: %s%s", msg.state.c_str(), spent, message.c_str());
    } else if (state_ == State::RESURVEY || state_ == State::RETARGET || state_ == State::REPARK) {
        LOG_WARN("[task] %s: %s%s", msg.state.c_str(), spent, message.c_str());
    } else {
        LOG_INFO("[task] %s: %s%s", msg.state.c_str(), spent, message.c_str());
    }
}

Event TaskNode::checkStream(std::string &message) {
    ros::Time seen;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        seen = frame_seen_at_;
    }
    // A camera that was already producing when task/start was pressed counts. Demanding a frame
    // that lands after this state was entered fails a camera slower than the timeout for no
    // reason, and the cloud node buffers regardless of when the window opened.
    const double age = seen.isZero() ? -1.0 : (ros::Time::now() - seen).toSec();
    if (age >= 0.0 && age < config_.stream_timeout_s) {
        char line[160];
        std::snprintf(line, sizeof(line), "the camera is producing, last frame %.1f s ago", age);
        message = line;
        return Event::STREAMING;
    }
    if ((ros::Time::now() - entered_at_).toSec() >= config_.stream_timeout_s) {
        char line[160];
        std::snprintf(line, sizeof(line), "nothing on %s in the last %.1f s", config_.topic_camera_cloud.c_str(),
                      config_.stream_timeout_s);
        message = line;
        return Event::NO_STREAM;
    }
    return Event::NONE;
}

Event TaskNode::checkCollect(std::string &message) {
    if (processing_ && state_ == State::COLLECT) {
        return Event::PROCESSING;
    }
    const actionlib::SimpleClientGoalState goal = collect_.getState();
    if (goal == actionlib::SimpleClientGoalState::SUCCEEDED) {
        cloud_ = collect_.getResult()->cloud;
        if (phase_ == Phase::SPOT) {
            message = cloud_.summary;
            return cloud_.candidates.empty() ? Event::NO_CANDIDATES_SPOT : Event::CANDIDATES_SPOT;
        }
        message = cloud_.summary + freshMapExtent() + comparedWithSnapshot();
        return cloud_.candidates.empty() ? Event::NO_CANDIDATES_GRASP : Event::CANDIDATES_GRASP;
    }
    if (goal.isDone()) {
        message = "cloud: " + goal.getText();
        return Event::FAILURE;
    }
    return Event::NONE;
}

Event TaskNode::checkPark(std::string &message) {
    // The search is over the moment the drive starts, whatever the action does afterwards.
    if (driving_ && state_ == State::PICKSPOT) {
        message = "a spot was chosen, driving there";
        return Event::SPOT_CHOSEN;
    }
    const actionlib::SimpleClientGoalState goal = park_.getState();
    if (goal == actionlib::SimpleClientGoalState::SUCCEEDED) {
        const msgs::ParkResultConstPtr result = park_.getResult();
        message = result->message;
        // Declining to move is not the same as nowhere to park. When the pose it is standing on
        // still holds candidates, the vehicle has not moved and no park has been spent, so the
        // planner gets its turn: park's route test is a straight line, and the planner searches.
        // Re-parking instead cannot help, because nothing moved for it to reconsider.
        if (!result->success) {
            return result->chosen.grasps_admitted > 0 ? Event::SPOT_UNCHANGED : Event::NO_SPOT;
        }
        cloud_  = result->locked;
        locked_ = result->locked;
        // Standing still and arriving differ only in whether anything drove; either way the
        // snapshot behind the decision is now the reference, not the thing being planned.
        return state_ == State::GOTOSPOT ? Event::ARRIVED : Event::SPOT_UNCHANGED;
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

// BLOCKED is the only contact the arm can report while it is moving, so it is the only thing
// that latches. Everything else the executor can say is worth another look.
Event TaskNode::checkExecute(std::string &message) {
    const actionlib::SimpleClientGoalState goal = executor_.getState();
    if (!goal.isDone()) {
        return Event::NONE;
    }
    const msgs::ExecuteResultConstPtr result = executor_.getResult();
    if (!result) {
        message = "executor: " + goal.getText();
        return Event::FAILURE;
    }
    message = result->message;
    switch (result->outcome) {
    case msgs::ExecuteResult::REACHED:
        return Event::REACHED;
    case msgs::ExecuteResult::BLOCKED:
        return Event::COLLIDED;
    case msgs::ExecuteResult::STALLED:
        return Event::STALLED;
    default:
        return Event::FAILURE;
    }
}

// The jaw has settled once its reading holds still for settle_time; stopping short of closed
// means it holds something. Closing on nothing is recorded and carries on: there is no handle
// to catch in simulation, and the thresholds have never been checked on the bench.
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

// Nothing has been decided and nothing has moved, so an empty survey costs only the frames it
// waited for. It is worth retrying freely: the vehicle may be mid-descent, or the scene may
// simply not be in view yet. The budget is there so a run with nothing in front of it ends.
Event TaskNode::checkResurvey(std::string &message) {
    if ((ros::Time::now() - surveying_since_).toSec() >= config_.spot_search_s) {
        char line[160];
        std::snprintf(line, sizeof(line), "nothing worth parking for came into view in %.0f s",
                      config_.spot_search_s);
        message = line;
        return Event::OUT_OF_TIME;
    }
    if ((ros::Time::now() - entered_at_).toSec() >= config_.retry_delay_s) {
        message = "surveying again";
        return Event::SURVEY_AGAIN;
    }
    return Event::NONE;
}

// Nothing has moved since the park, so looking again costs only the frames it waits for. What
// it cannot do is find a way out of a pose the arm genuinely cannot work from, which is what
// running out of looks means, and why that hands over to REPARK rather than failing.
Event TaskNode::checkRetarget(std::string &message) {
    if (config_.retarget_attempts > 0 && look_attempt_ >= static_cast<uint32_t>(config_.retarget_attempts)) {
        message = "looked from the park pose " + std::to_string(look_attempt_) + " times without a grasp";
        return Event::OUT_OF_LOOKS;
    }
    if ((ros::Time::now() - entered_at_).toSec() >= config_.retry_delay_s) {
        message = "looking again";
        return Event::LOOK_AGAIN;
    }
    return Event::NONE;
}

// Two ways to run out. The count is places the vehicle actually stood, which is what the
// allowance is about. The clock is there because a search that keeps electing not to move
// spends no count at all, and without it that loop would go round for ever.
Event TaskNode::checkRepark(std::string &message) {
    if (config_.park_attempts > 0 && park_attempt_ >= static_cast<uint32_t>(config_.park_attempts)) {
        message = "stood in " + std::to_string(park_attempt_) + " parking spots without a grasp";
        return Event::OUT_OF_PARKS;
    }
    if ((ros::Time::now() - reparking_since_).toSec() >= config_.repark_search_s) {
        char line[176];
        std::snprintf(line, sizeof(line), "%.0f s of surveying without the search moving the vehicle anywhere better",
                      config_.repark_search_s);
        message = line;
        return Event::OUT_OF_PARKS;
    }
    if ((ros::Time::now() - entered_at_).toSec() >= config_.retry_delay_s) {
        message = "surveying again for somewhere else to park";
        return Event::PARK_AGAIN;
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
