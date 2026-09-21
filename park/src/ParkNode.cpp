// Copyright by BeeX [2026]

#include <kine/ArmBody.h>
#include <kine/ReachBuild.h>
#include <park/ParkNode.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>

namespace park {
namespace {

Eigen::Isometry3d fromParts(const std::array<double, 3> &position, const std::array<double, 3> &rpy) {
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear()          = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                           .toRotationMatrix();
    out.translation() = Eigen::Vector3d(position[0], position[1], position[2]);
    return out;
}

}  // namespace

ParkNode::ParkNode(const ParkConfig &config)
        : config_(config),
          server_(*mainNodeHandle, config.action_park, boost::bind(&ParkNode::onPark, this, _1), false) {
    const kine::ArmModel model(config_.arm);
    body_.reset(new kine::ArmBody(model, config_.arm.jaw, config_.blade_sample_step));
    const kine::ArmBody &body = *body_;

    kine::ReachSpec spec;
    spec.along             = model.dimensions().wrist_to_jaw_mount + config_.grasp_point_from_mount;
    spec.floor_z           = config_.safety_floor_z;
    spec.cell              = config_.reach_cell;
    spec.blade_sample_step = config_.blade_sample_step;
    spec.roll_samples      = config_.reach_roll_samples;
    spec.axis_samples      = config_.reach_axis_samples;

    const uint64_t want = kine::ReachTable::hashConfig(config_.arm, spec);
    std::string    why;
    if (config_.reach_cache_path.empty() || !table_.load(config_.reach_cache_path, want, why)) {
        if (!config_.reach_cache_path.empty()) {
            LOG_INFO("[park] building the reach table: %s", why.c_str());
        }
        kine::buildReachTable(body, config_.arm, spec, table_);
        if (!config_.reach_cache_path.empty() && !table_.save(config_.reach_cache_path, why)) {
            LOG_WARN("[park] the reach table was not cached: %s", why.c_str());
        }
    }
    LOG_INFO("[park] reach table %d by %d cells, grasp point %.3f m down the wrist axis", table_.spec().cells_r,
             table_.spec().cells_z, spec.along);

    body_to_arm_ = fromParts(config_.arm_mount_position, config_.arm_mount_rpy).inverse();
    const Eigen::Vector3d camera_in_body(config_.camera_mount_position[0], config_.camera_mount_position[1],
                                         config_.camera_mount_position[2]);
    search_.reset(new PoseSearch(table_, config_.search, body_to_arm_, camera_in_body, config_.safety_floor_z));

    INIT_ROS_SUBSCRIBER(sub_joint_states_, config_.topic_joint_states, 1, &ParkNode::onJointStates);
    INIT_ROS_SERVICE_SERVER(srv_reset_, config_.service_reset, &ParkNode::onReset);
    pub_result_       = mainNodeHandle->advertise<msgs::ParkPose>(config_.topic_park_result, 1, true);
    pub_locked_cloud_ = mainNodeHandle->advertise<msgs::CloudResult>(config_.topic_locked_cloud, 1, true);
    broadcast(where_, ros::Time::now());
    tick_ = mainNodeHandle->createTimer(ros::Duration(1.0 / config_.move_rate_hz), &ParkNode::onTick, this);
    server_.start();
}

namespace {

// Where the arm would stand at `pose`, expressed back in the frame the obstacle map was built
// in, which is what the grid has to be queried with.
Eigen::Isometry3d stepOf(const Pose &pose) {
    Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
    step.linear()      = Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    step.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
    return step;
}

// The first thing BiRrtStar tries is the straight line from start to goal; when it is clear
// the plan is that line and nothing else has to be searched. A grasp that fails this is the
// one that comes back as no_path, so it is worth knowing before the vehicle commits.
bool lineClear(planner::CollisionChecker &checker, const kine::JointAngles &from, const kine::JointAngles &to,
               double step) {
    double furthest = 0.0;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        furthest = std::max(furthest, std::fabs(to[j] - from[j]));
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(furthest / step)));
    for (int n = 1; n < steps; ++n) {
        const double      share = static_cast<double>(n) / steps;
        kine::JointAngles between;
        for (int j = 0; j < kine::JOINT_COUNT; ++j) {
            between[j] = from[j] + (to[j] - from[j]) * share;
        }
        if (checker.check(between) != planner::Verdict::CLEAR) {
            return false;
        }
    }
    return true;
}

}  // namespace

Eigen::Isometry3d ParkNode::queryToMap(const Pose &pose) const {
    return body_to_arm_ * stepOf(pose) * body_to_arm_.inverse();
}

bool ParkNode::onReset(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res) {
    if (server_.isActive()) {
        res.success = false;
        res.message = "a park is running, stop it before resetting";
        return true;
    }
    {
        std::lock_guard<std::mutex> lock(where_mutex_);
        where_ = Pose();
    }
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        scene_locked_ = false;
    }
    broadcast(Pose(), ros::Time::now());
    res.success = true;
    res.message = "vehicle back at the origin";
    LOG_INFO("[park] %s", res.message.c_str());
    return true;
}

void ParkNode::onJointStates(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    latest_joints_ = *msg;
}

kine::JointAngles ParkNode::currentJoints() const {
    const kine::ArmModel model(config_.arm);
    kine::JointAngles    reported = config_.arm_home;
    std::lock_guard<std::mutex> lock(joints_mutex_);
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        const auto found =
                std::find(latest_joints_.name.begin(), latest_joints_.name.end(), config_.joint_names[j]);
        const size_t index = static_cast<size_t>(found - latest_joints_.name.begin());
        if (found != latest_joints_.name.end() && index < latest_joints_.position.size()) {
            reported[j] = latest_joints_.position[index];
        }
    }
    return model.toModel(reported);
}

bool ParkNode::transitClear(const Pose &from, const Pose &to, planner::ObstacleGrid &grid,
                            const kine::JointAngles &held, size_t blade_stride) const {
    planner::CollisionChecker checker(*body_, grid, config_.safety_floor_z, config_.link_sample_step,
                                      blade_stride);
    for (int n = 0; n <= config_.transit_samples; ++n) {
        const double share = static_cast<double>(n) / config_.transit_samples;
        Pose         along;
        along.x   = from.x + (to.x - from.x) * share;
        along.y   = from.y + (to.y - from.y) * share;
        along.z   = from.z + (to.z - from.z) * share;
        along.yaw = from.yaw + (to.yaw - from.yaw) * share;
        grid.setQueryToMap(queryToMap(along));
        if (checker.check(held) != planner::Verdict::CLEAR) {
            return false;
        }
    }
    return true;
}

ParkNode::Verdict ParkNode::verify(const std::vector<Grasp> &grasps, const Pose &pose, planner::ObstacleGrid &grid,
                                   const kine::JointAngles &start, size_t blade_stride, bool check_routes) const {
    // The map was built in the arm base as it stood at collect time. Where the arm would
    // stand at `pose`, expressed back in that frame, is what the grid has to be queried with.
    grid.setQueryToMap(queryToMap(pose));

    planner::CollisionChecker checker(*body_, grid, config_.safety_floor_z, config_.link_sample_step,
                                      blade_stride);
    const planner::GraspSettings settings{config_.grasp_point_from_mount, config_.max_approach_deviation,
                                          config_.joint_cost_weights};

    const double cos_yaw = std::cos(-pose.yaw);
    const double sin_yaw = std::sin(-pose.yaw);
    const auto   turn    = [&](const Eigen::Vector3d &v) {
        return Eigen::Vector3d(v.x() * cos_yaw - v.y() * sin_yaw, v.x() * sin_yaw + v.y() * cos_yaw, v.z());
    };

    Verdict out;
    for (size_t i = 0; i < grasps.size(); ++i) {
        planner::Candidate candidate;
        candidate.point = body_to_arm_
                          * turn(Eigen::Vector3d(grasps[i].point.x() - pose.x, grasps[i].point.y() - pose.y,
                                                 grasps[i].point.z() - pose.z));
        candidate.bar_axis = body_to_arm_.linear() * turn(grasps[i].bar_axis);
        candidate.approach = Eigen::Vector3d::Zero();
        std::vector<planner::GraspGoal> goals;
        planner::findGraspGoals(candidate, i, start, settings, checker, goals);
        if (goals.empty()) {
            continue;
        }
        ++out.held;
        if (!check_routes) {
            continue;
        }
        for (const planner::GraspGoal &goal : goals) {
            if (lineClear(checker, start, goal.joints, config_.edge_check_step)) {
                ++out.routable;
                break;
            }
        }
    }
    return out;
}

void ParkNode::onTick(const ros::TimerEvent &) {
    Pose now;
    {
        std::lock_guard<std::mutex> lock(where_mutex_);
        now = where_;
    }
    broadcast(now, ros::Time::now());
}

void ParkNode::broadcast(const Pose &pose, const ros::Time &stamp) {
    std::vector<geometry_msgs::TransformStamped> sent;

    geometry_msgs::TransformStamped body;
    body.header.stamp    = stamp;
    body.header.frame_id = config_.locked_frame;
    body.child_frame_id  = config_.body_frame;
    body.transform.translation.x = pose.x;
    body.transform.translation.y = pose.y;
    body.transform.translation.z = pose.z;
    body.transform.rotation.w    = std::cos(0.5 * pose.yaw);
    body.transform.rotation.z    = std::sin(0.5 * pose.yaw);
    sent.push_back(body);

    std::lock_guard<std::mutex> lock(scene_mutex_);
    if (scene_locked_) {
        const Eigen::Quaterniond turn(locked_to_scene_.linear());
        geometry_msgs::TransformStamped scene;
        scene.header.stamp    = stamp;
        scene.header.frame_id = config_.locked_frame;
        scene.child_frame_id  = config_.scene_frame;
        scene.transform.translation.x = locked_to_scene_.translation().x();
        scene.transform.translation.y = locked_to_scene_.translation().y();
        scene.transform.translation.z = locked_to_scene_.translation().z();
        scene.transform.rotation.x    = turn.x();
        scene.transform.rotation.y    = turn.y();
        scene.transform.rotation.z    = turn.z();
        scene.transform.rotation.w    = turn.w();
        sent.push_back(scene);
    }
    broadcaster_.sendTransform(sent);
}

void ParkNode::lockScene(const Pose &at) {
    Eigen::Isometry3d locked_to_body = Eigen::Isometry3d::Identity();
    locked_to_body.linear()          = Eigen::AngleAxisd(at.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    locked_to_body.translation()     = Eigen::Vector3d(at.x, at.y, at.z);

    std::lock_guard<std::mutex> lock(scene_mutex_);
    locked_to_scene_ = locked_to_body * body_to_arm_.inverse();
    scene_locked_    = true;
}

bool ParkNode::walkTo(const Pose &target, std::string &why) {
    Pose start;
    {
        std::lock_guard<std::mutex> lock(where_mutex_);
        start = where_;
    }
    const double distance = std::hypot(std::hypot(target.x - start.x, target.y - start.y), target.z - start.z);
    const int    steps = std::max(1, static_cast<int>(std::ceil(distance / config_.move_speed_m_s * config_.move_rate_hz)));

    ros::Rate rate(config_.move_rate_hz);
    for (int n = 1; n <= steps; ++n) {
        if (server_.isPreemptRequested() || !ros::ok()) {
            why = "the move was stopped";
            return false;
        }
        const double share = static_cast<double>(n) / steps;
        Pose         now;
        now.x   = start.x + (target.x - start.x) * share;
        now.y   = start.y + (target.y - start.y) * share;
        now.z   = start.z + (target.z - start.z) * share;
        now.yaw = start.yaw + (target.yaw - start.yaw) * share;
        {
            std::lock_guard<std::mutex> lock(where_mutex_);
            where_ = now;
        }

        msgs::ParkFeedback feedback;
        feedback.stage = "moving";
        server_.publishFeedback(feedback);
        rate.sleep();
    }
    return true;
}

msgs::ParkPose ParkNode::toMessage(const Scored &scored, size_t offered) const {
    msgs::ParkPose out;
    out.position.x     = scored.pose.x;
    out.position.y     = scored.pose.y;
    out.position.z     = scored.pose.z;
    out.yaw            = scored.pose.yaw;
    out.grasps_admitted = static_cast<uint32_t>(scored.admitted);
    out.grasps_total    = static_cast<uint32_t>(offered);
    out.travel          = std::hypot(std::hypot(scored.pose.x, scored.pose.y), scored.pose.z);
    out.score           = scored.score;
    return out;
}

void ParkNode::onPark(const msgs::ParkGoalConstPtr &goal) {
    msgs::ParkResult result;

    std::vector<Grasp> grasps;
    grasps.reserve(goal->cloud.candidates.size());
    for (const msgs::GraspPose &pose : goal->cloud.candidates) {
        Grasp grasp;
        grasp.point    = Eigen::Vector3d(pose.point.x, pose.point.y, pose.point.z);
        grasp.bar_axis = Eigen::Vector3d(pose.bar_axis.x, pose.bar_axis.y, pose.bar_axis.z);
        grasps.push_back(grasp);
    }
    if (goal->cloud.header.frame_id != config_.base_frame) {
        result.success = false;
        result.message = "the cloud is in '" + goal->cloud.header.frame_id + "' but the arm plans in '"
                         + config_.base_frame + "'";
        LOG_WARN("[park] %s", result.message.c_str());
        server_.setAborted(result);
        return;
    }

    // The candidates arrive in arm_base. The search moves the body, so bring them into the
    // body frame once here and let the search work in the frame it commands.
    const Eigen::Isometry3d arm_to_body = body_to_arm_.inverse();
    for (Grasp &grasp : grasps) {
        grasp.point    = arm_to_body * grasp.point;
        grasp.bar_axis = arm_to_body.linear() * grasp.bar_axis;
    }

    Scored stayed = search_->scoreOne(grasps, Pose());

    msgs::ParkFeedback feedback;
    feedback.stage = "searching";
    server_.publishFeedback(feedback);

    std::vector<Scored> shortlist;
    size_t              scored_count = 0;
    search_->shortlist(grasps, static_cast<size_t>(config_.verify_count), shortlist, scored_count);

    // Reach says the grasp point can be placed; it says nothing about the arm that has to get
    // there. Maximising reach alone drives the vehicle as deep into the scene as it can and
    // buries the arm in the very obstacles the handle sits on, so the shortlist is re-ranked
    // by what actually survives a full body collision check.
    Scored     chosen;
    bool       found = false;
    int        best_held = 0;
    const kine::JointAngles home = currentJoints();
    std::unique_ptr<planner::ObstacleGrid> grid;
    try {
        grid.reset(new planner::ObstacleGrid(goal->cloud.obstacles, config_.link_radius,
                                             (goal->cloud.obstacles.voxel_size_m + config_.blade_sample_step)
                                                     * std::sqrt(3.0) / 2.0));
    } catch (const std::invalid_argument &e) {
        result.success = false;
        result.message = std::string("the obstacle map is not usable: ") + e.what();
        LOG_WARN("[park] %s", result.message.c_str());
        server_.setAborted(result);
        return;
    }
    Pose before;
    {
        std::lock_guard<std::mutex> lock(where_mutex_);
        before = where_;
    }

    const ros::Time verify_started = ros::Time::now();
    size_t          screened       = 0;

    // Staying put is an option like any other and has to be checked the same way, or park can
    // talk itself into moving to a pose that holds fewer candidates than the one it is on.
    std::vector<Scored> options;
    options.push_back(search_->scoreOne(grasps, Pose()));
    options.insert(options.end(), shortlist.begin(), shortlist.end());

    std::vector<std::pair<int, Scored>> ranked;
    for (const Scored &option : options) {
        if ((ros::Time::now() - verify_started).toSec() > config_.verify_budget) {
            break;
        }
        ++screened;
        const Verdict seen = verify(grasps, option.pose, *grid, home, static_cast<size_t>(config_.verify_stride),
                                    false);
        if (seen.held > 0) {
            ranked.push_back(std::make_pair(seen.held, option));
        }
        if (screened % 100 == 0) {
            msgs::ParkFeedback progress;
            progress.stage        = "verifying";
            progress.poses_scored = static_cast<uint32_t>(screened);
            progress.best_score   = ranked.empty() ? 0.0 : ranked.front().first;
            server_.publishFeedback(progress);
        }
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const std::pair<int, Scored> &a, const std::pair<int, Scored> &b) { return a.first > b.first; });

    // Holding a candidate is not the same as being able to get to it. Rank what survives by
    // how many grasps the arm can reach in a straight line, because those are the ones the
    // planner turns into a path; a pose with more holds and no route is the no_path case.
    // Routes are traced at the screening stride: tracing a whole line at full blade fidelity
    // costs more than every other check in this node put together.
    const size_t exact = std::min<size_t>(static_cast<size_t>(config_.verify_exact), ranked.size());
    size_t       blocked_transit = 0;
    int          best_routable = -1;
    best_held                  = -1;
    for (size_t i = 0; i < exact; ++i) {
        // Holds at full fidelity, because that is the number the planner will reproduce.
        // Routes at the screening stride, because tracing a whole line exactly costs more than
        // every other check in this node together, and a missed contact only loses a route.
        // Rejected outright, never traded off: driving there rakes the arm through the scene.
        if (!transitClear(before, ranked[i].second.pose, *grid, home,
                          static_cast<size_t>(config_.verify_stride))) {
            ++blocked_transit;
            continue;
        }
        const Verdict solid = verify(grasps, ranked[i].second.pose, *grid, home, 1, false);
        if (solid.held == 0) {
            continue;
        }
        const Verdict route =
                verify(grasps, ranked[i].second.pose, *grid, home, static_cast<size_t>(config_.verify_stride), true);
        if (route.routable > best_routable || (route.routable == best_routable && solid.held > best_held)) {
            best_routable   = route.routable;
            best_held       = solid.held;
            chosen          = ranked[i].second;
            chosen.admitted = solid.held;
            found           = true;
        }
    }

    const Verdict stay = verify(grasps, Pose(), *grid, home, 1, true);
    LOG_INFO("[park] screened %zu poses in %.1f s, %zu could hold something, rechecked %zu; "
             "staying holds %d routes to %d, chosen holds %d routes to %d",
             screened, (ros::Time::now() - verify_started).toSec(), ranked.size(), exact, stay.held, stay.routable,
             best_held, best_routable);
    if (blocked_transit > 0) {
        LOG_INFO("[park] %zu of the %zu best poses were refused because the drive would have put the arm through "
                 "the scene",
                 blocked_transit, exact);
    }
    if (found && best_routable == 0) {
        LOG_WARN("[park] nothing in the box has a straight route from home; the planner will have to search for one");
    }
    if (stay.routable > best_routable || (stay.routable == best_routable && stay.held > best_held)) {
        LOG_INFO("[park] staying put is no worse than anything found, so the vehicle does not move");
        chosen          = stayed;
        chosen.pose     = Pose();
        chosen.admitted = stay.held;
        found           = stay.held > 0;
    }

    stayed.admitted = stay.held;
    result.stayed   = toMessage(stayed, grasps.size());
    result.chosen = toMessage(chosen, grasps.size());
    pub_result_.publish(result.chosen);

    // Moving is the one thing here that cannot be undone: park somewhere unworkable and the
    // arm is stuck looking at it. So the bar to move is a route, not a grasp. Short of that,
    // stay exactly where we are and say so; the caller can collect again and ask afresh, and
    // because nothing moved, asking again is free.
    if (found && best_routable < config_.min_routable) {
        result.success = false;
        result.message = "staying put: the best pose holds " + std::to_string(best_held) + " of "
                         + std::to_string(grasps.size()) + " candidates but can route to "
                         + std::to_string(best_routable) + ", short of " + std::to_string(config_.min_routable);
        LOG_WARN("[park] %s", result.message.c_str());
        server_.setSucceeded(result);
        return;
    }
    if (!found) {

        result.success = false;
        result.message = shortlist.empty()
                                 ? "no pose in the box brings any of the " + std::to_string(grasps.size())
                                           + " candidates within reach"
                                 : "reach found " + std::to_string(shortlist.size())
                                           + " possible poses, but the obstacle map blocked the arm at every one "
                                             "that was checked";
        LOG_WARN("[park] %s, %zu poses scored", result.message.c_str(), scored_count);
        server_.setAborted(result);
        return;
    }

    LOG_INFO("[park] %zu poses scored: staying reaches %d of %zu, moving (%.3f %.3f %.3f, %.1f deg) reaches %d",
             scored_count, stayed.admitted, grasps.size(), chosen.pose.x, chosen.pose.y, chosen.pose.z,
             kine::radToDeg(chosen.pose.yaw), chosen.admitted);

    // Latch the snapshot where it was taken, before anything moves. The candidates keep the
    // numbers cloud gave them; only the label changes, because the scene frame and the arm
    // base are the same place at this instant.
    lockScene(before);
    msgs::CloudResult locked = goal->cloud;
    locked.header.frame_id   = config_.scene_frame;
    locked.obstacles.header  = locked.header;
    pub_locked_cloud_.publish(locked);
    result.locked = locked;

    // The search answers with a displacement from where the vehicle stands, while walkTo and
    // the broadcast talk in locked_frame. They are the same thing only while the vehicle is
    // still at the origin, so compose before driving or the second pick goes somewhere else.
    Pose target;
    const double cos_before = std::cos(before.yaw);
    const double sin_before = std::sin(before.yaw);
    target.x   = before.x + cos_before * chosen.pose.x - sin_before * chosen.pose.y;
    target.y   = before.y + sin_before * chosen.pose.x + cos_before * chosen.pose.y;
    target.z   = before.z + chosen.pose.z;
    target.yaw = before.yaw + chosen.pose.yaw;

    std::string why;
    if (!walkTo(target, why)) {
        result.success = false;
        result.message = why;
        server_.setPreempted(result);
        return;
    }

    // Dead reckoning is imperfect, so the snapshot ends up slightly in the wrong place. Slip
    // the scene frame rather than the body: the vehicle believes it arrived exactly, and it is
    // the locked scene that is wrong, which is what drift actually does to you.
    const double travelled =
            std::hypot(std::hypot(chosen.pose.x, chosen.pose.y), chosen.pose.z);
    if (config_.drift_per_metre > 0.0 || config_.drift_yaw_per_metre > 0.0) {
        const double slip     = config_.drift_per_metre * travelled;
        const double slip_yaw = config_.drift_yaw_per_metre * travelled;
        std::lock_guard<std::mutex> lock(scene_mutex_);
        Eigen::Isometry3d           error = Eigen::Isometry3d::Identity();
        error.linear()      = Eigen::AngleAxisd(slip_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        error.translation() = Eigen::Vector3d(slip, 0.0, 0.0);
        locked_to_scene_    = error * locked_to_scene_;
        LOG_WARN("[park] the locked scene slipped %.1f mm and %.2f deg over %.3f m driven", slip * 1000.0,
                 kine::radToDeg(slip_yaw), travelled);
    }

    result.success = true;
    result.message = "parked, " + std::to_string(chosen.admitted) + " of " + std::to_string(grasps.size())
                     + " candidates in reach";
    server_.setSucceeded(result);
}

}  // namespace park
