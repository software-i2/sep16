// Copyright by BeeX [2026]

#include <cloud/CloudNode.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <chrono>
#include <thread>

namespace cloud {
namespace {

constexpr auto kWaitPollPeriod = std::chrono::milliseconds(20);

}  // namespace

CloudNode::CloudNode(const CloudConfig &config, const CloudPipeline &pipeline)
        : config_(config),
          pipeline_(pipeline),
          server_(*mainNodeHandle, config.action_collect, boost::bind(&CloudNode::onCollect, this, _1), false),
          tf_listener_(tf_buffer_) {
    pub_result_ = mainNodeHandle->advertise<msgs::CloudResult>(config_.topic_result, 1, true);
    server_.start();
}

void CloudNode::onCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(frames_mutex_);
    latest_cloud_ = msg;
    pairLatest();
}

void CloudNode::onGraspPoses(const geometry_msgs::PoseArray::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(frames_mutex_);
    latest_poses_ = msg;
    pairLatest();
}

void CloudNode::pairLatest() {
    if (!latest_cloud_ || !latest_poses_ || latest_cloud_->header.stamp != latest_poses_->header.stamp) {
        return;
    }
    CameraFrame frame;
    std::string why;
    if (toFrame(*latest_cloud_, *latest_poses_, frame, why)) {
        frames_.push_back(std::move(frame));
        ++arrived_since_window_;
        while (frames_.size() > static_cast<size_t>(config_.frames_to_collect)) {
            frames_.pop_front();
        }
    } else {
        frame_problem_ = why;
    }
    latest_cloud_.reset();
    latest_poses_.reset();
}

bool CloudNode::toFrame(const sensor_msgs::PointCloud2 &cloud, const geometry_msgs::PoseArray &poses,
                        CameraFrame &frame, std::string &why) {
    if (cloud.header.frame_id != config_.camera_frame || poses.header.frame_id != config_.camera_frame) {
        why = "the camera publishes in '" + cloud.header.frame_id + "' but camera.yaml says '" + config_.camera_frame
              + "'";
        return false;
    }

    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(config_.base_frame, config_.camera_frame, cloud.header.stamp,
                                               ros::Duration(config_.transform_wait_s));
    } catch (const tf2::TransformException &e) {
        why = std::string("no transform from the camera to the arm base: ") + e.what();
        return false;
    }
    const geometry_msgs::Vector3    &t = transform.transform.translation;
    const geometry_msgs::Quaternion &q = transform.transform.rotation;
    frame.camera_to_base = Eigen::Translation3d(t.x, t.y, t.z) * Eigen::Quaterniond(q.w, q.x, q.y, q.z);

    try {
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
        frame.points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
        for (; x != x.end(); ++x, ++y, ++z) {
            if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
                frame.points.emplace_back(*x, *y, *z);
            }
        }
    } catch (const std::runtime_error &e) {
        why = std::string("the cloud has no float x, y, z fields: ") + e.what();
        return false;
    }

    for (const geometry_msgs::Pose &pose : poses.poses) {
        const Eigen::Matrix3d rotation =
                Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                        .normalized()
                        .toRotationMatrix();
        frame.poses.push_back({Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z),
                               rotation.col(config_.bar_column), rotation.col(config_.approach_column)});
    }
    return true;
}

void CloudNode::finishFailed(const std::string &why) {
    msgs::CollectResult result;
    result.cloud.header.stamp    = ros::Time::now();
    result.cloud.header.frame_id = config_.base_frame;
    result.cloud.success         = false;
    result.cloud.summary         = why;
    LOG_ERROR("[cloud] %s", why.c_str());
    PUBLISH_ROS(pub_result_, result.cloud);
    server_.setAborted(result, why);
}

void CloudNode::onCollect(const msgs::CollectGoalConstPtr & /*goal*/) {
    const bool   averaging = config_.switches.candidate_averaging || config_.switches.obstacle_averaging;
    const size_t needed    = averaging ? static_cast<size_t>(config_.frames_to_collect) : 1;
    const size_t advance   = std::min(static_cast<size_t>(config_.frames_to_advance), needed);

    // Frames buffered across a long gap saw the arm moving through the view, so none of them can be reused.
    const bool reuse = !window_served_.isZero()
                       && ros::Time::now() - window_served_ <= ros::Duration(config_.max_reuse_gap_s);
    {
        std::lock_guard<std::mutex> lock(frames_mutex_);
        frame_problem_.clear();
        if (!reuse) {
            frames_.clear();
            arrived_since_window_ = 0;
        }
    }
    if (!subscribed_) {
        INIT_ROS_SUBSCRIBER(sub_cloud_, config_.topic_cloud, 1, &CloudNode::onCloud);
        INIT_ROS_SUBSCRIBER(sub_grasp_poses_, config_.topic_grasp_poses, 1, &CloudNode::onGraspPoses);
        subscribed_ = true;
    }
    LOG_INFO("[cloud] window of %zu frame(s), %zu of them new", needed, reuse ? advance : needed);

    msgs::CollectFeedback feedback;
    feedback.stage     = msgs::CollectFeedback::COLLECTING;
    const ros::Time deadline = ros::Time::now() + ros::Duration(config_.frame_timeout_s);
    std::vector<CameraFrame> frames;
    std::string              problem;
    for (;;) {
        {
            std::lock_guard<std::mutex> lock(frames_mutex_);
            feedback.frames_collected = static_cast<uint32_t>(frames_.size());
            problem                   = frame_problem_;
            // The window is served once it is full and enough of it has been replaced since the last one.
            if (frames_.size() >= needed && arrived_since_window_ >= advance) {
                frames.assign(frames_.end() - static_cast<long>(needed), frames_.end());
                arrived_since_window_ = 0;
                window_served_        = ros::Time::now();
                break;
            }
        }
        server_.publishFeedback(feedback);
        if (server_.isPreemptRequested() || !ros::ok()) {
            LOG_WARN("[cloud] collection cancelled");
            server_.setPreempted();
            return;
        }
        if (ros::Time::now() > deadline) {
            finishFailed("only " + std::to_string(feedback.frames_collected) + " of " + std::to_string(needed)
                         + " frames buffered within " + std::to_string(config_.frame_timeout_s) + " s"
                         + (problem.empty() ? "" : "; last problem: " + problem));
            return;
        }
        std::this_thread::sleep_for(kWaitPollPeriod);
    }

    feedback.stage            = msgs::CollectFeedback::PROCESSING;
    feedback.frames_collected = static_cast<uint32_t>(frames.size());
    server_.publishFeedback(feedback);

    const auto        started = std::chrono::steady_clock::now();
    const CloudOutput output  = pipeline_.process(frames);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    LOG_INFO("[cloud] %s (%.2f s)", output.summary.c_str(), seconds);

    msgs::CollectResult result;
    result.cloud = toMessage(output, frames.size());
    PUBLISH_ROS(pub_result_, result.cloud);
    server_.setSucceeded(result, output.summary);
}

msgs::CloudResult CloudNode::toMessage(const CloudOutput &output, size_t frames_used) const {
    const auto toPose = [](const GraspPose &pose) {
        msgs::GraspPose msg;
        msg.point.x    = pose.point.x();
        msg.point.y    = pose.point.y();
        msg.point.z    = pose.point.z();
        msg.bar_axis.x = pose.bar_axis.x();
        msg.bar_axis.y = pose.bar_axis.y();
        msg.bar_axis.z = pose.bar_axis.z();
        msg.approach.x = pose.approach.x();
        msg.approach.y = pose.approach.y();
        msg.approach.z = pose.approach.z();
        return msg;
    };

    msgs::CloudResult msg;
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = config_.base_frame;
    msg.success         = true;
    msg.summary         = output.summary;
    msg.frames_used     = static_cast<uint32_t>(frames_used);
    for (const GraspPose &pose : output.handle_poses) {
        msg.handle_poses.push_back(toPose(pose));
    }
    for (const GraspPose &pose : output.rope_poses) {
        msg.rope_poses.push_back(toPose(pose));
    }
    for (const GraspPose &pose : output.candidates) {
        msg.candidates.push_back(toPose(pose));
    }

    const VoxelGrid &grid          = pipeline_.grid();
    msg.obstacles.header           = msg.header;
    msg.obstacles.voxel_size_m     = grid.voxelSize();
    msg.obstacles.origin.x         = grid.origin().x();
    msg.obstacles.origin.y         = grid.origin().y();
    msg.obstacles.origin.z         = grid.origin().z();
    msg.obstacles.size_x           = static_cast<uint32_t>(grid.size());
    msg.obstacles.size_y           = static_cast<uint32_t>(grid.size());
    msg.obstacles.size_z           = static_cast<uint32_t>(grid.size());
    msg.obstacles.obstacle_cells   = output.obstacle_cells;
    msg.obstacles.handle_cells     = output.handle_cells;
    return msg;
}

}  // namespace cloud
