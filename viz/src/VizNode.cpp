// Copyright by BeeX [2026]

#include <kine/ForwardKinematics.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <viz/VizNode.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace viz {
namespace {

geometry_msgs::Point toPoint(const Eigen::Vector3d &v) {
    geometry_msgs::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

Eigen::Vector3d toEigen(const geometry_msgs::Point &p) { return Eigen::Vector3d(p.x, p.y, p.z); }
Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v) { return Eigen::Vector3d(v.x, v.y, v.z); }

uint8_t toByte(float channel) { return static_cast<uint8_t>(std::lround(std::min(std::max(channel, 0.0f), 1.0f) * 255.0f)); }

}  // namespace

VizNode::VizNode(const VizConfig &config)
        : config_(config), body_(kine::ArmModel(config.arm), config.arm.jaw, config.blade_draw_step) {
    INIT_ROS_SUBSCRIBER(sub_joint_states_, config_.topic_joint_states, 1, &VizNode::onJointStates);
    INIT_ROS_SUBSCRIBER(sub_cloud_result_, config_.topic_cloud_result, 1, &VizNode::onCloudResult);
    INIT_ROS_SUBSCRIBER(sub_plan_result_, config_.topic_plan_result, 1, &VizNode::onPlanResult);
    INIT_ROS_PUBLISHER(pub_arm_body_, visualization_msgs::MarkerArray, config_.topic_arm_body, 1);
    pub_floor_        = mainNodeHandle->advertise<visualization_msgs::Marker>(config_.topic_floor, 1, true);
    pub_obstacle_map_ = mainNodeHandle->advertise<sensor_msgs::PointCloud2>(config_.initial.topic_obstacle_map, 1, true);
    pub_grasp_poses_  = mainNodeHandle->advertise<visualization_msgs::Marker>(config_.initial.topic_grasp_poses, 1, true);
    pub_reprocess_map_ =
            mainNodeHandle->advertise<sensor_msgs::PointCloud2>(config_.reprocess.topic_obstacle_map, 1, true);
    pub_reprocess_poses_ =
            mainNodeHandle->advertise<visualization_msgs::Marker>(config_.reprocess.topic_grasp_poses, 1, true);
    pub_chosen_grasp_ = mainNodeHandle->advertise<visualization_msgs::MarkerArray>(config_.topic_chosen_grasp, 1, true);
    pub_planned_path_ = mainNodeHandle->advertise<visualization_msgs::Marker>(config_.topic_planned_path, 1, true);
    drawFloor();
}

visualization_msgs::Marker VizNode::marker(const std::string &name, int type, const std_msgs::ColorRGBA &colour) const {
    visualization_msgs::Marker m;
    m.header.frame_id    = config_.base_frame;
    m.header.stamp       = ros::Time::now();
    m.ns                 = name;
    m.type               = type;
    m.action             = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 1.0;
    m.color = colour;
    return m;
}

bool VizNode::modelJoints(const std::vector<std::string> &names, const std::vector<double> &positions,
                          kine::JointAngles &out) const {
    kine::JointAngles reported;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        const auto   found = std::find(names.begin(), names.end(), config_.joint_names[j]);
        const size_t index = static_cast<size_t>(found - names.begin());
        if (found == names.end() || index >= positions.size()) {
            return false;
        }
        reported[j] = positions[index];
    }
    out = body_.model().toModel(reported);
    return true;
}

void VizNode::onJointStates(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    latest_joints_ = *msg;
}

void VizNode::drawFloor() {
    visualization_msgs::Marker floor = marker("floor", visualization_msgs::Marker::TRIANGLE_LIST, config_.floor_colour);
    const kine::FloorGuard    &guard = config_.floor_guard;
    const double               z     = guard.floor_z;
    const Eigen::Vector3d      corners[4] = {{guard.min_x, guard.min_y, z},
                                             {guard.max_x, guard.min_y, z},
                                             {guard.max_x, guard.max_y, z},
                                             {guard.min_x, guard.max_y, z}};
    for (const int corner : {0, 1, 2, 0, 2, 3}) {
        floor.points.push_back(toPoint(corners[corner]));
    }
    PUBLISH_ROS(pub_floor_, floor);
}

void VizNode::drawArmBody() {
    sensor_msgs::JointState joints;
    {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        joints = latest_joints_;
    }
    kine::JointAngles model;
    if (!modelJoints(joints.name, joints.position, model)) {
        return;
    }
    kine::BodyPose pose;
    body_.pose(model, pose);

    visualization_msgs::MarkerArray array;
    const double                    diameter = 2.0 * config_.link_radius;
    for (int link = 0; link < kine::BODY_LINK_COUNT; ++link) {
        const kine::Segment  &segment = pose.links[link];
        const Eigen::Vector3d span    = segment.end - segment.start;
        const std::string     name    = kine::BODY_LINK_NAMES[link];

        visualization_msgs::Marker tube = marker(name, visualization_msgs::Marker::CYLINDER, config_.arm_body_colour);
        const Eigen::Quaterniond   turn = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),
                                                                           span.norm() > 0.0 ? span : Eigen::Vector3d::UnitZ());
        tube.pose.position    = toPoint(0.5 * (segment.start + segment.end));
        tube.pose.orientation.x = turn.x();
        tube.pose.orientation.y = turn.y();
        tube.pose.orientation.z = turn.z();
        tube.pose.orientation.w = turn.w();
        tube.scale.x = tube.scale.y = diameter;
        tube.scale.z                = span.norm();
        array.markers.push_back(tube);

        visualization_msgs::Marker ends =
                marker(name + "_ends", visualization_msgs::Marker::SPHERE_LIST, config_.arm_body_colour);
        ends.scale.x = ends.scale.y = ends.scale.z = diameter;
        ends.points                                = {toPoint(segment.start), toPoint(segment.end)};
        array.markers.push_back(ends);
    }

    visualization_msgs::Marker blades = marker("blades", visualization_msgs::Marker::SPHERE_LIST, config_.arm_body_colour);
    blades.scale.x = blades.scale.y = blades.scale.z = config_.blade_draw_step;
    for (const Eigen::Vector3d &point : pose.blade_points) {
        blades.points.push_back(toPoint(point));
    }
    array.markers.push_back(blades);
    PUBLISH_ROS(pub_arm_body_, array);
}

void VizNode::onCloudResult(const msgs::CloudResult::ConstPtr &msg) {
    if (msg->fresh) {
        drawCloud(*msg, config_.reprocess, pub_reprocess_poses_, pub_reprocess_map_);
        return;
    }
    drawCloud(*msg, config_.initial, pub_grasp_poses_, pub_obstacle_map_);
    // A new survey starts a new pick, so whatever the last park re-measured is no longer the scene.
    drawCloud(msgs::CloudResult(), config_.reprocess, pub_reprocess_poses_, pub_reprocess_map_);
}

void VizNode::drawCloud(const msgs::CloudResult &msg, const CloudView &view, ros::Publisher &poses_pub,
                        ros::Publisher &map_pub) {
    // Drawn in the frame the cloud says it is in, not the arm base. Once the vehicle can move,
    // a snapshot latched at the old pose and the live view are in different frames, and
    // stamping both with the arm base would draw them on top of each other.
    const std::string &frame = msg.header.frame_id.empty() ? config_.base_frame : msg.header.frame_id;

    visualization_msgs::Marker poses = marker("grasp_poses", visualization_msgs::Marker::SPHERE_LIST, view.grasp_pose);
    poses.header.frame_id = frame;
    poses.scale.x = poses.scale.y = poses.scale.z = config_.grasp_pose_size;
    for (const msgs::GraspPose &pose : msg.poses) {
        poses.points.push_back(pose.point);
    }
    if (poses.points.empty()) {
        poses.action = visualization_msgs::Marker::DELETEALL;
    }
    PUBLISH_ROS(poses_pub, poses);

    const msgs::ObstacleMap &map = msg.obstacles;
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame;
    cloud.header.stamp    = ros::Time::now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(map.obstacle_cells.size() + map.handle_cells.size());

    sensor_msgs::PointCloud2Iterator<float>   x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>   y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>   z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> b(cloud, "b");

    const auto addCells = [&](const std::vector<uint32_t> &cells, const std_msgs::ColorRGBA &colour) {
        const uint64_t layer = static_cast<uint64_t>(map.size_x) * map.size_y;
        for (uint32_t cell : cells) {
            const double cx = cell % map.size_x;
            const double cy = (cell / map.size_x) % map.size_y;
            const double cz = static_cast<double>(cell / layer);
            *x = static_cast<float>(map.origin.x + (cx + 0.5) * map.voxel_size_m);
            *y = static_cast<float>(map.origin.y + (cy + 0.5) * map.voxel_size_m);
            *z = static_cast<float>(map.origin.z + (cz + 0.5) * map.voxel_size_m);
            *r = toByte(colour.r);
            *g = toByte(colour.g);
            *b = toByte(colour.b);
            ++x, ++y, ++z, ++r, ++g, ++b;
        }
    };
    addCells(map.obstacle_cells, view.obstacle);
    addCells(map.handle_cells, view.handle_cell);
    PUBLISH_ROS(map_pub, cloud);
}

void VizNode::onPlanResult(const msgs::GraspPlan::ConstPtr &msg) {
    visualization_msgs::MarkerArray chosen;
    visualization_msgs::Marker      path =
            marker("planned_path", visualization_msgs::Marker::LINE_STRIP, config_.planned_path_colour);
    path.scale.x = config_.line_width;

    if (!msg->success) {
        visualization_msgs::Marker clear = marker("chosen_grasp", visualization_msgs::Marker::ARROW, config_.chosen_grasp_colour);
        clear.action                     = visualization_msgs::Marker::DELETEALL;
        chosen.markers.push_back(clear);
        path.action = visualization_msgs::Marker::DELETEALL;
        PUBLISH_ROS(pub_chosen_grasp_, chosen);
        PUBLISH_ROS(pub_planned_path_, path);
        return;
    }

    const Eigen::Vector3d point    = toEigen(msg->chosen.point);
    const Eigen::Vector3d approach = toEigen(msg->chosen.approach).normalized();
    const Eigen::Vector3d bar      = toEigen(msg->chosen.bar_axis).normalized();

    visualization_msgs::Marker arrow =
            marker("chosen_grasp_approach", visualization_msgs::Marker::ARROW, config_.chosen_grasp_colour);
    arrow.points  = {toPoint(point - approach * config_.chosen_grasp_length), toPoint(point)};
    arrow.scale.x = config_.line_width;
    arrow.scale.y = config_.grasp_pose_size;
    arrow.scale.z = 0.0;
    chosen.markers.push_back(arrow);

    visualization_msgs::Marker bar_line =
            marker("chosen_grasp_bar", visualization_msgs::Marker::LINE_LIST, config_.chosen_grasp_colour);
    bar_line.points  = {toPoint(point - bar * config_.chosen_grasp_length / 2.0),
                        toPoint(point + bar * config_.chosen_grasp_length / 2.0)};
    bar_line.scale.x = config_.line_width;
    chosen.markers.push_back(bar_line);
    PUBLISH_ROS(pub_chosen_grasp_, chosen);

    std::vector<kine::JointAngles> corners;
    for (const msgs::Waypoint &waypoint : msg->path.waypoints) {
        kine::JointAngles model;
        if (modelJoints(msg->path.joint_names, waypoint.positions, model)) {
            corners.push_back(model);
        }
    }
    for (size_t k = 0; k < corners.size(); ++k) {
        if (k == 0) {
            path.points.push_back(toPoint(kine::forwardKinematics(body_.model(), corners[0]).throat));
            continue;
        }
        double largest = 0.0;
        for (int j = 0; j < kine::JOINT_COUNT; ++j) {
            largest = std::max(largest, std::fabs(corners[k][j] - corners[k - 1][j]));
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(largest / config_.path_draw_step)));
        for (int s = 1; s <= steps; ++s) {
            kine::JointAngles joints;
            for (int j = 0; j < kine::JOINT_COUNT; ++j) {
                joints[j] = corners[k - 1][j] + (corners[k][j] - corners[k - 1][j]) * s / steps;
            }
            path.points.push_back(toPoint(kine::forwardKinematics(body_.model(), joints).throat));
        }
    }
    PUBLISH_ROS(pub_planned_path_, path);
}

}  // namespace viz
