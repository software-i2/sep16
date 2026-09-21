cmake_minimum_required(VERSION 3.0.2)
project(msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs actionlib_msgs)

add_message_files(FILES
    GraspPose.msg
    ObstacleMap.msg
    CloudResult.msg
    Waypoint.msg
    JointPath.msg
    GraspPlan.msg
    ParkPose.msg
    TaskState.msg)

add_action_files(FILES
    Collect.action
    Park.action
    Plan.action
    Execute.action)

generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs)
