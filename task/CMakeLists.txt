cmake_minimum_required(VERSION 3.0.2)
project(task)

set(EXTRA_CATKIN_PROJECTS params msgs actionlib actionlib_msgs std_srvs tf2_ros)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

compile_my_exec(task_node
    src/main.cpp
    src/TaskNode.cpp
    src/TaskConfig.cpp
    src/StateMachine.cpp)
target_link_libraries(task_node pthread)
