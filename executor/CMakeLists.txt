cmake_minimum_required(VERSION 3.0.2)
project(executor)

set(EXTRA_CATKIN_PROJECTS params kine msgs actionlib actionlib_msgs std_srvs)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

compile_my_exec(executor_node
    src/main.cpp
    src/ExecutorNode.cpp
    src/ExecutorConfig.cpp
    src/PathFollower.cpp)
target_link_libraries(executor_node pthread)
