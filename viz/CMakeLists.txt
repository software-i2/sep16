cmake_minimum_required(VERSION 3.0.2)
project(viz)

set(EXTRA_CATKIN_PROJECTS params kine msgs)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

compile_my_exec(viz_node
    src/main.cpp
    src/VizNode.cpp
    src/VizConfig.cpp)
target_link_libraries(viz_node pthread)
