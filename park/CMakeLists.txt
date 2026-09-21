cmake_minimum_required(VERSION 3.0.2)
project(park)

set(EXTRA_CATKIN_PROJECTS params kine msgs actionlib actionlib_msgs tf2_ros)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# BEEX_CMAKE's catkin_package exports neither headers nor libraries, so planner's admission
# half is compiled in from source rather than linked. These three carry no ROS dependency;
# taking a copy of them instead would let park and the planner quietly disagree.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../planner/include)

compile_my_exec(park_node
    src/main.cpp
    src/ParkNode.cpp
    src/ParkConfig.cpp
    src/PoseSearch.cpp
    ../planner/src/GraspGoals.cpp
    ../planner/src/CollisionChecker.cpp
    ../planner/src/ObstacleGrid.cpp)
target_link_libraries(park_node pthread)
