cmake_minimum_required(VERSION 3.0.2)
project(planner)

set(EXTRA_CATKIN_PROJECTS params kine msgs actionlib actionlib_msgs)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

compile_my_exec(planner_node
    src/main.cpp
    src/PlannerNode.cpp
    src/PlannerConfig.cpp
    src/CandidatePlanner.cpp
    src/GraspGoals.cpp
    src/BiRrtStar.cpp
    src/CollisionChecker.cpp
    src/ObstacleGrid.cpp)
target_link_libraries(planner_node pthread)
