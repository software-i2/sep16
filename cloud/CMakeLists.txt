cmake_minimum_required(VERSION 3.0.2)
project(cloud)

set(EXTRA_CATKIN_PROJECTS params kine msgs actionlib actionlib_msgs tf2_ros)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

compile_my_exec(cloud_node
    src/main.cpp
    src/CloudNode.cpp
    src/CloudConfig.cpp
    src/CloudPipeline.cpp
    src/VoxelGrid.cpp
    src/DepthImage.cpp
    src/OutlierFilter.cpp
    src/Occupancy.cpp
    src/Regions.cpp
    src/HandleClassifier.cpp
    src/CurvatureClassifier.cpp
    src/CandidateAveraging.cpp
    src/ConsensusAveraging.cpp)
target_link_libraries(cloud_node pthread)
