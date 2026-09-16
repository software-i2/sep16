cmake_minimum_required(VERSION 3.0.2)
project(kine)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES kine DEPENDS EIGEN3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -pedantic)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(kine
    src/ArmModel.cpp
    src/ForwardKinematics.cpp
    src/InverseKinematics.cpp
    src/ArmBody.cpp)

install(TARGETS kine
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
