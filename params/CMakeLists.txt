cmake_minimum_required(VERSION 3.0.2)
project(params)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
