cmake_minimum_required(VERSION 3.0.2)
project(description)

find_package(catkin REQUIRED)
catkin_package()

install(FILES alpha.urdf.xacro LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf xacro meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
