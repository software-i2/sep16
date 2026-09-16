cmake_minimum_required(VERSION 3.0.2)
project(driver)

set(EXTRA_CATKIN_PROJECTS params std_srvs)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

compile_my_exec(driver_node
    src/main.cpp
    src/DriverNode.cpp
    src/DriverConfig.cpp
    src/ArmLink.cpp
    src/SimulatedArm.cpp
    src/SerialPort.cpp
    src/Protocol.cpp)
target_link_libraries(driver_node pthread)
