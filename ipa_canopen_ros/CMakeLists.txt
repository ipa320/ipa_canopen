cmake_minimum_required(VERSION 2.8.3)
project(ipa_canopen_ros)

find_package(catkin REQUIRED COMPONENTS roscpp brics_actuator ipa_canopen_core urdf)
#######################################
## Declare ROS messages and services ##
#######################################

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS ipa_canopen_core
    CATKIN_DEPENDS roscpp brics_actuator ipa_canopen_core
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(canopen_ros src/canopen_ros.cpp src/JointLimits.cpp)

target_link_libraries(canopen_ros boost_system pthread ipa_canopen_core pcan ${catkin_LIBRARIES})
