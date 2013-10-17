cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
#rosbuild_add_executable(dummycontrollernode_single_device src/dummycontrollernode_single_device.cpp)
#rosbuild_add_executable(sd_controller src/sd_controller.cpp)
#rosbuild_add_executable(md_controller src/md_controller.cpp)
#rosbuild_add_executable(dummycontrollernode_multiple_devices src/dummycontrollernode_multiple_devices.cpp)
include_directories(include)
rosbuild_add_executable(canopen_ros src/canopen_ros.cpp src/JointLimits.cpp include/JointLimits.h)
rosbuild_add_executable(canopen_ros_elmo src/canopen_ros_elmo.cpp src/JointLimits.cpp include/JointLimits.h)
#rosbuild_add_executable(ros_canopenmasternode1 src/ros_canopenmasternode1.cpp)
# target_link_libraries(ros_canopen boost_system boost_filesystem pthread canopen pcan yaml-cpp)
target_link_libraries(canopen_ros boost_system pthread canopen pcan)
target_link_libraries(canopen_ros_elmo boost_system pthread canopen pcan)
#target_link_libraries(ros_canopenmasternode1 boost_system boost_filesystem pthread canopen pcan yaml-cpp)
