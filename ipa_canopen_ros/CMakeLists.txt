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

add_executable(${PROJECT_NAME} src/ipa_canopen_ros.cpp src/JointLimits.cpp)

target_link_libraries(${PROJECT_NAME} boost_system pthread pcan ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
