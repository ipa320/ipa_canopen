add_executable(homing homing.cpp)
target_link_libraries(homing pthread canopen ${catkin_LIBRARIES})

add_executable(move_device move_device.cpp)
target_link_libraries(move_device pthread canopen ${catkin_LIBRARIES})

add_executable(get_error get_error.cpp)
target_link_libraries(get_error pthread canopen ${catkin_LIBRARIES})
