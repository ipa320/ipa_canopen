add_executable(homing homing.cpp)
target_link_libraries(homing pthread canopen pcan)

add_executable(move_device move_device.cpp)
target_link_libraries(move_device pthread canopen pcan)

#add_executable(status status.cpp)
#target_link_libraries(status pthread canopen pcan)
