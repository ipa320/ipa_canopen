add_executable(homing homing.cpp)
target_link_libraries(homing pthread canopen pcan)

add_executable(move_device move_device.cpp)
target_link_libraries(move_device pthread canopen pcan)

add_executable(get_error get_error.cpp)
target_link_libraries(get_error pthread canopen pcan)

add_executable(elmo_move_device elmo_move_device.cpp)
target_link_libraries(elmo_move_device pthread canopen pcan)

add_executable(elmo_mapping elmo_mapping.cpp)
target_link_libraries(elmo_mapping pthread canopen pcan)

add_executable(elmo_position elmo_position.cpp)
target_link_libraries(elmo_position pthread canopen pcan)
