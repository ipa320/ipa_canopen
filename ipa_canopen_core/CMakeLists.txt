cmake_minimum_required(VERSION 2.8.3)
project(ipa_canopen_core)

find_package(catkin REQUIRED COMPONENTS roscpp libpcan)

if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        add_definitions("-std=gnu++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
   add_definitions("-std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

catkin_package(
    DEPENDS #
    CATKIN_DEPENDS roscpp libpcan
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories (include ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/)
#set(INCLUDE_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/)
  


###########
## src ##
###########
add_library(${PROJECT_NAME}
    src/ipa_canopen_core.cpp
  )
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
  
add_executable(${PROJECT_NAME}_homing tools/homing.cpp)
target_link_libraries(${PROJECT_NAME}_homing pthread ipa_canopen_core ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_move_device tools/move_device.cpp)
target_link_libraries(${PROJECT_NAME}_move_device pthread ipa_canopen_core ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_get_error tools/get_error.cpp)
target_link_libraries(${PROJECT_NAME}_get_error pthread ipa_canopen_core ${catkin_LIBRARIES})

#list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs" )


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_homing ${PROJECT_NAME}_move_device ${PROJECT_NAME}_get_error
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
