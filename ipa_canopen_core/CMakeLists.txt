
cmake_minimum_required(VERSION 2.8.3)
project(ipa_canopen_core)

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

find_package(catkin REQUIRED COMPONENTS roscpp libpcan)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS orocos_kdl
    CATKIN_DEPENDS roscpp libpcan
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories ("${PROJECT_SOURCE_DIR}/driver")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory (driver)
add_subdirectory (tools)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs" )
