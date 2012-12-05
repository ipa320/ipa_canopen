cmake_minimum_required (VERSION 2.6) 
project (canopen2) 
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x" ) # ${CMAKE_CXX_FLAGS}" ) # -g -ftest-coverage -fprofile-arcs")
# -fPIC
# Recurse into the "Hello" and "Demo" subdirectories. This does not actually 
# cause another cmake executable to run. The same process will walk through 
# the project's entire directory structure. 
include_directories ("${PROJECT_SOURCE_DIR}/driver")
#link_directories (/usr/lib)
add_subdirectory (driver) 
add_subdirectory (tools)
