cmake_minimum_required(VERSION 2.8)

SET(MOLE 13)

execute_process(
#	COMMAND if [ -f ROSBUILD_FOR_CANOPEN ]; then exit 1; fi
#	COMMAND if [ -f ROSBUILD_FOR_CANOPEN ]; then exit 1; fi
	COMMAND ls 7
	OUTPUT_VARIABLE MOLE
)

IF(${MOLE} EQUAL 7)
  MESSAGE("Building canopen for ROS.")
  include(CMakeLists_ROS.txt)
  execute_process(COMMAND rm 7)
  
ELSE()
  include(CMakeLists_Core.txt)
  MESSAGE("Building canopen core.")
ENDIF(${MOLE} EQUAL 7)
