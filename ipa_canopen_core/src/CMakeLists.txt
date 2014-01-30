add_library(canopen SHARED
  canopen.cpp
  )
set_target_properties(canopen PROPERTIES LINKER_LANGUAGE CXX)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    
install (TARGETS canopen 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install (FILES 
  canopen.h
  schunkErrors.h
  DESTINATION include)

