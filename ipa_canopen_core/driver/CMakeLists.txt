add_library(canopen SHARED
  canopen.cpp
  )
set_target_properties(canopen PROPERTIES LINKER_LANGUAGE CXX)

install (TARGETS canopen 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install (FILES 
  canopen.h
  schunkErrors.h
  DESTINATION include)

