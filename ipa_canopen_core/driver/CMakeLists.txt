
ADD_LIBRARY( canopen STATIC
  canopen.cpp
  )

install (TARGETS canopen 
#  LIBRARY DESTINATION lib)
  ARCHIVE DESTINATION lib)

install (FILES 
  canopen.h
  DESTINATION include)

