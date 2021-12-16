# Find the header files
set(g2o_DIR $ENV{HOME}/Code/catkin_ws/devel/.private/g2o_catkin)
message("srrg_cmake_modules|searching for g2o in: " ${g2o_DIR})

FIND_PATH(g2o_INCLUDE_DIR
  NAMES g2o/config.h
  HINTS ${g2o_DIR} )

if(g2o_INCLUDE_DIR)
  message("srrg_cmake_modules|found g2o:  ${g2o_INCLUDE_DIR}")
  find_library(g2o_LIBRARY g2o
    HINTS ${g2o_DIR}/lib/g2o)
  if(g2o_LIBRARY)
    message("srrg_cmake_modules|g2o LIB:  ${g2o_LIBRARY}")
  else()
    set(g2o_LIBRARY ${g2o_INCLUDE_DIR}/../lib/libg2o.so)
    message("srrg_cmake_modules|g2o LIB not found | bdc HARDCODING")
    message("srrg_cmake_modules|g2o LIB:  ${g2o_LIBRARY}")
  endif()
else()
  message("srrg_cmake_modules|g2o not found | bdc HARDCODING")
  set(g2o_INCLUDE_DIR ${g2o_DIR}/include)
  set(g2o_LIBRARY ${g2o_INCLUDE_DIR}/../build/libg2o.so)
  message("srrg_cmake_modules|g2o INCLUDE:  ${g2o_INCLUDE_DIR}")
  message("srrg_cmake_modules|g2o LIB    :  ${g2o_LIBRARY}")
endif()

set(g2o_DIRS ${g2o_DIR})
set(g2o_INCLUDE_DIRS ${g2o_INCLUDE_DIR})
set(g2o_LIBRARIES ${g2o_LIBRARY})