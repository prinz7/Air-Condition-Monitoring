INCLUDE( FindPackageHandleStandardArgs )

# Find BSEC library
FIND_PATH(BSEC_INCLUDE_DIR
  NAMES bsec_interface.h bsec_datatypes.h
  PATHS ${BSEC_PATH}
  REQUIRED
)

if(WIN32)
  set(CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
else()
  set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
endif()


#FIND_LIBRARY(BSEC_LIBRARY
  #NAMES algobsec
 # PATHS ${BSEC_PATH}
  #PATH_SUFFIXES lib
#  REQUIRED
#)

FIND_PACKAGE_HANDLE_STANDARD_ARGS( BSEC DEFAULT_MSG
  BSEC_INCLUDE_DIR
  #BSEC_LIBRARY
)

# Check if both the include directory and library were found
if (BSEC_FOUND)# AND BSEC_LIBRARY)
  SET( BSEC_INCLUDE_DIRS ${BSEC_INCLUDE_DIR} )
  #SET( BSEC_LIBRARIES ${BSEC_LIBRARY} )

  MARK_AS_ADVANCED(
   # BSEC_LIBRARY
    BSEC_INCLUDE_DIR
    BSEC_DIR
  )

  message(STATUS "Found BSEC: ${BSEC_INCLUDE_DIR}")
else ()
  message(FATAL_ERROR "BSEC library not found")
endif ()
