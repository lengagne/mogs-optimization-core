# - Try to find ADOLC
# Once done this will define
#  ADOLC_FOUND - System has ADOLC
#  ADOLC_INCLUDE_DIRS - The ADOLC include directories
#  ADOLC_LIBRARY_DIRS - The library directories needed to use ADOLC
#  ADOLC_LIBRARIES    - The libraries needed to use ADOLC


if (ADOLC_INCLUDE_DIR)
  # in cache already
  SET(ADOLC_FIND_QUIETLY TRUE)
endif (ADOLC_INCLUDE_DIR)

find_path(ADOLC_INCLUDE_DIR NAMES adolc.h
PATHS  "$ENV{ADOLC_HOME}/include/adolc"
	"/usr/local/include/adolc"

)
find_path(ADOLC_INCLUDE_DIR NAMES adouble.h
PATHS  "$ENV{ADOLC_HOME}/include/adolc"
	"/usr/local/include/adolc"

)
find_library( ADOLC_LIBRARY
		adolc
		PATHS "$ENV{ADOLC_HOME}/lib64"
		"/usr/local/lib64")


set(ADOLC_INCLUDE_DIRS "${ADOLC_INCLUDE_DIR}" "${ADOLC_INCLUDE_DIR}/../" )
set(ADOLC_LIBRARIES ${ADOLC_LIBRARY} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBCPLEX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ADOLC  DEFAULT_MSG
				ADOLC_LIBRARY ADOLC_INCLUDE_DIR)

mark_as_advanced(ADOLC_INCLUDE_DIR ADOLC_LIBRARY )
