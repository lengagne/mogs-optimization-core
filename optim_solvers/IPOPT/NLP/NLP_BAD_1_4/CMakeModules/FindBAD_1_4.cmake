# - Try to find BAD_1_4
# Once done this will define
#  BAD_1_4_FOUND - System has BAD_1_4
#  BAD_1_4_INCLUDE_DIRS - The BAD_1_4 include directories
#  BAD_1_4_LIBRARY_DIRS - The library directories needed to use BAD_1_4
#  BAD_1_4_LIBRARIES    - The libraries needed to use BAD_1_4


if (BAD_1_4_INCLUDE_DIR)
  # in cache already
  SET(BAD_1_4_FIND_QUIETLY TRUE)
endif (BAD_1_4_INCLUDE_DIR)

find_path(BAD_1_4_INCLUDE_DIR NAMES badiff.h
PATHS
    "./FADBAD++"
    "$ENV{BAD_1_4_HOME}/include/FADBAD++"
	"/usr/local/include/FADBAD++"
)


set(BAD_1_4_INCLUDE_DIRS "${BAD_1_4_INCLUDE_DIR}" "${BAD_1_4_INCLUDE_DIR}/../" )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBCPLEX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(BAD_1_4  DEFAULT_MSG  BAD_1_4_INCLUDE_DIR)

mark_as_advanced(BAD_1_4_INCLUDE_DIR)
