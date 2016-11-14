# - Try to find FADBAD_1_4
# Once done this will define
#  FADBAD_1_4_FOUND - System has FADBAD_1_4
#  FADBAD_1_4_INCLUDE_DIRS - The FADBAD_1_4 include directories
#  FADBAD_1_4_LIBRARY_DIRS - The library directories needed to use FADBAD_1_4
#  FADBAD_1_4_LIBRARIES    - The libraries needed to use FADBAD_1_4


if (FADBAD_1_4_INCLUDE_DIR)
  # in cache already
  SET(FADBAD_1_4_FIND_QUIETLY TRUE)
endif (FADBAD_1_4_INCLUDE_DIR)

find_path(FADBAD_1_4_INCLUDE_DIR NAMES fadiff.h
PATHS
    "./FADBAD++"
    "$ENV{FADBAD_1_4_HOME}/include/FADBAD++"
	"/usr/local/include/FADBAD++"
	"/home/lengagne/Bureau/mogs-all/mogs-optimization-core/optim_solvers/IPOPT/NLP/NLP_FADBAD__1_4/FADBAD++/"
)

message("FADBAD_1_4_INCLUDE_DIR = " ${FADBAD_1_4_INCLUDE_DIR})

set(FADBAD_1_4_INCLUDE_DIRS "${FADBAD_1_4_INCLUDE_DIR}" "${FADBAD_1_4_INCLUDE_DIR}/../" )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBCPLEX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(FADBAD_1_4  DEFAULT_MSG  FADBAD_1_4_INCLUDE_DIR)

mark_as_advanced(FADBAD_1_4_INCLUDE_DIR)
