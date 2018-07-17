PROJECT (MogsToZeroConstraint)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsToZeroConstraint_VERSION_MAJOR 1)
set (MogsToZeroConstraint_VERSION_MINOR 0)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -fpic")
set(CMAKE_VERBOSE_MAKEFILE OFF)

set( LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
SET( EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin )

foreach(p LIB INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

IF (NOT MogsOptimizationCore_FOUND)
    find_package(MogsOptimizationCore REQUIRED)
endIF (NOT MogsOptimizationCore_FOUND)

include_directories (
	include
	${MogsOptimizationCore_INCLUDE_DIRS}
)

configure_file ("${PROJECT_SOURCE_DIR}/config_MogsToZeroConstraint.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_MogsToZeroConstraint.h" )


FILE ( GLOB_RECURSE
		header_files_MogsToZeroConstraint
		include/*.h
		include/*.hpp
		include/*.hxx)

FILE ( GLOB_RECURSE
		source_files_MogsToZeroConstraint
		src/*.cpp)

ADD_LIBRARY(	MogsToZeroConstraint SHARED
		${source_files_MogsToZeroConstraint}
)

target_link_libraries( MogsToZeroConstraint
			${MogsIpoptOptimization_LIBRARIES})


#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsToZeroConstraint)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsToZeroConstraint} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsToZeroConstraint LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsToZeroConstraint_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsToZeroConstraint_LIBRARIES  MogsToZeroConstraint CACHE INTERNAL "")
SET(MogsToZeroConstraint_FLAGS "-DMogsToZeroConstraint_FOUND ${MogsIpoptOptimization_FLAGS}" CACHE INTERNAL "")
SET(MogsToZeroConstraint_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsToZeroConstraint_FLAGS})
#  add flags (done before)
configure_file(MogsToZeroConstraintConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsToZeroConstraintConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsToZeroConstraintConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsToZeroConstraints" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsConstraint ToZero ${INSTALL_LIB_DIRS}/libMogsToZeroConstraint.so)")
		
add_subdirectory(derivative)
