PROJECT (MogsTransformationConstraint)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsTransformationConstraint_VERSION_MAJOR 1)
set (MogsTransformationConstraint_VERSION_MINOR 0)

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

configure_file ("${PROJECT_SOURCE_DIR}/config_MogsTransformationConstraint.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_MogsTransformationConstraint.h" )


FILE ( GLOB_RECURSE
		header_files_MogsTransformationConstraint
		include/*.h
		include/*.hpp
		include/*.hxx)

FILE ( GLOB_RECURSE
		source_files_MogsTransformationConstraint
		src/*.cpp)

ADD_LIBRARY(	MogsTransformationConstraint SHARED
		${source_files_MogsTransformationConstraint}
)

target_link_libraries( MogsTransformationConstraint
			${MogsIpoptOptimization_LIBRARIES})


#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsTransformationConstraint)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsTransformationConstraint} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsTransformationConstraint LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsTransformationConstraint_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsTransformationConstraint_LIBRARIES  MogsTransformationConstraint CACHE INTERNAL "")
SET(MogsTransformationConstraint_FLAGS "-DMogsTransformationConstraint_FOUND ${MogsIpoptOptimization_FLAGS}" CACHE INTERNAL "")
SET(MogsTransformationConstraint_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsTransformationConstraint_FLAGS})
#  add flags (done before)
configure_file(MogsTransformationConstraintConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsTransformationConstraintConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsTransformationConstraintConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsTransformationConstraints" COMPONENT dev )


add_subdirectory(derivative)
