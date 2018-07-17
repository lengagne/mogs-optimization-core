PROJECT (MogsTorqueCriteria)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsTorqueCriteria_VERSION_MAJOR 1)
set (MogsTorqueCriteria_VERSION_MINOR 0)

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

configure_file ("${PROJECT_SOURCE_DIR}/config_MogsTorqueCriteria.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_MogsTorqueCriteria.h" )


FILE ( GLOB_RECURSE
		header_files_MogsTorqueCriteria
		include/*.h
		include/*.hpp
		include/*.hxx)

FILE ( GLOB_RECURSE
		source_files_MogsTorqueCriteria
		src/*.cpp)

ADD_LIBRARY(	MogsTorqueCriteria SHARED
		${source_files_MogsTorqueCriteria}
)

target_link_libraries( MogsTorqueCriteria
			${MogsIpoptOptimization_LIBRARIES})


#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsTorqueCriteria)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsTorqueCriteria} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsTorqueCriteria LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsTorqueCriteria_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsTorqueCriteria_LIBRARIES  MogsTorqueCriteria CACHE INTERNAL "")
SET(MogsTorqueCriteria_FLAGS "-DMogsTorqueCriteria_FOUND ${MogsIpoptOptimization_FLAGS}" CACHE INTERNAL "")
SET(MogsTorqueCriteria_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsTorqueCriteria_FLAGS})
#  add flags (done before)
configure_file(MogsTorqueCriteriaConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsTorqueCriteriaConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsTorqueCriteriaConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsTorqueCriteria" COMPONENT dev )

add_subdirectory(derivative)
