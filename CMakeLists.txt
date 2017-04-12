PROJECT (MogsOptimizationCore)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsOptimizationCore_VERSION_MAJOR 1)
set (MogsOptimizationCore_VERSION_MINOR 0)

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

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
set( LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
SET( EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin )

foreach(p LIB INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

find_package(MogsCore REQUIRED)

include_directories (
	include
	${MogsCore_INCLUDE_DIRS}
)

configure_file ("${PROJECT_SOURCE_DIR}/config_MogsOptimizationCore.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_MogsOptimizationCore.h" )
#
#
#get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#set(MOGS_CORE_INCLUDE_DIRS "${Include_Directories}" CACHE INTERNAL "" )
#
#link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
#)
#
## Perform the proper linking
#SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#

FILE ( GLOB_RECURSE
        header_files_MogsOptimizationCore
        include/*.h
        include/*.hpp)
#
## Perform the proper linking
#SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsOptimizationCore)

# install the include files
install(FILES ${header_files_MogsOptimizationCore} DESTINATION ${INSTALL_INCLUDE_DIRS})

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsOptimizationCore_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsOptimizationCore_FLAGS "-DMogsOptimizationCore_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
SET(MogsOptimizationCore_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsOptimizationCore_FLAGS})
#  add flags (done before)
configure_file(MogsOptimizationCoreConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsOptimizationCoreConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsOptimizationCoreConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsOptimizationCore" COMPONENT dev )

message("before include_directories")

add_subdirectory(optim_solvers)
add_subdirectory(criterias)

