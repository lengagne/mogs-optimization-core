PROJECT (MogsBoxCollisionConstraint)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsBoxCollisionConstraint_VERSION_MAJOR 1)
set (MogsBoxCollisionConstraint_VERSION_MINOR 0)

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

IF (NOT MogsBoxCollision_FOUND)
    find_package(MogsBoxCollision REQUIRED)
endIF (NOT MogsBoxCollision_FOUND)

include_directories (
	include
	${MogsOptimizationCore_INCLUDE_DIRS}
	${MogsBoxCollision_INCLUDE_DIRS}
)

configure_file ("${PROJECT_SOURCE_DIR}/config_MogsBoxCollisionConstraint.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_MogsBoxCollisionConstraint.h" )


FILE ( GLOB_RECURSE
		header_files_MogsBoxCollisionConstraint
		include/*.h
		include/*.hpp
		include/*.hxx)

FILE ( GLOB_RECURSE
		source_files_MogsBoxCollisionConstraint
		src/*.cpp)

ADD_LIBRARY(	MogsBoxCollisionConstraint SHARED
		${source_files_MogsBoxCollisionConstraint}
)

target_link_libraries( MogsBoxCollisionConstraint
			${MogsOptimizationCore_LIBRARIES}
			${MogsBoxCollision_LIBRARIES})

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsBoxCollisionConstraint)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsBoxCollisionConstraint} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsBoxCollisionConstraint LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsBoxCollisionConstraint_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsBoxCollisionConstraint_LIBRARIES  MogsBoxCollisionConstraint CACHE INTERNAL "")
SET(MogsBoxCollisionConstraint_FLAGS "-DMogsBoxCollisionConstraint_FOUND ${MogsIpoptOptimization_FLAGS}" CACHE INTERNAL "")
SET(MogsBoxCollisionConstraint_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsBoxCollisionConstraint_FLAGS})
#  add flags (done before)
configure_file(MogsBoxCollisionConstraintConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsBoxCollisionConstraintConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsBoxCollisionConstraintConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsBoxCollisionConstraints" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsConstraint BoxCollision ${INSTALL_LIB_DIRS}/libMogsBoxCollisionConstraint.so)")

add_subdirectory(derivative)
