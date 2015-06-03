PROJECT (MogsOptimization)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsOptimization_VERSION_MAJOR 1)
set (MogsOptimization_VERSION_MINOR 0)

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

find_package(IPOPT REQUIRED)

include_directories ( 
	include
	${MogsCore_INCLUDE_DIRS}
	${IPOPT_INCLUDE_DIRS}
)

configure_file ( "${PROJECT_SOURCE_DIR}/config_MogsOptimization.h.in"
		 "${PROJECT_SOURCE_DIR}/include/config_MogsOptimization.h" )


get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(MOGS_CORE_INCLUDE_DIRS "${Include_Directories}" CACHE INTERNAL "" )

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
			${IPOPT_LIBRARY_DIRS}
)

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

file(	GLOB_RECURSE
	MogsOptimization_SOURCES
	src/* )

FILE ( GLOB_RECURSE
        header_files_MogsOptimization
	include/*.h)

ADD_LIBRARY(	MogsOptimization SHARED 
		${MogsOptimization_SOURCES}
)

target_link_libraries( MogsOptimization
			MogsCore
			${IPOPT_LIBRARIES})
			
add_custom_target(test_solver)
			
add_executable(	test_ipopt	EXCLUDE_FROM_ALL
		"${CMAKE_CURRENT_SOURCE_DIR}/test/test_ipopt.cpp"
		"${CMAKE_CURRENT_SOURCE_DIR}/test/MyNLP.cpp")
		
target_link_libraries(	test_ipopt
			${IPOPT_LIBRARIES})
			
add_dependencies(test_solver test_ipopt)

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsOptimization)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsOptimization} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsOptimization LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsOptimization_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsOptimization_LIBRARY_DIRS  "${CMAKE_INSTALL_LIBDIR}/lib" CACHE INTERNAL "")
SET(MogsOptimization_LIBRARIES  MogsOptimization CACHE INTERNAL "")
SET(MogsOptimization_FLAGS "-DMogsOptimization_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
# SET(MogsOptimization_FLAGS "-DMogsOptimization_FOUND" CACHE INTERNAL "")
add_definitions(${MogsOptimization_FLAGS})
#  add flags (done before)
configure_file(MogsOptimizationConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsOptimizationConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsOptimizationConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsOptimization" COMPONENT dev ) 

install(CODE "execute_process(COMMAND mogs2 plugins add problem mogs-optimization ${INSTALL_LIB_DIRS}/libMogsOptimization.so)")