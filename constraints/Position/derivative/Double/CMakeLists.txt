cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsPositionConstraintDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsPositionConstraint_FOUND)
	find_package(MogsPositionConstraint REQUIRED)
endif(NOT MogsPositionConstraint_FOUND)

set( LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
SET( EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin )

foreach(p LIB INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()


include_directories (
	include
	${MogsPositionConstraint_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsPositionConstraintDouble.h.in"
		"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsPositionConstraintDouble.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsPositionConstraintDouble
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsPositionConstraintDouble
		src/*.cpp)

ADD_LIBRARY(	MogsPositionConstraintDouble SHARED
		${source_files_MogsPositionConstraintDouble}
)

target_link_libraries(  MogsPositionConstraintDouble
			${MogsPositionConstraint_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
			${MogsPositionConstraint_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsPositionConstraintDouble)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsPositionConstraintDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsPositionConstraintDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsPositionConstraintDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsPositionConstraintDouble_LIBRARIES  MogsPositionConstraintDouble CACHE INTERNAL "")
SET(MogsPositionConstraintDouble_FLAGS "-DMogsPositionConstraintDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsPositionConstraintDouble_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsPositionConstraintDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsPositionConstraintDoubleConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsPositionConstraintDoubleConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsPositionConstraintDouble" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsConstraintDouble Position ${INSTALL_LIB_DIRS}/libMogsPositionConstraintDouble.so)")
