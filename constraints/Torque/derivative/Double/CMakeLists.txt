cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsTorqueConstraintDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsTorqueConstraint_FOUND)
	find_package(MogsTorqueConstraint REQUIRED)
endif(NOT MogsTorqueConstraint_FOUND)

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
	${MogsTorqueConstraint_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsTorqueConstraintDouble.h.in"
		"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsTorqueConstraintDouble.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsTorqueConstraintDouble
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsTorqueConstraintDouble
		src/*.cpp)

ADD_LIBRARY(	MogsTorqueConstraintDouble SHARED
		${source_files_MogsTorqueConstraintDouble}
)

target_link_libraries(  MogsTorqueConstraintDouble
			${MogsTorqueConstraint_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
			${MogsTorqueConstraint_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsTorqueConstraintDouble)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsTorqueConstraintDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsTorqueConstraintDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsTorqueConstraintDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsTorqueConstraintDouble_LIBRARIES  MogsTorqueConstraintDouble CACHE INTERNAL "")
SET(MogsTorqueConstraintDouble_FLAGS "-DMogsTorqueConstraintDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsTorqueConstraintDouble_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsTorqueConstraintDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsTorqueConstraintDoubleConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsTorqueConstraintDoubleConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsTorqueConstraintDouble" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsConstraintDouble torque ${INSTALL_LIB_DIRS}/libMogsTorqueConstraintDouble.so)")
