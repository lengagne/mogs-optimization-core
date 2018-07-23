cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsBoxCollisionConstraintDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsBoxCollisionConstraint_FOUND)
	find_package(MogsBoxCollisionConstraint REQUIRED)
endif(NOT MogsBoxCollisionConstraint_FOUND)

if(NOT MogsDouble_FOUND)
	find_package(MogsDouble QUIET)
endif(NOT MogsDouble_FOUND)


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
	${MogsBoxCollisionConstraint_INCLUDE_DIRS}
	${MogsDouble_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsBoxCollisionConstraintDouble.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsBoxCollisionConstraintDouble.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsBoxCollisionConstraintDouble
		include/*.h
		include/*.hpp)

FILE ( GLOB_RECURSE
		source_files_MogsBoxCollisionConstraintDouble
		src/*.cpp)

ADD_LIBRARY(	MogsBoxCollisionConstraintDouble SHARED
		${source_files_MogsBoxCollisionConstraintDouble}
)

target_link_libraries(  MogsBoxCollisionConstraintDouble
						${MogsDouble_LIBRARIES}
						${MogsBoxCollisionConstraint_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsDouble_LIBRARY_DIRS}
					${MogsBoxCollisionConstraint_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsBoxCollisionConstraintDouble)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsBoxCollisionConstraintDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsBoxCollisionConstraintDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsBoxCollisionConstraintDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsBoxCollisionConstraintDouble_LIBRARIES  MogsBoxCollisionConstraintDouble CACHE INTERNAL "")
SET(MogsBoxCollisionConstraintDouble_FLAGS "-DMogsBoxCollisionConstraintDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
SET(MogsBoxCollisionConstraintDouble_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsBoxCollisionConstraintDouble_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsBoxCollisionConstraintDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsBoxCollisionConstraintDoubleConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsBoxCollisionConstraintDoubleConfig.cmake"
	DESTINATION "${INSTALL_LIB_PATH}/MogsBoxCollisionConstraintDouble" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsConstraintDouble BoxCollision ${INSTALL_LIB_DIRS}/libMogsBoxCollisionConstraintDouble.so)")
