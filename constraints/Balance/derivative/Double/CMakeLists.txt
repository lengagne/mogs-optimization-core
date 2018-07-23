cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsBalanceConstraintDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsBalanceConstraint_FOUND)
	find_package(MogsBalanceConstraint REQUIRED)
endif(NOT MogsBalanceConstraint_FOUND)

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
	${MogsPositionCriteria_INCLUDE_DIRS}
	${MogsDouble_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsBalanceConstraintDouble.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsBalanceConstraintDouble.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsBalanceConstraintDouble
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsBalanceConstraintDouble
		src/*.cpp)

ADD_LIBRARY(	MogsBalanceConstraintDouble SHARED
		${source_files_MogsBalanceConstraintDouble}
)

target_link_libraries(  MogsBalanceConstraintDouble
						${MogsDouble_LIBRARIES}
						${MogsBalanceConstraint_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsDouble_LIBRARY_DIRS}
					${MogsBalanceConstraint_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsBalanceConstraintDouble)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsBalanceConstraintDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsBalanceConstraintDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsBalanceConstraintDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsBalanceConstraintDouble_LIBRARIES  MogsBalanceConstraintDouble CACHE INTERNAL "")
SET(MogsBalanceConstraintDouble_FLAGS "-DMogsBalanceConstraintDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsBalanceConstraintDouble_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsBalanceConstraintDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsBalanceConstraintDoubleConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsBalanceConstraintDoubleConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsBalanceConstraintDouble" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsConstraintDouble Balance ${INSTALL_LIB_DIRS}/libMogsBalanceConstraintDouble.so)")
