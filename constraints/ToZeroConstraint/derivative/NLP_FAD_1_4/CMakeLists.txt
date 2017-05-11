cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsToZeroConstraintNlpFAD_1_4)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsToZeroConstraint_FOUND)
	find_package(MogsToZeroConstraint REQUIRED)
endif(NOT MogsToZeroConstraint_FOUND)

if(NOT MogsNlpFAD_1_4_FOUND)
	find_package(MogsNlpFAD_1_4 REQUIRED)
endif(NOT MogsNlpFAD_1_4_FOUND)



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
	${MogsNlpFAD_1_4_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsToZeroConstraintNlpFAD_1_4.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsToZeroConstraintNlpFAD_1_4.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsToZeroConstraintNlpFAD_1_4
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsToZeroConstraintNlpFAD_1_4
		src/*.cpp)

ADD_LIBRARY(	MogsToZeroConstraintNlpFAD_1_4 SHARED
		${source_files_MogsToZeroConstraintNlpFAD_1_4}
)

target_link_libraries(  MogsToZeroConstraintNlpFAD_1_4
						${MogsNlpFAD_1_4_LIBRARIES}
						${MogsToZeroConstraint_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsNlpFAD_1_4_LIBRARY_DIRS}
					${MogsToZeroConstraint_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsNlpFAD_1_4_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsToZeroConstraintNlpFAD_1_4)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsToZeroConstraintNlpFAD_1_4} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsToZeroConstraintNlpFAD_1_4 LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsToZeroConstraintNlpFAD_1_4_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsToZeroConstraintNlpFAD_1_4_LIBRARIES  MogsToZeroConstraintNlpFAD_1_4 CACHE INTERNAL "")
SET(MogsToZeroConstraintNlpFAD_1_4_FLAGS "-DMogsToZeroConstraintNlpFAD_1_4_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsToZeroConstraintNlpFAD_1_4_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsToZeroConstraintNlpFAD_1_4Config.cmake.in "${PROJECT_BINARY_DIR}/MogsToZeroConstraintNlpFAD_1_4Config.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsToZeroConstraintNlpFAD_1_4Config.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsToZeroConstraintNlpFAD_1_4" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsConstraintNlpFAD_1_4 ToZero ${INSTALL_LIB_DIRS}/libMogsToZeroConstraintNlpFAD_1_4.so)")
