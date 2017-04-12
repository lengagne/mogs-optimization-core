cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsCloseToMiddleCriteriaNlpFAD_1_4)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsCloseToMiddleCriteria_FOUND)
	find_package(MogsCloseToMiddleCriteria REQUIRED)
endif(NOT MogsCloseToMiddleCriteria_FOUND)

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
	${MogsCloseToMiddleCriteria_INCLUDE_DIRS}
	${MogsNlpFAD_1_4_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsCloseToMiddleCriteriaNlpFAD_1_4.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsCloseToMiddleCriteriaNlpFAD_1_4.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsCloseToMiddleCriteriaNlpFAD_1_4
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsCloseToMiddleCriteriaNlpFAD_1_4
		src/*.cpp)


ADD_LIBRARY(	MogsCloseToMiddleCriteriaNlpFAD_1_4 SHARED
		${source_files_MogsCloseToMiddleCriteriaNlpFAD_1_4}
)

target_link_libraries(  MogsCloseToMiddleCriteriaNlpFAD_1_4
						${MogsNlpFAD_1_4_LIBRARIES}
						${MogsCloseToMiddleCriteria_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsNlpFAD_1_4_LIBRARY_DIRS}
					${MogsCloseToMiddleCriteria_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsNlpFAD_1_4_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsCloseToMiddleCriteriaNlpFAD_1_4)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsCloseToMiddleCriteriaNlpFAD_1_4} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsCloseToMiddleCriteriaNlpFAD_1_4 LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsCloseToMiddleCriteriaNlpFAD_1_4_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsCloseToMiddleCriteriaNlpFAD_1_4_LIBRARIES  MogsCloseToMiddleCriteriaNlpFAD_1_4 CACHE INTERNAL "")
SET(MogsCloseToMiddleCriteriaNlpFAD_1_4_FLAGS "-DMogsCloseToMiddleCriteriaNlpFAD_1_4_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsCloseToMiddleCriteriaNlpFAD_1_4_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsCloseToMiddleCriteriaNlpFAD_1_4Config.cmake.in "${PROJECT_BINARY_DIR}/MogsCloseToMiddleCriteriaNlpFAD_1_4Config.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsCloseToMiddleCriteriaNlpFAD_1_4Config.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsCloseToMiddleCriteriaNlpFAD_1_4" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsCriteriaNlpFAD_1_4 close_to_middle ${INSTALL_LIB_DIRS}/libMogsCloseToMiddleCriteriaNlpFAD_1_4.so)")
