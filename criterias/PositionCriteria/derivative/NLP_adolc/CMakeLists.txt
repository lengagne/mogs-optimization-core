cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsPositionCriteriaNlpAdolc)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available
message("MogsPositionCriteria_FOUND = " ${MogsPositionCriteria_FOUND} )

message("MogsNlpAdolc_FOUND = " ${MogsNlpAdolc_FOUND} )


if(NOT MogsPositionCriteria_FOUND)
	find_package(MogsPositionCriteria REQUIRED)
endif(NOT MogsPositionCriteria_FOUND)

if(NOT MogsNlpAdolc_FOUND)
	find_package(MogsNlpAdolc REQUIRED)
endif(NOT MogsNlpAdolc_FOUND)



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
	${MogsNlpAdolc_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsPositionCriteriaNlpAdolc.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsPositionCriteriaNlpAdolc.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsPositionCriteriaNlpAdolc
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsPositionCriteriaNlpAdolc
		src/*.cpp)

ADD_LIBRARY(	MogsPositionCriteriaNlpAdolc SHARED
		${source_files_MogsPositionCriteriaNlpAdolc}
)

target_link_libraries(  MogsPositionCriteriaNlpAdolc
						${MogsNlpAdolc_LIBRARIES}
						${MogsPositionCriteria_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsNlpAdolc_LIBRARY_DIRS}
					${MogsPositionCriteria_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsNlpAdolc_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsPositionCriteriaNlpAdolc)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsPositionCriteriaNlpAdolc} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsPositionCriteriaNlpAdolc LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsPositionCriteriaNlpAdolc_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsPositionCriteriaNlpAdolc_LIBRARIES  MogsPositionCriteriaNlpAdolc CACHE INTERNAL "")
SET(MogsPositionCriteriaNlpAdolc_FLAGS "-DMogsPositionCriteriaNlpAdolc_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsPositionCriteriaNlpAdolc_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsPositionCriteriaNlpAdolcConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsPositionCriteriaNlpAdolcConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsPositionCriteriaNlpAdolcConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsPositionCriteriaNlpAdolc" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsCriteriaNlpAdolc position ${INSTALL_LIB_DIRS}/libMogsPositionCriteriaNlpAdolc.so)")
