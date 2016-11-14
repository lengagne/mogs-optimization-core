cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsCameraCriteriaNlpAdolc)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available
message("MogsCameraCriteria_FOUND = " ${MogsCameraCriteria_FOUND} )

message("MogsNlpAdolc_FOUND = " ${MogsNlpAdolc_FOUND} )


if(NOT MogsCameraCriteria_FOUND)
	find_package(MogsCameraCriteria REQUIRED)
endif(NOT MogsCameraCriteria_FOUND)

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
	${MogsCameraCriteria_INCLUDE_DIRS}
	${MogsNlpAdolc_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsCameraCriteriaNlpAdolc.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsCameraCriteriaNlpAdolc.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsCameraCriteriaNlpAdolc
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsCameraCriteriaNlpAdolc
		src/*.cpp)

ADD_LIBRARY(	MogsCameraCriteriaNlpAdolc SHARED
		${source_files_MogsCameraCriteriaNlpAdolc}
)

target_link_libraries(  MogsCameraCriteriaNlpAdolc
						${MogsNlpAdolc_LIBRARIES}
						${MogsCameraCriteria_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsNlpAdolc_LIBRARY_DIRS}
					${MogsCameraCriteria_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsNlpAdolc_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsCameraCriteriaNlpAdolc)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsCameraCriteriaNlpAdolc} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsCameraCriteriaNlpAdolc LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsCameraCriteriaNlpAdolc_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsCameraCriteriaNlpAdolc_LIBRARIES  MogsCameraCriteriaNlpAdolc CACHE INTERNAL "")
SET(MogsCameraCriteriaNlpAdolc_FLAGS "-DMogsCameraCriteriaNlpAdolc_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsCameraCriteriaNlpAdolc_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsCameraCriteriaNlpAdolcConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsCameraCriteriaNlpAdolcConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsCameraCriteriaNlpAdolcConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsCameraCriteriaNlpAdolc" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsCriteriaNlpAdolc camera ${INSTALL_LIB_DIRS}/libMogsCameraCriteriaNlpAdolc.so)")
