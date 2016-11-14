cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsCameraCriteriaNlpFADBAD_1_4)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available
message("MogsCameraCriteria_FOUND = " ${MogsCameraCriteria_FOUND} )

message("MogsNlpFADBAD_1_4_FOUND = " ${MogsNlpFADBAD_1_4_FOUND} )


if(NOT MogsCameraCriteria_FOUND)
	find_package(MogsCameraCriteria REQUIRED)
endif(NOT MogsCameraCriteria_FOUND)

if(NOT MogsNlpFADBAD_1_4_FOUND)
	find_package(MogsNlpFADBAD_1_4 REQUIRED)
endif(NOT MogsNlpFADBAD_1_4_FOUND)



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
	${MogsNlpFADBAD_1_4_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsCameraCriteriaNlpFADBAD_1_4.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsCameraCriteriaNlpFADBAD_1_4.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsCameraCriteriaNlpFADBAD_1_4
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsCameraCriteriaNlpFADBAD_1_4
		src/*.cpp)

ADD_LIBRARY(	MogsCameraCriteriaNlpFADBAD_1_4 SHARED
		${source_files_MogsCameraCriteriaNlpFADBAD_1_4}
)

target_link_libraries(  MogsCameraCriteriaNlpFADBAD_1_4
						${MogsNlpFADBAD_1_4_LIBRARIES}
						${MogsCameraCriteria_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsNlpFADBAD_1_4_LIBRARY_DIRS}
					${MogsCameraCriteria_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsNlpFADBAD_1_4_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsCameraCriteriaNlpFADBAD_1_4)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsCameraCriteriaNlpFADBAD_1_4} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsCameraCriteriaNlpFADBAD_1_4 LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsCameraCriteriaNlpFADBAD_1_4_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsCameraCriteriaNlpFADBAD_1_4_LIBRARIES  MogsCameraCriteriaNlpFADBAD_1_4 CACHE INTERNAL "")
SET(MogsCameraCriteriaNlpFADBAD_1_4_FLAGS "-DMogsCameraCriteriaNlpFADBAD_1_4_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsCameraCriteriaNlpFADBAD_1_4_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsCameraCriteriaNlpFADBAD_1_4Config.cmake.in "${PROJECT_BINARY_DIR}/MogsCameraCriteriaNlpFADBAD_1_4Config.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsCameraCriteriaNlpFADBAD_1_4Config.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsCameraCriteriaNlpFADBAD_1_4" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsCriteriaNlpFADBAD_1_4 camera ${INSTALL_LIB_DIRS}/libMogsCameraCriteriaNlpFADBAD_1_4.so)")
