cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsStaticPostureParameterizationDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# We assume (from compilation) that IPOPT is available

if(NOT MogsStaticPostureParameterization_FOUND)
	find_package(MogsStaticPostureParameterization REQUIRED)
endif(NOT MogsStaticPostureParameterization_FOUND)

# if(NOT MogsDouble_FOUND)
# 	find_package(MogsDouble REQUIRED)
# endif(NOT MogsDouble_FOUND)



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
# 	${MogsPositionCriteria_INCLUDE_DIRS}
# 	${MogsDouble_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsStaticPostureParameterizationDouble.h.in"
		"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsStaticPostureParameterizationDouble.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsStaticPostureParameterizationDouble
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsStaticPostureParameterizationDouble
		src/*.cpp)

ADD_LIBRARY(	MogsStaticPostureParameterizationDouble SHARED
		${source_files_MogsStaticPostureParameterizationDouble}
)

target_link_libraries(  MogsStaticPostureParameterizationDouble
						${MogsDouble_LIBRARIES}
						${MogsStaticPostureParameterization_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsDouble_LIBRARY_DIRS}
					${MogsStaticPostureParameterization_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsStaticPostureParameterizationDouble)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsStaticPostureParameterizationDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsStaticPostureParameterizationDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsStaticPostureParameterizationDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsStaticPostureParameterizationDouble_LIBRARIES  MogsStaticPostureParameterizationDouble CACHE INTERNAL "")
SET(MogsStaticPostureParameterizationDouble_FLAGS "-DMogsStaticPostureParameterizationDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsStaticPostureParameterizationDouble_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsStaticPostureParameterizationDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsStaticPostureParameterizationDoubleConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsStaticPostureParameterizationDoubleConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsStaticPostureParameterizationDouble" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsParameterizationDouble StaticPosture ${INSTALL_LIB_DIRS}/libMogsStaticPostureParameterizationDouble.so)")
