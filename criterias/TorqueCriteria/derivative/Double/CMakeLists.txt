cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsTorqueCriteriaDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsTorqueCriteria_FOUND)
	find_package(MogsTorqueCriteria REQUIRED)
endif(NOT MogsTorqueCriteria_FOUND)

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
	${MogsTorqueCriteria_INCLUDE_DIRS}
	${MogsDouble_INCLUDE_DIRS}
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsTorqueCriteriaDouble.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsTorqueCriteriaDouble.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsTorqueCriteriaDouble
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsTorqueCriteriaDouble
		src/*.cpp)


ADD_LIBRARY(	MogsTorqueCriteriaDouble SHARED
		${source_files_MogsTorqueCriteriaDouble}
)

target_link_libraries(  MogsTorqueCriteriaDouble
						${MogsDouble_LIBRARIES}
						${MogsTorqueCriteria_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsDouble_LIBRARY_DIRS}
					${MogsTorqueCriteria_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsTorqueCriteriaDouble)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsTorqueCriteriaDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsTorqueCriteriaDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsTorqueCriteriaDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsTorqueCriteriaDouble_LIBRARIES  MogsTorqueCriteriaDouble CACHE INTERNAL "")
SET(MogsTorqueCriteriaDouble_FLAGS "-DMogsTorqueCriteriaDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsTorqueCriteriaDouble_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsTorqueCriteriaDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsTorqueCriteriaDoubleConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsTorqueCriteriaDoubleConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsTorqueCriteriaDouble" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsCriteriaDouble torque ${INSTALL_LIB_DIRS}/libMogsTorqueCriteriaDouble.so)")
