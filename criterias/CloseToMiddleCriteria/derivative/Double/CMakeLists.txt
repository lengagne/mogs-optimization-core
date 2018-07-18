cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsCloseToMiddleCriteriaDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsCloseToMiddleCriteria_FOUND)
	find_package(MogsCloseToMiddleCriteria REQUIRED)
endif(NOT MogsCloseToMiddleCriteria_FOUND)


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
)

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsCloseToMiddleCriteriaDouble.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsCloseToMiddleCriteriaDouble.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsCloseToMiddleCriteriaDouble
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsCloseToMiddleCriteriaDouble
		src/*.cpp)


ADD_LIBRARY(	MogsCloseToMiddleCriteriaDouble SHARED
		${source_files_MogsCloseToMiddleCriteriaDouble}
)

target_link_libraries(  MogsCloseToMiddleCriteriaDouble
						${MogsDouble_LIBRARIES}
						${MogsCloseToMiddleCriteria_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsDouble_LIBRARY_DIRS}
					${MogsCloseToMiddleCriteria_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsCloseToMiddleCriteriaDouble)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsCloseToMiddleCriteriaDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsCloseToMiddleCriteriaDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsCloseToMiddleCriteriaDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsCloseToMiddleCriteriaDouble_LIBRARIES  MogsCloseToMiddleCriteriaDouble CACHE INTERNAL "")
SET(MogsCloseToMiddleCriteriaDouble_FLAGS "-DMogsCloseToMiddleCriteriaDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsCloseToMiddleCriteriaDouble_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsCloseToMiddleCriteriaDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsCloseToMiddleCriteriaDoubleConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsCloseToMiddleCriteriaDoubleConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsCloseToMiddleCriteriaDouble" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsCriteriaDouble close_to_middle ${INSTALL_LIB_DIRS}/libMogsCloseToMiddleCriteriaDouble.so)")
