cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsPositionCriteriaDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# We assume (from compilation) that IPOPT is available

if(NOT MogsPositionCriteria_FOUND)
	find_package(MogsPositionCriteria REQUIRED)
endif(NOT MogsPositionCriteria_FOUND)




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

configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsPositionCriteriaDouble.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsPositionCriteriaDouble.h" )



# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

FILE ( GLOB_RECURSE
		header_files_MogsPositionCriteriaDouble
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsPositionCriteriaDouble
		src/*.cpp)

ADD_LIBRARY(	MogsPositionCriteriaDouble SHARED
		${source_files_MogsPositionCriteriaDouble}
)

target_link_libraries(  MogsPositionCriteriaDouble
						${MogsDouble_LIBRARIES}
						${MogsPositionCriteria_LIBRARIES})

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
					${MogsDouble_LIBRARY_DIRS}
					${MogsPositionCriteria_LIBRARY_DIRS})


SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


### Installation procedure

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsPositionCriteriaDouble)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsPositionCriteriaDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsPositionCriteriaDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsPositionCriteriaDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsPositionCriteriaDouble_LIBRARIES  MogsPositionCriteriaDouble CACHE INTERNAL "")
SET(MogsPositionCriteriaDouble_FLAGS "-DMogsPositionCriteriaDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
add_definitions(${MogsPositionCriteriaDouble_FLAGS} ${MogsVisu_FLAGS})
#  add flags (done before)
configure_file(MogsPositionCriteriaDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsPositionCriteriaDoubleConfig.cmake" @ONLY)


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsPositionCriteriaDoubleConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsPositionCriteriaDouble" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add MogsCriteriaDouble position ${INSTALL_LIB_DIRS}/libMogsPositionCriteriaDouble.so)")
