
SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(IPOPT QUIET)

if(IPOPT_FOUND)

	if(NOT MogsOptimizationCore_FOUND)
		message("MogsOptimizationCore was not FOUND, so we do not compile MogsIpopt")
	else(NOT MogsOptimizationCore_FOUND)
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
			${IPOPT_INCLUDE_DIRS}
			${MogsOptimizationCore_INCLUDE_DIRS}
		)

		configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsIpopt.h.in"
				"${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsIpopt.h" )

		get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

		link_directories(${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
				 ${IPOPT_LIBRARY_DIRS}
				 ${MogsOptimizationCore_LIBRARY_DIRS}
		)

		# Perform the proper linking
		SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

		FILE ( GLOB_RECURSE
			header_files_MogsIpopt
			include/*.h
			include/*.hpp)

		FILE ( GLOB_RECURSE
			source_files_MogsIpopt
			src/*.cpp)

		ADD_LIBRARY(MogsIpopt SHARED
			${source_files_MogsIpopt}
			)

		target_link_libraries(  MogsIpopt
					${IPOPT_LIBRARIES}
					${MogsOptimizationCore_LIBRARIES}
			)

		###  Test of IPOPT

		add_custom_target(test_solver)

		add_executable(	test_ipopt	EXCLUDE_FROM_ALL
			"${CMAKE_CURRENT_SOURCE_DIR}/test/test_ipopt.cpp"
			"${CMAKE_CURRENT_SOURCE_DIR}/test/MyNLP.cpp")

		target_link_libraries(	test_ipopt
					${IPOPT_LIBRARIES})

		add_dependencies(test_solver test_ipopt)


		### Installation procedure

		# Perform the proper linking
		SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

		# install the include files
		SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsIpopt)
		SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

		# install the include files
		install(FILES ${header_files_MogsIpopt} DESTINATION ${INSTALL_INCLUDE_DIRS})

		# install the library files
		install( TARGETS MogsIpopt LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

		### Configure file
		get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
		SET(MogsIpopt_INCLUDE_DIRS_LOCAL  ${Include_Directories} CACHE INTERNAL "")
		SET(MogsIpopt_INCLUDE_DIRS ${MogsCore_INCLUDE_DIRS} ${Include_Directories} CACHE INTERNAL "")
		SET(MogsIpopt_LIBRARIES  MogsIpopt ${MogsCore_LIBRARIES} CACHE INTERNAL "")
		SET(MogsIpopt_FLAGS "-DMogsIpopt_FOUND ${IPOPT_FLAGS} -DHAVE_CSTDDEF" CACHE INTERNAL "")
		SET(MogsIpopt_FOUND TRUE CACHE INTERNAL "")

		add_definitions(${MogsIpopt_FLAGS})
		#  add flags (done before)
		configure_file(MogsIpoptConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsIpoptConfig.cmake" @ONLY)

		install(DIRECTORY 		${CMAKE_CURRENT_SOURCE_DIR}/data/templates
					${CMAKE_CURRENT_SOURCE_DIR}/data/xsd
			DESTINATION 	${CMAKE_INSTALL_LIBDIR}/share/mogs2)

		install(FILES "${PROJECT_BINARY_DIR}/MogsIpoptConfig.cmake"
			DESTINATION "${INSTALL_LIB_PATH}/MogsIpopt" COMPONENT dev )	
			
	endif(NOT MogsOptimizationCore_FOUND)
else()
    message("IPOPT was not FOUND, so we do not compile MogsIpopt")
endif(IPOPT_FOUND)

 
