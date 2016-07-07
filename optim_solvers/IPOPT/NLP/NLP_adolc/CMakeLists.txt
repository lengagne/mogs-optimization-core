
cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(NLP_adolc)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(IPOPT QUIET)
find_package(ADOLC QUIET)

if(NOT ADOLC_FOUND)
    message("We do not compile NLPAdolc since adolc was not found")
elseif(IPOPT_FOUND)

    if (NOT MogsIpoptOptimization_FOUND)
        find_package(MogsIpoptOptimization REQUIRED)
    endif(NOT MogsIpoptOptimization_FOUND)


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
        ${MogsCore_INCLUDE_DIRS}
        ${MogsNlpAdolcCore_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
        ${ADOLC_INCLUDE_DIRS}
    )

    configure_file ( "${CMAKE_CURRENT_SOURCE_DIR}/config_MogsNlpAdolc.h.in"
             "${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsNlpAdolc.h" )



    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    FILE ( GLOB_RECURSE
            header_files_MogsNlpAdolc
            include/*.h)

    FILE ( GLOB_RECURSE
            source_files_MogsNlpAdolc
            src/*.cpp)

    ADD_LIBRARY(	MogsNlpAdolc SHARED
            ${source_files_MogsNlpAdolc}
    )

    target_link_libraries(  MogsNlpAdolc
                            ${MogsNlpAdolc_LIBRARIES}
                            ${ADOLC_LIBRARIES})

	link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
						${ADOLC_LIBRARY_DIRS})
						
	message("ADOLC_LIBRARY_DIRS = " ${ADOLC_LIBRARY_DIRS})
	message("CMAKE_INSTALL_RPATH  = " ${CMAKE_INSTALL_RPATH})
	
	SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")	
	set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${ADOLC_LIBRARY_DIRS}")
	
	message("CMAKE_INSTALL_RPATH  = " ${CMAKE_INSTALL_RPATH})

### Installation procedure

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    # install the include files
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsNlpAdolc)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsNlpAdolc} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsNlpAdolc LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    SET(MogsNlpAdolc_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
    SET(MogsNlpAdolc_LIBRARIES  MogsNlpAdolc CACHE INTERNAL "")
    SET(MogsNlpAdolc_FLAGS "-DMogsNlpAdolc_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
    add_definitions(${MogsNlpAdolc_FLAGS})
    #  add flags (done before)
    configure_file(MogsNlpAdolcConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsNlpAdolcConfig.cmake" @ONLY)


    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsNlpAdolcConfig.cmake"
            DESTINATION "${INSTALL_LIB_PATH}/MogsNlpAdolc" COMPONENT dev )

    install(CODE "execute_process(COMMAND mogs2 plugins add ipopt_optimization_nlp NLP_Adolc ${INSTALL_LIB_DIRS}/libMogsNlpAdolc.so)")

else()
    mesage("IPOPT was not FOUND, so we do not compile MogsNlpAdolc")
endif(NOT ADOLC_FOUND)

