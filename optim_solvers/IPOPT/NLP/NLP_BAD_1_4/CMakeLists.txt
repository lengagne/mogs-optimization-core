cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(NLP_BAD_1_4)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available
# find_package(IPOPT QUIET)
find_package(BAD_1_4 REQUIRED)

if(NOT BAD_1_4_FOUND)
    message("We do not compile NLPBAD_1_4 since BAD_1_4 was not found")
elseif(IPOPT_FOUND)

	find_package(MogsVisu QUIET)

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
        ${MogsCriteria_INCLUDE_DIRS}
        ${MogsOptimizationCore_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
        ${BAD_1_4_INCLUDE_DIRS}
        ${MogsVisu_INCLUDE_DIRS}
    )

    configure_file ( "${CMAKE_CURRENT_SOURCE_DIR}/config_MogsNlpBAD_1_4.h.in"
             "${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsNlpBAD_1_4.h" )



    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    FILE ( GLOB_RECURSE
            header_files_MogsNlpBAD_1_4
            include/*.h)

    FILE ( GLOB_RECURSE
            source_files_MogsNlpBAD_1_4
            src/*.cpp)

    ADD_LIBRARY(	MogsNlpBAD_1_4 SHARED
            ${source_files_MogsNlpBAD_1_4}
    )

    target_link_libraries(  MogsNlpBAD_1_4
                            ${BAD_1_4_LIBRARIES}
                            ${MogsIpoptOptimization_LIBRARIES}
                            ${MogsCriteria_LIBRARIES}
                            ${MogsVisu_LIBRARIES})

	link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
						${BAD_1_4_LIBRARY_DIRS}
						${MogsVisu_LIBRARY_DIRS})

	SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
	set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${BAD_1_4_LIBRARY_DIRS}")


### Installation procedure

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    # install the include files
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsNlpBAD_1_4)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsNlpBAD_1_4} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsNlpBAD_1_4 LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    SET(MogsNlpBAD_1_4_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
    SET(MogsNlpBAD_1_4_LIBRARIES  MogsNlpBAD_1_4 CACHE INTERNAL "")
    SET(MogsNlpBAD_1_4_FLAGS "-DMogsNlpBAD_1_4_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
    SET(MogsNlpBAD_1_4_FOUND "TRUE" CACHE INTERNAL "")

    add_definitions(${MogsNlpBAD_1_4_FLAGS} ${MogsVisu_FLAGS})
    #  add flags (done before)
    configure_file(MogsNlpBAD_1_4Config.cmake.in "${PROJECT_BINARY_DIR}/MogsNlpBAD_1_4Config.cmake" @ONLY)


    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsNlpBAD_1_4Config.cmake"
            DESTINATION "${INSTALL_LIB_PATH}/MogsNlpBAD_1_4" COMPONENT dev )

    install(CODE "execute_process(COMMAND mogs2 plugins add ipopt_optimization_nlp NLP_BAD_1_4 ${INSTALL_LIB_DIRS}/libMogsNlpBAD_1_4.so)")

else()
    message("IPOPT was not FOUND, so we do not compile MogsNlpBAD_1_4")
endif(NOT BAD_1_4_FOUND)

