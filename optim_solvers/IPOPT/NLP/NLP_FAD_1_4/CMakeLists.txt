cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(NLP_FAD_1_4)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available
# find_package(IPOPT QUIET)
find_package(FADBAD_1_4 QUIET)

if(NOT FADBAD_1_4_FOUND)
    message("We do not compile NLPFAD_1_4 since FADBAD 14 was not found")
elseif(IPOPT_FOUND)

	find_package(MogsVisu REQUIRED)

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
        ${FAD_1_4_INCLUDE_DIRS}
        ${MogsVisu_INCLUDE_DIRS}
    )

    configure_file ( "${CMAKE_CURRENT_SOURCE_DIR}/config_MogsNlpFAD_1_4.h.in"
             "${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsNlpFAD_1_4.h" )



    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    FILE ( GLOB_RECURSE
            header_files_MogsNlpFAD_1_4
            include/*.h)

    FILE ( GLOB_RECURSE
            source_files_MogsNlpFAD_1_4
            src/*.cpp)

    ADD_LIBRARY(	MogsNlpFAD_1_4 SHARED
            ${source_files_MogsNlpFAD_1_4}
    )

    target_link_libraries(  MogsNlpFAD_1_4
                            ${FAD_1_4_LIBRARIES}
                            ${MogsIpoptOptimization_LIBRARIES}
                            ${MogsCriteria_LIBRARIES}
                            ${MogsVisu_LIBRARIES})

	link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
						${FAD_1_4_LIBRARY_DIRS}
						${MogsVisu_LIBRARY_DIRS})

	SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
	set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${FAD_1_4_LIBRARY_DIRS}")


### Installation procedure

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    # install the include files
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsNlpFAD_1_4)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsNlpFAD_1_4} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsNlpFAD_1_4 LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    SET(MogsNlpFAD_1_4_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
    SET(MogsNlpFAD_1_4_LIBRARIES  MogsNlpFAD_1_4 CACHE INTERNAL "")
    SET(MogsNlpFAD_1_4_FLAGS "-DMogsNlpFAD_1_4_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
    SET(MogsNlpFAD_1_4_FOUND "TRUE" CACHE INTERNAL "")
    
    add_definitions(${MogsNlpFAD_1_4_FLAGS} ${MogsVisu_FLAGS})
    #  add flags (done before)
    configure_file(MogsNlpFAD_1_4Config.cmake.in "${PROJECT_BINARY_DIR}/MogsNlpFAD_1_4Config.cmake" @ONLY)


    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsNlpFAD_1_4Config.cmake"
            DESTINATION "${INSTALL_LIB_PATH}/MogsNlpFAD_1_4" COMPONENT dev )

    install(CODE "execute_process(COMMAND mogs2 plugins add ipopt_optimization_nlp NLP_FAD_1_4 ${INSTALL_LIB_DIRS}/libMogsNlpFAD_1_4.so)")
    
    message("dans le cmake MogsNlpFAD_1_4_FOUND = " ${MogsNlpFAD_1_4_FOUND} )

else()
    message("IPOPT was not FOUND, so we do not compile MogsNlpFAD_1_4")
endif(NOT FADBAD_1_4_FOUND)

