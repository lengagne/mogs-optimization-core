SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(IPOPT QUIET)

if(IPOPT_FOUND)

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
        ${MogsNlpSampleCore_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
    )

    configure_file ( "${CMAKE_CURRENT_SOURCE_DIR}/config_MogsNlpSample.h.in"
             "${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsNlpSample.h" )



    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    FILE ( GLOB_RECURSE
            header_files_MogsNlpSample
            include/*.h)

    FILE ( GLOB_RECURSE
            source_files_MogsNlpSample
            src/*.cpp)

    ADD_LIBRARY(	MogsNlpSample SHARED
            ${source_files_MogsNlpSample}
    )

    target_link_libraries( MogsNlpSample
                ${MogsIpoptOptimization_LIBRARIES})


### Installation procedure

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    # install the include files
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsNlpSample)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsNlpSample} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsNlpSample LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    SET(MogsNlpSample_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
    SET(MogsNlpSample_LIBRARIES  MogsNlpSample CACHE INTERNAL "")
    SET(MogsNlpSample_FLAGS "-DMogsNlpSample_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
    add_definitions(${MogsNlpSample_FLAGS})
    #  add flags (done before)
    configure_file(MogsNlpSampleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsNlpSampleConfig.cmake" @ONLY)


    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsNlpSampleConfig.cmake"
            DESTINATION "${INSTALL_LIB_PATH}/MogsNlpSample" COMPONENT dev )

    install(CODE "execute_process(COMMAND mogs2 plugins add ipopt_optimization_nlp NLP_sample ${INSTALL_LIB_DIRS}/libMogsNlpSample.so)")

else()
    mesage("IPOPT was not FOUND, so we do not compile MogsNlpSample")
endif(IPOPT_FOUND)

