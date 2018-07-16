SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(MogsGeneticAlgorithm QUIET)

if(MogsGeneticAlgorithm_FOUND)
    message("MogsGeneticAlgorithm FOUND")

	find_package(MogsVisu QUIET)

    find_package(MogsCore REQUIRED)

    if (NOT MogsOptimizationCore_FOUND)
        find_package(MogsOptimizationCore REQUIRED)
    endif(NOT MogsOptimizationCore_FOUND)


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
        ${MogsGeneticAlgorithm_INCLUDE_DIRS}
        ${MogsVisu_INCLUDE_DIRS}
    )


    configure_file ( "${CMAKE_CURRENT_SOURCE_DIR}/config_MogsMGAOptimization.h.in"
             "${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsMGAOptimization.h" )


    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

    set(MOGS_CORE_INCLUDE_DIRS "${Include_Directories}" CACHE INTERNAL "" )

#    link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
#                        ${MogsGeneticAlgorithm_LIBRARY_DIRS}
#    )

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    FILE ( GLOB_RECURSE
            header_files_MogsMGAOptimization
            include/*.h)

    FILE ( GLOB_RECURSE
            source_files_MogsMGAOptimization
            src/*.cpp)

    ADD_LIBRARY(	MogsMGAOptimization SHARED
                    ${source_files_MogsMGAOptimization}
    )

    target_link_libraries( MogsMGAOptimization
                ${MogsGeneticAlgorithm_LIBRARIES}
                ${MogsCore_LIBRARIES}
                ${MogsCriteria_LIBRARIES}
                ${MogsVisu_LIBRARIES}
                ${MogsOptimizationCore_LIBRARIES}
                )



### Installation procedure

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    # install the include files
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsMGAOptimization)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsMGAOptimization} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsMGAOptimization LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    SET(MogsMGAOptimization_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
    SET(MogsMGAOptimization_LIBRARIES  MogsMGAOptimization CACHE INTERNAL "")
    SET(MogsMGAOptimization_FLAGS "-DMogsMGAOptimization_FOUND ${MGA_FLAGS}" CACHE INTERNAL "")
    SET(MogsMGAOptimization_FOUND TRUE CACHE INTERNAL "")

    add_definitions(${MogsMGAOptimization_FLAGS} ${MogsVisu_FLAGS})
    #  add flags (done before)
    configure_file(MogsMGAOptimizationConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsMGAOptimizationConfig.cmake" @ONLY)

    install(DIRECTORY 		${CMAKE_CURRENT_SOURCE_DIR}/data/templates
                            ${CMAKE_CURRENT_SOURCE_DIR}/data/xsd
            DESTINATION 	${CMAKE_INSTALL_LIBDIR}/share/mogs2)

    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsMGAOptimizationConfig.cmake"
            DESTINATION "${INSTALL_LIB_PATH}/MogsMGAOptimization" COMPONENT dev )

    install(CODE "execute_process(COMMAND mogs2 plugins add problem mogs-mga-optimization ${INSTALL_LIB_DIRS}/libMogsMGAOptimization.so)")

else()
    message("MGA was not FOUND, so we do not compile MogsMGAOptimization")
endif(MogsGeneticAlgorithm_FOUND)

#  add_subdirectory(NLP)
