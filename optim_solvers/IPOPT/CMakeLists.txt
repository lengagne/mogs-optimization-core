
SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(IPOPT QUIET)

if(IPOPT_FOUND)

    if (NOT MogsCore_FOUND)
        find_package(MogsCore REQUIRED)
    endif (NOT MogsCore_FOUND)

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
        ../criterias/include
        ${MogsCore_INCLUDE_DIRS}
        ${MogsOptimizationCore_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
    )

    configure_file ( "${CMAKE_CURRENT_SOURCE_DIR}/config_MogsIpoptOptimization.h.in"
             "${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsIpoptOptimization.h" )


    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    set(MOGS_CORE_INCLUDE_DIRS "${Include_Directories}" CACHE INTERNAL "" )

    link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
                ${IPOPT_LIBRARY_DIRS}
    )

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    FILE ( GLOB_RECURSE
            header_files_MogsIpoptOptimization
            include/*.h
            include/*.hpp)

    FILE ( GLOB_RECURSE
            source_files_MogsIpoptOptimization
            src/*.cpp)

    ADD_LIBRARY(	MogsIpoptOptimization SHARED
            ${source_files_MogsIpoptOptimization}
    )

    target_link_libraries( MogsIpoptOptimization
                ${IPOPT_LIBRARIES}
                ${MogsCore_LIBRARIES}
                ${MogsOptimizationCore_LIBRARIES})

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
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsIpoptOptimization)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsIpoptOptimization} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsIpoptOptimization LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    SET(MogsIpoptOptimization_INCLUDE_DIRS ${MogsOptimizationCore_INCLUDE_DIRS} ${Include_Directories} CACHE INTERNAL "")
    SET(MogsIpoptOptimization_LIBRARIES  MogsIpoptOptimization ${MogsOptimizationCore_LIBRARIES} CACHE INTERNAL "")
    SET(MogsIpoptOptimization_FLAGS "-DMogsIpoptOptimization_FOUND ${IPOPT_FLAGS} -DHAVE_CSTDDEF" CACHE INTERNAL "")
    SET(MogsIpoptOptimization_FOUND TRUE CACHE INTERNAL "")

    add_definitions(${MogsIpoptOptimization_FLAGS})
    #  add flags (done before)
    configure_file(MogsIpoptOptimizationConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsIpoptOptimizationConfig.cmake" @ONLY)

    install(DIRECTORY 		${CMAKE_CURRENT_SOURCE_DIR}/data/templates
                            ${CMAKE_CURRENT_SOURCE_DIR}/data/xsd
            DESTINATION 	${CMAKE_INSTALL_LIBDIR}/share/mogs2)

    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsIpoptOptimizationConfig.cmake"
            DESTINATION "${INSTALL_LIB_PATH}/MogsIpoptOptimization" COMPONENT dev )

    install(CODE "execute_process(COMMAND mogs2 plugins add optim_solver ipopt ${INSTALL_LIB_DIRS}/libMogsIpoptOptimization.so)")


else()
    message("IPOPT was not FOUND, so we do not compile MogsIpoptOptimization")
endif(IPOPT_FOUND)

 add_subdirectory(NLP)
