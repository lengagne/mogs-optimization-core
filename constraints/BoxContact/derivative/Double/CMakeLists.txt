cmake_minimum_required (VERSION 2.8)
enable_language(CXX)
project(MogsBoxContactConstraintDouble)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# We assume (from compilation) that IPOPT is available

if(NOT MogsBoxContactConstraint_FOUND)
	find_package(MogsBoxContactConstraint REQUIRED)
endif(NOT MogsBoxContactConstraint_FOUND)

if(NOT MogsBoxCollisionConstraintDouble_FOUND)
	find_package(MogsBoxCollisionConstraintDouble QUIET)
endif(NOT MogsBoxCollisionConstraintDouble_FOUND)

if(NOT MogsBoxCollisionConstraintDouble_FOUND)
    message("We do not compile MogsBoxContactConstraintDouble since MogsBoxCollisionConstraintDouble_FOUND was not found")
else(NOT MogsBoxCollisionConstraintDouble_FOUND)

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
            ${MogsBoxContact_INCLUDE_DIRS}
            ${MogsBoxCollisionConstraintDouble_INCLUDE_DIRS}
    )

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/config_MogsBoxContactConstraintDouble.h.in"
                                    "${CMAKE_CURRENT_SOURCE_DIR}/include/config_MogsBoxContactConstraintDouble.h" )


    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    FILE ( GLOB_RECURSE
                    header_files_MogsBoxContactConstraintDouble
                    include/*.h)

    FILE ( GLOB_RECURSE
                    source_files_MogsBoxContactConstraintDouble
                    src/*.cpp)

    ADD_LIBRARY(	MogsBoxContactConstraintDouble SHARED
                    ${source_files_MogsBoxContactConstraintDouble}
    )

    target_link_libraries(  MogsBoxContactConstraintDouble
                                                    ${MogsBoxCollisionConstraintDouble_LIBRARIES}
                                                    ${MogsBoxContactConstraint_LIBRARIES})

    link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
                                            ${MogsBoxCollisionConstraintDouble_LIBRARY_DIRS}
                                            ${MogsBoxContactConstraint_LIBRARY_DIRS})


    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH} ${MogsDouble_LIBRARY_DIRS}")


    ### Installation procedure

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    # install the include files
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsBoxContactConstraintDouble)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsBoxContactConstraintDouble} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsBoxContactConstraintDouble LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    SET(MogsBoxContactConstraintDouble_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
    SET(MogsBoxContactConstraintDouble_LIBRARIES  MogsBoxContactConstraintDouble CACHE INTERNAL "")
    SET(MogsBoxContactConstraintDouble_FLAGS "-DMogsBoxContactConstraintDouble_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
    SET(MogsBoxContactConstraintDouble_FOUND "TRUE" CACHE INTERNAL "")
    add_definitions(${MogsBoxContactConstraintDouble_FLAGS} ${MogsVisu_FLAGS})
    #  add flags (done before)
    configure_file(MogsBoxContactConstraintDoubleConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsBoxContactConstraintDoubleConfig.cmake" @ONLY)


    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsBoxContactConstraintDoubleConfig.cmake"
                    DESTINATION "${INSTALL_LIB_PATH}/MogsBoxContactConstraintDouble" COMPONENT dev )

    install(CODE "execute_process(COMMAND mogs2 plugins add MogsConstraintDouble BoxContact ${INSTALL_LIB_DIRS}/libMogsBoxContactConstraintDouble.so)")
endif()